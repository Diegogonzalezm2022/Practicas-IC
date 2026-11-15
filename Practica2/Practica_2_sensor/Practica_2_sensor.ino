/* ------------------------------------------------------------------
 * practica2_sensor.ino  -  Dispositivo SENSOR (Práctica 2 IoT, GII-IC)
 *
 * Dispositivo terminal sensor con ChibiOS/RT (biblioteca ChRt 1.3):
 *   - Recibe comandos binarios del supervisor por Serial1 (UART 9600)
 *   - Accede a DOS sensores de ultrasonidos SRF02 por I2C:
 *       sensor 0 -> 0xF2 (7-bit 0x79)
 *       sensor 1 -> 0xE0 (7-bit 0x70)
 *   - Confirma la ejecución de cada comando o notifica el error
 *   - Envía las medidas de forma asíncrona cuando hay disparo
 *     periódico activo
 *
 * RECURSOS DE ChibiOS UTILIZADOS (apartado opcional ChibiOS):
 *   hilos      : rx_thd       -> recepción/decodificación de comandos
 *                ranging_thd  -> UN HILO POR SENSOR que ejecuta los
 *                                disparos (one-shot o periódicos) sin
 *                                bloquear la recepción
 *   cerrojos   : i2c_mtx  -> serializa el acceso al bus I2C
 *                cfg_mtx  -> protege la configuración compartida
 *                tx_mtx   -> serializa el uso de Serial1 (rx + ranging)
 *                dbg_mtx  -> serializa el monitor de depuración
 *   mailboxes  : una por sensor; el hilo rx le entrega órdenes
 *                (disparar / arrancar / parar) a su hilo de rango.
 *                En el modo periódico la propia mailbox hace de
 *                "alarma": si expira el periodo se dispara, y si llega
 *                una orden se atiende al instante.
 *
 * NOTA sobre el tiempo: una vez ejecutado chBegin() el SysTick lo
 * gestiona el kernel, por lo que NO se usan delay()/millis(); se usa
 * chThdSleepMilliseconds() (dormir = ceder el CPU a otros hilos).
 * ------------------------------------------------------------------ */

#include <ChRt.h>
#include <Wire.h>
#include "protocol.h"

// ─── Puertos serie ─────────────────────────────────────────────────────────
constexpr uint32_t SERIAL_MONITOR_BAUDS = 115200; // depuración por USB
constexpr uint32_t SERIAL1_BAUDS        = 9600;   // enlace con supervisor

// ─── Registros y comandos del SRF02 (datasheet Devantech) ──────────────────
constexpr uint8_t  SRF02_REG_COMMAND  = 0x00;
constexpr uint8_t  SRF02_REG_REVISION = 0x00;
constexpr uint8_t  SRF02_REG_RANGE_HI = 0x02;
constexpr uint8_t  SRF02_REG_RANGE_LO = 0x03;
constexpr uint8_t  CMD_RANGE_INCHES   = 80;
constexpr uint8_t  CMD_RANGE_CMS      = 81;
constexpr uint8_t  CMD_RANGE_USECS    = 82;  // datasheet: µs; guion: "ms"
constexpr uint8_t  ADDR_CHG_1ST       = 160; // secuencia de cambio de dir. I2C
constexpr uint8_t  ADDR_CHG_2ND       = 170;
constexpr uint8_t  ADDR_CHG_3RD       = 165;

constexpr uint32_t SRF02_RANGING_TIME_MS = SRF02_MEASUREMENT_MS;
constexpr uint32_t SRF02_I2C_INIT_MS     = 100;
constexpr uint32_t SRF02_DEFAULT_PERIOD_MS = 500;

// ─── Estado de cada sensor ─────────────────────────────────────────────────
struct SensorCfg {
  uint8_t  unit;        // unidad activa (UNIT_CM / UNIT_INCH / UNIT_MS)
  bool     periodic;    // disparo periódico activo o no
  uint32_t period_ms;   // periodo del disparo periódico
  uint32_t min_delay_ms;// retardo minimo completo, sin truncar a un byte
  bool     present;     // detectado en el bus I2C (cache; se re-sondea)
};

static SensorCfg sensors[NUM_SENSORS];
static uint64_t last_shot[NUM_SENSORS];
static bool has_shot[NUM_SENSORS] = {false, false};
static bool measuring[NUM_SENSORS] = {false, false}; // bajo i2c_mtx

// ─── Cerrojos (mutex) ──────────────────────────────────────────────────────
MUTEX_DECL(i2c_mtx);  // bus I2C compartido entre los hilos de rango y rx
MUTEX_DECL(cfg_mtx);  // configuración/estado compartido
MUTEX_DECL(tx_mtx);   // transmisor Serial1 compartido
MUTEX_DECL(dbg_mtx);  // monitor de depuración USB

// ─── Mailboxes: orden de trabajo de cada hilo de rango ─────────────────────
// La mailbox lleva solo la orden. El periodo uint32_t vive en SensorCfg:
// empaquetarlo con desplazamientos perderia los bits altos.
constexpr uint8_t REQ_SHOOT = 0; // disparo único
constexpr uint8_t REQ_START = 1; // arrancar disparo periódico
constexpr uint8_t REQ_STOP  = 2; // detener disparo periódico
constexpr uint8_t REQ_REFRESH = 3; // despertar al cambiar el retardo

constexpr uint8_t REQ_MB_DEPTH = 4;

static msg_t req_buf0[REQ_MB_DEPTH], req_buf1[REQ_MB_DEPTH];
MAILBOX_DECL(req_mb0, req_buf0, REQ_MB_DEPTH);
MAILBOX_DECL(req_mb1, req_buf1, REQ_MB_DEPTH);

static mailbox_t* const req_mb[NUM_SENSORS] = { &req_mb0, &req_mb1 };

// ─── Pilas de los hilos ────────────────────────────────────────────────────
// CADA hilo necesita SU PROPIA área de trabajo (stack): en ChRt un hilo
// estático no puede compartir working area con otro.
static THD_WORKING_AREA(wa_rx_thd,       512);
static THD_WORKING_AREA(wa_ranging_thd0, 384); // rango del sensor 0 (0xF2)
static THD_WORKING_AREA(wa_ranging_thd1, 384); // rango del sensor 1 (0xE0)

// Extiende el contador del kernel a 64 bits. loop() lo actualiza cada
// segundo incluso si todos los periodos duran dias, para registrar vueltas.
static uint64_t clock_ms() {
  static systime_t previous = 0;
  static uint64_t ticks = 0;
  chSysLock();
  const systime_t now = chVTGetSystemTimeX();
  ticks += (systime_t)(now - previous);
  previous = now;
  const uint64_t snapshot = ticks;
  chSysUnlock();
  return snapshot * 1000ULL / CH_CFG_ST_FREQUENCY;
}

// ─── LED de actividad ──────────────────────────────────────────────────────
static bool led_state = false;
static inline void toggle_led() {
  led_state = !led_state;
  digitalWrite(LED_BUILTIN, led_state ? HIGH : LOW);
}

// ══════════════════════════════════════════════════════════════════════════
//  ACCESO I2C AL SRF02
//  i2c_mtx protege las transacciones. Durante la conversion del sensor
//  se libera el bus para poder atender al otro dispositivo.
// ══════════════════════════════════════════════════════════════════════════
static bool srf02_write(uint8_t addr7, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(addr7);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

// Devuelve el byte leído o -1 si falla
static int srf02_read(uint8_t addr7, uint8_t reg) {
  Wire.beginTransmission(addr7);
  Wire.write(reg);
  if (Wire.endTransmission() != 0) return -1;
  if (Wire.requestFrom(addr7, (uint8_t)1) == 0) return -1;
  if (!Wire.available()) return -1;
  return Wire.read();
}

// Devuelve 0 si la medida se obtuvo correctamente, -1 en caso contrario
static int srf02_measure(uint8_t idx, uint8_t unit, uint16_t* out) {
  const uint8_t addr = SRF02_ADDR7[idx];
  uint8_t cmd = CMD_RANGE_CMS;
  if      (unit == UNIT_INCH) cmd = CMD_RANGE_INCHES;
  else if (unit == UNIT_MS)   cmd = CMD_RANGE_USECS;

  chMtxLock(&i2c_mtx);
  bool ok = srf02_write(addr, SRF02_REG_COMMAND, cmd);
  if (!ok) {
    chMtxUnlock(&i2c_mtx);
    return -1;
  }
  last_shot[idx] = clock_ms();
  has_shot[idx] = true;
  measuring[idx] = true;
  chMtxUnlock(&i2c_mtx);

  chThdSleepMilliseconds(SRF02_RANGING_TIME_MS);

  chMtxLock(&i2c_mtx);
  int hi = srf02_read(addr, SRF02_REG_RANGE_HI);
  int lo = srf02_read(addr, SRF02_REG_RANGE_LO);
  measuring[idx] = false;
  chMtxUnlock(&i2c_mtx);
  if (hi < 0 || lo < 0) return -1;

  *out = (uint16_t)((hi << 8) | lo);
  return 0;
}

// Sondeo de presencia: ¿responde el sensor en su dirección?
static bool probe_sensor(uint8_t idx) {
  chMtxLock(&i2c_mtx);
  // Durante la medicion el SRF02 no responde a I2C: no es una desconexion.
  int rev = measuring[idx] ? 0 : srf02_read(SRF02_ADDR7[idx], SRF02_REG_REVISION);
  chMtxUnlock(&i2c_mtx);
  chMtxLock(&cfg_mtx);
  sensors[idx].present = (rev >= 0);
  bool present = sensors[idx].present;
  chMtxUnlock(&cfg_mtx);
  return present;
}

/* ------------------------------------------------------------------
 * Para reprogramar la dirección I2C de un SRF02 (de fábrica todos
 * están en 0xE0 y para esta configuracion hay que mover uno a 0xF2), basta con
 * enviar esta secuencia al sensor CON SU DIRECCIÓN ACTUAL y a
 * continuación usar la nueva (ojo: la nueva dirección se escribe ya
 * desplazada, es la dirección de escritura de 8 bits):
 *
 *   write_command(0xE0>>1, ADDR_CHG_1ST); // 0xA0
 *   write_command(0xE0>>1, ADDR_CHG_2ND); // 0xAA
 *   write_command(0xE0>>1, ADDR_CHG_3RD); // 0xA5
 *   write_command(0xE0>>1, 0xF2);         // nueva dirección (8 bits)
 *   delay(12);                            // > 12 ms antes de usarla
 * ------------------------------------------------------------------ */

// ══════════════════════════════════════════════════════════════════════════
//  ENVÍO DE FRAMES AL SUPERVISOR
//  Compartido por el hilo rx (confirmaciones) y los hilos de rango
//  (medidas), por eso va protegido con tx_mtx.
// ══════════════════════════════════════════════════════════════════════════
static void send_frame(const ProtoMsg& m) {
  uint8_t buf[PROTO_MSG_SIZE];
  m.serialize(buf);
  chMtxLock(&tx_mtx);
  Serial1.write(buf, PROTO_MSG_SIZE);
  toggle_led();
  chMtxUnlock(&tx_mtx);
}

// Lee la configuración vigente de un sensor (bajo cfg_mtx)
static SensorCfg get_cfg(uint8_t idx) {
  chMtxLock(&cfg_mtx);
  SensorCfg c = sensors[idx];
  chMtxUnlock(&cfg_mtx);
  return c;
}

static void reply_ok(uint8_t idx, uint8_t opcode) {
  SensorCfg c = get_cfg(idx);
  send_frame(make_command(OP_RESP_OK, idx, c.unit, c.periodic, opcode));
}

static void reply_error(uint8_t idx, uint8_t code, uint8_t opcode = 0) {
  send_frame(make_command(OP_RESP_ERROR, idx, UNIT_CM, false,
                          ((uint32_t)opcode << 8) | code));
  if (!Serial) return;
  chMtxLock(&dbg_mtx);
  Serial.print(F("[DBG] error "));
  Serial.print(code);
  Serial.print(F(" en sensor "));
  Serial.println(idx);
  chMtxUnlock(&dbg_mtx);
}

static bool post_request(uint8_t idx, msg_t req, uint8_t opcode) {
  bool ok = (chMBPostTimeout(req_mb[idx], req, TIME_MS2I(100)) == MSG_OK);
  if (!ok) reply_error(idx, ERR_QUEUE_FULL, opcode);
  return ok;
}

// ══════════════════════════════════════════════════════════════════════════
//  DISPARO Y ENVÍO DE LA MEDIDA
// ══════════════════════════════════════════════════════════════════════════
static void shoot_and_report(uint8_t idx, bool one_shot = false) {
  SensorCfg c = get_cfg(idx);
  const uint8_t opcode = one_shot ? OP_US_ONE_SHOT : 0;

  if (!c.present && !probe_sensor(idx)) {
    reply_error(idx, ERR_UNKNOWN_SENSOR, opcode);
    return;
  }

  uint16_t value = 0;
  if (srf02_measure(idx, c.unit, &value) != 0) {
    reply_error(idx, ERR_I2C_FAIL, opcode);
    return;
  }

  send_frame(make_command(OP_RESP_MEASURE, idx, c.unit,
                          !one_shot && c.periodic, value));
}

// Un hilo por sensor programa tanto los disparos unicos como los periodicos.
// El retardo se ESPERA, en vez de rechazar un one-shot por llegar pronto.
// La mailbox sigue atendiendo on/off mientras hay una espera pendiente.
static THD_FUNCTION(ranging_thd, arg) {
  const uint8_t idx = (uint8_t)(uintptr_t)arg;
  uint32_t pending_shots = 0;
  bool running = false;
  uint64_t next_periodic = 0;

  while (true) {
    SensorCfg c = get_cfg(idx);
    uint64_t now = clock_ms();
    uint32_t gap = c.min_delay_ms;
    if (gap < SRF02_MIN_TRIGGER_MS) gap = SRF02_MIN_TRIGGER_MS;
    uint64_t ready_at = has_shot[idx] ? last_shot[idx] + gap : now;
    bool periodic_due = running && c.periodic && now >= next_periodic;
    bool measured = false;

    if ((pending_shots > 0 || periodic_due) && now >= ready_at) {
      bool one_shot = pending_shots > 0;
      if (one_shot) --pending_shots;
      else next_periodic = now + c.period_ms;
      shoot_and_report(idx, one_shot);
      measured = true;
    }

    uint64_t wait_ms = measured ? 0 : 1000; // atiende la mailbox entre disparos
    if (pending_shots > 0 || periodic_due) {
      if (ready_at > now && ready_at - now < wait_ms) wait_ms = ready_at - now;
    } else if (running && c.periodic && next_periodic > now && next_periodic - now < wait_ms) {
      wait_ms = next_periodic - now;
    }

    msg_t req = 0;
    if (chMBFetchTimeout(req_mb[idx], &req, TIME_MS2I((uint32_t)wait_ms)) != MSG_OK) continue;
    switch ((uint8_t)req) {
      case REQ_SHOOT:
        if (pending_shots == UINT32_MAX) reply_error(idx, ERR_QUEUE_FULL, OP_US_ONE_SHOT);
        else {
          ++pending_shots;
          // Respuesta inmediata de recepcion; la medida confirma despues
          // la realizacion del disparo, aunque delay dure mas de 2 segundos.
          send_frame(make_command(OP_RESP_PENDING, idx, get_cfg(idx).unit));
        }
        break;
      case REQ_START:
        running = true;
        next_periodic = clock_ms() + get_cfg(idx).period_ms;
        reply_ok(idx, OP_US_PERIODIC_ON);
        break;
      case REQ_STOP:
        running = false;
        reply_ok(idx, OP_US_PERIODIC_OFF);
        break;
      case REQ_REFRESH:
        break;
    }
  }
}

// ══════════════════════════════════════════════════════════════════════════
//  ATENCIÓN AL COMANDO RECIBIDO (hilo rx)
// ══════════════════════════════════════════════════════════════════════════
static void handle_frame(const ProtoMsg& m) {
  // Validación del campo "I2C dir" (bits 38..35): el mensaje tiene que
  // ir dirigido a un sensor realmente presente en nuestro bus.
  uint8_t idx = sensor_idx_from_i2c_dir(m.i2c_dir);
  if (idx == 0xFF || idx != m.dir_sensor) {
    // Respondemos igualmente con el índice del bit 39 para que el
    // supervisor pueda identificar de qué sensor hablamos.
    reply_error(m.dir_sensor, ERR_UNKNOWN_SENSOR, m.opcode);
    return;
  }

  switch (m.opcode) {
    case OP_US_ONE_SHOT:
      post_request(idx, REQ_SHOOT, m.opcode);
      break;

    case OP_US_PERIODIC_ON: {
      if (!probe_sensor(idx)) {
        reply_error(idx, ERR_UNKNOWN_SENSOR, m.opcode);
        return;
      }
      uint32_t period = m.data;
      if (period < SRF02_MEASUREMENT_MS) {
        reply_error(idx, ERR_PERIOD_TOO_SHORT, m.opcode);
        return;
      }

      chMtxLock(&cfg_mtx);
      if (period < sensors[idx].min_delay_ms) {
        chMtxUnlock(&cfg_mtx);
        reply_error(idx, ERR_INVALID_DELAY, m.opcode);
        return;
      }
      const bool was_periodic = sensors[idx].periodic;
      const uint32_t old_period = sensors[idx].period_ms;
      sensors[idx].periodic  = true;
      sensors[idx].period_ms = period;
      chMtxUnlock(&cfg_mtx);

      if (!post_request(idx, REQ_START, m.opcode)) {
        chMtxLock(&cfg_mtx);
        sensors[idx].periodic = was_periodic;
        sensors[idx].period_ms = old_period;
        chMtxUnlock(&cfg_mtx);
      }
      break;
    }

    case OP_US_PERIODIC_OFF:
      // Idempotente: el guión pide detener "si lo estuviera".
      chMtxLock(&cfg_mtx);
      sensors[idx].periodic = false;
      chMtxUnlock(&cfg_mtx);

      post_request(idx, REQ_STOP, m.opcode);
      break;

    case OP_US_SET_UNIT:
      if (m.unit == UNIT_RSVD) {
        reply_error(idx, ERR_INVALID_UNIT, m.opcode);
        return;
      }
      chMtxLock(&cfg_mtx);
      sensors[idx].unit = m.unit;
      chMtxUnlock(&cfg_mtx);
      reply_ok(idx, m.opcode);
      break;

    case OP_US_SET_DELAY: {
      uint32_t d = m.data;
      chMtxLock(&cfg_mtx);
      if (sensors[idx].periodic && d > sensors[idx].period_ms) {
        chMtxUnlock(&cfg_mtx);
        reply_error(idx, ERR_INVALID_DELAY, m.opcode);
        return;
      }
      sensors[idx].min_delay_ms = d;
      chMtxUnlock(&cfg_mtx);
      chMBPostTimeout(req_mb[idx], REQ_REFRESH, TIME_IMMEDIATE);
      reply_ok(idx, m.opcode);
      break;
    }

    case OP_US_STATUS: {
      if (!probe_sensor(idx)) {
        reply_error(idx, ERR_UNKNOWN_SENSOR, m.opcode);
        return;
      }
      SensorCfg c = get_cfg(idx);
      send_frame(make_command(
        OP_RESP_STATUS, idx, c.unit, c.periodic,
        pack_status_data(c.min_delay_ms)));
      break;
    }

    case OP_US_LIST: {
      // Máscara de presencia: bit i = 1 si el sensor i responde
      uint32_t mask = 0;
      for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        if (probe_sensor(i)) mask |= (1UL << i);
      }
      send_frame(make_command(OP_RESP_LIST, 0, UNIT_CM, false, mask));
      break;
    }

    default:
      reply_error(idx, ERR_UNKNOWN_CMD, m.opcode);
      break;
  }
}

// ══════════════════════════════════════════════════════════════════════════
//  HILO RECEPTOR: monta frames de 6 bytes y despacha los comandos
// ══════════════════════════════════════════════════════════════════════════
static THD_FUNCTION(rx_thd, arg) {
  (void)arg;
  ProtoReceiver receiver;

  while (true) {
    if (Serial1.available() > 0) {
      ProtoMsg m;
      if (receiver.push((uint8_t)Serial1.read(),
                        (uint32_t)clock_ms(), false, &m)) handle_frame(m);
    } else {
      chThdSleepMilliseconds(1); // sin datos: ceder la CPU
    }
  }
}

// ══════════════════════════════════════════════════════════════════════════
//  ARRANQUE (patrón ChRt): setup() -> chBegin(chSetup) -> hilos -> loop()
// ══════════════════════════════════════════════════════════════════════════
static void print_detection_report() {
  bool detected[NUM_SENSORS];
  uint8_t count = 0;
  for (uint8_t i = 0; i < NUM_SENSORS; ++i) {
    detected[i] = probe_sensor(i);
    if (detected[i]) ++count;
  }

  chMtxLock(&dbg_mtx);
  Serial.println(F("=== SENSOR - Practica 2 IoT (ChibiOS/ChRt) ==="));
  Serial.println(F("Comprobacion de sensores SRF02 por I2C:"));
  for (uint8_t i = 0; i < NUM_SENSORS; ++i) {
    Serial.print(F("Sensor "));
    Serial.print(i);
    Serial.print(F(" - I2C 0x"));
    Serial.print(SRF02_ADDR7[i], HEX);
    Serial.print(F(" (direccion de 8 bits: 0x"));
    Serial.print(SRF02_ADDR8[i], HEX);
    Serial.println(detected[i] ? F(") -> DETECTADO") : F(") -> NO DETECTADO"));
  }
  Serial.print(F("Sensores detectados: "));
  Serial.print(count);
  Serial.print(F(" de "));
  Serial.println(NUM_SENSORS);
  Serial.println(count == NUM_SENSORS
    ? F("[OK] Ambos sensores responden. Terminal listo para recibir comandos.")
    : F("[AVISO] Faltan sensores: revisa alimentacion, SDA/SCL y direcciones I2C."));
  chMtxUnlock(&dbg_mtx);
}

void chSetup() {
  // Prioridad del hilo rx por encima de la de rango: un comando nunca
  // queda esperando a que termine un disparo.
  chThdCreateStatic(wa_rx_thd, sizeof(wa_rx_thd),
                    NORMALPRIO + 2, rx_thd, nullptr);

  chThdCreateStatic(wa_ranging_thd0, sizeof(wa_ranging_thd0),
                    NORMALPRIO + 1, ranging_thd, (void*)(uintptr_t)0);
  chThdCreateStatic(wa_ranging_thd1, sizeof(wa_ranging_thd1),
                    NORMALPRIO + 1, ranging_thd, (void*)(uintptr_t)1);
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Monitor de depuración (solo informativo; el camino crítico es Serial1)
  Serial.begin(SERIAL_MONITOR_BAUDS);
  // El terminal sensor funciona sin abrir su monitor USB.
  Serial1.begin(SERIAL1_BAUDS);

  Wire.begin();
  Wire.setClock(100000);
  delay(SRF02_I2C_INIT_MS); // arranque del bus I2C (antes del kernel OK)

  // Configuración por defecto de los dos sensores
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    sensors[i].unit         = UNIT_CM;
    sensors[i].periodic     = false;
    sensors[i].period_ms    = SRF02_DEFAULT_PERIOD_MS;
    sensors[i].min_delay_ms = SRF02_MIN_TRIGGER_MS;
    sensors[i].present      = false;
  }

  // OJO: el kernel de ChibiOS todavía NO está arrancado (chBegin está
  // más abajo), así que este sondeo se hace SIN cerrojos: aún no existe
  // ningún otro hilo que pueda entrar en el bus I2C.
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    int rev = srf02_read(SRF02_ADDR7[i], SRF02_REG_REVISION);
    sensors[i].present = (rev >= 0);
  }

  // Arranca el kernel de ChibiOS y crea los hilos; no vuelve nunca.
  chBegin(chSetup);
  while (true) { }
}

void loop() {
  clock_ms();
  // El monitor suele abrirse DESPUES de subir el programa. Se presenta
  // el diagnostico al conectar USB y cada vez que se vuelve a abrir,
  // sin esperar al monitor para arrancar Serial1 ni los hilos de medida.
  static bool monitor_was_open = false;
  // DTR indica el monitor abierto; operator bool tambien acepta RTS,
  // que algunos terminales dejan activo al cerrar el puerto.
  bool monitor_open = Serial.dtr();
  if (monitor_open && !monitor_was_open) {
    chThdSleepMilliseconds(100); // permite al monitor preparar la recepcion
    monitor_open = Serial.dtr();
    if (monitor_open) print_detection_report();
  }
  monitor_was_open = monitor_open;
  chThdSleepMilliseconds(50);
}

