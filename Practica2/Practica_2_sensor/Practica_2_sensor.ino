/* ------------------------------------------------------------------
 * Practica_2_sensor.ino  -  Dispositivo SENSOR (Práctica 2 IoT, GII-IC)
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

constexpr uint32_t SRF02_RANGING_TIME_MS = 70;   // datasheet: >= 65 ms
constexpr uint32_t SRF02_I2C_INIT_MS     = 100;
constexpr uint8_t  SRF02_MIN_DELAY_MS    = 65;   // retardo mínimo permitido
constexpr uint8_t  SRF02_MAX_DELAY_MS    = 255;  // el campo de estado es 8 bits
constexpr uint32_t SRF02_MIN_PERIOD_MS   = 100;  // < disparo (70) + margen
constexpr uint32_t SRF02_MAX_PERIOD_MS   = 3600000; // 1 h: TIME_MS2I usa
                                                    // aritmética de 32 bits
constexpr uint32_t SRF02_DEFAULT_PERIOD_MS = 500;

// ─── Estado de cada sensor ─────────────────────────────────────────────────
struct SensorCfg {
  uint8_t  unit;        // unidad activa (UNIT_CM / UNIT_INCH / UNIT_MS)
  bool     periodic;    // disparo periódico activo o no
  uint32_t period_ms;   // periodo del disparo periódico
  uint8_t  min_delay_ms;// retardo mínimo entre dos disparos consecutivos
  bool     present;     // detectado en el bus I2C (cache; se re-sondea)
};

static SensorCfg sensors[NUM_SENSORS];

// ─── Cerrojos (mutex) ──────────────────────────────────────────────────────
MUTEX_DECL(i2c_mtx);  // bus I2C compartido entre los hilos de rango y rx
MUTEX_DECL(cfg_mtx);  // configuración/estado compartido
MUTEX_DECL(tx_mtx);   // transmisor Serial1 compartido
MUTEX_DECL(dbg_mtx);  // monitor de depuración USB

// ─── Mailboxes: orden de trabajo de cada hilo de rango ─────────────────────
// Cada msg_t lleva: bits 1..0 = orden, bits 31..2 = periodo en ms.
constexpr uint8_t REQ_SHOOT = 0; // disparo único
constexpr uint8_t REQ_START = 1; // arrancar disparo periódico
constexpr uint8_t REQ_STOP  = 2; // detener disparo periódico

constexpr uint8_t REQ_MB_DEPTH = 4;

inline msg_t pack_request(uint8_t cmd, uint32_t period_ms) {
  return (msg_t)(((period_ms & 0x3FFFFFFFUL) << 2) | (cmd & 0x03));
}

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

// ─── LED de actividad ──────────────────────────────────────────────────────
static bool led_state = false;
static inline void toggle_led() {
  led_state = !led_state;
  digitalWrite(LED_BUILTIN, led_state ? HIGH : LOW);
}

// ══════════════════════════════════════════════════════════════════════════
//  ACCESO I2C AL SRF02
//  Cada transacción va precedida de chMtxLock(&i2c_mtx): aunque haya dos
//  hilos de rango disparando a la vez, solo uno toca el bus cada vez.
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
  chMtxUnlock(&i2c_mtx);
  if (!ok) return -1;

  // El sensor mide solo; dormimos sin retener el bus I2C para que el
  // otro sensor pueda usarlo durante estos 70 ms.
  chThdSleepMilliseconds(SRF02_RANGING_TIME_MS);

  chMtxLock(&i2c_mtx);
  int hi = srf02_read(addr, SRF02_REG_RANGE_HI);
  int lo = srf02_read(addr, SRF02_REG_RANGE_LO);
  chMtxUnlock(&i2c_mtx);
  if (hi < 0 || lo < 0) return -1;

  *out = (uint16_t)((hi << 8) | lo);
  return 0;
}

// Sondeo de presencia: ¿responde el sensor en su dirección?
static bool probe_sensor(uint8_t idx) {
  chMtxLock(&i2c_mtx);
  int rev = srf02_read(SRF02_ADDR7[idx], SRF02_REG_REVISION);
  chMtxUnlock(&i2c_mtx);
  chMtxLock(&cfg_mtx);
  sensors[idx].present = (rev >= 0);
  bool present = sensors[idx].present;
  chMtxUnlock(&cfg_mtx);
  return present;
}

/* ------------------------------------------------------------------
 * Para reprogramar la dirección I2C de un SRF02 (de fábrica todos
 * están en 0xF2 y en la práctica hay que mover uno a 0xE0), basta con
 * enviar esta secuencia al sensor CON SU DIRECCIÓN ACTUAL y a
 * continuación usar la nueva (ojo: la nueva dirección se escribe ya
 * desplazada, es la dirección de escritura de 8 bits):
 *
 *   write_command(0xF2>>1, ADDR_CHG_1ST); // 0xA0
 *   write_command(0xF2>>1, ADDR_CHG_2ND); // 0xAA
 *   write_command(0xF2>>1, ADDR_CHG_3RD); // 0xA5
 *   write_command(0xF2>>1, 0xE0);         // nueva dirección (8 bits)
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

static void reply_ok(uint8_t idx) {
  SensorCfg c = get_cfg(idx);
  send_frame(make_command(OP_RESP_OK, idx, c.unit, c.periodic, 0));
}

static void reply_error(uint8_t idx, uint8_t code) {
  send_frame(make_command(OP_RESP_ERROR, idx, UNIT_CM, false, code));
  chMtxLock(&dbg_mtx);
  Serial.print(F("[DBG] error "));
  Serial.print(code);
  Serial.print(F(" en sensor "));
  Serial.println(idx);
  chMtxUnlock(&dbg_mtx);
}

static bool post_request(uint8_t idx, msg_t req) {
  bool ok = (chMBPostTimeout(req_mb[idx], req, TIME_MS2I(100)) == MSG_OK);
  if (!ok) reply_error(idx, ERR_QUEUE_FULL);
  return ok;
}

// ══════════════════════════════════════════════════════════════════════════
//  DISPARO Y ENVÍO DE LA MEDIDA
// ══════════════════════════════════════════════════════════════════════════
static void shoot_and_report(uint8_t idx) {
  SensorCfg c = get_cfg(idx);

  if (!c.present && !probe_sensor(idx)) {
    reply_error(idx, ERR_UNKNOWN_SENSOR);
    return;
  }

  uint16_t value = 0;
  if (srf02_measure(idx, c.unit, &value) != 0) {
    reply_error(idx, ERR_I2C_FAIL);
    return;
  }

  send_frame(make_command(OP_RESP_MEASURE, idx, c.unit, c.periodic, value));

  // Retardo mínimo entre dos disparos consecutivos: además, al ser un
  // chThdSleep, el CPU se cede al resto de hilos en lugar de bloquearse.
  chThdSleepMilliseconds(c.min_delay_ms);
}

// Bucle del disparo periódico. La mailbox hace a la vez de "alarma" de
// periodo y de canal de órdenes, de modo que un 'off' del supervisor se
// atiende aunque estemos en medio de un periodo largo.
static void run_periodic(uint8_t idx, uint32_t period_ms) {
  mailbox_t* mb = req_mb[idx];

  while (get_cfg(idx).periodic) {
    msg_t req = 0;
    msg_t r = chMBFetchTimeout(mb, &req, TIME_MS2I(period_ms));

    if (r == MSG_OK) {
      uint8_t cmd = (uint8_t)(req & 0x03);
      if (cmd == REQ_STOP)  return;                 // 'off' del supervisor
      if (cmd == REQ_START) {                       // periodo nuevo
        period_ms = (uint32_t)(req >> 2);
        continue;
      }
      continue; // REQ_SHOOT dentro del modo periódico: ya dispara solo
    }

    // Timeout -> toca disparar
    if (!get_cfg(idx).periodic) return;             // seguridad
    shoot_and_report(idx);
  }
}

// ══════════════════════════════════════════════════════════════════════════
//  HILO DE RANGO: UN POR SENSOR
//  Recibe órdenes del hilo rx a través de su mailbox y ejecuta los
//  disparos sin bloquear la recepción de comandos.
// ══════════════════════════════════════════════════════════════════════════
static THD_FUNCTION(ranging_thd, arg) {
  uint8_t idx = (uint8_t)(uintptr_t)arg;

  while (true) {
    msg_t req = 0;
    if (chMBFetchTimeout(req_mb[idx], &req, TIME_INFINITE) != MSG_OK) continue;

    uint8_t cmd     = (uint8_t)(req & 0x03);
    uint32_t period = (uint32_t)(req >> 2);

    switch (cmd) {
      case REQ_SHOOT:
        shoot_and_report(idx);
        break;
      case REQ_START:
        run_periodic(idx, period);
        break;
      case REQ_STOP:
        // Si el hilo está en run_periodic() la orden la consume ahí
        // (mailbox compartido); si no, no hay nada que detener.
        break;
      default:
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
    reply_error(m.dir_sensor, ERR_UNKNOWN_SENSOR);
    return;
  }

  switch (m.opcode) {
    case OP_US_ONE_SHOT:
      if (post_request(idx, pack_request(REQ_SHOOT, 0))) reply_ok(idx);
      break;

    case OP_US_PERIODIC_ON: {
      uint32_t period = m.data;
      if (period < SRF02_MIN_PERIOD_MS) period = SRF02_MIN_PERIOD_MS;
      if (period > SRF02_MAX_PERIOD_MS) period = SRF02_MAX_PERIOD_MS;

      chMtxLock(&cfg_mtx);
      if (sensors[idx].periodic) {
        chMtxUnlock(&cfg_mtx);
        reply_error(idx, ERR_ALREADY_ACTIVE);
        return;
      }
      sensors[idx].periodic  = true;
      sensors[idx].period_ms = period;
      chMtxUnlock(&cfg_mtx);

      if (post_request(idx, pack_request(REQ_START, period))) reply_ok(idx);
      break;
    }

    case OP_US_PERIODIC_OFF:
      // Idempotente: el guión pide detener "si lo estuviera".
      chMtxLock(&cfg_mtx);
      sensors[idx].periodic = false;
      chMtxUnlock(&cfg_mtx);

      if (post_request(idx, pack_request(REQ_STOP, 0))) reply_ok(idx);
      break;

    case OP_US_SET_UNIT:
      if (m.unit == UNIT_RSVD) {
        reply_error(idx, ERR_INVALID_UNIT);
        return;
      }
      chMtxLock(&cfg_mtx);
      sensors[idx].unit = m.unit;
      chMtxUnlock(&cfg_mtx);
      reply_ok(idx);
      break;

    case OP_US_SET_DELAY: {
      uint32_t d = m.data;
      if (d < SRF02_MIN_DELAY_MS) d = SRF02_MIN_DELAY_MS;
      if (d > SRF02_MAX_DELAY_MS) d = SRF02_MAX_DELAY_MS;
      chMtxLock(&cfg_mtx);
      sensors[idx].min_delay_ms = (uint8_t)d;
      chMtxUnlock(&cfg_mtx);
      reply_ok(idx);
      break;
    }

    case OP_US_STATUS: {
      probe_sensor(idx);                    // sondeo I2C en vivo
      SensorCfg c = get_cfg(idx);
      send_frame(make_command(
        OP_RESP_STATUS, idx, c.unit, c.periodic,
        pack_status_data(SRF02_ADDR7[idx], c.unit, c.periodic,
                         c.min_delay_ms)));
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
      reply_error(idx, ERR_UNKNOWN_CMD);
      break;
  }
}

// ══════════════════════════════════════════════════════════════════════════
//  HILO RECEPTOR: monta frames de 6 bytes y despacha los comandos
// ══════════════════════════════════════════════════════════════════════════
static THD_FUNCTION(rx_thd, arg) {
  (void)arg;
  uint8_t buf[PROTO_MSG_SIZE];
  uint8_t n = 0;

  while (true) {
    if (Serial1.available() > 0) {
      buf[n++] = (uint8_t)Serial1.read();
      if (n == PROTO_MSG_SIZE) {
        n = 0;
        handle_frame(ProtoMsg::deserialize(buf));
      }
    } else {
      chThdSleepMilliseconds(1); // sin datos: ceder la CPU
    }
  }
}

// ══════════════════════════════════════════════════════════════════════════
//  ARRANQUE (patrón ChRt): setup() -> chBegin(chSetup) -> hilos -> loop()
// ══════════════════════════════════════════════════════════════════════════
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
  while (!Serial) { /* esperar USB */ }
  Serial1.begin(SERIAL1_BAUDS);

  Wire.begin();
  delay(SRF02_I2C_INIT_MS); // arranque del bus I2C (antes del kernel OK)

  // Configuración por defecto de los dos sensores
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    sensors[i].unit         = UNIT_CM;
    sensors[i].periodic     = false;
    sensors[i].period_ms    = SRF02_DEFAULT_PERIOD_MS;
    sensors[i].min_delay_ms = SRF02_MIN_DELAY_MS;
    sensors[i].present      = false;
  }

  // OJO: el kernel de ChibiOS todavía NO está arrancado (chBegin está
  // más abajo), así que este sondeo se hace SIN cerrojos: aún no existe
  // ningún otro hilo que pueda entrar en el bus I2C.
  Serial.println(F("=== SENSOR - Practica 2 IoT (ChibiOS/ChRt) ==="));
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    int rev = srf02_read(SRF02_ADDR7[i], SRF02_REG_REVISION);
    sensors[i].present = (rev >= 0);
    Serial.print(F("SRF02 idx "));
    Serial.print(i);
    Serial.print(F(" -> dir I2C 0x"));
    Serial.print(SRF02_ADDR7[i], HEX);
    Serial.print(F(" (cruda 0x"));
    Serial.print(SRF02_ADDR8[i], HEX);
    Serial.println(sensors[i].present ? F(") : detectado")
                                      : F(") : NO detectado"));
  }

  // Arranca el kernel de ChibiOS y crea los hilos; no vuelve nunca.
  chBegin(chSetup);
  while (true) { }
}

void loop() {
  // En ChRt, loop() es el HILO PRINCIPAL del sistema. Aquí no hace
  // trabajo: la recepción y los disparos los llevan sus propios hilos,
  // así que simplemente dormimos para no consumir CPU.
  chThdSleepMilliseconds(1000);
}

