/* ------------------------------------------------------------------
 * practica2_supervisor.ino  -  Dispositivo SUPERVISOR (Práctica 2 IoT)
 *
 * Asignatura: GII-IC (Internet de las Cosas), Universidad de Las Palmas
 *             de Gran Canaria.
 *
 * Dispositivo supervisor con ChibiOS/RT (biblioteca ChRt 1.3):
 *   - Lee e interpreta comandos que teclea el usuario en el monitor
 *     serie (USB) y los convierte en frames binarios de 6 bytes
 *     (ver protocol.h / codificacion.jpeg).
 *   - Los envía al dispositivo sensor por Serial1 (UART 9600, cables
 *     cruzados TX<->RX y GND en común) y espera su respuesta.
 *   - Muestra los resultados en el monitor serie, incluidas las MEDIDAS
 *     ASÍNCRONAS que llegan cuando el sensor tiene disparo periódico
 *     activo.
 *
 * RECURSOS DE ChibiOS UTILIZADOS (apartado opcional ChibiOS):
 *   hilos      : loop()      -> HILO PRINCIPAL: entrada de usuario,
 *                               parseo, envío y presentación de
 *                               resultados (en ChRt, loop() corre como
 *                               hilo principal tras chBegin()).
 *                rx_thd      -> recepción de frames por Serial1; solo
 *                               los encola y avisa con un semáforo.
 *   cerrojos   : tx_mtx   -> serializa Serial1
 *                queue_mtx  -> protege la cola de frames recibidos
 *   semáforos  : rsp_sem    -> señal de "llego un frame";
 *                              el hilo principal espera sobre él con
 *                              TIMEOUT (chSemWaitTimeout).
 *
 * DISEÑO ANTI-INTERFERENCIAS (sugerencia del guión): todas las
 * impresiones por el monitor las hace un ÚNICO hilo (el principal), y
 * las medidas asíncronas se encolan. Si el usuario está a mitad de
 * escribir un comando, la medida se muestra en su propia línea y se
 * repinta el prompt con lo tecleado, sin corromper su entrada.
 * ------------------------------------------------------------------ */

#include <ChRt.h>
#include "protocol.h"
#include <string.h>
#include <stdlib.h>
#include <errno.h>

// ─── Configuración ─────────────────────────────────────────────────────────
constexpr uint32_t SERIAL_MONITOR_BAUDS = 115200;
constexpr uint32_t SERIAL1_BAUDS        = 9600;
constexpr uint32_t RESPONSE_TIMEOUT_MS  = 2000; // espera de respuesta

constexpr uint8_t  CMD_LINE_MAX   = 64;  // ancho de línea del monitor
constexpr uint8_t  RSP_QUEUE_SIZE = 8;   // profundidad de la cola RX

// ─── Cola de frames recibidos ──────────────────────────────────────────────
static ProtoMsg rsp_queue[RSP_QUEUE_SIZE];
static uint8_t  rsp_head = 0, rsp_tail = 0, rsp_count = 0;

MUTEX_DECL(queue_mtx);        // cerrojo de la cola
SEMAPHORE_DECL(rsp_sem, 0);   // semaforo: "hay un frame nuevo"
MUTEX_DECL(tx_mtx);           // cerrojo de Serial1

// ─── Estado del hilo principal (entrada del usuario) ───────────────────────
static char    line_buf[CMD_LINE_MAX];
static uint8_t line_len = 0;
static bool    led_state = false;

// ─── Pilas de los hilos ────────────────────────────────────────────────────
static THD_WORKING_AREA(wa_rx_thd, 384);

// ─── Prototipos ────────────────────────────────────────────────────────────
static void print_help();
static void print_frame(const ProtoMsg& m);
static void drain_queue();
static bool parse_command(const char* line, ProtoMsg* out);
static void execute_command(const ProtoMsg& cmd);

// ══════════════════════════════════════════════════════════════════════════
//  COLA PROTEGIDA POR CERROJO + SEMÁFORO DE AVISO
// ══════════════════════════════════════════════════════════════════════════
static bool queue_push(const ProtoMsg& m, bool notify) {
  bool ok = false;
  chMtxLock(&queue_mtx);
  if (rsp_count < RSP_QUEUE_SIZE) {
    rsp_queue[rsp_head] = m;
    rsp_head = (uint8_t)((rsp_head + 1) % RSP_QUEUE_SIZE);
    rsp_count++;
    ok = true;
  }
  chMtxUnlock(&queue_mtx);

  if (ok && notify) chSemSignal(&rsp_sem); // despierta al hilo principal
  return ok;
}

static bool queue_pop(ProtoMsg* out) {
  bool has = false;
  chMtxLock(&queue_mtx);
  if (rsp_count > 0) {
    *out = rsp_queue[rsp_tail];
    rsp_tail = (uint8_t)((rsp_tail + 1) % RSP_QUEUE_SIZE);
    rsp_count--;
    has = true;
  }
  chMtxUnlock(&queue_mtx);
  return has;
}

static void drain_queue() {
  ProtoMsg m;
  while (queue_pop(&m)) print_frame(m);
}

// ══════════════════════════════════════════════════════════════════════════
//  HILO RECEPTOR: monta frames de 6 bytes y los encola
// ══════════════════════════════════════════════════════════════════════════
static THD_FUNCTION(rx_thd, arg) {
  (void)arg;
  ProtoReceiver receiver;

  while (true) {
    if (Serial1.available() > 0) {
      ProtoMsg m;
      if (receiver.push((uint8_t)Serial1.read(),
                        (uint32_t)((uint64_t)chVTGetSystemTimeX() * 1000ULL /
                                   CH_CFG_ST_FREQUENCY), true, &m)) {
        // Todas las tramas despiertan al hilo principal; este distingue
        // la respuesta esperada de las medidas periodicas al vaciar la cola.
        queue_push(m, true);
      }
    } else {
      chThdSleepMilliseconds(1); // sin datos: ceder la CPU
    }
  }
}

// ══════════════════════════════════════════════════════════════════════════
//  ENVÍO DE FRAMES AL SENSOR
// ══════════════════════════════════════════════════════════════════════════
static void send_frame(const ProtoMsg& m) {
  byte buf[PROTO_MSG_SIZE];
  m.serialize(buf);
  chMtxLock(&tx_mtx);
  Serial1.write(buf, PROTO_MSG_SIZE);
  led_state = !led_state;                     // parpadeo de actividad
  digitalWrite(LED_BUILTIN, led_state ? HIGH : LOW);
  chMtxUnlock(&tx_mtx);
}

// ══════════════════════════════════════════════════════════════════════════
//  PRESENTACIÓN DE LOS FRAMES RECIBIDOS (solo el hilo principal imprime)
// ══════════════════════════════════════════════════════════════════════════
static void print_frame(const ProtoMsg& m) {
  // Si el usuario está a mitad de escribir un comando, el mensaje se
  // muestra en su propia línea y después se repinta el prompt con lo
  // que llevara tecleado: así nunca se interfiere con su entrada
  // (sugerencia del guión para las medidas asíncronas).
  const bool re_prompt = (line_len > 0);
  if (re_prompt) Serial.println();

  switch (m.opcode) {
    case OP_RESP_PENDING:
      Serial.print(F("[RX] disparo solicitado - sensor "));
      Serial.print(m.dir_sensor);
      Serial.println(F("; se mostrara la medida cuando se complete"));
      break;
    case OP_RESP_OK:
      Serial.print(F("[RX] OK - sensor "));
      Serial.println(m.dir_sensor);
      break;

    case OP_RESP_ERROR:
      Serial.print(F("[RX] ERROR - sensor "));
      Serial.print(m.dir_sensor);
      Serial.print(F(": "));
      Serial.println(error_name((uint8_t)m.data));
      break;

    case OP_RESP_MEASURE:
      Serial.print(F("[RX] medida sensor "));
      Serial.print(m.dir_sensor);
      Serial.print(F(": "));
      if (m.unit == UNIT_MS) Serial.print(m.data / 1000.0, 3);
      else Serial.print(m.data);
      Serial.print(F(" "));
      Serial.print(unit_name(m.unit));
      Serial.println(m.periodic ? F(" (periodica)") : F(" (one-shot)"));
      break;

    case OP_RESP_STATUS: {
      uint8_t addr7    = SRF02_ADDR7[m.dir_sensor];
      uint8_t unit     = m.unit;
      bool periodic   = m.periodic;
      uint32_t delay  = m.data;

      Serial.print(F("[RX] STATUS - sensor "));
      Serial.println(m.dir_sensor);
      Serial.print(F("  dir I2C   : 0x"));
      Serial.println(addr7, HEX);
      Serial.print(F("  unidad    : "));
      Serial.println(unit_name(unit));
      Serial.print(F("  periodico : "));
      Serial.println(periodic ? F("SI") : F("NO"));
      Serial.print(F("  espera onda-lectura: "));
      Serial.print(delay);
      Serial.println(F(" ms"));
      break;
    }

    case OP_RESP_LIST:
      Serial.println(F("[RX] sensores de ultrasonidos disponibles:"));
      for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        bool present = ((m.data >> i) & 0x01) != 0;
        Serial.print(F("  sensor "));
        Serial.print(i);
        Serial.print(F(" (0x"));
        Serial.print(SRF02_ADDR8[i], HEX);
        Serial.print(F(" / 7-bit 0x"));
        Serial.print(SRF02_ADDR7[i], HEX);
        Serial.println(present ? F(") : CONECTADO") : F(") : no detectado"));
      }
      break;

    default:
      Serial.print(F("[RX] frame desconocido opcode=0x"));
      Serial.println(m.opcode, HEX);
      break;
  }

  // Restaurar el prompt con lo que el usuario llevara tecleado
  if (re_prompt) {
    Serial.print(F("> "));
    Serial.write((const uint8_t*)line_buf, line_len);
  }
}

// ══════════════════════════════════════════════════════════════════════════
//  ENVÍO DE UN COMANDO Y ESPERA DE SU RESPUESTA (con timeout)
// ══════════════════════════════════════════════════════════════════════════
static void execute_command(const ProtoMsg& cmd) {
  drain_queue();
  while (chSemWaitTimeout(&rsp_sem, TIME_IMMEDIATE) == MSG_OK) { }
  send_frame(cmd);
  const systime_t start = chVTGetSystemTimeX();
  bool matched = false;
  while (!matched && chVTGetSystemTimeX() - start < TIME_MS2I(RESPONSE_TIMEOUT_MS)) {
    ProtoMsg m;
    while (queue_pop(&m)) {
      print_frame(m); // las medidas periodicas siguen mostrandose durante la espera
      if (m.dir_sensor != cmd.dir_sensor) continue;
      matched |= (m.opcode == OP_RESP_ERROR && (m.data >> 8) == cmd.opcode) ||
                 (m.opcode == OP_RESP_OK && m.data == cmd.opcode) ||
                 (cmd.opcode == OP_US_ONE_SHOT && m.opcode == OP_RESP_PENDING) ||
                 (cmd.opcode == OP_US_STATUS && m.opcode == OP_RESP_STATUS) ||
                 (cmd.opcode == OP_US_LIST && m.opcode == OP_RESP_LIST);
    }
    if (!matched) chSemWaitTimeout(&rsp_sem, TIME_MS2I(10));
  }
  if (!matched) {
    Serial.print(F("[RX] TIMEOUT - sin respuesta del sensor en "));
    Serial.print(RESPONSE_TIMEOUT_MS);
    Serial.println(F(" ms"));
    Serial.println(F("Comprueba alimentacion, TX(14)->RX(13), RX(13)->TX(14) y GND comun."));
  }
}

static bool parse_number(const char* text, uint32_t* out) {
  if (!text || !*text) return false;
  for (const char* p = text; *p; ++p) if (*p < '0' || *p > '9') return false;
  errno = 0;
  char* end;
  unsigned long value = strtoul(text, &end, 10);
  if (errno == ERANGE || *end || value > UINT32_MAX) return false;
  *out = (uint32_t)value;
  return true;
}

static bool no_extra_args(char** save) {
  if (!strtok_r(nullptr, " \t", save)) return true;
  Serial.println(F("[ERROR] sobran argumentos. Escribe 'help'."));
  return false;
}

// ══════════════════════════════════════════════════════════════════════════
//  PARSEO DE LA LÍNEA ESCRITA POR EL USUARIO
// ══════════════════════════════════════════════════════════════════════════
static bool parse_command(const char* line, ProtoMsg* out) {
  char tmp[CMD_LINE_MAX];
  strncpy(tmp, line, sizeof(tmp) - 1);
  tmp[sizeof(tmp) - 1] = '\0';

  char* save = nullptr;
  char* tok  = strtok_r(tmp, " \t", &save);
  if (!tok) return false;

  digitalWrite(LED_BUILTIN, HIGH);

  // ── help ──
  if (strcmp(tok, "help") == 0) {
    if (!no_extra_args(&save)) return false;
    print_help();
    return false;
  }

  // ── us ... ──
  if (strcmp(tok, "us") != 0) {
    Serial.print(F("[ERROR] comando desconocido: "));
    Serial.println(line);
    Serial.println(F("        escribe 'help' para ver los comandos"));
    return false;
  }

  tok = strtok_r(nullptr, " \t", &save);
  if (!tok) {                       // "us" a secas -> listado
    *out = make_command(OP_US_LIST, 0);
    return true;
  }

  // Índice del sensor (0 => 0xF2, 1 => 0xE0)
  uint32_t sidx;
  if (!parse_number(tok, &sidx) || sidx >= NUM_SENSORS) {
    Serial.print(F("[ERROR] sensor invalido: "));
    Serial.println(tok);
    return false;
  }
  uint8_t idx = (uint8_t)sidx;

  tok = strtok_r(nullptr, " \t", &save);
  if (!tok) {
    Serial.println(F("[ERROR] falta el subcomando. Escribe 'help'."));
    return false;
  }

  // ── us <s> one-shot ──
  if (strcmp(tok, "one-shot") == 0) {
    if (!no_extra_args(&save)) return false;
    *out = make_command(OP_US_ONE_SHOT, idx);
    return true;
  }

  // ── us <s> on <period_ms> ──
  if (strcmp(tok, "on") == 0) {
    char* p = strtok_r(nullptr, " \t", &save);
    if (!p) {
      Serial.println(F("[ERROR] falta el periodo: us <s> on <ms>"));
      return false;
    }
    if (!no_extra_args(&save)) return false;
    uint32_t period;
    if (!parse_number(p, &period)) {
      Serial.println(F("[ERROR] periodo no representable: usa un entero entre 0 y 4294967295 ms"));
      return false;
    }
    *out = make_command(OP_US_PERIODIC_ON, idx, UNIT_CM, true, period);
    return true;
  }

  // ── us <s> off ──
  if (strcmp(tok, "off") == 0) {
    if (!no_extra_args(&save)) return false;
    *out = make_command(OP_US_PERIODIC_OFF, idx);
    return true;
  }

  // ── us <s> unit {inc|cm|ms} ──
  if (strcmp(tok, "unit") == 0) {
    char* p    = strtok_r(nullptr, " \t", &save);
    uint8_t u  = UNIT_CM;
    if (!p) {
      Serial.println(F("[ERROR] falta la unidad: cm, inc o ms"));
      return false;
    }
    if (!no_extra_args(&save)) return false;
    if (p) {
      if      (strcmp(p, "cm")   == 0) u = UNIT_CM;
      else if (strcmp(p, "inch") == 0) u = UNIT_INCH;
      else if (strcmp(p, "inc")  == 0) u = UNIT_INCH; // como en el guion
      else if (strcmp(p, "ms")   == 0) u = UNIT_MS;
      else {
        Serial.print(F("[ERROR] unidad desconocida: "));
        Serial.println(p);
        return false;
      }
    }
    *out = make_command(OP_US_SET_UNIT, idx, u);
    return true;
  }

  // ── us <s> delay <ms> ──
  if (strcmp(tok, "delay") == 0) {
    char* p = strtok_r(nullptr, " \t", &save);
    if (!p) {
      Serial.println(F("[ERROR] falta el retardo: us <s> delay <ms>"));
      return false;
    }
    if (!no_extra_args(&save)) return false;
    uint32_t d;
    if (!parse_number(p, &d)) {
      Serial.println(F("[ERROR] retardo no representable: usa un entero entre 0 y 4294967295 ms"));
      return false;
    }
    *out = make_command(OP_US_SET_DELAY, idx, UNIT_CM, false, d);
    return true;
  }

  // ── us <s> status ──
  if (strcmp(tok, "status") == 0) {
    if (!no_extra_args(&save)) return false;
    *out = make_command(OP_US_STATUS, idx);
    return true;
  }

  Serial.print(F("[ERROR] subcomando desconocido: "));
  Serial.println(tok);
  return false;
}

static void print_help() {
  Serial.println(F(""));
  Serial.println(F("=== COMANDOS DEL SUPERVISOR ==="));
  Serial.println(F("  help                      - esta ayuda"));
  Serial.println(F("  us                        - sensores disponibles"));
  Serial.println(F("  us <s> one-shot           - un unico disparo"));
  Serial.println(F("  us <s> on <period_ms>     - disparo periodico continuo"));
  Serial.println(F("  us <s> off                - detener el disparo periodico"));
  Serial.println(F("  us <s> unit {inc|cm|ms}   - cambiar unidad de medida"));
  Serial.println(F("  us <s> delay <ms>         - espera entre disparo y lectura (minimo 70 ms)"));
  Serial.println(F("  us <s> status             - configuracion del sensor"));
  Serial.println(F("  donde <s> es el indice: 0 => sensor 0xF2, 1 => 0xE0"));
  Serial.println(F("  El SRF02 requiere >= 65 ms entre disparos; como on exige superar delay (minimo 70 ms), el periodo valido minimo es 71 ms."));
  Serial.println(F("=============================="));
}

// ══════════════════════════════════════════════════════════════════════════
//  ARRANQUE (patrón ChRt): setup() -> chBegin(chSetup) -> hilos -> loop()
// ══════════════════════════════════════════════════════════════════════════
void chSetup() {
  // Hilo receptor: solo monta y encola frames; nunca imprime, para que
  // la salida del monitor quede en manos exclusivas del hilo principal.
  chThdCreateStatic(wa_rx_thd, sizeof(wa_rx_thd),
                    NORMALPRIO + 1, rx_thd, nullptr);
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(SERIAL_MONITOR_BAUDS);
  const uint32_t usb_start = millis();
  while (!Serial && millis() - usb_start < 2000) { }
  Serial1.begin(SERIAL1_BAUDS);

  Serial.println(F("=== SUPERVISOR - Practica 2 IoT (ChibiOS/ChRt) ==="));
  Serial.println(F("Escribe 'help' para ver los comandos disponibles."));
  Serial.print(F("> "));

  // Arranca el kernel de ChibiOS y crea los hilos; no vuelve nunca.
  // OJO: tras chBegin() no se usan delay()/millis() (el SysTick es del
  // kernel); se usa chThdSleepMilliseconds().
  chBegin(chSetup);
  while (true) { }
}

void loop() {
  // En ChRt, loop() es el HILO PRINCIPAL del sistema.

  // 1) Mostrar primero lo asíncrono que haya llegado (medidas, etc.)
  drain_queue();

  // 2) Leer la entrada del usuario
  if (Serial.available() > 0) {
    char c = (char)Serial.read();

    static bool previous_cr = false;
    static bool overflow = false;
    if (c == '\n' && previous_cr) { previous_cr = false; return; }
    previous_cr = (c == '\r');
    if (c == '\r' || c == '\n') {
      Serial.println();
      line_buf[line_len] = '\0';
      line_len = 0; // no repintar el comando ya enviado al imprimir respuestas
      if (overflow) Serial.println(F("[ERROR] linea demasiado larga; comando descartado"));
      else if (line_buf[0]) {
        ProtoMsg cmd;
        if (parse_command(line_buf, &cmd)) execute_command(cmd);
      }
      overflow = false;
      Serial.print(F("> "));
    } else {
      Serial.print(c);
      if (line_len < sizeof(line_buf) - 1) line_buf[line_len++] = c;
      else overflow = true;
    }
  } else {
    chThdSleepMilliseconds(2); // sin entrada: ceder la CPU
  }
}



