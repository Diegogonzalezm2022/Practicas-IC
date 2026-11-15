/* ------------------------------------------------------------------
 * protocol.h  -  Protocolo binario supervisor <-> sensor (Práctica 2)
 *
 * Asignatura: GII-IC (Internet de las Cosas), Universidad de Las Palmas
 *             de Gran Canaria.
 *
 * FORMATO DEL MENSAJE: 48 bits = 6 bytes, transmitidos en orden
 * big-endian (el byte más significativo primero), tal y como se
 * establece en la codificación de la práctica (codificacion.jpeg):
 *
 *   bit:  47      40 39  38     35 34  33 32  31                 0
 *        +----------+----+--------+------+------+---------------------+
 *        |    op    | d- | i2cdir | uni- | per. |       datos         |
 *        |          | s  |        | dad  |      |                     |
 *        +----------+----+--------+------+------+---------------------+
 *        |  byte 0  |          byte 1          | byte 2 | 3 | 4 | byte 5|
 *
 *   bits 47..40  op        : código de operación (8 bits, 256 ops.)
 *   bit  39      dir-sensor: índice lógico del sensor
 *                            0 => sensor A (dir. I2C 0xF2)
 *                            1 => sensor B (dir. I2C 0xE0)
 *   bits 38..35  i2cdir    : nibble bajo de la dirección I2C real de
 *                            7 bits del sensor (0x79 -> 0x9,
 *                            0x70 -> 0x0). El dispositivo sensor lo
 *                            valida para comprobar que el mensaje va
 *                            dirigido a un sensor que existe en su bus.
 *   bits 34..33  unidad    : 00 = cm, 01 = inch, 10 = ms
 *                            (el dato binario conserva microsegundos;
 *                            el supervisor convierte a ms), 11 = reservado.
 *   bit  32      per.      : 1 => disparo periódico activo / orden de
 *                            arranque periódico.
 *   bits 31..0   datos     : significado según el código de operación
 *                            (big-endian).
 *
 * DATOS según la operación:
 *   OP_US_ONE_SHOT    -> datos = 0 (disparo único)
 *   OP_US_PERIODIC_ON -> datos = periodo deseado en ms
 *   OP_US_PERIODIC_OFF-> datos = 0
 *   OP_US_SET_UNIT    -> datos = 0 (la unidad va en bits 34..33)
 *   OP_US_SET_DELAY   -> datos = retardo mínimo entre disparos, en ms
 *   OP_US_STATUS      -> datos = 0
 *   OP_US_LIST        -> datos = 0
 *   OP_RESP_OK        -> datos = opcode del comando confirmado
 *   OP_RESP_ERROR     -> datos = [15:8] opcode (0: error asincrono),
 *                                [7:0] codigo de error (ver ERR_*)
 *   OP_RESP_MEASURE   -> datos = medida devuelta por el SRF02
 *   OP_RESP_PENDING   -> disparo unico aceptado; se enviara medida o error
 *   OP_RESP_STATUS    -> datos = retardo minimo completo (uint32_t, ms).
 *                        Unidad y periodico estan en los flags; la direccion
 *                        I2C se obtiene del indice y de SRF02_ADDR7.
 *   OP_RESP_LIST      -> datos = máscara de presencia (bit i = 1 si
 *                        el sensor i está conectado en el bus I2C)
 * ------------------------------------------------------------------ */

#pragma once

#include <stdint.h>

// ─── Tamaño del mensaje ────────────────────────────────────────────────────
constexpr uint8_t PROTO_MSG_SIZE = 6;

// ─── Códigos de operación: supervisor -> sensor ────────────────────────────
constexpr uint8_t OP_US_ONE_SHOT     = 0x01; // Disparo único
constexpr uint8_t OP_US_PERIODIC_ON  = 0x02; // Arrancar disparo periódico
constexpr uint8_t OP_US_PERIODIC_OFF = 0x03; // Detener disparo periódico
constexpr uint8_t OP_US_SET_UNIT     = 0x04; // Cambiar unidad de medida
constexpr uint8_t OP_US_SET_DELAY    = 0x05; // Retardo mínimo entre disparos
constexpr uint8_t OP_US_STATUS       = 0x06; // Consultar configuración/estado
constexpr uint8_t OP_US_LIST         = 0x07; // Listar sensores disponibles

// ─── Códigos de operación: sensor -> supervisor ────────────────────────────
constexpr uint8_t OP_RESP_OK      = 0x10; // Confirmación de ejecución
constexpr uint8_t OP_RESP_ERROR   = 0x11; // Incidencia / error
constexpr uint8_t OP_RESP_MEASURE = 0x12; // Medida (one-shot o periódica)
constexpr uint8_t OP_RESP_STATUS  = 0x13; // Respuesta de estado
constexpr uint8_t OP_RESP_LIST    = 0x14; // Respuesta de listado
constexpr uint8_t OP_RESP_PENDING = 0x15; // Disparo unico pendiente de realizar

// ─── Unidades de medida (bits 34..33) ──────────────────────────────────────
constexpr uint8_t UNIT_CM     = 0x00;
constexpr uint8_t UNIT_INCH   = 0x01;
constexpr uint8_t UNIT_MS     = 0x02; // dato en us; el supervisor imprime ms
constexpr uint8_t UNIT_RSVD   = 0x03; // reservado -> error

// ─── Códigos de error (viajados en OP_RESP_ERROR) ──────────────────────────
constexpr uint8_t ERR_UNKNOWN_SENSOR = 0x01; // i2cdir no corresponde al sensor
constexpr uint8_t ERR_I2C_FAIL       = 0x02; // fallo de acceso al SRF02
constexpr uint8_t ERR_UNKNOWN_CMD    = 0x03; // código de operación desconocido
constexpr uint8_t ERR_INVALID_UNIT   = 0x05; // unidad reservada (11)
constexpr uint8_t ERR_QUEUE_FULL     = 0x06; // cola de órdenes del sensor llena
constexpr uint8_t ERR_INVALID_DELAY = 0x07; // retardo mayor que el periodo activo
constexpr uint8_t ERR_PERIOD_TOO_SHORT = 0x08;

// Restricciones de la medicion, no limites arbitrarios de configuracion.
constexpr uint32_t SRF02_MIN_TRIGGER_MS = 65;
constexpr uint32_t SRF02_MEASUREMENT_MS = 70; // espera de lectura del firmware

// ─── Sensores SRF02 del sistema ────────────────────────────────────────────
// De fabrica vienen en 0xE0; para este montaje se reprograma uno a 0xF2.
// En 7 bits (lo que usa Wire) son 0x79 y 0x70.
constexpr uint8_t  NUM_SENSORS   = 2;
constexpr uint8_t  SRF02_ADDR7[NUM_SENSORS] = { 0x79, 0x70 }; // I2C 7-bit
constexpr uint8_t  SRF02_ADDR8[NUM_SENSORS] = { 0xF2, 0xE0 }; // cruda (datasheet)

// Nibble bajo de la dirección 7-bit: el campo "I2C dir" de 4 bits (38..35)
constexpr uint8_t i2c_dir_nibble(uint8_t sensor_idx) {
  return (uint8_t)(SRF02_ADDR7[sensor_idx] & 0x0F);
}

// Devuelve el índice del sensor con ese nibble, o 0xFF si no existe
constexpr uint8_t sensor_idx_from_i2c_dir(uint8_t nibble) {
  return (nibble == (SRF02_ADDR7[0] & 0x0F)) ? 0 :
         (nibble == (SRF02_ADDR7[1] & 0x0F)) ? 1 : 0xFF;
}

// ─── Codificación de la respuesta de estado (OP_RESP_STATUS) ───────────────
inline uint32_t pack_status_data(uint32_t delay_ms) {
  return delay_ms;
}

// ─── Utilidades de impresión (compartidas por ambos dispositivos) ──────────
inline const char* unit_name(uint8_t unit) {
  switch (unit) {
    case UNIT_CM:   return "cm";
    case UNIT_INCH: return "inch";
    case UNIT_MS:   return "ms";
    default:        return "?";
  }
}

inline const char* error_name(uint8_t code) {
  switch (code) {
    case ERR_UNKNOWN_SENSOR: return "sensor desconocido / dir I2C no valida";
    case ERR_I2C_FAIL:       return "fallo de acceso I2C al SRF02";
    case ERR_UNKNOWN_CMD:    return "codigo de operacion desconocido";
    case ERR_INVALID_UNIT:   return "unidad de medida no valida";
    case ERR_QUEUE_FULL:     return "cola de ordenes del sensor llena";
    case ERR_INVALID_DELAY:  return "el periodo debe ser >= al retardo configurado; no se ha cambiado nada";
    case ERR_PERIOD_TOO_SHORT: return "el firmware necesita al menos 70 ms para completar una medicion";
    default:                 return "error desconocido";
  }
}

// ─── Estructura de mensaje ─────────────────────────────────────────────────
// Representa los 48 bits del frame; serialize()/deserialize() hacen de
// puente con el formato de la imagen.
struct ProtoMsg {
  uint8_t  opcode     = 0;   // bits 47..40
  uint8_t  dir_sensor = 0;   // bit  39     (0 o 1)
  uint8_t  i2c_dir    = 0;   // bits 38..35 (nibble de la dir. I2C 7-bit)
  uint8_t  unit       = 0;   // bits 34..33
  bool     periodic   = false; // bit  32
  uint32_t data       = 0;   // bits 31..0

  // Serializa en buf[0..5] (big-endian)
  void serialize(uint8_t buf[PROTO_MSG_SIZE]) const {
    buf[0] = opcode;
    buf[1] = (uint8_t)(((dir_sensor & 0x01u) << 7) |
                       ((i2c_dir    & 0x0Fu) << 3) |
                       ((unit       & 0x03u) << 1) |
                       (periodic ? 0x01u : 0x00u));
    buf[2] = (uint8_t)(data >> 24);
    buf[3] = (uint8_t)(data >> 16);
    buf[4] = (uint8_t)(data >>  8);
    buf[5] = (uint8_t)(data);
  }

  // Deserializa desde buf[0..5]
  static ProtoMsg deserialize(const uint8_t buf[PROTO_MSG_SIZE]) {
    ProtoMsg m;
    m.opcode     = buf[0];
    m.dir_sensor = (uint8_t)((buf[1] >> 7) & 0x01);
    m.i2c_dir    = (uint8_t)((buf[1] >> 3) & 0x0F);
    m.unit       = (uint8_t)((buf[1] >> 1) & 0x03);
    m.periodic   = (buf[1] & 0x01) != 0;
    m.data       = ((uint32_t)buf[2] << 24) | ((uint32_t)buf[3] << 16) |
                   ((uint32_t)buf[4] <<  8) |  (uint32_t)buf[5];
    return m;
  }
};

// Construye un mensaje (comando o respuesta) dirigido a un sensor:
// rellena el índice y el nibble de dirección I2C de forma coherente.
inline ProtoMsg make_command(uint8_t opcode, uint8_t sensor_idx,
                             uint8_t unit = UNIT_CM,
                             bool periodic = false,
                             uint32_t data = 0) {
  ProtoMsg m;
  m.opcode     = opcode;
  m.dir_sensor = (uint8_t)(sensor_idx & 0x01);
  m.i2c_dir    = i2c_dir_nibble(sensor_idx);
  m.unit       = (uint8_t)(unit & 0x03);
  m.periodic   = periodic;
  m.data       = data;
  return m;
}

// Conserva los seis bytes del formato original. Tras una pausa larga
// descarta una trama parcial; ante bytes invalidos busca el siguiente
// candidato. Sin checksum no es posible detectar todas las corrupciones.
class ProtoReceiver {
  uint8_t buf[PROTO_MSG_SIZE] = {};
  uint8_t count = 0;
  uint32_t last_ms = 0;
public:
  bool push(uint8_t b, uint32_t now_ms, bool response, ProtoMsg* out) {
    if ((uint32_t)(now_ms - last_ms) > 100) count = 0;
    last_ms = now_ms;
    buf[count++] = b;
    if (count < PROTO_MSG_SIZE) return false;
    ProtoMsg m = ProtoMsg::deserialize(buf);
    bool valid_op = response
      ? (m.opcode >= OP_RESP_OK && m.opcode <= OP_RESP_PENDING)
      : (m.opcode >= OP_US_ONE_SHOT && m.opcode <= OP_US_LIST);
    if (valid_op && sensor_idx_from_i2c_dir(m.i2c_dir) == m.dir_sensor) {
      count = 0;
      *out = m;
      return true;
    }
    for (uint8_t i = 1; i < PROTO_MSG_SIZE; ++i) buf[i - 1] = buf[i];
    count = PROTO_MSG_SIZE - 1;
    return false;
  }
};
