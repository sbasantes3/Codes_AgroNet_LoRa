// ================================================================
//  NODO 1 (TRANSMISOR) — Heltec V4 + LoRa + Deep Sleep + Ventana
//  Paquete BINARIO (~27 bytes) + ventana de escucha post-TX para
//  recibir comandos de cambio de intervalo desde el gateway.
//
//  Flujo en cada despertar:
//    1) lee sensores  2) arma paquete  3) apaga 3.3V  4) transmite
//    5) abre ventana de escucha (1.5s) por si hay comando nuevo
//    6) duerme con el intervalo (nuevo o el que ya tenía)
// ================================================================

#include <RadioLib.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <BH1750.h>
#include <SparkFun_SCD4x_Arduino_Library.h>
#include <ModbusMaster.h>
#include "esp_sleep.h"

#define ID_NODO_NUM   1     // N01 = 1

// ── DEEP SLEEP ───────────────────────────────────────────────
#define SLEEP_DEFAULT_SEG  30      // intervalo de fábrica (prueba)
#define VENTANA_MS         4000    // cuánto escucha tras transmitir
// OJO: el downlink en SF12/BW62.5 tarda ~2.5 s en transmitirse. Con 1500 ms
// la ventana se cerraba ANTES de que llegara el comando -> el nodo nunca
// aplicaba el nuevo intervalo. 4000 ms cubre el airtime + turnaround del gateway.
#define uS_POR_SEG         1000000ULL

// ── PINES LoRa (NO TOCAR al dormir) ─────────────────────────
#define LORA_NSS   8
#define LORA_SCK   9
#define LORA_MOSI 10
#define LORA_MISO 11
#define LORA_NRST 12
#define LORA_BUSY 13
#define LORA_DIO1 14
#define FEM_EN     2
#define FEM_PA    46

// ── PINES I2C ────────────────────────────────────────────────
#define SENS_SDA   41
#define SENS_SCL   42

// ── PINES RS485 (confirmados funcionando) ───────────────────
#define RS485_RE_DE 33
#define RS485_RX    34
#define RS485_TX    38

// ── MOSFET ───────────────────────────────────────────────────
#define MOSFET_SENSORES 7

// ── BATERÍA ──────────────────────────────────────────────────
#define BAT_ADC   1
#define ADC_CTRL 37
#define BAT_MULT  4.9f   // se mantiene igual hasta tener más datos

// ── VEXT (apaga OLED) ───────────────────────────────────────
#define VEXT_CTRL 36

// ── CENTINELAS (sensor inválido) ────────────────────────────
#define INV_U8   0xFF
#define INV_U16  0xFFFF
#define INV_I16  0x7FFF

// ── PAQUETE DE DATOS (uplink, 27 bytes) ─────────────────────
typedef struct __attribute__((packed)) {
  uint8_t  id_nodo;
  int16_t  temp_aire;
  uint8_t  hum_aire;
  uint16_t presion;
  uint16_t co2;
  uint16_t lux;
  uint16_t hum_suelo;
  int16_t  temp_suelo;
  uint16_t ec_suelo;
  uint16_t ph_suelo;
  uint16_t n_suelo;
  uint16_t p_suelo;
  uint16_t k_suelo;
  uint16_t voltaje;
  uint8_t  msg_id;
} PaqueteSensor;

// ── PAQUETE DE COMANDO (downlink, 3 bytes) ──────────────────
typedef struct __attribute__((packed)) {
  uint8_t  id_nodo;        // a quién va dirigido
  uint16_t intervalo_seg;  // nuevo intervalo de sueño
} ComandoDescarga;

// ── PERSISTE EN DEEP SLEEP ───────────────────────────────────
RTC_DATA_ATTR uint8_t  contador_msg     = 0;
RTC_DATA_ATTR uint32_t num_ciclo        = 0;
RTC_DATA_ATTR uint32_t intervalo_actual = SLEEP_DEFAULT_SEG;

// ── OBJETOS ──────────────────────────────────────────────────
SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_NRST, LORA_BUSY);
TwoWire           sensWire = TwoWire(1);
Adafruit_BME280   bme;
BH1750            bh1750;
SCD4x             scd40;
HardwareSerial    RS485(1);
ModbusMaster      modbus;

// ── ESTADO ───────────────────────────────────────────────────
bool ok_bme=false, ok_scd=false, ok_bh=false, ok_suelo=false;
// Validez POR REGISTRO del sensor de suelo: un registro que no responde ya no
// se transmite como 0 (ver leerSensorSuelo).
bool ok_s_hum=false, ok_s_temp=false, ok_s_ec=false, ok_s_ph=false;
bool ok_s_n=false, ok_s_p=false, ok_s_k=false;
float bme_temp=0, bme_hum=0, bme_pres=0;
float scd_co2=0;
uint16_t presion_aplicada=0;
float lux=0;
float    suelo_hum=0, suelo_temp=0, suelo_ph=0;
uint16_t suelo_ec=0, suelo_n=0, suelo_p=0, suelo_k=0;
float voltaje=0;

volatile bool comandoRecibido = false;
void IRAM_ATTR setFlagNodo(void) { comandoRecibido = true; }

// ── RS485 ────────────────────────────────────────────────────
void preTransmission()  { digitalWrite(RS485_RE_DE, HIGH); }
void postTransmission() { digitalWrite(RS485_RE_DE, LOW); }

bool leerRegistroSuelo(uint16_t dir, uint16_t &val) {
  for (int intento=0; intento<3; intento++) {
    if (modbus.readHoldingRegisters(dir, 1) == modbus.ku8MBSuccess) {
      val = modbus.getResponseBuffer(0);
      return true;
    }
    delay(50);
  }
  return false;
}

// El sensor sale de reset detrás de su elevador a 12 V y tarda en asentarse:
// mientras tanto responde Modbus con la humedad en 0 y la CE subiendo por una
// rampa (medido 2026-07-09: 171 -> 312 -> 499 -> 542, siendo ~540 el valor real;
// el NPK la sigue porque se deriva de la CE, y solo el pH sale correcto).
// Leer en ese estado produce datos con pinta de válidos pero falsos.
#define SUELO_ASENT_TIMEOUT_MS  10000  // margen máximo de espera
#define SUELO_ASENT_TOLERANCIA  10     // CE estable si 2 lecturas difieren < 10
#define SUELO_ASENT_PAUSA_MS    400

// Espera a que el sensor termine de asentarse: la CE deja de subir Y la humedad
// deja de marcar 0 exacto (en tierra nunca es 0.0; ver escala real del sensor).
bool esperarSensorAsentado() {
  uint16_t ce_prev = 0, ce = 0, hum = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < SUELO_ASENT_TIMEOUT_MS) {
    delay(SUELO_ASENT_PAUSA_MS);
    if (!leerRegistroSuelo(0x0015, ce))  continue;   // CE
    if (!leerRegistroSuelo(0x0012, hum)) continue;   // humedad
    bool ce_estable = (ce_prev != 0) &&
                      (abs((int)ce - (int)ce_prev) < SUELO_ASENT_TOLERANCIA);
    if (ce_estable && hum != 0) {
      Serial.printf("[SUELO] asentado en %lu ms (CE=%u hum=%.1f)\n",
                    millis() - t0, ce, hum / 10.0);
      return true;
    }
    ce_prev = ce;
  }
  Serial.println("[SUELO] NO se asentó dentro del timeout -> trama inválida");
  return false;
}

void leerSensorSuelo() {
  ok_suelo = false;
  ok_s_hum = ok_s_temp = ok_s_ec = ok_s_ph = false;
  ok_s_n = ok_s_p = ok_s_k = false;

  if (!esperarSensorAsentado()) return;

  uint16_t v_hum=0,v_temp=0,v_ec=0,v_ph=0,v_n=0,v_p=0,v_k=0;
  // Se comprueba el retorno de LOS SIETE registros. Antes solo se miraba el de
  // humedad y los otros seis se quedaban en su valor inicial 0, que viajaba a la
  // BD como dato bueno (de ahí las filas con temp_suelo=0 o N=0).
  ok_s_hum  = leerRegistroSuelo(0x0012, v_hum);  delay(100);
  ok_s_temp = leerRegistroSuelo(0x0013, v_temp); delay(100);
  ok_s_ec   = leerRegistroSuelo(0x0015, v_ec);   delay(100);
  ok_s_ph   = leerRegistroSuelo(0x0006, v_ph);   delay(100);
  ok_s_n    = leerRegistroSuelo(0x001E, v_n);    delay(100);
  ok_s_p    = leerRegistroSuelo(0x001F, v_p);    delay(100);
  ok_s_k    = leerRegistroSuelo(0x0020, v_k);    delay(100);

  // Humedad 0 exacto o pH 0 exacto = el bus respondió pero el sensor no mide.
  if (v_hum == 0) ok_s_hum = false;
  if (v_ph  == 0) ok_s_ph  = false;

  // Si la humedad no es fiable, el bloque entero de esa trama es sospechoso:
  // la CE viene sin asentar y el NPK se deriva de ella.
  if (!ok_s_hum) {
    Serial.println("[SUELO] humedad inválida -> se descarta el bloque completo");
    return;
  }

  ok_suelo   = true;
  suelo_hum  = v_hum / 10.0;
  suelo_temp = (int16_t)v_temp / 10.0;
  suelo_ec   = v_ec;
  suelo_ph   = v_ph / 100.0;
  suelo_n    = v_n;
  suelo_p    = v_p;
  suelo_k    = v_k;
}

// ── ENERGÍA ──────────────────────────────────────────────────
void encenderSensores() {
  Serial.println("[MOSFET] ON → encendiendo sensores...");
  RS485.end();
  delay(10);
  digitalWrite(MOSFET_SENSORES, HIGH);
  delay(800);

  sensWire.begin(SENS_SDA, SENS_SCL, 100000);
  delay(100);

  ok_bme = bme.begin(0x76, &sensWire);
  if (!ok_bme) ok_bme = bme.begin(0x77, &sensWire);
  if (ok_bme) {
    bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                    Adafruit_BME280::SAMPLING_X2, Adafruit_BME280::SAMPLING_X2,
                    Adafruit_BME280::SAMPLING_X2, Adafruit_BME280::FILTER_X2,
                    Adafruit_BME280::STANDBY_MS_500);
  }

  ok_scd = scd40.begin(sensWire, false, false);  // ASC off
  if (ok_scd) {
    scd40.stopPeriodicMeasurement();
    delay(500);
    scd40.startPeriodicMeasurement();
  }

  ok_bh = bh1750.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &sensWire);
  if (!ok_bh) ok_bh = bh1750.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x5C, &sensWire);

  pinMode(RS485_RE_DE, OUTPUT);
  digitalWrite(RS485_RE_DE, LOW);
  RS485.begin(9600, SERIAL_8N1, RS485_RX, RS485_TX);
  delay(50);
  modbus.begin(1, RS485);
  modbus.preTransmission(preTransmission);
  modbus.postTransmission(postTransmission);
  delay(2000);

  if (ok_bme && ok_scd) {
    delay(200);
    bme_pres = bme.readPressure() / 100.0;
    if (scd40.setAmbientPressure((uint16_t)bme_pres))
      presion_aplicada = (uint16_t)bme_pres;
  }

  Serial.printf("[INIT] BME:%s  SCD40:%s  BH1750:%s  RS485:listo\n",
                ok_bme?"OK":"FAIL", ok_scd?"OK":"FAIL", ok_bh?"OK":"FAIL");
}

void apagarSensores() {
  RS485.end();
  pinMode(RS485_RX, INPUT);
  pinMode(RS485_TX, INPUT);
  pinMode(RS485_RE_DE, INPUT);
  sensWire.end();
  pinMode(SENS_SDA, INPUT);
  pinMode(SENS_SCL, INPUT);
  digitalWrite(MOSFET_SENSORES, LOW);
  Serial.println("[MOSFET] OFF → 3.3V cortado");
}

// ── BATERÍA (promedio de 10 lecturas) ──────────────────────
void leerBateria() {
  pinMode(BAT_ADC, INPUT);
  pinMode(ADC_CTRL, OUTPUT);
  digitalWrite(ADC_CTRL, HIGH);
  delay(10);
  long suma = 0;
  for (int i=0; i<10; i++) { suma += analogRead(BAT_ADC); delay(2); }
  digitalWrite(ADC_CTRL, LOW);
  int adc_raw = suma / 10;
  voltaje = (adc_raw / 4095.0) * 3.3 * BAT_MULT;
}

// ── LEER TODO ────────────────────────────────────────────────
void leerTodo() {
  if (ok_bme) {
    bme_temp = bme.readTemperature();
    bme_hum  = bme.readHumidity();
    bme_pres = bme.readPressure() / 100.0;
  }
  if (ok_scd) {
    unsigned long t0 = millis();
    while (millis()-t0 < 5500) {
      if (scd40.readMeasurement()) { scd_co2 = scd40.getCO2(); break; }
      delay(100);
    }
  }
  if (ok_bh) lux = bh1750.readLightLevel();
  leerSensorSuelo();
  leerBateria();
}

// ── ARMAR PAQUETE ────────────────────────────────────────────
PaqueteSensor armarPaqueteBinario() {
  PaqueteSensor p;
  p.id_nodo    = ID_NODO_NUM;
  p.temp_aire  = ok_bme   ? (int16_t)(bme_temp*10)  : INV_I16;
  p.hum_aire   = ok_bme   ? (uint8_t)(bme_hum)      : INV_U8;
  p.presion    = ok_bme   ? (uint16_t)(bme_pres)    : INV_U16;
  p.co2        = ok_scd   ? (uint16_t)(scd_co2)     : INV_U16;
  p.lux        = ok_bh    ? (uint16_t)(lux)         : INV_U16;
  // Cada registro viaja con su propia validez: un registro que no respondió sale
  // como sentinela (-99 tras el gateway), nunca como 0.
  bool s = ok_suelo;
  p.hum_suelo  = (s && ok_s_hum ) ? (uint16_t)(suelo_hum*10): INV_U16;
  p.temp_suelo = (s && ok_s_temp) ? (int16_t)(suelo_temp*10): INV_I16;
  p.ec_suelo   = (s && ok_s_ec  ) ? suelo_ec                : INV_U16;
  p.ph_suelo   = (s && ok_s_ph  ) ? (uint16_t)(suelo_ph*100): INV_U16;
  p.n_suelo    = (s && ok_s_n   ) ? suelo_n                 : INV_U16;
  p.p_suelo    = (s && ok_s_p   ) ? suelo_p                 : INV_U16;
  p.k_suelo    = (s && ok_s_k   ) ? suelo_k                 : INV_U16;
  p.voltaje    = (uint16_t)(voltaje*100);
  p.msg_id     = contador_msg;
  return p;
}

// ── IMPRIMIR (depuración) ────────────────────────────────────
void imprimirSerial() {
  Serial.printf("\n┌─ CICLO #%lu   msg_id=%d   intervalo=%lus\n",
                num_ciclo, contador_msg, intervalo_actual);
  if (ok_bme) Serial.printf("│ BME280 : T=%.1f H=%.0f P=%.1f\n", bme_temp, bme_hum, bme_pres);
  else        Serial.println("│ BME280 : [FAIL]");
  if (ok_scd) Serial.printf("│ SCD40  : CO2=%.0f ppm\n", scd_co2);
  else        Serial.println("│ SCD40  : [FAIL]");
  if (ok_bh)  Serial.printf("│ BH1750 : %.1f lx\n", lux);
  else        Serial.println("│ BH1750 : [FAIL]");
  if (ok_suelo) {
    Serial.printf("│ SUELO  : H=%.1f T=%.1f EC=%d pH=%.2f\n", suelo_hum, suelo_temp, suelo_ec, suelo_ph);
    Serial.printf("│          N=%d P=%d K=%d\n", suelo_n, suelo_p, suelo_k);
  } else Serial.println("│ SUELO  : [NO RESPONDE]");
  Serial.printf("│ Batería: %.2f V\n", voltaje);
  Serial.println("└──────────────────────────");
}

// ── TRANSMITIR DATOS ──────────────────────────────────────────
void transmitirBinario(PaqueteSensor p) {
  Serial.printf("[LoRa] TX datos (%d bytes)\n", (int)sizeof(p));
  int estado = radio.transmit((uint8_t*)&p, sizeof(p));
  if (estado == RADIOLIB_ERR_NONE) Serial.println("[LoRa] TX OK");
  else Serial.printf("[LoRa] TX FALLO codigo %d\n", estado);
}

// ── VENTANA DE ESCUCHA (downlink) ────────────────────────────
void ventanaDeEscucha() {
  Serial.println("[VENTANA] Escuchando posible comando...");
  comandoRecibido = false;
  radio.setDio1Action(setFlagNodo);
  radio.startReceive();

  unsigned long t0 = millis();
  while (millis() - t0 < VENTANA_MS) {
    if (comandoRecibido) {
      comandoRecibido = false;
      uint8_t buf[10];
      size_t len = radio.getPacketLength();
      int est = radio.readData(buf, len);
      if (est == RADIOLIB_ERR_NONE && len == sizeof(ComandoDescarga)) {
        ComandoDescarga c;
        memcpy(&c, buf, sizeof(c));
        if (c.id_nodo == ID_NODO_NUM) {
          intervalo_actual = c.intervalo_seg;
          Serial.printf("[VENTANA] Comando recibido. Nuevo intervalo: %u s\n", c.intervalo_seg);
        }
      }
      break;
    }
    delay(10);
  }
  radio.standby();
  Serial.println("[VENTANA] Cerrada.");
}

// ── PREPARAR PINES PARA DORMIR (sin tocar LoRa ni batería) ──
void prepararPinesParaDormir() {
  pinMode(SENS_SDA, INPUT);
  pinMode(SENS_SCL, INPUT);
  pinMode(RS485_RX, INPUT);
  pinMode(RS485_TX, INPUT);
  pinMode(RS485_RE_DE, INPUT);
  pinMode(MOSFET_SENSORES, INPUT);  // pull-down lo mantiene apagado
  pinMode(VEXT_CTRL, OUTPUT);
  digitalWrite(VEXT_CTRL, HIGH);
  // NO se tocan: LoRa (8-14,2,46) ni batería (1,37)
}

// ── SETUP = UN CICLO COMPLETO, LUEGO DUERME ─────────────────
void setup() {
  Serial.begin(115200);
  delay(300);

  num_ciclo++;
  contador_msg++;

  pinMode(VEXT_CTRL, OUTPUT);
  digitalWrite(VEXT_CTRL, HIGH);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  Serial.println("\n========================================");
  Serial.printf("   NODO N01 — DESPERTAR #%lu\n", num_ciclo);
  Serial.println("========================================");

  pinMode(MOSFET_SENSORES, OUTPUT);
  digitalWrite(MOSFET_SENSORES, LOW);

  // LoRa
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
  pinMode(FEM_EN, OUTPUT);
  digitalWrite(FEM_EN, HIGH);
  int estadoLora = radio.begin(915.0, 62.5, 12, 8, 0x12, 22, 20, 1.8);
  if (estadoLora == RADIOLIB_ERR_NONE) {
    radio.setRfSwitchPins(RADIOLIB_NC, FEM_PA);
    radio.setCurrentLimit(140.0);
    radio.setCRC(true);
  } else {
    Serial.printf("[ERROR] LoRa codigo %d\n", estadoLora);
  }

  // 1) Leer sensores
  encenderSensores();
  leerTodo();
  imprimirSerial();

  // 2) Armar paquete con sensores aún encendidos
  PaqueteSensor pkt = armarPaqueteBinario();

  // 3) Apagar 3.3V
  apagarSensores();
  delay(300);

  // 4) Transmitir
  if (estadoLora == RADIOLIB_ERR_NONE) {
    transmitirBinario(pkt);
    // 5) Ventana de escucha por comando
    ventanaDeEscucha();
    radio.sleep();
  }

  // 6) Pines a bajo consumo
  prepararPinesParaDormir();

  // 7) Dormir con el intervalo (nuevo o el que ya tenía)
  Serial.printf("[SLEEP] Durmiendo %lu s...\n\n", intervalo_actual);
  Serial.flush();
  esp_sleep_enable_timer_wakeup(intervalo_actual * uS_POR_SEG);
  esp_deep_sleep_start();
}

void loop() {
  // vacío: todo ocurre en setup()
}

// ================================================================
//  GATEWAY (RECEPTOR) — Heltec V4 + LoRa
//  Recibe paquete binario, lo traduce a JSON (para el sistema web
//  / Raspberry Pi, sin cambios en ese formato), y permite mandar
//  comandos de cambio de intervalo a un nodo específico por Serial.
//
//  Para mandar un comando, escribe en el Serial Monitor:
//     N01:300
//  (significa: al nodo N01, en su próxima transmisión, dile que
//   su nuevo intervalo de sueño sea 300 segundos)
// ================================================================

#include <RadioLib.h>

#define LORA_NSS   8
#define LORA_SCK   9
#define LORA_MOSI 10
#define LORA_MISO 11
#define LORA_NRST 12
#define LORA_BUSY 13
#define LORA_DIO1 14
#define FEM_EN     2
#define FEM_PA    46

#define INV_U8   0xFF
#define INV_U16  0xFFFF
#define INV_I16  0x7FFF

// ── MISMA ESTRUCTURA QUE EL TRANSMISOR (uplink) ─────────────
typedef struct __attribute__((packed)) {
  uint8_t  id_nodo;
  int16_t  temp_aire;
  uint8_t  hum_aire;
  uint16_t presion;
  uint16_t co2;
  uint16_t lux;
  uint16_t hum_suelo;
  int16_t  temp_suelo;
  uint16_t ec_suelo;
  uint16_t ph_suelo;
  uint16_t n_suelo;
  uint16_t p_suelo;
  uint16_t k_suelo;
  uint16_t voltaje;
  uint8_t  msg_id;
} PaqueteSensor;

// ── COMANDO (downlink) ──────────────────────────────────────
typedef struct __attribute__((packed)) {
  uint8_t  id_nodo;
  uint16_t intervalo_seg;
} ComandoDescarga;

SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_NRST, LORA_BUSY);

volatile bool paqueteRecibido = false;
int contador = 0;
int ultimo_msg_id = -1;

// Comando pendiente para mandar al próximo paquete de ese nodo
int      comando_pendiente_id        = -1;
uint16_t comando_pendiente_intervalo = 0;

void IRAM_ATTR setFlag(void) { paqueteRecibido = true; }

// ── PROCESAR PAQUETE DE DATOS ────────────────────────────────
void procesar(PaqueteSensor p, float rssi, float snr) {
  contador++;

  float t   = (p.temp_aire  == INV_I16) ? -99 : p.temp_aire/10.0;
  float h   = (p.hum_aire   == INV_U8 ) ? -99 : (float)p.hum_aire;
  float pr  = (p.presion    == INV_U16) ? -99 : (float)p.presion;
  float co2 = (p.co2        == INV_U16) ? -99 : (float)p.co2;
  float lx  = (p.lux        == INV_U16) ? -99 : (float)p.lux;
  float sh  = (p.hum_suelo  == INV_U16) ? -99 : p.hum_suelo/10.0;
  float st  = (p.temp_suelo == INV_I16) ? -99 : p.temp_suelo/10.0;
  float ec  = (p.ec_suelo   == INV_U16) ? -99 : (float)p.ec_suelo;
  float ph  = (p.ph_suelo   == INV_U16) ? -99 : p.ph_suelo/100.0;
  float n   = (p.n_suelo    == INV_U16) ? -99 : (float)p.n_suelo;
  float pp  = (p.p_suelo    == INV_U16) ? -99 : (float)p.p_suelo;
  float k   = (p.k_suelo    == INV_U16) ? -99 : (float)p.k_suelo;
  float v   = p.voltaje/100.0;

  bool duplicado = (p.msg_id == ultimo_msg_id);
  ultimo_msg_id = p.msg_id;

  Serial.printf("\n╔═ PAQUETE #%d  (msg_id=%d%s) ════\n",
                contador, p.msg_id, duplicado ? " DUPLICADO" : "");
  Serial.printf("║ Nodo N%02d   Señal: RSSI %.0f dBm  SNR %.1f dB\n",
                p.id_nodo, rssi, snr);
  Serial.println("╠─ Ambiente ──────────────────────");
  Serial.printf("║ Temp:%.1f C  Hum:%.0f%%  Pres:%.0f hPa\n", t, h, pr);
  Serial.printf("║ CO2:%.0f ppm   Luz:%.0f lx\n", co2, lx);
  Serial.println("╠─ Suelo ─────────────────────────");
  Serial.printf("║ Hum:%.1f%%  Temp:%.1f C  EC:%.0f uS/cm  pH:%.2f\n", sh, st, ec, ph);
  Serial.printf("║ N:%.0f  P:%.0f  K:%.0f mg/kg\n", n, pp, k);
  Serial.printf("║ Batería: %.2f V\n", v);
  Serial.println("╚═════════════════════════════════");

  // Línea JSON para el sistema web (sin cambios en este formato)
  Serial.printf("JSON:{\"id\":\"N%02d\",\"t\":%.1f,\"h\":%.0f,\"p\":%.0f,"
    "\"co2\":%.0f,\"lux\":%.0f,\"sh\":%.1f,\"st\":%.1f,\"ec\":%.0f,"
    "\"ph\":%.2f,\"n\":%.0f,\"p_\":%.0f,\"k\":%.0f,\"v\":%.2f,"
    "\"msg\":%d,\"rssi\":%.0f,\"snr\":%.1f}\n",
    p.id_nodo, t, h, pr, co2, lx, sh, st, ec, ph, n, pp, k, v,
    p.msg_id, rssi, snr);
}

// ── LEER COMANDOS DESDE EL SERIAL (tú escribes: N01:300) ────
void revisarComandoSerial() {
  if (!Serial.available()) return;
  String linea = Serial.readStringUntil('\n');
  linea.trim();
  if (linea.length() == 0) return;

  int idx = linea.indexOf(':');
  if (idx <= 0) {
    Serial.println("[CMD] Formato inválido. Usa: N01:300");
    return;
  }

  String parteId  = linea.substring(0, idx);   // "N01"
  String parteSeg = linea.substring(idx + 1);  // "300"
  int idNum = parteId.substring(1).toInt();    // quita la "N" → 1
  int segs  = parteSeg.toInt();

  if (idNum > 0 && segs > 0) {
    comando_pendiente_id        = idNum;
    comando_pendiente_intervalo = segs;
    Serial.printf("[CMD] Pendiente para N%02d: %d s "
                  "(se enviará en su próxima transmisión)\n", idNum, segs);
  } else {
    Serial.println("[CMD] Formato inválido. Usa: N01:300");
  }
}

// ── SETUP ────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n========================================");
  Serial.println("   GATEWAY — binario → JSON + comandos");
  Serial.println("   Escribe 'N01:300' para cambiar intervalo");
  Serial.println("========================================");

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
  pinMode(FEM_EN, OUTPUT);
  digitalWrite(FEM_EN, HIGH);

  int estado = radio.begin(915.0, 62.5, 12, 8, 0x12, 22, 20, 1.8);
  if (estado != RADIOLIB_ERR_NONE) {
    Serial.printf("[ERROR] LoRa codigo %d\n", estado);
    while (true) delay(1000);
  }
  radio.setRfSwitchPins(RADIOLIB_NC, FEM_PA);
  radio.setCurrentLimit(140.0);
  radio.setCRC(true);

  radio.setDio1Action(setFlag);
  radio.startReceive();
  Serial.println("[OK] Escuchando...\n");
}

// ── LOOP ─────────────────────────────────────────────────────
void loop() {
  if (paqueteRecibido) {
    paqueteRecibido = false;

    uint8_t buf[40];
    size_t len = radio.getPacketLength();
    int estado = radio.readData(buf, len);

    if (estado == RADIOLIB_ERR_NONE && len == sizeof(PaqueteSensor)) {
      PaqueteSensor p;
      memcpy(&p, buf, sizeof(p));
      procesar(p, radio.getRSSI(), radio.getSNR());

      // ¿Hay un comando pendiente para este nodo? Responder YA,
      // mientras el nodo todavía tiene su ventana abierta.
      if (comando_pendiente_id == p.id_nodo) {
        ComandoDescarga c;
        c.id_nodo       = p.id_nodo;
        c.intervalo_seg = comando_pendiente_intervalo;
        int est2 = radio.transmit((uint8_t*)&c, sizeof(c));
        if (est2 == RADIOLIB_ERR_NONE) {
          Serial.printf("[DOWNLINK] Enviado a N%02d: nuevo intervalo=%us\n",
                        c.id_nodo, c.intervalo_seg);
        } else {
          Serial.printf("[DOWNLINK] Error al enviar: %d\n", est2);
        }
        comando_pendiente_id = -1;
      }
    } else if (estado == RADIOLIB_ERR_NONE) {
      Serial.printf("[RX] tamaño inesperado: %d bytes (esperaba %d)\n",
                    (int)len, (int)sizeof(PaqueteSensor));
    } else {
      Serial.printf("[RX] error: %d\n", estado);
    }
    radio.startReceive();
  }

  revisarComandoSerial();
}