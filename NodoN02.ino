CÓDIGO 1 — NODO N02 (RIEGO)
cpp// ================================================================
//  NODO 2 (RIEGO) — Heltec V4 + LoRa + Deep Sleep + Ventana
//  Paquete BINARIO uplink + ventana de escucha post-TX para
//  recibir comandos de riego (litros + nivel de fuerza) e
//  intervalo de sueño desde el gateway.
//
//  Flujo en cada despertar:
//    1) enciende nivel, mide tanque, apaga nivel
//    2) arma paquete de estado  3) transmite
//    4) abre ventana de escucha (1.5s) por si hay comando
//    5) si hay comando de riego -> verifica seguridad -> riega
//    6) arma paquete de resultado -> transmite
//    7) duerme con el intervalo (nuevo o el que ya tenía)
// ================================================================

#include <RadioLib.h>
#include "esp_sleep.h"

#define ID_NODO_NUM   2     // N02 = 2

// ── DEEP SLEEP ───────────────────────────────────────────────
#define SLEEP_DEFAULT_SEG  30      // intervalo de fábrica (prueba)
#define VENTANA_MS         1500    // cuánto escucha tras transmitir
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

// ── PINES SENSORES / ACTUADOR ───────────────────────────────
#define PIN_PWR_NIVEL   4     // alimenta VCC del HC-SR04 (corte de energía)
#define PIN_PWR_FLUJO   5     // alimenta VCC del YF-S201 (corte de energía)
#define PIN_TRIG       33
#define PIN_ECHO       34
#define PIN_FLOW        6
#define PIN_BOMBA      21     // módulo dual-MOS PWM (acepta 3.3V lógico)
#define FREC_PWM     1000
#define RESOLUCION      8

// ── BATERÍA ──────────────────────────────────────────────────
#define BAT_ADC   1
#define ADC_CTRL 37
#define BAT_MULT  4.9f

// ── VEXT (apaga OLED) ───────────────────────────────────────
#define VEXT_CTRL 36

// ── CONFIG BALDE REY PLAST 12.3L (calibrado y funcional) ────
const float ALTURA_TOTAL_TANQUE     = 27.8;  // lectura real sensor vacío
const float DISTANCIA_MINIMA_LLENO  = 3.0;   // mínimo que lee el sensor (lleno)
const float CAPACIDAD_MAXIMA_LITROS = 12.3;  // capacidad real del fabricante

// ── SEGURIDAD DE RIEGO ──────────────────────────────────────
const float PORCENTAJE_MINIMO_RIEGO = 15.0;   // no regar bajo este nivel
const unsigned long TIEMPO_LIMITE_MS = 60000; // corte de seguridad: 60s máx

// ── FLUJO ────────────────────────────────────────────────────
const float FACTOR_CONVERSION = 7.5;  // F(Hz) = 7.5 * Q(L/min)

// ── NIVELES DE FUERZA DE BOMBA (PWM 8 bits) ─────────────────
#define PWM_BAJO   153   // ~60%
#define PWM_MEDIO  204   // ~80%
#define PWM_ALTO   255   // 100%

// ── CENTINELAS ───────────────────────────────────────────────
#define INV_U8   0xFF
#define INV_U16  0xFFFF
#define INV_I16  0x7FFF

// ── PAQUETE UPLINK (estado + resultado de riego) ────────────
typedef struct _attribute_((packed)) {
  uint8_t  id_nodo;
  uint16_t distancia;        // cm *10
  uint8_t  porcentaje;       // 0-100
  uint16_t litros_tanque;    // L *100
  uint8_t  rego;             // 0=no rego, 1=rego
  uint16_t litros_entregados;// L *100 (0 si no regó)
  uint16_t duracion_riego;   // segundos
  uint8_t  motivo_stop;      // 0=n/a 1=volumen 2=tiempo 3=sin_agua
  uint16_t voltaje;          // V *100
  uint8_t  msg_id;
} PaqueteRiego;

// ── PAQUETE COMANDO DOWNLINK ────────────────────────────────
typedef struct _attribute_((packed)) {
  uint8_t  id_nodo;          // a quién va dirigido
  uint16_t intervalo_seg;    // nuevo intervalo (0 = no cambiar)
  uint16_t litros_objetivo;  // L *100 a regar (0 = no regar)
  uint8_t  nivel_fuerza;     // 1=bajo 2=medio 3=alto
} ComandoDescarga;

// ── PERSISTE EN DEEP SLEEP ───────────────────────────────────
RTC_DATA_ATTR uint8_t  contador_msg     = 0;
RTC_DATA_ATTR uint32_t num_ciclo        = 0;
RTC_DATA_ATTR uint32_t intervalo_actual = SLEEP_DEFAULT_SEG;

// ── OBJETOS ──────────────────────────────────────────────────
SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_NRST, LORA_BUSY);

// ── ESTADO NIVEL ─────────────────────────────────────────────
float distancia_cm = 0, agua_cm = 0, porcentaje_tanque = 0, litros_tanque = 0;
bool nivel_ok = false;

// ── ESTADO FLUJO / RIEGO ─────────────────────────────────────
volatile unsigned long pulsos = 0;
float volumen_regado = 0.0;
portMUX_TYPE muxPulsos = portMUX_INITIALIZER_UNLOCKED;

bool     rego_ejecutado   = false;
uint16_t litros_entregados = 0;
uint16_t duracion_riego    = 0;
uint8_t  motivo_stop       = 0;

// ── BATERÍA ──────────────────────────────────────────────────
float voltaje = 0;

// ── FLAG LoRa ────────────────────────────────────────────────
volatile bool comandoRecibido = false;
void IRAM_ATTR setFlagNodo(void) { comandoRecibido = true; }

// ── ISR FLUJO ────────────────────────────────────────────────
void IRAM_ATTR contarPulsos() {
  portENTER_CRITICAL_ISR(&muxPulsos);
  pulsos++;
  portEXIT_CRITICAL_ISR(&muxPulsos);
}

// ── ENERGÍA SENSORES ────────────────────────────────────────
void encenderNivel() {
  pinMode(PIN_PWR_NIVEL, OUTPUT);
  digitalWrite(PIN_PWR_NIVEL, HIGH);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  delay(200);  // estabilización del HC-SR04
}

void apagarNivel() {
  digitalWrite(PIN_PWR_NIVEL, LOW);
  pinMode(PIN_TRIG, INPUT);
  pinMode(PIN_ECHO, INPUT);
}

void encenderFlujo() {
  pinMode(PIN_PWR_FLUJO, OUTPUT);
  digitalWrite(PIN_PWR_FLUJO, HIGH);
  pinMode(PIN_FLOW, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_FLOW), contarPulsos, FALLING);
  delay(100);
}

void apagarFlujo() {
  detachInterrupt(digitalPinToInterrupt(PIN_FLOW));
  digitalWrite(PIN_PWR_FLUJO, LOW);
  pinMode(PIN_FLOW, INPUT);
}

// ── LECTURA NIVEL (promedio de 5, lógica calibrada) ─────────
void leerNivelTanque() {
  float suma = 0; int validas = 0;
  for (int i = 0; i < 5; i++) {
    digitalWrite(PIN_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_TRIG, LOW);

    long duration = pulseIn(PIN_ECHO, HIGH, 30000);
    float d = (duration * 0.0343) / 2;
    if (duration > 0 && d <= 400) { suma += d; validas++; }
    delay(60);
  }

  if (validas == 0) { nivel_ok = false; return; }

  distancia_cm = suma / validas;
  if (distancia_cm < DISTANCIA_MINIMA_LLENO) distancia_cm = DISTANCIA_MINIMA_LLENO;
  if (distancia_cm > ALTURA_TOTAL_TANQUE)    distancia_cm = ALTURA_TOTAL_TANQUE;

  float rangoUtil   = ALTURA_TOTAL_TANQUE - DISTANCIA_MINIMA_LLENO;
  agua_cm           = ALTURA_TOTAL_TANQUE - distancia_cm;
  porcentaje_tanque = (agua_cm / rangoUtil) * 100.0;
  litros_tanque     = (porcentaje_tanque / 100.0) * CAPACIDAD_MAXIMA_LITROS;
  nivel_ok = true;
}

// ── BATERÍA ──────────────────────────────────────────────────
void leerBateria() {
  pinMode(BAT_ADC, INPUT);
  pinMode(ADC_CTRL, OUTPUT);
  digitalWrite(ADC_CTRL, HIGH);
  delay(10);
  long suma = 0;
  for (int i = 0; i < 10; i++) { suma += analogRead(BAT_ADC); delay(2); }
  digitalWrite(ADC_CTRL, LOW);
  int adc_raw = suma / 10;
  voltaje = (adc_raw / 4095.0) * 3.3 * BAT_MULT;
}

// ── EJECUTAR RIEGO (litros objetivo + nivel de fuerza) ──────
void ejecutarRiego(float litrosObjetivo, uint8_t nivelFuerza) {
  // Seguridad: ¿hay agua suficiente?
  if (!nivel_ok || porcentaje_tanque < PORCENTAJE_MINIMO_RIEGO) {
    Serial.println("[RIEGO] CANCELADO - tanque bajo el mínimo de seguridad");
    rego_ejecutado    = false;
    litros_entregados = 0;
    duracion_riego    = 0;
    motivo_stop       = 3;  // sin_agua
    return;
  }

  // Seleccionar PWM según nivel de fuerza
  uint8_t pwm;
  switch (nivelFuerza) {
    case 1:  pwm = PWM_BAJO;  break;
    case 2:  pwm = PWM_MEDIO; break;
    case 3:  pwm = PWM_ALTO;  break;
    default: pwm = PWM_MEDIO; break;
  }

  Serial.printf("[RIEGO] INICIO - objetivo %.2f L, fuerza nivel %d (PWM %d)\n",
                litrosObjetivo, nivelFuerza, pwm);

  encenderFlujo();
  volumen_regado = 0.0;
  portENTER_CRITICAL(&muxPulsos);
  pulsos = 0;
  portEXIT_CRITICAL(&muxPulsos);

  ledcAttach(PIN_BOMBA, FREC_PWM, RESOLUCION);
  ledcWrite(PIN_BOMBA, pwm);

  unsigned long tInicio = millis();
  unsigned long tVentanaFlujo = millis();
  motivo_stop = 0;

  while (true) {
    // Ventana de flujo de 1s
    if (millis() - tVentanaFlujo >= 1000) {
      portENTER_CRITICAL(&muxPulsos);
      unsigned long copia = pulsos;
      pulsos = 0;
      portEXIT_CRITICAL(&muxPulsos);

      unsigned long ahora = millis();
      float tpasado = (ahora - tVentanaFlujo) / 1000.0;
      tVentanaFlujo = ahora;

      float frecuencia = copia / tpasado;
      float caudal = frecuencia / FACTOR_CONVERSION;
      volumen_regado += (caudal / 60.0) * tpasado;

      Serial.printf("[RIEGO] Caudal %.2f L/min | Acumulado %.3f / %.2f L\n",
                    caudal, volumen_regado, litrosObjetivo);
    }

    // Corte por volumen alcanzado
    if (volumen_regado >= litrosObjetivo) { motivo_stop = 1; break; }
    // Corte por tiempo límite de seguridad
    if (millis() - tInicio >= TIEMPO_LIMITE_MS) { motivo_stop = 2; break; }

    delay(20);
  }

  ledcWrite(PIN_BOMBA, 0);
  ledcDetach(PIN_BOMBA);
  apagarFlujo();

  duracion_riego    = (millis() - tInicio) / 1000;
  litros_entregados = (uint16_t)(volumen_regado * 100);
  rego_ejecutado    = true;

  Serial.printf("[RIEGO] FIN - %.3f L en %u s (motivo %d)\n",
                volumen_regado, duracion_riego,
                motivo_stop == 1 ? 1 : 2);
}

// ── ARMAR PAQUETE ────────────────────────────────────────────
PaqueteRiego armarPaquete() {
  PaqueteRiego p;
  p.id_nodo           = ID_NODO_NUM;
  p.distancia         = nivel_ok ? (uint16_t)(distancia_cm * 10) : INV_U16;
  p.porcentaje        = nivel_ok ? (uint8_t)(porcentaje_tanque)  : INV_U8;
  p.litros_tanque     = nivel_ok ? (uint16_t)(litros_tanque * 100) : INV_U16;
  p.rego              = rego_ejecutado ? 1 : 0;
  p.litros_entregados = litros_entregados;
  p.duracion_riego    = duracion_riego;
  p.motivo_stop       = motivo_stop;
  p.voltaje           = (uint16_t)(voltaje * 100);
  p.msg_id            = contador_msg;
  return p;
}

// ── IMPRIMIR ─────────────────────────────────────────────────
void imprimirSerial() {
  Serial.printf("\n┌─ CICLO #%lu   msg_id=%d   intervalo=%lus\n",
                num_ciclo, contador_msg, intervalo_actual);
  if (nivel_ok)
    Serial.printf("│ TANQUE : Dist=%.1f cm  Agua=%.1f cm  %.0f%%  %.2f L\n",
                  distancia_cm, agua_cm, porcentaje_tanque, litros_tanque);
  else
    Serial.println("│ TANQUE : [SIN LECTURA]");
  Serial.printf("│ Batería: %.2f V\n", voltaje);
  Serial.println("└──────────────────────────");
}

// ── TRANSMITIR ───────────────────────────────────────────────
void transmitir(PaqueteRiego p) {
  Serial.printf("[LoRa] TX (%d bytes)\n", (int)sizeof(p));
  int estado = radio.transmit((uint8_t*)&p, sizeof(p));
  if (estado == RADIOLIB_ERR_NONE) Serial.println("[LoRa] TX OK");
  else Serial.printf("[LoRa] TX FALLO codigo %d\n", estado);
}

// ── VENTANA DE ESCUCHA (devuelve true si hay riego pendiente) ─
bool ventanaDeEscucha(float &litrosOut, uint8_t &fuerzaOut) {
  Serial.println("[VENTANA] Escuchando posible comando...");
  bool hayRiego = false;
  comandoRecibido = false;
  radio.setDio1Action(setFlagNodo);
  radio.startReceive();

  unsigned long t0 = millis();
  while (millis() - t0 < VENTANA_MS) {
    if (comandoRecibido) {
      comandoRecibido = false;
      uint8_t buf[16];
      size_t len = radio.getPacketLength();
      int est = radio.readData(buf, len);
      if (est == RADIOLIB_ERR_NONE && len == sizeof(ComandoDescarga)) {
        ComandoDescarga c;
        memcpy(&c, buf, sizeof(c));
        if (c.id_nodo == ID_NODO_NUM) {
          if (c.intervalo_seg > 0) {
            intervalo_actual = c.intervalo_seg;
            Serial.printf("[VENTANA] Nuevo intervalo: %u s\n", c.intervalo_seg);
          }
          if (c.litros_objetivo > 0) {
            litrosOut = c.litros_objetivo / 100.0;
            fuerzaOut = c.nivel_fuerza;
            hayRiego  = true;
            Serial.printf("[VENTANA] Comando de riego: %.2f L, fuerza %d\n",
                          litrosOut, fuerzaOut);
          }
        }
      }
      break;
    }
    delay(10);
  }
  radio.standby();
  Serial.println("[VENTANA] Cerrada.");
  return hayRiego;
}

// ── PREPARAR PINES PARA DORMIR ──────────────────────────────
void prepararPinesParaDormir() {
  pinMode(PIN_PWR_NIVEL, INPUT);
  pinMode(PIN_PWR_FLUJO, INPUT);
  pinMode(PIN_TRIG, INPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_FLOW, INPUT);
  pinMode(PIN_BOMBA, INPUT);
  pinMode(VEXT_CTRL, OUTPUT);
  digitalWrite(VEXT_CTRL, HIGH);
  // NO se tocan: LoRa (8-14,2,46) ni batería (1,37)
}

// ── SETUP = UN CICLO COMPLETO ───────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(300);

  num_ciclo++;
  contador_msg++;

  pinMode(VEXT_CTRL, OUTPUT);
  digitalWrite(VEXT_CTRL, HIGH);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  // Bomba apagada desde el arranque (seguridad)
  pinMode(PIN_BOMBA, OUTPUT);
  digitalWrite(PIN_BOMBA, LOW);

  Serial.println("\n========================================");
  Serial.printf("   NODO N02 — DESPERTAR #%lu\n", num_ciclo);
  Serial.println("========================================");

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

  // 1) Medir nivel de tanque
  encenderNivel();
  leerNivelTanque();
  apagarNivel();
  leerBateria();
  imprimirSerial();

  // 2) Transmitir estado inicial
  PaqueteRiego pkt = armarPaquete();
  if (estadoLora == RADIOLIB_ERR_NONE) {
    transmitir(pkt);

    // 3) Ventana de escucha por comando
    float litrosCmd = 0; uint8_t fuerzaCmd = 2;
    bool hayRiego = ventanaDeEscucha(litrosCmd, fuerzaCmd);

    // 4) Si hay comando de riego, ejecutarlo con seguridad
    if (hayRiego) {
      ejecutarRiego(litrosCmd, fuerzaCmd);
      // 5) Transmitir resultado del riego
      PaqueteRiego pkt2 = armarPaquete();
      transmitir(pkt2);
    }

    radio.sleep();
  }

  // 6) Pines a bajo consumo
  prepararPinesParaDormir();

  // 7) Dormir
  Serial.printf("[SLEEP] Durmiendo %lu s...\n\n", intervalo_actual);
  Serial.flush();
  esp_sleep_enable_timer_wakeup(intervalo_actual * uS_POR_SEG);
  esp_deep_sleep_start();
}

void loop() {
  // vacío: todo ocurre en setup()
}