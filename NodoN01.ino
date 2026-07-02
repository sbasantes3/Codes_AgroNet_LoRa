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