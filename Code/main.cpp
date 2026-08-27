#include "HardwareSerial.h"
#include <WiFi.h>
#include <PubSubClient.h>

HardwareSerial BU03Serial(1);

#define BU03_RX   4
#define BU03_TX   5

uint8_t buf[64];
int     bufIdx  = 0;
bool    inFrame = false;

float liveDist = 0; // jarak terakhir (mm), hasil Median->Kalman->Round50

// ── GANTI SESUAI JARINGAN & ANCHOR INI ──
#define ANCHOR_ID "A3"   // ganti "A2" / "A3" di board lainnya — harus cocok dengan CONFIG.ANCHORS di server.js
const char* WIFI_SSID   = "Sher";
const char* WIFI_PASS   = "Rahasia123";
const char* MQTT_BROKER = "broker.hivemq.com";   // broker publik HiveMQ
const int   MQTT_PORT   = 1883;

const unsigned long KIRIM_INTERVAL_MS = 150; // seberapa sering kirim jarak ke MQTT

String       mqttTopic = String("uwb/anchor/") + ANCHOR_ID;
WiFiClient   espClient;
PubSubClient mqtt(espClient);

// ── Median Filter ──
struct MedianFilter {
  float buf[5];
  int   count;
  MedianFilter() : count(0) { memset(buf, 0, sizeof(buf)); }

  float update(float val) {
    for (int i = 4; i > 0; i--) buf[i] = buf[i-1];
    buf[0] = val;
    if (count < 5) count++;
    float sorted[5];
    memcpy(sorted, buf, count * sizeof(float));
    for (int i = 0; i < count-1; i++)
      for (int j = i+1; j < count; j++)
        if (sorted[j] < sorted[i]) {
          float t = sorted[i];
          sorted[i] = sorted[j];
          sorted[j] = t;
        }
    return sorted[count/2];
  }
} mf;

// ── Kalman Filter ──
struct KalmanFilter {
  float Q, R, P, K, x;
  bool  initialized;
  KalmanFilter(float q, float r) :
    Q(q), R(r), P(1.0), K(0.0), x(0.0), initialized(false) {}

  float update(float meas) {
    if (!initialized) {
      x = meas;
      initialized = true;
      return x;
    }
    P = P + Q;
    K = P / (P + R);
    x = x + K * (meas - x);
    P = (1 - K) * P;
    return x;
  }
} kf(0.018, 0.642);

float roundToNearest50(float val) {
  return round(val / 50.0) * 50.0;
}

void processFrame() {
  uint16_t raw = buf[7] | (buf[8] << 8);
  if (raw <= 0 || raw > 8000) return;

  float median = mf.update((float)raw);
  float kalman = kf.update(median);
  liveDist = roundToNearest50(kalman);
}

void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  Serial.print("Menyambung ke WiFi \"");
  Serial.print(WIFI_SSID);
  Serial.print("\"");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("WiFi tersambung, IP board: ");
  Serial.println(WiFi.localIP());
}

void connectMQTT() {
  while (!mqtt.connected()) {
    Serial.print("Menyambung ke MQTT broker ");
    Serial.print(MQTT_BROKER);
    Serial.print("...");
    String clientId = String("anchor-") + ANCHOR_ID;
    if (mqtt.connect(clientId.c_str())) {
      Serial.println(" terhubung");
    } else {
      Serial.print(" gagal (rc=");
      Serial.print(mqtt.state());
      Serial.println("), coba lagi 2 detik");
      delay(2000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  BU03Serial.begin(115200, SERIAL_8N1, BU03_RX, BU03_TX);
  delay(500);

  Serial.println("════════════════════════════════════");
  Serial.print  ("  UWB ANCHOR — "); Serial.println(ANCHOR_ID);
  Serial.println("  Pipeline: Raw→Median→Kalman→R50 → MQTT");
  Serial.println("════════════════════════════════════");

  connectWiFi();
  mqtt.setServer(MQTT_BROKER, MQTT_PORT);
}

unsigned long lastKirim = 0;

unsigned long bu03ByteCount      = 0;
unsigned long lastByteCountPrint = 0;
const unsigned long BYTE_COUNT_INTERVAL_MS = 3000;

void loop() {
  while (BU03Serial.available()) {
    uint8_t b = BU03Serial.read();
    bu03ByteCount++;

    if (!inFrame) {
      if (b == 0xAA) {
        bufIdx = 0;
        memset(buf, 0, 64);
        buf[bufIdx++] = b;
        inFrame = true;
      }
    } else {
      buf[bufIdx++] = b;

      if (bufIdx == 37) {
        if (buf[0] == 0xAA && buf[36] == 0x55) {
          processFrame();
          Serial.print("Frame valid! dist=");
          Serial.println(liveDist, 1);
        }
        inFrame = false;
        bufIdx  = 0;
      }

      if (bufIdx >= 64) {
        inFrame = false;
        bufIdx  = 0;
      }
    }
  }

  if (WiFi.status() != WL_CONNECTED) connectWiFi();
  if (!mqtt.connected()) connectMQTT();
  mqtt.loop();

  unsigned long now = millis();
  if (now - lastKirim >= KIRIM_INTERVAL_MS) {
    lastKirim = now;

    if (liveDist > 0) {
      String payload = "{\"dist_mm\":" + String(liveDist, 1) + "}";
      bool ok = mqtt.publish(mqttTopic.c_str(), payload.c_str());
      Serial.println(mqttTopic + " -> " + payload + (ok ? "" : "  (publish gagal)"));
      if (ok) {
        Serial.print("Published: dist_mm=");
        Serial.println(liveDist, 1);
      } else {
        Serial.println("Publish GAGAL");
      }
    }
  }

  if (now - lastByteCountPrint >= BYTE_COUNT_INTERVAL_MS) {
    lastByteCountPrint = now;
    Serial.print("BU03Serial bytes diterima (3s terakhir): ");
    Serial.println(bu03ByteCount);
    bu03ByteCount = 0;
  }
}
