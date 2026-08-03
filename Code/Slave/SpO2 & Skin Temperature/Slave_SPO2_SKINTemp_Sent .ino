#include <WiFi.h>
#include <esp_now.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// =============================================
// ESP-NOW CONFIG
// =============================================
uint8_t macMaster[] = {0x28, 0x37, 0x2f, 0x86, 0xb1, 0xa8};

typedef struct struct_message {
  int SYSx;
  int DIAx;
  int Tempx;
  int SpO2x;
  int HRx;
  int RRx;
  int ecgLead1;
  int ecgLead2;
  int ecgLead3;
  int spo2Wave;
  int respWave;
  int leadOff; 
  int tekananRT;
  int faseNIBP;
  int cntMenit;
  int battPct;   
  int korotkoffBeat;   
} struct_message;

struct_message dataSend;

// =============================================
// FIX: Signature callback diperbarui untuk ESP32 Core 3.x
// Sebelumnya: void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
// Core 3.x menggunakan wifi_tx_info_t sebagai parameter pertama
// =============================================
void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {}

// =============================================
// DS18B20 CONFIG
// =============================================
#define DS18B20_PIN     4
#define TEMP_INTERVAL   2000
#define TEMP_HANDLE_MS  100
#define TEMP_CONV_MS    800
#define TEMP_SCALE      10
#define TEMP_OFFSET     0.0f
#define TEMP_MIN        25.0f
#define TEMP_MAX        42.0f

OneWire           oneWire(DS18B20_PIN);
DallasTemperature tempSensor(&oneWire);

static float      lastTemp       = 0.0f;
static bool       tempValid      = false;
static uint32_t   lastTempMs     = 0;
static uint32_t   requestTempMs  = 0;
static bool       tempRequested  = false;

// =============================================
// BATTERY CONFIG
// =============================================
#define BAT_PIN         34
#define BAT_ADC_RES     4095       // 12-bit ADC
#define BAT_VREF        3.3f       // Referensi ADC ESP32 (V)
#define BAT_V0          2.36f      //   0%
#define BAT_V25         2.59f      //  25%
#define BAT_V50         2.83f      //  50%
#define BAT_V75         3.07f      //  75%
#define BAT_V100        3.30f      // 100%
#define BAT_INTERVAL    5000       // Baca baterai tiap 5 detik
#define BAT_AVG_SAMPLES 8          // Rata-rata 8 sample untuk stabilitas

static uint32_t lastBatMs  = 0;
static int      lastBatPct = -1;   // -1 = belum pernah dibaca

// Kurva lookup: {voltase, persentase}
static const float batCurve[5][2] = {
  {BAT_V0,   0.0f},
  {BAT_V25,  25.0f},
  {BAT_V50,  50.0f},
  {BAT_V75,  75.0f},
  {BAT_V100, 100.0f}
};

// Interpolasi linier antar titik kurva → persentase kontinu
float voltToPct(float v) {
  if (v <= BAT_V0)   return 0.0f;
  if (v >= BAT_V100) return 100.0f;
  for (int i = 0; i < 4; i++) {
    float v0 = batCurve[i][0],   p0 = batCurve[i][1];
    float v1 = batCurve[i+1][0], p1 = batCurve[i+1][1];
    if (v >= v0 && v <= v1) {
      return p0 + (v - v0) / (v1 - v0) * (p1 - p0);
    }
  }
  return 0.0f;
}

// Bulatkan ke kelipatan 25 terdekat
int snapTo25(float pct) {
  int rounded = (int)(pct / 25.0f + 0.5f) * 25;
  return constrain(rounded, 0, 100);
}

void handleBattery() {
  uint32_t now = millis();
  if (now - lastBatMs < BAT_INTERVAL) return;
  lastBatMs = now;

  // Rata-rata multi-sample untuk meredam noise ADC
  long sum = 0;
  for (int i = 0; i < BAT_AVG_SAMPLES; i++) {
    sum += analogRead(BAT_PIN);
    delayMicroseconds(200);
  }
  float adcAvg = (float)sum / BAT_AVG_SAMPLES;

  float voltage   = adcAvg / BAT_ADC_RES * BAT_VREF;
  float pctFloat  = voltToPct(voltage);
  int   pctSnap   = snapTo25(pctFloat);   // 0 / 25 / 50 / 75 / 100

  dataSend.battPct = pctSnap;

  // Tampilkan log hanya bila nilai berubah
  if (pctSnap != lastBatPct) {
    Serial.printf("[BAT] ADC=%.0f  V=%.3fV  Pct=%.1f%%  → %d%%\n",
                  adcAvg, voltage, pctFloat, pctSnap);
    lastBatPct = pctSnap;
  }
}

// =============================================
// WAVEFORM PARAMETER
// =============================================
float vposSpo2     = -800.0f;
int   hStretchSpo2 = -50;

static int  prev_wave    = 128;
static int  decimCounter = 0;
#define SEND_DECIM 1

// =============================================
// SMOOTHING CONFIG
// =============================================
#define MA_SIZE   5
static int   maBuffer[MA_SIZE] = {0};
static int   maIndex           = 0;
static long  maSum             = 0;
static bool  maFilled          = false;

#define IIR_ALPHA 0.60f
static float iirOut = 128.0f;

#define HP_BETA   0.60f
static float hpPrev    = 0.0f;
static float hpPrevRaw = 0.0f;

#define AC_SCALE 200.0f

// =============================================
// UART SLAVE CONFIG
// =============================================
#define SLAVE_RXD 16
#define SLAVE_TXD 17

// =============================================
// DATA TERIMA
// =============================================
int   spo2Val    = 0;
int   bpmVal     = 0;
float ratioVal   = 0;

int   ppgRedLast = 0;
bool  fingerOn   = false;
bool  masterReady = false;

// =============================================
// BUFFER STRING UART
// =============================================
String rxBuffer = "";

// =============================================
// SMOOTHING PIPELINE
// =============================================
int movingAverage(int raw) {
  maSum -= maBuffer[maIndex];
  maBuffer[maIndex] = raw;
  maSum += raw;
  maIndex = (maIndex + 1) % MA_SIZE;
  if (maIndex == 0) maFilled = true;
  int count = maFilled ? MA_SIZE : maIndex;
  return (int)(maSum / count);
}

float iirLowPass(int maVal) {
  iirOut = IIR_ALPHA * (float)maVal + (1.0f - IIR_ALPHA) * iirOut;
  return iirOut;
}

float highPassDC(float lpVal) {
  float ac = lpVal - hpPrev;
  hpPrev   = HP_BETA * hpPrev + (1.0f - HP_BETA) * lpVal;
  return ac;
}

float smoothPPG(int raw) {
  int   ma = movingAverage(raw);
  float lp = iirLowPass(ma);
  float ac = highPassDC(lp);
  return ac;
}

int acToWave(float ac) {
  int val = (int)(48.0f + (ac / AC_SCALE) * 127.0f);
  return constrain(val, 0, 255);
}

// =============================================
// DS18B20: REQUEST ASYNC (non-blocking)
// =============================================
void requestTempAsync() {
  tempSensor.setWaitForConversion(false);
  tempSensor.requestTemperatures();
  tempRequested = true;
  requestTempMs = millis();
}

void readTempAsync() {
  if (!tempRequested) return;

  float raw = tempSensor.getTempCByIndex(0);

  if (raw == DEVICE_DISCONNECTED_C) {
    Serial.println("[WARN] DS18B20 tidak terdeteksi / disconnected");
    tempValid      = false;
    tempRequested  = false;
    dataSend.Tempx = 0;
    return;
  }

  float corrected = raw + TEMP_OFFSET;

  if (corrected < TEMP_MIN || corrected > TEMP_MAX) {
    Serial.printf("[WARN] Suhu di luar range: %.2f°C\n", corrected);
  }

  lastTemp      = corrected;
  tempValid     = true;
  tempRequested = false;

  dataSend.Tempx = (int)(corrected * TEMP_SCALE + 0.5f);

  Serial.printf("[TEMP] Skin Temp = %.2f°C  → Tempx = %d\n",
                corrected, dataSend.Tempx);
}

// =============================================
// TEMPERATURE TASK
// =============================================
void handleTemperature() {
  uint32_t now = millis();

  static bool lastFingerOn = false;
  if (fingerOn != lastFingerOn) {
    if (fingerOn) {
      tempSensor.setResolution(11);
    } else {
      tempSensor.setResolution(12);
    }
    lastFingerOn = fingerOn;
    tempRequested = false;
  }

  uint32_t convMs = fingerOn ? 400 : TEMP_CONV_MS;

  if (!tempRequested && (now - lastTempMs >= TEMP_INTERVAL)) {
    lastTempMs = now;
    requestTempAsync();
  }

  if (tempRequested && (now - requestTempMs >= convMs)) {
    readTempAsync();
  }
}

// =============================================
// ESP-NOW SEND (helper internal)
// =============================================
static void _sendWithInterp(int waveVal) {
  if (hStretchSpo2 <= 1) {
    decimCounter++;
    if (decimCounter >= max(1, SEND_DECIM)) {
      decimCounter      = 0;
      dataSend.spo2Wave = waveVal;
      esp_now_send(macMaster, (uint8_t*)&dataSend, sizeof(dataSend));
    }
    prev_wave = waveVal;
  } else {
    int decimThresh = max(1, SEND_DECIM / hStretchSpo2);
    for (int i = 0; i < hStretchSpo2; i++) {
      float t      = (float)i / hStretchSpo2;
      int   interp = (int)(prev_wave + (waveVal - prev_wave) * t + 0.5f);
      decimCounter++;
      if (decimCounter >= decimThresh) {
        decimCounter      = 0;
        dataSend.spo2Wave = constrain(interp, 0, 255);
        esp_now_send(macMaster, (uint8_t*)&dataSend, sizeof(dataSend));
      }
    }
    prev_wave = waveVal;
  }
}

void resetFilters() {
  iirOut    = 128.0f;
  hpPrev    = 0.0f;
  hpPrevRaw = 0.0f;
  maSum     = 0;
  maIndex   = 0;
  maFilled  = false;
  memset(maBuffer, 0, sizeof(maBuffer));
}

void sendNoFinger() {
  resetFilters();
  dataSend.SpO2x    = 0;
  dataSend.spo2Wave = 128;
  _sendWithInterp(128);
}

// =============================================
// PARSE DATA DARI MASTER (via UART)
// =============================================
void parseLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  if (line.startsWith("STATUS:")) {
    String status = line.substring(7);
    if (status == "MASTER_READY") {
      masterReady = true;
      Serial.println("[INFO] Master terhubung");
    } else if (status == "NO_FINGER") {
      fingerOn = false;
      sendNoFinger();
      Serial.println("[INFO] Jari dilepas");
    } else if (status == "CALIBRATING_REMOVE_FINGER") {
      Serial.println("[INFO] Kalibrasi: lepas jari");
    }
  }

  else if (line.startsWith("CALIB:")) {
    Serial.print("[CALIB] "); Serial.println(line.substring(6));
  }

  else if (line.startsWith("BASELINE:")) {
    Serial.println("[CALIB] Selesai - letakkan jari");
  }

  else if (line.startsWith("PPG:")) {
    int raw    = line.substring(4).toInt();
    ppgRedLast = raw;
    fingerOn   = true;

    float ac      = smoothPPG(raw);
    int   waveOut = acToWave(ac);

    Serial.print("ppgRaw:"); Serial.print(raw);
    Serial.print("\tppgAC:"); Serial.print(ac, 2);
    Serial.print("\twaveOut:"); Serial.println(waveOut);

    dataSend.SpO2x    = spo2Val;
    dataSend.spo2Wave = waveOut;
    _sendWithInterp(waveOut);

    Serial.print("SpO2:"); Serial.print(spo2Val);
    Serial.print("\tTemp:"); Serial.print(lastTemp, 1);
    Serial.print("°C\tBatt:"); Serial.print(dataSend.battPct);
    Serial.println("%");
  }

  else if (line.startsWith("SPO2DATA:")) {
    String data      = line.substring(9);
    float  fields[7] = {0};
    int    idx = 0, start = 0;

    for (int i = 0; i <= (int)data.length(); i++) {
      if (i == (int)data.length() || data[i] == ',') {
        fields[idx++] = data.substring(start, i).toFloat();
        start = i + 1;
        if (idx >= 7) break;
      }
    }

    spo2Val  = (int)fields[2];
    bpmVal   = (int)fields[3];
    ratioVal = fields[6];

    dataSend.SpO2x = spo2Val;

    Serial.print("[SPO2] SpO2="); Serial.print(spo2Val);
    Serial.print("% BPM=");       Serial.print(bpmVal);
    Serial.print(" R=");           Serial.println(ratioVal, 4);
  }
}

// =============================================
// SETUP
// =============================================
void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, SLAVE_RXD, SLAVE_TXD);
  delay(300);

  // --- ADC Battery Init ---
  analogReadResolution(12);        // pastikan 12-bit
  analogSetAttenuation(ADC_11db);  // rentang 0–3.3V pada pin 34
  Serial.printf("[BAT] Pin=%d  Vmin=%.2fV(0%%)  Vmax=%.2fV(100%%)\n",
                BAT_PIN, BAT_V0, BAT_V100);

  // --- DS18B20 Init ---
  tempSensor.begin();
  uint8_t nDev = tempSensor.getDeviceCount();
  Serial.printf("[DS18B20] Sensor ditemukan: %d\n", nDev);

  if (nDev == 0) {
    Serial.println("[WARN] DS18B20 tidak ditemukan! Cek wiring & resistor pull-up 4.7kΩ");
  } else {
    tempSensor.setResolution(12);
    Serial.println("[DS18B20] Resolusi: 12-bit (0.0625°C)");
  }

  // --- WiFi & ESP-NOW Init ---
  WiFi.mode(WIFI_STA);
  Serial.println("[SLAVE SPO2] MAC: " + WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ERR] ESP-NOW INIT FAILED");
    while (1) delay(1000);
  }
  esp_now_register_send_cb(OnDataSent);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, macMaster, 6);
  peer.channel = 0;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("[ERR] Add peer FAILED");
  } else {
    Serial.println("[OK] Peer Master terdaftar");
  }

  memset(&dataSend, 0, sizeof(dataSend));

  Serial.printf("[CFG] vpos=%.0f hStretch=%d alpha=%.2f beta=%.2f MA=%d AC_SCALE=%.1f\n",
    vposSpo2, hStretchSpo2, IIR_ALPHA, HP_BETA, MA_SIZE, AC_SCALE);
  Serial.printf("[CFG] DS18B20 pin=%d interval=%dms offset=%.1f°C\n",
    DS18B20_PIN, TEMP_INTERVAL, TEMP_OFFSET);
  Serial.println("[READY] Menunggu data dari Master...");
}

// =============================================
// LOOP
// =============================================
void loop() {
  // Baca UART dengan prioritas TERTINGGI
  while (Serial2.available()) {
    char c = (char)Serial2.read();
    if (c == '\n') {
      parseLine(rxBuffer);
      rxBuffer = "";
    } else if (c != '\r') {
      rxBuffer += c;
      if (rxBuffer.length() > 200) rxBuffer = "";
    }
  }

  // Throttle handleTemperature()
  static uint32_t lastTempHandle = 0;
  uint32_t now = millis();
  if (now - lastTempHandle >= TEMP_HANDLE_MS) {
    lastTempHandle = now;
    handleTemperature();
  }

  // Battery monitor (non-blocking)
  handleBattery();
}