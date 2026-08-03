/*
 * ESP32 SLAVE ECG + ESP-NOW
 * Board : ESP32 (Arduino Core v3.x)
 * IC Amp : AD620 (Instrumentasi)
 * Filter : HPF DC-removal + LPF + Normalisasi
 *
 * PERUBAHAN v9.3 (FIX LEAD 3):
 * ─────────────────────────────────────────────────────────────
 * ROOT CAUSE:
 *   AD620 pada dua channel berbeda menghasilkan DC offset berbeda
 *   karena perbedaan impedansi input, gain resistor, dan referensi
 *   tegangan masing-masing OpAmp. Baseline tracker orde-1 (0.001/0.999)
 *   TERLALU LAMBAT mengejar offset DC besar (~1381mV gap L1 vs L2).
 *   Akibatnya sig3 = sig2 - sig1 membawa residu DC besar + distorsi.
 *
 * FIX YANG DITERAPKAN:
 *   1. HPF eksplisit (cutoff ~0.5Hz) menggantikan baseline tracker lambat
 *      → buang DC offset sepenuhnya sebelum hitung sig3
 *   2. LPF alpha diturunkan 0.65 → 0.25 (lebih halus, reject noise AD620)
 *   3. Auto-recalibrate baseline saat startup (200 sampel rata-rata)
 *   4. Gain per-lead independen untuk kompensasi beda amplitudo AD620
 *   5. Lead 3 invert flag — jika masih terbalik tinggal set LEAD3_INVERT=true
 *
 * FIX COMPILE (Arduino Core v3.x):
 *   1. esp_now_register_send_cb → signature baru pakai wifi_tx_info_t*
 *   2. esp_now_register_recv_cb → signature baru pakai esp_now_recv_info_t*
 *   3. timerBegin(freq)         → API baru, prescaler dihilangkan
 *   4. timerAttachInterrupt     → tanpa parameter edge (true dihapus)
 *   5. timerAlarmWrite/Enable   → diganti timerAlarm(timer, ticks, reload, ...)
 * ─────────────────────────────────────────────────────────────
 */

#include <WiFi.h>
#include <esp_now.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

// ─── ALAMAT MAC PENERIMA (MASTER) ──────────────────────────────────────────
uint8_t receiverAddress[] = {0x28, 0x37, 0x2f, 0x86, 0xb1, 0xa8};

// ─── STRUKTUR DATA ─────────────────────────────────────────────────────────
typedef struct struct_message {
  int SYSx, DIAx, Tempx, SpO2x, HRx, RRx;
  int ecgLead1, ecgLead2, ecgLead3;
  int spo2Wave, respWave;
  int leadOff;
  int tekananRT;
  int faseNIBP;
  int cntMenit;
  int battPct;
  int korotkoffBeat;
} struct_message;

typedef struct lead_command {
  int activeLead;
} lead_command;

struct_message dataKirim;
volatile int activeLead = 0;

// ─── FIX #1: Callback send — signature Core v3.x ──────────────────────────
// Core v2: void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
// Core v3: void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status)
void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  // kosong — tidak ada yang perlu dilakukan
}

// ─── FIX #2: Callback recv — signature Core v3.x ──────────────────────────
// Core v2: void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len)
// Core v3: void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len)
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  if (len == sizeof(lead_command)) {
    lead_command cmd;
    memcpy(&cmd, incomingData, sizeof(cmd));
    if (cmd.activeLead >= 0 && cmd.activeLead <= 2) {
      activeLead = cmd.activeLead;
      Serial.println("📡 Lead aktif: Lead " + String(activeLead + 1));
    }
  }
}

// ─── PIN ADC ───────────────────────────────────────────────────────────────
#define LEAD_I_PIN     4
#define LEAD_II_PIN    5
#define ADC_OVERSAMPLE 4

// ─── PARAMETER SINYAL ──────────────────────────────────────────────────────
float lpf_alpha = 0.65f;
float hpf_alpha = 0.990f;

float gainL1 = 0.45f;
float gainL2 = 0.45f;

#define LEAD3_INVERT  false

float vpos1 = -1400.0f;
float vpos2 = -1400.0f;
float vpos3 = -1400.0f;

int hStretch = 3;

#define ADC_VREF_MV   3100.0f
#define ADC_MAX_VAL   4095.0f

inline float rawToMV(float raw) {
  return (raw / ADC_MAX_VAL) * ADC_VREF_MV;
}

// ─── VARIABEL FILTER ───────────────────────────────────────────────────────
float lpf_l1 = 2048.0f, lpf_l2 = 2048.0f;
float hpf_l1  = 0.0f,   hpf_l2  = 0.0f;
float prev_lpf_l1 = 2048.0f, prev_lpf_l2 = 2048.0f;
float dcRef1 = 2048.0f, dcRef2 = 2048.0f;

static int prev_l1 = 128, prev_l2 = 128, prev_l3 = 128;

// ─── LEAD-OFF DETECTION ────────────────────────────────────────────────────
#define LEADOFF_WINDOW_MS    500
#define LEADOFF_SAMPLE_RATE  250
#define LEADOFF_WINDOW_SIZE  (LEADOFF_SAMPLE_RATE * LEADOFF_WINDOW_MS / 1000)

#define FLAT_VAR_THRESHOLD_L1  5.0f
#define FLAT_VAR_THRESHOLD_L2  5.0f
#define FLAT_VAR_THRESHOLD_L3  1.5f

#define LEADOFF_LA  (1 << 0)
#define LEADOFF_RA  (1 << 1)
#define LEADOFF_LL  (1 << 2)
#define LEADOFF_RL  (1 << 3)

struct WelfordStats {
  unsigned long n;
  float mean, M2;
  void reset()  { n = 0; mean = 0.0f; M2 = 0.0f; }
  void update(float x) {
    n++;
    float delta = x - mean;
    mean       += delta / (float)n;
    M2         += delta * (x - mean);
  }
  float variance() { return (n > 1) ? (M2 / (float)(n - 1)) : 0.0f; }
};

WelfordStats stats1, stats2, stats3;
unsigned long leadOffSampleCount = 0;
volatile int  currentLeadOff     = 0;
volatile float lastVar1 = 0.0f, lastVar2 = 0.0f, lastVar3 = 0.0f;

float dcResidual1 = 0.0f, dcResidual2 = 0.0f;

void evaluateLeadOff(float var1, float var2, float var3) {
  bool flat1 = (var1 < FLAT_VAR_THRESHOLD_L1);
  bool flat2 = (var2 < FLAT_VAR_THRESHOLD_L2);
  bool flat3 = (var3 < FLAT_VAR_THRESHOLD_L3);

  int flag = 0;
  if (flat1 && flat2 && flat3) {
    flag = LEADOFF_LA | LEADOFF_RA | LEADOFF_LL | LEADOFF_RL;
  } else {
    if (flat1) flag |= LEADOFF_LA;
    if (flat2) flag |= LEADOFF_LL;
    if (flat3) flag |= LEADOFF_RA;
  }
  currentLeadOff = flag;
}

// ─── TIMER & ISR ───────────────────────────────────────────────────────────
hw_timer_t*  ecgTimer  = NULL;
portMUX_TYPE timerMux  = portMUX_INITIALIZER_UNLOCKED;

volatile bool          sampleReady = false;
volatile int           isr_raw1    = 0;
volatile int           isr_raw2    = 0;
volatile unsigned long isrCount    = 0;

void IRAM_ATTR onEcgTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  isrCount++;
  long s1 = 0, s2 = 0;
  for (int i = 0; i < ADC_OVERSAMPLE; i++) {
    s1 += analogRead(LEAD_I_PIN);
    s2 += analogRead(LEAD_II_PIN);
  }
  isr_raw1    = (int)(s1 / ADC_OVERSAMPLE);
  isr_raw2    = (int)(s2 / ADC_OVERSAMPLE);
  sampleReady = true;
  portEXIT_CRITICAL_ISR(&timerMux);
}

// ─── ANTRIAN & TASK PENGIRIMAN ─────────────────────────────────────────────
typedef struct { int l1, l2, l3; } EcgPacket;
QueueHandle_t ecgQueue;

#define QUEUE_SIZE      64
#define SEND_DECIMATION  5

void taskSend(void* param) {
  EcgPacket pkt;
  int decimCounter = 0;
  for (;;) {
    if (xQueueReceive(ecgQueue, &pkt, portMAX_DELAY) == pdTRUE) {
      decimCounter++;
      int decimThreshold = max(1, SEND_DECIMATION / hStretch);
      if (decimCounter < decimThreshold) continue;
      decimCounter = 0;

      dataKirim.ecgLead1 = 0;
      dataKirim.ecgLead2 = 0;
      dataKirim.ecgLead3 = 0;

      switch(activeLead) {
        case 0: dataKirim.ecgLead1 = pkt.l1; break;
        case 1: dataKirim.ecgLead2 = pkt.l2; break;
        case 2: dataKirim.ecgLead3 = pkt.l3; break;
      }

      dataKirim.leadOff = currentLeadOff;
      esp_now_send(receiverAddress, (uint8_t*)&dataKirim, sizeof(dataKirim));
    }
  }
}

// ─── FUNGSI BANTU ──────────────────────────────────────────────────────────
float oversampleADC(int pin) {
  long sum = 0;
  for (int i = 0; i < ADC_OVERSAMPLE; i++) sum += analogRead(pin);
  return (float)sum / ADC_OVERSAMPLE;
}

int mapECG(float f) {
  return constrain((int)map((long)f, 0, 4095, 0, 255), 0, 255);
}

void calibrateBaseline() {
  float sum1 = 0, sum2 = 0;
  Serial.print("Kalibrasi DC offset AD620...");
  for (int i = 0; i < 200; i++) {
    sum1 += oversampleADC(LEAD_I_PIN);
    sum2 += oversampleADC(LEAD_II_PIN);
    delay(2);
  }
  dcRef1 = sum1 / 200.0f;
  dcRef2 = sum2 / 200.0f;

  lpf_l1 = dcRef1;  lpf_l2 = dcRef2;
  prev_lpf_l1 = dcRef1;  prev_lpf_l2 = dcRef2;
  hpf_l1 = 0.0f;  hpf_l2 = 0.0f;

  Serial.print("Warm-up HPF");
  for (int i = 0; i < 1000; i++) {
    float s1 = oversampleADC(LEAD_I_PIN);
    float s2 = oversampleADC(LEAD_II_PIN);
    float f1 = lpf_alpha * s1 + (1.0f - lpf_alpha) * lpf_l1;
    float f2 = lpf_alpha * s2 + (1.0f - lpf_alpha) * lpf_l2;
    hpf_l1 = hpf_alpha * (hpf_l1 + f1 - prev_lpf_l1);
    hpf_l2 = hpf_alpha * (hpf_l2 + f2 - prev_lpf_l2);
    prev_lpf_l1 = f1;  prev_lpf_l2 = f2;
    lpf_l1 = f1;       lpf_l2 = f2;
    if (i % 200 == 0) Serial.print(".");
    delay(1);
  }
  Serial.println(" OK");
  Serial.printf("HPF residual setelah warmup → L1:%.2f  L2:%.2f (target <5)\n",
                fabsf(hpf_l1), fabsf(hpf_l2));
  Serial.printf("DC Ref → L1:%.1f (%.1fmV)  L2:%.1f (%.1fmV)\n",
                dcRef1, rawToMV(dcRef1), dcRef2, rawToMV(dcRef2));
  Serial.printf("Gap DC L1-L2: %.1f ADC counts (%.1fmV)\n",
                fabsf(dcRef1 - dcRef2), rawToMV(fabsf(dcRef1 - dcRef2)));
}

// ─── SETUP ─────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== ESP32 SLAVE ECG v9.3 — Fix Lead 3 AD620 ===");
  Serial.printf("gainL1=%.2f gainL2=%.2f | hStretch=%d\n", gainL1, gainL2, hStretch);
  Serial.printf("LPF alpha=%.3f | HPF alpha=%.4f\n", lpf_alpha, hpf_alpha);
  Serial.printf("LEAD3_INVERT=%s\n\n", LEAD3_INVERT ? "true" : "false");

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  pinMode(LEAD_I_PIN,  INPUT);
  pinMode(LEAD_II_PIN, INPUT);
  delay(500);

  calibrateBaseline();

  memset(&dataKirim, 0, sizeof(dataKirim));
  stats1.reset(); stats2.reset(); stats3.reset();
  leadOffSampleCount = 0;

  WiFi.mode(WIFI_STA);
  Serial.print("MAC Slave ECG: "); Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW GAGAL!"); while(1) delay(1000);
  }
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  esp_now_peer_info_t peerMaster = {};
  memcpy(peerMaster.peer_addr, receiverAddress, 6);
  peerMaster.channel = 0;
  peerMaster.encrypt = false;
  if (esp_now_add_peer(&peerMaster) != ESP_OK) {
    Serial.println("Tambah peer Master GAGAL!"); while(1) delay(1000);
  }
  Serial.println("ESP-NOW OK — Master terdaftar");

  ecgQueue = xQueueCreate(QUEUE_SIZE, sizeof(EcgPacket));
  xTaskCreatePinnedToCore(taskSend, "taskSend", 4096, NULL, 1, NULL, 0);

  // ─── FIX #3: Timer API Core v3.x ────────────────────────────────────────
  // Core v2: timerBegin(timer_num, prescaler, count_up)
  //          → prescaler 80 pada clock 80MHz → tick = 1µs
  //          → timerAlarmWrite(timer, 4000, true) → ISR setiap 4000µs = 250Hz
  // Core v3: timerBegin(frequency_Hz) — frekuensi timer langsung dalam Hz
  //          → timerBegin(250000) → tick 1/250000 detik = 4µs
  //          → timerAlarm(timer, 1, true) → ISR setiap 1 tick = 4µs… SALAH
  //
  // Cara benar Core v3 untuk 250Hz ISR:
  //   timerBegin(1000000)  → resolusi 1µs  (sama seperti prescaler 80 dulu)
  //   timerAttachInterrupt tanpa edge flag
  //   timerAlarm(timer, 4000, true, 0) → tembak ISR tiap 4000 tick = 4000µs = 250Hz
  ecgTimer = timerBegin(1000000);               // resolusi 1µs
  timerAttachInterrupt(ecgTimer, &onEcgTimer);  // tanpa edge flag
  timerAlarm(ecgTimer, 4000, true, 0);          // 4000µs = 250Hz, auto-reload

  Serial.println("Sistem berjalan — sampling 250Hz");
  Serial.println("Tunggu ~2 detik untuk HPF stabil...\n");
}

// ─── LOOP ──────────────────────────────────────────────────────────────────
static unsigned long lastDebugMs = 0;

void loop() {
  if (!sampleReady) return;

  portENTER_CRITICAL(&timerMux);
  float raw1 = isr_raw1;
  float raw2 = isr_raw2;
  sampleReady = false;
  portEXIT_CRITICAL(&timerMux);

  // ─── STEP 1: LPF (noise rejection) ────────────────────────────────────
  float cur_lpf_l1 = lpf_alpha * raw1 + (1.0f - lpf_alpha) * lpf_l1;
  float cur_lpf_l2 = lpf_alpha * raw2 + (1.0f - lpf_alpha) * lpf_l2;

  // ─── STEP 2: HPF eksplisit — buang DC offset AD620 ────────────────────
  hpf_l1 = hpf_alpha * (hpf_l1 + cur_lpf_l1 - prev_lpf_l1);
  hpf_l2 = hpf_alpha * (hpf_l2 + cur_lpf_l2 - prev_lpf_l2);

  prev_lpf_l1 = cur_lpf_l1;
  prev_lpf_l2 = cur_lpf_l2;
  lpf_l1 = cur_lpf_l1;
  lpf_l2 = cur_lpf_l2;

  // ─── STEP 3: Gain + hitung sig3 (Einthoven) ───────────────────────────
  float sig1 = hpf_l1 * gainL1;
  float sig2 = hpf_l2 * gainL2;
  float sig3 = sig2 - sig1;

  if (LEAD3_INVERT) sig3 = -sig3;

  // ─── STEP 4: Mapping ke range display [0–4095] ────────────────────────
  float out1 = constrain(sig1 + 2048.0f + vpos1, 0, 4095);
  float out2 = constrain(sig2 + 2048.0f + vpos2, 0, 4095);
  float out3 = constrain(sig3 + 2048.0f + vpos3, 0, 4095);

  int cur_l1 = mapECG(out1);
  int cur_l2 = mapECG(out2);
  int cur_l3 = mapECG(out3);

  // ─── LEAD-OFF DETECTION ────────────────────────────────────────────────
  stats1.update((float)cur_l1);
  stats2.update((float)cur_l2);
  stats3.update((float)cur_l3);
  leadOffSampleCount++;

  dcResidual1 = 0.001f * hpf_l1 + 0.999f * dcResidual1;
  dcResidual2 = 0.001f * hpf_l2 + 0.999f * dcResidual2;

  if (leadOffSampleCount >= LEADOFF_WINDOW_SIZE) {
    lastVar1 = stats1.variance();
    lastVar2 = stats2.variance();
    lastVar3 = stats3.variance();
    evaluateLeadOff(lastVar1, lastVar2, lastVar3);
    stats1.reset(); stats2.reset(); stats3.reset();
    leadOffSampleCount = 0;
  }

  // ─── DEBUG SERIAL ──────────────────────────────────────────────────────
  unsigned long now = millis();
  if (now - lastDebugMs >= 1000) {
    lastDebugMs = now;

    portENTER_CRITICAL(&timerMux);
    unsigned long isr_snap = isrCount;
    isrCount = 0;
    portEXIT_CRITICAL(&timerMux);

    Serial.printf("ISR/s:%lu | RAW L1:%d L2:%d | activeLead:L%d\n",
                  isr_snap, (int)raw1, (int)raw2, activeLead + 1);
    Serial.printf("HPF  L1:%.2f  L2:%.2f\n", hpf_l1, hpf_l2);
    Serial.printf("SIG  L1:%.2f  L2:%.2f  L3(=L2-L1):%.2f%s\n",
                  sig1, sig2, sig3, LEAD3_INVERT ? " [INV]" : "");
    Serial.printf("OUT  L1:%.0f  L2:%.0f  L3:%.0f\n", out1, out2, out3);
    Serial.printf("MAP  L1:%d  L2:%d  L3:%d\n", cur_l1, cur_l2, cur_l3);
    Serial.printf("VAR  L1:%.2f  L2:%.2f  L3:%.2f\n", lastVar1, lastVar2, lastVar3);
    Serial.printf("HPF DC residual: L1=%.2f  L2=%.2f  (idealnya < 5)\n",
                  fabsf(dcResidual1), fabsf(dcResidual2));

    if (currentLeadOff == 0) {
      Serial.println("LEAD-OFF: ✅ Semua kabel terpasang");
    } else {
      Serial.print("LEAD-OFF: ⚠️  Lepas →");
      if (currentLeadOff & LEADOFF_LA) Serial.print(" LA");
      if (currentLeadOff & LEADOFF_RA) Serial.print(" RA");
      if (currentLeadOff & LEADOFF_LL) Serial.print(" LL");
      if (currentLeadOff & LEADOFF_RL) Serial.print(" RL");
      Serial.printf("  (flag=0x%02X)\n", currentLeadOff);
    }
    Serial.println("─────────────────────────────────────────");
  }

  // ─── KIRIM KE ANTRIAN ─────────────────────────────────────────────────
  EcgPacket pkt;
  if (hStretch <= 1) {
    pkt.l1 = cur_l1; pkt.l2 = cur_l2; pkt.l3 = cur_l3;
    xQueueSend(ecgQueue, &pkt, 0);
  } else {
    for (int i = 0; i < hStretch; i++) {
      float t  = (float)i / hStretch;
      pkt.l1 = (int)(prev_l1 + (cur_l1 - prev_l1) * t + 0.5f);
      pkt.l2 = (int)(prev_l2 + (cur_l2 - prev_l2) * t + 0.5f);
      pkt.l3 = (int)(prev_l3 + (cur_l3 - prev_l3) * t + 0.5f);
      xQueueSend(ecgQueue, &pkt, 0);
    }
  }

  prev_l1 = cur_l1;
  prev_l2 = cur_l2;
  prev_l3 = cur_l3;
}