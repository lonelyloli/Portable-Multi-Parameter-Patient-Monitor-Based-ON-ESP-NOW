// =============================================
// ESP32-S3 SPO2 MONITOR
// Driver LED : Direct GPIO Register (presisi 1kHz)
//   LED_RED = 39, LED_IR = 38
// ADC       : acRed=7, dcRed=5, acIR=8, dcIR=3
// FIX       : - Tambah soc/gpio_struct.h
//             - Timer API diupdate ke core v3.x
// =============================================

#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "soc/gpio_struct.h"   // FIX #1: expose struct GPIO untuk register access

// --- Pin LED Driver ---
#define LED_RED 38
#define LED_IR  39

#define RED_HIGH() do { GPIO.out1_w1ts.val = (1UL << (LED_RED - 32)); } while(0)
#define RED_LOW()  do { GPIO.out1_w1tc.val = (1UL << (LED_RED - 32)); } while(0)
#define IR_HIGH()  do { GPIO.out1_w1ts.val = (1UL << (LED_IR  - 32)); } while(0)
#define IR_LOW()   do { GPIO.out1_w1tc.val = (1UL << (LED_IR  - 32)); } while(0)

// --- Pin ADC → channel ADC1 ---
#define CH_AC_IR   ADC1_CHANNEL_6   // GPIO7  ← AC IR
#define CH_DC_RED  ADC1_CHANNEL_4   // GPIO5  ← DC Red
#define CH_AC_RED  ADC1_CHANNEL_7   // GPIO8  ← AC Red
#define CH_DC_IR   ADC1_CHANNEL_2   // GPIO3  ← DC IR

// --- UART Slave ---
#define SLAVE_RX 17
#define SLAVE_TX 16

// =============================================
// TIMING
// 8 phase @ 250µs = 1 siklus 2ms → 500Hz efektif
//   Phase 0 : RED ON,  IR OFF   (LED switch)
//   Phase 1 : settle RED
//   Phase 2 : baca ADC Red
//   Phase 3 : RED OFF
//   Phase 4 : IR ON,  RED OFF   (LED switch)
//   Phase 5 : settle IR
//   Phase 6 : baca ADC IR
//   Phase 7 : IR OFF
// =============================================
#define FREQ_HZ   500            // siklus per detik (1 siklus = 8 phase)
#define PHASE_US  250            // durasi tiap phase (µs)  = 1/(500*8)*1e6

const int PRINT_INTERVAL  = 10;    // ms — throttle Serial print saja
const int OUTPUT_INTERVAL = 5000;  // ms — output SpO2/BPM setiap 5 detik

// =============================================
// BASELINE MANUAL
// =============================================
const int BASELINE_ACIR = 2200;
const int BASELINE_ACR  = 2210;

// =============================================
// VARIABEL ADC
// =============================================
volatile uint8_t  adcPhase    = 0;
volatile bool     readPending = false;
int ACIR = 0, DCIR = 0;
int ACR  = 0, DCR  = 0;
volatile bool newSample = false;

// =============================================
// TIMER ISR
// 8 phase — phase ganjil untuk settle,
// ADC dibaca di phase 2 (Red) dan 6 (IR)
// =============================================
hw_timer_t*  sensorTimer = NULL;
portMUX_TYPE timerMux    = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onSensorTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  switch (adcPhase) {
    case 0: RED_HIGH(); IR_LOW();  break;   // RED ON  → mulai settle
    case 1:                        break;   // settle RED (250µs)
    case 2: readPending = true;    break;   // flag: baca ADC Red (LED masih ON)
    case 3: RED_LOW();             break;   // RED OFF
    case 4: IR_HIGH(); RED_LOW();  break;   // IR ON   → mulai settle
    case 5:                        break;   // settle IR (250µs)
    case 6: readPending = true;    break;   // flag: baca ADC IR (LED masih ON)
    case 7: IR_LOW();              break;   // IR OFF
  }
  adcPhase = (adcPhase + 1) % 8;
  portEXIT_CRITICAL_ISR(&timerMux);
}

// =============================================
// readADC() + processing TIDAK di-gate
// dipanggil setiap kali readPending=true
// =============================================
void readADC() {
  uint8_t donePhase = (adcPhase + 7) % 8;
  switch (donePhase) {
    case 2:   // selesai settle RED → baca Red
      ACR  = adc1_get_raw(CH_AC_RED);
      DCR  = adc1_get_raw(CH_DC_RED);
      break;
    case 6:   // selesai settle IR → baca IR, sample lengkap
      ACIR = adc1_get_raw(CH_AC_IR);
      DCIR = adc1_get_raw(CH_DC_IR);
      newSample = true;
      break;
  }
}

// =============================================
// BUFFER PPG
// =============================================
const int BUF_SIZE = 200;
int  bufIR[BUF_SIZE], bufRed[BUF_SIZE];
int  bufIdx = 0;

// =============================================
// DC AVERAGING (window 20 sample)
// =============================================
const int DC_AVG_SIZE = 20;
long  dcIR_sum = 0, dcRed_sum = 0;
int   dcIR_buf[DC_AVG_SIZE], dcRed_buf[DC_AVG_SIZE];
int   dc_idx  = 0;
bool  dcReady = false;
float dcIR_avg = 2300.0, dcRed_avg = 2300.0;

// =============================================
// PEAK-TO-PEAK (window 100 sample, avg 10 window)
// =============================================
const int PEAK_WIN_SIZE = 100;
const int PEAK_AVG_SIZE = 10;
int   winIR[PEAK_WIN_SIZE],      winRed[PEAK_WIN_SIZE];
float peakIR_buf[PEAK_AVG_SIZE], peakRed_buf[PEAK_AVG_SIZE];
int   winIdx = 0, peakAvgIdx = 0;
bool  peakAvgFull = false;
float peakIR_avg  = 0.0, peakRed_avg = 0.0;

// =============================================
// KALKULASI SPO2
// =============================================
const float SPO2_A = 118.0;
const float SPO2_B = 39.16;

const int R_AVG_SIZE    = 5;
const int SPO2_AVG_SIZE = 2;
float rBuf[R_AVG_SIZE]       = {0};
float spo2Buf[SPO2_AVG_SIZE] = {0};
int   rIdx = 0, spo2Idx = 0;
bool  rFull = false, spo2Full = false;
float SPO2   = 0.0;
float R_last = 0.0;

// =============================================
// MOVING AVERAGE PPG (window 5)
// =============================================
const int MA_SIZE = 5;
int maACIR[MA_SIZE] = {0}, maACR[MA_SIZE] = {0};
int maIdx = 0;

// =============================================
// DETEKSI JARI
// =============================================
const int DC_THRESHOLD = 2420;

// =============================================
// DETEKSI BPM
// =============================================
const int BPM_AVG_SIZE = 5;
const int AMP_BUF_SIZE = 40;
unsigned long bpmBuf[BPM_AVG_SIZE] = {0};
int  ampBuf[AMP_BUF_SIZE] = {0};
int  bpmIdx = 0, ampIdx = 0;
int  BPM = 0, peakVal = 0;
bool rising = false;
int  prevPpgIR = 0;
unsigned long lastPeakTime = 0;

// =============================================
// TIMING
// =============================================
unsigned long prevMillis = 0, prevOutputMillis = 0;

// =============================================
// FUNGSI HELPER
// =============================================
int movingAvg(int* buf, int newVal) {
  buf[maIdx % MA_SIZE] = newVal;
  long sum = 0;
  for (int i = 0; i < MA_SIZE; i++) sum += buf[i];
  return (int)(sum / MA_SIZE);
}

void updateDCAvg() {
  dcIR_sum  -= dcIR_buf[dc_idx];
  dcRed_sum -= dcRed_buf[dc_idx];
  dcIR_buf[dc_idx] = DCIR;  dcRed_buf[dc_idx] = DCR;
  dcIR_sum  += DCIR;        dcRed_sum += DCR;
  dc_idx = (dc_idx + 1) % DC_AVG_SIZE;
  if (dc_idx == 0) dcReady = true;
  dcIR_avg  = dcIR_sum  / (float)DC_AVG_SIZE;
  dcRed_avg = dcRed_sum / (float)DC_AVG_SIZE;
}

float calcPeakToPeak(int* buf, int size) {
  int mn = buf[0], mx = buf[0];
  for (int i = 1; i < size; i++) {
    if (buf[i] < mn) mn = buf[i];
    if (buf[i] > mx) mx = buf[i];
  }
  return (float)(mx - mn);
}

void updatePeakWindow(int ppgIR, int ppgRed) {
  winIR[winIdx] = ppgIR;  winRed[winIdx] = ppgRed;
  if (++winIdx >= PEAK_WIN_SIZE) {
    winIdx = 0;
    float pkIR  = calcPeakToPeak(winIR,  PEAK_WIN_SIZE);
    float pkRed = calcPeakToPeak(winRed, PEAK_WIN_SIZE);
    peakIR_buf[peakAvgIdx]  = pkIR;
    peakRed_buf[peakAvgIdx] = pkRed;
    peakAvgIdx = (peakAvgIdx + 1) % PEAK_AVG_SIZE;
    if (peakAvgIdx == 0) peakAvgFull = true;
    int cnt = peakAvgFull ? PEAK_AVG_SIZE : peakAvgIdx;
    float sIR = 0, sRed = 0;
    for (int i = 0; i < cnt; i++) { sIR += peakIR_buf[i]; sRed += peakRed_buf[i]; }
    peakIR_avg  = sIR  / cnt;
    peakRed_avg = sRed / cnt;
  }
}

void flushSpo2Buffers() {
  rIdx = 0; rFull = false; spo2Idx = 0; spo2Full = false;
  for (int i = 0; i < R_AVG_SIZE;    i++) rBuf[i]    = 0;
  for (int i = 0; i < SPO2_AVG_SIZE; i++) spo2Buf[i] = 0;
}

void calcSpO2() {
  if (!dcReady) return;
  if (peakIR_avg < 5.0 || peakRed_avg < 5.0) return;
  if (dcIR_avg < 100.0 || dcRed_avg < 100.0) return;

  float R = (peakRed_avg / dcRed_avg) / (peakIR_avg / dcIR_avg);
  if (R < 0.0 || R > 2.1) return;
  R_last = R;

  rBuf[rIdx] = R;
  rIdx = (rIdx + 1) % R_AVG_SIZE;
  if (rIdx == 0) rFull = true;
  int rCnt = rFull ? R_AVG_SIZE : rIdx;
  float rSum = 0;
  for (int i = 0; i < rCnt; i++) rSum += rBuf[i];
  float rAvg = rSum / rCnt;

  float spo2Raw = constrain(SPO2_A - SPO2_B * rAvg, 70.0f, 101.0f);

  if (SPO2 > 0 && fabs(spo2Raw - SPO2) > 3.0f) flushSpo2Buffers();

  spo2Buf[spo2Idx] = spo2Raw;
  spo2Idx = (spo2Idx + 1) % SPO2_AVG_SIZE;
  if (spo2Idx == 0) spo2Full = true;
  int vCnt = spo2Full ? SPO2_AVG_SIZE : spo2Idx;
  float sSum = 0;
  for (int i = 0; i < vCnt; i++) sSum += spo2Buf[i];

  float newSpo2 = sSum / vCnt;
  SPO2 = (SPO2 == 0.0f) ? newSpo2 : (SPO2 * 0.7f + newSpo2 * 0.3f);
}

int getAdaptiveThreshold() {
  int mn = ampBuf[0], mx = ampBuf[0];
  for (int i = 1; i < AMP_BUF_SIZE; i++) {
    if (ampBuf[i] < mn) mn = ampBuf[i];
    if (ampBuf[i] > mx) mx = ampBuf[i];
  }
  return mn + (mx - mn) * 30 / 100;
}

void detectBPM(int ppgIR) {
  ampBuf[ampIdx % AMP_BUF_SIZE] = ppgIR;
  ampIdx++;
  int  thr      = getAdaptiveThreshold();
  bool isRising = (ppgIR > prevPpgIR);

  if      (!rising && isRising && ppgIR > thr) { rising = true; peakVal = ppgIR; }
  else if ( rising && isRising && ppgIR > peakVal) peakVal = ppgIR;
  else if ( rising && !isRising) {
    if (peakVal > thr) {
      unsigned long now = millis();
      if (lastPeakTime > 0) {
        unsigned long iv = now - lastPeakTime;
        if (iv > 300 && iv < 1500) {
          bpmBuf[bpmIdx] = iv;
          bpmIdx = (bpmIdx + 1) % BPM_AVG_SIZE;
          long s = 0; int c = 0;
          for (int i = 0; i < BPM_AVG_SIZE; i++) if (bpmBuf[i] > 0) { s += bpmBuf[i]; c++; }
          if (c > 0) BPM = 60000 / (s / c);
        }
      }
      lastPeakTime = now;
    }
    rising = false; peakVal = 0;
  }
  prevPpgIR = ppgIR;
}

void resetAll() {
  bufIdx = 0; SPO2 = 0; BPM = 0; R_last = 0;
  peakIR_avg = 0; peakRed_avg = 0;
  peakAvgIdx = 0; peakAvgFull = false;
  winIdx = 0; spo2Idx = 0; spo2Full = false;
  rIdx = 0; rFull = false;
  dc_idx = 0; dcReady = false;
  dcIR_avg = 2300.0; dcRed_avg = 2300.0;
  lastPeakTime = 0; prevPpgIR = 0;
  rising = false; peakVal = 0; maIdx = 0; ampIdx = 0;

  dcIR_sum = dcRed_sum = 2300L * DC_AVG_SIZE;
  for (int i = 0; i < DC_AVG_SIZE;   i++) { dcIR_buf[i] = dcRed_buf[i] = 2300; }
  for (int i = 0; i < SPO2_AVG_SIZE; i++) spo2Buf[i] = 0;
  for (int i = 0; i < BPM_AVG_SIZE;  i++) bpmBuf[i]  = 0;
  for (int i = 0; i < R_AVG_SIZE;    i++) rBuf[i]    = 0;
  for (int i = 0; i < MA_SIZE;       i++) { maACIR[i] = maACR[i] = 0; }
  for (int i = 0; i < AMP_BUF_SIZE;  i++) ampBuf[i]  = 0;
  for (int i = 0; i < PEAK_WIN_SIZE; i++) { winIR[i]  = winRed[i] = 0; }
  for (int i = 0; i < PEAK_AVG_SIZE; i++) { peakIR_buf[i] = peakRed_buf[i] = 0; }

  Serial2.println("STATUS:NO_FINGER");
}

// =============================================
// SETUP
// =============================================
void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, SLAVE_RX, SLAVE_TX);
  delay(500);

  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(CH_AC_RED, ADC_ATTEN_DB_11);
  adc1_config_channel_atten(CH_DC_RED, ADC_ATTEN_DB_11);
  adc1_config_channel_atten(CH_AC_IR,  ADC_ATTEN_DB_11);
  adc1_config_channel_atten(CH_DC_IR,  ADC_ATTEN_DB_11);

  pinMode(LED_RED, OUTPUT);
  pinMode(LED_IR,  OUTPUT);
  RED_LOW(); IR_LOW();

  // FIX #2: Timer API untuk ESP32 Arduino core v3.x
  // timerBegin(frequency_hz) — 1MHz = resolusi 1µs per tick
  sensorTimer = timerBegin(1000000);
  // timerAttachInterrupt tanpa argumen ke-3 (edge)
  timerAttachInterrupt(sensorTimer, &onSensorTimer);
  // timerAlarm(timer, period_in_ticks, auto_reload, reload_count)
  // PHASE_US = 250 → 250 tick @ 1MHz = 250µs
  timerAlarm(sensorTimer, PHASE_US, true, 0);

  for (int i = 0; i < DC_AVG_SIZE; i++) { dcIR_buf[i] = dcRed_buf[i] = 2300; }
  dcIR_sum = dcRed_sum = 2300L * DC_AVG_SIZE;

  Serial.println("=== ESP32-S3 SPO2 MONITOR ===");
  Serial.printf("Driver LED : Red=GPIO%d, IR=GPIO%d\n", LED_RED, LED_IR);
  Serial.printf("Timing     : %d phase x %dµs = %.0fHz siklus\n",
                8, PHASE_US, 1e6 / (8.0 * PHASE_US));
  Serial.printf("Baseline   : ACIR=%d  ACR=%d (manual)\n", BASELINE_ACIR, BASELINE_ACR);

  Serial.println("--- RAW ADC tanpa jari (3 detik) ---");
  unsigned long t = millis();
  while (millis() - t < 3000) {
    if (readPending) {
      portENTER_CRITICAL(&timerMux);
      readPending = false;
      portEXIT_CRITICAL(&timerMux);
      readADC();
      if (newSample) {
        newSample = false;
        Serial.printf("ACIR:%4d  ACR:%4d  DCIR:%4d  DCR:%4d\n", ACIR, ACR, DCIR, DCR);
      }
    }
  }
  Serial.println("--- Isi BASELINE_ACIR & BASELINE_ACR lalu upload ulang ---");
  Serial.println("Letakkan jari, monitoring dimulai...");

  Serial2.println("STATUS:MASTER_READY");
  Serial2.printf("BASELINE:%d,%d\n", BASELINE_ACIR, BASELINE_ACR);
}

// =============================================
// LOOP
// Processing (detectBPM, updatePeakWindow, calcSpO2)
// dijalankan SETIAP newSample — tidak di-gate prevMillis.
// Gate prevMillis hanya untuk Serial print.
// =============================================
void loop() {
  // Baca ADC setiap ISR set flag
  if (readPending) {
    portENTER_CRITICAL(&timerMux);
    readPending = false;
    portEXIT_CRITICAL(&timerMux);
    readADC();
  }

  if (!newSample) return;
  newSample = false;

  // --- Smoothing ---
  int smoothIR  = movingAvg(maACIR, ACIR);
  int smoothRed = movingAvg(maACR,  ACR);
  maIdx = (maIdx + 1) % MA_SIZE;

  int ppgIR  = smoothIR  - BASELINE_ACIR;
  int ppgRed = smoothRed - BASELINE_ACR;

  bool finger = (DCIR < DC_THRESHOLD && DCR < DC_THRESHOLD);

  if (finger) {
    updateDCAvg();
    detectBPM(ppgIR);
    updatePeakWindow(ppgIR, ppgRed);
    calcSpO2();

    bufIR[bufIdx]  = ppgIR;
    bufRed[bufIdx] = ppgRed;
    bufIdx = (bufIdx + 1) % BUF_SIZE;

    // Throttle Serial print saja (tidak mempengaruhi kalkulasi)
    unsigned long now = millis();
    if (now - prevMillis >= PRINT_INTERVAL) {
      prevMillis = now;
      Serial.printf("ppgIR:%d  ppgRed:%d\n", ppgIR, ppgRed);
      Serial2.printf("PPG:%d\n", ppgRed);
    }

    // Output SpO2/BPM setiap OUTPUT_INTERVAL ms
    if (now - prevOutputMillis >= OUTPUT_INTERVAL) {
      prevOutputMillis = now;
      Serial.printf("[OUT] SpO2:%d%%  BPM:%d  R:%.4f  dcIR:%.0f  dcRed:%.0f\n",
                    (int)SPO2, BPM, R_last, dcIR_avg, dcRed_avg);
      Serial2.printf("SPO2DATA:%.1f,%.1f,%d,%d,%d,%d,%.4f\n",
                     peakIR_avg, peakRed_avg, (int)SPO2, BPM, DCIR, DCR, R_last);
    }
  } else {
    resetAll();
  }
}