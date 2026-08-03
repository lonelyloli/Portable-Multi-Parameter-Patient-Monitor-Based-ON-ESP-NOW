// ============================================================
// BAGIAN 1 — HEADER & LIBRARY
// ============================================================
#include <Arduino.h>      // Core framework Arduino
#include <math.h>         // Fungsi matematika (sqrtf, fabsf, dll)
#include <WiFi.h>         // Manajemen mode WiFi ESP32
#include <esp_now.h>      // Protokol komunikasi peer-to-peer ESP-NOW
#include <esp_wifi.h>     // Kontrol level rendah WiFi (power saving)

// ============================================================
// BAGIAN 2 — ALAMAT MAC PENERIMA & STRUKTUR DATA KIRIM
// ============================================================
uint8_t receiverAddress[] = {0x28, 0x37, 0x2f, 0x86, 0xb1, 0xa8};

typedef struct struct_message {
  int SYSx;           // Tekanan darah sistolik (mmHg)
  int DIAx;           // Tekanan darah diastolik (mmHg)
  int Tempx;          // Suhu tubuh
  int SpO2x;          // Saturasi oksigen darah (%)
  int HRx;            // Heart Rate / denyut jantung (bpm)
  int RRx;            // Respiration Rate / laju napas (napas/menit)
  int ecgLead1;       // Gelombang ECG lead 1
  int ecgLead2;       // Gelombang ECG lead 2
  int ecgLead3;       // Gelombang ECG lead 3
  int spo2Wave;       // Gelombang plethysmografi SpO2
  int respWave;       // Gelombang respirasi (skala 10–200)
  int leadOff;        // Status elektroda: 1=lepas, 0=terpasang
  int tekananRT;      // Tekanan real-time manset NIBP
  int faseNIBP;       // Fase pengukuran NIBP (inflate/deflate/idle)
  int cntMenit;       // Counter menit untuk timer NIBP otomatis
  int battPct;        // Persentase baterai
  int korotkoffBeat;  // Deteksi bunyi Korotkoff untuk NIBP auskultasi
} struct_message;

// ============================================================
// BAGIAN 3 — DEFINISI PIN & SAMPLING RATE
// ============================================================
#define ECG_PIN         4     // Pin ADC input sinyal ECG
#define FS              250   // Sampling rate 250 Hz (standar klinis)
#define ECG_QUEUE_LEN   512   // Panjang antrian FreeRTOS

// ============================================================
// BAGIAN 4a — KONSTANTA DC REMOVAL
// ============================================================
#define DC_ALPHA_FAST   0.9995f   // Koefisien IIR untuk estimasi DC (time constant ~8 detik)
#define DC_ALPHA_SEED   0.0005f   // Komplemen DC_ALPHA_FAST

// ============================================================
// BAGIAN 4b — KONSTANTA ENVELOPE TRACKER (EDR-AM)
// ============================================================
#define RESP_BASELINE   50    // Nilai tengah output gelombang napas
#define RESP_AMPLITUDE  40    // Amplitudo gelombang napas (±40 dari baseline)
#define RESP_OUT_MIN    10    // Batas bawah output resp wave
#define RESP_OUT_MAX    200   // Batas atas output resp wave

#define ENV_ATTACK      0.65f     // Kecepatan naik envelope (cepat saat R-peak besar)
#define ENV_DECAY       0.015f    // Kecepatan turun envelope (lambat antar R-peak)
#define ENV_SCALE       100.0f    // Skala envelope sebelum difilter
#define EDR_SCALE       500000.0f // Skala sinyal EDR setelah filter
#define EDR_DC_ALPHA    0.992f    // Koefisien DC removal sinyal EDR
#define EDR_DC_BETA     0.008f    // Komplemen EDR_DC_ALPHA
#define EDR_LPF_ALPHA   0.988f    // Koefisien LPF final EDR

// ============================================================
// BAGIAN 4c — KONSTANTA NORMALISASI EDR
// ============================================================
#define NORM_ATTACK         0.30f   // Kecepatan adaptasi batas atas/bawah EDR
#define NORM_DECAY_FAST     0.002f  // Decay cepat saat range sinyal kecil
#define NORM_DECAY_MED      0.0008f // Decay sedang
#define NORM_DECAY_SLOW     0.0004f // Decay lambat saat range sinyal besar
#define NORM_RANGE_MIN      0.40f   // Batas minimum range normalisasi
#define NORM_DEADZONE_SMALL 0.20f   // Deadzone saat range sinyal kecil
#define NORM_DEADZONE_MED   0.08f   // Deadzone sedang
#define NORM_DEADZONE_LARGE 0.04f   // Deadzone saat range sinyal besar

// ============================================================
// BAGIAN 4d — KONSTANTA DETEKSI BREATH HOLD (NAFAS BERHENTI)
// ============================================================
#define HOLD_WIN_LEN            500       // Window 2 detik @ 250Hz untuk hitung variansi EDR
#define BREATH_HOLD_THRESH      0.003f    // Variansi EDR < ini = nafas berhenti
#define BREATH_HOLD_CONFIRM_MS  2000UL    // Harus low-variance selama 2 detik untuk konfirmasi
#define RR_MIN_PEAKS_FOR_HOLD   4         // Butuh minimal 4 peak napas valid sebelum cek breath hold

// ============================================================
// BAGIAN 4e — KONSTANTA PAN-TOMPKINS (DETEKSI R-PEAK)
// ============================================================
#define MWI_LEN         28      // Panjang Moving Window Integral (112ms @ 250Hz)
#define MWI_LEN_FAST    18      // Panjang MWI saat HR > 150 bpm
#define MWI_HR_FAST_THRESH 150.0f // Batas HR untuk switch ke MWI pendek
#define TWAVE_GAP_MAX   90.0f   // Max 90 sampel (360ms) setelah R untuk cek T-wave
#define TWAVE_AMP_RATIO 0.55f   // T-wave < 55% amplitudo R-peak = bukan R-peak
#define REFR_RATIO      0.60f   // Refractory = 60% dari estimasi RRI
#define REFR_MAX        100     // Refractory maksimum (100 sampel = 400ms)
#define REFR_MIN        40      // Refractory minimum (40 sampel = 160ms)
#define REFR_DEFAULT    75      // Refractory default saat HR belum diketahui
#define SPKI_INIT       0.5f    // Nilai awal Signal Peak Index
#define NPKI_INIT       0.05f   // Nilai awal Noise Peak Index
#define THR_INIT_RATIO  0.25f   // Rasio threshold saat inisialisasi
#define THR_RUN_RATIO   0.35f   // Rasio threshold saat running normal
#define SPKI_ALPHA      0.125f  // Learning rate update SPKI normal
#define NPKI_ALPHA      0.125f  // Learning rate update NPKI
#define SPKI_ALPHA_FAST 0.05f   // Learning rate SPKI saat refractory
#define TWAVE_HIST_LEN  4       // Panjang riwayat amplitudo R-peak untuk T-wave check

// ============================================================
// BAGIAN 4f — KONSTANTA RRI & HEART RATE
// ============================================================
#define RRI_MIN_S           0.17f  // RRI minimum (HR max ~352 bpm)
#define RRI_MAX_S           5.00f  // RRI maksimum (HR min ~12 bpm)
#define RRI_BUF_LEN         8      // Buffer 8 RRI terakhir
#define RRI_OUTLIER_HIGH    2.0f   // RRI > 2× median = outlier, diabaikan
#define RRI_OUTLIER_LOW     0.5f   // RRI < 0.5× median = outlier, diabaikan
#define RRI_JUMP_THRESH     30.0f  // Perubahan HR > 30 bpm = lompatan besar
#define RRI_CONSISTENT_TOL  0.15f  // Toleransi RRI dianggap konsisten
#define RRI_CONSISTENT_NEED 2      // Butuh 2 RRI konsisten untuk konfirmasi lompatan HR
#define HR_ALPHA_JUMP_XL    0.80f  // Alpha EMA saat HR lompat > 80 bpm
#define HR_ALPHA_JUMP_L     0.65f  // Alpha EMA saat HR lompat > 40 bpm
#define HR_ALPHA_JUMP_M     0.45f  // Alpha EMA saat HR lompat > 20 bpm
#define HR_ALPHA_JUMP_S     0.20f  // Alpha EMA saat HR lompat > 10 bpm
#define HR_ALPHA_FAST       0.40f  // Alpha EMA normal cepat (delta > 10 bpm)
#define HR_ALPHA_SLOW       0.20f  // Alpha EMA normal lambat (stabil)

// ============================================================
// BAGIAN 4g — KONSTANTA DISPLAY HR/RR & FREEZE HANDLING
// ============================================================
#define HR_MIN_VALID        14.0f   // HR minimum valid untuk ditampilkan
#define HR_MAX_VALID        352.0f  // HR maksimum valid
#define RR_MIN_VALID        3.0f    // RR minimum valid (napas/menit)
#define RR_MAX_VALID        90.0f   // RR maksimum valid
#define HR_OUTLIER_BPM      5.0f    // Selisih mean vs median HR > ini = outlier
#define HR_JUMP_DISPLAY     15.0f   // Lompatan HR > ini → paksa update display
#define NO_RPEAK_RESET_MS   15000UL // Reset HR jika 15 detik tidak ada R-peak
#define DISPLAY_FREEZE_MS   3000UL  // Freeze tampilan HR/RR selama 3 detik saat signal hilang
#define RESP_FREEZE_MS      4000UL  // Freeze gelombang resp selama 4 detik saat signal hilang

// ============================================================
// BAGIAN 4h — KONSTANTA DETEKSI LAJU NAPAS (RR DETECTION)
// ============================================================
#define RR_MED_LEN          7       // Panjang buffer median filter RR
#define RR_PERIOD_MIN       0.50f   // Periode napas minimum (2 detik = 120 napas/menit)
#define RR_PERIOD_MAX       20.0f   // Periode napas maksimum (20 detik = 3 napas/menit)
#define RR_THRESH_RATIO     0.45f   // Threshold = 45% dari puncak EDR tertinggi
#define RR_THRESH_MIN       0.25f   // Threshold minimum deteksi puncak napas
#define RR_ALPHA_JUMP_XL    0.60f   // Alpha EMA RR saat delta > 15 napas/menit
#define RR_ALPHA_JUMP_L     0.35f   // Alpha EMA RR saat delta > 8 napas/menit
#define RR_ALPHA_JUMP_M     0.20f   // Alpha EMA RR saat delta > 3 napas/menit
#define RR_ALPHA_SLOW       0.10f   // Alpha EMA RR normal
#define RR_DELTA_JUMP_XL    15.0f   // Ambang lompatan RR sangat besar
#define RR_DELTA_JUMP_L     8.0f    // Ambang lompatan RR besar
#define RR_DELTA_JUMP_M     3.0f    // Ambang lompatan RR sedang

// ============================================================
// BAGIAN 4h — KONSTANTA KUALITAS SINYAL (SIGNAL QUALITY)
// ============================================================
#define SQ_WIN_LEN          500     // Window RMS 2 detik @ 250Hz
#define SQ_RMS_MIN          5.0f    // RMS minimum (sinyal terlalu lemah = elektroda buruk)
#define SQ_RMS_MAX          1500.0f // RMS maksimum (sinyal terlalu kuat = noise/interferensi)
#define SQ_SAT_LOW          80      // Batas bawah ADC saturasi (elektroda lepas)
#define SQ_SAT_HIGH         4000    // Batas atas ADC saturasi
#define SQ_SAT_COUNT_MAX    10      // Berapa kali saturasi berturut-turut = signal invalid
#define SQ_LOST_MS          3500UL  // Tunggu 3.5 detik sebelum hard reset
#define SQ_VALID_HYSTERESIS 25      // Butuh 25 sampel valid berturut-turut untuk unlock
#define LEADOFF_RAIL_LOW    100     // Rail bawah deteksi lead-off
#define LEADOFF_RAIL_HIGH   4000    // Rail atas deteksi lead-off
#define LEADOFF_RAIL_COUNT  12      // Berapa kali rail berturut-turut = lead-off

// ============================================================
// BAGIAN 4i — KONSTANTA DETEKSI PERUBAHAN ELEKTRODA
// ============================================================
#define ELEC_WIN_LEN        500     // Window histogram 2 detik
#define ELEC_CHANGE_THRESH  600     // Perubahan P10 > 600 ADC unit = elektroda baru/pindah
#define HIST_BUCKETS        256     // Jumlah bucket histogram
#define HIST_BUCKET_SIZE    16      // Ukuran setiap bucket (4096 / 256)

// ============================================================
// BAGIAN 4j — KONSTANTA PHANTOM DETECTOR
// ============================================================
#define JITTER_WIN_LEN          8       // Buffer 8 RRI untuk hitung jitter
#define JITTER_THRESH           0.012f  // Std dev RRI < 12ms = sinyal terlalu reguler = phantom
#define PHANTOM_CONFIRM_MS      3000UL  // Konfirmasi phantom setelah 3 detik
#define PHANTOM_RELEASE_MS      500UL   // Rilis phantom setelah 500ms jitter normal
#define RMS_PHANTOM_OFF_LOW     150.0f  // RMS batas bawah kondisi phantom-off
#define RMS_PHANTOM_OFF_HIGH    270.0f  // RMS batas atas kondisi phantom-off
#define PHANTOM_OFF_CONFIRM_MS  4000UL  // Konfirmasi phantom-off setelah 4 detik
#define PHANTOM_JITTER_CHANGE_PCT 0.05f // Threshold perubahan RRI di mode phantom

// ============================================================
// BAGIAN 5 — DEKLARASI TIMER HARDWARE
// ============================================================
hw_timer_t* sampTimer = NULL; // Handle timer hardware ESP32 untuk sampling 250 Hz

// ============================================================
// BAGIAN 6 — KOEFISIEN FILTER DIGITAL IIR (SOS BIQUAD)
// Format tiap baris: [b0, b1, b2, a1, a2]
// y[n] = b0*w[n] + b1*w[n-1] + b2*w[n-2]
// w[n] = x[n] - a1*w[n-1] - a2*w[n-2]
// ============================================================

// 6a. Notch Filter 50 Hz (4 stage) — menghapus noise PLI dari listrik
#define NOTCH_STAGES 4
const float notch_sos[NOTCH_STAGES][5] = {
  {+1.0000000000000e+00f,-6.1897704996653e-01f,+1.0003175798310e+00f,-5.5594184155116e-01f,+9.1067140170081e-01f},
  {+1.0000000000000e+00f,-6.1865410782003e-01f,+9.9968245996992e-01f,-6.2632090466303e-01f,+9.1173552530244e-01f},
  {+1.0000000000000e+00f,-6.1853417207484e-01f,+1.0000665103821e+00f,-5.1903150993542e-01f,+9.6174461049192e-01f},
  {+1.0000000000000e+00f,-6.1909698885821e-01f,+9.9993355508050e-01f,-6.9270315576596e-01f,+9.6286218069479e-01f}
};
const float notch_gain = +8.7685388967325e-01f;

// 6b. High-Pass Filter ~0.5 Hz (1 stage) — menghapus baseline wander / gerak tubuh
#define HPF_STAGES 1
const float hpf_sos[HPF_STAGES][5] = {
  {+1.0000000000000e+00f,-2.0000000000000e+00f,+1.0000000000000e+00f,-1.9822289297925e+00f,+9.8238545061413e-01f}
};
const float hpf_gain = +9.9115359510166e-01f;

// 6c. Band-Pass Filter ~5–15 Hz (6 stage) — mengisolasi QRS complex
#define BPF_STAGES 6
const float bpf_sos[BPF_STAGES][5] = {
  {+1.0000000000000e+00f,+2.0000076048995e+00f,+1.0000020287939e+00f,-1.6904193148551e+00f,+7.4965840860860e-01f},
  {+1.0000000000000e+00f,-2.0000068860856e+00f,+1.0000010079677e+00f,-1.6813540365866e+00f,+7.7989271827895e-01f},
  {+1.0000000000000e+00f,+2.0023575685709e+00f,+1.0023631536609e+00f,-1.7870458722306e+00f,+8.1718556256109e-01f},
  {+1.0000000000000e+00f,+1.9976348265296e+00f,+9.9764039366170e-01f,-1.8795980899042e+00f,+8.9872933425626e-01f},
  {+1.0000000000000e+00f,-2.0024210404345e+00f,+1.0024269269188e+00f,-1.7781203591873e+00f,+9.0885330158672e-01f},
  {+1.0000000000000e+00f,-1.9975720734799e+00f,+9.9757794326913e-01f,-1.9512981931286e+00f,+9.6720938469124e-01f}
};
const float bpf_gain = +2.4972225268904e-06f;

// 6d. Resp High-Pass Filter ~0.1 Hz (1 stage) — menghapus DC drift dari sinyal EDR
#define RESP_HP_STAGES 1
const float resp_hp_sos[RESP_HP_STAGES][5] = {
  {+1.0000000000000e+00f,-2.0000000000000e+00f,+1.0000000000000e+00f,-1.9982228472918e+00f,+9.9822442502640e-01f}
};
const float resp_hp_gain = +9.9911181807956e-01f;

// 6e. Resp Low-Pass Filter ~0.5 Hz (2 stage) — mengisolasi sinyal napas 0.1–0.5 Hz (6–30 napas/menit)
#define RESP_LP_STAGES 2
const float resp_lp_sos[RESP_LP_STAGES][5] = {
  {+1.0000000000000e+00f,+2.0000000335436e+00f,+9.9999998552351e-01f,-1.9540019616482e+00f,+9.5461925135634e-01f},
  {+1.0000000000000e+00f,+1.9999999664564e+00f,+1.0000000144765e+00f,-1.9803238591505e+00f,+9.8094946421992e-01f}
};
const float resp_lp_gain = +2.4136223135161e-08f;

// ============================================================
// BAGIAN 7 — STATE VARIABLES FILTER (DELAY STATE BIQUAD)
// Setiap stage biquad butuh 2 nilai memori [w(n-1), w(n-2)]
// ============================================================
float w_notch  [NOTCH_STAGES][2]   = {0}; // State notch filter
float w_hpf    [HPF_STAGES][2]     = {0}; // State HPF
float w_bpf    [BPF_STAGES][2]     = {0}; // State BPF
float w_resp_hp[RESP_HP_STAGES][2] = {0}; // State resp high-pass
float w_resp_lp[RESP_LP_STAGES][2] = {0}; // State resp low-pass

// ============================================================
// BAGIAN 8 — BUFFER ADC & ANTRIAN DATA FREERTOS
// ============================================================
#define ADC_BUF_LEN  512
#define ADC_MASK     (ADC_BUF_LEN - 1)
volatile int      adc_buf[ADC_BUF_LEN]; // Circular buffer sampel ADC mentah
volatile uint32_t adc_head   = 0;        // Head pointer circular buffer
volatile uint32_t total_samp = 0;        // Total sampel sejak boot
portMUX_TYPE      bufMux = portMUX_INITIALIZER_UNLOCKED; // Spinlock untuk akses buffer dari ISR

// Struktur satu sampel data yang dikirim ke loop() via queue
typedef struct {
  float    time_s;   // Waktu dalam detik
  uint16_t lead2;    // Nilai ADC raw Lead 2
  float    hr;       // Heart Rate saat ini (bpm)
  float    rr;       // Respiration Rate saat ini (napas/menit)
  float    edr;      // Nilai EDR ternormalisasi [-1, +1]
} DataSample;

QueueHandle_t     dataQueue;   // Antrian FreeRTOS: taskProcessing → loop()
SemaphoreHandle_t dataMutex;   // Mutex: proteksi akses struct dataKirim
struct_message    dataKirim;   // Paket data yang dikirim via ESP-NOW ke master

// ============================================================
// BAGIAN 9 — STATE VARIABLES PAN-TOMPKINS
// ============================================================
float  mwi_buf[MWI_LEN] = {0}; // Circular buffer Moving Window Integral
float  mwi_sum = 0;             // Running sum MWI untuk kalkulasi O(1)
int    mwi_idx = 0;             // Index circular buffer MWI
float  derBuf[5] = {0};         // Buffer 5 sampel untuk derivatif 5-titik
int    derIdx    = 0;           // Index buffer derivatif
float  SPKI = SPKI_INIT;        // Signal Peak Index (estimasi amplitudo QRS)
float  NPKI = NPKI_INIT;        // Noise Peak Index (estimasi level noise)
float  THR  = 0;                // Threshold deteksi R-peak (adaptif)
bool   thr_init   = false;      // Flag: threshold sudah diinisialisasi dari data nyata
int    refr_count = 0;          // Counter refractory period (sampel tersisa)
#define MAX_R 512
volatile uint32_t r_idx[MAX_R]; // Array index sampel tiap R-peak terdeteksi
volatile int      r_count = 0;  // Total R-peak terdeteksi
volatile uint32_t last_r  = 0;  // Index sampel R-peak terakhir

// Buffer riwayat amplitudo R-peak untuk referensi T-wave rejection
static float  twave_hist[TWAVE_HIST_LEN] = {0};
static int    twave_hist_idx   = 0;
static int    twave_hist_count = 0;

// Rata-rata amplitudo R-peak dari riwayat (untuk T-wave check)
float twaveGetMeanAmp() {
  if (twave_hist_count == 0) return 0.0f;
  float s = 0;
  int n = (twave_hist_count < TWAVE_HIST_LEN) ? twave_hist_count : TWAVE_HIST_LEN;
  for (int i = 0; i < n; i++) s += twave_hist[i];
  return s / n;
}

// ============================================================
// BAGIAN 10 — STATE VARIABLES HEART RATE (HR)
// ============================================================
volatile float hr_bpm     = 0.0f; // HR terfilter EMA (ditampilkan ke user)
volatile float hr_instant = 0.0f; // HR instan dari satu RRI (responsif tapi berisik)
static float rri_buf[RRI_BUF_LEN] = {0}; // Buffer 8 RRI terakhir untuk median
static int   rri_buf_idx = 0;             // Index tulis buffer RRI
static int   rri_buf_count = 0;           // Jumlah RRI valid dalam buffer
static int   rri_consistent_count = 0;    // Counter konfirmasi saat HR lompat besar
static float rri_consistent_val   = 0.0f; // Nilai RRI yang berulang konsisten

// ============================================================
// BAGIAN 11 — STATE VARIABLES EDR-AM (ECG-DERIVED RESPIRATION)
// ============================================================
static float    rp_env      = 0;     // Nilai envelope R-peak saat ini
static float    rp_curr_amp = 0;     // Amplitudo R-peak terbaru
static uint32_t rp_curr_idx = 0;     // Index sampel R-peak terbaru
static bool     rp_ready    = false; // Flag: sudah ada R-peak pertama
static float    edr_dc      = 0;     // Komponen DC sinyal EDR (di-remove)
static float    out_lpf     = 0;     // Output LPF final EDR sebelum normalisasi
static float    norm_min    = 0;     // Batas bawah normalisasi adaptif EDR
static float    norm_max    = 0;     // Batas atas normalisasi adaptif EDR
static bool     norm_inited = false; // Flag: normalisasi sudah diinisialisasi

// ============================================================
// BAGIAN 12 — STATE VARIABLES BREATH HOLD DETECTION
// ============================================================
static float    edr_var_buf[HOLD_WIN_LEN] = {0}; // Buffer variansi EDR (running)
static int      edr_var_idx  = 0;                 // Index circular buffer variansi
static float    edr_var_sum  = 0;                 // Running sum variansi EDR
static bool     breath_hold  = false;             // Flag: pasien sedang menahan napas
static bool     hold_candidate    = false;         // Flag: kandidat breath hold (belum konfirmasi)
static uint32_t hold_candidate_ms = 0;             // Waktu mulai kandidat breath hold
static int      rr_valid_peak_count = 0;           // Counter peak napas valid (untuk FIX-E)
static int      rr_display_frozen   = 0;           // Nilai RR terakhir valid (untuk freeze)

// ============================================================
// BAGIAN 13 — STATE VARIABLES RESP WAVE FREEZE
// ============================================================
static int      resp_wave_frozen   = RESP_BASELINE; // Nilai resp wave terakhir valid
static uint32_t signal_lost_ms     = 0;              // Waktu sinyal pertama kali invalid
static bool     signal_was_invalid = false;           // Flag: sinyal sebelumnya invalid

// ============================================================
// BAGIAN 14 — STATE VARIABLES PHANTOM DETECTOR
// ============================================================
static float    jitter_rri_buf[JITTER_WIN_LEN] = {0}; // Buffer 8 RRI untuk hitung jitter
static int      jitter_buf_idx   = 0;      // Index buffer jitter
static int      jitter_buf_count = 0;      // Jumlah data valid dalam buffer jitter
static float    rri_jitter       = 0.0f;   // Std dev RRI saat ini (HRV indicator)
static bool     amd_is_phantom   = false;  // Flag: sinyal terdeteksi sebagai phantom
static bool     jitter_low_now   = false;  // Flag: jitter saat ini rendah
static uint32_t jitter_low_since = 0;      // Waktu jitter pertama kali rendah
static uint32_t jitter_hi_since  = 0;      // Waktu jitter kembali tinggi
static uint32_t phantom_off_since  = 0;    // Waktu deteksi phantom-off mulai
static bool     phantom_off_armed  = false; // Flag: phantom-off sedang dikonfirmasi
volatile bool   is_phantom_off     = false; // Flag: kondisi phantom-off terkonfirmasi

// ============================================================
// BAGIAN 15 — STATE VARIABLES RR DETECTION
// ============================================================
static float    rr_prev_val   = 0;     // Nilai EDR satu sampel lalu (untuk deteksi puncak)
static float    rr_prev_prev  = 0;     // Nilai EDR dua sampel lalu
static uint32_t rr_last_peak  = 0;     // Index sampel puncak napas terakhir
static bool     rr_peak_init  = false; // Flag: puncak napas pertama sudah ditemukan
static float    rr_bpm        = 0;     // Laju napas terfilter EMA (napas/menit)
static float    rr_dyn_thresh = 0.5f;  // Threshold dinamis deteksi puncak EDR
static float    rr_peak_max   = 0.5f;  // Nilai puncak EDR tertinggi (untuk threshold)
static float    rr_med_buf[RR_MED_LEN]; // Buffer 7 nilai RR untuk median filter
static int      rr_med_idx    = 0;      // Index buffer median RR

volatile float edr_out_last  = 0;           // Output EDR ternormalisasi terakhir [-1,+1]
volatile int   resp_wave_out = RESP_BASELINE; // Output resp wave untuk display [10-200]

// ============================================================
// BAGIAN 16 — STATE VARIABLES DISPLAY HR/RR & FREEZE HANDLING
// ============================================================
volatile int    hr_display        = 0; // Nilai HR yang ditampilkan saat ini
volatile int    rr_display        = 0; // Nilai RR yang ditampilkan saat ini
static int      hr_display_frozen = 0; // Nilai HR terakhir valid (untuk freeze)
static uint32_t display_lost_ms   = 0; // Waktu sinyal mulai hilang
static bool     display_was_lost  = false; // Flag: display sedang dalam mode freeze
static uint32_t last_rpeak_ms     = 0; // Timestamp R-peak terakhir (untuk timeout reset)

// ============================================================
// BAGIAN 17 — STATE VARIABLES SIGNAL QUALITY
// ============================================================
static int      leadoff_rail_cnt  = 0;     // Counter sampel di rail ADC (lead-off)
volatile bool   lead_off          = false; // Flag: elektroda terlepas
static float    sq_sum_sq         = 0;     // Running sum of squares untuk RMS
static float    sq_buf[SQ_WIN_LEN] = {0};  // Circular buffer untuk running RMS
static int      sq_buf_idx  = 0;           // Index circular buffer RMS
static int      sq_sat_count = 0;          // Counter sampel saturasi ADC berturut-turut
volatile bool   sq_signal_ok      = false; // Flag: kualitas sinyal OK
static uint32_t sq_lost_since_ms  = 0;     // Waktu sinyal mulai jelek
static bool     sq_was_lost       = true;  // Flag: sinyal sebelumnya jelek
static int      sq_valid_count    = 0;     // Counter hysteresis valid signal

// ============================================================
// BAGIAN 18 — STATE VARIABLES ELECTRODE CHANGE DETECTOR
// Running histogram O(1) untuk menghitung P10 distribusi ADC
// ============================================================
static uint16_t elec_hist[HIST_BUCKETS]     = {0}; // Histogram distribusi ADC
static int      elec_circ_buf[ELEC_WIN_LEN] = {0}; // Circular buffer nilai ADC
static int      elec_circ_idx  = 0;    // Index circular buffer histogram
static bool     elec_buf_full  = false; // Flag: buffer histogram sudah penuh
static int      elec_p10_prev  = 2048;  // P10 distribusi ADC periode sebelumnya
volatile bool   elec_check_flag = false; // Flag: minta evaluasi perubahan elektroda

// Tambah satu nilai ke histogram (O(1))
inline void histAdd(int raw) {
  int b = raw / HIST_BUCKET_SIZE;
  if (b < 0) b = 0;
  if (b >= HIST_BUCKETS) b = HIST_BUCKETS - 1;
  elec_hist[b]++;
}

// Hapus satu nilai dari histogram (O(1))
inline void histRemove(int raw) {
  int b = raw / HIST_BUCKET_SIZE;
  if (b < 0) b = 0;
  if (b >= HIST_BUCKETS) b = HIST_BUCKETS - 1;
  if (elec_hist[b] > 0) elec_hist[b]--;
}

// Hitung persentil ke-10 dari histogram (P10)
int histGetP10() {
  int target = ELEC_WIN_LEN / 10; // 10% dari 500 = 50
  int cum = 0;
  for (int i = 0; i < HIST_BUCKETS; i++) {
    cum += elec_hist[i];
    if (cum >= target)
      return i * HIST_BUCKET_SIZE + HIST_BUCKET_SIZE / 2;
  }
  return (HIST_BUCKETS - 1) * HIST_BUCKET_SIZE;
}

// ============================================================
// BAGIAN 19 — INTERRUPT SERVICE ROUTINE (ISR) TIMER SAMPLING
// ============================================================
volatile bool samp_flag = false;

// ISR: dipanggil setiap 4ms (250 Hz) oleh timer hardware
// IRAM_ATTR: fungsi disimpan di RAM bukan Flash agar tidak ada cache miss
void IRAM_ATTR onSampTimer() { samp_flag = true; }

// ============================================================
// BAGIAN 20 — FUNGSI HELPER: THREAD-SAFE DATA UPDATE
// ============================================================
// Update struct dataKirim dengan mutex untuk keamanan multi-task
void updateDataKirim(int hr, int rr, int resp, int lo) {
  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
    dataKirim.HRx      = hr;
    dataKirim.RRx      = rr;
    dataKirim.respWave = resp;
    dataKirim.leadOff  = lo;
    xSemaphoreGive(dataMutex);
  }
}

// ============================================================
// BAGIAN 21 — FUNGSI HELPER: PHANTOM DETECTOR (JITTER UPDATE)
// Mendeteksi sinyal "palsu" berdasarkan std dev RRI terlalu kecil
// ============================================================
void jitterUpdate(float rri_s) {
  if (rri_s < RRI_MIN_S || rri_s > RRI_MAX_S) return;

  // Jika sedang phantom: cek apakah RRI berubah signifikan (reset buffer)
  if (amd_is_phantom && jitter_buf_count >= 4) {
    float sum_old = 0;
    int n = (jitter_buf_count < JITTER_WIN_LEN) ? jitter_buf_count : JITTER_WIN_LEN;
    for (int i = 0; i < n; i++) sum_old += jitter_rri_buf[i];
    float mean_old = sum_old / n;
    if (mean_old > 1e-6f) {
      float diff_pct = fabsf(rri_s - mean_old) / mean_old;
      if (diff_pct > PHANTOM_JITTER_CHANGE_PCT) {
        // RRI berubah drastis → reset buffer jitter
        for (int i = 0; i < JITTER_WIN_LEN; i++) jitter_rri_buf[i] = rri_s;
        jitter_buf_idx   = JITTER_WIN_LEN;
        jitter_buf_count = JITTER_WIN_LEN;
        rri_jitter = 0.0f;
        return;
      }
    }
  }

  // Isi circular buffer
  jitter_rri_buf[jitter_buf_idx % JITTER_WIN_LEN] = rri_s;
  jitter_buf_idx++;
  if (jitter_buf_count < JITTER_WIN_LEN) jitter_buf_count++;

  // Butuh buffer penuh untuk evaluasi
  if (jitter_buf_count < JITTER_WIN_LEN) {
    amd_is_phantom = false; jitter_low_now = false; return;
  }

  // Hitung std dev RRI (sample variance)
  float sum = 0;
  for (int i = 0; i < JITTER_WIN_LEN; i++) sum += jitter_rri_buf[i];
  float mean_rri = sum / JITTER_WIN_LEN;
  float var = 0;
  for (int i = 0; i < JITTER_WIN_LEN; i++) {
    float dv = jitter_rri_buf[i] - mean_rri; var += dv * dv;
  }
  rri_jitter = sqrtf(var / (JITTER_WIN_LEN - 1));

  // Evaluasi: jitter rendah = phantom, jitter tinggi = real signal
  bool jitter_below = (rri_jitter < JITTER_THRESH);
  if (jitter_below) {
    if (!jitter_low_now) { jitter_low_since=millis(); jitter_low_now=true; jitter_hi_since=0; }
    if ((millis()-jitter_low_since) > PHANTOM_CONFIRM_MS) amd_is_phantom = true;
  } else {
    if (jitter_low_now) {
      if (jitter_hi_since==0) jitter_hi_since=millis();
      if ((millis()-jitter_hi_since) > PHANTOM_RELEASE_MS) {
        jitter_low_now=false; jitter_hi_since=0; amd_is_phantom=false;
      }
    } else amd_is_phantom = false;
  }
}

// ============================================================
// BAGIAN 22 — FUNGSI HELPER: MEDIAN FILTER RR (7 TITIK)
// Lebih robust dari mean terhadap outlier deteksi napas
// ============================================================
float rr_median() {
  float tmp[RR_MED_LEN];
  memcpy(tmp, rr_med_buf, sizeof(tmp));
  for (int i=0;i<RR_MED_LEN-1;i++)
    for (int j=i+1;j<RR_MED_LEN;j++)
      if (tmp[j]<tmp[i]){float t=tmp[i];tmp[i]=tmp[j];tmp[j]=t;}
  return tmp[RR_MED_LEN/2];
}

// ============================================================
// BAGIAN 23 — FUNGSI HELPER: MEDIAN FILTER RRI (HR)
// ============================================================
float rriMedian(float* arr, int n) {
  float tmp[RRI_BUF_LEN];
  memcpy(tmp, arr, n*sizeof(float));
  for (int i=0;i<n-1;i++)
    for (int j=i+1;j<n;j++)
      if (tmp[j]<tmp[i]){float t=tmp[i];tmp[i]=tmp[j];tmp[j]=t;}
  return tmp[n/2];
}

// ============================================================
// BAGIAN 24 — FUNGSI HELPER: REFRACTORY PERIOD ADAPTIF
// Refractory disesuaikan dengan HR: HR tinggi → refractory lebih pendek
// ============================================================
inline int adaptiveRefr(float hr_est) {
  if (hr_est < 1.0f) return REFR_DEFAULT;
  int b = (int)((60.0f/hr_est)*FS*REFR_RATIO);
  if (b>REFR_MAX) b=REFR_MAX;
  if (b<REFR_MIN) b=REFR_MIN;
  return b;
}

// ============================================================
// BAGIAN 25 — FUNGSI HELPER: SOS BIQUAD FILTER ENGINE
// Core pemrosesan filter IIR bertingkat (Direct Form II Transposed)
// ============================================================
float sosBiquad(float x, const float sos[][5], int stages, float gain, float w[][2]) {
  float v = x * gain;
  for (int i=0;i<stages;i++) {
    float b0=sos[i][0],b1=sos[i][1],b2=sos[i][2],a1=sos[i][3],a2=sos[i][4];
    float w0=v-a1*w[i][0]-a2*w[i][1];
    float y=b0*w0+b1*w[i][0]+b2*w[i][1];
    w[i][1]=w[i][0]; w[i][0]=w0; v=y;
  }
  return v;
}

// ============================================================
// BAGIAN 26 — FUNGSI HELPER: DERIVATIF 5-TITIK
// Menghitung slope sinyal — slope tinggi = transisi cepat = kandidat QRS
// Rumus: (FS/8) × [x(n) + 2x(n-1) - 2x(n-3) - x(n-4)]
// ============================================================
float deriv5pt(float x) {
  derBuf[derIdx]=x;
  int i4=derIdx,i3=(derIdx+4)%5,i1=(derIdx+2)%5,i0=(derIdx+1)%5;
  derIdx=(derIdx+1)%5;
  return (FS/8.0f)*(derBuf[i4]+2*derBuf[i3]-2*derBuf[i1]-derBuf[i0]);
}

// ============================================================
// BAGIAN 27 — FUNGSI HELPER: MOVING WINDOW INTEGRAL (MWI)
// Mengubah puncak tajam kuadrat derivatif menjadi kurva lebar
// Circular buffer O(1) — tidak perlu re-sum setiap sampel
// ============================================================
float mwiUpdate(float x) {
  mwi_sum-=mwi_buf[mwi_idx];
  mwi_buf[mwi_idx]=x;
  mwi_sum+=x;
  mwi_idx=(mwi_idx+1)%MWI_LEN;
  return mwi_sum/MWI_LEN;
}

// ============================================================
// BAGIAN 28 — FUNGSI HELPER: SIGNAL QUALITY CHECK (RMS)
// Dua kriteria: (1) ADC tidak saturasi, (2) RMS dalam batas wajar
// ============================================================
bool sqUpdate(float ecg_dc, int raw) {
  // Cek saturasi ADC (elektroda lepas atau short)
  if (raw<SQ_SAT_LOW||raw>SQ_SAT_HIGH) sq_sat_count++;
  else sq_sat_count=0;
  if (sq_sat_count>=SQ_SAT_COUNT_MAX) return false;

  // Running RMS via sum of squares (O(1))
  sq_sum_sq -= sq_buf[sq_buf_idx]*sq_buf[sq_buf_idx];
  sq_buf[sq_buf_idx]=ecg_dc;
  sq_sum_sq+=ecg_dc*ecg_dc;
  if (sq_sum_sq<0) sq_sum_sq=0;
  sq_buf_idx=(sq_buf_idx+1)%SQ_WIN_LEN;
  float rms=sqrtf(sq_sum_sq/SQ_WIN_LEN);
  return (rms>=SQ_RMS_MIN && rms<=SQ_RMS_MAX);
}

// ============================================================
// BAGIAN 29 — FUNGSI RESET: FILTER SAJA (sqFilterReset)
// Dipanggil saat elektroda berpindah/diganti
// Hanya reset filter & algoritma, TIDAK reset HR history
// ============================================================
void sqFilterReset() {
  SPKI=SPKI_INIT; NPKI=NPKI_INIT; THR=NPKI+THR_INIT_RATIO*(SPKI-NPKI);
  thr_init=false; refr_count=0;
  rp_env=0; rp_curr_amp=0; rp_curr_idx=0; rp_ready=false;
  edr_dc=0; out_lpf=0; norm_inited=false;
  rr_prev_val=0; rr_prev_prev=0; rr_last_peak=0; rr_peak_init=false;
  rr_bpm=0; rr_dyn_thresh=0.5f; rr_peak_max=0.5f; edr_out_last=0;
  memset(edr_var_buf,0,sizeof(edr_var_buf));
  edr_var_sum=0; edr_var_idx=0; breath_hold=false;
  hold_candidate=false; hold_candidate_ms=0; rr_valid_peak_count=0;
  memset(w_notch,0,sizeof(w_notch)); memset(w_hpf,0,sizeof(w_hpf));
  memset(w_bpf,0,sizeof(w_bpf));    memset(w_resp_hp,0,sizeof(w_resp_hp));
  memset(w_resp_lp,0,sizeof(w_resp_lp));
  memset(mwi_buf,0,sizeof(mwi_buf)); memset(derBuf,0,sizeof(derBuf));
  mwi_sum=0; mwi_idx=0; derIdx=0;
  memset(sq_buf,0,sizeof(sq_buf)); sq_sum_sq=0; sq_buf_idx=0; sq_sat_count=0;
  memset(twave_hist,0,sizeof(twave_hist)); twave_hist_idx=0; twave_hist_count=0;
  sq_valid_count=0;
}

// ============================================================
// BAGIAN 30 — FUNGSI RESET: TOTAL SYSTEM RESET (sqHardReset)
// Dipanggil saat sinyal hilang > SQ_LOST_MS (3.5 detik)
// Reset semua state termasuk HR history, phantom detector, display
// ============================================================
void sqHardReset() {
  sqFilterReset();
  last_r=0; hr_bpm=0; hr_instant=0;
  rri_buf_count=0; rri_buf_idx=0; rri_consistent_count=0; rri_consistent_val=0;
  for (int i=0;i<RRI_BUF_LEN;i++) rri_buf[i]=0.5f;
  for (int i=0;i<RR_MED_LEN;i++)  rr_med_buf[i]=16.0f;
  rr_display_frozen=0; rr_valid_peak_count=0;
  memset(jitter_rri_buf,0,sizeof(jitter_rri_buf));
  jitter_buf_idx=0; jitter_buf_count=0; rri_jitter=0; amd_is_phantom=false;
  jitter_low_now=false; jitter_low_since=0; jitter_hi_since=0;
  leadoff_rail_cnt=0; lead_off=false;
  phantom_off_since=0; phantom_off_armed=false; is_phantom_off=false;
  memset(elec_hist,0,sizeof(elec_hist));
  memset(elec_circ_buf,0,sizeof(elec_circ_buf));
  elec_circ_idx=0; elec_buf_full=false; elec_p10_prev=2048;
  hr_display=0; hr_display_frozen=0;
  rr_display=0; rr_display_frozen=0;
  resp_wave_frozen=RESP_BASELINE;
  display_was_lost=false; signal_was_invalid=false;
  last_rpeak_ms=millis(); sq_was_lost=true; sq_valid_count=0;
}

// ============================================================
// BAGIAN 31 — FUNGSI OUTPUT: UPDATE GELOMBANG RESPIRASI
// Mengubah EDR [-1,+1] → resp wave [10–200] untuk display master
// ============================================================
void updateRespWave() {
  // Sembunyikan gelombang saat sinyal phantom
  if (amd_is_phantom || is_phantom_off) {
    resp_wave_out=0;
    updateDataKirim(is_phantom_off?0:hr_display, 0, 0, 0);
    return;
  }
  float t=edr_out_last;
  if (t>1) t=1; if (t<-1) t=-1;
  // Mapping: baseline 50, amplitudo ±40
  int rw=RESP_BASELINE+(int)(t*RESP_AMPLITUDE);
  if (rw<RESP_OUT_MIN) rw=RESP_OUT_MIN;
  if (rw>RESP_OUT_MAX) rw=RESP_OUT_MAX;
  resp_wave_out=rw;
  resp_wave_frozen=rw; // Simpan untuk freeze saat signal hilang
  updateDataKirim(hr_display, rr_display, resp_wave_out, 0);
}

// ============================================================
// BAGIAN 32 — CALLBACK ESP-NOW: KONFIRMASI PENGIRIMAN
// ============================================================
void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  // Body kosong: callback wajib didaftarkan
  // Bisa diisi untuk logging statistik pengiriman jika diperlukan
}

// ============================================================
// BAGIAN 33 — TASK FREERTOS: PENGIRIMAN DATA ESP-NOW (50 Hz)
// Berjalan di Core 0, mengirim paket ke master setiap 20ms
// ============================================================
void taskESPNOW(void* pv) {
  const TickType_t interval=pdMS_TO_TICKS(20); // 50 Hz update rate
  TickType_t last=xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&last,interval); // Interval presisi (tidak terpengaruh waktu eksekusi)
    struct_message snap;
    // Ambil snapshot data dengan mutex (thread-safe)
    if (xSemaphoreTake(dataMutex,pdMS_TO_TICKS(5))==pdTRUE) {
      memcpy(&snap,&dataKirim,sizeof(snap)); xSemaphoreGive(dataMutex);
    } else continue;
    esp_now_send(receiverAddress,(uint8_t*)&snap,sizeof(snap));
  }
}

// ============================================================
// BAGIAN 34 — TASK FREERTOS: MONITOR PERUBAHAN ELEKTRODA (0.5 Hz)
// Berjalan di Core 0, cek histogram ADC setiap 2 detik
// ============================================================
void taskElecMonitor(void* pv) {
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(2000)); // Cek setiap 2 detik
    if (!elec_buf_full) continue;    // Tunggu hingga buffer histogram penuh
    elec_check_flag=true;            // Beri sinyal ke taskProcessing untuk evaluasi
  }
}

// ============================================================
// BAGIAN 35 — TASK FREERTOS: PEMROSESAN UTAMA (250 Hz)
// Berjalan di Core 1 Prioritas 2 — INTI SELURUH SISTEM
// ============================================================
void taskProcessing(void* pv) {
  uint32_t proc_n=0;
  static float ecg_mean=2048.0f; // Estimasi DC awal (tengah range ADC 12-bit)

  for (;;) {
    // Tunggu flag dari timer ISR (250 Hz)
    if (!samp_flag) { vTaskDelay(1); continue; }
    samp_flag=false;

    // ── 35.1: BACA ADC & SIMPAN KE CIRCULAR BUFFER ───────
    int raw=analogRead(ECG_PIN);
    proc_n++;
    portENTER_CRITICAL(&bufMux);
      adc_buf[adc_head]=raw; adc_head=(adc_head+1)&ADC_MASK; total_samp++;
    portEXIT_CRITICAL(&bufMux);

    // ── 35.2: DC REMOVAL ─────────────────────────────────
    // IIR low-pass untuk estimasi DC, lalu kurangi dari sinyal
    ecg_mean=DC_ALPHA_FAST*ecg_mean+DC_ALPHA_SEED*raw;
    float ecg=raw-ecg_mean;

    // ── 35.3: UPDATE HISTOGRAM ELEKTRODA (O(1)) ──────────
    int old_raw=elec_circ_buf[elec_circ_idx];
    if (elec_buf_full) histRemove(old_raw);
    elec_circ_buf[elec_circ_idx]=raw;
    histAdd(raw);
    elec_circ_idx=(elec_circ_idx+1)%ELEC_WIN_LEN;
    if (!elec_buf_full && elec_circ_idx==0) elec_buf_full=true;

    // ── 35.4: CEK PERUBAHAN ELEKTRODA (saat flag aktif) ──
    if (elec_check_flag) {
      elec_check_flag=false;
      int p10_now=histGetP10();
      int min_delta=abs(p10_now-elec_p10_prev);
      if (min_delta>ELEC_CHANGE_THRESH) {
        // P10 berubah drastis → elektroda berpindah → reset filter
        sqFilterReset(); ecg_mean=(float)p10_now+200.0f;
      }
      elec_p10_prev=p10_now;
    }

    // ── 35.5: DETEKSI LEAD-OFF (ELEKTRODA LEPAS) ─────────
    if (raw<LEADOFF_RAIL_LOW||raw>LEADOFF_RAIL_HIGH) {
      if (++leadoff_rail_cnt>=LEADOFF_RAIL_COUNT) lead_off=true;
    } else { leadoff_rail_cnt=0; lead_off=false; }

    // ── 35.6: CEK KUALITAS SINYAL (RMS) ──────────────────
    sq_signal_ok=sqUpdate(ecg,raw);
    float rms_now=sqrtf(sq_sum_sq/SQ_WIN_LEN);

    // ── 35.7: DETEKSI PHANTOM-OFF ────────────────────────
    // RMS di zona 150–270: bukan ECG valid tapi ada sinyal (elektroda tidak di tubuh)
    if (!sq_signal_ok && !amd_is_phantom &&
        rms_now>RMS_PHANTOM_OFF_LOW && rms_now<RMS_PHANTOM_OFF_HIGH) {
      if (!phantom_off_armed) { phantom_off_since=millis(); phantom_off_armed=true; }
      if ((millis()-phantom_off_since)>PHANTOM_OFF_CONFIRM_MS) is_phantom_off=true;
    } else { phantom_off_armed=false; is_phantom_off=false; }

    bool signal_invalid=lead_off||!sq_signal_ok;

    // ── 35.8: HANDLING SINYAL INVALID ────────────────────
    if (signal_invalid) {
      // Mulai timer signal loss
      if (!sq_was_lost) { sq_lost_since_ms=millis(); sq_was_lost=true; }
      sq_valid_count=0;
      // Hard reset jika signal hilang terlalu lama
      if ((millis()-sq_lost_since_ms)>SQ_LOST_MS) {
        sqHardReset(); sq_lost_since_ms=millis();
      }
      // Freeze HR/RR display selama DISPLAY_FREEZE_MS
      if (!display_was_lost) {
        display_lost_ms=millis(); display_was_lost=true;
        hr_display_frozen=hr_display;
      }
      uint32_t lost_dur=millis()-display_lost_ms;
      int disp_hr = (lost_dur<DISPLAY_FREEZE_MS) ? hr_display_frozen : 0;
      int disp_rr = (lost_dur<DISPLAY_FREEZE_MS) ? rr_display_frozen : 0;
      hr_display=disp_hr; rr_display=disp_rr;
      // Freeze resp wave selama RESP_FREEZE_MS
      if (!signal_was_invalid) { signal_lost_ms=millis(); signal_was_invalid=true; }
      int disp_resp = ((millis()-signal_lost_ms)<RESP_FREEZE_MS) ? resp_wave_frozen : 0;
      resp_wave_out=disp_resp;
      updateDataKirim(disp_hr, disp_rr, disp_resp, 1);
      DataSample ds={proc_n/(float)FS,(uint16_t)raw,(float)disp_hr,(float)disp_rr,0};
      xQueueSend(dataQueue,&ds,0);
      continue; // Lewati semua pemrosesan DSP
    }

    // Sinyal kembali valid
    display_was_lost=false;
    signal_was_invalid=false;

    // ── 35.9: HYSTERESIS VALID SIGNAL ────────────────────
    // Butuh 25 sampel valid berturut-turut sebelum aktifkan kembali
    if (sq_was_lost) {
      sq_valid_count++;
      if (sq_valid_count>=SQ_VALID_HYSTERESIS) { sq_was_lost=false; sq_valid_count=0; }
      else {
        DataSample ds={proc_n/(float)FS,(uint16_t)raw,0,0,0};
        xQueueSend(dataQueue,&ds,0); continue;
      }
    }

    // Reset phantom state jika buffer jitter belum penuh
    if (jitter_buf_count<JITTER_WIN_LEN) {
      amd_is_phantom=false; jitter_low_now=false;
      jitter_low_since=0; jitter_hi_since=0;
      is_phantom_off=false; phantom_off_armed=false;
    }
    updateDataKirim(dataKirim.HRx, dataKirim.RRx, dataKirim.respWave, 0);

    // Skip DSP jika phantom-off aktif
    if (is_phantom_off) {
      hr_display=0; rr_display=0; resp_wave_out=0;
      updateDataKirim(0,0,0,1);
      DataSample ds={proc_n/(float)FS,(uint16_t)raw,0,0,0};
      xQueueSend(dataQueue,&ds,0); continue;
    }

    // ── 35.10: PAN-TOMPKINS — RANTAI FILTER ECG ──────────
    // Notch 50Hz → HPF → BPF → Derivatif 5-titik → Kuadrat → MWI
    float fn=sosBiquad(ecg,notch_sos,NOTCH_STAGES,notch_gain,w_notch); // Hapus PLI 50Hz
    float fh=sosBiquad(fn, hpf_sos,  HPF_STAGES,  hpf_gain,  w_hpf);  // Hapus baseline wander
    float bp=sosBiquad(fh, bpf_sos,  BPF_STAGES,  bpf_gain,  w_bpf);  // Isolasi QRS
    float d=deriv5pt(bp);   // Slope sinyal → tinggi saat QRS
    float sq2=d*d;          // Kuadrat: semua positif, perkuat puncak tajam
    // MWI lebih pendek saat HR > 150 bpm agar responsif
    float mwi_scale=(hr_bpm>MWI_HR_FAST_THRESH)?((float)MWI_LEN/(float)MWI_LEN_FAST):1.0f;
    float mwi=mwiUpdate(sq2)*mwi_scale; // Smoothing energi selama 112ms

    // ── 35.11: INISIALISASI THRESHOLD ADAPTIF ────────────
    if (!thr_init && proc_n>MWI_LEN+20) {
      SPKI=mwi*THR_INIT_RATIO; NPKI=mwi*(THR_INIT_RATIO*0.2f);
      THR=NPKI+THR_INIT_RATIO*(SPKI-NPKI); thr_init=true;
    }

    bool r_detected=false;

    // ── 35.12: REFRACTORY PERIOD & DETEKSI R-PEAK ────────
    if (refr_count>0) {
      refr_count--;
      // Selama refractory: update SPKI cepat tapi tidak deteksi R-peak
      if (thr_init&&mwi>SPKI) SPKI=SPKI_ALPHA_FAST*mwi+(1.0f-SPKI_ALPHA_FAST)*SPKI;
    } else if (thr_init&&mwi>=THR) {

      // ── 35.13: T-WAVE REJECTION ──────────────────────
      bool is_twave=false;
      if (last_r>0&&rp_ready) {
        float gap=(float)(uint32_t)(proc_n-last_r);
        if (gap<TWAVE_GAP_MAX) { // Dalam 360ms setelah R-peak
          float ca=fabsf(bp)*(1.0f/bpf_gain);
          float ref_amp=twaveGetMeanAmp();
          float t_thresh=(ref_amp>0)?(TWAVE_AMP_RATIO*ref_amp):(TWAVE_AMP_RATIO*rp_curr_amp);
          if (ca<t_thresh) { // Amplitudo < 55% R-peak = T-wave
            is_twave=true;
            NPKI=NPKI_ALPHA*mwi+(1.0f-NPKI_ALPHA)*NPKI;
            THR=NPKI+THR_RUN_RATIO*(SPKI-NPKI);
          }
        }
      }

      if (!is_twave) {
        // ── 35.14: KALKULASI RRI & UPDATE HR ───────────
        if (last_r>0) {
          float rri_raw=(float)(uint32_t)(proc_n-last_r)/(float)FS;
          if (rri_raw>=RRI_MIN_S&&rri_raw<=RRI_MAX_S) {
            jitterUpdate(rri_raw); // Update phantom detector
            rri_buf[rri_buf_idx%RRI_BUF_LEN]=rri_raw; rri_buf_idx++;
            if (rri_buf_count<RRI_BUF_LEN) rri_buf_count++;
            int n_med=(rri_buf_count<RRI_BUF_LEN)?rri_buf_count:RRI_BUF_LEN;
            float rri_med=rriMedian(rri_buf,n_med);
            float hr_med=60.0f/rri_med;
            // Outlier rejection: abaikan RRI yang terlalu jauh dari median
            bool is_outlier=(rri_buf_count>=3)&&
              (rri_raw>RRI_OUTLIER_HIGH*rri_med||rri_raw<RRI_OUTLIER_LOW*rri_med);
            if (!is_outlier) {
              hr_instant=60.0f/rri_raw;
              float delta=fabsf(hr_med-hr_bpm);
              if (delta>RRI_JUMP_THRESH) {
                // HR lompat besar: butuh konfirmasi 2 RRI konsisten
                if (fabsf(rri_raw-rri_consistent_val)<RRI_CONSISTENT_TOL) rri_consistent_count++;
                else { rri_consistent_count=1; rri_consistent_val=rri_raw; }
                if (rri_consistent_count>=RRI_CONSISTENT_NEED) {
                  hr_bpm=hr_med; rri_consistent_count=0;
                  for (int i=0;i<RRI_BUF_LEN;i++) rri_buf[i]=rri_raw; // Reset buffer
                } else {
                  // Belum konfirmasi: adaptasi parsial
                  float a=(delta>80)?HR_ALPHA_JUMP_XL:(delta>40)?HR_ALPHA_JUMP_L:
                          (delta>20)?HR_ALPHA_JUMP_M:HR_ALPHA_JUMP_S;
                  if (hr_bpm<1) hr_bpm=hr_med; else hr_bpm=a*hr_med+(1.0f-a)*hr_bpm;
                }
              } else {
                // HR stabil: update EMA normal
                rri_consistent_count=0;
                float a=(delta>10)?HR_ALPHA_FAST:HR_ALPHA_SLOW;
                if (hr_bpm<1) hr_bpm=hr_med; else hr_bpm=a*hr_med+(1.0f-a)*hr_bpm;
              }
            }
          }
        }
        // Catat R-peak
        last_r=proc_n;
        portENTER_CRITICAL(&bufMux);
          r_idx[r_count%MAX_R]=proc_n; r_count++;
        portEXIT_CRITICAL(&bufMux);
        SPKI=SPKI_ALPHA*mwi+(1.0f-SPKI_ALPHA)*SPKI;
        THR=NPKI+THR_RUN_RATIO*(SPKI-NPKI);
        refr_count=adaptiveRefr(hr_bpm);
        last_rpeak_ms=millis(); r_detected=true;
      }
    } else if (thr_init) {
      // Di bawah threshold: update NPKI
      NPKI=NPKI_ALPHA*mwi+(1.0f-NPKI_ALPHA)*NPKI;
      THR=NPKI+THR_RUN_RATIO*(SPKI-NPKI);
    }

    // ── 35.15: EDR-AM — EKSTRAKSI SINYAL NAPAS DARI ECG ──
    // Modulasi amplitudo R-peak mengikuti siklus pernapasan
    if (r_detected) {
      float na=fabsf(bp)*(1.0f/bpf_gain); // Normalisasi amplitudo R-peak
      twave_hist[twave_hist_idx%TWAVE_HIST_LEN]=na; twave_hist_idx++;
      if (twave_hist_count<TWAVE_HIST_LEN) twave_hist_count++;
      rp_curr_amp=na; rp_curr_idx=proc_n; rp_ready=true;
    }
    // Envelope tracker asimetris: naik cepat, turun lambat
    if (rp_ready) {
      float tgt=rp_curr_amp;
      rp_env=(tgt>rp_env)?(ENV_ATTACK*tgt+(1.0f-ENV_ATTACK)*rp_env)
                         :(ENV_DECAY*tgt+(1.0f-ENV_DECAY)*rp_env);
    }
    // Filter envelope → isolasi komponen napas
    float env_hp=sosBiquad(rp_env*ENV_SCALE,resp_hp_sos,RESP_HP_STAGES,resp_hp_gain,w_resp_hp);
    float env_lp=sosBiquad(env_hp,resp_lp_sos,RESP_LP_STAGES,1.0f,w_resp_lp);
    float edr_bp=env_lp*EDR_SCALE;
    // DC removal dari sinyal EDR
    edr_dc=EDR_DC_ALPHA*edr_dc+EDR_DC_BETA*edr_bp;
    float edr_zc=edr_bp-edr_dc;
    // Normalisasi adaptif ke [-1, +1]
    if (!norm_inited) { norm_min=edr_zc-0.5f; norm_max=edr_zc+0.5f; norm_inited=true; }
    float cur_range=norm_max-norm_min;
    float norm_decay=(cur_range<0.5f)?NORM_DECAY_FAST:(cur_range<2.0f)?NORM_DECAY_MED:NORM_DECAY_SLOW;
    norm_max=(edr_zc>norm_max)?(NORM_ATTACK*edr_zc+(1.0f-NORM_ATTACK)*norm_max)
                               :(norm_decay*edr_zc+(1.0f-norm_decay)*norm_max);
    norm_min=(edr_zc<norm_min)?(NORM_ATTACK*edr_zc+(1.0f-NORM_ATTACK)*norm_min)
                               :(norm_decay*edr_zc+(1.0f-norm_decay)*norm_min);
    float range=norm_max-norm_min;
    if (range<NORM_RANGE_MIN) range=NORM_RANGE_MIN;
    float mid=0.5f*(norm_max+norm_min);
    float edr_n=(edr_zc-mid)/(0.5f*range);
    if (edr_n>1) edr_n=1; if (edr_n<-1) edr_n=-1;
    // Deadzone adaptif: reduksi noise kecil
    float nkt=(range<0.5f)?NORM_DEADZONE_SMALL:(range<2.0f)?NORM_DEADZONE_MED:NORM_DEADZONE_LARGE;
    if (fabsf(edr_n)<nkt) edr_n=0;
    out_lpf=EDR_LPF_ALPHA*out_lpf+(1.0f-EDR_LPF_ALPHA)*edr_n;
    edr_out_last=out_lpf;

    // ── 35.16: DETEKSI BREATH HOLD (NAFAS BERHENTI) ──────
    // Running variance sinyal EDR via circular buffer
    edr_var_sum-=edr_var_buf[edr_var_idx];
    edr_var_buf[edr_var_idx]=edr_out_last*edr_out_last;
    edr_var_sum+=edr_var_buf[edr_var_idx];
    edr_var_idx=(edr_var_idx+1)%HOLD_WIN_LEN;
    float edr_var_mean=edr_var_sum/HOLD_WIN_LEN;

    bool var_low=(edr_var_mean<BREATH_HOLD_THRESH);
    // Hanya aktifkan setelah ada minimal 4 peak napas valid
    if (var_low && rr_valid_peak_count>=RR_MIN_PEAKS_FOR_HOLD) {
      if (!hold_candidate) { hold_candidate=true; hold_candidate_ms=millis(); }
      // Konfirmasi setelah low-variance berlangsung 2 detik terus
      if ((millis()-hold_candidate_ms)>=BREATH_HOLD_CONFIRM_MS) breath_hold=true;
    } else {
      hold_candidate=false;
      breath_hold=false; // Langsung clear saat variasi napas kembali
    }

    // ── 35.17: DETEKSI LAJU NAPAS (RR DARI EDR) ──────────
    // Cari puncak lokal pada sinyal EDR dengan threshold dinamis
    {
      float ev=edr_out_last;
      // Kondisi puncak lokal: sebelumnya naik, sekarang turun, melewati threshold
      if (rr_prev_prev<rr_prev_val && rr_prev_val>ev && rr_prev_val>rr_dyn_thresh) {
        uint32_t pk=proc_n-1;
        if (rr_peak_init) {
          float period=(float)(uint32_t)(pk-rr_last_peak)/(float)FS;
          if (period>=RR_PERIOD_MIN&&period<=RR_PERIOD_MAX) {
            float rr_new=60.0f/period; // Konversi periode → napas/menit
            rr_med_buf[rr_med_idx%RR_MED_LEN]=rr_new; rr_med_idx++;
            float rr_med=rr_median(); // Median 7 nilai untuk kestabilan
            float rr_delta=fabsf(rr_med-rr_bpm);
            // EMA multi-rate: adaptasi cepat saat lompatan besar
            float arr;
            if      (rr_bpm<1)                   { rr_bpm=rr_med; arr=1; }
            else if (rr_delta>RR_DELTA_JUMP_XL)    arr=RR_ALPHA_JUMP_XL;
            else if (rr_delta>RR_DELTA_JUMP_L)     arr=RR_ALPHA_JUMP_L;
            else if (rr_delta>RR_DELTA_JUMP_M)     arr=RR_ALPHA_JUMP_M;
            else                                    arr=RR_ALPHA_SLOW;
            if (arr<1) rr_bpm=arr*rr_med+(1.0f-arr)*rr_bpm;
            // Hitung peak respirasi valid untuk guard breath hold
            if (rr_valid_peak_count<RR_MIN_PEAKS_FOR_HOLD+2) rr_valid_peak_count++;
          }
        }
        rr_last_peak=pk; rr_peak_init=true;
        // Update threshold dinamis berdasarkan puncak tertinggi
        rr_peak_max=(rr_prev_val>rr_peak_max)?rr_prev_val:(0.998f*rr_peak_max+0.002f*rr_prev_val);
        rr_dyn_thresh=RR_THRESH_RATIO*rr_peak_max;
        if (rr_dyn_thresh<RR_THRESH_MIN) rr_dyn_thresh=RR_THRESH_MIN;
      }
      rr_prev_prev=rr_prev_val; rr_prev_val=ev;
    }

    // ── 35.18: DISPLAY HR & RR ────────────────────────────
    if ((millis()-last_rpeak_ms)>NO_RPEAK_RESET_MS) {
      // Timeout: tidak ada R-peak selama 15 detik → reset HR
      hr_bpm=0; hr_instant=0; hr_display=0; rr_display=0;
      rri_buf_count=0; rri_buf_idx=0; rri_consistent_count=0;
      last_rpeak_ms=millis();
    } else {
      // Update HR display dengan outlier rejection
      if (hr_bpm>=HR_MIN_VALID&&hr_bpm<=HR_MAX_VALID) {
        if (rri_buf_count>=3) {
          int nd=(rri_buf_count<RRI_BUF_LEN)?rri_buf_count:RRI_BUF_LEN;
          float sr=0; for (int i=0;i<nd;i++) sr+=rri_buf[i];
          float ha=60.0f/(sr/nd), hm=60.0f/rriMedian(rri_buf,nd);
          bool out=(fabsf(ha-hm)>HR_OUTLIER_BPM)||(fabsf(hr_bpm-(float)hr_display)>HR_JUMP_DISPLAY);
          float hv=out?hr_bpm:ha;
          if (hv>=HR_MIN_VALID&&hv<=HR_MAX_VALID) hr_display=(int)(hv+0.5f);
        } else hr_display=(int)(hr_bpm+0.5f);
      }
      // Update RR display berdasarkan kondisi
      if (amd_is_phantom) {
        // Phantom: sembunyikan RR dan reset
        rr_display=0; rr_display_frozen=0; rr_bpm=0; rr_peak_init=false;
        for (int i=0;i<RR_MED_LEN;i++) rr_med_buf[i]=16.0f;
      } else if (!breath_hold) {
        // Normal: tampilkan RR jika valid
        if (rr_bpm>=RR_MIN_VALID&&rr_bpm<=RR_MAX_VALID) {
          rr_display=(int)(rr_bpm+0.5f); rr_display_frozen=rr_display;
        }
      } else {
        // Breath hold aktif: tampilkan 0
        rr_display = 0;
      }
    }

    // ── 35.19: OUTPUT GELOMBANG RESPIRASI ─────────────────
    if (breath_hold) {
      // Saat tahan napas: flat di baseline, RR = 0
      resp_wave_out = RESP_BASELINE;
      resp_wave_frozen = RESP_BASELINE;
      updateDataKirim(hr_display, 0, RESP_BASELINE, 0);
    } else {
      updateRespWave(); // Normal: konversi EDR ke gelombang resp
    }

    // ── 35.20: KIRIM DATA KE ANTRIAN SERIAL OUTPUT ────────
    DataSample ds;
    ds.time_s=proc_n/(float)FS; ds.lead2=(uint16_t)raw;
    ds.hr=(float)hr_display; ds.rr=(float)rr_display;
    ds.edr=(amd_is_phantom||is_phantom_off)?0.0f:(float)edr_out_last;
    xQueueSend(dataQueue,&ds,0);
  }
}

// ============================================================
// BAGIAN 36 — SETUP: INISIALISASI SELURUH SISTEM
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("=== BOOT START ===");
  Serial.flush();

  // 36.1: Buat FreeRTOS Queue & Mutex
  dataQueue = xQueueCreate(ECG_QUEUE_LEN, sizeof(DataSample));
  if (!dataQueue) { Serial.println("FATAL: Queue fail"); while(1); }
  dataMutex = xSemaphoreCreateMutex();
  if (!dataMutex) { Serial.println("FATAL: Mutex fail"); while(1); }
  Serial.println("Queue & Mutex OK");

  // 36.2: Konfigurasi ADC
  analogReadResolution(12);         // 12-bit → 0-4095
  analogSetAttenuation(ADC_11db);   // Range 0–3.9V (cukup untuk sinyal ECG yang di-bias)

  // 36.3: Reset semua buffer & state ke kondisi awal
  memset(w_notch,   0, sizeof(w_notch));
  memset(w_hpf,     0, sizeof(w_hpf));
  memset(w_bpf,     0, sizeof(w_bpf));
  memset(w_resp_hp, 0, sizeof(w_resp_hp));
  memset(w_resp_lp, 0, sizeof(w_resp_lp));
  memset(mwi_buf,   0, sizeof(mwi_buf));
  memset(derBuf,    0, sizeof(derBuf));
  memset(sq_buf,    0, sizeof(sq_buf));
  memset(edr_var_buf,    0, sizeof(edr_var_buf));
  memset(jitter_rri_buf, 0, sizeof(jitter_rri_buf));
  memset(twave_hist,     0, sizeof(twave_hist));
  memset(elec_hist,      0, sizeof(elec_hist));
  memset(elec_circ_buf,  0, sizeof(elec_circ_buf));
  memset(&dataKirim,     0, sizeof(dataKirim));

  // 36.4: Inisialisasi nilai default buffer
  for (int i = 0; i < RRI_BUF_LEN; i++) rri_buf[i]    = 0.5f;  // RRI default = 0.5s = 120bpm
  for (int i = 0; i < RR_MED_LEN;  i++) rr_med_buf[i] = 16.0f; // RR default = 16 napas/menit

  // 36.5: Inisialisasi threshold Pan-Tompkins
  SPKI = SPKI_INIT;
  NPKI = NPKI_INIT;
  THR  = NPKI + THR_INIT_RATIO * (SPKI - NPKI);
  last_rpeak_ms    = millis();
  sq_lost_since_ms = millis();
  resp_wave_frozen = RESP_BASELINE;
  Serial.println("Variables OK");

  // 36.6: Setup Timer Hardware 250 Hz
  sampTimer = timerBegin(1000000);         // Base clock 1 MHz
  if (!sampTimer) { Serial.println("FATAL: timerBegin fail"); while(1); }
  timerAttachInterrupt(sampTimer, &onSampTimer);
  timerAlarm(sampTimer, 4000, true, 0);    // 1,000,000 / 4000 = 250 Hz
  Serial.println("Timer OK");

  // 36.7: Inisialisasi WiFi & ESP-NOW
  WiFi.mode(WIFI_STA);
  esp_wifi_set_ps(WIFI_PS_NONE); // Matikan power saving untuk latensi minimal
  Serial.print("MAC: ");
  Serial.println(WiFi.macAddress());
  if (esp_now_init() != ESP_OK) { Serial.println("FATAL: ESP-NOW init fail"); while(1); }
  Serial.println("ESP-NOW init OK");
  esp_now_register_send_cb(OnDataSent);

  // 36.8: Daftarkan peer (master)
  esp_now_peer_info_t pi = {};
  memcpy(pi.peer_addr, receiverAddress, 6);
  pi.channel = 0;
  pi.encrypt = false;
  if (esp_now_add_peer(&pi) != ESP_OK) { Serial.println("FATAL: Add peer fail"); while(1); }
  Serial.println("ESP-NOW peer OK");

  Serial.println("ECG Monitor v10.1 — stable body signal + breath hold fix");
  Serial.println("Format: time,lead2,hr,rr,edr");

  // 36.9: Buat FreeRTOS Tasks
  // Core 1 Prioritas 2: pemrosesan real-time 250Hz (stack besar 16KB)
  xTaskCreatePinnedToCore(taskProcessing,  "Processing",  16384, NULL, 2, NULL, 1);
  // Core 0 Prioritas 1: pengiriman ESP-NOW 50Hz
  xTaskCreatePinnedToCore(taskESPNOW,      "ESPNOW",      4096,  NULL, 1, NULL, 0);
  // Core 0 Prioritas 1: monitor elektroda 0.5Hz
  xTaskCreatePinnedToCore(taskElecMonitor, "ElecMon",     2048,  NULL, 1, NULL, 0);
  Serial.println("All tasks created — entering loop()");
}

// ============================================================
// BAGIAN 37 — LOOP: OUTPUT SERIAL CSV
// Berjalan di Core 0 — hanya baca queue & cetak ke Serial
// Format: waktu(s), raw_ADC, HR(bpm), RR(napas/min), EDR[-1,+1]
// ============================================================
void loop() {
  DataSample ds;
  int count = 0;
  while (xQueueReceive(dataQueue, &ds, 0) == pdTRUE) {
    Serial.printf("%.3f,%d,%.0f,%.0f,%.6f\n",
      ds.time_s,      // Waktu dalam detik sejak boot
      (int)ds.lead2,  // Nilai ADC raw (0–4095)
      ds.hr,          // Heart Rate (bpm)
      ds.rr,          // Respiration Rate (napas/menit)
      ds.edr);        // EDR ternormalisasi [-1,+1]
    // yield() setiap 20 sampel: beri waktu WiFi stack & WDT agar tidak reset
    if (++count >= 20) { yield(); count = 0; }
  }
  delay(1); // Beri waktu idle task & Watchdog Timer
}