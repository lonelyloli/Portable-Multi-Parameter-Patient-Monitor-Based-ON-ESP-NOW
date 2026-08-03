// ============================================================
//  ESP32-S3 KOROTKOFF V4
//  Peran: Slave perantara antara NIBP dan Master
//  Tugas utama:
//    1. Terima data NIBP via UART2 (Serial2)
//    2. Deteksi bunyi Korotkoff via ADC GPIO4 (mikrofon/piezo)
//    3. Kirim semua data ke Master via ESP-NOW
//    4. Kelola countdown timer pengukuran otomatis 10 menit
// ============================================================

#include <WiFi.h>
#include <esp_now.h>
#include <freertos/FreeRTOS.h>   // FreeRTOS kernel
#include <freertos/task.h>        // API task (xTaskCreatePinnedToCore)
#include <freertos/queue.h>       // API queue (xQueueCreate, xQueueSend, xQueueReceive)

// ── Pin UART2 — komunikasi dengan slave NIBP ──
#define UART_TX_PIN  18   // GPIO18 = TX ke NIBP (NIBP.ino = RX GPIO16)
#define UART_RX_PIN  8    // GPIO8  = RX dari NIBP (NIBP.ino = TX GPIO17)
#define UART_BAUD    115200

// MAC address ESP32 Master — tujuan pengiriman ESP-NOW
uint8_t masterAddress[] = {0x28, 0x37, 0x2F, 0x86, 0xB1, 0xA8};

// ── Struct data yang dikirim ke Master (sama persis dengan struct di Master) ──
// Harus identik byte-per-byte agar memcpy/ESP-NOW bisa decode dengan benar
typedef struct struct_message {
  int SYSx;          // Tekanan sistolik hasil akhir
  int DIAx;          // Tekanan diastolik hasil akhir
  int Tempx;         // Suhu (tidak dipakai di slave ini, diisi 0)
  int SpO2x;         // SpO2 (tidak dipakai di slave ini, diisi 0)
  int HRx;           // HR (tidak dipakai, diisi 0)
  int RRx;           // RR (tidak dipakai, diisi 0)
  int ecgLead1;      // (tidak dipakai, diisi 0)
  int ecgLead2;      // (tidak dipakai, diisi 0)
  int ecgLead3;      // (tidak dipakai, diisi 0)
  int spo2Wave;      // (tidak dipakai, diisi 0)
  int respWave;      // (tidak dipakai, diisi 0)
  int leadOff;       // (tidak dipakai, diisi 0)
  int tekananRT;     // Tekanan real-time manset saat ini (mmHg)
  int faseNIBP;      // Fase pengukuran: 0=wait,1=reset,2=inflate,3=deflate,4=sistole,5=diastole,6=hasil,7=timer
  int cntMenit;      // Sisa menit countdown otomatis
  int battPct;       // (tidak dipakai, diisi 0)
  int korotkoffBeat; // Flag: 1 = bunyi Korotkoff baru terdeteksi
} struct_message;

struct_message dataKirim; // Buffer global untuk data yang akan dikirim ke Master

// ── Struct hasil parsing pesan UART dari NIBP ──
typedef struct {
  int  tek;       // Nilai tekanan yang diparsing dari field "TEK:"
  int  mrk;       // Nilai mark yang diparsing dari field "MRK:"
  char fase[12];  // Label fase sebagai string: "INFLATE","DEFLASI","SISTOLE","DIASTOLE","RESET"
  bool isHasil;   // true  = ini pesan HASIL (sistolik/diastolik final)
                   // false = ini pesan FASE (update tekanan real-time)
  int  sys;       // Nilai sistolik dari pesan HASIL
  int  dia;       // Nilai diastolik dari pesan HASIL
} ParsedData;

// Queue untuk passing ParsedData dari uartRxTask (Core 1) ke espnowTask (Core 0)
// Kapasitas 30 item — mencegah data hilang jika espnowTask sedang sibuk kirim ESP-NOW
QueueHandle_t parsedQueue;

// ── Countdown timer variabel ──
volatile bool countdownActive = false; // true = countdown 10 menit sedang berjalan
                                        // volatile: diakses dari beberapa task
volatile int  cntMenitSisa    = 10;    // Sisa menit countdown (mulai dari 10)
unsigned long lastCntTick     = 0;     // Timestamp tick menit terakhir (millis())

// Simpan hasil terakhir sistolik/diastolik — untuk dikirim ulang saat countdown/beat
int lastSys = 0; // Sistolik terakhir yang diterima dari NIBP
int lastDia = 0; // Diastolik terakhir yang diterima dari NIBP

// ── Variabel shared antar task (volatile) ──
volatile int   g_tek      = 0;     // Tekanan saat ini (dari uartRxTask, dibaca espnowTask)
volatile int   g_mrk      = 0;     // Mark saat ini
volatile float g_filter   = 0.0f;  // Nilai filtered sinyal Korotkoff (dari adcTask, untuk log)
volatile float g_envelope = 0.0f;  // Nilai envelope sinyal Korotkoff (dari adcTask, untuk log)
volatile bool  g_beat     = false; // Flag: true = beat Korotkoff baru terdeteksi oleh adcTask
                                    // Di-reset oleh espnowTask setelah dikirim ke Master
volatile bool  g_deflasi  = false; // true = fase saat ini adalah DEFLASI
                                    // Beat Korotkoff hanya valid saat deflasi

volatile bool  sensorTerpasang = false; // true = sensor Korotkoff terpasang dan valid
unsigned long  lastSensorCheck = 0;     // Timestamp cek sensor terakhir

// ============================================================
// CALLBACK ESP-NOW: OnDataSent
// Dipanggil setelah esp_now_send() selesai (sukses/gagal)
// Kosong — tidak perlu aksi apapun saat ini
// Signature berubah di ESP32 Arduino v3.x (pakai wifi_tx_info_t)
// ============================================================
void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {}

// ============================================================
// CALLBACK ESP-NOW: OnDataRecv — ISR
// Dipanggil saat ada perintah masuk dari Master via ESP-NOW
// Dua perintah yang dikenal:
//   SYSx == 1  → START: teruskan ke NIBP via Serial2
//   SYSx == 99 → STOP:  teruskan ke NIBP, kirim status reset ke Master
// ============================================================
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  if (len < (int)sizeof(struct_message)) return; // Tolak paket korup/terlalu kecil

  const uint8_t *mac_addr = info->src_addr; // MAC address pengirim (Master)

  struct_message temp;
  memcpy(&temp, incomingData, sizeof(temp)); // Salin raw bytes ke struct

  if (temp.SYSx == 1) {
    // Perintah START dari Master
    if (countdownActive) {
      countdownActive = false; // Hentikan countdown yang sedang berjalan
      cntMenitSisa    = 10;    // Reset ke 10 menit
      Serial.println("[COUNTDOWN] Dihentikan — perintah START dari Master");
    }
    Serial2.println("CMD:START"); // Teruskan perintah START ke NIBP via UART
    Serial.println("[ESP-NOW RECV] START dari Master → diteruskan ke NIBP");
  }

  if (temp.SYSx == 99) {
    // Perintah STOP dari Master
    if (countdownActive) {
      countdownActive = false;
      cntMenitSisa    = 10;
      Serial.println("[COUNTDOWN] Dihentikan — perintah STOP dari Master");
    }
    Serial2.println("CMD:STOP"); // Teruskan perintah STOP ke NIBP via UART
    kirimKemaster(0, 0, 0, 0, 0); // Kirim status reset ke Master (fase=0=waiting)
    Serial.println("[ESP-NOW RECV] STOP dari Master → diteruskan ke NIBP");
  }
}

// ============================================================
// FUNGSI: kirimKemaster
// Bungkus pengiriman ESP-NOW ke Master
// Isi struct dataKirim lalu kirim via esp_now_send
// Parameter:
//   fase  = kode fase NIBP (0-7)
//   tekRT = tekanan real-time manset
//   sys   = sistolik (hasil akhir atau lastSys)
//   dia   = diastolik (hasil akhir atau lastDia)
//   menit = sisa menit countdown
// ============================================================
void kirimKemaster(int fase, int tekRT, int sys, int dia, int menit) {
  memset(&dataKirim, 0, sizeof(dataKirim)); // Nolkan semua field — cegah nilai sampah
  dataKirim.faseNIBP      = fase;
  dataKirim.tekananRT     = tekRT;
  dataKirim.SYSx          = sys;
  dataKirim.DIAx          = dia;
  dataKirim.cntMenit      = menit;
  dataKirim.korotkoffBeat = 0; // Selalu 0 di sini — beat dikirim langsung dari espnowTask
  esp_now_send(masterAddress, (uint8_t*)&dataKirim, sizeof(dataKirim));
}

// ============================================================
// TASK: uartRxTask — berjalan di Core 1, prioritas 2
// Baca dan parse pesan dari NIBP via Serial2
// Tiga format pesan dari NIBP:
//   1. "NOTIF:START"              → NIBP mulai sesi baru
//   2. "HASIL:120|80"             → hasil akhir sistolik|diastolik
//   3. "FASE:INFLATE|TEK:100|MRK:0" → update real-time tekanan & fase
// ============================================================
void uartRxTask(void *pvParameters) {
  String buf = ""; // Buffer akumulasi karakter per baris

  for (;;) {
    while (Serial2.available()) {
      char c = (char)Serial2.read();

      if (c == '\n') { // Newline = akhir satu pesan
        buf.trim();

        if (buf.length() > 0) {
          ParsedData pd;
          memset(&pd, 0, sizeof(pd)); // Bersihkan struct sebelum diisi

          if (buf == "NOTIF:START") {
            // NIBP memberi tahu sesi pengukuran baru dimulai
            // Reset countdown jika sedang berjalan
            if (countdownActive) {
              countdownActive = false;
              cntMenitSisa    = 10;
              Serial.println("[COUNTDOWN] Dihentikan — NOTIF:START dari NIBP");
            }
            kirimKemaster(0, 0, 0, 0, 0); // Kirim fase 0 (waiting) ke Master

          } else if (buf.startsWith("HASIL:")) {
            // Format: "HASIL:120|80"
            pd.isHasil = true;
            int sep = buf.indexOf('|', 6); // Cari '|' mulai dari posisi 6 (setelah "HASIL:")
            if (sep > 0) {
              pd.sys = buf.substring(6, sep).toInt();       // "120"
              pd.dia = buf.substring(sep + 1).toInt();      // "80"
            }
            Serial.println("[UART] HASIL SYS:" + String(pd.sys) + " DIA:" + String(pd.dia));
            xQueueSend(parsedQueue, &pd, 0); // Push ke queue → espnowTask proses

          } else if (buf.startsWith("FASE:")) {
            // Format: "FASE:INFLATE|TEK:100|MRK:0"
            pd.isHasil = false;

            // Parsing label fase (antara "FASE:" dan "|" pertama)
            int p1 = buf.indexOf('|');
            if (p1 > 0) {
              String faseStr = buf.substring(5, p1); // Ambil "INFLATE"
              strncpy(pd.fase, faseStr.c_str(), sizeof(pd.fase) - 1); // Salin ke char array
            }

            // Parsing nilai TEK (tekanan)
            int p2 = buf.indexOf("TEK:");  // Cari posisi "TEK:"
            int p3 = buf.indexOf('|', p2); // Cari '|' setelah "TEK:"
            if (p2 >= 0) {
              pd.tek = (p3 > p2)
                       ? buf.substring(p2 + 4, p3).toInt() // Ada '|' setelah → ambil sampai '|'
                       : buf.substring(p2 + 4).toInt();     // Tidak ada '|' → ambil sampai akhir
            }

            // Parsing nilai MRK
            int p4 = buf.indexOf("MRK:");
            if (p4 >= 0) {
              pd.mrk = buf.substring(p4 + 4).toInt(); // Ambil angka setelah "MRK:"
            }

            // Update variabel global — dibaca oleh adcTask dan espnowTask
            g_tek = pd.tek;
            g_mrk = pd.mrk;

            // Cek apakah fase saat ini adalah DEFLASI
            // Beat Korotkoff hanya valid dan relevan saat fase deflasi
            String faseCheck = buf.substring(5, buf.indexOf('|'));
            g_deflasi = (faseCheck == "DEFLASI"); // true/false

            xQueueSend(parsedQueue, &pd, 0); // Push ke queue
          }
        }
        buf = ""; // Reset buffer untuk baris berikutnya

      } else {
        buf += c; // Akumulasi karakter
      }
    }
    vTaskDelay(pdMS_TO_TICKS(5)); // Yield 5ms — beri waktu task lain
  }
}

// ============================================================
// TASK: espnowTask — berjalan di Core 0, prioritas 1
// Dua tanggung jawab:
//   1. Kirim notifikasi beat Korotkoff ke Master (dari g_beat)
//   2. Ambil ParsedData dari queue → konversi fase → kirim ke Master
//
// Berjalan di Core 0 karena esp_now_send() sebaiknya di Core 0
// (WiFi stack berjalan di Core 0 pada ESP32)
// ============================================================
void espnowTask(void *pvParameters) {
  ParsedData pd;

  for (;;) {
    // ── Bagian 1: Cek flag beat Korotkoff ──
    if (g_beat) {
      g_beat = false; // Konsumsi flag segera agar tidak double-kirim

      // Beat hanya dikirim jika:
      //   sensorTerpasang = sensor fisik valid (tidak lepas)
      //   g_deflasi       = fase saat ini adalah deflasi
      //   !countdownActive = bukan sedang countdown (tidak sedang ukur)
      if (sensorTerpasang && g_deflasi && !countdownActive) {
        memset(&dataKirim, 0, sizeof(dataKirim));
        dataKirim.faseNIBP      = 3;          // Fase 3 = DEFLASI (beat terjadi saat deflasi)
        dataKirim.tekananRT     = g_tek;       // Tekanan manset saat beat terdeteksi
        dataKirim.SYSx          = lastSys;     // Sistolik terakhir (untuk context)
        dataKirim.DIAx          = lastDia;     // Diastolik terakhir
        dataKirim.cntMenit      = cntMenitSisa;
        dataKirim.korotkoffBeat = 1;           // FLAG UTAMA: 1 = ada beat Korotkoff
        esp_now_send(masterAddress, (uint8_t*)&dataKirim, sizeof(dataKirim));
        // Master akan tampilkan "*" di indikator NIBP saat terima korotkoffBeat=1
      }
    }

    // ── Bagian 2: Proses data dari queue (timeout 10ms) ──
    if (xQueueReceive(parsedQueue, &pd, pdMS_TO_TICKS(10)) == pdTRUE) {

      if (pd.isHasil) {
        // ── Terima HASIL AKHIR sistolik/diastolik ──
        if (countdownActive) {
          // Jika countdown lama masih jalan → reset dulu
          countdownActive = false;
          cntMenitSisa    = 10;
          Serial.println("[COUNTDOWN] Direset — hasil baru masuk");
        }

        lastSys = pd.sys; // Simpan hasil untuk dipakai saat countdown/beat
        lastDia = pd.dia;

        // Kirim hasil ke Master dengan fase 6 (HASIL AKHIR)
        kirimKemaster(6, 0, pd.sys, pd.dia, 0);
        Serial.println("[ESP-NOW] Kirim SYS:" + String(pd.sys) + " DIA:" + String(pd.dia));

        // Mulai countdown 10 menit untuk pengukuran otomatis berikutnya
        cntMenitSisa    = 10;
        countdownActive = true;
        lastCntTick     = millis(); // Catat waktu mulai countdown
        kirimKemaster(7, 0, lastSys, lastDia, cntMenitSisa);
        // Fase 7 = TIMER — Master tampilkan "10 MENIT"
        Serial.println("[COUNTDOWN] Mulai 10 menit...");

      } else {
        // ── Terima update FASE real-time ──
        String f = String(pd.fase); // Konversi char[] ke String untuk perbandingan

        // Konversi label string fase ke kode integer untuk Master
        int faseVal = 0;
        if      (f == "RESET")    faseVal = 1; // Kalibrasi sensor
        else if (f == "INFLATE")  faseVal = 2; // Pompa mengisi
        else if (f == "DEFLASI")  faseVal = 3; // Tekanan turun
        else if (f == "SISTOLE")  faseVal = 4; // Sistolik terdeteksi → Master flash hijau
        else if (f == "DIASTOLE") faseVal = 5; // Diastolik terdeteksi → Master flash kuning

        kirimKemaster(faseVal, pd.tek, lastSys, lastDia, 0);
        // Kirim ke Master: fase, tekanan saat ini, hasil terakhir (lastSys/lastDia)
      }
    }
    vTaskDelay(pdMS_TO_TICKS(2)); // Yield 2ms
  }
}

// ============================================================
// FUNGSI: handleCountdown
// Dipanggil dari loop() tiap iterasi
// Setiap 60 detik kurangi cntMenitSisa 1 menit
// Saat cntMenitSisa == 0 → kirim CMD:AUTO ke NIBP (ukur otomatis)
// ============================================================
void handleCountdown() {
  if (!countdownActive) return; // Tidak aktif → langsung return

  unsigned long now = millis();
  if (now - lastCntTick >= 60000) { // 60.000ms = 1 menit
    lastCntTick   = now;
    cntMenitSisa--;

    Serial.printf("[CDT] %d mnt\n", cntMenitSisa);
    kirimKemaster(7, 0, lastSys, lastDia, cntMenitSisa); // Update tampilan countdown di Master

    if (cntMenitSisa <= 0) {
      // Countdown habis → trigger pengukuran otomatis
      countdownActive = false;
      cntMenitSisa    = 10; // Reset untuk sesi berikutnya
      Serial.println("[CDT] Selesai! Kirim CMD:AUTO ke NIBP");
      Serial2.println("CMD:AUTO"); // NIBP.ino akan proses CMD:AUTO sama seperti CMD:START
      kirimKemaster(0, 0, 0, 0, 0); // Reset status Master ke WAITING
    }
  }
}

// ============================================================
// KONSTANTA DETEKSI BUNYI KOROTKOFF
// ============================================================

#define CHECK_SAMPLES      50    // Jumlah sampel untuk cek apakah sensor terpasang
#define CHECK_INTERVAL_MS  500   // Interval cek sensor: tiap 500ms dari loop()
#define ADC_MIN_VALID      800   // Batas bawah nilai ADC valid sensor terpasang
                                  // Di bawah 800 → sensor lepas / short
#define ADC_MAX_VALID      3000  // Batas atas nilai ADC valid
                                  // Di atas 3000 → sensor lepas / open circuit
#define ADC_MAX_STD        600   // Batas maksimum standar deviasi ADC
                                  // Jika noise terlalu besar (>600) → sensor tidak stabil

#define PIN_RAW             4    // GPIO4 = pin ADC mikrofon/piezo Korotkoff
#define SAMPLE_INTERVAL_US  1000 // Interval sampling ADC: 1000µs = 1ms → fs = 1000 Hz

#define PRINT_EVERY         10   // Cetak ke Serial tiap 10 sampel → 100 baris/detik

// ============================================================
// ALPHA_HP — Koefisien High Pass Filter IIR orde 1
// ------------------------------------------------------------
// RUMUS FILTER:
//   hp[n] = α × (hp[n-1] + raw[n] - raw[n-1])
//
// Ini adalah HPF IIR orde 1 bentuk diferensial.
// Diturunkan dari transfer function z-domain:
//   H(z) = α × (1 - z⁻¹) / (1 - α×z⁻¹)
//
// RUMUS HITUNG CUTOFF FREQUENCY (fc):
//   α = 1 / (1 + 2π × fc × T)
//   → fc = (1 - α) / (2π × α × T)
//
// Dimana:
//   T  = periode sampling = 1/fs = 1/1000 = 0.001 detik
//   fs = 1000 Hz (sampling rate ADC)
//   α  = ALPHA_HP = 0.8641
//
// PERHITUNGAN:
//   fc = (1 - 0.8641) / (2π × 0.8641 × 0.001)
//   fc = 0.1359 / (6.2832 × 0.0008641)
//   fc = 0.1359 / 0.005429
//   fc ≈ 25 Hz  
//
// EFEK:
//   - Frekuensi < 25 Hz → diredam (dibuang)
//   - Frekuensi > 25 Hz → diloloskan
//   - Membuang: DC bias ADC, drift lambat tekanan manset
//   - Mempertahankan: sinyal denyut Korotkoff (~40–150 Hz)
//
// CARA HITUNG BALIK (jika ingin ubah fc):
//   α = 1 / (1 + 2π × fc_baru × T)
//   Contoh fc=50Hz: α = 1/(1 + 2π×50×0.001) = 1/1.3142 ≈ 0.761
// ============================================================
#define ALPHA_HP  0.8641f

// ============================================================
// ALPHA_LP — Koefisien Low Pass Filter IIR orde 1 (EMA)
// ------------------------------------------------------------
// RUMUS FILTER:
//   lp[n] = lp[n-1] + α × (hp[n] - lp[n-1])
//
// Bentuk equivalen:
//   lp[n] = α × hp[n] + (1 - α) × lp[n-1]
//
// Ini adalah EMA (Exponential Moving Average) yang identik
// dengan LPF IIR orde 1.
// Transfer function z-domain:
//   H(z) = α / (1 - (1-α)×z⁻¹)
//
// RUMUS HITUNG CUTOFF FREQUENCY (fc):
// Untuk LPF EMA, hubungan α dengan fc menggunakan pendekatan:
//   α = 1 - e^(-2π × fc / fs)
//   → fc = -fs × ln(1 - α) / (2π)
//
// Dimana:
//   fs = 1000 Hz
//   α  = ALPHA_LP = 0.7640
//
// PERHITUNGAN:
//   fc = -1000 × ln(1 - 0.7640) / (2π)
//   fc = -1000 × ln(0.2360) / 6.2832
//   fc = -1000 × (-1.4426) / 6.2832
//   fc = 1442.6 / 6.2832
//   fc ≈ 229 Hz  
//
// CATATAN:
//   Cutoff 229 Hz berarti filter ini SANGAT LEMAH untuk fs=1kHz.
//   Hampir semua frekuensi di bawah 229Hz lolos termasuk noise.
//   Untuk meredam noise lebih baik, fc idealnya ~100 Hz:
//
//   α_ideal = 1 - e^(-2π × 100 / 1000)
//           = 1 - e^(-0.6283)
//           = 1 - 0.5336
//           ≈ 0.466
//
//   Namun nilai ini perlu diuji ulang dengan sinyal nyata karena
//   filter yang terlalu ketat bisa melemahkan sinyal Korotkoff.
//   Nilai kompromi yang bisa dicoba: 0.55 (fc ≈ 140 Hz)
//
// CARA HITUNG BALIK (jika ingin ubah fc):
//   α = 1 - e^(-2π × fc_baru / fs)
//   Contoh fc=100Hz: α = 1 - e^(-0.6283) ≈ 0.466
//   Contoh fc=150Hz: α = 1 - e^(-0.9425) ≈ 0.610
// ============================================================
#define ALPHA_LP  0.7640f

// ============================================================
// ENV_ATTACK & ENV_DECAY — Envelope Follower Asimetris
// ------------------------------------------------------------
// RUMUS ENVELOPE:
//   Jika lp_rect > env_prev (sinyal naik):
//     env[n] = env[n-1] + ATTACK × (lp_rect - env[n-1])
//            = ATTACK × lp_rect + (1-ATTACK) × env[n-1]
//
//   Jika lp_rect <= env_prev (sinyal turun):
//     env[n] = env[n-1] × DECAY
//
// ATTACK = 0.8 — kecepatan naik
//   Dalam 1 sampel (1ms), envelope mencapai:
//   80% dari nilai baru dalam 1 langkah
//   Sisa 20% dicapai dalam beberapa sampel berikutnya.
//   Waktu untuk mencapai 99% nilai baru (time to settle):
//     0.2^n = 0.01 → n = log(0.01)/log(0.2) ≈ 3 sampel = 3ms
//   → Envelope naik penuh dalam ~3ms: sangat cepat, tidak miss beat
//
// DECAY = 0.98 — kecepatan turun
//   Setiap sampel (1ms), envelope turun menjadi 98% dari sebelumnya.
//   Waktu turun ke 1/e (≈37%) dari nilai puncak = TIME CONSTANT (τ):
//     0.98^n = 0.3679
//     n × ln(0.98) = ln(0.3679)
//     n = ln(0.3679) / ln(0.98)
//     n = (-1.0000) / (-0.0202)
//     n ≈ 49 sampel = 49ms
//
//   Waktu turun dari puncak ke ~1% (hampir nol):
//     0.98^n = 0.01
//     n = ln(0.01) / ln(0.98)
//     n = (-4.6052) / (-0.0202)
//     n ≈ 228 sampel = 228ms
//
//   Interval denyut nadi normal: 600–1000ms (60–100 bpm)
//   → Dalam 600ms, envelope turun ke: 0.98^600 ≈ 0.000006 ≈ ~0
//   → Envelope sudah hampir nol sebelum beat berikutnya ✅
//   → Tidak ada carry-over antar beat
// ============================================================
#define ENV_ATTACK  0.8f
#define ENV_DECAY   0.98f

// ============================================================
// THRESHOLD — Ambang Batas Deteksi Beat
// ------------------------------------------------------------
// Nilai envelope minimum agar dianggap sebagai bunyi Korotkoff.
// Satuan: unit ADC (0–4095 untuk 12-bit, range efektif tergantung
//         amplitudo sinyal mikrofon/piezo yang digunakan).
//
// CARA MENENTUKAN NILAI THRESHOLD:
//   1. Jalankan kode, buka Serial Plotter
//   2. Amati kolom ENVELOPE saat tidak ada bunyi → catat nilai noise floor
//   3. Amati kolom ENVELOPE saat ada denyut nyata → catat nilai puncak
//   4. THRESHOLD = nilai antara noise floor dan puncak, lebih dekat ke puncak
//      Contoh: noise floor = 50, puncak beat = 500 → THRESHOLD = 300 (tengah ke atas)
//
// Nilai 300 di sini adalah hasil empiris/eksperimen dengan hardware ini.
// Jika terlalu banyak false positive → naikkan THRESHOLD
// Jika beat tidak terdeteksi padahal ada bunyi → turunkan THRESHOLD
// ============================================================
#define THRESHOLD  300.0f

// ============================================================
// REFRACTORY_MS — Periode Refrakter setelah Beat
// ------------------------------------------------------------
// Setelah satu beat terdeteksi, sinyal DIABAIKAN selama N sampel.
// N = REFRACTORY_MS = 50 sampel @ 1kHz = 50ms
//
// TUJUAN:
//   Mencegah satu bunyi Korotkoff terhitung >1 kali.
//   Bunyi Korotkoff nyata berlangsung ~20–50ms per ketukan.
//   Tanpa refractory: satu bunyi bisa trigger 5–20 beat palsu.
//
// CARA HITUNG NILAI YANG TEPAT:
//   - Durasi bunyi Korotkoff: ~20–50ms
//   - Jarak minimum antar denyut (HR max = 200 bpm):
//     T_min = 60000ms / 200 = 300ms
//   - REFRACTORY harus: > durasi bunyi, < jarak antar denyut
//   - 50ms > 50ms (durasi bunyi) DAN 50ms << 300ms (jarak denyut) ✅
//   - Nilai aman: 50–150ms untuk HR hingga 200 bpm
// ============================================================
#define REFRACTORY_MS  50

// ── Ring buffer untuk ADC — isi oleh ISR timer, baca oleh adcTask ──
#define BUFFER_SIZE 512          // Kapasitas ring buffer (harus pangkat 2 untuk efisiensi modulo)
volatile uint16_t adcBuf[BUFFER_SIZE]; // Buffer array — volatile karena diisi ISR
volatile int  bufHead    = 0;    // Indeks posisi tulis ISR (ISR isi di sini)
volatile int  bufTail    = 0;    // Indeks posisi baca adcTask (task baca dari sini)
volatile bool bufOverrun = false; // Flag: true jika ISR tidak bisa tulis (buffer penuh)
                                   // Terjadi jika adcTask terlalu lambat mengosongkan buffer

// ── Timer hardware untuk sampling ADC berkala ──
hw_timer_t*  adcTimer    = NULL;                        // Pointer ke timer hardware ESP32-S3
portMUX_TYPE adcTimerMux = portMUX_INITIALIZER_UNLOCKED; // Mutex untuk proteksi akses buffer di ISR

// ============================================================
// ISR: onAdcTimer — dipanggil oleh timer hardware tiap 1ms (1kHz)
// IRAM_ATTR: fungsi disimpan di IRAM agar akses cepat saat interrupt
// Baca satu sampel ADC → simpan ke ring buffer
// ============================================================
void IRAM_ATTR onAdcTimer() {
  portENTER_CRITICAL_ISR(&adcTimerMux); // Kunci mutex — cegah adcTask baca saat ISR tulis

  int next = (bufHead + 1) % BUFFER_SIZE; // Hitung posisi head berikutnya (circular)

  if (next == bufTail) {
    // Buffer penuh — adcTask belum mengosongkan
    // Tidak tulis — set flag overrun, data sampel ini hilang
    bufOverrun = true;
  } else {
    adcBuf[bufHead] = (uint16_t)analogRead(PIN_RAW); // Baca ADC 12-bit → simpan ke buffer
    bufHead = next; // Geser head ke posisi berikutnya
  }

  portEXIT_CRITICAL_ISR(&adcTimerMux); // Buka mutex
}

// ============================================================
// FUNGSI: cekSensorKorotkoff
// Verifikasi apakah sensor mikrofon/piezo terpasang dengan benar
// Dipanggil dari loop() tiap 500ms — TIDAK dari ISR atau adcTask
// Metode: ambil 50 sampel → cek rata-rata (range valid) & std (noise wajar)
// ============================================================
void cekSensorKorotkoff() {
  long sum = 0;
  int  samples[CHECK_SAMPLES]; // Array 50 sampel lokal

  for (int i = 0; i < CHECK_SAMPLES; i++) {
    samples[i] = analogRead(PIN_RAW);
    sum += samples[i];
    delayMicroseconds(200); // Jeda 200µs antar sampel — total ~10ms untuk 50 sampel
  }

  long mean = sum / CHECK_SAMPLES; // Nilai rata-rata ADC

  // Cek range rata-rata: di luar range → sensor lepas atau short
  if (mean < ADC_MIN_VALID || mean > ADC_MAX_VALID) {
    sensorTerpasang = false;
    return;
  }

  // Hitung variance → std (standar deviasi)
  long varSum = 0;
  for (int i = 0; i < CHECK_SAMPLES; i++) {
    long diff = samples[i] - mean;
    varSum += diff * diff; // Akumulasi (xi - mean)^2
  }
  long std = (long)sqrt((float)(varSum / CHECK_SAMPLES)); // sqrt(variance) = std

  // Std terlalu besar → sensor tidak stabil / ada interferensi besar
  // Std terlalu kecil → sensor short (tidak ada sinyal sama sekali)
  sensorTerpasang = (std < ADC_MAX_STD); // Valid jika std < 600
}

// ============================================================
// TASK: adcTask — berjalan di Core 1, prioritas 1
// Menguras ring buffer ADC yang diisi ISR timer
// Proses setiap sampel melalui rantai filter digital:
//   Raw ADC → High Pass Filter → Low Pass Filter → Envelope → Beat Detection
//
// Pipeline sinyal Korotkoff:
//   1. HP filter: buang DC bias dan drift lambat → isolasi sinyal denyut (~20-100Hz)
//   2. LP filter: haluskan sinyal HP → kurangi noise frekuensi tinggi
//   3. Rectifier: abs(lp) → semua nilai positif untuk envelope
//   4. Envelope: tracking level energi → naik cepat, turun lambat
//   5. Threshold: jika envelope >= 300 → beat terdeteksi
//   6. Refractory: 50ms setelah beat → abaikan sinyal (cegah double count)
// ============================================================
void adcTask(void *pvParameters) {
  float hp_prev       = 0.0f; // Nilai output HP filter iterasi sebelumnya
  float raw_prev      = 0.0f; // Nilai raw ADC iterasi sebelumnya (dibutuhkan rumus HP)
  float lp_prev       = 0.0f; // Nilai output LP filter iterasi sebelumnya
  float env_prev      = 0.0f; // Nilai envelope iterasi sebelumnya
  int   refractoryCount = 0;  // Counter refrakter tersisa (countdown dari 50 ke 0)
  int   printCnt      = 0;    // Counter cetak Serial (reset tiap 10 sampel)

  for (;;) {
    // ── Tangani overrun — reset flag jika buffer sempat penuh ──
    if (bufOverrun) {
      portENTER_CRITICAL(&adcTimerMux);
      bufOverrun = false; // Hanya reset flag — data yang hilang tidak bisa dikembalikan
      portEXIT_CRITICAL(&adcTimerMux);
    }

    // ── Proses semua sampel yang tersedia di buffer ──
    while (bufTail != bufHead) {
      // Ada data — ambil satu sampel dari tail
      float raw = (float)adcBuf[bufTail];
      bufTail = (bufTail + 1) % BUFFER_SIZE; // Geser tail (circular)

      // ══════════════════════════════════════════════════════
      // STEP 1: HIGH PASS FILTER (HPF) IIR ORDE 1
      // ══════════════════════════════════════════════════════
      // RUMUS:
      //   hp[n] = α × (hp[n-1] + raw[n] - raw[n-1])
      //
      // Diturunkan dari transfer function z-domain:
      //   H(z) = α × (1 - z⁻¹) / (1 - α×z⁻¹)
      //
      // Dalam bentuk difference equation:
      //   hp[n] - α×hp[n-1] = α×raw[n] - α×raw[n-1]
      //   hp[n] = α×hp[n-1] + α×(raw[n] - raw[n-1])
      //         = α × (hp[n-1] + raw[n] - raw[n-1])  ← bentuk di kode
      //
      // CUTOFF FREQUENCY:
      //   α  = 0.8641, fs = 1000 Hz, T = 0.001 s
      //   fc = (1 - α) / (2π × α × T)
      //      = (1 - 0.8641) / (2π × 0.8641 × 0.001)
      //      = 0.1359 / 0.005429
      //      ≈ 25 Hz
      //
      // EFEK PADA SINYAL:
      //   < 25 Hz → dibuang  (DC bias ADC, drift tekanan lambat)
      //   > 25 Hz → diloloskan (sinyal Korotkoff 40–150 Hz) ✅
      // ══════════════════════════════════════════════════════
      float hp = ALPHA_HP * (hp_prev + raw - raw_prev);

      // ══════════════════════════════════════════════════════
      // STEP 2: LOW PASS FILTER (LPF) IIR ORDE 1 — EMA
      // ══════════════════════════════════════════════════════
      // RUMUS:
      //   lp[n] = lp[n-1] + α × (hp[n] - lp[n-1])
      //
      // Bentuk equivalen (EMA):
      //   lp[n] = α × hp[n] + (1-α) × lp[n-1]
      //
      // Transfer function z-domain:
      //   H(z) = α / (1 - (1-α)×z⁻¹)
      //
      // CUTOFF FREQUENCY:
      //   α  = 0.7640, fs = 1000 Hz
      //   fc = -fs × ln(1 - α) / (2π)
      //      = -1000 × ln(1 - 0.7640) / (2π)
      //      = -1000 × ln(0.2360) / 6.2832
      //      = -1000 × (-1.4426) / 6.2832
      //      ≈ 229 Hz
      //
      /
      // ══════════════════════════════════════════════════════
      float lp = lp_prev + ALPHA_LP * (hp - lp_prev);

      // ══════════════════════════════════════════════════════
      // STEP 3: FULL-WAVE RECTIFIER
      // ══════════════════════════════════════════════════════
      // RUMUS:
      //   lp_rect = |lp[n]|
      //
      // Sinyal audio/akustik berupa gelombang AC (bolak-balik
      // positif dan negatif di sekitar nol).
      // Envelope follower hanya bisa bekerja pada nilai positif.
      // fabsf() = absolute value untuk float (lebih cepat dari abs())
      //
      // EFEK:
      //   Gelombang sinus: -300 → +300 → -300
      //   Setelah rectify:  300 →  300 →  300  (semua positif)
      //   → Envelope bisa tracking amplitudo dengan benar
      // ══════════════════════════════════════════════════════
      float lp_rect = fabsf(lp);

      // ══════════════════════════════════════════════════════
      // STEP 4: ENVELOPE FOLLOWER ASIMETRIS
      // ══════════════════════════════════════════════════════
      // RUMUS NAIK (attack):
      //   env[n] = env[n-1] + ATTACK × (lp_rect - env[n-1])
      //          = ATTACK × lp_rect + (1-ATTACK) × env[n-1]
      //          = 0.8 × lp_rect + 0.2 × env[n-1]
      //
      // RUMUS TURUN (decay):
      //   env[n] = env[n-1] × DECAY
      //          = env[n-1] × 0.98
      //
      // MENGAPA ASIMETRIS (attack ≠ decay)?
      //   - Beat Korotkoff datang tiba-tiba → harus ditangkap CEPAT (attack tinggi)
      //   - Antara beat, sinyal pelan-pelan menghilang → turun LAMBAT (decay tinggi)
      //   - Jika attack lambat → puncak beat terlewat, threshold tidak tercapai
      //   - Jika decay cepat → envelope langsung nol, tidak stabil
      //
      // ANALISIS WAKTU ATTACK (ENV_ATTACK = 0.8):
      //   Sampel ke-1: env = 0.8×puncak + 0.2×0     = 80% puncak
      //   Sampel ke-2: env = 0.8×puncak + 0.2×80%   = 96% puncak
      //   Sampel ke-3: env = 0.8×puncak + 0.2×96%   = 99.2% puncak
      //   → Envelope penuh dalam ~3 sampel = 3ms ✅
      //
      // ANALISIS WAKTU DECAY (ENV_DECAY = 0.98):
      //   Time constant τ (turun ke 37% = 1/e):
      //     0.98^n = 0.3679
      //     n = ln(0.3679) / ln(0.98) = -1.0 / -0.0202 ≈ 49 sampel = 49ms
      //
      //   Turun ke ~1% (hampir nol):
      //     0.98^n = 0.01
      //     n = ln(0.01) / ln(0.98) = -4.6052 / -0.0202 ≈ 228 sampel = 228ms
      //
      //   HR normal 60 bpm → jarak antar beat = 1000ms
      //   HR cepat 100 bpm → jarak antar beat = 600ms
      //   228ms << 600ms → envelope sempat turun sebelum beat berikutnya ✅
      // ══════════════════════════════════════════════════════
      float env;
      if (lp_rect > env_prev) {
        // Sinyal naik → ikuti dengan cepat (ATTACK = 0.8)
        env = env_prev + ENV_ATTACK * (lp_rect - env_prev);
      } else {
        // Sinyal turun → ikuti dengan sangat lambat (DECAY = 0.98)
        env = env_prev * ENV_DECAY;
      }

      // ══════════════════════════════════════════════════════
      // STEP 5: BEAT DETECTION + REFRACTORY PERIOD
      // ══════════════════════════════════════════════════════
      // LOGIKA DETEKSI:
      //   if refractoryCount > 0:
      //     → Masih dalam periode refrakter → abaikan sinyal
      //     → Kurangi counter 1 per sampel
      //   else if env >= THRESHOLD (300):
      //     → Beat terdeteksi!
      //     → Set refractoryCount = 50 (block 50ms ke depan)
      //
      // MENGAPA PERLU REFRACTORY?
      //   Satu bunyi Korotkoff berlangsung ~20–50ms.
      //   Tanpa refractory, satu bunyi bisa trigger 20–50 beat
      //   (karena envelope tinggi selama durasi bunyi).
      //   Dengan refractory 50ms: satu bunyi = tepat 1 beat ✅
      //
      // VALIDASI NILAI 50ms:
      //   Durasi bunyi Korotkoff: ~20–50ms → 50ms cukup menutup 1 bunyi
      //   HR maksimum manusia: ~200 bpm → jarak min antar beat = 300ms
      //   50ms << 300ms → tidak ada beat nyata yang terlewat ✅
      // ══════════════════════════════════════════════════════
      bool beat = false;
      if (refractoryCount > 0) {
        refractoryCount--; // Hitung mundur refrakter — belum boleh deteksi beat
      } else if (env >= THRESHOLD) {
        beat = true;
        refractoryCount = REFRACTORY_MS; // Blokir 50 sampel ke depan
      }

      // ── Update state untuk iterasi berikutnya ──
      raw_prev = raw; // Dibutuhkan rumus HP filter
      hp_prev  = hp;
      lp_prev  = lp;
      env_prev = env;

      // ── Update variabel global untuk log dan espnowTask ──
      g_filter   = lp_rect; // Untuk log Serial (nilai filter setelah rectify)
      g_envelope = env;      // Untuk log Serial (nilai envelope)

      // ── Set flag beat — esp_now_send dilakukan di espnowTask ──
      // TIDAK kirim langsung dari adcTask karena esp_now_send harus di Core 0
      // g_beat = true → espnowTask (Core 0) akan baca dan kirim
      if (beat && sensorTerpasang && g_deflasi && !countdownActive) {
        g_beat = true;
      }

      // ── Log ke Serial USB tiap 10 sampel (100 baris/detik) ──
      printCnt++;
      if (printCnt < PRINT_EVERY) continue; // Lewati sampel ini untuk log
      printCnt = 0;

      int markVal = g_beat ? 20 : 0; // 20 = ada beat (terlihat menonjol di plotter), 0 = tidak ada

      // Format: TEK | RAW | FILTER | ENVELOPE | THRESHOLD | MARK
      // Cocok untuk Serial Plotter Arduino IDE (tab-separated)
      Serial.print(g_tek);       // Tekanan manset saat ini
      Serial.print("\t");
      Serial.print((int)adcBuf[(bufTail == 0) ? BUFFER_SIZE-1 : bufTail-1]); // Nilai RAW ADC
      // Mengambil sampel terakhir yang dibaca (bufTail-1, dengan wrap-around)
      Serial.print("\t");
      Serial.print(g_filter, 2);   // Nilai sinyal setelah HP+LP+rectify (2 desimal)
      Serial.print("\t");
      Serial.print(g_envelope, 2); // Nilai envelope (2 desimal)
      Serial.print("\t");
      Serial.print(THRESHOLD, 2);  // Garis threshold konstan (untuk referensi visual di plotter)
      Serial.print("\t");
      Serial.println(markVal);     // 0 atau 20 — spike visual saat beat
    }

    vTaskDelay(pdMS_TO_TICKS(2)); // Yield 2ms — beri waktu ISR mengisi buffer
  }
}

// ============================================================
// FUNGSI: setup — sekali saat boot
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("ESP32-S3 KOROTKOFF V4 BOOT");

  analogReadResolution(12);         // ADC 12-bit (0–4095)
  analogSetAttenuation(ADC_11db);   // Atenuasi 11dB → range input ADC 0–3.3V
                                     // Tanpa ini, range default hanya 0–1.1V
                                     // (sinyal mikrofon bisa clipping)

  Serial2.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  Serial.println("UART2 OK — RX:GPIO8 TX:GPIO18");

  WiFi.mode(WIFI_STA);              // WAJIB sebelum esp_now_init()
  Serial.print("MAC Korotkoff: ");
  Serial.println(WiFi.macAddress()); // Cetak MAC agar bisa dimasukkan ke Master

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW FAILED!"); while(1) delay(1000);
  }
  esp_now_register_send_cb(OnDataSent); // Daftarkan callback kirim
  esp_now_register_recv_cb(OnDataRecv); // Daftarkan callback terima
  Serial.println("ESP-NOW OK");

  // Daftarkan Master sebagai peer tujuan pengiriman
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, masterAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Add peer FAILED!"); while(1) delay(1000);
  }
  Serial.println("Peer Master OK");

  // Buat queue 30 item ParsedData (UART → ESP-NOW)
  parsedQueue = xQueueCreate(30, sizeof(ParsedData));
  if (parsedQueue == NULL) {
    Serial.println("Queue GAGAL!"); while(1);
  }
  Serial.println("Queue OK");

  memset(&dataKirim, 0, sizeof(dataKirim)); // Bersihkan buffer kirim

  // ── Buat 3 FreeRTOS task ──
  xTaskCreatePinnedToCore(uartRxTask, "UART_RX",     4096, NULL, 2, NULL, 1);
  // uartRxTask: Core 1, prioritas 2 (lebih tinggi dari adcTask agar parsing tidak delay)
  Serial.println("UART RX Task OK — Core 1");

  xTaskCreatePinnedToCore(espnowTask, "ESPNOW_Task", 4096, NULL, 1, NULL, 0);
  // espnowTask: Core 0 — sama dengan WiFi/ESP-NOW stack, menghindari konflik
  Serial.println("ESP-NOW Task OK — Core 0");

  xTaskCreatePinnedToCore(adcTask,    "ADC_Task",    4096, NULL, 1, NULL, 1);
  // adcTask: Core 1, prioritas 1 (sama dengan espnowTask tapi core berbeda)
  Serial.println("ADC Task OK — Core 1");

  // ── Inisialisasi timer hardware untuk sampling ADC 1kHz ──
  // API BARU ESP32-S3 Arduino v3.x:
  adcTimer = timerBegin(1000000);           // Frekuensi clock timer = 1.000.000 Hz (1MHz)
  timerAttachInterrupt(adcTimer, &onAdcTimer); // Pasang ISR ke timer ini
  timerAlarm(adcTimer, SAMPLE_INTERVAL_US, true, 0);
  // SAMPLE_INTERVAL_US = 1000 → alarm tiap 1000 tick @ 1MHz = tiap 1ms = 1kHz
  // true = auto-reload (berulang terus)
  // 0    = tidak ada auto-stop setelah N alarm
  Serial.println("ADC Timer OK — 1000 Hz");

  kirimKemaster(0, 0, 0, 0, 0); // Kirim status awal ke Master: fase 0 = WAITING
  Serial.println("KOROTKOFF V4 READY\n");
  Serial.println("=== GUI Format: TEK | RAW | FILTER | ENVELOPE | THRESHOLD | MARK ===");
}

// ============================================================
// FUNGSI: loop — berjalan di Core 1 (Arduino default task)
// Tugas ringan yang tidak perlu presisi real-time:
//   1. Kelola countdown timer (cek tiap 10ms, tick tiap 60 detik)
//   2. Cek kondisi sensor Korotkoff tiap 500ms
//   3. Terima perintah manual via Serial USB (debugging)
// ============================================================
void loop() {
  handleCountdown(); // Kurangi countdown tiap menit, trigger CMD:AUTO saat habis

  // ── Cek sensor tiap 500ms ──
  // Dilakukan dari loop() bukan dari task agar tidak mengganggu timing adcTask
  // (adcTask fokus proses buffer, bukan lakukan analogRead tambahan)
  unsigned long nowMs = millis();
  if (nowMs - lastSensorCheck >= CHECK_INTERVAL_MS) {
    lastSensorCheck = nowMs;
    cekSensorKorotkoff(); // Verifikasi sensor masih terpasang
  }

  // ── Perintah manual via Serial USB (untuk debugging) ──
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toLowerCase();
    if (cmd == "start") {
      if (countdownActive) {
        countdownActive = false;
        cntMenitSisa    = 10;
        Serial.println("[COUNTDOWN] Dihentikan — start manual");
      }
      Serial.println("[MANUAL] Kirim CMD:START ke NIBP");
      Serial2.println("CMD:START"); // Langsung kirim START ke NIBP tanpa lewat ESP-NOW
    }
  }

  delay(10); // Delay 10ms — task ringan, tidak perlu lebih cepat
}