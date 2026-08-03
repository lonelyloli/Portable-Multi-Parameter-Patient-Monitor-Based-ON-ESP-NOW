/*
 * ESP32 GATEWAY v2.0
 * 
 * PERUBAHAN DARI v1.5:
 * - uploadCSVToSheets() diganti → kirim file sekaligus (1x HTTP POST)
 * - Tidak ada lagi loop batch 15 baris
 * - Jauh lebih cepat untuk semua ukuran file
 * 
 * WIRING SD CARD:
 * CS   → GPIO 5  | MOSI → GPIO 23
 * MISO → GPIO 19 | SCK  → GPIO 18
 * VCC  → 3.3V    | GND  → GND
 * 
 * WIRING SERIAL FROM MASTER:
 * Gateway RX2(16) ← Master TX(43)
 * Gateway TX2(17) → Master RX(44)
 */

#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <SPI.h>
#include <SD.h>

// ============================================================
// KONFIGURASI
// ============================================================
const char* WIFI_SSID     = "rd";
const char* WIFI_PASSWORD = "55555555";

// URL lama — masih dipakai untuk START event via uploadToSheets()
const char* SCRIPT_URL = "https://script.google.com/macros/s/AKfycbzaubZChXcVPhaCHY3R2whP5PD2CJt0Ps8ZL7V8ta5Cwt1LDDdrKMRPB3Zi0x4eJISqcw/exec";

// URL baru — untuk upload file sekaligus saat STOP
const char* DRIVE_SCRIPT_URL = "https://script.google.com/macros/s/AKfycby6gHN7HInTDjOHj0M0jx8B75_HqI10KxQeaCsUFNjMLneOib-iIeJBLHfD2G6q7c5Neg/exec";

// ============================================================
// PIN
// ============================================================
#define SD_CS   5
#define SD_MOSI 23
#define SD_MISO 19
#define SD_SCK  18

HardwareSerial masterSerial(2);
const int MASTER_RX = 16;
const int MASTER_TX = 17;
const int LED_PIN   = 2;

// ============================================================
// STATE
// ============================================================
bool isRecording   = false;
bool wifiConnected = false;
bool sdCardReady   = false;

String        recordingId  = "";
File          csvFile;
unsigned long sampleCount  = 0;
unsigned long recStartTime = 0;

// ============================================================
// GATEWAY STATUS SENDER
// ============================================================
void sendStatus(const char* st) {
  String json = "{\"st\":\"";
  json += st;
  json += "\"}";
  masterSerial.println(json);
}

// ============================================================
// WIFI
// ============================================================
void connectWiFi() {
  Serial.println("📶 WiFi: " + String(WIFI_SSID));
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int t = 0;
  while (WiFi.status() != WL_CONNECTED && t < 20) {
    delay(500); Serial.print("."); t++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    Serial.println("\n✅ WiFi OK: " + WiFi.localIP().toString());
    digitalWrite(LED_PIN, HIGH);
    if (!isRecording) sendStatus("WIFI_OK");
  } else {
    wifiConnected = false;
    Serial.println("\n❌ WiFi GAGAL");
    digitalWrite(LED_PIN, LOW);
    if (!isRecording) sendStatus("WIFI_OFF");
  }
}

void checkWiFi() {
  if (WiFi.status() != WL_CONNECTED) {
    wifiConnected = false;
    connectWiFi();
  }
}

// ============================================================
// SD CARD
// ============================================================
bool initSDCard() {
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS) || SD.cardType() == CARD_NONE) {
    Serial.println("❌ SD GAGAL!");
    return false;
  }
  Serial.println("✅ SD OK");
  if (!SD.exists("/recordings")) SD.mkdir("/recordings");
  return true;
}

// ============================================================
// CSV
// ============================================================
bool createCSVFile(String id) {
  String fn = "/recordings/" + id + ".csv";
  csvFile = SD.open(fn, FILE_WRITE);
  if (!csvFile) { Serial.println("❌ CSV gagal"); return false; }
  csvFile.println("Timestamp_ms,ECG_Lead1,ECG_Lead2,ECG_Lead3,SpO2_Wave,Resp_Wave");
  csvFile.flush();
  Serial.println("✅ CSV: " + fn);
  return true;
}

void writeCSVLine(unsigned long ts, int e1, int e2, int e3, int sw, int rw) {
  if (!csvFile) return;
  csvFile.printf("%lu,%d,%d,%d,%d,%d\n", ts, e1, e2, e3, sw, rw);
  sampleCount++;
  if (sampleCount % 20 == 0) csvFile.flush();
}

void closeCSVFile() {
  if (csvFile) { csvFile.close(); Serial.println("✅ CSV closed"); }
}

// ============================================================
// HTTP UPLOAD (masih dipakai untuk START event)
// ============================================================
bool uploadToSheets(const String& payload) {
  if (!wifiConnected) { Serial.print("[NoWiFi]"); return false; }

  HTTPClient http;
  http.begin(SCRIPT_URL);
  http.addHeader("Content-Type", "application/json");
  http.setTimeout(25000);
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);

  int code = http.POST(payload);
  String resp = "";
  if (code > 0) resp = http.getString();
  http.end();

  if (code == 200 && resp.length() > 0) {
    bool hasJson = resp.indexOf("{") >= 0 && resp.indexOf("}") >= 0;
    bool isError = resp.indexOf("Error") >= 0 && resp.indexOf("DOCTYPE") >= 0;
    if (hasJson && !isError) {
      if (resp.indexOf("error") >= 0) {
        Serial.print("[srv-err]");
        return false;
      }
      return true;
    }
    Serial.print("[bad-resp]");
    return false;
  }
  if (code == 302) return true;
  Serial.print("[" + String(code) + "]");
  return false;
}

// ============================================================
// UPLOAD CSV → GOOGLE DRIVE + SHEETS (v2.0 — 1x POST)
// ============================================================
// ============================================================
// GANTI HANYA FUNGSI INI di file .ino kamu
// Fungsi lain (writeCSVLine, closeCSVFile, dll) tidak disentuh
//
// CARA KERJA:
// - Baca file CSV dari SD card langsung ke HTTP stream
// - RAM ESP32 hanya pakai buffer kecil 512 bytes
// - Tidak ada copy data ke String besar di RAM
// - 1x HTTP POST untuk semua data, secepat WiFi
// ============================================================
void uploadCSVToSheets() {
  Serial.println("\n📤 Stream CSV ke Google Drive...");
  sendStatus("UPLOADING");

  // Kirim START event ke Apps Script LAMA
  // supaya sheet Recordings terupdate seperti biasa
  Serial.print("START event: ");
  String startJson = "{\"event\":\"START\",\"recId\":\"" + recordingId + "\"}";
  bool ok = false;
  for (int r = 0; r < 3 && !ok; r++) {
    if (r > 0) { Serial.print("⟳"); delay(2000); checkWiFi(); }
    ok = uploadToSheets(startJson);
  }
  Serial.println(ok ? "✓" : "✗ (lanjut)");

  // Buka file CSV dari SD card
  String filename = "/recordings/" + recordingId + ".csv";
  File file = SD.open(filename);
  if (!file) {
    Serial.println("❌ File tidak bisa dibuka!");
    sendStatus(wifiConnected ? "WIFI_OK" : "WIFI_OFF");
    return;
  }

  size_t fileSize = file.size();
  Serial.printf("📄 File: %d bytes\n", fileSize);

  if (fileSize == 0) {
    Serial.println("❌ File kosong!");
    file.close();
    sendStatus(wifiConnected ? "WIFI_OK" : "WIFI_OFF");
    return;
  }

  // Kirim dengan retry maksimal 3x
  bool success = false;

  for (int attempt = 1; attempt <= 3 && !success; attempt++) {
    if (attempt > 1) {
      Serial.printf("⟳ Coba ulang (%d/3)...\n", attempt);
      delay(3000);
      checkWiFi();
      file.seek(0); // kembali ke awal file sebelum retry
    }

    if (!wifiConnected) {
      Serial.println("❌ WiFi tidak terhubung");
      continue;
    }

    HTTPClient http;

    // Buat URL dengan recId sebagai query parameter
    // Apps Script baca recId dari e.parameter["recId"]
    String urlWithParam = String(DRIVE_SCRIPT_URL) + "?recId=" + recordingId;

    http.begin(urlWithParam);
    http.addHeader("Content-Type", "text/plain"); // kirim CSV mentah, bukan JSON
    http.setTimeout(60000); // 60 detik — cukup untuk file besar
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);

    Serial.printf("📡 Streaming %d bytes...", fileSize);

    // ── INI INTI DARI PILIHAN 2 ──
    // sendRequest langsung dari File object (SD card)
    // ESP32 membaca file per blok kecil dan langsung kirim ke WiFi
    // RAM yang dipakai hanya buffer internal HTTP (~512 bytes)
    // Tidak ada String besar di memori sama sekali
    file.seek(0);
    int code = http.sendRequest("POST", &file, fileSize);

    String resp = "";
    if (code > 0) {
      resp = http.getString();
    }
    http.end();

    Serial.printf(" HTTP %d\n", code);

    if (code == 200 && resp.indexOf("\"ok\"") >= 0) {
      success = true;
      // Tampilkan jumlah baris dari response
      int idx = resp.indexOf("\"rows\":");
      if (idx >= 0) {
        String rowStr = resp.substring(idx + 7, idx + 13);
        rowStr.trim();
        Serial.println("✅ Upload sukses! Baris: " + rowStr);
      } else {
        Serial.println("✅ Upload sukses!");
      }
    } else if (code == 302) {
      success = true;
      Serial.println("✅ Upload sukses (redirect)");
    } else {
      Serial.printf("❌ Gagal. Code: %d\n", code);
      if (resp.length() > 0) {
        Serial.println("   Response: " + resp.substring(0, 150));
      }
    }
  }

  file.close();

  // Summary
  Serial.println("\n══════════════════════════════");
  if (success) {
    Serial.println("✅ Upload selesai!");
    Serial.println("   File  : " + filename);
    Serial.println("   RecID : " + recordingId);
    Serial.printf( "   Size  : %d bytes\n", fileSize);
  } else {
    Serial.println("❌ Upload gagal setelah 3x percobaan");
    Serial.println("   File tetap tersimpan di SD card");
    Serial.println("   Bisa di-upload ulang nanti");
  }
  Serial.println("══════════════════════════════\n");

  sendStatus("UPLOAD_OK");
  delay(5000);
  sendStatus(wifiConnected ? "WIFI_OK" : "WIFI_OFF");
}
// ============================================================
// PROCESS COMMAND FROM MASTER
// ============================================================
void processCommand(const String& js) {
  StaticJsonDocument<384> doc;
  if (deserializeJson(doc, js)) {
    Serial.println("⚠️ JSON parse error");
    return;
  }

  const char* cmd = doc["cmd"];
  if (!cmd) return;

  if (strcmp(cmd, "START") == 0) {
    recordingId  = String((const char*)doc["recId"]);
    isRecording  = true;
    sampleCount  = 0;
    recStartTime = millis();
    Serial.println("\n🔴 RECORDING START: " + recordingId);
    if (!createCSVFile(recordingId)) { isRecording = false; return; }
    for (int i = 0; i < 3; i++) {
      digitalWrite(LED_PIN, LOW);  delay(100);
      digitalWrite(LED_PIN, HIGH); delay(100);
    }

  } else if (strcmp(cmd, "DATA") == 0) {
    if (!isRecording) return;
    writeCSVLine(
      doc["ts"] | 0UL,
      doc["e1"] | 0, doc["e2"] | 0,
      doc["e3"] | 0, doc["sw"] | 0, doc["rw"] | 0
    );

  } else if (strcmp(cmd, "STOP") == 0) {
    if (!isRecording) return;
    closeCSVFile();
    isRecording = false;
    unsigned long dur = (millis() - recStartTime) / 1000;
    Serial.println("\n⏹  RECORDING STOP");
    Serial.println("   Samples  : " + String(sampleCount));
    Serial.println("   Duration : " + String(dur) + "s");
    uploadCSVToSheets();
  }
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(2000);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.println("\n╔══════════════════════════════════════╗");
  Serial.println("║   ESP32 GATEWAY v2.0                 ║");
  Serial.println("╚══════════════════════════════════════╝\n");

  sdCardReady = initSDCard();
  if (!sdCardReady) {
    Serial.println("⚠️ SD tidak ditemukan! Cek wiring.");
    while (1) { digitalWrite(LED_PIN, !digitalRead(LED_PIN)); delay(200); }
  }

  masterSerial.begin(115200, SERIAL_8N1, MASTER_RX, MASTER_TX);
  masterSerial.setRxBufferSize(512);
  Serial.println("✅ Serial2 OK (RX:" + String(MASTER_RX) + " TX:" + String(MASTER_TX) + ")");

  connectWiFi();

  Serial.println("\n✅ Gateway Siap! Menunggu perintah dari Master...\n");
}

// ============================================================
// LOOP
// ============================================================
void loop() {
  if (masterSerial.available()) {
    String line = masterSerial.readStringUntil('\n');
    line.trim();
    if (line.length() > 5) {
      Serial.println("📥 " + line.substring(0, min(60, (int)line.length())) + "...");
      processCommand(line);
    }
  }

  if (isRecording) {
    static unsigned long lb = 0;
    if (millis() - lb >= 500) {
      lb = millis();
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    }
  }

  static unsigned long lw = 0;
  if (millis() - lw >= 10000) {
    lw = millis();
    checkWiFi();
    if (!isRecording) {
      if (wifiConnected) sendStatus("WIFI_OK");
      else               sendStatus("WIFI_OFF");
    }
  }

  delay(1);
}