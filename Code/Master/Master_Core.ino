/*
 * ESP32 MASTER - FINAL WITH MUTE ALARM + LEAD SELECTOR + RECORDING
 *
 * TAMBAHAN V19:
 * - Alarm baterai bunyi mulai 25% (slave & master)
 * - Deteksi slave SpO2 offline via timeout 10 detik
 * - txBatSL tampilkan WAITING saat slave SpO2 tidak terhubung
 * - Alarm wav1 hanya bunyi kalau slave SpO2 benar terhubung
 * - [TAMBAH] korotkoffBeat: dot * di txStatusNIBP saat beat Korotkoff
 * - [TAMBAH] nibpActive: update Nextion hanya saat fase NIBP berjalan
 */

#include <WiFi.h>
#include <esp_now.h>

// ============================================================
// SERIAL PORTS
// ============================================================

HardwareSerial nexSerial(2);
HardwareSerial gatewaySerial(1);

const int NEXTION_RX = 18;
const int NEXTION_TX = 17;
const int GATEWAY_RX = 11;
const int GATEWAY_TX = 10;

// ============================================================
// MAC ADDRESS SLAVE
// ============================================================

uint8_t macSlaveECG  [] = {0xCC, 0x8D, 0xA2, 0x0C, 0x2B, 0x70};
uint8_t macSlaveHRRR [] = {0x3C, 0x0F, 0x02, 0xD8, 0x08, 0x94};
uint8_t macSlaveNIBP [] = {0x3C, 0x0F, 0x02, 0xD8, 0x03, 0xBC};
uint8_t macSlaveSpO2 [] = {0xA4, 0xF0, 0x0F, 0x75, 0x71, 0x34};

// ============================================================
// DATA STRUCTURES
// ============================================================

typedef struct lead_command {
  int activeLead;
} lead_command;

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
  int korotkoffBeat;  // [TAMBAH] 1 = ada beat Korotkoff, 0 = tidak
} struct_message;

portMUX_TYPE   dataMux = portMUX_INITIALIZER_UNLOCKED;
struct_message dataTerima;
bool           dataReceived   = false;
volatile bool  gatewayPending = false;

struct ThresholdValues {
  int hrMax = 100;
  int hrMin = 60;
  int spo2Max = 100;
  int spo2Min = 95;
  int rrMax = 20;
  int rrMin = 12;
} thresholds;

// ============================================================
// STATE VARIABLES
// ============================================================

bool alarmEnabled = false;
bool speakerMuted = false;
bool hrAlarmActive = false;
bool spo2AlarmActive = false;
bool rrAlarmActive = false;
bool speakerOn = false;

int leadMode = 0;

unsigned long lastEcgUpdate  = 0;
unsigned long lastSpo2Update = 0;
unsigned long lastRespUpdate = 0;

const unsigned long ecgInterval  = 7.5;
const unsigned long spo2Interval = 10;
const unsigned long respInterval = 12.5;

float emaEcg  = 128.0f;
float emaSpo2 = 128.0f;
float emaResp = 128.0f;

const float alphaEcg  = 0.75f;
const float alphaSpo2 = 0.50f;
const float alphaResp = 0.40f;

#define GATEWAY_MIN_INTERVAL 20UL
unsigned long lastGatewaySend    = 0;

unsigned long lastAlarmCheckTime = 0;
const unsigned long alarmCheckInterval = 500;

unsigned long packetCount = 0;
unsigned long lastStatsTime = 0;

// ── Lead off define ──────────────────────────────────────────
#define LEADOFF_LA  (1 << 0)
#define LEADOFF_RA  (1 << 1)
#define LEADOFF_LL  (1 << 2)
#define LEADOFF_RL  (1 << 3)

// ── NIBP status variables ────────────────────────────────────
unsigned long lastNIBPBlinkTime   = 0;
const unsigned long NIBP_BLINK_MS = 400;
bool nibpBlinkState               = false;
int  lastFaseNIBP                 = 0;
bool sistoleFlash                 = false;
bool diastoleFlash                = false;
unsigned long sistoleFlashStart   = 0;
unsigned long diastoleFlashStart  = 0;
const unsigned long FLASH_DURATION = 400;
volatile bool nibpDataUpdated     = false;
volatile bool ecgDataUpdated      = false;
volatile bool korotkoffUpdated    = false;  // [TAMBAH]
volatile bool nibpActive          = false; 
bool nibpWasActive = false;  // track perubahan nibpActive
 // [TAMBAH] true saat fase NIBP berjalan

// [TAMBAH] Timer polling NIBP — update txStatusNIBP tiap 300ms
unsigned long lastNIBPPoll        = 0;
#define NIBP_POLL_MS 300UL

// ── BATTERY WAV STATE VARIABLES ─────────────────────────────
bool          batWavActive  = false;
bool          batWavPlaying = false;
unsigned long batWavTimer   = 0;
#define BAT_WAV_ON_MS  1850UL
#define BAT_WAV_OFF_MS 6500UL

// [+V18] Timer update baterai slave
unsigned long lastBatSlUpdate = 0;

// [+V19] Deteksi slave SpO2 online/offline
unsigned long lastRecvSpO2Ms = 0;
#define SPO2_SLAVE_TIMEOUT 10000UL

// ============================================================
// RECORDING VARIABLES
// ============================================================

bool isRecording = false;
String recordingId = "";
unsigned long recStartMs = 0;
unsigned long sampleCount = 0;

// ============================================================
// BATTERY MONITOR VARIABLES
// ============================================================

#define BAT_PIN       2
#define BAT_SAMPLES   64
#define BAT_INTERVAL  30000UL
#define BAT_BLINK_MS  500UL

unsigned long lastBatTime   = 0;
unsigned long lastBlinkTime = 0;
bool          batFirstRead  = true;
bool          batBlinkState = false;
bool          batCritical   = false;

// ============================================================
// LED RGB VARIABLES
// ============================================================

#define LED_RED      4
#define LED_GREEN    5
#define LED_BLINK_MS 250UL

unsigned long lastLedBlinkTime = 0;
bool          ledBlinkState    = false;

// ============================================================
// GATEWAY STATUS VARIABLES
// ============================================================

String gatewayRxBuffer = "";

// ============================================================
// LED RGB FUNCTIONS
// ============================================================

void updateLED(unsigned long currentTime) {
  if (!alarmEnabled) {
    digitalWrite(LED_RED,   LOW);
    digitalWrite(LED_GREEN, LOW);
    return;
  }

  bool anyAlarm = hrAlarmActive || spo2AlarmActive || rrAlarmActive;

  if (!anyAlarm) {
    digitalWrite(LED_RED,   LOW);
    digitalWrite(LED_GREEN, HIGH);
  } else {
    digitalWrite(LED_GREEN, LOW);
    if (currentTime - lastLedBlinkTime >= LED_BLINK_MS) {
      lastLedBlinkTime = currentTime;
      ledBlinkState    = !ledBlinkState;
      digitalWrite(LED_RED, ledBlinkState ? HIGH : LOW);
    }
  }
}

// ============================================================
// GATEWAY STATUS HANDLER
// ============================================================

void showRecordStatus(String st) {
  if (st == "WIFI_OK") {
    nexCmd("txRecord.bco=2016");
    nexCmd("txRecord.txt=\"WiFi: OK\"");
  } else if (st == "WIFI_OFF") {
    nexCmd("txRecord.bco=50712");
    nexCmd("txRecord.txt=\"WiFi: Off\"");
  } else if (st == "UPLOADING") {
    nexCmd("txRecord.bco=65504");
    nexCmd("txRecord.txt=\"Uploading...\"");
  } else if (st == "UPLOAD_OK") {
    nexCmd("txRecord.bco=2016");
    nexCmd("txRecord.txt=\"Upload OK!\"");
  }
}

void handleGatewayStatus() {
  while (gatewaySerial.available()) {
    char c = (char)gatewaySerial.read();
    if (c == '\n') {
      gatewayRxBuffer.trim();
      if (gatewayRxBuffer.length() > 0) {
        Serial.println("GW>> " + gatewayRxBuffer);
        int idx = gatewayRxBuffer.indexOf("\"st\":\"");
        if (idx >= 0) {
          int start = idx + 6;
          int end   = gatewayRxBuffer.indexOf("\"", start);
          if (end > start) {
            String st = gatewayRxBuffer.substring(start, end);
            showRecordStatus(st);
          }
        }
      }
      gatewayRxBuffer = "";
    } else {
      gatewayRxBuffer += c;
    }
  }
}

// ============================================================
// BATTERY MONITOR FUNCTIONS
// ============================================================

int getBatPercent(float vadc) {
  if (vadc >= 3.07f) return 100;
  if (vadc >= 2.83f) return 75;
  if (vadc >= 2.59f) return 50;
  if (vadc >= 2.36f) return 25;
  return 0;
}

void updateBattery() {
  if (isRecording) return;

  long sum = 0;
  for (int i = 0; i < BAT_SAMPLES; i++) {
    sum += analogRead(BAT_PIN);
    delayMicroseconds(100);
  }
  float vadc = ((float)(sum / BAT_SAMPLES) / 4095.0f) * 3.3f;
  int   pct  = getBatPercent(vadc);

  batCritical = (pct <= 25);

  if (!batCritical) {
    nexCmd("txStatusBat.bco=50712");
    nexCmd("txStatusBat.txt=\"BAT-M=" + String(pct) + "%\"");
  }

  Serial.println("BAT: " + String(vadc, 3) + "V = " + String(pct) + "%"
                 + (batCritical ? " [CRITICAL!]" : ""));
}

void blinkBattery(unsigned long currentTime) {
  if (!batCritical) return;

  if (currentTime - lastBlinkTime >= BAT_BLINK_MS) {
    lastBlinkTime = currentTime;
    batBlinkState = !batBlinkState;

    if (batBlinkState) {
      nexCmd("txStatusBat.bco=63488");
      nexCmd("txStatusBat.txt=\"BAT: 0%\"");
    } else {
      nexCmd("txStatusBat.bco=0");
      nexCmd("txStatusBat.txt=\"BAT: 0%\"");
    }
  }
}

// ============================================================
// NEXTION COMMUNICATION
// ============================================================

void nexCmd(String cmd) {
  nexSerial.print(cmd);
  nexSerial.write(0xFF);
  nexSerial.write(0xFF);
  nexSerial.write(0xFF);
}

// ============================================================
// SPEAKER CONTROL
// ============================================================

void speakerON() {
  if (!speakerOn && !speakerMuted) {
    nexCmd("wav0.en=1");
    speakerOn = true;
    Serial.println("🔊 SPEAKER ALARM ON!");
  }
}

void speakerOFF() {
  if (speakerOn) {
    nexCmd("wav0.en=0");
    speakerOn = false;
    Serial.println("🔇 SPEAKER ALARM OFF");
  }
}

// ============================================================
// LEAD SELECTOR FUNCTIONS
// ============================================================

void applyLeadMode() {
  nexCmd("cle 1,0");

  switch(leadMode) {
    case 0:
      nexCmd("txNameEcg.txt=\"ECG LEAD 1\"");
      nexCmd("bLeadS.txt=\"LEAD 1\"");
      Serial.println("📊 Lead: ECG LEAD 1");
      break;
    case 1:
      nexCmd("txNameEcg.txt=\"ECG LEAD 2\"");
      nexCmd("bLeadS.txt=\"LEAD 2\"");
      Serial.println("📊 Lead: ECG LEAD 2");
      break;
    case 2:
      nexCmd("txNameEcg.txt=\"ECG LEAD 3\"");
      nexCmd("bLeadS.txt=\"LEAD 3\"");
      Serial.println("📊 Lead: ECG LEAD 3");
      break;
  }
}

void handleLeadSelectorButton() {
  leadMode = (leadMode + 1) % 3;
  applyLeadMode();

  lead_command cmd;
  cmd.activeLead = leadMode;
  esp_now_send(macSlaveECG, (uint8_t*)&cmd, sizeof(cmd));
  Serial.println("📡 Kirim perintah lead ke Slave ECG: Lead " + String(leadMode + 1));
}

// ============================================================
// ESP-NOW CALLBACK
// ============================================================

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  const uint8_t *mac_addr = info->src_addr;
  if (len < (int)sizeof(struct_message)) return;

  struct_message temp;
  memcpy(&temp, incomingData, sizeof(temp));

  portENTER_CRITICAL_ISR(&dataMux);

  if (memcmp(mac_addr, macSlaveECG, 6) == 0) {
    dataTerima.ecgLead1 = temp.ecgLead1;
    dataTerima.ecgLead2 = temp.ecgLead2;
    dataTerima.ecgLead3 = temp.ecgLead3;
    dataTerima.leadOff  = temp.leadOff;
    ecgDataUpdated      = true;

  } else if (memcmp(mac_addr, macSlaveHRRR, 6) == 0) {
    dataTerima.HRx      = temp.HRx;
    dataTerima.RRx      = temp.RRx;
    dataTerima.respWave = temp.respWave;

  } else if (memcmp(mac_addr, macSlaveNIBP, 6) == 0) {
    dataTerima.SYSx          = temp.SYSx;
    dataTerima.DIAx          = temp.DIAx;
    dataTerima.tekananRT     = temp.tekananRT;
    dataTerima.faseNIBP      = temp.faseNIBP;
    dataTerima.cntMenit      = temp.cntMenit;
    dataTerima.korotkoffBeat = temp.korotkoffBeat;
    // [TAMBAH] nibpActive hanya true saat fase bukan WAITING (0)
    nibpActive               = (temp.faseNIBP != 0);
    // Tidak set nibpDataUpdated — update via polling di loop()

  } else if (memcmp(mac_addr, macSlaveSpO2, 6) == 0) {
    dataTerima.SpO2x    = temp.SpO2x;
    dataTerima.Tempx    = temp.Tempx;
    dataTerima.spo2Wave = temp.spo2Wave;
    dataTerima.battPct  = temp.battPct;
    lastRecvSpO2Ms      = millis();

  } else {
    portEXIT_CRITICAL_ISR(&dataMux);
    return;
  }

  portEXIT_CRITICAL_ISR(&dataMux);

  dataReceived   = true;
  gatewayPending = true;
  packetCount++;
}

// ============================================================
// ALARM FUNCTIONS
// ============================================================

void checkAlarms() {
  if (!alarmEnabled) {
    hrAlarmActive = false;
    spo2AlarmActive = false;
    rrAlarmActive = false;
    speakerOFF();
    return;
  }

  int hr, spo2, rr;
  portENTER_CRITICAL(&dataMux);
    hr = dataTerima.HRx; spo2 = dataTerima.SpO2x; rr = dataTerima.RRx;
  portEXIT_CRITICAL(&dataMux);

  bool prevHrAlarm   = hrAlarmActive;
  bool prevSpo2Alarm = spo2AlarmActive;
  bool prevRrAlarm   = rrAlarmActive;

  hrAlarmActive   = (hr   > thresholds.hrMax   || hr   < thresholds.hrMin);
  spo2AlarmActive = (spo2 > thresholds.spo2Max || spo2 < thresholds.spo2Min);
  rrAlarmActive   = (rr   > thresholds.rrMax   || rr   < thresholds.rrMin);

  bool anyAlarmActive = hrAlarmActive || spo2AlarmActive || rrAlarmActive;

  if (anyAlarmActive && !speakerMuted) {
    speakerON();
  } else {
    speakerOFF();
  }

  if (hrAlarmActive && !prevHrAlarm) {
    Serial.println("⚠️ HR ALARM! Value: " + String(hr) + " (Range: " +
                   String(thresholds.hrMin) + "-" + String(thresholds.hrMax) + ")");
  } else if (!hrAlarmActive && prevHrAlarm) {
    Serial.println("✓ HR Normal: " + String(hr));
  }

  if (spo2AlarmActive && !prevSpo2Alarm) {
    Serial.println("⚠️ SpO2 ALARM! Value: " + String(spo2) + " (Range: " +
                   String(thresholds.spo2Min) + "-" + String(thresholds.spo2Max) + ")");
  } else if (!spo2AlarmActive && prevSpo2Alarm) {
    Serial.println("✓ SpO2 Normal: " + String(spo2));
  }

  if (rrAlarmActive && !prevRrAlarm) {
    Serial.println("⚠️ RR ALARM! Value: " + String(rr) + " (Range: " +
                   String(thresholds.rrMin) + "-" + String(thresholds.rrMax) + ")");
  } else if (!rrAlarmActive && prevRrAlarm) {
    Serial.println("✓ RR Normal: " + String(rr));
  }
}

// ============================================================
// DISPLAY UPDATE
// ============================================================

void updateDisplay() {
  struct_message snap;
  portENTER_CRITICAL(&dataMux);
    memcpy(&snap, &dataTerima, sizeof(snap));
  portEXIT_CRITICAL(&dataMux);

  unsigned long now = millis();

  static unsigned long lastTextUpdate = 0;
  if (now - lastTextUpdate >= 500) {
    lastTextUpdate = now;
    nexCmd("xSYS.txt=\""  + String(snap.SYSx)  + "\"");
    nexCmd("xDIA.txt=\""  + String(snap.DIAx)  + "\"");
    nexCmd("xTemp.txt=\"" + String(snap.Tempx / 10) + "." + String(snap.Tempx % 10) + "\"");
    nexCmd("xSpO2.txt=\"" + String(snap.SpO2x) + "\"");
    nexCmd("xHR.txt=\""   + String(snap.HRx)   + "\"");
    nexCmd("xRR.txt=\""   + String(snap.RRx)   + "\"");
  }

  if (now - lastEcgUpdate >= ecgInterval) {
    lastEcgUpdate = now;
    int rawEcg = 0;
    switch(leadMode) {
      case 0: rawEcg = snap.ecgLead1; break;
      case 1: rawEcg = snap.ecgLead2; break;
      case 2: rawEcg = snap.ecgLead3; break;
    }
    emaEcg = alphaEcg * rawEcg + (1.0f - alphaEcg) * emaEcg;
    nexCmd("add 1,0," + String((int)(emaEcg + 0.5f)));
  }

  if (now - lastSpo2Update >= spo2Interval) {
    lastSpo2Update = now;
    emaSpo2 = alphaSpo2 * snap.spo2Wave + (1.0f - alphaSpo2) * emaSpo2;
    nexCmd("add 3,0," + String((int)(emaSpo2 + 0.5f)));
  }

  if (now - lastRespUpdate >= respInterval) {
    lastRespUpdate = now;
    emaResp = alphaResp * snap.respWave + (1.0f - alphaResp) * emaResp;
    nexCmd("add 5,0," + String((int)(emaResp + 0.5f)));
  }
}

// ============================================================
// READ THRESHOLD VALUES
// ============================================================

void readThresholdValues() {
  Serial.println("\n📖 Reading threshold values...");

  while(nexSerial.available()) nexSerial.read();

  nexCmd("get txbaHR.txt");
  delay(100);
  thresholds.hrMax = readNextionResponse();

  nexCmd("get txbbHR.txt");
  delay(100);
  thresholds.hrMin = readNextionResponse();

  nexCmd("get txbaSpO2.txt");
  delay(100);
  thresholds.spo2Max = readNextionResponse();

  nexCmd("get txbbSpO2.txt");
  delay(100);
  thresholds.spo2Min = readNextionResponse();

  nexCmd("get txbaRR.txt");
  delay(100);
  thresholds.rrMax = readNextionResponse();

  nexCmd("get txbbRR.txt");
  delay(100);
  thresholds.rrMin = readNextionResponse();

  Serial.println("✓ Threshold updated!");
  Serial.println("   HR:   " + String(thresholds.hrMin) + "-" + String(thresholds.hrMax));
  Serial.println("   SpO2: " + String(thresholds.spo2Min) + "-" + String(thresholds.spo2Max));
  Serial.println("   RR:   " + String(thresholds.rrMin) + "-" + String(thresholds.rrMax) + "\n");
}

int readNextionResponse() {
  unsigned long startTime = millis();
  String valueStr = "";

  while (millis() - startTime < 200) {
    if (nexSerial.available()) {
      byte b = nexSerial.read();
      if (b == 0x70) {
        while (nexSerial.available()) {
          byte c = nexSerial.read();
          if (c == 0xFF) break;
          if (c >= '0' && c <= '9') valueStr += (char)c;
        }
        while (nexSerial.available() && nexSerial.read() == 0xFF);
        if (valueStr.length() > 0) return valueStr.toInt();
      }
    }
  }
  return 0;
}

// ============================================================
// SLIDER LOCK/UNLOCK
// ============================================================

void lockSliderUpdate() {
  nexCmd("tsw baHR,0");
  nexCmd("tsw bbHR,0");
  nexCmd("tsw baSpO2,0");
  nexCmd("tsw bbSpO2,0");
  nexCmd("tsw baRR,0");
  nexCmd("tsw bbRR,0");
  Serial.println("🔒 Sliders LOCKED\n");
}

void unlockSliderUpdate() {
  nexCmd("tsw baHR,1");
  nexCmd("tsw bbHR,1");
  nexCmd("tsw baSpO2,1");
  nexCmd("tsw bbSpO2,1");
  nexCmd("tsw baRR,1");
  nexCmd("tsw bbRR,1");
  Serial.println("🔓 Sliders UNLOCKED\n");
}

// ============================================================
// RECORDING FUNCTIONS
// ============================================================

void startRecording() {
  if (isRecording) return;

  recordingId = "REC_" + String(millis() / 1000);
  isRecording = true;
  recStartMs  = millis();
  sampleCount = 0;

  String json = "{\"cmd\":\"START\",\"recId\":\"" + recordingId + "\"}";
  gatewaySerial.println(json);

  Serial.println("\n🔴 RECORDING: " + recordingId);

  nexCmd("bStartR.bco=63488");
  nexCmd("bStopR.bco=2016");
}

void stopRecording() {
  if (!isRecording) return;

  unsigned long duration = (millis() - recStartMs) / 1000;

  String json = "{\"cmd\":\"STOP\",\"samples\":" + String(sampleCount)
              + ",\"duration\":" + String(duration) + "}";
  gatewaySerial.println(json);

  isRecording = false;

  Serial.println("⏹️  STOPPED: " + String(sampleCount) + " samples");

  nexCmd("bStartR.bco=2016");
  nexCmd("bStopR.bco=50712");
}

// ============================================================
// SETUP PAGE 1
// ============================================================

void setupPage1() {
  Serial.println("🖥️  Setup Page1...");

  nexCmd("xSYS.txt=\"---\"");
  nexCmd("xDIA.txt=\"---\"");
  nexCmd("xTemp.txt=\"--\"");
  nexCmd("xSpO2.txt=\"--\"");
  nexCmd("xHR.txt=\"--\"");
  nexCmd("xRR.txt=\"--\"");

  nexCmd("baHR.val=100");
  nexCmd("bbHR.val=60");
  nexCmd("baSpO2.val=100");
  nexCmd("bbSpO2.val=95");
  nexCmd("baRR.val=20");
  nexCmd("bbRR.val=12");

  nexCmd("txbaHR.txt=\"100\"");
  nexCmd("txbbHR.txt=\"60\"");
  nexCmd("txbaSpO2.txt=\"100\"");
  nexCmd("txbbSpO2.txt=\"95\"");
  nexCmd("txbaRR.txt=\"20\"");
  nexCmd("txbbRR.txt=\"12\"");

  unlockSliderUpdate();

  nexCmd("bSetA.val=0");
  nexCmd("bSetA.bco=50712");
  nexCmd("bSetA.txt=\"SET ALARM\"");

  nexCmd("btMA.val=0");
  nexCmd("btMA.bco=63488");
  nexCmd("btMA.txt=\"ALARM\"");
  speakerMuted = false;

  nexCmd("wav0.en=0");

  nexCmd("bStartR.bco=2016");
  nexCmd("bStartR.txt=\"RECORD\"");
  nexCmd("bStopR.bco=50712");
  nexCmd("bStopR.txt=\"STOP\"");

  leadMode = 0;
  applyLeadMode();

  for (int i = 0; i < 60; i++) {
    nexCmd("add 1,0,128");
    nexCmd("add 3,0,128");
    nexCmd("add 5,0,128");
    delay(5);
  }

  analogReadResolution(12);
  nexCmd("txStatusBat.txt=\"BAT: --\"");
  nexCmd("txRecord.bco=50712");
  nexCmd("txRecord.txt=\"WiFi: --\"");

  nexCmd("txStatusNIBP.bco=0");
  nexCmd("txStatusNIBP.pco=63488");
  nexCmd("txStatusNIBP.txt=\"WAITING\"");
  lastFaseNIBP   = 0;
  nibpBlinkState = false;
  sistoleFlash   = false;
  diastoleFlash  = false;

  nexCmd("txBatSL.bco=50712");
  nexCmd("txBatSL.pco=0");
  nexCmd("txBatSL.txt=\"BAT-SL=WAIT\"");

  Serial.println("✓ Page1 Ready!\n");
  Serial.println("📊 Interval — ECG:" + String(ecgInterval) + "ms"
                + " SpO2:" + String(spo2Interval) + "ms"
                + " Resp:" + String(respInterval) + "ms");
}

// ============================================================
// NIBP TRIGGER
// ============================================================

void handleStartNIBPButton() {
  Serial.println("🩺 START NIBP AUTO MODE → trigger ke Slave NIBP");
  struct_message nibpCmd;
  memset(&nibpCmd, 0, sizeof(nibpCmd));
  nibpCmd.SYSx = 1;
  esp_now_send(macSlaveNIBP, (uint8_t*)&nibpCmd, sizeof(nibpCmd));
}

void handleStopNIBPButton() {
  Serial.println("🛑 STOP NIBP → trigger ke Korotkoff");
  struct_message nibpCmd;
  memset(&nibpCmd, 0, sizeof(nibpCmd));
  nibpCmd.SYSx = 99;
  esp_now_send(macSlaveNIBP, (uint8_t*)&nibpCmd, sizeof(nibpCmd));
}

void sendDataToGateway() {
  struct_message snap;
  portENTER_CRITICAL(&dataMux);
    memcpy(&snap, &dataTerima, sizeof(snap));
  portEXIT_CRITICAL(&dataMux);

  unsigned long ts = millis() - recStartMs;

  int recLead1 = (leadMode == 0) ? snap.ecgLead1 : 0;
  int recLead2 = (leadMode == 1) ? snap.ecgLead2 : 0;
  int recLead3 = (leadMode == 2) ? snap.ecgLead3 : 0;

  String json = "{\"cmd\":\"DATA\"";
  json += ",\"ts\":"  + String(ts);
  json += ",\"e1\":"  + String(recLead1);
  json += ",\"e2\":"  + String(recLead2);
  json += ",\"e3\":"  + String(recLead3);
  json += ",\"sw\":"  + String(snap.spo2Wave);
  json += ",\"rw\":"  + String(snap.respWave);
  json += "}";

  gatewaySerial.println(json);
  sampleCount++;
}

// ============================================================
// NEXTION EVENT HANDLER
// ============================================================

void handleNextionEvents() {
  static byte buffer[10];
  static int bufferIndex = 0;
  static int ffCount = 0;

  while (nexSerial.available()) {
    byte inByte = nexSerial.read();

    if (inByte == 0xFF) {
      ffCount++;
      if (ffCount >= 3 && bufferIndex > 0) {
        processNextionMessage(buffer, bufferIndex);
        bufferIndex = 0;
        ffCount = 0;
      }
    } else {
      ffCount = 0;
      if (bufferIndex < 10) buffer[bufferIndex++] = inByte;
    }
  }
}

void processNextionMessage(byte* data, int length) {
  if (length < 1) return;

  byte eventType = data[0];

  if (eventType == 0x70) {
    String msg = "";
    for (int i = 1; i < length; i++) {
      if (data[i] != 0xFF) msg += (char)data[i];
    }
    if (msg.indexOf("PAGE1_READY") >= 0) {
      Serial.println("✅ Nextion: PAGE1_READY diterima");
      setupPage1();
    }
    return;
  }

  if (eventType == 0x65 && length >= 4) {
    byte pageId      = data[1];
    byte componentId = data[2];
    byte touchEvent  = data[3];

    if (componentId == 39 && touchEvent == 0x01) {
      handleSetAlarmButton();
    }
    else if (componentId == 34 && touchEvent == 0x01) {
      handleMuteAlarmButton();
    }
    else if (componentId == 35 && touchEvent == 0x01) {
      handleLeadSelectorButton();
    }
    else if (componentId == 37 && touchEvent == 0x01) {
      startRecording();
    }
    else if (componentId == 38 && touchEvent == 0x01) {
      stopRecording();
    }
    else if (componentId == 36 && touchEvent == 0x01) {
      handleStartNIBPButton();
    }
    else if (componentId == 40 && touchEvent == 0x01) {
      handleStopNIBPButton();
    }
  }
}

// ============================================================
// BUTTON HANDLERS
// ============================================================

void handleSetAlarmButton() {
  alarmEnabled = !alarmEnabled;

  if (alarmEnabled) {
    Serial.println("\n╔════════════════════════════════════╗");
    Serial.println("║  ✅ ALARM ENABLED!                ║");
    Serial.println("╚════════════════════════════════════╝");

    readThresholdValues();
    lockSliderUpdate();

    nexCmd("bSetA.val=1");
    nexCmd("bSetA.bco=2016");
    nexCmd("bSetA.txt=\"SETALARM\"");

    Serial.println("🔊 Speaker: " + String(speakerMuted ? "MUTED" : "READY"));
    Serial.println("════════════════════════════════════\n");

  } else {
    Serial.println("\n╔════════════════════════════════════╗");
    Serial.println("║  ❌ ALARM DISABLED                ║");
    Serial.println("╚════════════════════════════════════╝\n");

    unlockSliderUpdate();
    speakerOFF();

    nexCmd("bSetA.val=0");
    nexCmd("bSetA.bco=50712");
    nexCmd("bSetA.txt=\"SET ALARM\"");
  }
}

void handleMuteAlarmButton() {
  speakerMuted = !speakerMuted;

  if (speakerMuted) {
    Serial.println("\n🔇 SPEAKER MUTED!");
    Serial.println("   Alarm tetap aktif, tapi speaker OFF\n");

    speakerOFF();

    nexCmd("btMA.val=1");
    nexCmd("btMA.bco=2016");
    nexCmd("btMA.txt=\"MUTED\"");

  } else {
    Serial.println("\n🔊 SPEAKER UNMUTED!");
    Serial.println("   Speaker akan bunyi jika ada alarm\n");

    nexCmd("btMA.val=0");
    nexCmd("btMA.bco=63488");
    nexCmd("btMA.txt=\"ALARM\"");

    if (alarmEnabled) checkAlarms();
  }
}

// ============================================================
// UPDATE LEAD DETEC
// ============================================================

void updateLeadDetec() {
  static unsigned long lastLeadUpdate = 0;
  unsigned long now = millis();
  if (now - lastLeadUpdate < 1000) return;
  lastLeadUpdate = now;

  int leadOff;
  portENTER_CRITICAL(&dataMux);
    leadOff = dataTerima.leadOff;
  portEXIT_CRITICAL(&dataMux);

  if (leadOff == 0) {
    nexCmd("txLeadDetec.bco=2016");
    nexCmd("txLeadDetec.pco=0");
    nexCmd("txLeadDetec.txt=\"LEAD OK\"");
  } else {
    String txt = "";
    if (leadOff & LEADOFF_LA) txt += "LA ";
    if (leadOff & LEADOFF_RA) txt += "RA ";
    if (leadOff & LEADOFF_LL) txt += "LL ";
    if (leadOff & LEADOFF_RL) txt += "RL";
    txt.trim();
    nexCmd("txLeadDetec.bco=63488");
    nexCmd("txLeadDetec.pco=65535");
    nexCmd("txLeadDetec.txt=\"" + txt + "\"");
  }
}

// ============================================================
// UPDATE NIBP STATUS
// ============================================================

void updateNIBPStatus() {
  int  fase, tek, menit, sys, dia, beat;
  portENTER_CRITICAL(&dataMux);
    fase   = dataTerima.faseNIBP;
    tek    = dataTerima.tekananRT;
    menit  = dataTerima.cntMenit;
    sys    = dataTerima.SYSx;
    dia    = dataTerima.DIAx;
    beat   = dataTerima.korotkoffBeat;
  portEXIT_CRITICAL(&dataMux);

  unsigned long now = millis();

  if (fase == 4 && lastFaseNIBP != 4) {
    sistoleFlash      = true;
    sistoleFlashStart = now;
  }
  if (fase == 5 && lastFaseNIBP != 5) {
    diastoleFlash      = true;
    diastoleFlashStart = now;
  }
  lastFaseNIBP = fase;

  if (sistoleFlash) {
    if (now - sistoleFlashStart < FLASH_DURATION) {
      nexCmd("txStatusNIBP.bco=0");
      nexCmd(nibpBlinkState ? "txStatusNIBP.pco=2016" : "txStatusNIBP.pco=63488");
      if (now - lastNIBPBlinkTime >= NIBP_BLINK_MS) {
        lastNIBPBlinkTime = now;
        nibpBlinkState    = !nibpBlinkState;
      }
      nexCmd("txStatusNIBP.txt=\"" + String(tek) + "\"");
      return;
    } else {
      sistoleFlash   = false;
      nibpBlinkState = false;
      nexCmd("txStatusNIBP.pco=63488");
    }
  }

  if (diastoleFlash) {
    if (now - diastoleFlashStart < FLASH_DURATION) {
      nexCmd("txStatusNIBP.bco=0");
      nexCmd(nibpBlinkState ? "txStatusNIBP.pco=65504" : "txStatusNIBP.pco=63488");
      if (now - lastNIBPBlinkTime >= NIBP_BLINK_MS) {
        lastNIBPBlinkTime = now;
        nibpBlinkState    = !nibpBlinkState;
      }
      nexCmd("txStatusNIBP.txt=\"" + String(tek) + "\"");
      return;
    } else {
      diastoleFlash  = false;
      nibpBlinkState = false;
      nexCmd("txStatusNIBP.pco=63488");
    }
  }

  switch (fase) {
    case 0:
      nexCmd("txStatusNIBP.bco=0");
      nexCmd("txStatusNIBP.pco=63488");
      nexCmd("txStatusNIBP.txt=\"WAITING\"");
      break;
    case 1:
      nexCmd("txStatusNIBP.bco=0");
      nexCmd("txStatusNIBP.pco=63488");
      nexCmd("txStatusNIBP.txt=\"0\"");
      break;
    case 2:
      nexCmd("txStatusNIBP.bco=0");
      nexCmd("txStatusNIBP.pco=63488");
      nexCmd("txStatusNIBP.txt=\"^ " + String(tek) + "\"");
      break;
    case 3:
      nexCmd("txStatusNIBP.bco=0");
      nexCmd("txStatusNIBP.pco=63488");
      if (beat == 1) {
        nexCmd("txStatusNIBP.txt=\"v " + String(tek) + " *\"");
        portENTER_CRITICAL(&dataMux);
          dataTerima.korotkoffBeat = 0;
        portEXIT_CRITICAL(&dataMux);
      } else {
        nexCmd("txStatusNIBP.txt=\"v " + String(tek) + "\"");
      }
      break;
    case 6:
      nexCmd("txStatusNIBP.bco=0");
      nexCmd("txStatusNIBP.pco=63488");
      nexCmd("txStatusNIBP.txt=\"" + String(sys) + "/" + String(dia) + "\"");
      break;
    case 7:
      nexCmd("txStatusNIBP.bco=0");
      nexCmd("txStatusNIBP.pco=63488");
      nexCmd("txStatusNIBP.txt=\"" + String(menit) + " MENIT\"");
      break;
    default:
      break;
  }
}

// ============================================================
// UPDATE BATERAI SLAVE
// ============================================================

void updateBatSlave() {
  int pct;
  portENTER_CRITICAL(&dataMux);
    pct = dataTerima.battPct;
  portEXIT_CRITICAL(&dataMux);

  bool spo2Connected = (lastRecvSpO2Ms > 0 &&
                        (millis() - lastRecvSpO2Ms) < SPO2_SLAVE_TIMEOUT);

  if (!spo2Connected) {
    nexCmd("txBatSL.bco=50712");
    nexCmd("txBatSL.pco=0");
    nexCmd("txBatSL.txt=\"BAT-SL=WAIT\"");
    Serial.println("🔋 BAT-SL: WAITING (slave offline)");
    return;
  }

  Serial.println("🔋 BAT-SL: " + String(pct) + "%");

  String txt = "BAT-SL=" + String(pct) + "%";

  if (pct >= 75) {
    nexCmd("txBatSL.bco=2016");
    nexCmd("txBatSL.pco=0");
  } else if (pct >= 50) {
    nexCmd("txBatSL.bco=65504");
    nexCmd("txBatSL.pco=0");
  } else if (pct >= 25) {
    nexCmd("txBatSL.bco=64512");
    nexCmd("txBatSL.pco=0");
  } else {
    nexCmd("txBatSL.bco=63488");
    nexCmd("txBatSL.pco=65535");
  }
  nexCmd("txBatSL.txt=\"" + txt + "\"");
}

// ============================================================
// UPDATE BAT WAV
// ============================================================

void updateBatWav(bool slaveKritis, bool masterKritis) {
  bool anyKritis = slaveKritis || masterKritis;
  unsigned long now = millis();

  if (!anyKritis) {
    if (batWavActive) {
      nexCmd("wav1.en=0");
      batWavActive  = false;
      batWavPlaying = false;
      Serial.println("✅ BAT WARN: cleared — wav1 OFF");
    }
    return;
  }

  if (!batWavActive) {
    nexCmd("wav1.en=1");
    batWavActive  = true;
    batWavPlaying = true;
    batWavTimer   = now;
    Serial.println("🔔 BAT WARN: wav1 ON"
                   " (slave=" + String(slaveKritis) +
                   " master=" + String(masterKritis) + ")");
    return;
  }

  if (batWavPlaying) {
    if (now - batWavTimer >= BAT_WAV_ON_MS) {
      nexCmd("wav1.en=0");
      batWavPlaying = false;
      batWavTimer   = now;
      Serial.println("🔕 BAT WARN: wav1 OFF (interval)");
    }
  } else {
    if (now - batWavTimer >= BAT_WAV_OFF_MS) {
      nexCmd("wav1.en=1");
      batWavPlaying = true;
      batWavTimer   = now;
      Serial.println("🔔 BAT WARN: wav1 ON (resume)");
    }
  }
}

// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║  ESP32 MASTER + RECORDING - FIXED     ║");
  Serial.println("╚════════════════════════════════════════╝\n");

  nexSerial.begin(250000, SERIAL_8N1, NEXTION_RX, NEXTION_TX);
  while(nexSerial.available()) nexSerial.read();
  Serial.println("✅ Nextion OK");

  pinMode(LED_RED,   OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  digitalWrite(LED_RED,   LOW);
  digitalWrite(LED_GREEN, LOW);
  Serial.println("✅ LED RGB OK (R:GPIO4 G:GPIO5)");

  gatewaySerial.begin(115200, SERIAL_8N1, GATEWAY_RX, GATEWAY_TX);
  Serial.println("✅ Gateway OK (TX:" + String(GATEWAY_TX) + " RX:" + String(GATEWAY_RX) + ")");

  WiFi.mode(WIFI_STA);
  Serial.println("📡 MAC: " + WiFi.macAddress() + "\n");

  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ ESP-NOW FAILED!");
    while(1) delay(1000);
  }

  esp_now_register_recv_cb(OnDataRecv);
  Serial.println("✅ ESP-NOW OK\n");

  esp_now_peer_info_t peerNIBP = {};
  memcpy(peerNIBP.peer_addr, macSlaveNIBP, 6);
  peerNIBP.channel = 0;
  peerNIBP.encrypt = false;
  if (esp_now_add_peer(&peerNIBP) != ESP_OK) {
    Serial.println("❌ Add peer NIBP FAILED!");
  } else {
    Serial.println("✅ Peer NIBP OK");
  }

  esp_now_peer_info_t peerECG = {};
  memcpy(peerECG.peer_addr, macSlaveECG, 6);
  peerECG.channel = 0;
  peerECG.encrypt = false;
  if (esp_now_add_peer(&peerECG) != ESP_OK) {
    Serial.println("❌ Add peer ECG FAILED!");
  } else {
    Serial.println("✅ Peer ECG OK");
  }

  memset(&dataTerima, 0, sizeof(dataTerima));

  Serial.println("🖥️  Nextion Setup...");

  nexCmd("bkcmd=1");
  delay(100);
  nexCmd("page 0");
  delay(1000);

  nexCmd("xSYS.txt=\"---\"");
  nexCmd("xDIA.txt=\"---\"");
  nexCmd("xTemp.txt=\"--\"");
  nexCmd("xSpO2.txt=\"--\"");
  nexCmd("xHR.txt=\"--\"");
  nexCmd("xRR.txt=\"--\"");

  nexCmd("baHR.val=100");
  nexCmd("bbHR.val=60");
  nexCmd("baSpO2.val=100");
  nexCmd("bbSpO2.val=95");
  nexCmd("baRR.val=20");
  nexCmd("bbRR.val=12");

  nexCmd("txbaHR.txt=\"100\"");
  nexCmd("txbbHR.txt=\"60\"");
  nexCmd("txbaSpO2.txt=\"100\"");
  nexCmd("txbbSpO2.txt=\"95\"");
  nexCmd("txbaRR.txt=\"20\"");
  nexCmd("txbbRR.txt=\"12\"");

  unlockSliderUpdate();

  nexCmd("bSetA.val=0");
  nexCmd("bSetA.bco=50712");
  nexCmd("bSetA.txt=\"SET ALARM\"");

  nexCmd("btMA.val=0");
  nexCmd("btMA.bco=63488");
  nexCmd("btMA.txt=\"ALARM\"");
  speakerMuted = false;

  nexCmd("wav0.en=0");

  nexCmd("bStartR.bco=2016");
  nexCmd("bStartR.txt=\"RECORD\"");
  nexCmd("bStopR.bco=50712");
  nexCmd("bStopR.txt=\"STOP\"");

  leadMode = 0;
  applyLeadMode();

  nexCmd("add 1,0,0");
  nexCmd("add 3,0,0");
  nexCmd("add 5,0,0");

  nexCmd("txStatusNIBP.bco=0");
  nexCmd("txStatusNIBP.pco=63488");
  nexCmd("txStatusNIBP.txt=\"WAITING\"");

  Serial.println("✓ Ready!\n");
  Serial.println("📊 Interval — ECG:" + String(ecgInterval) + "ms"
                 + " SpO2:" + String(spo2Interval) + "ms"
                 + " Resp:" + String(respInterval) + "ms");

  analogReadResolution(12);
  nexCmd("txStatusBat.txt=\"BAT: --\"");
  nexCmd("txRecord.bco=50712");
  nexCmd("txRecord.txt=\"WiFi: --\"");

  lastStatsTime = millis();
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop() {
  unsigned long currentTime = millis();

  handleNextionEvents();
  handleGatewayStatus();

  if (isRecording && gatewayPending) {
    unsigned long now = millis();
    if (now - lastGatewaySend >= GATEWAY_MIN_INTERVAL) {
      lastGatewaySend = now;
      gatewayPending  = false;
      sendDataToGateway();
    } else {
      gatewayPending = false;
    }
  }

  updateDisplay();

  // [TAMBAH] Polling NIBP tiap 300ms — tidak bergantung interrupt
  // Hanya update saat fase NIBP aktif
    if (currentTime - lastNIBPPoll >= NIBP_POLL_MS) {
      lastNIBPPoll = currentTime;
      if (nibpActive) {
        updateNIBPStatus();
        nibpWasActive = true;
      } else if (nibpWasActive) {
        nibpWasActive = false;
        updateNIBPStatus();
  }
}
  if (nibpDataUpdated) {
    nibpDataUpdated = false;
  }
  if (ecgDataUpdated) {
    ecgDataUpdated = false;
    updateLeadDetec();
  }
  if (korotkoffUpdated) {
    korotkoffUpdated = false;
  }

  dataReceived = false;

  if (currentTime - lastAlarmCheckTime >= alarmCheckInterval) {
    lastAlarmCheckTime = currentTime;
    if (packetCount > 0 && alarmEnabled) checkAlarms();
  }

  if (currentTime - lastStatsTime >= 10000) {
    lastStatsTime = currentTime;

    Serial.println("┌──────────────────────────────────────┐");
    Serial.print  ("│ Alarm:" + String(alarmEnabled ? "ON " : "OFF"));
    Serial.print  (" Spkr:" + String(speakerMuted ? "MUTE" : "ON  "));
    Serial.println(" Rec:" + String(isRecording ? "🔴" : "⏹️") + "   │");

    if (alarmEnabled) {
      Serial.println("├──────────────────────────────────────┤");
      Serial.print  ("│ HR:" + String(hrAlarmActive ? "⚠️ " : "✓ "));
      Serial.print  (" SpO2:" + String(spo2AlarmActive ? "⚠️ " : "✓ "));
      Serial.println(" RR:" + String(rrAlarmActive ? "⚠️ " : "✓ ") + "              │");
    }

    Serial.println("└──────────────────────────────────────┘\n");

    packetCount = 0;
  }

  if (!isRecording) {
    if (batFirstRead && currentTime >= 10000) {
      batFirstRead = false;
      lastBatTime  = currentTime;
      updateBattery();
    } else if (!batFirstRead && (currentTime - lastBatTime >= BAT_INTERVAL)) {
      lastBatTime = currentTime;
      updateBattery();
    }
  }

  updateLED(currentTime);
  blinkBattery(currentTime);

  if (currentTime - lastBatSlUpdate >= 5000) {
    lastBatSlUpdate = currentTime;
    updateBatSlave();
  }

  {
    bool spo2Connected = (lastRecvSpO2Ms > 0 &&
                          (millis() - lastRecvSpO2Ms) < SPO2_SLAVE_TIMEOUT);
    bool slaveKritis   = (spo2Connected && dataTerima.battPct <= 25);
    updateBatWav(slaveKritis, batCritical);
  }

  delay(5);
}