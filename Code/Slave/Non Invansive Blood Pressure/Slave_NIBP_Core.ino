  #include "HX710.h"
  #include <freertos/FreeRTOS.h>
  #include <freertos/task.h>
  #include <freertos/queue.h>
  #define UART_TX_PIN  17
  #define UART_RX_PIN  16
  #define UART_BAUD    115200

  typedef struct {
    int  tek;
    int  mrk;
    char fase[12];
  } UARTPayload;

  QueueHandle_t uartQueue;

  volatile bool sistoleJustDetected  = false;
  volatile bool diastoleJustDetected = false;
  volatile bool nibpStopFlag = false;

  const int DOUT      = 32;
  const int PD_SCK    = 33;
  const int motor     = 26;
  const int selenoid  = 27;
  const int selenoid2 = 25;

  const float ADCmmHg  = 8900;
  const float alpha    = 0.9;
  float resettekanan   = 0;
  float smoothedValue  = 0;
  float value          = 0;
  float mmhg = 0, mmhgx = 0;

  int sistole = 0, diastole = 0, sistolex = 0, diastolex = 0;
  int mark = 0, var = 0, diastole1 = 0, tk = 0;

  volatile bool nibpStartFlag = false;

  // ===== ADAPTIVE INFLATION =====
  float envelope       = 0;
  float maxEnvelope    = 0;
  float recentAmps[7];
  int   ampIdx         = 0;
  float last0 = 0, last1 = 0, last2 = 0;
  bool  candidateLocked   = false;
  float candidateAmp      = 0;
  float candidatePressure = 0;
  int   peakCount         = 0;
  float targetPressure    = 180;
  // ==============================

  HX710 ps;

  // ===== HELPER ADAPTIVE =====
  float updateEnvelope(float currentEnv, float oscAmp) {
    if (oscAmp > currentEnv)
      currentEnv += 0.2 * (oscAmp - currentEnv);
    else
      currentEnv -= 0.05 * (currentEnv - oscAmp);
    return currentEnv;
  }

  bool isPeak(float a, float b, float c) {
    return (b > a && b > c);
  }
  // ===========================

  void uartTask(void *pvParameters) {
    UARTPayload p;
    String rxBuf = "";

    for (;;) {
      if (xQueueReceive(uartQueue, &p, 0) == pdTRUE) {
        Serial2.print("FASE:");
        Serial2.print(p.fase);
        Serial2.print("|TEK:");
        Serial2.print(p.tek);
        Serial2.print("|MRK:");
        Serial2.println(p.mrk);
      }

      while (Serial2.available()) {
        char c = (char)Serial2.read();
        if (c == '\n') {
          rxBuf.trim();
          if (rxBuf == "CMD:START" || rxBuf == "CMD:AUTO") {
            nibpStartFlag = true;
            Serial.println("[UART RX] " + rxBuf);
          }
          if (rxBuf == "CMD:STOP") {
            nibpStopFlag = true;
            Serial.println("[UART RX] CMD:STOP diterima");
          }
          rxBuf = "";
        } else {
          rxBuf += c;
        }
      }

      vTaskDelay(5 / portTICK_PERIOD_MS);
    }
  }

  void kirimUART(const char* faseLabel) {
    UARTPayload p;
    p.tek = (int)mmhg;
    p.mrk = mark;
    strncpy(p.fase, faseLabel, sizeof(p.fase) - 1);
    p.fase[sizeof(p.fase) - 1] = '\0';
    xQueueSend(uartQueue, &p, 0);
  }

  void kirimHasil() {
    Serial2.print("HASIL:");
    Serial2.print(sistolex);
    Serial2.print("|");
    Serial2.println(diastolex);
  }

  void setup() {
    Serial.begin(115200);

    pinMode(motor,     OUTPUT);
    pinMode(selenoid,  OUTPUT);
    pinMode(selenoid2, OUTPUT);
    digitalWrite(motor,     LOW);
    digitalWrite(selenoid,  HIGH);
    digitalWrite(selenoid2, LOW);

    ps.initialize(PD_SCK, DOUT);
    Serial.println("NIBP READY");

    Serial2.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    Serial.println("UART2 OK — TX:GPIO17 RX:GPIO16");

    uartQueue = xQueueCreate(30, sizeof(UARTPayload));
    if (uartQueue == NULL) {
      Serial.println("QUEUE GAGAL!"); while(1);
    }
    Serial.println("Queue OK");

    xTaskCreatePinnedToCore(
      uartTask,
      "UART_Task",
      4096,
      NULL,
      1,
      NULL,
      0
    );
    Serial.println("UART Task OK — Core 0");
    Serial.println("NIBP V9 READY");
  }

  void loop() {
    sensorread();

    if (nibpStartFlag) {
      nibpStartFlag = false;
      startNIBP();
    }
    if (Serial.available() > 0) {
      String command = Serial.readStringUntil('\n');
      command.trim();
      command.toLowerCase();
      if (command == "start") startNIBP();
    }
    delay(20);
  }

  void startNIBP() {
    nibpStopFlag = false;

    Serial2.println("NOTIF:START");
    digitalWrite(motor,     LOW);
    digitalWrite(selenoid,  HIGH);
    digitalWrite(selenoid2, HIGH);
    delay(1000);

    kirimUART("RESET");
    resetsensor();

    for (int i = 0; i < 10; i++) {
      sensorread();
      delay(200);
    }

    mark      = 0;
    mmhgx     = mmhg;
    sistole   = 0;
    diastole  = 0;
    sistolex  = 0;
    diastolex = 0;

    // ===== RESET ADAPTIVE =====
    envelope = 0; maxEnvelope = 0;
    candidateLocked = false;
    candidateAmp = 0; candidatePressure = 0;
    peakCount = 0;
    targetPressure = 180;
    for (int i = 0; i < 7; i++) recentAmps[i] = 0;
    // ==========================

    digitalWrite(selenoid,  HIGH);
    digitalWrite(selenoid2, HIGH);
    digitalWrite(motor,     HIGH);

    mulai1();

    Serial.print("||Sis");
    Serial.print(sistolex);
    Serial.print("||Dia");
    Serial.print(diastolex);
    Serial.println("||");

    if (sistolex > 0 && diastolex > 0) {
      kirimHasil();
    }

    digitalWrite(motor,     LOW);
    digitalWrite(selenoid,  HIGH);
    digitalWrite(selenoid2, LOW);
    delay(5000);
  }

  void sensorread() {
    if (ps.isReady()) {
      ps.readAndSelectNextData(HX710_DIFFERENTIAL_INPUT_40HZ);
      value = ps.getLastDifferentialInput();
      float calibratedValue = value - resettekanan;
      smoothedValue = alpha * calibratedValue + (1 - alpha) * smoothedValue;
      mmhg = smoothedValue / ADCmmHg;
      tk   = (int)mmhg;

      // ===== OSCILLATION DETECTION =====
      static float prevMmhg = 0;
      float oscAmp = abs(mmhg - prevMmhg);
      prevMmhg = mmhg;

      recentAmps[ampIdx] = oscAmp;
      ampIdx = (ampIdx + 1) % 7;
      float smoothAmp = 0;
      for (int i = 0; i < 7; i++) smoothAmp += recentAmps[i];
      smoothAmp /= 7.0;

      envelope = updateEnvelope(envelope, smoothAmp);
      if (envelope > maxEnvelope) maxEnvelope = envelope;

      last0 = last1; last1 = last2; last2 = smoothAmp;
      if (isPeak(last0, last1, last2)) {
        if (smoothAmp > 0.8 && mmhg > 90) {
          peakCount++;
          if (!candidateLocked && peakCount >= 2) {
            candidateLocked    = true;
            candidatePressure  = mmhg;
            targetPressure     = candidatePressure + 40;
            if (targetPressure > 200) targetPressure = 200;
            Serial.print("[ADAPTIVE] Target: ");
            Serial.println(targetPressure);
          }
        }
      }
      // =================================
    }
  }

  void resetsensor() {
    resettekanan = 0;
    for (int i = 0; i < 10; i++) {
      while (!ps.isReady());
      ps.readAndSelectNextData(HX710_DIFFERENTIAL_INPUT_40HZ);
      resettekanan += ps.getLastDifferentialInput();
      delay(200);
    }
    resettekanan /= 10;
    smoothedValue  = 0;
  }

  void kirimData() {
    Serial.print("Tekanan: ");
    Serial.print(mmhg, 1);
    Serial.print(" mmHg | Mark: ");
    Serial.println(mark);

    Serial.print(mmhg);      Serial.print("\t");
    Serial.println(mark * 10);

    if (sistoleJustDetected) {
      sistoleJustDetected = false;
      kirimUART("SISTOLE");
    } else if (diastoleJustDetected) {
      diastoleJustDetected = false;
      kirimUART("DIASTOLE");
    } else {
      switch (mark) {
        case 0: kirimUART("INFLATE");  break;
        case 1: kirimUART("DEFLASI");  break;
        case 2: kirimUART("DEFLASI");  break;
        case 3: kirimUART("DEFLASI");  break;
        default: kirimUART("INFLATE"); break;
      }
    }
  }

  void mulai1() {
    if (nibpStopFlag) {
      nibpStopFlag = false;
      digitalWrite(motor,     LOW);
      digitalWrite(selenoid,  HIGH);
      digitalWrite(selenoid2, LOW);
      Serial.println("[NIBP] Dihentikan paksa via CMD:STOP");
      return;
    }
    sensorread();

    if ((mmhg >= mmhgx + 0.85) && (mmhg > 50) && (mark == 1)) {
      sistole = (int)mmhg;
      mark    = 2;
      sistoleJustDetected = true;
    }

    if ((sistole >= 145) && (mark == 2)) var = sistole - 50;
    if ((sistole >= 125) && (mark == 2) && (sistole<145)) var = sistole - 40;
    if ((sistole >= 80)  && (mark == 2) && (sistole < 125)) var = sistole - 40;
    if ((sistole <  80)  && (mark == 2)) var = sistole - 25; // ← tambah ini
    
    if ((mmhg <= mmhgx - 0.01) && (mmhg > 25) && (mmhg <= var) && (mark == 2)) {
      diastole1 = (int)mmhg;
      diastole  = diastole1;
      mark      = 3;
      diastoleJustDetected = true;
    }

    // ===== GANTI INFLATE FIXED → ADAPTIVE =====
    if ((mmhg >= targetPressure) && (mark == 0) && candidateLocked) {
      digitalWrite(motor,    LOW);
      digitalWrite(selenoid, LOW);
      mark = 1;
      Serial.print("[ADAPTIVE] Deflasi dari: ");
      Serial.println((int)mmhg);
    }
    // ==========================================

    mmhgx = mmhg;

    if ((mark == 3) && (mmhg < diastole)) {
      delay(1000);
      mark      = 0;
      sistolex  = sistole;
      diastolex = diastole;
      digitalWrite(motor,    LOW);
      digitalWrite(selenoid, LOW);
      return;
    }

    kirimData();
    delay(160);
    mulai1();
  }
