
![alt text](https://github.com/lonelyloli/Portable-Multi-Parameter-Patient-Monitor-Based-ON-ESP-NOW/blob/main/Documentation/Design%201.png)



# Portable Multi-Parameter Patient Monitor Based on ESP-NOW

A low-cost, open-hardware, portable patient monitor that simultaneously acquires **three-lead ECG**, **non-invasive blood pressure (NIBP)**, **SpO₂**, **skin temperature**, **heart rate (HR)**, and **respiration rate (RR)** — all coordinated wirelessly through Espressif's **ESP-NOW** protocol, without requiring a Wi-Fi access point or patient-side cabling between sensing and display units.

This repository contains the complete hardware design (schematics/PCB), firmware (Arduino/ESP32), 3D-printed enclosure files, Nextion HMI project, MATLAB validation scripts, and the companion Visual Basic data-visualization application described in the accompanying HardwareX-style article.

> **Total hardware cost:** ≈ **US $510.18** per complete six-module unit — a small fraction of the cost of a comparable certified commercial multi-parameter monitor.

---

## Table of Contents

- [Overview](#overview)
- [Key Features](#key-features)
- [System Architecture](#system-architecture)
- [Hardware Specifications](#hardware-specifications)
- [Sensing Branches](#sensing-branches)
  - [1. ECG (3-Lead)](#1-ecg-3-lead)
  - [2. Heart Rate / Respiration Rate](#2-heart-rate--respiration-rate)
  - [3. NIBP (Oscillometric + Korotkoff)](#3-nibp-oscillometric--korotkoff)
  - [4. SpO₂ and Skin Temperature (with BMS)](#4-spo₂-and-skin-temperature-with-bms)
  - [5. Master Unit](#5-master-unit)
- [Firmware](#firmware)
- [Data Logging & Visualization](#data-logging--visualization)
- [Repository Structure / Design Files](#repository-structure--design-files)
- [Bill of Materials (Summary)](#bill-of-materials-summary)
- [Build Instructions](#build-instructions)
  - [3D-Printed Enclosures](#3d-printed-enclosures)
  - [Firmware Flashing & MAC Pairing](#firmware-flashing--mac-pairing)
  - [Hardware Assembly](#hardware-assembly)
  - [Algorithm Validation (MATLAB)](#algorithm-validation-matlab)
  - [Nextion HMI Setup](#nextion-hmi-setup)
- [Operating Instructions](#operating-instructions)
- [Validation & Performance Results](#validation--performance-results)
- [Comparison with a Commercial Monitor](#comparison-with-a-commercial-monitor)
- [Limitations & Future Work](#limitations--future-work)
- [Authors](#authors)
- [License](#license)
- [Citation](#citation)
- [Acknowledgments](#acknowledgments)

---

## Overview

Continuous, multi-parameter monitoring of vital signs is essential in modern healthcare, but many low-cost portable devices are limited to a single parameter, depend on wired transmission, or lack basic safety features such as lead-off detection or configurable alarms. Conventional bedside monitors capable of tracking multiple parameters simultaneously are bulky, cable-tethered, and expensive — putting them out of reach for many primary-care clinics, ambulatory settings, and resource-limited healthcare facilities.

This project addresses that gap with an **integrated, portable, multi-parameter patient monitor** built entirely on the **ESP32 platform** and the **ESP-NOW** peer-to-peer wireless protocol. Physically separate sensing ("slave") modules acquire and pre-process each physiological signal at the patient's point of contact, then transmit processed data wirelessly to a display ("master") unit at the nurse/observation station — eliminating patient-side data cabling entirely.

The design combines:

- Three-lead ECG with **lead-selector** and **lead-fail-detector** features (with computational reconstruction of Lead III via Einthoven's law)
- Oscillometric NIBP measurement, cross-validated with Korotkoff-sound detection, plus a spreadsheet-based data logger
- SpO₂ and skin-temperature measurement with an integrated **battery management system (BMS)**
- ECG-derived HR/RR extraction with a **configurable safety-alarm subsystem** (visual LED bar + audible speaker)
- An **offline-capable**, spreadsheet-based data logger visualized through a companion **Visual Basic** desktop application — removing dependency on continuous internet access

System performance was validated against clinical calibrators/simulators (ECG, NIBP, and SpO₂) and benchmarked against a reference commercial multi-parameter patient monitor (YKDmed YKD 1000Plus) across multiple respondents.

---

## Key Features

| Feature | Description |
|---|---|
| **Wireless slave-master architecture** | No data cable between sensing units and the display unit; communication via ESP-NOW (2.4 GHz, no access point required) |
| **Multi-parameter acquisition** | ECG (3-lead), NIBP, SpO₂, skin temperature, HR, RR — all simultaneously |
| **Lead-selector / lead-fail detector** | Switch between Lead I/II/III on demand; automatically flags disconnected electrodes with sub-100 ms response |
| **Configurable safety alarm** | Clinician-adjustable HR/RR/SpO₂ thresholds trigger a visual RGB LED bar and audible speaker alert |
| **Battery Management System (BMS)** | Automatic AC/battery switching, state-of-charge estimation in 25% steps, thermal-safe charging |
| **Offline data logging** | Local SD-card CSV storage synced to a spreadsheet-based logger, reviewable via a Visual Basic desktop app — no continuous internet required |
| **Low cost, open hardware** | ≈ US $510.18 total component cost, released under CC BY-SA 4.0 |
| **Clinically acceptable accuracy** | All six measured parameters achieve mean errors below ~3% against reference instruments |

---

## System Architecture

The system uses a **two-tier, distributed sensor-network architecture**:

1. **Slave units (7 physical ESP32/ESP32-S3 boards across 6 branches):** each acquires and pre-processes one physiological signal via a dedicated analog front-end (instrumentation amplifiers, active filters, demultiplexers) and onboard digital signal processing (Pan-Tompkins QRS detection, amplitude-modulation ECG-derived respiration, oscillometric NIBP algorithms, ratio-of-ratios SpO₂ estimation).
2. **Master units (ESP32-S3 + 10.1" Nextion touchscreen):** aggregate incoming ESP-NOW packets, render live numeric values and waveforms on the touchscreen, execute safety-alarm logic, and forward data to the logging subsystem (SD card → Google Sheets/Apps Script → Visual Basic desktop app).

```
        ┌────────────────────┐        ESP-NOW (2.4 GHz, P2P)        ┌─────────────────────┐
        │   Slave Modules     │ ───────────────────────────────────▶ │   Master Modules      │
        │  (ECG, HR/RR, NIBP, │ ◀─────────────────────────────────── │ (Nextion 10.1" HMI,   │
        │  Korotkoff, SpO2/   │                                      │  Safety Alarm, SD     │
        │  Temp/BMS)          │                                      │  Card Gateway)         │
        └────────────────────┘                                      └──────────┬───────────┘
                                                                                  │ Wi-Fi/HTTPS
                                                                                  ▼
                                                              ┌───────────────────────────────┐
                                                              │ Google Sheets API / Apps Script │
                                                              │ (Spreadsheet + Drive Logger)     │
                                                              └───────────────┬───────────────┘
                                                                              ▼
                                                              ┌───────────────────────────────┐
                                                              │ Visual Basic Desktop Application │
                                                              │ (Offline Waveform Review, Export) │
                                                              └───────────────────────────────┘
```

Each ESP32 unit registers its communication peer's MAC address before any data exchange takes place, using a bidirectional registration scheme (master knows every slave's address; every slave knows the master's address).

---

## Hardware Specifications

**Closest commercial analog:** Multi-parameter bedside/portable patient monitor with wireless slave-to-master architecture
**Subject areas:** Electronics and Microcontroller Systems · Biomedical Instrumentation · Medical/Biomedical Engineering
**Open-source license:** [CC BY-SA 4.0](https://creativecommons.org/licenses/by-sa/4.0/)
**Estimated hardware cost:** US $510.18
**Source file repository (OSF):** [DOI 10.17605/OSF.IO/ZMVT8](https://doi.org/10.17605/OSF.IO/ZMVT8)

### ESP32 Microcontroller (core of every branch)

| Item | Specification |
|---|---|
| MCU | Xtensa® Dual-Core 32-bit LX7, up to 240 MHz, 512 DMIPS |
| Wi-Fi | 802.11 b/g/n, HT20/HT40, 2.4 GHz |
| Bluetooth | Bluetooth 5 LE |
| SRAM | 512 kB |
| Flash | SPI Flash up to 16 MB |
| GPIO | Up to 45 |
| PWM | LEDC PWM, up to 8 channels |
| SPI / I²C / I²S / UART | 4 / 2 / 2 / 3 |
| ADC | 12-bit |
| CAN | TWAI (CAN 2.0 compatible), 1 controller |
| ESP-NOW | Peer-to-peer, up to 20 peers, 250-byte payload |
| Working temperature | -40 °C to +85 °C |

---

## Sensing Branches

### 1. ECG (3-Lead)

- **Front end:** AD620 instrumentation amplifier (gain ≈ 495×, closed-loop `Acl = 1 + 49 kΩ/Rg`), two-stage TL084 active filters (HPF fc = 0.05 Hz, LPF fc = 100 Hz), OP07-based notch filter (fc = 50 Hz) and adder/level-shift stage (shifts signal into the ESP32's 0–3.3 V ADC range).
- **Sampling:** 250 Hz via hardware-timer interrupt (12-bit ADC, 11 dB attenuation).
- **Lead III** is computed arithmetically from Lead I and Lead II using **Einthoven's triangle** relationship, avoiding the need for a third physical electrode pair.
- **Lead-fail detection:** electrode-variance statistics flag disconnected electrodes by name (e.g., "RA", "LA") with sub-100 ms response.
- Electrode placement: RA, LA, LL (RL as ground).

### 2. Heart Rate / Respiration Rate

- Reuses a Lead-II ECG front end sampled at 250 Hz.
- **Heart rate:** Pan-Tompkins QRS-detection pipeline (derivative → squaring → moving-window integration → adaptive thresholding).
- **Respiration rate:** amplitude-modulation ECG-derived respiration (EDR-AM), tracking R-peak-amplitude variation caused by chest-wall movement during breathing; the extracted respiration waveform is also rendered live on the Nextion display.
- Digital filtering (notch/high-pass/band-pass IIR, second-order-section/biquad form) was designed and validated offline in MATLAB (`butter()` + `tf2sos()`) before being translated into fixed-point embedded C on the ESP32.
- No dedicated pulse or respiration sensor is required — both HR and RR are derived from the same ECG electrodes used for the 3-lead branch.

### 3. NIBP (Oscillometric + Korotkoff)

- **Pressure sensing:** MPS3117 pressure sensor (0–12 psi, 2.4–5.5 V) + HX710B 24-bit differential ADC (128× gain).
- **Actuation:** pump-motor driver + two solenoid-valve drivers, each initialized to a safe (de-energized) state at power-on.
- **Algorithm:** stepped-deflation oscillometric state machine identifying systolic/diastolic breakpoints from cuff-pressure oscillation amplitude.
- **Cross-validation:** a second dedicated slave listens for Korotkoff sounds via a MAX4466 electret-microphone preamplifier, communicating with the NIBP slave over UART2 (FreeRTOS-queue-buffered).
- Automatic or manual measurement (10-minute periodic mode, or START NIBP button).

### 4. SpO₂ and Skin Temperature (with BMS)

- **SpO₂:** DS100A Nellcor finger sensor, HEF4051 demultiplexer, LM324-based amplifier/filter stage. Red (660 nm) / infrared (940 nm) LEDs alternately toggled at ~500 Hz each (1 kHz combined drive cycle); AC/DC ratio computed for each wavelength and converted to SpO₂ via empirical ratio-of-ratios regression.
- **Skin temperature:** DS18B20 one-wire digital temperature probe (positioned at the axilla), 4.7 kΩ pull-up, no additional analog conditioning required.
- **Battery Management System (BMS):** resistive voltage-divider network (10 kΩ / 28.1 kΩ) mapping battery voltage to state-of-charge in 25% steps (0% = 2.36 V → 100% = 3.30 V); automatic switching between 220 V AC adaptor and Li-ion battery pack.

### 5. Master Unit

- ESP32-S3 + 10.1" Nextion touchscreen (UART, GPIO17/TX, GPIO18/RX).
- Registers the MAC address of every slave (ECG, HR/RR, NIBP, Korotkoff, SpO₂/temperature/BMS) as an ESP-NOW peer.
- Hosts the safety-alarm indicator stage (RGB LED bar + speaker), the SD-card data-logging gateway (SPI: MOSI/MISO/SCK/CS), and the dual-power-supply switching circuit.

---

## Firmware

All firmware was developed in the **Arduino IDE (v2.3.6)** with the ESP32 core, organized into **seven sketches**:

| Sketch | Runs on | Role |
|---|---|---|
| `Master_Core.ino` | ESP32 master | Registers slave MAC addresses as ESP-NOW peers, receives `struct_message` packets, updates the Nextion HMI, evaluates alarm thresholds |
| `Master_Datalogger.ino` | ESP32 master | Manages local SD-card CSV storage and uploads to the spreadsheet-based logger |
| `Slave_ECG3Lead.ino` | ECG slave | Lead I/II acquisition, filtering, Lead III reconstruction, lead-fail detection |
| `Slave_HR_RR.ino` | HR/RR slave | Pan-Tompkins QRS detection + EDR-AM respiration extraction |
| `Slave_NIBP_Core.ino` | NIBP slave | Oscillometric pump/valve control state machine |
| `Slave_NIBP_Send.ino` | NIBP slave | Send to Master machine |
| `Slave_SKINTEMP_Sent` | SpO₂/Temp/BMS slave | DS18B20 temperature acquisition |
| `Slave_SPO2_Core.ino` | SpO₂/Temp/BMS slave | Red/IR LED timing, demultiplexing, ratio-of-ratios SpO₂ computation |

Key libraries: `esp_now.h`, `WiFi.h` (station mode, `WIFI_STA`), `freertos/task.h` & `freertos/queue.h` (ECG/HR-RR), `HardwareSerial` (NIBP/Korotkoff inter-slave UART + SD-card library on the gateway), `OneWire` and `DallasTemperature` (SpO₂/Temp/BMS branch).

**MAC-address pairing:** each ESP32's unique MAC address must be discovered (via a dedicated address-discovery sketch) and hard-coded into the corresponding peer's firmware — bidirectionally — before `esp_now_add_peer()` can establish a valid slave↔master communication channel.

---

## Data Logging & Visualization

- **On-device:** the NIBP branch's master hosts an SD-card gateway that writes time-stamped CSV records with optimized write-cycle management to preserve flash lifetime.
- **Cloud sync (optional, offline-tolerant):** records are forwarded to a Google Sheets-based spreadsheet logger via Apps Script/HTTPS, reviewable from a smartphone browser.
- **Companion desktop app:** a **Visual Basic** application retrieves logged sessions via the Sheets API and reconstructs ECG, SpO₂ (PPG), and respiration waveforms as time-aligned graphical plots, with zoom, pan, and export (PNG/PDF/CSV) — all without requiring an active connection to the physical hardware.

---

## Repository Structure / Design Files

| File | Type | License |
|---|---|---|
| `PastMontFIX.sch` | Eagle schematic | CC BY-SA 4.0 |
| `Master_Core.ino` | Arduino firmware | CC BY-SA 4.0 |
| `Master_Datalogger.ino` | Arduino firmware | CC BY-SA 4.0 |
| `Slave_ECG3Lead.ino` | Arduino firmware | CC BY-SA 4.0 |
| `Slave_HR_RR.ino` | Arduino firmware | CC BY-SA 4.0 |
| `Slave_NIBP_Core.ino` | Arduino firmware | CC BY-SA 4.0 |
| `Slave_NIBP_Send.ino` | Arduino firmware | CC BY-SA 4.0 |
| `Slave_SKINTEMP_Sent` | Arduino firmware | CC BY-SA 4.0 |
| `SPO2.ino` | Arduino firmware | CC BY-SA 4.0 |
| `Slave_HR_RR_MATLAB_E.m.txt` | MATLAB validation script | CC BY-SA 4.0 |
| `3D PASMONT.f3z` | Fusion 360 3D-printing source | CC BY-SA 4.0 |
| `PASMONT FIX.HMI` | Nextion HMI project | CC BY-SA 4.0 |

All files are archived at **DOI: [10.17605/OSF.IO/ZMVT8](https://doi.org/10.17605/OSF.IO/ZMVT8)**.

---

## Bill of Materials (Summary)

Full BOM with supplier links is included in the design-file repository. Notable components:

| Component | Qty | Unit Cost (US $) |
|---|---|---|
| ESP32-S3 Development Board | 8 | 7.08 |
| AD620 Instrumentation Amplifier | 2 | 0.99 |
| TL084 Quad Op-Amp | 2 | 0.99 |
| MPS3117 Pressure Sensor | 1 | 8.73 |
| HX710B 24-bit ADC Module | 1 | 2.16 |
| DS100A Nellcor Finger SpO₂ Sensor | 1 | 19.35 |
| DS18B20 Temperature Sensor | 1 | 1.08 |
| BMS 3S Battery Management Module | 1 | 3.45 |
| 10.1" Nextion LCD Display | 1 | 137.50 |
| Custom PCB (×8) | 8 | 10.00 |
| ECG Patient Cable | 1 | 42.00 |
| 3.7 V Li-ion Battery Cells (×8) | 8 | 8.46 |
| PLA 3D Printing Filament | 1 | 10.93 |

**Total estimated cost: US $510.18**

---

## Build Instructions

### 3D-Printed Enclosures

- Designed in **Fusion 360**, exported to STL, sliced with **Ultimaker Cura**.
- Print settings: 0.2 mm standard-quality profile, 0.4 mm nozzle, 80% print speed, **30% infill**, raft-type build-plate adhesion (extra margin 15 mm).
- Printer settings: nozzle 225 °C, bed 60 °C (PLA, 1.75 mm, per Esun spec range 205–225 °C).
- Dimensional accuracy of printed parts: ≈ 0.2 mm.

### Firmware Flashing & MAC Pairing

1. Flash the address-discovery sketch to each of the 5 slave units and to the master; read each 6-byte MAC address from the Arduino Serial Monitor.
2. Enter the **master's** MAC address into `receiverAddress[]` inside each slave sketch.
3. Enter each **slave's** MAC address into the master's `macSlaveECG[]`, `macSlaveHRRR[]`, `macSlaveNIBP[]`, `macSlaveSpO2[]` arrays.
4. Flash `MASTER_core.ino` + `MASTER_datalogger.ino` to the master; flash each module-specific sketch to its slave.

### Hardware Assembly

- Wire the Nextion display to the ESP32 master via UART (TX/RX), plus the LED-bar/speaker alarm connectors and dual-power-supply switching circuit.
- Wire each slave's analog front-end output to its ESP32 ADC pin; connect actuator drivers (NIBP pump/valve) and sensor headers (SpO₂ finger clip, DS18B20 probe) accordingly.
- Mount each board inside its corresponding 3D-printed enclosure using the pre-designed mounting holes (no additional drilling required). Each slave is powered by its own dedicated battery pack.

### Algorithm Validation (MATLAB)

- `HR_RR_MATLAB_E.m.txt` imports a recorded ECG CSV, resamples to 250 Hz, applies Butterworth notch (50 Hz)/high-pass (0.5 Hz)/band-pass (5–15 Hz) filters (converted to numerically stable second-order-section/biquad form via `butter()` + `tf2sos()`), runs the Pan-Tompkins R-peak detection sequence, and extracts the EDR-AM respiration envelope — validating filter coefficients before they are transferred into `HR_RR.ino`.

### Nextion HMI Setup

- Open the `.HMI` project in **Nextion Editor**, adjust layout as needed, compile to `.tft`, and upload via micro-SD or UART.
- `MASTER_core.ino` addresses each Nextion component by its component ID (e.g., `txHR.txt=...`) — UI changes only require re-editing the Nextion project, not the firmware.

---

## Operating Instructions

1. Power off all units before attaching sensors.
2. Place the three ECG electrodes (RA/LA/LL, RL = ground) per the Einthoven-triangle configuration — this same placement feeds both the ECG and HR/RR branches.
3. Clip the SpO₂ finger sensor onto a fingertip; position the DS18B20 probe at the axilla; wrap the NIBP cuff on the upper arm with the Korotkoff microphone beneath it, over the brachial artery.
4. Power on the master first, then each slave in turn; confirm ESP-NOW pairing on the touchscreen.
5. ECG waveform, HR, RR, SpO₂/PPG, and skin temperature update automatically and continuously.
6. Switch ECG leads via the on-screen lead selector; the lead-fail detector auto-flags disconnections.
7. Trigger NIBP manually (START NIBP) or enable periodic auto-measurement (every 10 minutes).
8. Configure HR/RR/SpO₂ alarm thresholds via on-screen sliders and confirm with SET ALARM.
9. Start/stop session recording with START RECORD / STOP RECORD; review recorded sessions offline via the Visual Basic desktop app.

---

## Validation & Performance Results

Testing followed an after-only pre-experimental design with repeated measurements across multiple respondents, benchmarked against clinical calibrators/simulators and a reference commercial patient monitor (YKDmed YKD 1000Plus).

| Parameter | Reference Instrument | Mean Error |
|---|---|---|
| ECG waveform fidelity | Contec MS400 calibrator (40–120 bpm) | 0% |
| Heart Rate (Pan-Tompkins) | Contec MS400 calibrator (0–350 bpm) | 0.047% |
| Heart Rate (Pan-Tompkins) | YKD 1000Plus patient monitor | 0.662% (SD 0.664) |
| Respiration Rate (EDR-AM) | YKD 1000Plus patient monitor | 0.813% (SD 0.41) |
| NIBP Systolic | Reference tensimeter (ONEMED Tensi One 1A) | ≈ ± 1.0% |
| NIBP Diastolic | Reference tensimeter | ≈ ± 1.8% |
| SpO₂ | Reference patient monitor | 0.614% |
| SpO₂ | Dedicated SpO₂ simulator (16 points, 70–100%) | 2.924% |
| Skin Temperature | Reference patient monitor | 0.637% |

**Functional test results:**

| Feature | Outcome |
|---|---|
| Lead selector | Correct waveform switching, response < 100 ms |
| Lead-fail detector | Correctly identifies disconnections; no false alarms |
| Safety alarm system | LED + speaker trigger correctly, auto-clear on normalization |
| BMS | State-of-charge correctly mapped (0/25/50/75/100%) |
| Data logger (SD + spreadsheet) | Records saved and uploaded correctly |
| Visual Basic visualization | Waveforms reconstructed; zoom/pan/export functional |
| ESP-NOW communication | Reliable indoors up to **15–20 m** (line-of-sight) |

Signal-quality analysis showed the raw ECG SNR improving from −7.56 dB to 34.49 dB after the 50 Hz notch filter and 22.37 dB after the high-pass filter, with the QRS-focused band-pass stage narrowing SNR to 2.93 dB by design (for optimal QRS detection), and the extracted EDR-AM respiration signal achieving 6.55 dB.

---

## Comparison with a Commercial Monitor

| Parameter | This system | Basis of comparison | Advantage |
|---|---|---|---|
| HR | 0.047–0.662% error | YKD 1000Plus | Matches commercial accuracy without a dedicated pulse sensor |
| RR | 0.813% error | YKD 1000Plus | Adds a live respiration waveform, not just a numeric value |
| NIBP | ±1.0% / ±1.8% | Prior portable NIBP designs (4.6–6.6% error) | Markedly lower error |
| SpO₂ | 0.614–2.924% error | Prior MAX30100/MAX30102 designs (1.2–4.8% error) | Lower error |
| Skin temperature | 0.637% error | Prior temperature-module designs (up to 11.7% error) | Substantially lower error |
| Wireless range | 15–20 m indoors (ESP-NOW) | Wired bedside monitors | Eliminates patient-side cabling |
| Data logging | Offline spreadsheet + VB waveform viewer | Manufacturer-proprietary reporting | Works without continuous internet access |
| Cost | US $510.18 | Commercial monitors (several hundred to >1,000 USD) | Comparable/better accuracy at a fraction of the cost |

---

## Limitations & Future Work

- SpO₂ accuracy degrades at low-saturation extremes (70–86%) due to the linear ratio-of-ratios regression model (up to 10.57% error at the lowest tested point) — a known limitation the authors suggest addressing with non-linear or machine-learning-based calibration.
- ECG and NIBP waveform display becomes less stable beyond 15 m indoors (though the ESP-NOW link itself remains connected).
- Suggested future improvements: accelerometer-based motion-artifact compensation for the ECG/EDR pipeline, and long-range wireless alternatives (Wi-Fi long-range mode, LoRa) to extend range and reliability for broader clinical deployment.

---

## Authors

- I Gede Oka Pradnyananda Kusuma — Conceptualization, ECG analog front-end design, lead-selector/lead-fail-detector firmware
- I Putu Andika Budi Pratama — HR/RR algorithm implementation (Pan-Tompkins, EDR-AM), safety-alarm design, MATLAB validation
- Muhammad Fa'izun Nuha — SpO₂ and skin-temperature circuit design, BMS development
- Risyad Dani Tri Wardana — NIBP circuit design, oscillometric algorithm, data-logger integration, 3D design
- Bambang Guruh Irianto — Supervision and Methodology
- I Dewa Gede Hari Wisana — Supervision and Validation

**Affiliation:** Department of Medical Electronics Technology, Poltekkes Kemenkes Surabaya, Indonesia
**Corresponding author:** bgi_dha@poltekkesdepkes-sby.ac.id

---

## License

This hardware design and associated firmware/software are released under the **[CC BY-SA 4.0](https://creativecommons.org/licenses/by-sa/4.0/)** license.

---

## Citation

If you use this design in your research, please cite the original article and the design-file repository:

> I Gede Oka Pradnyananda Kusuma et al., "Portable Multi-Parameter Patient Monitor Based on ESP-NOW," Department of Medical Electronics Technology, Poltekkes Kemenkes Surabaya. Design files: DOI [10.17605/OSF.IO/ZMVT8](https://doi.org/10.17605/OSF.IO/ZMVT8).

---

## Acknowledgments

The authors thank the Department of Electromedical Engineering, Poltekkes Kemenkes Surabaya, for providing facilities and support during this research.
