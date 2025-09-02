<div align="center">

# Motor Speed Control via Hand Gesture
**ESP32 (ESP-IDF) · FreeRTOS · PID · MQTT · OpenCV Gesture Recognition · Sklearn Random Forest Modek · PyQt5 Dashboard**

</div>

## 1. Project Summary
This project implements a closed‑loop motor speed control system on an ESP32 using a PID controller. The target speed (setpoint) is selected hands‑free by counting raised fingers in front of a camera (gesture recognition on a host machine with random forest model). A lightweight PyQt5 desktop dashboard provides live telemetry (speed, setpoint, mode) and allows runtime tuning of PID gains without reflashing. Communication between the ESP32 firmware and the desktop / gesture subsystems is performed over MQTT.

## 2. Gesture Mapping (Default)
| Fingers | Mode Name | Target % of Max | Notes |
|---------|-----------|-----------------|-------|
| 0 | Stop | 0 | Motor disabled |
| 1 | Low | 100 | Soft start tier |
| 2 | Medium | 150 | Moderate speed |
| 3 | Mid+ | 200 | Balanced performance |
| 4 | High | 250 | Near full load |
| 5 | Max | 300 | Full duty (capped by motor and driver limits) |

## 3. Hardware Overview (Fill In)
Add specifics:
| Item | Example | Notes |
|------|---------|-------|
| MCU | ESP32-DevKitC | 240 MHz dual-core |
| Motor | DC / BLDC (specify) | Provide rated voltage/current |
| Driver | L298N / MOSFET / ESC | Ensure adequate current headroom |
| Encoder | Incremental (N PPR) | Used for RPM feedback |
| Power | 12V motor supply | Common ground required |
| Camera | USB / Laptop Cam / IP cam | For gesture host side |

Include wiring diagram / pin table (GPIO for PWM, encoder A/B, etc.).

## 4. Software Stack
| Layer | Technology |
|-------|------------|
| Firmware SDK | ESP-IDF (version: FILL_IN) |
| RTOS | FreeRTOS (bundled) |
| Messaging | MQTT (broker: Mosquitto / Cloud) |
| Vision | Python + OpenCV / MediaPipe / random forest model |
| UI | PyQt5 + (pyqtgraph / matplotlib) |
| Build Tools | idf.py, CMake, Python virtual env |

## 5. Build & Flash (ESP-IDF Firmware)


```bash

  # (1) Export ESP-IDF environment 
  . path/to/esp-idf/export.sh

# (2) Configure project (Wi-Fi ,Broker, etc.)
idf.py menuconfig

# (3) Build
idf.py build

# (4) Flash (adjust port)
idf.py -p <PORT> flash

# (5) Monitor
idf.py -p <PORT> monitor
```
Use Ctrl+] to exit the monitor.

## 6. Desktop Environment Setup (Placeholders)
Steps to fill in:
1. Create virtual environment in each
gesture_rovognition and gui or just use uv sync

2. Install Python dependencies requirements.txt file
```bash
pip install -r requirements.txt
```
of using uv (only for because it lock with pyqt5-qt==5.15 which is only in linux)
```bash
uv sync
```
3. Launch gesture service
##### linux
```bash
#in gesture_recognition
uv run main.py --port 1883 --broker "192.168.0.81"
```
##### windows
```cmd
REM in gesture_recognition
.venv/Scripts/activate
python -m main --port 1883 --broker "192.168.0.81"
```
4. Launch PyQt5 dashboard (new terminal)

the same as 3.