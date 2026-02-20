# Comprehensive System Audit Report

**Date:** 2025-05-15
**Auditor:** Jules

## 1. Directory Structure & Purpose

| Folder/File | Purpose |
| :--- | :--- |
| **firmware/** | ESP32 firmware code (PlatformIO project). Handles sensors, actuators, and communication. |
| **desktop-app/** | Electron-based desktop application. Acts as a local controller via WiFi (HTTP) or Bluetooth (BLE). |
| **web-app/** | Firebase Hosting project. Likely a Single Page Application (SPA) for remote monitoring/control via Firebase Realtime Database. |
| **web/** | Contains a simple `index.html`, purpose unclear but likely a landing page or test. |
| **hardware-test/** | Simple sketches for testing hardware components. |
| **firmware-future/** | Experimental firmware features. |
| **PROJECT_STATUS.md** | Project tracking document. **Note:** References a `mobile-app/` folder which is **missing** from the repository root. |

## 2. Dependency Mapping

### Firmware (`firmware/platformio.ini`)
- **ArduinoJson (^6.21.3):** JSON parsing/serialization for API and Config.
- **Firebase Arduino Client Library (^4.4.14):** Communication with Firebase Realtime Database.
- **WiFiManager (^2.0.17):** Captive portal for initial WiFi configuration.

### Desktop App (`desktop-app/package.json`)
- **electron (^28.0.0):** Framework for building desktop apps with web technologies.
- **electron-builder (^24.9.1):** Packaging and distribution tool.
- **Chart.js (via CDN):** Used in `index.html` for visualization.

### Mobile App
- **MISSING:** The `mobile-app` folder referenced in documentation is not present. `desktop-app` or `web-app` may be serving this purpose, but no dedicated React Native project was found.

## 3. Communication Flow

### A. Cloud / Remote (Firmware ↔ Firebase ↔ Web App)
1.  **Status Push:** Firmware pushes state (level, pump status, etc.) to `/tank/status` in Firebase Realtime Database every ~500ms.
2.  **Control Pull:** Firmware listens for changes in `/tank/control` (e.g., `target_setpoint`, `reset_wifi`) and `/tank/config`.
3.  **Web App:** Listens to `/tank` for updates and writes user commands to `/tank/control`.

### B. Local / Direct (Firmware ↔ Desktop App)
1.  **WiFi (HTTP):**
    -   **Status:** Desktop app polls `GET /status` (returns JSON with `level_percent`, etc.).
    -   **Config/Control:** Desktop app sends `POST` requests to `/config` and `/pid`.
2.  **Bluetooth (BLE):**
    -   **Status:** Firmware notifies on `BLE_CHARACTERISTIC_UUID` (Read/Notify).
    -   **Control:** Desktop app writes JSON to `BLE_CONTROL_UUID` (Write).

## 4. Potential Failure Points

### 🛑 Critical: API & JSON Key Mismatches
There is a severe misalignment between the JSON keys sent by the `desktop-app` and those expected by the `firmware`. This breaks configuration and control from the Desktop App.

| Field | Desktop App Sends (Key) | Firmware Expects (Key) | Consequence |
| :--- | :--- | :--- | :--- |
| **Setpoint** | `setpoint` | `target_setpoint` | **Fails.** Target level cannot be set via Desktop App. |
| **Lower Limit** | `lower_limit` | `start_level` | **Fails.** Pump start level cannot be updated. |
| **Upper Limit** | `setpoint` (Confusingly mapped to Upper Limit in UI) | `stop_level` | **Fails.** Pump stop level cannot be updated. |
| **Tank Height** | `tank_height_cm` | Ignored in HTTP POST | **Fails (HTTP).** Geometry config ignored via HTTP. |

*Note:* The firmware's `handleConfig` (POST) completely ignores geometry parameters (`tank_height_cm`, etc.), meaning they cannot be updated via the Desktop App's "Save Configuration" button over WiFi.

### ⚠️ Inconsistent Pin Mappings
In `firmware/include/config.h`:
```cpp
// Relay output to pump (GPIO 2 / D2)
// Relay output to pump (GPIO 16)
#define PUMP_RELAY_PIN 16
```
The comment states GPIO 2, but the code uses GPIO 16. This could lead to wiring errors if following comments.

### ⚠️ Missing "Mobile App"
The `PROJECT_STATUS.md` claims a `mobile-app/` exists and is a React Native Expo app. Its absence means any mobile-specific build configuration or native features are lost or were never committed.

### ℹ️ Potential Stability Risks
-   **Heap Fragmentation:** Frequent use of `String` concatenation in `logSystem` and `setPump` (for logging) could lead to fragmentation over long runtimes.
-   **Blocking Operations:** `readDistanceMedian` uses `delay(10)` inside a loop. While small, excessive blocking in the main loop can affect network responsiveness, though standard ESP32 async handling usually mitigates this.

## 5. Firmware Workflow Analysis (`firmware/src/main.cpp`)

This section details the execution flow of the main firmware file.

### A. Initialization Phase (`setup()`)
The `setup()` function runs once on boot:
1.  **Serial Console:** Initializes UART at 115200 baud.
2.  **Filesystem (LittleFS):** Mounts the filesystem; formats it if mounting fails.
3.  **Persistence Load:** Reads stored config (tank height, PID gains, setpoints) from Non-Volatile Storage (NVS) using `Preferences`.
4.  **Hardware Init:** Sets pin modes for Ultrasonic Sensor (Trig/Echo), Pump Relay, and Analog Output.
5.  **WiFi Manager:**
    -   Starts an Access Point (`TankLogic-Setup`) if no known WiFi is found.
    -   Displays a captive portal allowing the user to configure WiFi credentials and custom parameters (Tank Depth, Sensor Gap, Setpoint).
    -   Saves new parameters to NVS if changed.
6.  **External Services:**
    -   **Firebase:** Authenticates and begins the Realtime Database connection.
    -   **BLE:** Initializes the BLE Server (`Tank Logic Pro`) and Services (Status & Control characteristics).
    -   **WebServer:** Registers HTTP endpoints (`/`, `/status`, `/config`, etc.) and starts the server.
    -   **mDNS:** Sets up `tank-controller.local` resolution.
7.  **Watchdog:** Initializes the Task Watchdog Timer (60s timeout).

### B. Main Execution Loop (`loop()`)
The `loop()` function runs continuously:
1.  **Safety Checks:**
    -   Resets the Watchdog Timer.
    -   Checks the physical BOOT button (GPIO 0). If held for 3 seconds, performs a Factory Reset (wipes WiFi settings).
2.  **WebServer Handling:** Processes any incoming HTTP client requests.
3.  **Control Cycle (Every 500ms):**
    -   **Sensor Reading:** Reads water level via Ultrasonic sensor (median of 5 samples). Returns `-1.0` if invalid.
    -   **Safety Logic:** If reading is invalid, turns Pump OFF and sets Analog Output to 0V.
    -   **Control Logic (Valid Reading):**
        -   **Pump (Hysteresis):**
            -   Stops pump if level >= `pumpStopLevel`.
            -   Starts pump if level <= `pumpStartLevel`.
        -   **Valve (PID):**
            -   Calculates PID output based on `targetLevelPercent` vs `currentLevel`.
            -   Writes result to Analog DAC Output.
    -   **Cloud Sync (Firebase):**
        -   **Push:** Uploads current status (Level, PID, Pump state, etc.) to `/tank/status`.
        -   **Pull:** Downloads configuration from `/tank` (Control, Config, PID tunings) and updates local variables/NVS if changed.
    -   **BLE Notify:** Updates the BLE Status Characteristic with the latest level and pump state.

### C. Critical Logic Notes
-   **Blocking Delay:** The ultrasonic reading (`readDistanceMedian`) introduces a delay of ~50ms (5 samples * 10ms delay). This occurs every 500ms.
-   **Firebase Batching:** Firebase operations are batched to occur only once every 500ms to avoid network congestion.
-   **Priority:** Local safety logic (pump deadband) runs *before* cloud sync, ensuring the system reacts to water levels even if the internet is lost.
