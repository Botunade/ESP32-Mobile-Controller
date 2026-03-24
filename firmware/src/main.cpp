#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <esp_task_wdt.h>
#include <deque>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <LittleFS.h>
#include <vector>
#include <algorithm>
#include <Firebase_ESP_Client.h>
#include "secrets.h"
#include "config.h"
#include "pid.h"
#include <WiFiManager.h>
#include <addons/TokenHelper.h>
#include <addons/RTDBHelper.h>

// ========== CORE OBJECTS ==========
WebServer server(80);
IPAddress apIp;
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
bool signupOK = false;
unsigned long lastFirebaseSend = 0;

// ========== GLOBAL RUNTIME VARIABLES ==========

// Primary Control Parameters
float targetLevelPercent = 50.0f;           // Valve Target (PID)
float pumpStopLevel = DEFAULT_SETPOINT;     // Upper Cut-off (Pump)
float pumpStartLevel = DEFAULT_LOWER_LIMIT; // Lower Turn-on (Pump)

// Legacy compatibility (Ensures dashboard v0.4.0 and earlier don't break)
float setpointPercent = DEFAULT_SETPOINT;
float lowerLimitPercent = DEFAULT_LOWER_LIMIT;

// Transient State
bool pumpOn = false;
float lastLevelPercent = 0.0f;
float lastPidOutput = 0.0f;
unsigned long lastControlTimeMs = 0;

// Tank Geometry Configuration
float tankHeightCm = TANK_HEIGHT_CM;
float minDistanceCm = MIN_DISTANCE_CM;
float maxDistanceCm = MAX_DISTANCE_CM;

// PID Controller Settings
float currentKp = PID_KP;
float currentKi = PID_KI;
float currentKd = PID_KD;
PIDController pid(PID_KP, PID_KI, PID_KD);

// DAC Output Settings (8-bit: 0-255)
int currentDacMin = DAC_MIN_VAL;
int currentDacMax = DAC_MAX_VAL;

// Persistence
Preferences preferences;

// ========== SAFETY & LOGGING ==========
#define WDT_TIMEOUT 60 
std::deque<String> systemLogs;

void logSystem(String msg)
{
    if (systemLogs.size() >= 20)
    {
        systemLogs.pop_front();
    }

    String logEntry = String(millis() / 1000);
    logEntry += "s: ";
    logEntry += msg;
    systemLogs.push_back(logEntry);
    Serial.println(msg);
}

// WiFiManager Context
WiFiManager wm;
bool shouldSaveConfig = false;
void saveConfigCallback()
{
    Serial.println("[WM] Config save triggered");
    shouldSaveConfig = true;
}

// BLE (Bluetooth) Context
BLEServer *pServer = NULL;
BLECharacteristic *pStatusCharacteristic = NULL;
BLECharacteristic *pControlCharacteristic = NULL;
bool deviceConnected = false;

// ---------- ANALYTICS & DIAGNOSTICS ----------

void printDiagnostics()
{
    Serial.print("Level: ");
    Serial.print(lastLevelPercent, 1);
    Serial.print("% | Start: ");
    Serial.print(pumpStartLevel, 0);
    Serial.print("% | Stop: ");
    Serial.print(pumpStopLevel, 0);
    Serial.print("% | Target: ");
    Serial.print(targetLevelPercent, 0);
    Serial.print("% | Pump: ");
    Serial.print(pumpOn ? "ON" : "OFF");
    Serial.print(" | PID: ");
    Serial.println(lastPidOutput, 1);
}

// ---------- SENSOR & CONTROL HELPERS ----------

float clampValue(float x, float minVal, float maxVal)
{
    if (x < minVal) return minVal;
    if (x > maxVal) return maxVal;
    return x;
}

void sendJson(WebServer &srv, JsonDocument &doc, int statusCode = 200)
{
    String output;
    serializeJson(doc, output);
    srv.sendHeader("Access-Control-Allow-Origin", "*");
    srv.sendHeader("Access-Control-Allow-Methods", "GET,POST,OPTIONS");
    srv.sendHeader("Access-Control-Allow-Headers", "Content-Type");
    srv.send(statusCode, "application/json", output);
}

float readDistanceCm()
{
    digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

    unsigned long duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH, 30000UL);
    if (duration == 0) return -1.0f;
    return (duration * 0.0343f) / 2.0f;
}

float readDistanceMedian(int samples)
{
    if (samples <= 0) return -1.0f;
    const int MAX_SAMPLES = 64;
    if (samples > MAX_SAMPLES) samples = MAX_SAMPLES;

    float readings[MAX_SAMPLES];
    int count = 0;
    for (int i = 0; i < samples; i++)
    {
        float r = readDistanceCm();
        if (r > 0)
            readings[count++] = r;
        delay(10);
    }

    if (count == 0)
        return -1.0f;

    std::sort(readings, readings + count);

    if (count % 2 == 0)
    {
        return (readings[count / 2 - 1] + readings[count / 2]) / 2.0f;
    }
    else
    {
        return readings[count / 2];
    }
}

float readLevelPercent()
{
    float distance = readDistanceMedian(5);
    if (distance < 0) return -1.0f;

    float waterDepth = maxDistanceCm - distance;
    float maxWaterDepth = maxDistanceCm - minDistanceCm;

    if (maxWaterDepth <= 0) return 0.0f;
    float level = (waterDepth / maxWaterDepth) * 100.0f;
    return clampValue(level, 0.0f, 100.0f);
}

void setPump(bool on)
{
    if (pumpOn == on) return;
    pumpOn = on;
    digitalWrite(PUMP_RELAY_PIN, on ? HIGH : LOW);

    String logMsg = "Pump hardware switched ";
    logMsg += (on ? "ON" : "OFF");
    logSystem(logMsg);
}

void updateAnalogOutput(float pidOutputPercent)
{
    int dacValue = (int)map((long)pidOutputPercent, 0, 100, currentDacMin, currentDacMax);
    dacValue = constrain(dacValue, currentDacMin, currentDacMax);
    dacWrite(ANALOG_OUTPUT_PIN, dacValue);
}

// ---------- BLE INTERFACE CALLBACKS ----------

class MyServerCallbacks : public BLEServerCallbacks
{
    void onConnect(BLEServer *pServer)
    {
        deviceConnected = true;
        Serial.println("[BLE] Client Linked");
    };
    void onDisconnect(BLEServer *pServer)
    {
        deviceConnected = false;
        Serial.println("[BLE] Client Unlinked");
    }
};

class MyControlCallbacks : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *pCharacteristic)
    {
        String incoming = pCharacteristic->getValue().c_str();
        if (incoming.length() > 0)
        {
            StaticJsonDocument<512> packet;
            if (deserializeJson(packet, incoming)) return;

            if (packet.containsKey("target_setpoint"))
            {
                targetLevelPercent = clampValue(packet["target_setpoint"], 0.0f, 100.0f);
                setpointPercent = targetLevelPercent;
                preferences.putFloat("targetSetpoint", targetLevelPercent);
            }
            if (packet.containsKey("stop_level"))
            {
                pumpStopLevel = clampValue(packet["stop_level"], 0.0f, 100.0f);
                preferences.putFloat("stopLevel", pumpStopLevel);
            }
            if (packet.containsKey("start_level"))
            {
                pumpStartLevel = clampValue(packet["start_level"], 0.0f, 100.0f);
                lowerLimitPercent = pumpStartLevel;
                preferences.putFloat("startLevel", pumpStartLevel);
            }
            if (packet.containsKey("tank_height_cm"))
            {
                tankHeightCm = packet["tank_height_cm"];
                preferences.putFloat("tankHeight", tankHeightCm);
            }
            logSystem("BLE Local Config Applied");
        }
    }
};

// ---------- HTTP INTERFACE HANDLERS ----------

void handleOptions()
{
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.sendHeader("Access-Control-Allow-Methods", "GET,POST,OPTIONS");
    server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
    server.send(204);
}

void handleRoot()
{
    File file = LittleFS.open("/index.html", "r");
    if (!file)
    {
        server.send(500, "text/plain", "Missing Dashboard Image (LittleFS).");
        return;
    }
    server.streamFile(file, "text/html");
    file.close();
}

void handleStatus()
{
    StaticJsonDocument<512> status;
    status["level_percent"] = lastLevelPercent;
    status["pump_on"] = pumpOn;
    status["target_setpoint"] = targetLevelPercent;
    status["stop_level"] = pumpStopLevel;
    status["start_level"] = pumpStartLevel;
    status["pid_output"] = lastPidOutput;
    status["rssi"] = WiFi.RSSI();
    status["uptime"] = millis() / 1000;

    // Aliases
    status["setpoint_percent"] = targetLevelPercent;
    status["lower_limit"] = pumpStartLevel;

    sendJson(server, status);
}

void handleConfig()
{
    if (server.method() == HTTP_GET)
    {
        StaticJsonDocument<512> cfg;
        cfg["tank_height_cm"] = tankHeightCm;
        cfg["min_distance_cm"] = minDistanceCm;
        cfg["max_distance_cm"] = maxDistanceCm;
        cfg["target_setpoint"] = targetLevelPercent;
        cfg["start_level"] = pumpStartLevel;
        cfg["stop_level"] = pumpStopLevel;
        cfg["kp"] = currentKp;
        cfg["ki"] = currentKi;
        cfg["kd"] = currentKd;
        sendJson(server, cfg);
    }
    else if (server.method() == HTTP_POST)
    {
        if (!server.hasArg("plain")) return;
        StaticJsonDocument<512> update;
        deserializeJson(update, server.arg("plain"));

        if (update.containsKey("target_setpoint"))
        {
            targetLevelPercent = clampValue(update["target_setpoint"], 0.0f, 100.0f);
            setpointPercent = targetLevelPercent;
            preferences.putFloat("targetSetpoint", targetLevelPercent);
        }
        if (update.containsKey("start_level"))
        {
            pumpStartLevel = clampValue(update["start_level"], 0.0f, 100.0f);
            lowerLimitPercent = pumpStartLevel;
            preferences.putFloat("startLevel", pumpStartLevel);
        }
        if (update.containsKey("stop_level"))
        {
            pumpStopLevel = clampValue(update["stop_level"], 0.0f, 100.0f);
            preferences.putFloat("stopLevel", pumpStopLevel);
        }

        StaticJsonDocument<64> ack;
        ack["status"] = "OK";
        sendJson(server, ack);
    }
}

void handlePidUpdate()
{
    if (!server.hasArg("plain")) return;
    StaticJsonDocument<256> pidDoc;
    deserializeJson(pidDoc, server.arg("plain"));
    if (pidDoc.containsKey("setpoint"))
    {
        targetLevelPercent = clampValue(pidDoc["setpoint"], 0.0f, 100.0f);
        setpointPercent = targetLevelPercent;
        preferences.putFloat("targetSetpoint", targetLevelPercent);
    }
    StaticJsonDocument<64> ok;
    ok["status"] = "PID Sync Done";
    sendJson(server, ok);
}

void handleLogs()
{
    StaticJsonDocument<2048> doc;
    JsonArray array = doc.createNestedArray("logs");
    for (const auto &l : systemLogs)
    {
        array.add(l);
    }
    sendJson(server, doc);
}

void handleResetWifi()
{
    server.send(200, "text/plain", "Factory Reset in Progress...");
    delay(1000);
    wm.resetSettings();
    ESP.restart();
}

// ---------- SYSTEM BOOT SEQUENCE ----------

void setup()
{
    Serial.begin(115200);
    delay(100);
    Serial.println("\n\n############################");
    Serial.println("# TANK CONTROLLER v0.8.0  #");
    Serial.println("############################\n");

    if (!LittleFS.begin())
    {
        Serial.println("[FS] Init Failed, fixing...");
        LittleFS.format();
        LittleFS.begin();
    }

    // 1. Persistence Load
    preferences.begin("tank-config", false);
    tankHeightCm = preferences.getFloat("tankHeight", TANK_HEIGHT_CM);
    minDistanceCm = preferences.getFloat("minDist", MIN_DISTANCE_CM);
    maxDistanceCm = preferences.getFloat("maxDist", MAX_DISTANCE_CM);
    pumpStopLevel = preferences.getFloat("stopLevel", DEFAULT_SETPOINT);
    pumpStartLevel = preferences.getFloat("startLevel", DEFAULT_LOWER_LIMIT);
    targetLevelPercent = preferences.getFloat("targetSetpoint", 50.0f);

    setpointPercent = targetLevelPercent;
    lowerLimitPercent = pumpStartLevel;

    currentKp = preferences.getFloat("kp", PID_KP);
    currentKi = preferences.getFloat("ki", PID_KI);
    currentKd = preferences.getFloat("kd", PID_KD);
    currentDacMin = preferences.getInt("dacMin", DAC_MIN_VAL);
    currentDacMax = preferences.getInt("dacMax", DAC_MAX_VAL);

    pid.setTunings(currentKp, currentKi, currentKd);

    // 2. Hardware Mapping
    pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
    pinMode(ULTRASONIC_ECHO_PIN, INPUT);
    pinMode(PUMP_RELAY_PIN, OUTPUT);
    pinMode(0, INPUT_PULLUP);

    setPump(false);
    dacWrite(ANALOG_OUTPUT_PIN, currentDacMin);

    // 3. WiFi Connectivity
    wm.setSaveConfigCallback(saveConfigCallback);

    char hStr[10]; dtostrf(tankHeightCm, 1, 1, hStr);
    char mStr[10]; dtostrf(maxDistanceCm, 1, 1, mStr);
    char tStr[10]; dtostrf(targetLevelPercent, 1, 1, tStr);

    WiFiManagerParameter custom_h("h", "Tank Depth (cm)", hStr, 6);
    WiFiManagerParameter custom_m("m", "Sensor Gap (cm)", mStr, 6);
    WiFiManagerParameter custom_t("t", "Primary Setpoint (%)", tStr, 6);

    wm.addParameter(&custom_h);
    wm.addParameter(&custom_m);
    wm.addParameter(&custom_t);

    if (!wm.autoConnect("TankLogic-Setup", "tank1234"))
    {
        Serial.println("[WIFI] Critical Fail. Restarting.");
        ESP.restart();
    }

    if (shouldSaveConfig)
    {
        tankHeightCm = atof(custom_h.getValue());
        maxDistanceCm = atof(custom_m.getValue());
        targetLevelPercent = atof(custom_t.getValue());
        setpointPercent = targetLevelPercent;

        preferences.putFloat("tankHeight", tankHeightCm);
        preferences.putFloat("maxDist", maxDistanceCm);
        preferences.putFloat("targetSetpoint", targetLevelPercent);
    }

    // 4. External Services
    config.api_key = FIREBASE_API_KEY;
    config.database_url = FIREBASE_DATABASE_URL;
    if (Firebase.signUp(&config, &auth, "", ""))
        signupOK = true;
    Firebase.begin(&config, &auth);
    Firebase.reconnectWiFi(true);

    BLEDevice::init("Tank Logic Pro");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    BLEService *pService = pServer->createService(BLE_SERVICE_UUID);

    pStatusCharacteristic = pService->createCharacteristic(
        BLE_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_INDICATE);
    pStatusCharacteristic->addDescriptor(new BLE2902());

    pControlCharacteristic = pService->createCharacteristic(
        BLE_CONTROL_UUID,
        BLECharacteristic::PROPERTY_WRITE);
    pControlCharacteristic->setCallbacks(new MyControlCallbacks());

    pService->start();
    BLEDevice::startAdvertising();

    server.on("/", handleRoot);
    server.on("/status", handleStatus);
    server.on("/config", handleConfig);
    server.on("/pid", handlePidUpdate);
    server.on("/logs", handleLogs);
    server.on("/resetwifi", handleResetWifi);
    server.on("/handshake", handleRoot);
    server.onNotFound(handleRoot);
    server.begin();

    if (MDNS.begin("tank-controller"))
        logSystem("DNS Responder Attached");

    // 5. Safety Watchdog
    esp_task_wdt_init(WDT_TIMEOUT, true);
    esp_task_wdt_add(NULL);

    Serial.println("\n[BOOT] COMPLETED.");
    lastControlTimeMs = millis();
}

// ---------- MAIN RUNTIME ----------

void loop()
{
    esp_task_wdt_reset(); // Feed the dog
    server.handleClient();

    // Hardware Reset Hook (HOLD BOOT FOR 3S)
    if (digitalRead(0) == LOW)
    {
        delay(3000);
        if (digitalRead(0) == LOW)
        {
            logSystem("LOCAL WIPE COMMAND ACKNOWLEDGED");
            wm.resetSettings();
            ESP.restart();
        }
    }

    unsigned long now = millis();

    // Control Heartbeat (500ms Precision)
    if (now - lastControlTimeMs >= CONTROL_INTERVAL_MS)
    {
        lastControlTimeMs = now;

        float currentLevel = readLevelPercent();
        if (currentLevel < 0)
        {
            if (pumpOn)
                setPump(false);
            lastLevelPercent = -1.0f;
            updateAnalogOutput(0.0f);
        }
        else
        {
            lastLevelPercent = currentLevel;

            // 1. Pump Deadband Logic
            if (currentLevel >= pumpStopLevel && pumpOn)
            {
                setPump(false);
            }
            else if (currentLevel <= pumpStartLevel && !pumpOn)
            {
                setPump(true);
            }

            // 2. Valve PID Logic
            float dt = CONTROL_INTERVAL_MS / 1000.0f;
            lastPidOutput = clampValue(pid.compute(targetLevelPercent, currentLevel, dt), 0.0f, 100.0f);
            updateAnalogOutput(lastPidOutput);
        }

        // 3. Cloud Integration (500ms Pulse - JSON BATCHING OPTIMIZED)
        if (Firebase.ready() && signupOK && (now - lastFirebaseSend > 500))
        {
            lastFirebaseSend = now;

            // --- BATCH SEND STATUS ---
            FirebaseJson status;
            status.set("level_percent", lastLevelPercent);
            status.set("target_setpoint", targetLevelPercent);
            status.set("start_level", pumpStartLevel);
            status.set("stop_level", pumpStopLevel);
            status.set("pump_on", pumpOn);
            status.set("heartbeat", now);
            status.set("pid_output", lastPidOutput);
            status.set("kp", currentKp);
            status.set("ki", currentKi);
            status.set("kd", currentKd);
            status.set("dac_min_v", (currentDacMin * 3.3f / 255.0f));
            status.set("dac_max_v", (currentDacMax * 3.3f / 255.0f));
            status.set("tank_height", tankHeightCm);
            Firebase.RTDB.updateNode(&fbdo, "/tank/status", &status);

            // --- BATCH RECEIVE CONTROL/CONFIG ---
            if (Firebase.RTDB.getJSON(&fbdo, "/tank"))
            {
                FirebaseJson &json = fbdo.jsonObject();
                FirebaseJsonData data;

                // Control Pulls
                if (json.get(data, "control/target_setpoint") && data.typeNum == FirebaseJson::JSON_FLOAT)
                {
                    float v = data.floatValue;
                    if (abs(v - targetLevelPercent) > 0.1) { targetLevelPercent = v; setpointPercent = v; preferences.putFloat("targetSetpoint", v); }
                }
                if (json.get(data, "control/stop_level") && data.typeNum == FirebaseJson::JSON_FLOAT)
                {
                    float v = data.floatValue;
                    if (abs(v - pumpStopLevel) > 0.1) { pumpStopLevel = v; preferences.putFloat("stopLevel", v); }
                }
                if (json.get(data, "control/start_level") && data.typeNum == FirebaseJson::JSON_FLOAT)
                {
                    float v = data.floatValue;
                    if (abs(v - pumpStartLevel) > 0.1) { pumpStartLevel = v; lowerLimitPercent = v; preferences.putFloat("startLevel", v); }
                }

                // Config Profile Pulls
                if (json.get(data, "config/tank_height") && data.typeNum == FirebaseJson::JSON_FLOAT)
                {
                    float v = data.floatValue;
                    if (abs(v - tankHeightCm) > 0.1) { tankHeightCm = v; preferences.putFloat("tankHeight", v); }
                }

                // PID & DAC Tuning Pulls
                if (json.get(data, "config/pid/kp") && data.typeNum == FirebaseJson::JSON_FLOAT)
                {
                    float v = data.floatValue;
                    if (abs(v - currentKp) > 0.001) { currentKp = v; preferences.putFloat("kp", v); pid.setTunings(currentKp, currentKi, currentKd); logSystem("Kp Sync"); }
                }
                if (json.get(data, "config/pid/ki") && data.typeNum == FirebaseJson::JSON_FLOAT)
                {
                    float v = data.floatValue;
                    if (abs(v - currentKi) > 0.001) { currentKi = v; preferences.putFloat("ki", v); pid.setTunings(currentKp, currentKi, currentKd); logSystem("Ki Sync"); }
                }
                if (json.get(data, "config/pid/kd") && data.typeNum == FirebaseJson::JSON_FLOAT)
                {
                    float v = data.floatValue;
                    if (abs(v - currentKd) > 0.001) { currentKd = v; preferences.putFloat("kd", v); pid.setTunings(currentKp, currentKi, currentKd); logSystem("Kd Sync"); }
                }
                if (json.get(data, "config/dac/min_volt") && data.typeNum == FirebaseJson::JSON_FLOAT)
                {
                    float v = data.floatValue;
                    int u = (int)((v / 3.3f) * 255);
                    if (u != currentDacMin) { currentDacMin = constrain(u, 0, 255); preferences.putInt("dacMin", currentDacMin); logSystem("DAC Min OK"); }
                }
                if (json.get(data, "config/dac/max_volt") && data.typeNum == FirebaseJson::JSON_FLOAT)
                {
                    float v = data.floatValue;
                    int u = (int)((v / 3.3f) * 255);
                    if (u != currentDacMax) { currentDacMax = constrain(u, 0, 255); preferences.putInt("dacMax", currentDacMax); logSystem("DAC Max OK"); }
                }

                // Remote Factory Wipe
                if (json.get(data, "control/reset_wifi") && data.typeNum == FirebaseJson::JSON_BOOL && data.boolValue)
                {
                    Firebase.RTDB.setBool(&fbdo, "/tank/control/reset_wifi", false);
                    logSystem("REMOTE WIPE");
                    delay(1000); wm.resetSettings(); ESP.restart();
                }
            }

            // Print Local Diagnostics
            printDiagnostics();
        }

        // Bluetooth Pulse
        if (deviceConnected)
        {
            StaticJsonDocument<128> bDoc;
            bDoc["level"] = lastLevelPercent;
            bDoc["pump"] = pumpOn;
            String out; serializeJson(bDoc, out);
            pStatusCharacteristic->setValue(out.c_str());
            pStatusCharacteristic->notify();
        }
    }
}
