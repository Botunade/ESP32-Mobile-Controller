#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <esp_task_wdt.h>
#include <deque>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <LittleFS.h>
#include <vector>
#include <algorithm>
#include <Firebase_ESP_Client.h>
#include <Update.h>
#include <HTTPClient.h>
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
float targetLevelPercent = DEFAULT_SETPOINT;
float pumpStopLevel = DEFAULT_SETPOINT;
float pumpStartLevel = DEFAULT_LOWER_LIMIT;

// Transient State
bool pumpOn = false;
float lastLevelPercent = 0.0f;
float lastPidOutput = 0.0f;
unsigned long lastControlTimeMs = 0;

// Tank Geometry Configuration
float tankHeightCm = TANK_HEIGHT_CM;
float minDistanceCm = MIN_DISTANCE_CM;
float maxDistanceCm = MAX_DISTANCE_CM;

// AP Mode Toggle
bool alwaysOnAp = false;

// PID Controller Settings
float currentKp = PID_KP;
float currentKi = PID_KI;
float currentKd = PID_KD;
// Initialize PID with 0-100% output limits
PIDController pid(PID_KP, PID_KI, PID_KD, 0.0f, 100.0f);

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

// ---------- FIRMWARE UPDATE (OTA) ----------

void performOTA(String url) {
    if (url.length() == 0) return;

    logSystem("Starting OTA Update...");
    // Disable safety WDT or reset it frequently, OTA takes time
    esp_task_wdt_delete(NULL);

    WiFiClient client;
    HTTPClient http;
    http.begin(client, url);
    int httpCode = http.GET();

    if (httpCode == HTTP_CODE_OK) {
        int contentLength = http.getSize();
        bool canBegin = Update.begin(contentLength);

        if (canBegin) {
            logSystem("Writing Firmware...");
            WiFiClient *stream = http.getStreamPtr();
            size_t written = Update.writeStream(*stream);

            if (written == contentLength) {
                logSystem("OTA Write Done. Verifying...");
                if (Update.end()) {
                    if (Update.isFinished()) {
                        logSystem("OTA Success! Rebooting...");
                        // Clear the update flag in DB to prevent loop
                        Firebase.RTDB.setString(&fbdo, "/tank/firmware/update_url", "");
                        delay(1000);
                        ESP.restart();
                    } else {
                        logSystem("OTA Failed: Not Finished");
                    }
                } else {
                    logSystem("OTA Error: " + String(Update.getError()));
                }
            } else {
                logSystem("OTA Write Failed: Short Write");
            }
        } else {
            logSystem("OTA Init Failed: Not enough space");
        }
    } else {
        logSystem("OTA HTTP Failed: " + String(httpCode));
    }
    http.end();

    // Re-enable WDT if failed
    esp_task_wdt_init(WDT_TIMEOUT, true);
    esp_task_wdt_add(NULL);
}

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

// Helper to unify parameter updates
void updateTargetSetpoint(float v) {
    v = clampValue(v, 0.0f, 100.0f);
    if (abs(targetLevelPercent - v) > 0.1f) {
        targetLevelPercent = v;
        preferences.putFloat("targetSetpoint", v);
        logSystem("Target Setpoint Updated: " + String(v, 1));
    }
}

void updatePumpStartLevel(float v) {
    v = clampValue(v, 0.0f, 100.0f);
    if (abs(pumpStartLevel - v) > 0.1f) {
        pumpStartLevel = v;
        preferences.putFloat("startLevel", v);
        logSystem("Pump Start Level Updated: " + String(v, 1));
    }
}

void updatePumpStopLevel(float v) {
    v = clampValue(v, 0.0f, 100.0f);
    if (abs(pumpStopLevel - v) > 0.1f) {
        pumpStopLevel = v;
        preferences.putFloat("stopLevel", v);
        logSystem("Pump Stop Level Updated: " + String(v, 1));
    }
}

void updateTankHeight(float v) {
    if (v > 0 && abs(tankHeightCm - v) > 0.1f) {
        tankHeightCm = v;
        preferences.putFloat("tankHeight", v);
        logSystem("Tank Height Updated: " + String(v, 1));
    }
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
    std::vector<float> readings;
    for (int i = 0; i < samples; i++)
    {
        float r = readDistanceCm();
        if (r > 0)
            readings.push_back(r);
        delay(10);
    }
    if (readings.empty())
        return -1.0f;
    std::sort(readings.begin(), readings.end());

    if (readings.size() % 2 == 0)
    {
        return (readings[readings.size() / 2 - 1] + readings[readings.size() / 2]) / 2.0f;
    }
    else
    {
        return readings[readings.size() / 2];
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
    // Map 0-100% to DAC_MIN (0.66V) - DAC_MAX (3.3V)
    // Using float math for better precision before casting
    float range = (float)(currentDacMax - currentDacMin);
    float val = (float)currentDacMin + (pidOutputPercent / 100.0f) * range;

    int dacValue = (int)val;
    dacValue = constrain(dacValue, currentDacMin, currentDacMax);
    dacWrite(ANALOG_OUTPUT_PIN, dacValue);
}

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
    status["ap_mode_active"] = alwaysOnAp;
    status["firmware_version"] = FIRMWARE_VERSION;

    // Aliases for compatibility
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
        // Aliases
        cfg["setpoint"] = targetLevelPercent;
        cfg["lower_limit"] = pumpStartLevel;

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

        if (update.containsKey("target_setpoint")) updateTargetSetpoint(update["target_setpoint"]);
        if (update.containsKey("setpoint")) updateTargetSetpoint(update["setpoint"]);

        if (update.containsKey("start_level")) updatePumpStartLevel(update["start_level"]);
        if (update.containsKey("lower_limit")) updatePumpStartLevel(update["lower_limit"]);

        if (update.containsKey("stop_level")) updatePumpStopLevel(update["stop_level"]);

        if (update.containsKey("tank_height_cm")) updateTankHeight(update["tank_height_cm"]);

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

    if (pidDoc.containsKey("setpoint")) updateTargetSetpoint(pidDoc["setpoint"]);
    if (pidDoc.containsKey("lower_limit")) updatePumpStartLevel(pidDoc["lower_limit"]);

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

    alwaysOnAp = preferences.getBool("alwaysOnAp", false);

    currentKp = preferences.getFloat("kp", PID_KP);
    currentKi = preferences.getFloat("ki", PID_KI);
    currentKd = preferences.getFloat("kd", PID_KD);
    currentDacMin = preferences.getInt("dacMin", DAC_MIN_VAL);
    currentDacMax = preferences.getInt("dacMax", DAC_MAX_VAL);

    pid.setTunings(currentKp, currentKi, currentKd);
    pid.setOutputLimits(0.0f, 100.0f);

    // 2. Hardware Mapping
    pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
    pinMode(ULTRASONIC_ECHO_PIN, INPUT);
    pinMode(PUMP_RELAY_PIN, OUTPUT);
    pinMode(0, INPUT_PULLUP);

    setPump(false);
    dacWrite(ANALOG_OUTPUT_PIN, 0); // Start at 0V

    // 3. WiFi Connectivity
    wm.setSaveConfigCallback(saveConfigCallback);

    char hStr[10]; dtostrf(tankHeightCm, 1, 1, hStr);
    char mStr[10]; dtostrf(maxDistanceCm, 1, 1, mStr);
    char tStr[10]; dtostrf(targetLevelPercent, 1, 1, tStr);

    // Create a custom checkbox parameter for AP Mode
    char apStr[4];
    if (alwaysOnAp) strcpy(apStr, "1"); else strcpy(apStr, "0");

    WiFiManagerParameter custom_h("h", "Tank Depth (cm)", hStr, 6);
    WiFiManagerParameter custom_m("m", "Sensor Gap (cm)", mStr, 6);
    WiFiManagerParameter custom_t("t", "Primary Setpoint (%)", tStr, 6);
    WiFiManagerParameter custom_ap("ap", "Enable Always-On AP (1=Yes, 0=No)", apStr, 3);

    wm.addParameter(&custom_h);
    wm.addParameter(&custom_m);
    wm.addParameter(&custom_t);
    wm.addParameter(&custom_ap);

    // Initial Mode Set
    if (alwaysOnAp) {
        WiFi.mode(WIFI_AP_STA);
        Serial.println("[BOOT] Mode: AP+STA (Persistent)");
    } else {
        Serial.println("[BOOT] Mode: Standard");
    }

    if (!wm.autoConnect("TankLogic-Setup", "tank1234"))
    {
        Serial.println("[WIFI] Critical Fail. Restarting.");
        ESP.restart();
    }

    // Force AP to stay on with defined credentials
    if (alwaysOnAp || WiFi.getMode() == WIFI_AP_STA) {
        WiFi.softAP(AP_SSID, AP_PASSWORD);
        Serial.print("[WIFI] Persistent AP Active: ");
        Serial.println(WiFi.softAPIP());
    }

    Serial.print("[WIFI] STA Connected: ");
    Serial.println(WiFi.localIP());

    // Apply Saved Config
    if (shouldSaveConfig)
    {
        float h = atof(custom_h.getValue());
        float m = atof(custom_m.getValue());
        float t = atof(custom_t.getValue());
        int apVal = atoi(custom_ap.getValue());

        updateTankHeight(h);
        if (abs(maxDistanceCm - m) > 0.1f) {
            maxDistanceCm = m;
            preferences.putFloat("maxDist", m);
        }
        updateTargetSetpoint(t);

        bool newApState = (apVal == 1);
        if (newApState != alwaysOnAp) {
            alwaysOnAp = newApState;
            preferences.putBool("alwaysOnAp", alwaysOnAp);
            Serial.print("[CONFIG] AP Mode Updated: ");
            Serial.println(alwaysOnAp ? "ON" : "OFF");
            // If turning on, enable AP immediately
            if (alwaysOnAp) {
                WiFi.mode(WIFI_AP_STA);
                WiFi.softAP(AP_SSID, AP_PASSWORD);
            }
        }
    }

    // 4. External Services
    config.api_key = FIREBASE_API_KEY;
    config.database_url = FIREBASE_DATABASE_URL;
    if (Firebase.signUp(&config, &auth, "", ""))
        signupOK = true;
    Firebase.begin(&config, &auth);
    Firebase.reconnectWiFi(true);

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
            // Sensor Error - Fail Safe
            if (pumpOn) setPump(false);
            lastLevelPercent = -1.0f;

            // Output 0V for safety
            dacWrite(ANALOG_OUTPUT_PIN, 0);
            lastPidOutput = 0.0f;
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

            // 2. Valve / Actuator Logic
            if (!pumpOn)
            {
                // Force 0V (absolute zero, not 0.66V minimum)
                dacWrite(ANALOG_OUTPUT_PIN, 0);
                lastPidOutput = 0.0f;
                // Optionally reset PID integral to avoid windup during off-time
                pid.reset();
            }
            else
            {
                // Normal Operation: PID Control (0.66V - 3.3V)
                float dt = CONTROL_INTERVAL_MS / 1000.0f;
                lastPidOutput = clampValue(pid.compute(targetLevelPercent, currentLevel, dt), 0.0f, 100.0f);
                updateAnalogOutput(lastPidOutput);
            }
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
            status.set("firmware_version", FIRMWARE_VERSION);
            Firebase.RTDB.updateNode(&fbdo, "/tank/status", &status);

            // --- BATCH RECEIVE CONTROL/CONFIG ---
            if (Firebase.RTDB.getJSON(&fbdo, "/tank"))
            {
                FirebaseJson &json = fbdo.jsonObject();
                FirebaseJsonData data;

                // OTA Trigger Check
                if (json.get(data, "firmware/update_url") && data.typeNum == FirebaseJson::JSON_STRING)
                {
                    String url = data.stringValue;
                    if (url.length() > 10) {
                        performOTA(url);
                    }
                }

                // Control Pulls
                if (json.get(data, "control/target_setpoint") && data.typeNum == FirebaseJson::JSON_FLOAT)
                    updateTargetSetpoint(data.floatValue);

                if (json.get(data, "control/stop_level") && data.typeNum == FirebaseJson::JSON_FLOAT)
                    updatePumpStopLevel(data.floatValue);

                if (json.get(data, "control/start_level") && data.typeNum == FirebaseJson::JSON_FLOAT)
                    updatePumpStartLevel(data.floatValue);

                // Config Profile Pulls
                if (json.get(data, "config/tank_height") && data.typeNum == FirebaseJson::JSON_FLOAT)
                    updateTankHeight(data.floatValue);

                // PID & DAC Tuning Pulls
                bool tuningsChanged = false;
                if (json.get(data, "config/pid/kp") && data.typeNum == FirebaseJson::JSON_FLOAT)
                {
                    float v = data.floatValue;
                    if (abs(v - currentKp) > 0.001) { currentKp = v; preferences.putFloat("kp", v); tuningsChanged = true; logSystem("Kp Sync"); }
                }
                if (json.get(data, "config/pid/ki") && data.typeNum == FirebaseJson::JSON_FLOAT)
                {
                    float v = data.floatValue;
                    if (abs(v - currentKi) > 0.001) { currentKi = v; preferences.putFloat("ki", v); tuningsChanged = true; logSystem("Ki Sync"); }
                }
                if (json.get(data, "config/pid/kd") && data.typeNum == FirebaseJson::JSON_FLOAT)
                {
                    float v = data.floatValue;
                    if (abs(v - currentKd) > 0.001) { currentKd = v; preferences.putFloat("kd", v); tuningsChanged = true; logSystem("Kd Sync"); }
                }

                if (tuningsChanged) pid.setTunings(currentKp, currentKi, currentKd);

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
    }
}
