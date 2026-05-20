// ============================================================
// ARDUINO GIGA R1 - PEM ELECTROLYZER CONTROLLER (merged)
// FINAL FIX: COMPLETELY USB INDEPENDENT
// ============================================================

#include <Wire.h>
#include "RTClib.h"
#include <WiFi.h>
#include <ArduinoJson.h>
#include <math.h>

// CRITICAL: Set MQTT max packet size BEFORE including PubSubClient
#define MQTT_MAX_PACKET_SIZE 1024

#include <PubSubClient.h>

// ========== HARDWARE DEFINITIONS ==========
#define BAT_PIN A0
#define TEMP_SENSOR A1
#define VOLTAGE_SENSOR A2
#define CURRENT_SENSOR A3
#define GRID_RELAY 8
#define PV_RELAY 9
#define BUZZER_PIN 7

// ========== NETWORK CONFIGURATION ==========
const char* ssid = "Galaxy A14 FD8D";
const char* password = "qwertyzxcvbnm";

// MQTT Configuration
const char* mqttBroker = "broker.hivemq.com";
const int mqttPort = 1883;
const char* mqttClientId = "arduino_pem_giga_industrial";

// ========== HARDWARE SERIAL PORTS ==========
// GSM Module on Serial2 (pins D19/D18)
HardwareSerial &simSerial = Serial2;

// Nextion HMI on Serial3 (pins D17/D16)
HardwareSerial &nexSerial = Serial3;

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
RTC_DS3231 rtc;

// ========== GSM AT COMMAND CONSTANTS ==========
const char CMD_AT[] = "AT\r\n";
const char CMD_AT_CSQ[] = "AT+CSQ\r\n";
const char CMD_AT_CPIN[] = "AT+CPIN?\r\n";
const char CMD_AT_CGATT[] = "AT+CGATT?\r\n";
const char CMD_AT_CSTT[] = "AT+CSTT=\"safaricom\"\r\n";
const char CMD_AT_CIICR[] = "AT+CIICR\r\n";
const char CMD_AT_CIFSR[] = "AT+CIFSR\r\n";
const char CMD_AT_CIPSTART[] = "AT+CIPSTART=\"TCP\",\"broker.hivemq.com\",\"1883\"\r\n";
const char CMD_AT_CIPSEND[] = "AT+CIPSEND\r\n";

// HTTP Backup endpoint (webhook for dashboard)
const char* httpBackupEndpoint = "http://your-dashboard.com/api/pem-data";
const char* httpBackupHost = "your-dashboard.com";
const int httpBackupPort = 80;

// ========== GSM / GPRS CONFIGURATION ==========
bool gsmConnected = false;
bool usingGSM = false;
bool gsmInitialized = false;
const uint32_t gsmBaud = 115200;
unsigned long lastGsmCheckup = 0;
const unsigned long GSM_CHECKUP_MS = 10000;

// ========== TIMING CONFIGURATION ==========
unsigned long lastFullUpdate = 0;
unsigned long lastMqttUpdate = 0;
unsigned long lastSafetyCheck = 0;
unsigned long lastMPCUpdate = 0;
unsigned long lastEfficiencyUpdate = 0;
unsigned long lastNextionHeartbeat = 0;
unsigned long lastNextionInit = 0;
unsigned long lastSliderUpdate = 0;  // Track slider updates
unsigned long lastStatusUpdate = 0;  // Track status heartbeat

const unsigned long FULL_UPDATE_MS = 5000;
const unsigned long MQTT_UPDATE_MS = 1000;
const unsigned long SAFETY_CHECK_MS = 500;
const unsigned long MPC_UPDATE_MS = 100;
const unsigned long EFFICIENCY_UPDATE_MS = 2000;
const unsigned long NEXTION_HEARTBEAT_MS = 1000;  // Send heartbeat every 1 second (more frequent)
const unsigned long NEXTION_INIT_RETRY_MS = 2000; // Retry init every 2 seconds if failed
const unsigned long SLIDER_UPDATE_MS = 100;       // Update slider display every 100ms
const unsigned long STATUS_UPDATE_MS = 3000;      // Send status heartbeat every 3 seconds

// ========== STATE MACHINE ==========
enum SystemMode { MODE_NONE, MODE_AUTO, MODE_MANUAL, MODE_MPC, MODE_ECONOMIC };
enum SystemState { STATE_WAITING, STATE_STARTING, STATE_RUNNING, STATE_STOPPING, STATE_STOPPED };

SystemMode systemMode = MODE_NONE;
SystemState systemState = STATE_WAITING;
unsigned long stateStartTime = 0;
const unsigned long START_DELAY = 2000;
const unsigned long STOP_DELAY = 2000;

// ========== CONNECTIVITY STATUS ==========
bool wifiOK = false;
bool gsmOK = false;
bool mqttConnected = false;
int gsmRssiRaw = 99;
bool nextionActive = false;
bool nextionInitialized = false;

// ========== CONTROL VARIABLES ==========
float prodRateSetpoint = 30.0f;
int prodRateSetpointInt = 30;  // Integer version for display
float mpcOptimalCurrent = 150.0f;
float appliedCurrent = 150.0f;
float manualCurrentSetpoint = 150.0f;
bool sliderMoved = false;
bool mpcControlActive = false;

// ========== ASCII GAUGE FOR EFFICIENCY ==========
const char gaugeMap[72] = "()*+,-./0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_abcdefghijklmno";
float smoothEfficiency = 0.0f;

// ========== DUAL POWER CONTROL ==========
struct PowerConfig {
  String source = "auto";
  float gridRatio = 0.5;
  float pvRatio = 0.5;
  bool gridAvailable = true;
  bool pvAvailable = true;
  float gridPrice = 0.15;
  float pvPrice = 0.05;
} powerConfig;

// ========== MPC PARAMETERS ==========
const int MPC_HORIZON = 5;
const float MPC_SAMPLE_TIME = 0.1;
float mpcStates[MPC_HORIZON] = {0};
float mpcControls[MPC_HORIZON] = {0};
float mpcReference[MPC_HORIZON] = {0};

const float A_MATRIX = 0.95;
const float B_MATRIX = 0.8;
const float C_MATRIX = 1.0;
const float Q_WEIGHT = 10.0;
const float R_WEIGHT = 1.0;
const float S_WEIGHT = 100.0;

// ========== SAFETY PARAMETERS ==========
const float MAX_CURRENT = 200.0f;
const float MIN_CURRENT = 100.0f;
const float MAX_TEMPERATURE = 80.0f;
const float MIN_O2_PURITY = 99.0f;
const float MAX_VOLTAGE = 45.0f;

// ========== SENSOR VALUES ==========
float cellTemperature = 65.0f;
float stackVoltage = 38.0f;
float stackCurrent = 150.0f;
float o2Purity = 99.5f;
float h2ProductionRate = 0.0f;
float o2ProductionRate = 0.0f;

// ========== ADC CONFIGURATION ==========
const int ADC_BITS = 12;
const float ADC_REF_VOLTAGE = 3.3f;
const float R_TOP = 1000000.0f;
const float R_BOTTOM = 100000.0f;
const float VOLTAGE_DIVIDER_FACTOR = (R_TOP + R_BOTTOM) / R_BOTTOM;

// ========== WATER TANK SIMULATION ==========
float water_tank = 100.0f;
const float water_tank_initial = 100.0f;
const float water_low_threshold = 10.0f;
const float water_emergency_threshold = 5.0f;
float water_consumption_scale = 100.0f;
const float liters_per_s_per_A = 18.015f / (2.0f * 96485.0f * 1000.0f);
bool water_alert_published = false;

// ========== MPC CONTROLLER CLASS ==========
class IndustrialMPC {
private:
    float states[MPC_HORIZON];
    float controls[MPC_HORIZON];
    float reference[MPC_HORIZON];
    
public:
    IndustrialMPC() {
        for(int i = 0; i < MPC_HORIZON; i++) {
            states[i] = 0.0f;
            controls[i] = 0.0f;
            reference[i] = 0.0f;
        }
    }
    
    void setReference(float setpoint) {
        for(int i = 0; i < MPC_HORIZON; i++) {
            reference[i] = setpoint;
        }
    }
    
    float computeControl(float currentState, float previousControl, PowerConfig power) {
        float optimalControl = previousControl;
        float minCost = 1e9;
        
        for(float u = max(MIN_CURRENT, previousControl - 20.0f); 
            u <= min(MAX_CURRENT, previousControl + 20.0f); 
            u += 5.0f) {
            
            float cost = evaluateCost(currentState, u, power);
            
            if(cost < minCost) {
                minCost = cost;
                optimalControl = u;
            }
        }
        
        return optimalControl;
    }
    
private:
    float evaluateCost(float currentState, float control, PowerConfig power) {
        float totalCost = 0.0f;
        float state = currentState;
        
        for(int k = 0; k < MPC_HORIZON; k++) {
            state = A_MATRIX * state + B_MATRIX * control;
            
            float trackingError = state - reference[k];
            totalCost += Q_WEIGHT * trackingError * trackingError;
            
            if(k > 0) {
                float powerCost = control * 1.8 * (power.gridRatio * power.gridPrice + power.pvRatio * power.pvPrice) / 1000.0;
                totalCost += R_WEIGHT * control * control + powerCost * 10.0;
            }
        }
        
        float terminalError = state - reference[MPC_HORIZON-1];
        totalCost += S_WEIGHT * terminalError * terminalError;
        
        return totalCost;
    }
    
    float max(float a, float b) { return (a > b) ? a : b; }
    float min(float a, float b) { return (a < b) ? a : b; }
};

IndustrialMPC industrialMPC;

// ========== MQTT FUNCTIONS ==========
void setupMQTT() {
  mqttClient.setServer(mqttBroker, mqttPort);
  mqttClient.setCallback(mqttCallback);
  ensureMQTTConnected();
}

// MQTT connection - requires WiFi (preferred) or GSM fallback
void ensureMQTTConnected() {
  if (!mqttClient.connected()) {
    Serial.print("[MQTT] Connecting to broker...");
    int state = mqttClient.connect(mqttClientId);
    if (state) {
      Serial.println(" [OK] 💚");
      Serial.print("[MQTT] Connection state: "); Serial.println(state);
      mqttClient.subscribe("pem/control");
      mqttClient.subscribe("pem/current");
      mqttClient.subscribe("matlab/control");
      mqttClient.subscribe("matlab/current");
      mqttClient.subscribe("web/control");
      mqttClient.subscribe("web/mpc/decision");
      mqttClient.subscribe("power/source/selection");
      mqttClient.subscribe("power/blending/ratio");
      mqttClient.subscribe("power/mpc/decision");
      mqttClient.subscribe("power/grid/setpoint");
      mqttClient.subscribe("matlab/water/alert");
      mqttClient.subscribe("web/water/set");
      mqttClient.subscribe("web/water/command");
      mqttClient.subscribe("pem/web/control");  // Add dashboard control topic
      
      Serial.println("[MQTT] Subscribed to all control topics");
      mqttClient.publish("arduino/status", "connected");
      mqttConnected = true;
    } else {
      Serial.print("[MQTT] Connection failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" - Retrying in 5s");
      mqttConnected = false;
    }
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  Serial.print("[MQTT] RX ");
  Serial.print(topic);
  Serial.print(": ");
  Serial.println(message);
  
  // Try to parse as JSON first (for web/control commands)
  StaticJsonDocument<256> jsonCmd;
  bool isJson = false;
  if (strstr(topic, "web/") != nullptr) {
    DeserializationError err = deserializeJson(jsonCmd, message);
    if (!err) {
      isJson = true;
      // Handle manualProduction from web/control
      if (jsonCmd.containsKey("manualProduction")) {
        float prodValue = jsonCmd["manualProduction"];
        prodValue = constrain(prodValue, 0.0f, 100.0f);
        prodRateSetpoint = prodValue;
        prodRateSetpointInt = (int)round(prodValue);
        serializeJson(jsonCmd, Serial);
        Serial.print(" -> Production Rate: ");
        Serial.print(prodValue);
        Serial.println("%");
        // Update HMI slider immediately
        setNumber("h0", prodRateSetpointInt);
      }
      // Handle other JSON commands here if needed
    }
  }
  
  // Handle string-based commands
  if (!isJson) {
    if (strcmp(topic, "pem/control") == 0 || 
        strcmp(topic, "matlab/control") == 0 || 
        strcmp(topic, "web/control") == 0) {
      
      if (message == "start") {
        if (systemState == STATE_WAITING || systemState == STATE_STOPPED) {
          systemState = STATE_STARTING;
          stateStartTime = millis();
          updateSystemStateOnHMI();
          Serial.println("[Command] Remote START received");
        }
      } else if (message == "stop") {
        if (systemState == STATE_RUNNING || systemState == STATE_STARTING) {
          systemState = STATE_STOPPING;
          stateStartTime = millis();
          updateSystemStateOnHMI();
          Serial.println("[Command] Remote STOP received");
        }
      }
    }
    // Handle dashboard commands from index.html
    else if (strcmp(topic, "pem/web/control") == 0) {
      if (message == "START") {
        if (systemState == STATE_WAITING || systemState == STATE_STOPPED) {
          systemState = STATE_STARTING;
          stateStartTime = millis();
          updateSystemStateOnHMI();
          Serial.println("[Dashboard] START command received");
        }
      } else if (message == "STOP") {
        if (systemState == STATE_RUNNING || systemState == STATE_STARTING) {
          systemState = STATE_STOPPING;
          stateStartTime = millis();
          updateSystemStateOnHMI();
          Serial.println("[Dashboard] STOP command received");
        }
      } else if (message == "AUTO") {
        systemMode = MODE_AUTO;
        if(systemState == STATE_STARTING||systemState == STATE_WAITING||systemState == STATE_STOPPED) systemState = STATE_RUNNING;
        updateSystemStateOnHMI();
        Serial.println("[Dashboard] AUTO mode selected");
      } else if (message == "MANUAL") {
        systemMode = MODE_MANUAL;
        if(systemState == STATE_STARTING||systemState == STATE_WAITING||systemState == STATE_STOPPED) systemState = STATE_RUNNING;
        updateSystemStateOnHMI();
        Serial.println("[Dashboard] MANUAL mode selected");
      }
    }
    else if (strcmp(topic, "pem/current") == 0 || 
             strcmp(topic, "matlab/current") == 0) {
      float newCurrent = message.toFloat();
    if (newCurrent >= 50 && newCurrent <= 200) {
      appliedCurrent = newCurrent;
      Serial.print("🎯 Current set to: ");
      Serial.print(newCurrent);
      Serial.println("A");
    }
  }
  else if (strcmp(topic, "power/source/selection") == 0) {
    powerConfig.source = message;
    Serial.print("[Power] Source: ");
    Serial.println(message);
    updatePowerRelays();
  }
  else if (strcmp(topic, "power/blending/ratio") == 0) {
    float newRatio = message.toFloat();
    if (newRatio >= 0 && newRatio <= 1) {
      powerConfig.gridRatio = newRatio;
      powerConfig.pvRatio = 1.0 - newRatio;
      Serial.print("[Power] Blend - Grid: ");
      Serial.print(powerConfig.gridRatio * 100);
      Serial.print("%, PV: ");
      Serial.print(powerConfig.pvRatio * 100);
      Serial.println("%");
      updatePowerRelays();
    }
  }
  else if (strcmp(topic, "web/mpc/decision") == 0 || 
           strcmp(topic, "power/mpc/decision") == 0) {
    try {
      StaticJsonDocument<256> doc;
      DeserializationError error = deserializeJson(doc, message);
      if (!error) {
        if (doc.containsKey("grid_ratio")) {
          powerConfig.gridRatio = doc["grid_ratio"];
          powerConfig.pvRatio = 1.0 - powerConfig.gridRatio;
          updatePowerRelays();
          Serial.println("[MPC] Web MPC decision applied");
        }
        if (doc.containsKey("optimal_current")) {
          appliedCurrent = doc["optimal_current"];
          Serial.print("[MPC] Optimal current: ");
          Serial.print(appliedCurrent);
          Serial.println("A");
        }
      }
    } catch (...) {
      Serial.println("[MPC] Error parsing MPC decision");
    }
  }
  else if (strcmp(topic, "matlab/water/alert") == 0) {
    StaticJsonDocument<256> doc;
    DeserializationError err = deserializeJson(doc, message);
    if (!err) {
      const char* alert = doc["alert"];
      if(alert && String(alert) == "LOW_WATER"){
         float mwater = doc["water_l"] | -1.0;
         Serial.print("[MQTT] MATLAB water alert: ");
         Serial.println(mwater);
            digitalWrite(BUZZER_PIN, HIGH);
      }
    }
  }
  else if (strcmp(topic, "web/water/set") == 0) {
    float newL = message.toFloat();
    if (newL >= 0.0) {
      water_tank = newL;
      water_alert_published = false;
      digitalWrite(BUZZER_PIN, LOW);
      // Water level updated
      Serial.print("[MQTT] Water level set by web: ");
      Serial.println(newL);
    }
  }
  else if (strcmp(topic, "web/water/command") == 0) {
    if (message == "refill") {
      water_tank = water_tank_initial;
      water_alert_published = false;
      digitalWrite(BUZZER_PIN, LOW);
      // Water tank refilled
      Serial.println("[MQTT] Refilled water tank");
    } else if (message == "reset_alerts") {
      water_alert_published = false;
      digitalWrite(BUZZER_PIN, LOW);
      // Alerts reset
      Serial.println("[MQTT] Alerts reset");
    }
  }
  }
}

// ========== MQTT DATA TRANSMISSION ==========
// Published to: arduino/sensors via MQTT (WiFi preferred)
// Falls back to HTTP POST via GPRS if WiFi fails
void sendStatusHeartbeat() {
  if (!mqttClient.connected()) {
    return;  // Can't send without connection
  }
  
  // Send status with timestamp to keep dashboard updated
  StaticJsonDocument<256> status;
  status["device_id"] = "arduino_pem_giga_001";
  status["status"] = "connected";
  status["uptime_ms"] = millis();
  status["wifi_ok"] = wifiOK;
  status["gsm_ok"] = gsmOK;
  status["mqtt_connected"] = mqttClient.connected();
  
  DateTime now = rtc.now(); 
  char dateStr[64]; 
  snprintf(dateStr, sizeof(dateStr), "%02d-%02d-%04d %02d:%02d:%02d", 
           now.day(), now.month(), now.year(), now.hour(), now.minute(), now.second()); 
  status["rtcDateTime"] = dateStr;
  
  char buffer[256];
  serializeJson(status, buffer);
  
  mqttClient.publish("arduino/status", buffer);
  Serial.print("💚 Status heartbeat: ");
  Serial.println(buffer);
}

// ========== MQTT DATA TRANSMISSION ==========
// Published to: arduino/sensors via MQTT (WiFi preferred)
// Falls back to HTTP POST via GPRS if WiFi fails
void sendMQTTData() {
  // Try primary transport first (WiFi)
  if (wifiOK && mqttClient.connected()) {
    if (!mqttClient.connected()) {
      ensureMQTTConnected();
      return;
    }
    
    StaticJsonDocument<1024> doc;
    float safeProduction = constrain(prodRateSetpoint, 0.0f, 100.0f);
    int safeSlider = constrain((int)round(prodRateSetpoint), 0, 100);
    
    doc["prodRateSet"] = safeProduction;
    doc["mode"] = systemMode == MODE_AUTO ? "AUTO" : 
                  systemMode == MODE_MANUAL ? "MANUAL" : 
                  systemMode == MODE_MPC ? "MPC" : 
                  systemMode == MODE_ECONOMIC ? "ECONOMIC" : "NONE";
    doc["state"] = (int)systemState;
    doc["battery"] = smoothBatteryVoltage();
    doc["gsm"] = gsmRssiRaw;
    doc["slider"] = safeSlider;
    doc["temperature"] = cellTemperature;
    doc["voltage"] = stackVoltage;
    doc["current"] = stackCurrent;
    doc["o2Purity"] = o2Purity;
    doc["appliedCurrent"] = appliedCurrent;
    doc["mpcControlActive"] = mpcControlActive;
    doc["h2ProductionRate"] = h2ProductionRate;
    doc["o2ProductionRate"] = o2ProductionRate;
    doc["power_source"] = powerConfig.source;
    doc["grid_ratio"] = powerConfig.gridRatio;
    doc["pv_ratio"] = powerConfig.pvRatio;
    doc["grid_available"] = powerConfig.gridAvailable;
    doc["pv_available"] = powerConfig.pvAvailable;
    doc["water_tank_l"] = water_tank;
    doc["water_low"] = (water_tank <= water_low_threshold) ? 1 : 0;
    doc["efficiency"] = smoothEfficiency;
    
    DateTime now = rtc.now(); 
    char dateStr[64]; 
    snprintf(dateStr, sizeof(dateStr), "%02d-%02d-%04d %02d:%02d:%02d", 
             now.day(), now.month(), now.year(), now.hour(), now.minute(), now.second()); 
    doc["rtcDateTime"] = dateStr;
    
    char buffer[1024];
    int bufLen = serializeJson(doc, buffer);
    
    if (bufLen == 0) {
      Serial.println("📤 MQTT TX (WiFi): ❌ SERIALIZE FAILED (buffer too small or doc too big)");
      return;
    }
    
    Serial.print("📤 Serialized ");
    Serial.print(bufLen);
    Serial.print(" bytes | ");
    
    // CRITICAL: Split large payload into 2 smaller publishes to avoid buffer overflow
    // This ensures each publish succeeds individually
    
    // Part 1: System info + production data
    StaticJsonDocument<512> doc1;
    doc1["prodRateSet"] = safeProduction;
    doc1["mode"] = systemMode == MODE_AUTO ? "AUTO" : 
                  systemMode == MODE_MANUAL ? "MANUAL" : 
                  systemMode == MODE_MPC ? "MPC" : 
                  systemMode == MODE_ECONOMIC ? "ECONOMIC" : "NONE";
    doc1["state"] = (int)systemState;
    doc1["battery"] = smoothBatteryVoltage();
    doc1["slider"] = safeSlider;
    doc1["temperature"] = cellTemperature;
    doc1["voltage"] = stackVoltage;
    doc1["current"] = stackCurrent;
    
    char buf1[512];
    serializeJson(doc1, buf1);
    bool pub1 = mqttClient.publish("arduino/sensors/primary", buf1);
    Serial.print(pub1 ? "✅P1 " : "❌P1 ");
    
    // Part 2: Efficiency + power data
    StaticJsonDocument<512> doc2;
    doc2["o2Purity"] = o2Purity;
    doc2["h2ProductionRate"] = h2ProductionRate;
    doc2["o2ProductionRate"] = o2ProductionRate;
    doc2["power_source"] = powerConfig.source;
    doc2["grid_ratio"] = powerConfig.gridRatio;
    doc2["pv_ratio"] = powerConfig.pvRatio;
    doc2["water_tank_l"] = water_tank;
    doc2["efficiency"] = smoothEfficiency;
    doc2["rtcDateTime"] = dateStr;
    
    char buf2[512];
    serializeJson(doc2, buf2);
    bool pub2 = mqttClient.publish("arduino/sensors/secondary", buf2);
    Serial.print(pub2 ? "✅P2" : "❌P2");
    
    Serial.print(" | State=");
    Serial.println(mqttClient.state());
  }
  // Fallback to GSM/HTTP if WiFi is down
  else if (!wifiOK && gsmConnected) {
    Serial.println("📤 WiFi down - using GSM/GPRS backup...");
    sendDataViaGSMHTTP();
  }
}

// ========== POWER RELAY CONTROL ==========
void updatePowerRelays() {
  if (powerConfig.gridRatio > 0.1 && powerConfig.gridAvailable) {
    digitalWrite(GRID_RELAY, HIGH);
  } else {
    digitalWrite(GRID_RELAY, LOW);
  }
  
  if (powerConfig.pvRatio > 0.1 && powerConfig.pvAvailable) {
    digitalWrite(PV_RELAY, HIGH);
  } else {
    digitalWrite(PV_RELAY, LOW);
  }
  
  Serial.print("🔌 Power relays - Grid: ");
  Serial.print(digitalRead(GRID_RELAY) ? "ON" : "OFF");
  Serial.print(", PV: ");
  Serial.println(digitalRead(PV_RELAY) ? "ON" : "OFF");
}

// ========== NEXTION HMI FUNCTIONS ==========
void sendToNextion(const char *fmt, ...) {
  if (!nextionActive) return;
  
  char buf[300];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  
  nexSerial.print(buf);
  nexSerial.write(0xFF); 
  nexSerial.write(0xFF); 
  nexSerial.write(0xFF);
}

void setText(const char *component, const char *txt) {
  char buf[340]; 
  snprintf(buf, sizeof(buf), "%s.txt=\"%s\"", component, txt);
  sendToNextion("%s", buf);
}

void setText(const char *component, String s) { 
  setText(component, s.c_str()); 
}

void setNumber(const char *component, int val) {
  char buf[100];
  snprintf(buf, sizeof(buf), "%s.val=%d", component, val);
  sendToNextion("%s", buf);
}

// FIXED: Proper Nextion initialization that works WITHOUT USB
void initNextionDisplay() {
  Serial.println("📺 Initializing Nextion display (standalone mode)...");
  
  // CRITICAL FIX: Initialize Serial3 FIRST, before anything else
  nexSerial.begin(9600);
  delay(1000);  // Give the display time to power up and stabilize
  
  // Clear any pending data
  while(nexSerial.available()) {
    nexSerial.read();
  }
  
  // Send wake-up sequence multiple times
  for(int i = 0; i < 3; i++) {
    nexSerial.print("sleep=0");
    nexSerial.write(0xFF); nexSerial.write(0xFF); nexSerial.write(0xFF);
    delay(100);
    
    nexSerial.print("thup=1");
    nexSerial.write(0xFF); nexSerial.write(0xFF); nexSerial.write(0xFF);
    delay(100);
    
    nexSerial.print("dim=100");
    nexSerial.write(0xFF); nexSerial.write(0xFF); nexSerial.write(0xFF);
    delay(100);
  }
  
  // Set to page 0
  nexSerial.print("page 0");
  nexSerial.write(0xFF); nexSerial.write(0xFF); nexSerial.write(0xFF);
  delay(200);
  
  // Clear screen by sending spaces to text fields
  nexSerial.print("t0.txt=\"--\"");
  nexSerial.write(0xFF); nexSerial.write(0xFF); nexSerial.write(0xFF);
  nexSerial.print("t1.txt=\"--\"");
  nexSerial.write(0xFF); nexSerial.write(0xFF); nexSerial.write(0xFF);
  nexSerial.print("t2.txt=\"--\"");
  nexSerial.write(0xFF); nexSerial.write(0xFF); nexSerial.write(0xFF);
  nexSerial.print("t3.txt=\"--\"");
  nexSerial.write(0xFF); nexSerial.write(0xFF); nexSerial.write(0xFF);
  nexSerial.print("t4.txt=\"--\"");
  nexSerial.write(0xFF); nexSerial.write(0xFF); nexSerial.write(0xFF);
  
  // Check if display responded
  unsigned long timeout = millis() + 2000;
  bool response = false;
  while (millis() < timeout) {
    if (nexSerial.available()) {
      response = true;
      while (nexSerial.available()) nexSerial.read();
      break;
    }
    delay(10);
  }
  
  if (response) {
    nextionActive = true;
    nextionInitialized = true;
    Serial.println("✅ Nextion display initialized and responding");
    
    // Set initial values
    setText("t7", "WAITING");
    setText("t4", "65.0C");
    setText("t6", " ");
    setText("t9", "0%");
    setNumber("j3", 0);  // H2 gauge
    setNumber("j2", 0);  // O2 gauge
    setText("t12", "INITIALIZING");
    setNumber("j0", 100);  // Water gauge
    setNumber("h0", (int)round(prodRateSetpoint));
    
    // FIXED: Initialize slider value on display
    setNumber("h0", prodRateSetpointInt);
  } else {
    nextionActive = false;
    nextionInitialized = false;
    Serial.println("⚠️ Nextion display not responding - will retry");
  }
}

// Send heartbeat to keep Nextion awake
void nextionHeartbeat() {
  if (!nextionActive) {
    if (!nextionInitialized && millis() - lastNextionInit >= NEXTION_INIT_RETRY_MS) {
      lastNextionInit = millis();
      initNextionDisplay();
    }
    return;
  }
  
  // Send keep-alive commands
  nexSerial.print("sleep=0");
  nexSerial.write(0xFF); nexSerial.write(0xFF); nexSerial.write(0xFF);
  delay(5);
  nexSerial.print("thup=1");
  nexSerial.write(0xFF); nexSerial.write(0xFF); nexSerial.write(0xFF);
}

// Update slider value on HMI
void updateSliderOnHMI() {
  if (!nextionActive) return;
  
  // Only update if value changed
  static int lastSliderValue = -1;
  if (prodRateSetpointInt != lastSliderValue) {
    setNumber("h0", prodRateSetpointInt);
    lastSliderValue = prodRateSetpointInt;
    Serial.print("🎚️ Slider updated on HMI: ");
    Serial.println(prodRateSetpointInt);
  }
}

// ========== BATTERY MONITORING ==========
float readBatteryVoltage() { 
  float rawVoltage = (float)analogRead(BAT_PIN) / ((1 << ADC_BITS) - 1) * ADC_REF_VOLTAGE;
  return rawVoltage * VOLTAGE_DIVIDER_FACTOR;
}

float smoothBatteryVoltage() {
  static float lastBatt = 3.8f;
  float newV = readBatteryVoltage();
  lastBatt = 0.9f * lastBatt + 0.1f * newV;
  return lastBatt;
}

int batteryPercentFromVoltage(float v) {
  float pct = (v - 0.3f) / (0.44f - 0.3f) * 100.0f;
  return constrain((int)round(pct), 0, 100);
}

void updateBatteryOnHMI() {
  float v = smoothBatteryVoltage();
  int pct = batteryPercentFromVoltage(v);
  setNumber("j8", pct);
}

// ========== SENSOR READING FUNCTIONS ==========
float readCellTemperature() {
  DateTime now = rtc.now();
  if (now.year() >= 2024 && now.year() <= 2099) {
    float rtcTemp = rtc.getTemperature();
    // add small noise, but no forced range
    rtcTemp += (random(-20, 20) / 10.0f);
    return rtcTemp;
  } else {
    // fallback – still a simulation, but without 60–80 constraint
    return 25.0f + (random(-50, 50) / 10.0f);
  }
}

float readStackVoltage() {
  float voltage = 38.0f + (random(-20, 20) / 10.0f);
  return constrain(voltage, 35.0f, 42.0f);
}

float readStackCurrent() {
  return appliedCurrent + (random(-10, 10) / 10.0f);
}

float readO2Purity() {
  float purity = 99.5f + (random(-5, 5) / 10.0f);
  return constrain(purity, 95.0f, 100.0f);
}

void readAllSensors() {
  cellTemperature = readCellTemperature();
  stackVoltage = readStackVoltage();
  stackCurrent = readStackCurrent();
  o2Purity = readO2Purity();
  
  h2ProductionRate = appliedCurrent * 0.00042f;
  o2ProductionRate = appliedCurrent * 0.00021f;
}

// ========== GSM FUNCTIONS (BACKUP CONNECTIVITY) ==========
void updateGsmSignal() {
  // DISABLED: AT command polling was interfering with WiFi/MQTT
  // Since GSM SIM is expired, skip polling to keep MQTT stable
  // GSM will still connect if WiFi fails (via connectGPRS fallback)
  
  // Mark GSM bars as unavailable on HMI
  for (int i=4;i<=7;++i){
    setNumber(String("j" + String(i)).c_str(), 0);
    sendToNextion("j%d.bco=65504", i); // Gray (inactive)
  }
  
  gsmOK = false;  // Don't claim GSM is available if SIM is expired
}

// ========== GSM AT COMMAND FUNCTIONS ==========

String sendATCommand(const char *cmd, unsigned long timeout = 1000) {
  simSerial.print(cmd);
  unsigned long start = millis();
  String response = "";
  
  while (millis() - start < timeout) {
    while (simSerial.available()) {
      response += (char)simSerial.read();
    }
  }
  return response;
}

bool initializeGSMModem() {
  if (gsmInitialized) return true;
  
  Serial.println("[GSM] Initializing SIM800L modem...");
  delay(500);
  
  // Test basic AT command
  String resp = sendATCommand(CMD_AT, 1000);
  if (resp.indexOf("OK") < 0) {
    Serial.println("[GSM] ❌ No response from modem");
    return false;
  }
  
  // Check SIM status
  resp = sendATCommand(CMD_AT_CPIN, 1000);
  if (resp.indexOf("READY") < 0) {
    Serial.println("[GSM] ❌ SIM not ready");
    return false;
  }
  
  Serial.println("[GSM] ✅ Modem initialized, SIM ready");
  gsmInitialized = true;
  return true;
}

bool connectGPRS() {
  if (!initializeGSMModem()) return false;
  
  Serial.println("[GSM] Connecting to GPRS (Safaricom)...");
  
  // Set APN
  String resp = sendATCommand(CMD_AT_CSTT, 2000);
  if (resp.indexOf("OK") < 0) {
    Serial.println("[GSM] ❌ Failed to set APN");
    return false;
  }
  
  // Activate GPRS
  resp = sendATCommand(CMD_AT_CIICR, 3000);
  if (resp.indexOf("OK") < 0) {
    Serial.println("[GSM] ❌ Failed to activate GPRS");
    return false;
  }
  
  // Get local IP
  resp = sendATCommand(CMD_AT_CIFSR, 2000);
  if (resp.indexOf(".") > 0) {
    Serial.print("[GSM] ✅ GPRS connected, IP: ");
    Serial.println(resp);
    gsmConnected = true;
    return true;
  }
  
  Serial.println("[GSM] ❌ Failed to get IP address");
  gsmConnected = false;
  return false;
}

void checkGSMStatus() {
  if (!wifiOK) {
    String resp = sendATCommand(CMD_AT_CSQ, 1000);
    int idx = resp.indexOf("+CSQ:");
    if (idx >= 0) {
      int comma = resp.indexOf(",", idx);
      if (comma > idx) {
        String rssiStr = resp.substring(idx + 5, comma);
        int rssi = rssiStr.toInt();
        if (rssi >= 0 && rssi <= 31) {
          gsmRssiRaw = rssi;
          Serial.print("[GSM] Signal quality: ");
          Serial.print(rssi);
          Serial.println("/31");
        }
      }
    }
  }
}

void ensureGSMConnected() {
  if (!gsmConnected && !wifiOK) {
    connectGPRS();
  }
}

void ensureConnectivity() {
  ensureWiFiConnected();
  
  if (!wifiOK && !gsmConnected) {
    ensureGSMConnected();
  }
  
  if (wifiOK) {
    usingGSM = false;
  } else if (gsmConnected) {
    usingGSM = true;
    Serial.println("🔄 Using GSM/GPRS for connectivity");
  }
}

// ========== GSM HTTP BACKUP (Send data via HTTP POST when WiFi fails) ==========
void sendDataViaGSMHTTP() {
  if (!gsmConnected || wifiOK) return;  // Only use if GSM connected AND WiFi down
  
  Serial.println("[GSM-HTTP] Sending data via GPRS backup...");
  
  // Build JSON payload (same as MQTT)
  StaticJsonDocument<1024> doc;
  float safeProduction = constrain(prodRateSetpoint, 0.0f, 100.0f);
  int safeSlider = constrain((int)round(prodRateSetpoint), 0, 100);
  
  doc["prodRateSet"] = safeProduction;
  doc["mode"] = systemMode == MODE_AUTO ? "AUTO" : 
                systemMode == MODE_MANUAL ? "MANUAL" : 
                systemMode == MODE_MPC ? "MPC" : 
                systemMode == MODE_ECONOMIC ? "ECONOMIC" : "NONE";
  doc["state"] = (int)systemState;
  doc["battery"] = smoothBatteryVoltage();
  doc["gsm"] = gsmRssiRaw;
  doc["slider"] = safeSlider;
  doc["temperature"] = cellTemperature;
  doc["voltage"] = stackVoltage;
  doc["current"] = stackCurrent;
  doc["appliedCurrent"] = appliedCurrent;
  doc["h2ProductionRate"] = h2ProductionRate;
  doc["o2ProductionRate"] = o2ProductionRate;
  doc["water_tank_l"] = water_tank;
  doc["efficiency"] = smoothEfficiency;
  
  DateTime now = rtc.now(); 
  char dateStr[64]; 
  snprintf(dateStr, sizeof(dateStr), "%02d-%02d-%04d %02d:%02d:%02d", 
           now.day(), now.month(), now.year(), now.hour(), now.minute(), now.second()); 
  doc["rtcDateTime"] = dateStr;
  doc["device_id"] = "arduino_pem_giga_001";
  
  char buffer[1024];
  serializeJson(doc, buffer);
  
  // Send via HTTP POST over GPRS using AT commands
  sendATCommand("AT+HTTPINIT\r\n", 1000);
  delay(100);
  
  String setparam = "AT+HTTPPARA=\"CID\",1\r\n";
  sendATCommand(setparam.c_str(), 1000);
  delay(100);
  
  // Set URL
  String seturl = "AT+HTTPPARA=\"URL\",\"" + String(httpBackupEndpoint) + "\"\r\n";
  sendATCommand(seturl.c_str(), 1000);
  delay(100);
  
  // Set content type to JSON
  sendATCommand("AT+HTTPPARA=\"CONTENT\",\"application/json\"\r\n", 1000);
  delay(100);
  
  // Send POST request with data
  String postCmd = "AT+HTTPDATA=" + String(strlen(buffer)) + ",10000\r\n";
  sendATCommand(postCmd.c_str(), 1000);
  delay(100);
  
  simSerial.print(buffer);
  delay(100);
  simSerial.write(0x1A);  // Ctrl+Z to end data input
  delay(500);
  
  String resp = sendATCommand("AT+HTTPACTION=1\r\n", 5000);
  Serial.print("[GSM-HTTP] Response: ");
  Serial.println(resp);
  
  if (resp.indexOf("200") > 0 || resp.indexOf("201") > 0) {
    Serial.println("[GSM-HTTP] ✅ Data sent successfully via GPRS!");
  } else {
    Serial.println("[GSM-HTTP] ⚠️ Failed to send via GPRS");
  }
  
  sendATCommand("AT+HTTPTERM\r\n", 1000);
}

// ========== WIFI CONNECTIVITY ==========
void ensureWiFiConnected() {
  // Only attempt reconnection if not currently connected
  if(WiFi.status() != WL_CONNECTED) {
    WiFi.begin(ssid, password); 
    unsigned long start = millis(); 
    
    // Try for up to 4 seconds to connect
    while(WiFi.status() != WL_CONNECTED && millis() - start < 4000) {
      delay(200);
    }
  } 
  
  // Update WiFi status flag
  wifiOK = (WiFi.status() == WL_CONNECTED);
  
  // If WiFi fails, GSM will be used as backup (monitored in updateSystemStateOnHMI)
  if (!wifiOK && gsmOK) {
    Serial.println("WiFi disconnected - GSM backup available");
  }
}

void updateWiFiVisual() {
  if(!wifiOK){
    for(int i=0;i<=4;i++) sendToNextion("vis p%d,0", i); 
    sendToNextion("vis p0,1"); 
    return;
  }
  
  long rssi = WiFi.RSSI();
  int level;
  if (rssi > -50) level = 4;
  else if (rssi > -60) level = 3;
  else if (rssi > -70) level = 2;
  else if (rssi > -80) level = 1;
  else level = 0;
  
  for(int i=0;i<=4;i++) {
    sendToNextion("vis p%d,%d", i, (i<=level)?1:0);
  }
}

// ========== EFFICIENCY GAUGE UPDATE ==========
void updateEfficiencyOnHMI() {
  float supplyV = smoothBatteryVoltage();
  float supplyFactor = constrain(supplyV / 3.9f, 0.8f, 1.1f);
  float tempFactor = constrain(1.0f - 0.006f * (cellTemperature - 25.0f), 0.6f, 1.1f);
  float loadEfficiency = 1.0f - 0.00025f * sq(appliedCurrent - 150.0f);
  float efficiencyVal = constrain(60.0f + 0.5f * prodRateSetpoint * tempFactor * supplyFactor * loadEfficiency, 0.0f, 100.0f);
  smoothEfficiency += (efficiencyVal - smoothEfficiency) * 0.08f;
  int effInt = (int)round(smoothEfficiency);
  effInt = constrain(effInt, 0, 100);
  
  // NEW: Only update if changed
  static int lastEffInt = -1;
  if (effInt != lastEffInt) {
    lastEffInt = effInt;
    const int mapLen = 71;
    int idx = (int)round((float)effInt * (mapLen - 1) / 100.0f);
    idx = constrain(idx, 0, mapLen - 1);
    char gaugeChar[3] = {gaugeMap[idx], 0, 0};
    if (effInt == 0) setText("t6", " ");
    else setText("t6", String(gaugeChar));
    setText("t9", String(effInt) + "%");
    Serial.print("📊 Efficiency: ");
    Serial.print(effInt);
    Serial.println("%");
  }
}

// ========== SYSTEM STATE DISPLAY ==========
void updateSystemStateOnHMI() {
  if (!nextionActive) return;
  
  ensureWiFiConnected();
  updateWiFiVisual();
  updateGsmSignal();
  float battV = smoothBatteryVoltage();
  bool powerOK = (battV>0.3f);
  
  String status;
  switch(systemState){
    case STATE_WAITING: status="WAITING"; break;
    case STATE_STARTING: status="STARTING"; break;
    case STATE_RUNNING: 
      if(wifiOK && gsmOK && powerOK) {
        status = "RUNNING:";
        switch(systemMode) {
          case MODE_MPC: status += "MPC"; break;
          case MODE_ECONOMIC: status += "ECO"; break;
          case MODE_AUTO: status += "AUTO"; break;
          case MODE_MANUAL: status += "MANUAL"; break;
          default: status += "GOOD"; break;
        }
      } else {
        status="RUNNING:BAD";
      }
      break;
    case STATE_STOPPING: status="STOPPING"; break;
    case STATE_STOPPED: status="STOPPED"; break;
  }
  
  setText("t7", status);
  
  // FIXED: Don't overwrite t9 here - efficiency function owns t9 exclusively!
  // String modeText = ...
  // setText("t9", modeText);  // REMOVED - was overwriting efficiency %
  
  String powerText = powerConfig.source + " G:" + String(int(powerConfig.gridRatio*100)) + "% P:" + String(int(powerConfig.pvRatio*100)) + "%";
  setText("t12", powerText);
  
  // Update gauge elements with sensor data
  // Convert production rates to percentages (0-100%) to match dashboard display
  float h2Percent = constrain(h2ProductionRate * 100.0f, 0.0f, 100.0f);
  float o2Percent = constrain(o2ProductionRate * 100.0f, 0.0f, 100.0f);
  
  setNumber("j3", (int)round(h2Percent));  // H2 gauge (percentage)
  setNumber("j2", (int)round(o2Percent));  // O2 gauge (percentage)
  
  // Temperature gauge
  setNumber("j1", (int)round(cellTemperature));
  
  // Water level gauge
  setNumber("j0", (int)round(water_tank));
  
  Serial.print("📺 HMI status: ");
  Serial.println(status);
}

// ========== MPC CONTROL ==========
float runIndustrialMPC() {
    if(systemMode != MODE_MPC && systemMode != MODE_ECONOMIC) {
        return appliedCurrent;
    }
    
    float productionReference = prodRateSetpoint / 100.0f * 0.05f;
    industrialMPC.setReference(productionReference);
    
    float currentState = h2ProductionRate;
    float optimalControl = industrialMPC.computeControl(currentState, appliedCurrent, powerConfig);
    
    Serial.print("🎯 Industrial MPC: ");
    Serial.print("State="); Serial.print(currentState, 4);
    Serial.print(", Ref="); Serial.print(productionReference, 4);
    Serial.print(", Control="); Serial.print(optimalControl, 1);
    Serial.println("A");
    
    return optimalControl;
}

// ========== SAFETY FUNCTIONS ==========
void enforceSafetyConstraints() {
  float safeCurrent = appliedCurrent;
  
  if (cellTemperature > 75.0f) {
    safeCurrent = min(safeCurrent, 150.0f);
    Serial.println("🛡️ Temperature derating: 150A max");
  }
  if (cellTemperature > MAX_TEMPERATURE - 2.0f) {
    safeCurrent = min(safeCurrent, 120.0f);
    Serial.println("🛡️ Critical temperature: 120A max");
  }
  if (stackVoltage > MAX_VOLTAGE - 2.0f) {
    safeCurrent = min(safeCurrent, 160.0f);
    Serial.println("🛡️ High voltage: 160A max");
  }
  if (o2Purity < MIN_O2_PURITY + 0.2f) {
    safeCurrent = min(safeCurrent, 140.0f);
    Serial.println("🛡️ Low purity: 140A max");
  }
  
  safeCurrent = constrain(safeCurrent, MIN_CURRENT, MAX_CURRENT);
  
  if (abs(safeCurrent - appliedCurrent) > 5.0f) {
    Serial.print("🛡️ Safety override: ");
    Serial.print(appliedCurrent);
    Serial.print("A -> ");
    Serial.print(safeCurrent);
    Serial.println("A");
    appliedCurrent = safeCurrent;
  }
}

void performSafetyCheck() {
  readAllSensors();
  enforceSafetyConstraints();
  
  float dt_sec = (float)SAFETY_CHECK_MS / 1000.0f;
  float water_used = appliedCurrent * liters_per_s_per_A * dt_sec * water_consumption_scale;
  water_tank = water_tank - water_used;
  if (water_tank < 0.0f) water_tank = 0.0f;

  setNumber("j0", (int)round(water_tank));  // Update water gauge

  if (water_tank <= water_low_threshold && !water_alert_published) {
      Serial.println("⚠️ WATER LOW - publishing alert and engaging protection");
      StaticJsonDocument<128> alert;
      alert["alert"] = "LOW_WATER";
      alert["water_l"] = water_tank;
      char abuf[128]; serializeJson(alert, abuf);
      mqttClient.publish("arduino/water/alert", abuf);
      water_alert_published = true;

      digitalWrite(BUZZER_PIN, HIGH);
      Serial.println("⚠️ LOW WATER ALERT - Stopping system");

      if (systemState == STATE_RUNNING) {
         systemState = STATE_STOPPING;
         stateStartTime = millis();
         Serial.println("[SAFETY] Auto stopping due to low water");
         updateSystemStateOnHMI();
      }
  }

  if (water_tank <= water_emergency_threshold) {
      Serial.println("[SAFETY] CRITICAL: emergency stop - out of water");
      systemState = STATE_STOPPING;
      stateStartTime = millis();
      appliedCurrent = 0.0f;
      digitalWrite(GRID_RELAY, LOW);
      digitalWrite(PV_RELAY, LOW);
      digitalWrite(BUZZER_PIN, HIGH);
  }

  if (cellTemperature > MAX_TEMPERATURE || o2Purity < MIN_O2_PURITY) {
    Serial.println("[SAFETY] CRITICAL: Emergency shutdown!");
    systemState = STATE_STOPPING;
    stateStartTime = millis();
    appliedCurrent = 0.0f;
    digitalWrite(GRID_RELAY, LOW);
    digitalWrite(PV_RELAY, LOW);
  }
}

// ========== CURRENT CONTROL ==========
void applyCurrent(float current) {
  current = constrain(current, MIN_CURRENT, MAX_CURRENT);
  appliedCurrent = current;
  Serial.print("🔧 Applying current: ");
  Serial.print(current);
  Serial.println("A");
  // Do NOT touch h0 – it belongs to production rate setpoint
}

// ========== NEXTION INPUT HANDLING ==========
int extractFirstInt(const String &s){
  String num="";
  for(unsigned int i=0;i<s.length();++i){
    char c=s.charAt(i);
    if(isDigit(c)||(c=='-'&&num.length()==0)) num+=c;
    else if(num.length()) break;
  }
  return num.length() ? num.toInt() : 0;
}

void handleNextionInput(){
  static String buf=""; 
  static uint8_t ffCount=0;
  
  while(nexSerial.available()){
    char c=nexSerial.read();
    if((uint8_t)c==0xFF){
      ffCount++; 
      if(ffCount>=3){
        String proc=buf; 
        buf=""; 
        ffCount=0; 
        proc.trim(); 
        
        if(proc.length()==0) continue;
        
        // FIXED: Handle slider input properly
        if (proc.startsWith("h0=") || proc.startsWith("h0.val=")) {
          int value = extractFirstInt(proc);
          value = constrain(value, 0, 100);
          
          if (value != prodRateSetpointInt) {
            prodRateSetpointInt = value;
            prodRateSetpoint = (float)value;
            sliderMoved = true;
            
            // Convert slider to current (0-100% -> 100-200A)
            manualCurrentSetpoint = map(value, 0, 100, 100, 200);
            
            Serial.print("🎚️ Slider: ");
            Serial.print(value);
            Serial.print("% -> ");
            Serial.print(manualCurrentSetpoint);
            Serial.println("A");
            
            if(systemMode == MODE_MANUAL) {
              applyCurrent(manualCurrentSetpoint);
            }
            
            // Update display immediately
            setNumber("h0", value);
            
            // Send MQTT update
            if(mqttConnected){
              StaticJsonDocument<128> j; 
              j["prodRate"]=value;
              j["manualCurrent"]=manualCurrentSetpoint;
              char b[128]; serializeJson(j,b); 
              mqttClient.publish("arduino/slider", b);
            }
          }
        }
        // Handle button presses
        else if (proc.startsWith("b0=")) {
          if(systemState==STATE_WAITING||systemState==STATE_STOPPED){
            systemState=STATE_STARTING; 
            stateStartTime=millis(); 
            updateSystemStateOnHMI(); 
            Serial.println("[HMI] START");
          }
        }
        else if (proc.startsWith("b1=")) {
          if(systemState==STATE_RUNNING||systemState==STATE_STARTING){
            systemState=STATE_STOPPING; 
            stateStartTime=millis(); 
            updateSystemStateOnHMI(); 
            Serial.println("[HMI] STOP");
          }
        }
        else if (proc.startsWith("b2=")) {
          systemMode=MODE_AUTO; 
          if(systemState==STATE_STARTING||systemState==STATE_WAITING||systemState==STATE_STOPPED) systemState=STATE_RUNNING; 
          updateSystemStateOnHMI();
        }
        else if (proc.startsWith("b3=")) {
          systemMode=MODE_MANUAL; 
          if(systemState==STATE_STARTING||systemState==STATE_WAITING||systemState==STATE_STOPPED) systemState=STATE_RUNNING; 
          updateSystemStateOnHMI();
        }
        else if (proc.startsWith("b4=")) {
          systemMode=MODE_MPC; 
          mpcControlActive=true; 
          if(systemState==STATE_STARTING||systemState==STATE_WAITING||systemState==STATE_STOPPED) systemState=STATE_RUNNING; 
          updateSystemStateOnHMI(); 
          Serial.println("[HMI] MPC MODE");
        }
        else if (proc.startsWith("b5=")) {
          systemMode=MODE_ECONOMIC; 
          mpcControlActive=true; 
          if(systemState==STATE_STARTING||systemState==STATE_WAITING||systemState==STATE_STOPPED) systemState=STATE_RUNNING; 
          updateSystemStateOnHMI(); 
          Serial.println("[HMI] ECONOMIC MPC MODE");
        }
      }
    } else {
      buf += c;
    }
  }
}

// ========== CONTROL LOGIC ==========
void handleMPCControl() {
  if((systemMode == MODE_MPC || systemMode == MODE_ECONOMIC) && mpcControlActive) {
    float mpcCurrent = runIndustrialMPC();
    applyCurrent(mpcCurrent);
    performSafetyCheck();
  } else if(systemMode == MODE_MANUAL) {
    applyCurrent(manualCurrentSetpoint);
    performSafetyCheck();
  } else if(systemMode == MODE_AUTO) {
    float autoCurrent = 150.0f;
    applyCurrent(autoCurrent);
    performSafetyCheck();
  }
}

// ========== SETUP ==========
void setup() {
  // CRITICAL FIX: Initialize Nextion FIRST, before any other serial
  // This ensures it works without USB
  nexSerial.begin(9600);
  delay(500);
  
  // Now initialize Serial for debugging (optional, not required)
  Serial.begin(115200);
  delay(100);
  
  // Initialize GSM at 115200 baud (SIM800L requirement)
  simSerial.begin(gsmBaud);
  
  Serial.println("\n🏭 INDUSTRIAL PEM ELECTROLYZER CONTROLLER");
  Serial.println("==========================================");
  
  // Initialize pins
  pinMode(GRID_RELAY, OUTPUT);
  pinMode(PV_RELAY, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(GRID_RELAY, LOW);
  digitalWrite(PV_RELAY, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  
  analogReadResolution(ADC_BITS);
  randomSeed(analogRead(A0) ^ micros());
  
  // Initialize Nextion display
  initNextionDisplay();
  
  // Initialize RTC (do this after Nextion)
  Serial.println("⏰ RTC Initialization...");
  if (!rtc.begin()) {
    Serial.println("[RTC] RTC not found!");
  } else {
    DateTime nowRTC = rtc.now();
    if (nowRTC.year() < 2024 || nowRTC.year() > 2099) {
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
      Serial.println("🕒 RTC adjusted from compile time");
    } else {
      Serial.print("✅ RTC running: ");
      Serial.print(nowRTC.day()); Serial.print("/");
      Serial.print(nowRTC.month()); Serial.print("/");
      Serial.print(nowRTC.year()); Serial.print(" ");
      Serial.print(nowRTC.hour()); Serial.print(":");
      Serial.print(nowRTC.minute()); Serial.print(":");
      Serial.println(nowRTC.second());
    }
  }
  
  // Network initialization
  Serial.println("🌐 Network Initialization...");
  ensureWiFiConnected();
  updateWiFiVisual();
  updateGsmSignal();
  updateBatteryOnHMI();
  
  // MQTT Setup
  Serial.println("📡 MQTT Setup...");
  setupMQTT();
  
  // Final initialization
  updateSystemStateOnHMI();
  
  lastFullUpdate = lastMqttUpdate = lastSafetyCheck = lastMPCUpdate = lastEfficiencyUpdate = lastNextionHeartbeat = lastNextionInit = lastSliderUpdate = lastStatusUpdate = millis();
  
  Serial.println("✅ INDUSTRIAL CONTROLLER READY");
}

// ========== MAIN LOOP ==========
void loop(){
  unsigned long now = millis();
  
  // Handle MQTT communication
  mqttClient.loop();
  
  // Process HMI inputs (do this frequently)
  handleNextionInput();
  
  // Update slider on HMI frequently
  if(now - lastSliderUpdate >= SLIDER_UPDATE_MS){
    lastSliderUpdate = now;
    updateSliderOnHMI();
  }
  
  // Send heartbeat to Nextion to keep it awake
  if(now - lastNextionHeartbeat >= NEXTION_HEARTBEAT_MS){
    lastNextionHeartbeat = now;
    nextionHeartbeat();
  }
  
  // MPC control updates (10 Hz)
  if(now - lastMPCUpdate >= MPC_UPDATE_MS){
    lastMPCUpdate = now;
    handleMPCControl();
  }
  
  // Safety checks (2 Hz)
  if(now - lastSafetyCheck >= SAFETY_CHECK_MS){
    lastSafetyCheck = now;
    performSafetyCheck();
  }
  
  // Update time and status display (0.2 Hz)
  if(now - lastFullUpdate >= FULL_UPDATE_MS){
    lastFullUpdate = now; 
    DateTime dt = rtc.now();
    
    char buf[32]; 
    snprintf(buf, sizeof(buf), "%02d", dt.hour()); 
    setText("t0", buf); 
    snprintf(buf, sizeof(buf), "%02d", dt.minute()); 
    setText("t1", buf);
    snprintf(buf, sizeof(buf), "%02d", dt.second()); 
    setText("t2", buf); 
    
    char dateStr[32]; 
    snprintf(dateStr, sizeof(dateStr), "%02d %s %04d", 
             dt.day(), 
             (const char*[]){"Jan","Feb","Mar","Apr","May","Jun","Jul","Aug","Sep","Oct","Nov","Dec"}[dt.month()-1], 
             dt.year()); 
    setText("t3", dateStr);
    
    char upd[16]; 
    snprintf(upd, sizeof(upd), "%02d:%02d:%02d", dt.hour(), dt.minute(), dt.second()); 
    setText("t8", upd);
    
    updateBatteryOnHMI(); 
    updateSystemStateOnHMI();
  }
  
  // Update efficiency gauge (1 Hz)
  if(now - lastEfficiencyUpdate >= EFFICIENCY_UPDATE_MS) {
    lastEfficiencyUpdate = now;
    updateEfficiencyOnHMI();
  }
  
  // State machine transitions
  if(systemState == STATE_STARTING && now - stateStartTime >= START_DELAY){
    if(systemMode == MODE_AUTO || systemMode == MODE_MANUAL || systemMode == MODE_MPC || systemMode == MODE_ECONOMIC){
      systemState = STATE_RUNNING; 
      updateSystemStateOnHMI(); 
      Serial.println("[SYSTEM] STARTING -> RUNNING");
    }
  }
  else if(systemState == STATE_STOPPING && now - stateStartTime >= STOP_DELAY){
    systemState = STATE_STOPPED; 
    systemMode = MODE_NONE; 
    mpcControlActive = false;
    appliedCurrent = 0.0f;
    digitalWrite(GRID_RELAY, LOW);
    digitalWrite(PV_RELAY, LOW);
    updateSystemStateOnHMI(); 
    Serial.println("[SYSTEM] STOPPING -> STOPPED");
  }
  
  // MQTT data transmission (1 Hz)
  if(now - lastMqttUpdate >= MQTT_UPDATE_MS){
    lastMqttUpdate = now; 
    sendMQTTData();
  }
  
  // Status heartbeat (0.33 Hz - every 3 seconds)
  if(now - lastStatusUpdate >= STATUS_UPDATE_MS) {
    lastStatusUpdate = now;
    sendStatusHeartbeat();
  }
  
  // GSM connectivity check (0.1 Hz - every 10 seconds)
  if(now - lastGsmCheckup >= GSM_CHECKUP_MS) {
    lastGsmCheckup = now;
    ensureConnectivity();
    checkGSMStatus();
  }
  
  delay(10); // Run faster to catch all Nextion inputs
}
