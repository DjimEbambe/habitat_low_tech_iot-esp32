#include <WiFi.h>
#include <ArduinoJson.h>
#include <EEPROM.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <Adafruit_SHT31.h>
#include <Adafruit_VEML7700.h>
#include "ACS712.h"
#include <ESPmDNS.h>
#include <time.h>

// Définition des broches et des composants
#define VEML7700_ADDR 0x10
#define SHT31_ADDR 0x44
#define CURRENT_SENSOR_PIN 34
#define VOLTAGE_SENSOR_PIN 33
#define FAN_PWM_PIN 26
#define SSR_RELAY_PIN1 27
#define SSR_RELAY_PIN2 14
#define SSR_RELAY_PIN3 12

#define EEPROM_SIZE 512

#define PWM_FREQ 5000
#define PWM_RESOLUTION 8
#define PWM_CHANNEL_FAN 0

// Structure de configuration essentielle
struct Config {
    float Kp;
    float Ki;
    float Kd;
    float setpointTemp;
    float setpointLux;
    char ap_ssid[32];
    char ap_password[32];
    bool autoMode;
} config;

Config defaultConfig = {2.0, 5.0, 1.0, 25.0, 300.0, "ESP32_AP", "12345678", true};

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// Objets capteurs
Adafruit_SHT31 sht31 = Adafruit_SHT31();
Adafruit_VEML7700 veml7700 = Adafruit_VEML7700();
ACS712 acs712(CURRENT_SENSOR_PIN, 3.3, 4095, 33); 

// Variables de contrôle et de suivi de l'énergie
float integralTemp = 0, previous_errorTemp = 0;
float integralLux = 0, previous_errorLux = 0;
int fanSpeedManual = 100;
bool lamp1State = false, lamp2State = false, lamp3State = false; 
time_t lastUpdateTime = 0;
float totalEnergy = 0;
float peakPower = 0;
time_t peakPowerTime = 0;
time_t startTime;
time_t currentTime;
float batteryLevel = 75.0;
float voltage = 0.0;

void setup() {
    Serial.begin(115200);
    EEPROM.begin(EEPROM_SIZE);
    loadConfig();
    
    initSensors();

    // Configuration des relais et PWM
    pinMode(SSR_RELAY_PIN1, OUTPUT);
    pinMode(SSR_RELAY_PIN2, OUTPUT);
    pinMode(SSR_RELAY_PIN3, OUTPUT);
    digitalWrite(SSR_RELAY_PIN1, LOW);
    digitalWrite(SSR_RELAY_PIN2, LOW);
    digitalWrite(SSR_RELAY_PIN3, LOW);
    
    ledcSetup(PWM_CHANNEL_FAN, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(FAN_PWM_PIN, PWM_CHANNEL_FAN);
    
    setupAP();
    if (!MDNS.begin("low-tech")) {
        Serial.println("Erreur lors de la configuration de mDNS !");
    } else {
        Serial.println("mDNS démarré");
    }

    ws.onEvent(onWsEvent);
    server.addHandler(&ws);
    server.begin();

    startTime = time(nullptr);

    xTaskCreate(readSensorsTask, "Read Sensors", 2048, NULL, 1, NULL);
    xTaskCreate(controlSystemTask, "Control System", 2048, NULL, 1, NULL);
    xTaskCreate(energyManagementTask, "Energy Management", 2048, NULL, 1, NULL);
}

void loop() {
    ws.cleanupClients();
}

// Fonction pour initialiser les capteurs
void initSensors() {
    if (!sht31.begin(SHT31_ADDR)) {
        Serial.println("Erreur de communication avec le capteur de température SHT31 !");
    }
    if (!veml7700.begin()) {
        Serial.println("Erreur de communication avec le capteur de luminosité VEML7700 !");
    }
    acs712.autoMidPoint();
}

void loadConfig() {
    if (EEPROM.read(0) == 1) {
        EEPROM.get(1, config);
        Serial.println("Configuration chargée depuis l'EEPROM.");
    } else {
        config = defaultConfig;
        Serial.println("Configuration par défaut utilisée.");
        saveConfig();
    }
}

void saveConfig() {
    EEPROM.write(0, 1);
    EEPROM.put(1, config);
    EEPROM.commit();
    Serial.println("Configuration sauvegardée dans l'EEPROM.");
}

void setupAP() {
    Serial.println("Configuration du point d'accès...");
    WiFi.softAP(config.ap_ssid, config.ap_password);
    Serial.print("SSID AP: ");
    Serial.println(config.ap_ssid);
    Serial.print("Adresse IP AP: ");
    Serial.println(WiFi.softAPIP());
}

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_CONNECT) {
        Serial.printf("Client WebSocket connecté depuis %s\n", client->remoteIP().toString().c_str());
        sendCurrentConfig(client);
    } else if (type == WS_EVT_DISCONNECT) {
        Serial.printf("Client WebSocket déconnecté\n");
    } else if (type == WS_EVT_DATA) {
        data[len] = '\0';
        Serial.printf("Reçu: %s\n", data);

        StaticJsonDocument<256> doc;
        DeserializationError error = deserializeJson(doc, (const char*)data);

        if (!error) {
            handleConfigUpdate(doc);
        } else {
            Serial.println("Erreur de parsing JSON.");
        }
    }
}

void sendCurrentConfig(AsyncWebSocketClient *client) {
    StaticJsonDocument<256> doc;
    doc["settings"]["setpointTemp"] = config.setpointTemp;
    doc["settings"]["setpointLux"] = config.setpointLux;
    doc["settings"]["Kp"] = config.Kp;
    doc["settings"]["Ki"] = config.Ki;
    doc["settings"]["Kd"] = config.Kd;
    doc["settings"]["ap_ssid"] = config.ap_ssid;
    doc["settings"]["ap_password"] = config.ap_password;
    doc["settings"]["autoMode"] = config.autoMode;
    doc["manual"]["fanSpeed"] = fanSpeedManual; 
    doc["manual"]["lamp1State"] = lamp1State;
    doc["manual"]["lamp2State"] = lamp2State;
    doc["manual"]["lamp3State"] = lamp3State;
    doc["energy"]["totalEnergy"] = totalEnergy;
    doc["energy"]["peakPower"] = peakPower;
    doc["energy"]["peakPowerTime"] = peakPowerTime;
    doc["energy"]["currentTime"] = currentTime;
    doc["energy"]["uptime"] = time(nullptr) - startTime;
    doc["battery"]["level"] = batteryLevel;

    char buffer[256];
    serializeJson(doc, buffer);
    client->text(buffer);
}

void handleConfigUpdate(StaticJsonDocument<256>& doc) {
    if (doc.containsKey("setpointTemp")) {
        config.setpointTemp = doc["setpointTemp"].as<float>();
    }
    if (doc.containsKey("setpointLux")) {
        config.setpointLux = doc["setpointLux"].as<float>();
    }
    if (doc.containsKey("Kp")) config.Kp = doc["Kp"].as<float>();
    if (doc.containsKey("Ki")) config.Ki = doc["Ki"].as<float>();
    if (doc.containsKey("Kd")) config.Kd = doc["Kd"].as<float>();

    if (doc.containsKey("ap_ssid")) strncpy(config.ap_ssid, doc["ap_ssid"], sizeof(config.ap_ssid));
    if (doc.containsKey("ap_password")) strncpy(config.ap_password, doc["ap_password"], sizeof(config.ap_password));

    if (doc.containsKey("autoMode")) {
        config.autoMode = doc["autoMode"].as<bool>();
    }

    if (doc.containsKey("fanSpeed")) {
        fanSpeedManual = doc["fanSpeed"].as<int>();
        ledcWrite(PWM_CHANNEL_FAN, map(fanSpeedManual, 1, 100, 0, 255));
    }

    if (doc.containsKey("lamp1State")) {
        lamp1State = doc["lamp1State"].as<bool>();
        digitalWrite(SSR_RELAY_PIN1, lamp1State ? HIGH : LOW);
    }
    if (doc.containsKey("lamp2State")) {
        lamp2State = doc["lamp2State"].as<bool>();
        digitalWrite(SSR_RELAY_PIN2, lamp2State ? HIGH : LOW);
    }
    if (doc.containsKey("lamp3State")) {
        lamp3State = doc["lamp3State"].as<bool>();
        digitalWrite(SSR_RELAY_PIN3, lamp3State ? HIGH : LOW);
    }

    if (doc.containsKey("currentTime")) {
        currentTime = doc["currentTime"].as<time_t>();
    }

    sendCurrentConfigToAllClients();

    if (doc.containsKey("ap_ssid") || doc.containsKey("ap_password")) {
        ESP.restart();
    }
}

void sendCurrentConfigToAllClients() {
    StaticJsonDocument<256> doc;
    doc["settings"]["setpointTemp"] = config.setpointTemp;
    doc["settings"]["setpointLux"] = config.setpointLux;
    doc["settings"]["Kp"] = config.Kp;
    doc["settings"]["Ki"] = config.Ki;
    doc["settings"]["Kd"] = config.Kd;
    doc["settings"]["ap_ssid"] = config.ap_ssid;
    doc["settings"]["ap_password"] = config.ap_password;
    doc["settings"]["autoMode"] = config.autoMode;
    doc["manual"]["fanSpeed"] = fanSpeedManual; 
    doc["manual"]["lamp1State"] = lamp1State;
    doc["manual"]["lamp2State"] = lamp2State;
    doc["manual"]["lamp3State"] = lamp3State;
    doc["energy"]["totalEnergy"] = totalEnergy;
    doc["energy"]["peakPower"] = peakPower;
    doc["energy"]["peakPowerTime"] = peakPowerTime;
    doc["energy"]["currentTime"] = currentTime;
    doc["energy"]["uptime"] = time(nullptr) - startTime;
    doc["battery"]["level"] = batteryLevel;

    char buffer[256];
    serializeJson(doc, buffer);
    ws.textAll(buffer);
}

void readSensorsTask(void* parameter) {
    while (1) {
        float temp = sht31.readTemperature();
        float hum = sht31.readHumidity();
        float lux = veml7700.readLux();
        float current = acs712.mA_DC();
        voltage = analogRead(VOLTAGE_SENSOR_PIN) * (3.3 / 4095.0) * 2;
        float power = voltage * current;
        totalEnergy += power * (millis() - lastUpdateTime) / 3600000.0;
        lastUpdateTime = millis();

        batteryLevel = calculateBatteryLevel(voltage);

        if (power > peakPower) {
            peakPower = power;
            peakPowerTime = currentTime;
        }

        StaticJsonDocument<128> doc;
        doc["temperature"] = temp;
        doc["humidity"] = hum;
        doc["lux"] = lux;
        doc["current"] = current;
        doc["voltage"] = voltage;
        doc["power"] = power;
        doc["battery"] = batteryLevel;

        char buffer[128];
        serializeJson(doc, buffer);
        ws.textAll(buffer);

        Serial.println("Données capteurs envoyées.");
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}

void controlSystemTask(void* parameter) {
    while (1) {
        PIDControlTemperature();
        PIDControlLight();
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void energyManagementTask(void* parameter) {
    while (1) {
        Serial.printf("Énergie totale consommée : %.2f Wh\n", totalEnergy);
        Serial.printf("Pic de consommation : %.2f W à %ld\n", peakPower, peakPowerTime);
        Serial.printf("Temps de fonctionnement : %ld secondes\n", time(nullptr) - startTime);
        Serial.printf("Niveau de batterie : %.2f %%\n", batteryLevel);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

// Contrôle PID pour la température
void PIDControlTemperature() {
    float temp = sht31.readTemperature();
    float errorTemp = config.setpointTemp - temp;
    integralTemp += errorTemp;
    float derivativeTemp = errorTemp - previous_errorTemp;
    float outputTemp = config.Kp * errorTemp + config.Ki * integralTemp + config.Kd * derivativeTemp;

    int fanOutput = map(constrain(outputTemp, 0, 100), 0, 100, 0, 255);
    ledcWrite(PWM_CHANNEL_FAN, fanOutput);
    Serial.printf("Vitesse du ventilateur ajustée à %d %%\n", fanOutput);

    previous_errorTemp = errorTemp;
}

// Contrôle PID pour la luminosité
void PIDControlLight() {
    float lux = veml7700.readLux();
    float errorLux = config.setpointLux - lux;
    integralLux += errorLux;
    float derivativeLux = errorLux - previous_errorLux;
    float outputLux = config.Kp * errorLux + config.Ki * integralLux + config.Kd * derivativeLux;

    // Contrôler les lampes selon la sortie PID
    if (outputLux > 0) {
        lamp1State = lamp2State = lamp3State = true;
    } else {
        lamp1State = lamp2State = lamp3State = false;
    }

    digitalWrite(SSR_RELAY_PIN1, lamp1State ? HIGH : LOW);
    digitalWrite(SSR_RELAY_PIN2, lamp2State ? HIGH : LOW);
    digitalWrite(SSR_RELAY_PIN3, lamp3State ? HIGH : LOW);

    Serial.printf("État des lampes : %s\n", lamp1State ? "ON" : "OFF");

    previous_errorLux = errorLux;
}

float calculateBatteryLevel(float voltage) {
    float minVoltage = 21.0;
    float maxVoltage = 25.6;
    float batteryLevel = ((voltage - minVoltage) / (maxVoltage - minVoltage)) * 100.0;
    return constrain(batteryLevel, 0, 100);
}
