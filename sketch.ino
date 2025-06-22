#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <TinyGPS++.h>
#include <DHTesp.h>
#include <ArduinoJson.h>


const char* ssid = "Wokwi-GUEST";
const char* password = "";


const char* serverUrl = "http://192.168.1.100:5000/api/v1/telemetry/records";

const char* apiKey = "super-secret-key-123";

const int DHT_PIN = 15;
const int RX2_PIN = 16;
const int TX2_PIN = 17;

TinyGPSPlus gps;
DHTesp dhtSensor;
HTTPClient http;

float lastKnownTemperature = 0.0;
float lastKnownHumidity = 0.0;

unsigned long previousDhtReadTime = 0;
const long dhtReadInterval = 2000;

unsigned long previousSendTime = 0;
const long sendInterval = 10000;

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected!");
  Serial.print("ESP32 IP Address: ");
  Serial.println(WiFi.localIP());
}

void sendDataToServer() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("Error: No WiFi connection. Cannot send data.");
        return;
    }
    
 
    StaticJsonDocument<512> jsonDoc;

    jsonDoc["device_id"] = "ESP32_Tracker_01";
    
    JsonObject sensorData = jsonDoc.createNestedObject("sensor_data");
    sensorData["temperature"] = lastKnownTemperature;
    sensorData["humidity"] = lastKnownHumidity;

    JsonObject gpsData = jsonDoc.createNestedObject("gps_data");
    gpsData["latitude"] = gps.location.isValid() ? gps.location.lat() : 0.0;
    gpsData["longitude"] = gps.location.isValid() ? gps.location.lng() : 0.0;
    gpsData["altitude_meters"] = gps.altitude.isValid() ? gps.altitude.meters() : 0.0;
    gpsData["speed_kmph"] = gps.speed.isValid() ? gps.speed.kmph() : 0.0;

    if (gps.date.isValid() && gps.time.isValid()) {
        char timestamp[25];
        sprintf(timestamp, "%04d-%02d-%02dT%02d:%02d:%02dZ", 
                gps.date.year(), gps.date.month(), gps.date.day(), 
                gps.time.hour(), gps.time.minute(), gps.time.second());
        gpsData["timestamp_utc"] = timestamp;
    } else {
        gpsData["timestamp_utc"] = (const char*)nullptr;
    }

    String jsonString;
    serializeJson(jsonDoc, jsonString);

    http.begin(serverUrl);
    http.addHeader("Content-Type", "application/json");
    http.addHeader("X-API-Key", apiKey);

    Serial.println("\n>>> Sending data to Edge Service...");
    Serial.println(jsonString);

    int httpResponseCode = http.POST(jsonString);

    if (httpResponseCode > 0) {
        Serial.print("<<< HTTP Response Code: ");
        Serial.println(httpResponseCode);
        String payload = http.getString();
        Serial.print("<<< Server Response: ");
        Serial.println(payload);
    } else {
        Serial.print("<<< HTTP Send Error. Code: ");
        Serial.println(httpResponseCode);
        Serial.println("    (This could be due to a network error, incorrect server IP, or server not running)");
    }

    http.end();
}


void setup() {
  Serial.begin(9600);
  Serial2.begin(9600, SERIAL_8N1, RX2_PIN, TX2_PIN);
  Serial.println("Initializing Monitoring System for UPC IoT Bicas Team Edge Service...");
  
  setup_wifi();
  
  dhtSensor.setup(DHT_PIN, DHTesp::DHT22);
  Serial.println("DHT22 sensor initialized.");
}


void loop() {
  unsigned long currentTime = millis();

  if (currentTime - previousDhtReadTime >= dhtReadInterval) {
    previousDhtReadTime = currentTime;
    TempAndHumidity data = dhtSensor.getTempAndHumidity();
    if (!isnan(data.temperature) && !isnan(data.humidity)) {
      lastKnownTemperature = data.temperature;
      lastKnownHumidity = data.humidity;
    }
  }

  while (Serial2.available() > 0) {
    if (gps.encode(Serial2.read())) {
      
      Serial.println("----------- Local Update Received -----------");
      Serial.print("Temp/Hum: "); Serial.print(lastKnownTemperature, 1); Serial.print("C, "); Serial.print(lastKnownHumidity, 1); Serial.println("%");
      Serial.print("Location: "); Serial.print(gps.location.lat(), 6); Serial.print(", "); Serial.println(gps.location.lng(), 6);
      
      if (currentTime - previousSendTime >= sendInterval) {
        previousSendTime = currentTime;
        sendDataToServer();
      }
    }
  }

  if (millis() > 5000 && gps.charsProcessed() < 10) {
    Serial.println(F("WARNING: No GPS data received. Check wiring or wait for satellite fix."));
    while(true);
  }
}