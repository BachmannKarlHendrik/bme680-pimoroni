#include <Arduino.h>
#include "bsec.h"
#include <Wire.h>
#include <WiFi.h>
extern "C" {
	#include "freertos/FreeRTOS.h"
	#include "freertos/timers.h"
}
#include <AsyncMqttClient.h>
#include <credentials.h>


//Wifi info and debugging led
int LED_BUILTIN = 2;

String clientId = "KarliESP-PimoroniBme680-1";
String command = "";
String TOPIC = "s/us";
unsigned long lastMsg = 0;

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

Bsec iaqSensor;

String output;
int sda = 21;
int scl = 23;

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFINAME, WIFIPASS);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
  command = "100,"+clientId+",c8y_MQTTdevice";
  mqttClient.publish(TOPIC.c_str(), 0, false, command.c_str());
}

void WiFiEvent(WiFiEvent_t event) {
    Serial.printf("[WiFi-event] event: %d\n", event);
    switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
        Serial.println("WiFi connected");
        Serial.println("IP address: ");
        Serial.println(WiFi.localIP());
        connectToMqtt();
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        Serial.println("WiFi lost connection");
        xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
        xTimerStart(wifiReconnectTimer, 0);
        break;
    default:
      Serial.println("Unhandled wifi event just happened!");
    }
}

void onMqttConnect(bool sessionPresent) {
  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("Disconnected from MQTT.");

  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}

void onMqttUnsubscribe(uint16_t packetId) {
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  Serial.println("Publish received.");
  Serial.print("  topic: ");
  Serial.println(topic);
  Serial.print("  qos: ");
  Serial.println(properties.qos);
  Serial.print("  dup: ");
  Serial.println(properties.dup);
  Serial.print("  retain: ");
  Serial.println(properties.retain);
  Serial.print("  len: ");
  Serial.println(len);
  Serial.print("  index: ");
  Serial.println(index);
  Serial.print("  total: ");
  Serial.println(total);
}

void onMqttPublish(uint16_t packetId) {
  Serial.println("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

// Helper function definitions
void checkIaqSensorStatus(void)
{
  if (iaqSensor.status != BSEC_OK) {
    if (iaqSensor.status < BSEC_OK) {
      output = "BSEC error code : " + String(iaqSensor.status);
      Serial.println(output);
    } else {
      output = "BSEC warning code : " + String(iaqSensor.status);
      Serial.println(output);
    }
  }

  if (iaqSensor.bme680Status != BME680_OK) {
    if (iaqSensor.bme680Status < BME680_OK) {
      output = "BME680 error code : " + String(iaqSensor.bme680Status);
      Serial.println(output);
    } else {
      output = "BME680 warning code : " + String(iaqSensor.bme680Status);
      Serial.println(output);
    }
  }

  while(true) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(2500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(2500);
  }
}

void setup() {
  //Start serial and LED
  Serial.begin(115200);
  while(!Serial);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.println();
  Serial.println();

  //Sensor setup
  Wire.begin(sda,scl);

  iaqSensor.begin(BME680_I2C_ADDR_PRIMARY, Wire);
  output = "\nBSEC library version " + String(iaqSensor.version.major) + "." + String(iaqSensor.version.minor) + "." + String(iaqSensor.version.major_bugfix) + "." + String(iaqSensor.version.minor_bugfix);
  Serial.println(output);
  checkIaqSensorStatus();

  bsec_virtual_sensor_t sensorList[10] = {
    BSEC_OUTPUT_RAW_TEMPERATURE,
    BSEC_OUTPUT_RAW_PRESSURE,
    BSEC_OUTPUT_RAW_HUMIDITY,
    BSEC_OUTPUT_RAW_GAS,
    BSEC_OUTPUT_IAQ,
    BSEC_OUTPUT_STATIC_IAQ,
    BSEC_OUTPUT_CO2_EQUIVALENT,
    BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
  };

  iaqSensor.updateSubscription(sensorList, 10, BSEC_SAMPLE_RATE_LP);
  checkIaqSensorStatus();

  //Create MQTT timers
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  //Create WIFI state listener
  WiFi.onEvent(WiFiEvent);

  //Create MQTT state methods
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(URL, 1883);
  mqttClient.setCredentials(TENANT,IOTPASS);
  mqttClient.setClientId(clientId.c_str());

  //Start connection
  connectToWifi();
}

void loop() {
  unsigned long now = millis();
  if (now - lastMsg > 5000) { // Wait 5 seconds between every request
    if (iaqSensor.run()) { // If new data is available
      lastMsg = now;
      output = String(iaqSensor.pressure);
      output += ", " + String(iaqSensor.iaq);
      output += ", " + String(iaqSensor.temperature);
      output += ", " + String(iaqSensor.humidity);
      output += ", " + String(iaqSensor.co2Equivalent);
      output += ", " + String(iaqSensor.breathVocEquivalent);
      Serial.println(output);
      command = "200,pressureMeasurement,hecto pascals,"+String(iaqSensor.pressure)+",hPa";
      mqttClient.publish(TOPIC.c_str(), 0, false, command.c_str());
      command = "200,iaqMeasurement,indoor air quality,"+String(iaqSensor.iaq)+",iaq";
      mqttClient.publish(TOPIC.c_str(), 0, false, command.c_str());
      command = "211,"+String(iaqSensor.temperature);
      mqttClient.publish(TOPIC.c_str(), 0, false, command.c_str());
      command = "200,humidityMeasurement,percent,"+String(iaqSensor.humidity)+",%";
      mqttClient.publish(TOPIC.c_str(), 0, false, command.c_str());
      command = "200,eco2Measurement,particles per million,"+String(iaqSensor.co2Equivalent)+",ppm";
      mqttClient.publish(TOPIC.c_str(), 0, false, command.c_str());
      command = "200,ebvoc,particles per million,"+String(iaqSensor.breathVocEquivalent)+",ppm";
      mqttClient.publish(TOPIC.c_str(), 0, false, command.c_str());
    }
    else {
      checkIaqSensorStatus();
    }
  }
}