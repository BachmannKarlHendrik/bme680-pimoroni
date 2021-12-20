#include "bsec.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include "credentials.h"

//Wifi info and debugging led
int LED_BUILTIN = 2;

const char* ssid     = WIFINAME;
const char* password = WIFIPASS;
const char* url = URL; //Endpoint server
const char* tenant = TENANT;
const char* passwordiot = IOTPASS;
String clientId = "KarliESP-PimoroniBme680-1";
String command = "";


WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// Helper functions declarations
void checkIaqSensorStatus(void);
void reconnect();
void alarmSend(String,bool);

// Create an object of the class Bsec
Bsec iaqSensor;

String output;
int sda = 21;
int scl = 23;

// Entry point for the example
void setup(void)
{
  Serial.begin(115200);
  while(!Serial);
  //=========================================
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
  }
  Serial.println("WiFi connected");
  //=========================================
  //MQTT
  mqttClient.setServer(url, 1883);
  reconnect();
  //=========================================
  digitalWrite(LED_BUILTIN, LOW);

  //=========================================
  Wire.begin(scl,sda);

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

  // Print the header
  output = "Timestamp [ms], raw temperature [°C], pressure [hPa], raw relative humidity [%], gas [Ohm], IAQ, IAQ accuracy, temperature [°C], relative humidity [%], Static IAQ, CO2 equivalent, breath VOC equivalent";
  Serial.println(output);
}

void reconnect() {
  // Loop until we're reconnected
  while (!mqttClient.connected()) {
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqttClient.connect(clientId.c_str(),tenant,passwordiot)) {
      Serial.println("Connected");
      String command = "100,"+clientId+",c8y_MQTTdevice";
      mqttClient.publish("s/us", (char*) command.c_str());
      digitalWrite(LED_BUILTIN, LOW);
      // Connected - do something useful - subscribe to topics, publish messages, etc.
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      // Wait 5 seconds before retrying
      Serial.println("Disconnecting wifi");
      WiFi.disconnect();
      Serial.println("Reconnecting wifi in 5 seconds");
      Serial.print("Connecting to ");
      Serial.println(ssid);
      delay(5000);
      WiFi.reconnect();
      int wifiCounter = 0;
      while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
        wifiCounter +=1;
        if(wifiCounter > 30) {
          Serial.println("Wifi reconnect timed out. Restarting ESP.");
          ESP.restart();
        }
      }
    }
  }
}

// Function that is looped forever
void loop(void) {
  if (!mqttClient.connected()) {
    reconnect();
  }

  mqttClient.loop();
  
  unsigned long time_trigger = millis();
  if (iaqSensor.run()) { // If new data is available
    output = String(time_trigger);
    output += ", " + String(iaqSensor.pressure);
    output += ", " + String(iaqSensor.iaq);
    output += ", " + String(iaqSensor.temperature);
    output += ", " + String(iaqSensor.humidity);
    output += ", " + String(iaqSensor.co2Equivalent);
    output += ", " + String(iaqSensor.breathVocEquivalent);
    Serial.println(output);
    command = "200,pressureMeasurement,hecto pascals,"+String(iaqSensor.pressure)+",hPa";
    mqttClient.publish("s/us", (char*) command.c_str());
    command = "200,iaqMeasurement,indoor air quality,"+String(iaqSensor.iaq)+",iaq";
    mqttClient.publish("s/us", (char*) command.c_str());
    command = "211,"+String(iaqSensor.temperature);
    mqttClient.publish("s/us", (char*) command.c_str());
    command = "200,humidityMeasurement,percent,"+String(iaqSensor.humidity)+",%";
    mqttClient.publish("s/us", (char*) command.c_str());
    command = "200,eco2Measurement,particles per million,"+String(iaqSensor.co2Equivalent)+",ppm";
    mqttClient.publish("s/us", (char*) command.c_str());
    command = "200,ebvoc,particles per million,"+String(iaqSensor.breathVocEquivalent)+",ppm";
    mqttClient.publish("s/us", (char*) command.c_str());
  } else {
    checkIaqSensorStatus();
  }
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
      alarmSend(output,true);
    } else {
      output = "BME680 warning code : " + String(iaqSensor.bme680Status);
      alarmSend(output,false);
    }
  }
}

void alarmSend(String message, bool finish) {
  Serial.println(message);
  if(finish) {
    command = "301,c8y_CriticalAlarm,"+message;
    mqttClient.publish("s/us", (char*) command.c_str());
  }
  else {
    command = "304,c8y_WarningAlarm,"+message;
    mqttClient.publish("s/us", (char*) command.c_str());
  }
  while(finish) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(2500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(2500);
  }
}