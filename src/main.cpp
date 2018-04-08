/*
   MQTT Sensor - Temperature, Humidity and Pressure (BME280) for Home-Assistant - NodeMCU (ESP8266)
   https://home-assistant.io/components/sensor.mqtt/

   Libraries :
    - ESP8266 core for Arduino : https://github.com/esp8266/Arduino
    - PubSubClient : https://github.com/knolleary/pubsubclient
    - BME280 : https://github.com/adafruit/Adafruit_BME280_Library
    - ArduinoJson : https://github.com/bblanchon/ArduinoJson

   Sources :
    - File > Examples > ES8266WiFi > WiFiClient
    - File > Examples > PubSubClient > mqtt_auth
    - File > Examples > PubSubClient > mqtt_esp8266
    - File > Examples > BME280 sensor library > bme280test
    - File > Examples > ArduinoJson > JsonGeneratorExample

   Configuration (HA) :
    sensor 1:
      platform: mqtt
      state_topic: 'office/bme280'
      name: 'Temperature'
      unit_of_measurement: '°C'
      value_template: '{{ value_json.temperature }}'

    sensor 2:
      platform: mqtt
      state_topic: 'office/bme280'
      name: 'Humidity'
      unit_of_measurement: '%'
      value_template: '{{ value_json.humidity }}'

    sensor 3:
      platform: mqtt
      state_topic: 'office/bme280'
      name: 'Pressure'
      unit_of_measurement: 'mBar'
      value_template: '{{ value_json.pressure }}'

   Samuel M. - v1.1 - 08.2016
   If you like this example, please add a star! Thank you!
   https://github.com/mertenats/open-home-automation
*/

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085.h>
#include <ArduinoJson.h>

#define MQTT_VERSION MQTT_VERSION_3_1_1

// Wifi: SSID and password
const char* WIFI_SSID = "HomeAssistantMQTT";
const char* WIFI_PASSWORD = "junkilin";

// MQTT: ID, server IP, port, username and password
const PROGMEM char* MQTT_CLIENT_ID = "terraza";
const PROGMEM char* MQTT_SERVER_IP = "192.168.0.104";
const PROGMEM uint16_t MQTT_SERVER_PORT = 1883;
const PROGMEM char* MQTT_USER = "homeassistant";
const PROGMEM char* MQTT_PASSWORD = "raspberry";

// MQTT: topic
const PROGMEM char* MQTT_SENSOR_TOPIC = "terraza/sensor";

// DHT 22
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
#define DHTPIN D1       // what digital pin we're connected to

// sleeping time
const PROGMEM uint16_t SLEEPING_TIME_IN_SECONDS = 600; // 10 minutes x 60 seconds

WiFiClient wifiClient;
PubSubClient client(wifiClient);
DHT dht(DHTPIN, DHTTYPE);
Adafruit_BMP085 bmp;

// function called to publish the temperature and the humidity
void publishData(float p_temperature, float p_humidity, float p_pressure) {
  // create a JSON object
  // doc : https://github.com/bblanchon/ArduinoJson/wiki/API%20Reference
  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  // INFO: the data must be converted into a string; a problem occurs when using floats...
  root["temperature"] = (String)p_temperature;
  root["humidity"] = (String)p_humidity;
  root["pressure"] = (String)p_pressure;
  root.prettyPrintTo(Serial);
  Serial.println("");
  /*
     {
        "temperature": "23.20",
        "humidity": "43.70",
		    "pressure": "934.81"
     }
  */
  char data[200];
  root.printTo(data, root.measureLength() + 1);
  client.publish(MQTT_SENSOR_TOPIC, data, true);
}

// function called when a MQTT message arrived
void callback(char* p_topic, byte* p_payload, unsigned int p_length) {
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("INFO: Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD)) {
      Serial.println("INFO: connected");
    } else {
      Serial.print("ERROR: failed, rc=");
      Serial.print(client.state());
      Serial.println("DEBUG: try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup() {
  // init the serial
  Serial.begin(115200);

  dht.begin();
  Wire.begin(D2, D3); // sda, scl
  bmp.begin();

  // init the WiFi connection
  Serial.println();
  Serial.println();
  Serial.print("INFO: Connecting to ");
  WiFi.mode(WIFI_STA);
  Serial.println(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  // Fixed IP
  IPAddress IP(192,168,0,204);
  IPAddress GATEWAY(192,168,0,1);
  IPAddress SUBNET(255,255,255,0);
  WiFi.config(IP, GATEWAY, SUBNET);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("INFO: WiFi connected");
  Serial.println("INFO: IP address: ");
  Serial.println(WiFi.localIP());

  // init the MQTT connection
  client.setServer(MQTT_SERVER_IP, MQTT_SERVER_PORT);
  client.setCallback(callback);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Reading temperature, humidity or pressure takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature()-2;
  // Read humidity as %
  float h = dht.readHumidity();
  // Read pressure en mBar
  float p = bmp.readPressure()/100;

  if (isnan(h) || isnan(t)) {
    Serial.println("ERROR: Failed to read from DHT sensor!");
    return;
  } else {
    //Serial.println(t);
    //Serial.println(h);
	  //Serial.println(p);
    publishData(t, h, p);
  }

  delay(600*1000);

  // Serial.println("INFO: Closing the MQTT connection");
  // client.disconnect();
  //
  // Serial.println("INFO: Closing the Wifi connection");
  // WiFi.disconnect();
  //
  // ESP.deepSleep(SLEEPING_TIME_IN_SECONDS * 1000000, WAKE_RF_DEFAULT);
  // delay(500); // wait for deep sleep to happen
}
