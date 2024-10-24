/////////////////////////////////////////////////////////////////
/*
  Home Assistant with multiple ESP32 Wroom Devkit and DHT11 Sensor
  Home Assistant configuration.yml
  ************************************************************
  mqtt:
  camera:
    - topic: esp32/cam_0
  sensor:
    - name: "esp32_Temp"
      state_topic: "esp32/dht_0"
      unit_of_measurement: "°C"
      value_template: "{{ value_json.temperature }}"
      

    - name: "esp32_Hum"
      state_topic: "esp32/dht_0"
      unit_of_measurement: "%"
      value_template: "{{ value_json.humidity }}"
   *************************************************************   
  Created by MOONDORI
*/
/////////////////////////////////////////////////////////////////

/*
- ESP32 Arduino 
- TestDevice: ESP32-CAM AI-Thinker, XIAO ESP32S3 Sense

**Required Library**
- MQTT: 2.5.1
https://github.com/256dpi/arduino-mqtt
*/

#include <WiFiClientSecure.h>
#include <MQTT.h>

#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include "DHT.h"


#define SENSOR_DHT11      // DHT11 Temperature and Humidity Sensor
#define SENSOR_GPS_NEO6M   // NEO6M GPS Sensor
#define SENSOR_MPU_6050    // MPU 6050 Accelorator and Gyro Sensor 

#include "sensors_pins.h"
// ******************Gps NEO-6M****************************

//static const int GPSRXPIN = 16, GPSTXPIN = 17;
//static const uint32_t GPSBaud = 9600;

TinyGPSPlus gps; // The TinyGPS++ object
SoftwareSerial gps_s(GPSRXPIN, GPSTXPIN); // The serial connection to the GPS device

//**********DHT Sensor(Temperature, Humidity*****************
//#define DHTPIN 15     // Digital pin connected to the DHT sensor
// Feather HUZZAH ESP8266 note: use pins 3, 4, 5, 12, 13 or 14 --
// Pin 15 can work but DHT must be disconnected during program upload.

// DHT(DHT11, DHT22, DHT21, Temperature/Humidity Sensor) setup 
//#define DHTTYPE DHT11   // DHT 11
//#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

// Connect pin 1 (on the left) of the sensor to +5V
// NOTE: If using a board with 3.3V logic like an Arduino Due connect pin 1
// to 3.3V instead of 5V!
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor

// Initialize DHT sensor.
// Note that older versions of this library took an optional third parameter to
// tweak the timings for faster processors.  This parameter is no longer needed
// as the current DHT reading algorithm adjusts itself to work on faster procs.
DHT dht(DHTPIN, DHTTYPE);

//**********MPU-6050 (Gyro Sensor)*******************
#include <Wire.h>
#include <MPU6050_tockn.h>
MPU6050 mpu(Wire);


const char ssid[] = "moon";
const char pass[] = "41295076";

#define CONFIG_BROKER_URL "192.168.0.202"

#define CONFIG_BROKER_USERNAME "mqtt_user"
#define CONFIG_BROKER_PASSWORD "ecomen0987"

//For the First Camera
#define ESP32_DEVICE "ESP32-Wroom-0"
#define ESP32_PUBLISH_TOPIC_SENSOR1 "esp32/gps0"
#define ESP32_PUBLISH_TOPIC_SENSOR2 "esp32/mpu0"
#define ESP32_PUBLISH_TOPIC_SENSOR3 "esp32/dht0"


const int bufferSize = 1024 * 25;  // 25KB
const int sensor_delay = 0.1; // sensing delay 1000: 1second

WiFiClient net;
MQTTClient client = MQTTClient(bufferSize);
char data[80];  //DHT MQTT MSG

void connect() {
  Serial.print("checking wifi...");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(sensor_delay);
  }

  Serial.print("\nconnecting...");
  while (!client.connect(ESP32_DEVICE, CONFIG_BROKER_USERNAME, CONFIG_BROKER_PASSWORD)) {
    Serial.print(".");
    delay(sensor_delay);
  }

  Serial.println("\nconnected!");
}

void setup() {
  Serial.begin(115200);

  // DHT 센서 시작
  dht.begin();
  
  // I2C 통신을 위해 ESP32-CAM 핀 12, 13을 SCL, SDA로 설정
  Wire.begin(SDA, SCL); // SDA = 21, SCL = 22 or ESP32-CAM: SDA = 15, SCL = 14
 // MPU6050 초기화
  mpu.begin();
  mpu.calcGyroOffsets(true);  // 자이로 오프셋 자동 계산
  Serial.println("MPU6050 초기화 성공!");
  
  gps_s.begin(GPSBAUD);

  WiFi.begin(ssid, pass);
  client.begin(CONFIG_BROKER_URL, 1883, net);
  connect();

}

void loop() {
  reconnect();
  client.loop();
//  delay(sensor_delay);

  if (!client.connected()) {
    connect();
  } else {

    getDHTdata();
    updateMPUdata();

  }

  while (gps_s.available() > 0)
  if (gps.encode(gps_s.read()))
  displayInfo();

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
   while(true);
  }

}

// GPS displayInfo
void displayInfo() {

  double latitude = (gps.location.lat());
  double longitude = (gps.location.lng());
  String sensorsReadings = "{\"slatitude\":\"" + String(latitude) + "\", \"slongitude\":\"" + String(longitude) + "\", \"flatitude\":\"" + latitude + "\", \"flongitude\":\"" + longitude + "\" }";
    sensorsReadings.toCharArray(data, (sensorsReadings.length() + 1));
    Serial.println("********** Publish MQTT data to Ecomen HA");
    Serial.print("Publish message: ");
    Serial.println(data);
      client.publish(ESP32_PUBLISH_TOPIC_SENSOR1, data);
    Serial.println("> MQTT data published");
    Serial.println("********** End ");
    Serial.println("*****************************************************");
    delay(sensor_delay);// delay 
/*
  if (gps.location.isValid()) {
     String sensorsReadings = "{\"latitude\":\"" + String(latitude) + "\", \"longitude\":\"" + String(longitude) + "\"}";
     sensorsReadings.toCharArray(mqtt_payload, (dhtReadings.length() + 1));

      Serial.println("********** Publish MQTT data to Ecomen HA");
      Serial.print("Publish message: ");
      Serial.println(mqtt_payload);
      client.publish(pubTopic, mqtt_payload);
      Serial.println("> MQTT data published");
      Serial.println("********** End ");
      Serial.println("*****************************************************");

      delay(writeInterval);// delay 
  } else {
      Serial.println(F("INVALID"));
      Serial.println("********** Publish MQTT data to Ecomen HA");
      snprintf (mqtt_payload, 50, "m1=searching....!");
      Serial.print("Publish message: ");
      Serial.println(mqtt_payload);
      client.publish(pubTopic, mqtt_payload);
      Serial.println("> MQTT data published");
      Serial.println("********** End ");
      Serial.println("*****************************************************");

  }
*/


}


void getDHTdata() {
  delay(sensor_delay * 100);
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f = dht.readTemperature(true);

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println(F("Failed to read from DHT sensor!"));
   return;
  }

  // Compute heat index in Fahrenheit (the default)
  float hif = dht.computeHeatIndex(f, h);
  // Compute heat index in Celsius (isFahreheit = false)
  float hic = dht.computeHeatIndex(t, h, false);

  Serial.print(F("Humidity: "));
  Serial.print(h);
  Serial.print(F("%  Temperature: "));
  Serial.print(t);
  Serial.print(F("°C "));
  Serial.print(f);
  Serial.print(F("°F  Heat index: "));
  Serial.print(hic);
  Serial.print(F("°C "));
  Serial.print(hif);
  Serial.println(F("°F"));


    String dhtReadings = "{\"temperature\":\"" + String(t) + "\", \"humidity\":\"" + String(h) + "\"}";
    dhtReadings.toCharArray(data, (dhtReadings.length() + 1));
  if (!client.publish(ESP32_PUBLISH_TOPIC_SENSOR3, data)) {
    Serial.println("[Failure] Uploading DHT Data via MQTT");
  }

}

void updateMPUdata() {
  // 센서 값 업데이트
  mpu.update();

  // 가속도계 데이터 출력
  Serial.print("가속도계 X: "); Serial.print(mpu.getAccX()); Serial.print(" m/s² ");
  Serial.print("Y: "); Serial.print(mpu.getAccY()); Serial.print(" m/s² ");
  Serial.print("Z: "); Serial.print(mpu.getAccZ()); Serial.println(" m/s² ");

  // 자이로스코프 데이터 출력
  Serial.print("자이로스코프 X: "); Serial.print(mpu.getGyroX()); Serial.print(" °/s ");
  Serial.print("Y: "); Serial.print(mpu.getGyroY()); Serial.print(" °/s ");
  Serial.print("Z: "); Serial.print(mpu.getGyroZ()); Serial.println(" °/s ");
  

    String mpuReadings = "{\"accx\":\"" + String(mpu.getAccX()) + "\", \"accy\":\"" + String(mpu.getAccY()) + "\", \"accz\":\"" + String(mpu.getAccZ()) + "\", \"gyrox\":\"" + String(mpu.getGyroX()) + "\", \"gyroy\":\"" + String(mpu.getGyroY()) + "\", \"gyroz\":\"" + String(mpu.getGyroZ()) + "\"}";
    mpuReadings.toCharArray(data, (mpuReadings.length() + 1));
  if (!client.publish(ESP32_PUBLISH_TOPIC_SENSOR2, data)) {
    Serial.println("[Failure] Uploading MPU Data via MQTT");
  }

  // 500ms 대기
  delay(1000);
}




//MQTT reconnect
void reconnect() {
// Loop until we're reconnected
while (!client.connected()) {
Serial.print("********** Attempting MQTT connection...");
// Attempt to connect
if (client.connect(ESP32_DEVICE, CONFIG_BROKER_USERNAME, CONFIG_BROKER_PASSWORD)) { 
  Serial.println("-> MQTT client connected");
} else {
  Serial.print("failed, rc=");
//Serial.print(client.state());
  Serial.println("-> try again in 5 seconds");
// Wait 5 seconds before retrying
  delay(5000);
}
}
}
