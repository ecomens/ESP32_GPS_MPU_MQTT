# ESP32_GPS_MPU_MQTT
o Hardware
 - ESP32 Woorm Devkit
 - NEO 6M GPS Sensor
 - MPU 6050 Acc/Gyro 6-axis Sensor
 - 3D Printed Test Case Box supported Jake(JH.Moon)

o Software
 - Arduino IDE : ESP32 upload firmware
 - Home Assistant + MQTT + Plotly Graph Card

o *** consider the Pin map set in "sensors_pins.h"

o ESP-HA(Home Assistant) MQTT Payroad Format.
  ![ESP32-HA-MQTT01](https://github.com/user-attachments/assets/ad0f2012-a86f-4eeb-bd34-b48444db5d4a)

o HA configuration.yaml.
  ![ESP32-GPS-MPU01-HAConfiguration01](https://github.com/user-attachments/assets/95a3a932-0081-4303-8f46-7cf1a059ea63)

o HA Display 6-Axis( ACC, Gyro) Card Sample.
  ![ESP32-GPS-MPU01-HACardYaml01](https://github.com/user-attachments/assets/b1d5d91b-0723-4f15-8b56-e59c597fbe4a)
  - card yaml sample.
      type: custom:plotly-graph
      entities:
        - entity: sensor.accx
        - entity: sensor.accy
        - entity: sensor.accz
      hours_to_show: 24
      refresh_interval: 10
      title: Accelator(가속도)
 o HA Display Board.
   ![ESP32-GPS-MPU01](https://github.com/user-attachments/assets/625872dd-e8fd-4d84-8021-16121297c2b4)
Created by: MOONDORI (moonbj90@gmail.com). Limited. 2024.10-23
