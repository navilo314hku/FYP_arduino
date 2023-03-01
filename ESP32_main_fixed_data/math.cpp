#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Arduino.h>



void print_imu_data(sensors_event_t a,  sensors_event_t g,
float x_acc_bias,
float y_acc_bias,
float z_acc_bias,
float x_ang_bias,
float y_ang_bias,
float z_ang_bias){
  Serial.print(a.acceleration.x - x_acc_bias);
  Serial.print(',');
  Serial.print(a.acceleration.y-y_acc_bias);
  Serial.print(',');
  Serial.print(a.acceleration.z-z_acc_bias);
  Serial.print(',');
  Serial.print(g.gyro.x-x_ang_bias);
  Serial.print(',');
  Serial.print(g.gyro.y-y_ang_bias);
  Serial.print(',');
  Serial.print(g.gyro.z-z_ang_bias);
  Serial.println("");
}
