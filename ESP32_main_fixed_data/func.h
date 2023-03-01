#ifndef FUNC_H 
#define FUNC_H
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Arduino.h>


void print_imu_data(sensors_event_t a,  sensors_event_t g, float x_acc_bias,float y_acc_bias,float z_acc_bias,float x_ang_bias,float y_ang_bias,float z_ang_bias);
#endif
