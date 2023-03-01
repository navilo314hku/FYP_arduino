// Basic demo for accelerometer readings from Adafruit MPU6050
#define START_BUTTON_PIN 19 // GIOP21 pin connected to button
#define SAVE_BUTTON_PIN 18
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "func.h" 
//button variables
int lastState = HIGH; // the previous state from the input pin
int currentState;     // the current reading from the input pin
int saveButtonLastState = HIGH;
int saveButtonCurrentState;
//button variables end
//main prog variable
int main_loop_count=0;
//sampling var start
SAMPLE_SIZE=22;
//sampling var end
//callibration variables start
int start_button_count = 0;
int CALIBRATION_SAMPLE_SIZE = 30;
int CALIBRATION_FREQENCY = 30;
int calibrating = 0;
int calibrate_count = 0;
float x_acc_bias = 0;
float y_acc_bias = 0;
float z_acc_bias = 0;
float x_ang_bias = 0;
float y_ang_bias = 0;
float z_ang_bias = 0;

//callibration variables end

Adafruit_MPU6050 mpu;

void setup(void) {
  calibrating = 0;
  Serial.begin(9600);
  pinMode(START_BUTTON_PIN, INPUT_PULLUP);
  pinMode(SAVE_BUTTON_PIN, INPUT_PULLUP);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  //Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  //Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  //Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
    case MPU6050_RANGE_2_G:
      Serial.println("+-2G");
      break;
    case MPU6050_RANGE_4_G:
      Serial.println("+-4G");
      break;
    case MPU6050_RANGE_8_G:
      //Serial.println("+-8G");
      break;
    case MPU6050_RANGE_16_G:
      Serial.println("+-16G");
      break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
    case MPU6050_RANGE_250_DEG:
      Serial.println("+- 250 deg/s");
      break;
    case MPU6050_RANGE_500_DEG:
      Serial.println("+- 500 deg/s");
      break;
    case MPU6050_RANGE_1000_DEG:
      Serial.println("+- 1000 deg/s");
      break;
    case MPU6050_RANGE_2000_DEG:
      Serial.println("+- 2000 deg/s");
      break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
    case MPU6050_BAND_260_HZ:
      Serial.println("260 Hz");
      break;
    case MPU6050_BAND_184_HZ:
      Serial.println("184 Hz");
      break;
    case MPU6050_BAND_94_HZ:
      Serial.println("94 Hz");
      break;
    case MPU6050_BAND_44_HZ:
      Serial.println("44 Hz");
      break;
    case MPU6050_BAND_21_HZ:
      Serial.println("21 Hz");
      break;
    case MPU6050_BAND_10_HZ:
      Serial.println("10 Hz");
      break;
    case MPU6050_BAND_5_HZ:
      Serial.println("5 Hz");
      break;
  }

  Serial.println("");
  delay(100);
}
void autoCalibrate(int start_button_count, int main_loop_count,int CALIBRATION_FREQENCY){
  //if not writing 
  if (start_button_count%2==0 & main_loop_count%CALIBRATION_FREQENCY==0 ){//not writing 
    calibrating=1;//Start calibration 
  }
}
void record_bias(float acc_x,float acc_y,float acc_z,float ang_x,float ang_y,float ang_z) {
  //Serial.println("Calibrating the IMU, please keep it stationary!!!");
  //add x_gyro into x_acc bias
  x_acc_bias += acc_x;
  y_acc_bias +=acc_y;
  z_acc_bias +=acc_z;
  x_ang_bias +=ang_x;
  y_ang_bias +=ang_y;
  z_ang_bias +=ang_z;
  
}
void averaging_bias(){
  x_acc_bias /= CALIBRATION_SAMPLE_SIZE;
  y_acc_bias/=CALIBRATION_SAMPLE_SIZE;
  z_acc_bias/=CALIBRATION_SAMPLE_SIZE;
  x_ang_bias/=CALIBRATION_SAMPLE_SIZE;
  y_ang_bias/=CALIBRATION_SAMPLE_SIZE;
  z_ang_bias/=CALIBRATION_SAMPLE_SIZE;
  }
void printGyroAndCalibratate() { //print gyro_data and callibration process
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */
  //Serial.print("Acceleration X: ");

  if (calibrating) {
    calibrate_count++;
    if (calibrate_count == 1) {
      //reset bias
      Serial.println("### CALIBRATION START!!! ### ");
      x_acc_bias = 0;
      y_acc_bias = 0;
      z_acc_bias = 0;
      x_ang_bias = 0;
      y_ang_bias = 0;
      z_ang_bias = 0;
    }
    record_bias(a.acceleration.x,a.acceleration.y,a.acceleration.z,g.gyro.x,g.gyro.y,g.gyro.z);
    
    if (calibrate_count == CALIBRATION_SAMPLE_SIZE) {

      averaging_bias(); 
      calibrating = 0; //quit calibrating condition
      calibrate_count = 0;
      
      //calculate sum
    }
  }
  else {
    print_imu_data(a,g,x_acc_bias,y_acc_bias,z_acc_bias,x_ang_bias,y_ang_bias,z_ang_bias);
  }
}
void printButtons() {
  // read the state of the switch/button:
  currentState = digitalRead(START_BUTTON_PIN);
  if (lastState == LOW && currentState == HIGH) { //detect start button
    Serial.println("start");
    start_button_count += 1;
    if (start_button_count % 2 == 0) {
      calibrating = 1;
    }
  }
  // save the last state
  lastState = currentState;
#
  saveButtonCurrentState = digitalRead(SAVE_BUTTON_PIN);
  if (saveButtonLastState == LOW && saveButtonCurrentState == HIGH) { //detect save button
    Serial.println("stop");
    //change callibrating to True
    calibrating = 1;
  }
  saveButtonLastState = saveButtonCurrentState;

}

void loop() {
  main_loop_count+=1;
  if (main_loop_count>60000){//int overflow prevention
    main_loop_count=0;
  }
  autoCalibrate(start_button_count,main_loop_count,CALIBRATION_FREQENCY);
  printGyroAndCalibratate();
  printButtons();
  //Serial.println(calibrating);
  //delay(50);

}
/*
  TODO: Test the second click of start button can start the calibration process



*/
