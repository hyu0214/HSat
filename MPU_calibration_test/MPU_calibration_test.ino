#include <Arduino.h>
#include <TinyMPU6050.h>
#include <SoftwareSerial.h>
#include <Wire.h>

SoftwareSerial BTSerial(8,9);
MPU6050 mpu (Wire);
unsigned long timer;
unsigned long timer2;
unsigned long count;
float radius;

void setup() {
  // put your setup code here, to run once:
  BTSerial.begin(38400);
  mpu.Initialize();
  BTSerial.println("calibration in process: DO NOT MOVE");
  mpu.Calibrate();
  BTSerial.println("calibration complete");
  //mpu.SetGyroOffsets (-114.05, -3.91, 101.12);//measured offset value of gyroscope
  timer = 0;
  timer2 = 0;
  count = 0;
  radius = 3.0;
}

void loop() {
  // put your main code here, to run repeatedly:
  count++;
  unsigned long now = millis();
  mpu.Execute();//update MPU6050
  float accX=abs(mpu.GetAccX());//centrepetal_force
  float r_speed = abs(mpu.GetRawGyroZ()-101.12)*3.14159/180*131;
  float angle = mpu.GetAngZ();
  radius =(radius*(count-1) + accX*100/(r_speed*r_speed))/count;//radius in cm
  timer2 = 0;
  if(now-timer>100){
    timer = 0;
    BTSerial.println(angle);
  }
}