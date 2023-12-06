#include <Arduino.h>
#include <TinyMPU6050.h>
#include <SoftwareSerial.h>
#include <Wire.h>

SoftwareSerial BTSerial(8,9);
MPU6050 mpu (Wire);
unsigned long timer;

void setup() {
  // put your setup code here, to run once:
  BTSerial.begin(38400);
  mpu.Initialize();
  mpu.SetGyroOffsets (-114.05, -3.91, 101.12);//measured offset value of gyroscope
  timer = 0;
}

void loop() {
  // put your main code here, to run repeatedly:
  timer = millis();
  mpu.Execute();//update MPU6050
  float speed = mpu.GetGyroZ ();
  if(timer>100){
    BTSerial.println(speed);
  }
}