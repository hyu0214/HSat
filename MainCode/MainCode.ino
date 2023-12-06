#include <Wire.h>
//A4, A5 pin designated for MPU6050 I2C connection
//pin config: Vcc-3.3V SCL-A5 SDA-A4
#include <SoftwareSerial.h>
#include <TinyMPU6050.h>

MPU6050 mpu (Wire);//attach MPU6050 library to Wire.h

#define MPU6050     0x68// MPU6050 address
#define GyZ         0x47// MPU6050 Gyro Z address
#define BT_RX       8//HM-10 TX pin num
#define BT_TX       9//HM-10 RX pin num
#define DIRECTION1//Motor Rotation Direction
#define DIRECTION2//Motor Rotation Direction
#define PWM_Pin//Motor PWM pin
#define CDS1
#define CDS2
//include more pin numbers
const int16_t accelY_offset;
const int16_t gyroZ_offset;
unsigned int op_mode;
float angle;
float accelSpeed;
float speed;
float set_speed;
float set_angle;
int pwm;
const float Kp;//P controller Gain
const float Ki;//I controller Gain
const float alpha;//complementary filter gain
unsigned long T_interval;
unsigned long now;
const float error_ref;//integrator shut off reference value

SoftwareSerial BTSerial(BT_RX, BT_TX);//BTSerial: HM10 comm

void get_angle(){//function to obtain angle from MPU6050
  mpu.Execute();

}
int PIcontrol(float setpoint, float currentvalue){
  float error = setpoint - currentvalue;
  float r_speed = abs(mpu.GetAngGyroZ())*3.14159/180;
  if(error < err_ref){
    cummulated_error += error * T_interval;
  }
  else{
    cummulated_error = 0;
  }
  pwm = Kp * error + Ki * cummulated_error;//PIcontrol feedback value
  pwm = constrain(pwm, -255, 255);//constrained feedback value(-255,255)
  return pwm;
}

void Motor_control(int pwm) {
  if (pwm <= 0) {
    digitalWrite(DIRECTION, LOW);
    pwm = -pwm;
  } else {
    digitalWrite(DIRECTION, HIGH);
  }
  //write absolute value of PWM into PWM pin
}

void setup() {
  // put your setup code here, to run once:
  //set register
  //set pin mode

  //initializing values
  angle = 0;
  accelSpeed = 0;
  cummulated_error = 0;
  rpm = 0;//default set RPM =0
  op_mode = 0;//defualt set op_mode: Stabilization
  BTSerial.begin(38400);//Bluetooth HM10 Baudrate: 38400
}

void loop() {
  // put your main code here, to run repeatedly:
  int command;
  if(BTSerial.available()){//get operation mode and set rpm from HM10
    command = BTSerial.parseInt();
    if(command == 20000){//mode0: stabilization
      op_mode=0;
      BTSerial.println("Stabilize");
    }
    else if(command == 25000){//mode1: constant RPM rotation
      op_mode=1;
      set_speed = 30;//default speed 60rpm
      BTSerial.println("Constant RPM:30");
    }
    else if(command == 30000){//mode2: solar tracking
      op_mode=2;
      BTSerial.println("Tracking Sun")
    }
    else{
      op_mode = 1;
      rpm = constrain(command,-120,120);//limit RPM to maximum 120 RPM
      BTSerial.print("Constant RPM:");
      BTSerial.print(rpm);
    }
  }

  if(op_mode == 0){//for stabilization mode
    Stabilization();
  }
  else if(op_mode == 1){
    constantRPM(rpm);
  }
  else if(op_mode == 2){
    while(){
    
    }
  }
}

void Stabilization(){//stabilization mode function
  set_speed = 0;
  get_angle();
  
  Motor_control(PIcontrol(set_speed, angle));
}

void constantRPM(int rpm){//constant RPM mode function
  set_speed = (rpm)*6;//convert RPM into deg/sec
  get_angle();
  Motor_control(PIcontrol(set_speed, angle));
}

void SolarTrack(){//solar tracking mode function
  get_angle();
}