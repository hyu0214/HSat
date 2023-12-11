#include <Wire.h>
//A4, A5 pin designated for MPU6050 I2C connection
//pin config: Vcc-3.3V SCL-A5 SDA-A4
#include <SoftwareSerial.h>
#include <TinyMPU6050.h>

MPU6050 mpu (Wire);//attach MPU6050 library to Wire.h

#define MPU6050     0x68 // MPU6050 address
#define BT_RX       12 //HM-10 TX pin num
#define BT_TX       13 //HM-10 RX pin num
#define IN3         6 //Motor Rotation Direction
#define IN4         7 //Motor Rotation Direction
#define PWM_Pin     9 //Motor PWM pin
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
float cumulated_error;
int pwm;
const float Kp;//P controller Gain
const float Ki;//I controller Gain
const float alpha;//complementary filter gain
unsigned long T_interval;
unsigned long now;
const float error_ref;//integrator shut off reference value
int settling_counter;//counter for selective PI control system
bool sun_flag;//bool flag for whether system is oriented to the sun

SoftwareSerial BTSerial(BT_RX, BT_TX);//BTSerial: HM10 comm

void setup() {
  // put your setup code here, to run once:
  //set register
  //set pin mode
  
  //initiating Bluetooth Comm
  BTSerial.begin(38400);//Bluetooth HM10 Baudrate: 38400
  //initializing MPU6050
  mpu.Initialize();
  BTSerial.println("Sensor Calibration in Process: DO NOT MOVE");
  mpu.Calibrate ();//calibrating MPU6050 for sensor offset value
  BTSerial.println("Calibration COMPLETE: H_SAT Operational");

  //initializing values
  set_angle=0;
  accelSpeed = 0;
  cumulated_error = 0;
  rpm = 0;//default set RPM =0
  op_mode = 0;//default set op_mode: Stabilization
  T_interval = 0;
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
    else if(command == 25000){//mode1: moving to set_angle
      op_mode=1;
      BTSerial.print("Rotating to orientation:");
      BTSerial.println(set_angle);
    }
    else if(command == 30000){//mode2: solar tracking
      op_mode=2;
      BTSerial.println("Tracking Sun")
    }
    else if(abs(command)<180){
      op_mode = 1;
      set_angle = constrain(command,-180.0,180.0);//get set angle
      BTSerial.print("Rotating to orientation:");
      BTSerial.println(set_angle);
    }
  }

  if(op_mode == 0){//for stabilization mode
    stabilization();
  }
  else if(op_mode == 1){
    orientation();
  }
  else if(op_mode == 2){
    while(){
      SolarTrack();
    }
  }
}

void stabilization(){//stabilization mode function
  set_speed = 0;
  mpu.Execute();
  float speed = getGyroZ();
  Motor_control(PIcontrol(set_speed, speed));
}

void orientation(){//constant RPM mode function
  mpu.Execute();
  float angle = getAngZ();
  Motor_control(PIcontrol(set_angle, angle));
}

void SolarTrack(){//solar tracking mode function
  mpu.Execute();
}

void mpu.Execute(){//function to update speed and angle from MPU6050
  mpu.Execute();
}

int PIcontrol(float setpoint, float currentvalue){
  now = micros();
  float error = setpoint - currentvalue;
  float r_speed = abs(mpu.GetAngGyroZ())*3.14159/180;

  if(abs(error)<err_ref) counter ++;
  else counter = 0;

  if(counter>5){//start Integrator if entered steady-state
    cumulated_error += error;
  }
  else cumulated_error = 0;//reset integrator during transient response

  pwm = Kp * error + Ki * cumulated_error;//PIcontrol feedback value
  pwm = constrain(pwm, -255, 255);//constrained feedback value(-255,255)
  return pwm;
}

void Motor_control(int pwm) {
  if (pwm <= 0) {//set direction according to sign of 'pwm'
    digitalWrite(IN3, LOW); //CW
    digitalWrite(IN4, HIGH);
  } else {
    digitalWrite(IN3, HIGH); //CCW
    digitalWrite(IN4, LOW);
  }
  digitalWrite(PWM_pin,abs(pwm));//write absolute value of PWM into PWM pin
}
