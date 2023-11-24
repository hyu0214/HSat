#include <Wire.h>
#include <SoftwareSerial.h>


#define MPU6050//MPU6050 address
#define Accel_config//MPU6050 accelerometer sensitivity setting address
#define Gyro_config//MPU6050 accelerometer sensitivity setting address
#define bt_RX//HM-10 TX pin num
#define bt_TX//HM-10 RX pin num
//include more pin numbers
const int16_t accelY_offset;
const int16_t gyroZ_offset;
float angle;
float accelSpeed;
float speed;
float set_speed;
float set_angle;
float PIcontrol;
const float Kp;
const float Ki;
const float alpha;//complementary filter gain
const float T_interval;
const float error_ref;//integrator shut off reference value

SoftwareSerial btSerial(bt_RX,bt_TX);

void get_angle(){//function to obtain angle from MPU6050
  //get accel_Y, get gyro_Z from MPU6050
  accel_Y += accelY_offset;
  gyro_Z += gyroZ_offset;
  accelSpeed+=accel_Y*T_interval;
  speed = (accelSpeed)*(alpha)+(gyro_Z)*(1-alpha);//calculating angularspeed using complementary filter with gain alpha
  angle += (speed) * (T_interval);//calculate angle by integrating angular speed
}

float PIcontrol(setpoint, currentvalue){
  float error = setpoint - currentvalue;
  if(error < err_ref){
    cummulated_error += error * T_interval;
  }
  else{
    cummulated_error = 0;
  }
  PIcontrol = Kp * error + Ki * cummulated_error;//PIcontrol feedback value
  PIcontrol = constrain(PIcontrol, -799, 799);//constrained feedback value(-799~799)
  return PIcontrol;
}

void Motor_control(int pwm) {
  if (pwm <= 0) {
    digitalWrite(DIRECTION, LOW);
    pwm = -pwm;
  } else {
    digitalWrite(DIRECTION, HIGH);
  }
  setPWM(map(pwm, 0, 255, PWMVALUE, 0));
}

void setup() {
  // put your setup code here, to run once:
  //set register
  //set pin mode

  //initializing values
  angle = 0;
  accelSpeed = 0;
  cummulated_error = 0;
  btSerial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  char command;
  if(btSerial.available()){
    //get operation mode and set speed
    command = BTSerial.read();
    op_mode = parseInt(c);
    
  }

}

