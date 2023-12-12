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
float speed;
float set_speed;
float set_angle;
float cumulated_error;
const float err_ref = 5;//reference value for deciding steady-state
int pwm;
const float Kp;//P controller Gain
const float Ki;//I controller Gain
const float alpha;//complementary filter gain
const float error_ref;//integrator shut off reference value
int settling_counter;//counter for selective PI control system
const int analogPins[] = {0, 1};                                       //아날로그 핀들을 정의
const int numPins = sizeof(analogPins) / sizeof(analogPins[0]);        //아날로그 핀들의 개수 정의
bool orientation_flag//bool flag for whether system is oriented to set_angle

SoftwareSerial BTSerial(BT_RX, BT_TX);//BTSerial: HM10 comm

void setup() {
  // put your setup code here, to run once:
  //set register
  //set pin mode

  Motor_control(0);//initially stop motor on activation
  //initiating Bluetooth Comm
  BTSerial.begin(38400);//Bluetooth HM10 Baudrate: 38400
  //initializing MPU6050
  mpu.Initialize();
  //calibrating MPU6050 for sensor offset value
  BTSerial.println("Sensor Calibration in Process: DO NOT MOVE");
  mpu.Calibrate ();
  BTSerial.println("Calibration COMPLETE: H-SAT Operational");
  
  //initializing variables
  set_angle=0;
  cumulated_error = 0;
  rpm = 0;//default set RPM =0
  op_mode = 0;//default set op_mode: Stabilization
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
      orientation_flag = false;
      BTSerial.print("Rotating to orientation:");
      BTSerial.println(set_angle);
    }
    else if(command == 30000){//mode2: solar tracking
      op_mode=2;
      BTSerial.println("Tracking Sun");
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
      SolarTrack();
  }
}

void stabilization(){//stabilization mode function
  set_speed = 0;
  Update_MPU();
  Motor_control(PIcontrol(set_speed, speed));
}

void orientation(){//constant RPM mode function
  //should update to set orientation_flag if orientation is complete
  Update_MPU();
  Motor_control(PIcontrol(set_angle, angle));
}

void SolarTrack(){//solar tracking mode function
  init_CDS_ADC();
  Update_MPU();
  int max_illuminance = 0;//maximum illuminance
  int illuminance;
  float origin_angle = angle;
  float sun_orientation;//angle of maximum illuminance(=angle of sun)
  bool orientation_flag = false;//bool flag for whether system is oriented to the sun
  set_speed = 10;
  while(abs(angle-origin_angle)<0.5){//rotate 360 degree and find maximum illuminance angle
    Update_MPU();
    Motor_control(PIcontrol(set_speed, speed));
    illuminance = measure_CDS();
    if(illuminance>max_illuminance){
      max_illuminance = illuminance;
      sun_orientation = angle;
    }
  }
  set_angle=sun_orientation;
  orientation_flag = false;
  while(!(orientation_flag)){
    orientation();
  }
  BTSerial.println("Orientated to Sun");
}

int PIcontrol(float setpoint, float currentvalue){
  now = micros();
  float error = setpoint - currentvalue;
  //float r_speed = abs(mpu.GetAngGyroZ())*3.14159/180;

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

void Update_MPU(){//fetch speed & angle from MPU6050
  mpu.Execute();
  speed = getGyroZ();//speed in deg/sec
  angle = getAngZ();//angle in deg
}

void Motor_control(int pwm) {
  if (pwm <= 0) {//set direction according to sign of 'pwm'
    digitalWrite(IN3, LOW); //CW rotation
    digitalWrite(IN4, HIGH);
  } else {
    digitalWrite(IN3, HIGH); //CCW rotation
    digitalWrite(IN4, LOW);
  }
  digitalWrite(PWM_pin,abs(pwm));//write absolute value of PWM into PWM pin
}

void init_CDS_ADC(){                                     //각기 다른 조도센서와 연결된 아날로그 핀들의 공통설정
  ADMUX |= (0<<REFS1) | (1<<REFS0);                      //참조 전압 5V로 설정
  ADMUX |= (0<<ADLAR);                                   //왼쪽 정렬
  ADCSRA |= (1<<ADEN);                                   //ADC 시작
  ADCSRA |= (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);        //Prescaler=128
}

int measure_CDS(){//function to measure CDS_value
  //measure average of two bottom mounted CDS sensor
  //in the future, should come up with an algorithm to preclude when top mounted CDS sensor measures over certain value
  //(=interference detected)
    for (int i =0; i < numPins; i++){
    ADMUX = (ADMUX & 0xF0) | (analogPins[i] & 0x0F);
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    uint16_t value = ADC;

    uint16_t Pin0, Pin1;
    if (i == 0) {              //i=0, i=1일때의 각각 다른 값을 부여
      Pin0 = value;
    }else if (i == 1){
      Pin1 = value;
      } 
    if (Pin0 != 0 && Pin1 != 0){
    return (Pin0 + Pin1) / 2;
    }  
  } 
  Pin0=0; //Pin의 값을 최고화
  Pin1=0;  
}
