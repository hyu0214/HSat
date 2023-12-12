#define enB 9
#define IN3 6
#define IN4 7
int pwm;

void setup() {
  Serial.begin(9600);
  pinMode(enB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pwm = 0;
}

void loop() {
  if(Serial.available()){
    pwm = Serial.parseInt();
    Serial.print("Set PWM value: ");
    Serial.println(pwm);
  }
  Motor_control(pwm);
}

void Motor_control(int pwm) {
  if (pwm <= 0) {//set direction according to sign of 'pwm'
    digitalWrite(IN3, LOW); //CW rotation (-)
    digitalWrite(IN4, HIGH);
  } else {
    digitalWrite(IN3, HIGH); //CCW rotation (+)
    digitalWrite(IN4, LOW);
  }
  analogWrite(enB,abs(pwm));//write absolute value of PWM into PWM pin
}
