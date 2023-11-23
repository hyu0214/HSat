#include <Wire.h>
#include <MPU6050.h>

#define enA 9
#define in1 6
#define in2 7

MPU6050 mpu;

const int MaxAccValue = 16384;  // The maximum value from MPU6050 accelerometer
const int threshold = 2000;

int rotDirection = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();

  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  
  // Set initial rotation direction
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
}

void loop() {
  // Read accelerometer values
  int16_t accelX = mpu.getAccelerationX();

  // Map the accelerometer value range(+-16384) to motor PWM range(0 ~ 255)
  int pwmOutput = map(accelX, -MaxAccValue, MaxAccValue, 0, 255);
  pwmOutput = constrain(pwmOutput, 0, 255);  // Ensure the PWM value is in the valid range

  analogWrite(enA, pwmOutput);

  // Adjust the threshold
  if (abs(accelX) > threshold) {
    // Change rotation direction based on acc value
    if (accelX > 0 && rotDirection == 0) {
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      rotDirection = 1;
    } else if (accelX < 0 && rotDirection == 1) {
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      rotDirection = 0;
    }
  }
}
