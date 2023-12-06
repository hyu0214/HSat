#define enA 9
#define in1 6
#define in2 7

void setup() {
  Serial.begin(9600);

  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
}

void loop() {
  // Run
  analogWrite(enA, 255);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);

  delay(3000);

  // Stop_using brake
  analogWrite(enA, 0);

  delay(100);

  digitalWrite(in1, HIGH);
  digitalWrite(in2, HIGH);

  // Reverse 
  analogWrite(enA, 255);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);

  delay(3000);

  // Stop_using brake
  analogWrite(enA, 0);

  delay(100);

  digitalWrite(in1, HIGH);
  digitalWrite(in2, HIGH);
}
