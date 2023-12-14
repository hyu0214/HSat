#include <SoftwareSerial.h> 

SoftwareSerial BTSerial(12, 13); //pin(TX,RX) 

void setup(){
Serial.begin(9600);
Serial.println("Hello!");  
BTSerial.begin(38400);
}

void loop(){
  while (BTSerial.available()){ 
    byte data = BTSerial.read();
    Serial.write(data);
  }  
 
  while (Serial.available()){
    byte data = Serial.read();
    BTSerial.write(data); 
  }
}
