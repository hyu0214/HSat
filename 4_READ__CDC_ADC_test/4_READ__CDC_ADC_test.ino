
const int analogPins[] = {0, 1};
const int numPins = sizeof(analogPins) / sizeof(analogPins[0]);


void init_CDS_ADC(){
  ADMUX |= (0<<REFS1) | (1<<REFS0);
  ADMUX |= (0<<ADLAR); 
  ADCSRA |= (1<<ADEN);
  ADCSRA |= (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
  }


void setup() {
  Serial.begin(9600);
  init_CDS_ADC();
}

void loop() {
  for (int i =0; i < numPins; i++){
    ADMUX = (ADMUX & 0xF0) | (analogPins[i] & 0x0F);
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    uint16_t value = ADC;

    Serial.print("value");
    Serial.print(i);
    Serial.print(":");
    Serial.println(value);
    delay(1000);
  }
 
  

  //ADMUX |= (0 << MUX3) | (0 << MUX2) | (0 << MUX1) | (1 << MUX0);
  //ADCSRA |= (1 << ADSC);
  //while (ADCSRA & (1 << ADSC));
  //uint16_t ADC_01 = ADC;

  //Serial.print("value1:");
  //Serial.println(ADC_01);

  //ADMUX |= (0 << MUX3) | (0 << MUX2) | (1 << MUX1) | (0 << MUX0);
  //ADCSRA |= (1 << ADSC);
  //while (ADCSRA & (1 << ADSC));
  //uint16_t adcValue2 = ADC;

 
  
}
