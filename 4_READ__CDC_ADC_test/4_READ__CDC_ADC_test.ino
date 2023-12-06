
const int analogPins[] = {0, 1};                                       //아날로그 핀들을 정의
const int numPins = sizeof(analogPins) / sizeof(analogPins[0]);        //아날로그 핀들의 개수 정의


void init_CDS_ADC(){                                     //각기 다른 조도센서와 연결된 아날로그 핀들의 공통설정
  ADMUX |= (0<<REFS1) | (1<<REFS0);                      //참조 전압 5V로 설정
  ADMUX |= (0<<ADLAR);                                   //왼쪽 정렬
  ADCSRA |= (1<<ADEN);                                   //ADC 시작
  ADCSRA |= (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);        //Prescaler=128
  }


void setup() {
  Serial.begin(9600);
  init_CDS_ADC();        
}

void loop() {
  for (int i =0; i < numPins; i++){                      //A0부터 A[i]까지 반복
    ADMUX = (ADMUX & 0xF0) | (analogPins[i] & 0x0F);     //기존의 설정 재확인
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
