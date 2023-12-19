#define CDS1        A0 //Analog Pin0
#define CDS2        A1 //Analog Pin1
#define CDS3        A2 //Analog Pin2
const int analogPins[] = {0, 1, 2};                                       //define each analogue pin
const int numPins = sizeof(analogPins) / sizeof(analogPins[0]);

struct t_CDSvalue {
    int sidelight;
    int toplight;
};

void init_CDS_ADC(){
  ADMUX |= (0<<REFS1) | (1<<REFS0);
  ADMUX |= (1<<ADLAR); 
  ADCSRA |= (1<<ADEN);
  ADCSRA |= (1<<ADPS2);  //16
}

t_CDSvalue measure_CDS(){
  t_CDSvalue destination;
  uint8_t APin0, APin1, APin2;
    for (int i =0; i < numPins-1; i++){
    ADMUX = (ADMUX & 0xF0) | (analogPins[i] & 0x0F);
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    uint8_t value = ADCH;
    if (i == 0) {
      APin0 = value;
    }
    else if (i == 1){
      APin1 = value;
      } 
    }  
 
  ADMUX = (ADMUX & 0xF0) | (analogPins[numPins - 1] & 0x0F);
  ADCSRA |= (1 << ADSC);
  while (ADCSRA & (1 << ADSC));
  APin2 = ADCH;

  destination.sidelight = (APin0 + APin1) / 2;
  destination.toplight = APin2;
  return destination;
}

void setup() {
  Serial.begin(115200);
  init_CDS_ADC();
}


void loop() {
  t_CDSvalue cdsValues = measure_CDS();
  
  // Print the measured values
  //Serial.print("Sidelight: ");
  Serial.print(cdsValues.sidelight);
  //Serial.print("\tToplight: ");
  Serial.println(cdsValues.toplight);

  // Add a delay if needed
  //delay(1000);  // Adjust the delay time as needed
} 
  



