
int idrtl = A0; //top_left_idr
int idrbl = A1; //bottom_left_idr
int idrtr = A2; //top_right_idr
//int idrbr =A3; //bottom_left_idr

void setup() {
  Serial.begin(9600);
}

void loop() {
  int tl =analogRead(idrtl);
  int bl =analogRead(idrbl);
  int tr =analogRead(idrtr);
  //int br =analogRead(idrbr);
  delay(1000);
  Serial.print("tl= ");
  Serial.println(tl);
  Serial.print("bl= ");
  Serial.println(bl);
  Serial.print("tr= ");
  Serial.println(tr);
  //Serial.print("br= ");
  //Serial.println(br); 
}
