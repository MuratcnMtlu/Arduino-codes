


  

void setup() {
  
  Serial.begin(9600);

  
}
void loop() {
  Serial.write((byte)0x00); //Alıcı Adresi HIGH
  Serial.write(0x29);       //Alıcı Adresi LOW
  Serial.write(0x12);       //Alıcı Kanalı =0x17=23    (410M+23=433 MHz)
  Serial.write(0x10);
  Serial.println();
  
  delay(200);
}
