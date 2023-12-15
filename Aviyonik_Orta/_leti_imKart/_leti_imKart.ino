
#include <SoftwareSerial.h>

#define rxPin PB7
#define txPin ,PB6

SoftwareSerial mySerial =  SoftwareSerial(rxPin, txPin);



void setup() {
  Serial.begin(9600);
  
  mySerial.begin(9600);

  Serial.println("Lora Deneme");
  delay(200);

}

void loop() {
Serial.println("Lora Deneme");
/*
  if (mySerial.available() > 0) {
    char data = mySerial.read();
    Serial.print(data);

  }
  */
  delay(200);

}
