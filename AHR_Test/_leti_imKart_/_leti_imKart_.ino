#include <SoftwareSerial.h>
                     /* Tx -Rx*/
SoftwareSerial MainPort(PB6, PB7);


void setup() {
  Serial.begin(9600);
  MainPort.begin(9600);
 pinMode(PA13,OUTPUT);
 Serial.println("Kouspace 22-23");
}

void loop() {
  
  if(MainPort.available()>0){
      
    Serial.print(MainPort.read());
    digitalWrite(PA13,!digitalRead(PA13));
  }
delay(200);



}
