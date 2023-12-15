#include <TinyGPSPlus.h>


static const uint32_t GPSBaud = 9600;

TinyGPSPlus gps;

void setup()
{
 Serial.begin(9600);
Serial1.begin(9600); 
Serial.println("Deneme");

}  




void loop()
{
   
  while (Serial.available() > 0)
    if (gps.encode(Serial.read())){
  displayInfo();
    delay(200);
   }
  
  

}

void displayInfo()
{
   
  if (gps.location.isValid())
  {/*
    Serial.write((byte)0x00); //Alıcı Adresi HIGH
    Serial.write(0x29);       //Alıcı Adresi LOW
    Serial.write(0x12);
    */
    
    Serial.print(F("rl")); 
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.println(gps.location.lng(), 6);
    
  }
  else{/*
    Serial.write((byte)0x00); //Alıcı Adresi HIGH
    Serial.write(0x29);       //Alıcı Adresi LOW
    Serial.write(0x12);
    */
    Serial.println(F("INVALID"));
    
  }
}


 
