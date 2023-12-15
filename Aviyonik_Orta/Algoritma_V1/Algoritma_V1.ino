#include <Adafruit_MPL3115A2.h>
#include<Wire.h>
#include<ADXL345_WE.h>
#define ADXL345_I2CADDR 0x53 // 0x1D if SDO = HIGH
ADXL345_WE myAcc = ADXL345_WE(ADXL345_I2CADDR);

Adafruit_MPL3115A2 baro;

#define SEA_LEVEL_PRESSURE    1015.0f  
#define take_off_altitude 100 
#define burn_Out_altitude 400

bool take_off_info = false;
bool take_off = false;
bool burnOut = false;
bool fall_detected = false;
bool main_detected = false;

byte info =0;
byte detect =0;

int altitude_box[2] = {0,0};
int altitude_last =0;
int altitude_first =0;
int i = 0;
int mesur =0;

unsigned long eskiZaman =0;
unsigned long anlikZaman;


//#include "BluetoothSerial.h"

//#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
//#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
//#endif
//
//#if !defined(CONFIG_BT_SPP_ENABLED)
//#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
//#endif

//BluetoothSerial Serial2;

void setup()
{
  Wire.begin();
  pinMode(4,OUTPUT);
  Serial.begin(9600);
  Serial2.begin(9600); //Bluetooth device name
  

  

   
  pinMode(12,OUTPUT);
      
  if (!baro.begin()) {
    Serial.println("Could not find sensor. Check wiring.");
    while(1);
  }
  
  baro.setSeaPressure(1013.26) ;
  myAcc.setCorrFactors(-266.0, 285.0, -268.0, 278.0, -291.0, 214.0);
  delay(2000);
  myAcc.measureAngleOffsets();
  myAcc.setDataRate(ADXL345_DATA_RATE_50);
  Serial.print(myAcc.getDataRateAsString());

/* Choose the measurement range
    ADXL345_RANGE_16G    16g     
    ADXL345_RANGE_8G      8g     
    ADXL345_RANGE_4G      4g   
    ADXL345_RANGE_2G      2g
*/
  myAcc.setRange(ADXL345_RANGE_8G);

}
void loop()
{
  anlikZaman = millis();
  Wire.begin();
  xyzFloat corrAngles = myAcc.getCorrAngles();
  
 int alti = int(baro.getAltitude());  

   if (alti > take_off_altitude) {
    take_off = true;
    info=1;
    if(take_off_info==false){
  
     delay(200);
     take_off_info= true;
    }
    
  }

  if(take_off >0 && alti > burn_Out_altitude){
    burnOut=true;
    info=2;
   
     
    }  

  if(burnOut){
   recovery(alti);    
  }


 if(anlikZaman - eskiZaman > 100){
  flight_status(alti,info);
//  Serial2.print("Açı: ");
//  Serial2.println((String)corrAngles.x);
  Serial.print("Angle x = ");
  Serial2.println();
  eskiZaman=anlikZaman;
 }
   
}

void  flight_status(int altitude, byte info){
  if(info==1){

 Serial2.print("Alt:");
 Serial2.print(String(altitude*2));
 Serial2.print("\t"); 
 Serial2.print("Yükseliyor ");
 Serial2.print("\t");
  }
  else if(info==2){
 Serial2.print("Alt:");
 Serial2.print(String(altitude*2));
 Serial2.print("\t");
 Serial2.print("BurnOut - Yükseliyor");
 Serial2.print("\t");
    
  }
  else if(info==3){
 Serial2.print("Alt:");
 Serial2.print(String(altitude*2));
 Serial2.print("\t");
 Serial2.print("Düşüyor ");
 Serial2.print("\t");
    
  }
  
  else{
    
 Serial2.print("İrtifa:");
 Serial2.print(String(altitude*2));
 Serial2.print("\t");
 Serial2.print("Rampa ");
 Serial2.print("\t");
  }
  

if(detect==1){
  Serial2.print(" K1:True K2:False");
}
else if(detect==2){
  Serial2.print(" K1:True K2:True");
}
else{
  Serial2.print(" K1:False K2:False");
}

}



void recovery(int altitude){
  

altitude_box[i] = altitude;
delay(10);
i=i+1;
  
if(i==2){
  
 i=0;
 mesur = altitude_box[1] - altitude_box[0];
  
}



 if(mesur < 0){
  info=3;
  digitalWrite(4,1);
  detect=1;
 }

if((altitude-50)<90){
digitalWrite(4,!digitalRead(4));
detect=2;
}

}
