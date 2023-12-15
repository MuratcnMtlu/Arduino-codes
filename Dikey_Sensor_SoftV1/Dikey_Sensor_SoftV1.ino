#include "BluetoothSerial.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_MPL3115A2.h>
#include <CuteBuzzerSounds.h>



#define DELAY_MS (100)
#define BUZZER 33 
#define SafetyPin 12

bool StartControl = false;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
Adafruit_MPL3115A2 baro;

void setup() {

  Serial.begin(115200);
  Serial2.begin(115200);
  //SerialBT.begin("KouSpace_ESP32"); //Bluetooth device name

   if(!bno.begin())
  {
   
    Serial.println("BNO055 Hata!");
   
  }
  if (!baro.begin()) {
    Serial.println("MPL3115 Hata!");
  
  }

  delay(1000);

  /* Display some basic information on this sensor */
 // displaySensorDetails();
  bno.setExtCrystalUse(true);
  baro.setSeaPressure(1013.26);

  cute.init(BUZZER);
}

void loop() {
 
  
if(StartControl){
  PrintData(); 
}
 
  
 
 // Serial.print(",");
 // Serial.print(baro.getAltitude());
 
  //BT_Command();
  
delay(DELAY_MS);
  
}

void BT_Command(void){
  

 
}



void PrintData(void){
  
  sensors_event_t event;
  bno.getEvent(&event);


  
  Serial.print(event.orientation.x, 0);
  Serial.print(",");
  Serial.print(event.orientation.y, 0);
  Serial.print(",");
  Serial.print(event.orientation.z, 0);   
 // Serial.print(",");
 // Serial.print(baro.getAltitude());
  Serial.print("\r\n");

  
}

void IsFalling(void){
  if(digitalRead(12)>0){
    StartControl = true;
    cute.play(S_CONNECTION);
  }
  else{
    StartControl = false;
    cute.play(S_MODE3);
  }
}













void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void displayCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system)
  {
    Serial.print("! ");
  }

  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.print(mag, DEC);
}
