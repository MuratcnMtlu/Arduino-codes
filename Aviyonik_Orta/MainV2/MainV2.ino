#include <Wire.h>
#include <SPI.h>
#include<Wire.h>
#include "DFRobot_BMP280.h"
#include<ADXL345_WE.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include "BluetoothSerial.h"

#define ADXL345_I2CADDR 0x53

#define SEA_LEVEL_PRESSURE    1015.0f
#define rxPin 9
#define txPin 10
#define mySerial Serial1
#define buzzer 4
#define trig 25


#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

float alt_ref = 0;
float Altitude = 0;
uint8_t dataPacket[27];
double latitude = 0;
double longitude = 0;
float   alt_last = 0;
float   alt_first = 0;
float   alt_0 = 0;
float   alt_1 = 0;

unsigned long t_0             = millis();
unsigned long t_1             = millis();
unsigned long print_timer     = millis();
unsigned long vel_timer       = millis();





typedef DFRobot_BMP280_IIC    BMP;
BMP   bmp(&Wire, BMP::eSdoLow);
BluetoothSerial SerialBT;



void printLastOperateStatus(BMP::eStatus_t eStatus)
{
  switch (eStatus) {
    case BMP::eStatusOK:    Serial.println("everything ok"); break;
    case BMP::eStatusErr:   Serial.println("unknow error"); break;
    case BMP::eStatusErrDeviceNotDetected:    Serial.println("device not detected"); break;
    case BMP::eStatusErrParameter:    Serial.println("parameter error"); break;
    default: Serial.println("unknow status"); break;
  }
}


ADXL345_WE myAcc = ADXL345_WE(ADXL345_I2CADDR);
TinyGPSPlus gps;

static const uint32_t GPSBaud = 9600;

void setup() 
{
  pinMode(trig, OUTPUT);
  pinMode(buzzer, OUTPUT);
  digitalWrite(buzzer, 0);
  digitalWrite(trig, 0);
  Wire.begin();
  Serial.begin(9600);
  SerialBT.begin("SIGABORT"); 
  Serial1.begin(GPSBaud);
  Serial2.begin(9600);
  if (!myAcc.init()) {
    Serial.println("ADXL345 not connected!");
  }
  myAcc.setCorrFactors(-266.0, 285.0, -268.0, 278.0, -291.0, 214.0);
  myAcc.measureAngleOffsets();
  myAcc.setDataRate(ADXL345_DATA_RATE_50);
  myAcc.setRange(ADXL345_RANGE_2G);
  bmp.reset();

  digitalWrite(buzzer,1);
  delay(75);
  digitalWrite(buzzer,0);
  for(int i = 0;i<9;i++)
  {
    alt_ref =  bmp.calAltitude(SEA_LEVEL_PRESSURE, bmp.getPressure());
    i++;
  }
  
}

void loop() {
  float   temp = bmp.getTemperature();
  uint32_t    press = bmp.getPressure();
  float   alti = bmp.calAltitude(SEA_LEVEL_PRESSURE, press);
  xyzFloat g = myAcc.getGValues();
  xyzFloat angle = myAcc.getAngles();
  Altitude = bmp.calAltitude(SEA_LEVEL_PRESSURE, press) - alt_ref;
  printBT();
  
  
  if (Serial.available() > 0)
  {
    if (gps.encode(Serial.read()))
      displayInfo(&latitude, &longitude);
  }
  
  if (millis() - print_timer > 200)
  {
     print_timer = millis();
     int vel = round(readVelocity());
     createPacket(byte(angle.y), byte(angle.x), random(0, 100), byte(g.y), int(Altitude), vel, latitude, longitude);
     for (int i = 0; i < sizeof(dataPacket); i++) 
     {
        printHex(dataPacket[i]);
     }
     Serial.println();
  }


}

void printBT()
{
    if (SerialBT.available()) 
    {
       char c = SerialBT.read();
       if(c == 't')
       {
        digitalWrite(25, 1);
        delay(1000);
        digitalWrite(25, 0);
       }
    }
}

float readVelocity() 
{
  float Velocity;
  if (millis() - vel_timer >  50) {
    vel_timer = millis();
    alt_1 = Altitude;
    t_1   = millis();
    float dh = alt_1 - alt_0;
    float dt = ((float)t_1 - (float)t_0) / (float)1000;
    Velocity = (dh / dt);
    t_0 = t_1;
    alt_0 = alt_1;
  }
  return (Velocity);
}




void displayInfo(double *latp, double  *lngp)
{

  if (gps.location.isValid())
  { 
    *latp = gps.location.lat();
    *lngp = gps.location.lng();
  }
  else 
  {
    /*
      Serial.write((byte)0x00); //Alıcı Adresi HIGH
      Serial.write(0x29);       //Alıcı Adresi LOW
      Serial.write(0x12);
    */
    *latp = 0;
    *lngp = 0;
  }
}

/***********************************************************************************************************************************/

void double2Hex(double value, int pos) {
  union {
    double   dNumber;
    uint8_t  bytes[8];
  } doubleConversion;
  doubleConversion.dNumber = value;
  for (int i = 0; i < 8; i++) {
    dataPacket[pos + 7 - i] = doubleConversion.bytes[i];
  }
}

void printHex(uint8_t num) {
  char hexCar[2];
  sprintf(hexCar, "%02X", num);
  Serial.print(hexCar);
}

void createPacket(byte angle_x, byte angle_y, byte pil, byte accel, int alt, int vel, double lat, double lng) {
  dataPacket[0] = 0x19;
  dataPacket[1] = 0x00;
  dataPacket[2] = angle_x;
  dataPacket[3] = angle_y;
  dataPacket[4] = pil;
  dataPacket[5] = accel;
  dataPacket[6] = highByte(alt);
  dataPacket[7] = lowByte(alt);
  dataPacket[8] = highByte(vel);
  dataPacket[9] = lowByte(vel);
  double2Hex(lat, 10);
  double2Hex(lng, 18);

  unsigned long checkSum = 0;
  for (int i = 0; i < 26; i++) {
    checkSum += dataPacket[i];
  }
  checkSum =  0xFF ^ checkSum;
  checkSum += 0x01;
  union {
    unsigned long val;
    uint8_t bytes[4];
  } long_num;
  long_num.val = checkSum;
  dataPacket[26] = long_num.bytes[0];
}
