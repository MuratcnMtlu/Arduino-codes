#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include<Wire.h>
#include<ADXL345_WE.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

#define ADXL345_I2CADDR 0x53
#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)
#define rxPin 9
#define txPin 10
#define mySerial Serial1

int alt_ref = 0;
int Altitude = 0;
uint8_t dataPacket[27];
double latitude =0;
double longitude = 0;










//SoftwareSerial mySerial =  SoftwareSerial(rxPin, txPin);
Adafruit_BMP280 bmp; // I2C
ADXL345_WE myAcc = ADXL345_WE(ADXL345_I2CADDR);
TinyGPSPlus gps;

static const uint32_t GPSBaud = 9600;

void setup() {
  
  
  Wire.begin();
  Serial.begin(9600);
  Serial1.begin(GPSBaud);

  while ( !Serial ) delay(100);
  unsigned status;
  status = bmp.begin();
  if (!status) {
    Serial.println(F("BMP 280 HATA !"));

    while (1) delay(10);
  }

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */


  if (!myAcc.init()) {
    Serial.println("ADXL345 not connected!");
  }

  myAcc.setCorrFactors(-266.0, 285.0, -268.0, 278.0, -291.0, 214.0);
  myAcc.measureAngleOffsets();
  myAcc.setDataRate(ADXL345_DATA_RATE_50);
  myAcc.setRange(ADXL345_RANGE_2G);

  for (int i = 0; i < 10; i++) {
    alt_ref = bmp.readAltitude(1013.25);
    delay(100);
  }



}

void loop() {
 // displayInfo();
  xyzFloat g = myAcc.getGValues();
  xyzFloat angle = myAcc.getAngles();
  // readAltitude();

   while (Serial1.available() > 0)
    if (gps.encode(Serial1.read())){
  displayInfo();
   
   }
  createPacket(byte(angle.y), byte(angle.z), random(0,100), byte(g.z), int(readAltitude()), random(-350,350), latitude, longitude);
  for (int i = 0; i < sizeof(dataPacket); i++) {
    printHex(dataPacket[i]);
  }
  Serial.println();
  delay(200);
  
  
  
  
  
  /*
   * 
   *  Serial.print("g-x     = ");
  Serial.print(g.x);

  Serial.print("  |  g-z     = ");
  Serial.println(g.z);
    Serial.print("Angle x  = ");
    Serial.print(angle.x);
    Serial.print("  |  Angle y  = ");
    Serial.print(angle.y);
    Serial.print("  |  Angle z  = ");
    Serial.println(angle.z);



    Serial.print(F("Temperature = "));
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");

    Serial.print(F("Pressure = "));
    Serial.print(bmp.readPressure());
    Serial.println(" Pa");
  */

 
}




void displayInfo()
{

  if (gps.location.isValid())
  {/*
    Serial.write((byte)0x00); //Alıcı Adresi HIGH
    Serial.write(0x29);       //Alıcı Adresi LOW
    Serial.write(0x12);
    */
        
   latitude = gps.location.lat();
   longitude =gps.location.lng();
    
  }
  else{/*
    Serial.write((byte)0x00); //Alıcı Adresi HIGH
    Serial.write(0x29);       //Alıcı Adresi LOW
    Serial.write(0x12);
    */
    latitude = 0;
    longitude =0;
  }
    }
  






int readAltitude() {
  return Altitude = bmp.readAltitude(1013.25) - alt_ref;
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
