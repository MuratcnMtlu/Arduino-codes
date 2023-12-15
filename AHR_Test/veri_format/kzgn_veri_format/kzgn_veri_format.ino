uint8_t dataPacket[27];
void setup() {
  Serial.begin(9600);

}

void loop() {
  createPacket(random(0,180), random(0,180), random(0,100), random(0,90), random(0,3200), random(-350,350), random(1, 500) / 100.0, random(1, 500) / 100.0);
  for (int i = 0; i < sizeof(dataPacket); i++) {
    printHex(dataPacket[i]);
  }
  Serial.println();
  delay(150);
}

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
