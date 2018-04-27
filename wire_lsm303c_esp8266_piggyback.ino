#include <Wire.h>
//  #define acc_address  0x19  //Address for LSM303DLHC accelerometer binary 0011001b (Datasheet DLHC)

#define acc_address  0x1D  //Address for LSM303C accelerometer binary 0011101b (Datsheet C)

#define OUT_X_L_A  0x28

void setup() {
  //Internal Pullups on D7 (scl), D6 (sda)
  pinMode(D7, INPUT_PULLUP);
  pinMode(D6, INPUT_PULLUP);
  pinMode(D8, LOW);


  Wire.begin(D6, D7);       // Define I2C Pins as D1=scl D2=sda
  Serial.begin(115200);  // start serial for output



  // Enable the accelerometer
  Wire.beginTransmission(acc_address);
  Wire.write((byte)0x20);  // I2C subaddress
  Wire.write((byte) 0x27);  // I2C byte data
  Wire.endTransmission();

  //***************************************
  //Set accelerometer seetings

  // 0x08 = 0b00001000 changes register settings to:
  // FS = 00 (+/- 2 g full scale); HR = 1 (high resolution enable)
  Wire.beginTransmission(acc_address);
  Wire.write((byte)0x23);  //address for CTRL_REG4_A
  Wire.write(0x08);

  // 0x47 = 0b01000111 changes register settings to:
  // ODR = 0100 (50 Hz ODR); LPen = 0 (normal mode); Zen = Yen = Xen = 1 (all axes enabled)
  Wire.beginTransmission(acc_address);
  Wire.write((byte)0x20);  //address for CTRL_REG1_A
  Wire.write(0x47);
  //****************************************

  Serial.println("Setup Complete");
}

void loop() {

  Wire.beginTransmission(acc_address);
  Wire.write(OUT_X_L_A | (1 << 7));
  Wire.endTransmission();

  Wire.requestFrom(acc_address, 6);
  while (!Wire.available());  // busy-wait
  byte xla = Wire.read();
  byte xha = Wire.read();
  byte yla = Wire.read();
  byte yha = Wire.read();
  byte zla = Wire.read();
  byte zha = Wire.read();

  // combine high and low bytes
  int ax = (int16_t)(xha << 8 | xla);
  int ay = (int16_t)(yha << 8 | yla);
  int az = (int16_t)(zha << 8 | zla);

  Serial.print("AccelX = " + String(ax));
  Serial.print("  ");
  Serial.print("AccelY = " + String(ay));
  Serial.print("  ");
  Serial.println("AccelZ = " + String(az));

  delay(1000);
}
