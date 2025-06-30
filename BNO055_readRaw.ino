// collect raw data from BNO055 9DOF sensor
// S. James Remington 6/28/2025
#include <Wire.h>
#include "BNO055.h";
// rate gyro scale is 16LSB/dps or 900LSB/(radian/sec)

// utility routines for Wire
// get N bytes of data, starting from address "reg"
void I2C_get(uint8_t I2Caddr, uint8_t reg, uint8_t * buf, uint8_t N) {
  Wire.beginTransmission(I2Caddr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(I2Caddr, N);
  while (Wire.available()) *buf++ = Wire.read();
}

// write a single byte to a device register
void I2C_writeReg(uint8_t I2Caddr, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(I2Caddr);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission(true);
}

void setup() {

  uint8_t result = 0;

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW); //off

  delay(1000); //wait for BNO055 to boot up
  Serial.begin(115200);
  Wire.begin();

  //detect BNO055 presence
  I2C_get(BNO055_A0, BNO055_WHO_AM_I, &result, 1);
  if (result != BNO055_CHIP_ID) {
    Serial.print("BNO055 not found: ");
    Serial.println(result, HEX);
    while (1) delay(1); //hang
  }
  
 // disable default sensor fusion mode and enable raw data collection
  I2C_writeReg(BNO055_A0, BNO055_SYS_TRIGGER, 0x20); //system reset
  delay(1000);
  I2C_writeReg(BNO055_A0, BNO055_SYS_TRIGGER, 0x80); //use external crystal.
  // set to measurement mode, raw data
  I2C_writeReg(BNO055_A0, BNO055_OPER_MODE, BNO055_OPER_MODE_AMG);
  delay(1000); //minimum 7 ms delay
  I2C_get(BNO055_A0, BNO055_OPER_MODE, &result, 1); //report new setting

  Serial.print("Operation Mode set to ");
  Serial.println(result, HEX);
  Serial.println("Setup done");
  Serial.println("Turn sensor slowly to collect acc/mag data for /r/n 
    several hundred orientations uniformly covering the 3D sphere");
  Serial.println("copy/paste the data and create acc/mag .CSV files for input to calibrate3.py");
}

void loop() {
  int acc[3] = {0};  //raw acceleration values
  I2C_get(BNO055_A0, BNO055_ACCEL_DATA, (uint8_t *)acc, 6);  //little endian in both sensor and Arduino
  for (int i = 0; i < 3; i++) {
    Serial.print(acc[i]);
    Serial.print(", ");
  }
  int mag[3] = {0};  //raw magnetometer values
  I2C_get(BNO055_A0, BNO055_MAG_DATA, (uint8_t *)mag, 6);  //little endian in both sensor and Arduino
  for (int i = 0; i < 3; i++) {
    Serial.print(mag[i]);
    if (i<2) Serial.print(", ");
  }
  /* optional print gyro data
    int gyro[3] = {0};  //raw gyro values
  I2C_get(BNO055_A0, BNO055_GYRO_DATA, (uint8_t *)gyro, 6);  //little endian in both sensor and Arduino
  for (int i = 0; i < 3; i++) {
    Serial.print(gyro[i]);
    if (i < 2) Serial.print(", ");
  }
  */
  Serial.println();
  delay(200);
}
