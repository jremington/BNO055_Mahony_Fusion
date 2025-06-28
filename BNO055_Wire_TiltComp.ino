
#include <Wire.h>
#include "BNO055.h"

// rate gyro scale is 16LSB/dps or 900LSB/(radian/sec)
// raw data vectors
int16_t acc[3], gyro[3], mag[3];

// VERY IMPORTANT!
//These are the previously determined offsets and scale factor corrections for accelerometer and magnetometer

//accelerometer
float A_B[3]
{ -7.89, -22.65, 13.72};

float A_Ainv[3][3]
{ { 1.03871 , -0.00808 , 0.01779 },
  { -0.00808 , 1.03709 , 0.00248 },
  { 0.01779 , 0.00248 , 1.00729 }
};

// magnetometer
float M_B[3]
{ -977.51, -256.69, 232.77};

float M_Ainv[3][3]
{ { 1.18301 , 0.01897 , -0.00353 },
  { 0.01897 , 1.18829 , 0.00052 },
  { -0.00353 , 0.00052 , 1.22251 }
};

// local magnetic declination in degrees
float declination = -14.84;
/*
  This tilt-compensated code assumes that the sensor board is oriented with Accel X pointing
  to the North, Y pointing West, and Z pointing up for heading = 0 degrees
  The code compensates for tilts of up to 90 degrees away from horizontal.
  Facing vector p is the direction of travel and allows reassigning these directions.
  It should be defined as pointing forward,
  parallel to the ground, with coordinates {X, Y, Z} (in magnetometer frame of reference).
*/
float p[] = {1, 0, 0};
//float p[] = {0, -1, 0};  //if Y marking on sensor board points toward 180 (due South)

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


#define PRINT_SPEED 300 // ms between angle prints
unsigned long lastPrint = 0; // Keep track of print time

void setup() {

  uint8_t result;

  pinMode(LED_BUILTIN, OUTPUT);

  delay(1000); //wait for BNO055 to boot up

  Wire.begin();
  Serial.begin(115200);

  //detect BNO055 presence
  I2C_get(BNO055_A0, BNO055_WHO_AM_I, &result, 1);
  if (result != BNO055_CHIP_ID) {
    Serial.print("BNO055 not found: ");
    Serial.println(result, HEX);
    while (1) delay(1); //hang
  }

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
  delay(1000); //need some delay or get zeros

  Serial.println("Tilt compensated compass started");
}

void loop() {
  static float Axyz[3], Mxyz[3]; //centered and scaled accel/mag data

  if (millis() - lastPrint > PRINT_SPEED)
  {
    I2C_get(BNO055_A0, BNO055_ACCEL_DATA, (uint8_t *)acc, 6);  //little endian in both sensor and Arduino
    I2C_get(BNO055_A0, BNO055_MAG_DATA, (uint8_t *)mag, 6);  //little endian in both sensor and Arduino
    get_scaled_IMU(Axyz, Mxyz);
    /*
      // debug: normalized measurements

        Serial.print(Axyz[0]);
        Serial.print(", ");
        Serial.print(Axyz[1]);
        Serial.print(", ");
        Serial.print(Axyz[2]);
        Serial.print(", ");
        Serial.print(Mxyz[0]);
        Serial.print(", ");
        Serial.print(Mxyz[1]);
        Serial.print(", ");
        Serial.println(Mxyz[2]);
 */
    //  compute heading in degrees
    Serial.print("Heading: ");
    Serial.println(get_heading(Axyz, Mxyz, p, declination));
    lastPrint = millis(); // Update lastPrint time
  }
  // consider averaging a few headings for better results
}

// Returns a heading (in degrees) given an acceleration vector a due to gravity, a magnetic vector m, and a facing vector p.
// applies magnetic declination
int get_heading(float Acc[3], float Mag[3], float p[3], float magdec)
{
  float W[3], N[3]; //derived direction vectors

  // cross "Up" (acceleration vector, g) with magnetic vector (magnetic north + inclination) with  to produce "West"
  vector_cross(Acc, Mag, W);
  vector_normalize(W);

  // cross "West" with "Up" to produce "North" (parallel to the ground)
  vector_cross(W, Acc, N);
  vector_normalize(N);

  // compute heading in horizontal plane, correct for local magnetic declination in degrees

  float h = atan2(vector_dot(W, p), vector_dot(N, p)) * 180 / M_PI;
  //minus: conventional nav, heading increases North to East
  int heading = round(-(h + magdec));
  heading = (heading + 720) % 360; //apply compass wrap
  return heading;
}

// basic vector operations
void vector_cross(float a[3], float b[3], float out[3])
{
  out[0] = a[1] * b[2] - a[2] * b[1];
  out[1] = a[2] * b[0] - a[0] * b[2];
  out[2] = a[0] * b[1] - a[1] * b[0];
}

float vector_dot(float a[3], float b[3])
{
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

void vector_normalize(float a[3])
{
  float mag = sqrt(vector_dot(a, a));
  a[0] /= mag;
  a[1] /= mag;
  a[2] /= mag;
}

// function to subtract offsets and apply scale/correction matrices to IMU data

void get_scaled_IMU(float Axyz[3], float Mxyz[3]) {
  byte i;
  float temp[3];

  Axyz[0] = acc[0];
  Axyz[1] = acc[1];
  Axyz[2] = acc[2];
  Mxyz[0] = mag[0];
  Mxyz[1] = mag[1];
  Mxyz[2] = mag[2];

  //apply accel offsets (bias) and scale factors from Magneto

  for (i = 0; i < 3; i++) temp[i] = (Axyz[i] - A_B[i]);
  Axyz[0] = A_Ainv[0][0] * temp[0] + A_Ainv[0][1] * temp[1] + A_Ainv[0][2] * temp[2];
  Axyz[1] = A_Ainv[1][0] * temp[0] + A_Ainv[1][1] * temp[1] + A_Ainv[1][2] * temp[2];
  Axyz[2] = A_Ainv[2][0] * temp[0] + A_Ainv[2][1] * temp[1] + A_Ainv[2][2] * temp[2];
  vector_normalize(Axyz);

  //apply mag offsets (bias) and scale factors from Magneto

  for (i = 0; i < 3; i++) temp[i] = (Mxyz[i] - M_B[i]);
  Mxyz[0] = M_Ainv[0][0] * temp[0] + M_Ainv[0][1] * temp[1] + M_Ainv[0][2] * temp[2];
  Mxyz[1] = M_Ainv[1][0] * temp[0] + M_Ainv[1][1] * temp[1] + M_Ainv[1][2] * temp[2];
  Mxyz[2] = M_Ainv[2][0] * temp[0] + M_Ainv[2][1] * temp[1] + M_Ainv[2][2] * temp[2];
  vector_normalize(Mxyz);
}
