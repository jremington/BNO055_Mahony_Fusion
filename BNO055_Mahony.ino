#include <Wire.h>
#include "BNO055.h"

// rate gyro scale is 16LSB/dps or 900LSB/(radian/sec)
// raw data vectors
int16_t acc[3], gyro[3], mag[3];

// VERY IMPORTANT!
//These are the previously determined offsets and scale factor corrections for accelerometer and magnetometer
float gyro_offsets[3] = {0.0}; //insert predetermined offsets
bool cal_gyro = true;      //  or calibrate at startup

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

// These are the free parameters in the Mahony filter and fusion scheme,
// Kp for proportional feedback, Ki for integral
// Kp is not optimized and will depend on the sensor. Kp =1.0 works; Ki seems not to be needed.

#define Kp 1.0
#define Ki 0.0

unsigned long now = 0, lastUpdate = 0, lastPrint = 0; //micros() timers for AHRS loop
float deltat = 0;  //loop time in seconds
#define PRINT_SPEED 300 // ms between angle prints
float eMag = 0.0; //debug, magnitude of error vector (global)

// Vector to hold quaternion
float q[4] = {1.0, 0.0, 0.0, 0.0};
float yaw, pitch, roll; //Euler angle output

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

  uint8_t result;

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

  //  Serial.print("Operation Mode set to ");
  //  Serial.println(result, HEX);
  //  Serial.println("Setup done");
  delay(1000); //need some delay or get zeros

  if (cal_gyro) get_gyro_offsets();
  lastPrint = millis(); //reset to start

  //  Serial.println("Starting Mahony filter");
}

void loop() {

  float Gxyz[3] = {0}, Axyz[3] = {0}, Mxyz[3] = {0}; //centered and scaled gyro/accel/mag data

  I2C_get(BNO055_A0, BNO055_ACCEL_DATA, (uint8_t *)acc, 6);  //little endian in both sensor and Arduino
  I2C_get(BNO055_A0, BNO055_MAG_DATA, (uint8_t *)mag, 6);  //little endian in both sensor and Arduino
  I2C_get(BNO055_A0, BNO055_GYRO_DATA, (uint8_t *)gyro, 6);  //little endian in both sensor and Arduino
  get_scaled_IMU(Axyz, Mxyz);

  //subtract offsets and scale gyro
  for (int i = 0; i < 3; i++) Gxyz[i] = (gyro[i] - gyro_offsets[i]) / 900.0; //convert to radians/sec


  // if necessary, remap axes here. X axis is assumed to point magnetic North for yaw = 0

  now = micros();
  if (now - lastUpdate > 50000) { //20 Hz update rate for magnetometer. Faster OK?
    deltat = (now - lastUpdate) * 1.0e-6; //seconds since last update

    MahonyQuaternionUpdate(Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2],
                           Mxyz[0], Mxyz[1], Mxyz[2], deltat);

    if (millis() - lastPrint > PRINT_SPEED) {

      // occasionally report Tait-Bryan angles.
      // Strictly valid only for approximately level movement

      // Standard sensor orientation : X magnetic North, Y West, Z Up (NWU)
      // this code corrects for magnetic declination.
      // Pitch is angle between sensor x-axis and Earth ground plane, toward the
      // Earth is positive, up toward the sky is negative. Roll is angle between
      // sensor y-axis and Earth ground plane, y-axis up is positive roll.
      // Tait-Bryan angles as well as Euler angles are
      // non-commutative; that is, the get the correct orientation the rotations
      // must be applied in the correct order.
      //
      // http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
      // which has additional links.

      // WARNING: This angular conversion is for DEMONSTRATION PURPOSES ONLY. It WILL
      // MALFUNCTION for certain combinations of angles! See https://en.wikipedia.org/wiki/Gimbal_lock

      roll  = atan2((q[0] * q[1] + q[2] * q[3]), 0.5 - (q[1] * q[1] + q[2] * q[2]));
      pitch = asin(2.0 * (q[0] * q[2] - q[1] * q[3]));
      yaw   = atan2((q[1] * q[2] + q[0] * q[3]), 0.5 - ( q[2] * q[2] + q[3] * q[3]));
      // to degrees
      yaw   *= 180.0 / PI;
      pitch *= 180.0 / PI;
      roll *= 180.0 / PI;

      // http://www.ngdc.noaa.gov/geomag-web/#declination
      //conventional nav, yaw increases CW from North, corrected for local magnetic declination

      yaw = -(yaw + declination);
      if (yaw < 0) yaw += 360.0;
      if (yaw >= 360.0) yaw -= 360.0;

      Serial.print(yaw, 0);
      Serial.print(", ");
      Serial.print(pitch, 0);
      Serial.print(", ");
      Serial.print(roll, 0);
      Serial.print(", ");
      long tmp = 10000 * eMag;
      if (tmp > 300) tmp = 300; //debug error vector magnitude
      Serial.print(tmp);
      Serial.println();
      lastPrint = millis(); // Update lastPrint time
    }
    lastUpdate=micros();
  }
}


// vector math
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
void get_gyro_offsets(void) {
  long sum[3] = {0};
  for (int i = 1; i < 100; i++ ) { //average some readings
    I2C_get(BNO055_A0, BNO055_GYRO_DATA, (uint8_t *)gyro, 6);  //little endian in both sensor and Arduino
    for (int j = 0; j < 3; j++) sum[j] += gyro[j];
    delay(20); //gyro is faster than mag
  }
  for (int j = 0; j < 3; j++) gyro_offsets[j] = sum[j] / 100.0; // store result in global offset array
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

// Mahony orientation filter, assumed World Frame NWU (xNorth, yWest, zUp)
// Modified from Madgwick version to remove Z component of magnetometer:
// The two reference vectors are now Up (Z, Acc) and West (Acc cross Mag)
// sjr 3/2021
// input vectors ax, ay, az and mx, my, mz MUST be normalized!
// gx, gy, gz must be in units of radians/second
//
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat)
{
  // Vector to hold integral error for Mahony method
  static float eInt[3] = {0.0, 0.0, 0.0};
  // short name local variable for readability
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];
  float norm;
  float hx, hy, hz;  //observed West horizon vector W = AxM
  float ux, uy, uz, wx, wy, wz; //calculated A (Up) and W in body frame
  float ex, ey, ez;

  // Auxiliary variables to avoid repeated arithmetic
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;

  // Measured horizon vector = a x m (in body frame)
  hx = ay * mz - az * my;
  hy = az * mx - ax * mz;
  hz = ax * my - ay * mx;
  // Normalise horizon vector
  norm = (hx * hx + hy * hy + hz * hz);
  if (norm == 0.0f) return; // Handle div by zero, which is unlikely

  norm = 1.0f / sqrt(norm);
  hx *= norm;
  hy *= norm;
  hz *= norm;

  // Estimated direction of Up reference vector
  ux = 2.0f * (q2q4 - q1q3);
  uy = 2.0f * (q1q2 + q3q4);
  uz = q1q1 - q2q2 - q3q3 + q4q4;

  // estimated direction of horizon (West) reference vector
  wx = 2.0f * (q2q3 + q1q4);
  wy = q1q1 - q2q2 + q3q3 - q4q4;
  wz = 2.0f * (q3q4 - q1q2);

  // Error is the summed cross products of estimated and measured directions of the reference vectors
  // It is assumed small, so sin(theta) ~ theta IS the angle required to correct the orientation error.

  ex = (ay * uz - az * uy) + (hy * wz - hz * wy);
  ey = (az * ux - ax * uz) + (hz * wx - hx * wz);
  ez = (ax * uy - ay * ux) + (hx * wy - hy * wx);
  eMag = ex * ex + ey * ey + ez * ez; //debug, squared magnitude of error vector
  
  if (Ki > 0.0f)
  {
    eInt[0] += ex;      // accumulate integral error
    eInt[1] += ey;
    eInt[2] += ez;
    // Apply I feedback
    gx += Ki * eInt[0]*deltat;
    gy += Ki * eInt[1]*deltat;
    gz += Ki * eInt[2]*deltat;
  }


  // Apply P feedback
  gx = gx + Kp * ex;
  gy = gy + Kp * ey;
  gz = gz + Kp * ez;


  //update quaternion with integrated contribution
  // small correction 1/11/2022, see https://github.com/kriswiner/MPU9250/issues/447
  gx = gx * (0.5 * deltat); // pre-multiply common factors
  gy = gy * (0.5 * deltat);
  gz = gz * (0.5 * deltat);
  float qa = q1;
  float qb = q2;
  float qc = q3;
  q1 += (-qb * gx - qc * gy - q4 * gz);
  q2 += (qa * gx + qc * gz - q4 * gy);
  q3 += (qa * gy - qb * gz + q4 * gx);
  q4 += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
  norm = 1.0f / norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;
}
