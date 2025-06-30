# BNO055_Mahony_Fusion

**New life for your old BNO055!**

Fusion filter replaces faulty firmware

Many users have reported the poor performance of the obsolete Bosch BNO055 Absolute Orientation Sensor, which uses proprietary firmware to compute 3D orientations from the built in 9DOF sensors. The problem appears to be largely due to the poor performance of the built in sensor calibration, which cannot be turned off.

**C/C++ Arduino** code in this repository can be used to access the raw sensor data from the accelerometer, magnetometer and gyro, which are considerably noisier than more modern replacements, but are still useful.  I include a tilt compensated compass, which reports heading, a Mahony fusion filter which reports 3D orientation, (all tested on an Arduino Uno R3) and a sensor calibration program in Python, which corrects offsets and relative axial scale factor errors in the magnetometer and accelerometer data.

To use the sensor calibration program, run the BNO055_readRaw program to collect accelerometer and magnetometer data, while rotating the sensor steadily and slowly to cover as much of the 3D sphere as possible. A terminal program can be used to capture a log file, which will be in Excel .CVS format (comma separated values): one line each of acc_x, acc_y, acc_z, mag_x, mag_y and mag_z.  

Edit this file to create two .cvs files, on each for the accelerometer data, and submit each in a separate run of calibrate3.py (edit input file name in code).

The output of calibrate3.py will be offsets and a correction matrix to be applied to subsequent raw accelerometer and magnetometer data. The correction data need to be edited and included in the source code for the fusion and tilt-compensated compass programs.

The two programs BNO055_tiltComp.ino and BNO055_Mahony.ino show how the corrections are applied.

#Example output representation of calibrated accelerometer data (automatically produced by calibrate3.py)

![bno055_acc_cal](https://github.com/user-attachments/assets/44bc021d-9d8b-48cd-ad0e-90138bdcdff9)


#NOTES on sensor coordinate system

The default x,y,z Cartesian coordinate system for the BNO055 is found from the IC package orientation.
The code assumes X points North and Z up for yaw=0, pitch and roll are about the Y and X axes respectively. 

The default axial assignments can be remapped in the code AFTER calibration corrections are applied, but be sure to maintain a right handed coordinate system. See comments in code.

![BNO055_sensor_axes](https://github.com/user-attachments/assets/29e71baa-8792-407b-90f1-b32d8829fae2)

Example output of the Mahony filter, hand held sensor.   
Euler angles shown: **yaw = blue, pitch = red, roll = green**. 

The magnetometer, which is the basis for the yaw angle measurement, is nearly a factor of ten noiser than the accelerometer. This explains the increased noise in the blue yaw trace, even when the sensor is  held still. Y scale is degrees.

![ypr](https://github.com/user-attachments/assets/ed5f563e-096e-40ac-a082-0c9a9466bd5f)

