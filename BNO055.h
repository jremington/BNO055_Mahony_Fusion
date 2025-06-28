// I2C address, register and symbol definitions for BNO055
//  I2C Slave Addresses
#define BNO055_A0             (0x28)  //Arduino 7 bit addresses
#define BNO055_A1             (0x29)

#define BNO055_CHIP_ID              0xa0

//  Register map
// page 0
#define BNO055_WHO_AM_I             0x00
#define BNO055_SW_ID_L        0x04
#define BNO055_SW_ID_H        0x05
#define BNO055_PAGE_ID              0x07
#define BNO055_ACCEL_DATA           0x08
#define BNO055_MAG_DATA             0x0e
#define BNO055_GYRO_DATA            0x14
#define BNO055_FUSED_EULER          0x1a
#define BNO055_FUSED_QUAT           0x20
#define BNO055_TEMP         0x34
#define BNO055_CALIB_STAT     0x35
#define BNO055_ST_RESULT      0x36
#define BNO055_SYS_STATUS     0x39
#define BNO055_SYS_CLK_STATUS   0x38
#define BNO055_SYS_ERR        0x3a
#define BNO055_UNIT_SEL             0x3b
#define BNO055_OPER_MODE            0x3d
#define BNO055_PWR_MODE             0x3e
#define BNO055_SYS_TRIGGER          0x3f
#define BNO055_AXIS_MAP_CONFIG      0x41
#define BNO055_AXIS_MAP_SIGN        0x42
#define BNO055_CAL_DATA             0x55  //11 16 bit integers, offsets and scale

// page 1  (set PAGE_ID = 1) (regs 0-6 not used)
#define BNO055_ACC_CONFIG    0x08   //default 0x0D  (from data sheet)
#define BNO055_MAG_CONFIG    0x09   //default 0x6D
#define BNO055_GYRO_CONFIG_0  0x0A   //default 0x38
#define BNO055_GYRO_CONFIG_1  0x0B   //default 0x00

//  Operation modes
#define BNO055_OPER_MODE_CONFIG     0x00
#define BNO055_OPER_MODE_FMC_OFF    0x0b
#define BNO055_OPER_MODE_NDOF       0x0c
#define BNO055_OPER_MODE_AMG        0x07

//  Power modes
#define BNO055_PWR_MODE_NORMAL      0x00
