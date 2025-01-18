/*
 * mpu6050.h
 *
 *  Created on: Jan 13, 2025
 *      Author: viorel_serbu
 */

#ifndef MAIN_MPU6050_H_
#define MAIN_MPU6050_H_

#define MPU_I2C_ADDRESS		0x68
#define MPU_NOT_INIT		0
#define MPU_INIT			1
#define MPU_ACTIVE			2

#define DEFAULT_GFS			1		// default gyro full scale = +-500 deg/sec
#define DEFAULT_AFS			0		// default accel full scale = +-2g

#define XGOFFS_TC			0x00 // Bit 7 PWR_MODE, bits 6:1 XG_OFFS_TC, bit 0 OTP_BNK_VLD
#define YGOFFS_TC			0x01
#define ZGOFFS_TC			0x02
#define X_FINE_GAIN			0x03 // [7:0] fine gain
#define Y_FINE_GAIN			0x04
#define Z_FINE_GAIN			0x05
#define XA_OFFSET_H			0x06 // User-defined trim values for accelerometer
#define XA_OFFSET_L_TC		0x07
#define YA_OFFSET_H			0x08
#define YA_OFFSET_L_TC		0x09
#define ZA_OFFSET_H			0x0A
#define ZA_OFFSET_L_TC		0x0B

#define SELF_TEST_X			0x0D
#define SELF_TEST_Y			0x0E
#define SELF_TEST_Z			0x0F
#define SELF_TEST_A			0x10

#define SMPLRT_DIV			0x19
#define CONFIG				0x1A
#define GYRO_CONFIG			0x1B
#define ACCEL_CONFIG		0x1C

#define FIFO_EN				0x23
#define I2C_MST_CTRL		0x24

#define I2C_MST_STATUS		0x36
#define INT_PIN_CFG			0x37
#define INT_ENABLE			0x38
#define DMP_INT_STATUS		0x39  // Check DMP interrupt
#define INT_STATUS			0x3A
#define ACCEL_XOUT_H		0x3B
#define ACCEL_XOUT_L		0x3C
#define ACCEL_YOUT_H		0x3D
#define ACCEL_YOUT_L		0x3E
#define ACCEL_ZOUT_H		0x3F
#define ACCEL_ZOUT_L		0x40
#define TEMP_OUT_H			0x41
#define TEMP_OUT_L			0x42
#define GYRO_XOUT_H			0x43
#define GYRO_XOUT_L			0x44
#define GYRO_YOUT_H			0x45
#define GYRO_YOUT_L			0x46
#define GYRO_ZOUT_H			0x47
#define GYRO_ZOUT_L			0x48

#define SIGNAL_PATH_RESET	0x68
#define MOT_DETECT_CTRL		0x69
#define USER_CTRL			0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1			0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2			0x6C
#define DMP_BANK			0x6D  // Activates a specific bank in the DMP
#define DMP_RW_PNT			0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG				0x6F  // Register in DMP from which to read or to which to write
#define DMP_REG_1			0x70
#define DMP_REG_2			0x71
#define FIFO_COUNTH			0x72
#define FIFO_COUNTL			0x73
#define FIFO_R_W			0x74
#define WHO_AM_I		 	0x75 // Should return 0x68

#define TO_INT(a, b)		(int16_t)(((int16_t)a << 8) | b)

void register_mpu();
int do_mpu(int argc, char **argv);

#endif /* MAIN_MPU6050_H_ */
