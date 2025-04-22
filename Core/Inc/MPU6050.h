/*
 * MPU6050.h
 *
 *  Created on: Apr 22, 2025
 *      Author: Furkan
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "main.h"

#define MPU6050_WHO_I_AM			0x68

#define MPU6050_AUX_VDDIO			0x01
#define MPU6050_SMPLRT_DIV			0x19
#define MPU6050_CONFIG				0x1A
#define MPU6050_GYRO_CONFIG			0x1B
#define MPU6050_ACCEL_CONFIG		0x1C
#define MPU6050_MOTION_THRESH		0x1F
#define MPU6050_INT_PIN_CFG			0x37
#define MPU6050_INT_ENABLE			0x38
#define MPU6050_INT_STATUS			0x3A
#define MPU6050_ACCEL_XOUT_H		0x3B
#define MPU6050_ACCEL_XOUT_L		0x3C
#define MPU6050_ACCEL_YOUT_H		0x3D
#define MPU6050_ACCEL_YOUT_L		0x3E
#define MPU6050_ACCEL_ZOUT_H		0x3F
#define MPU6050_ACCEL_ZOUT_L		0x40
#define MPU6050_TEMP_OUT_H			0x41
#define MPU6050_TEMP_OUT_L			0x42
#define MPU6050_GYRO_XOUT_H			0x43
#define MPU6050_GYRO_XOUT_L			0x44
#define MPU6050_GYRO_YOUT_H			0x45
#define MPU6050_GYRO_YOUT_L			0x46
#define MPU6050_GYRO_ZOUT_H			0x47
#define MPU6050_GYRO_ZOUT_L			0x48
#define MPU6050_SIGNAL_PATH_RESET	0x68
#define MPU6050_MOT_DETECT_CTRL		0x69
#define MPU6050_USER_CTRL			0x6A
#define MPU6050_PWR_MGMT_1			0x6B
#define MPU6050_PWR_MGMT_2			0x6C
#define MPU6050_FIFO_COUNTH			0x72
#define MPU6050_FIFO_COUNTL			0x73
#define MPU6050_FIFO_R_W			0x74
#define MPU6050_WHO_AM_I			0x75

/* Gyroscope sens (degrees/s) */
#define MPU6050_GYRO_SENS_250S			((float) 131)
#define MPU6050_GYRO_SENS_500S			((float) 65.5)
#define MPU6050_GYRO_SENS_1000S			((float) 32.8)
#define MPU6050_GYRO_SENS_2000S			((float) 16.4)

/* Accelerometer sens (g/s) */
#define MPU6050_ACCEL_SENS_2G			((float) 16384)
#define MPU6050_ACCEL_SENS_4G			((float) 8192)
#define MPU6050_ACCEL_SENS_8G			((float) 4096)
#define MPU6050_ACCEL_SENS_16G			((float) 2048)

#define MPU6050_SAMPLE_RATE_100HZ				79
#define MPU6050_SAMPLE_RATE_125HZ				63
#define MPU6050_SAMPLE_RATE_250HZ				31
#define MPU6050_SAMPLE_RATE_500HZ				15
#define MPU6050_SAMPLE_RATE_1KZ					7
#define MPU6050_SAMPLE_RATE_2KZ					3
#define MPU6050_SAMPLE_RATE_4KHZ				1
#define MPU6050_SAMPLE_RATE_8KHZ				0

typedef enum {
	MPU6050_OK,
	MPU6050_ERROR,
	MPU6050_DEVICE_NOT_CONNECTED,
	MPU6050_INVALID_DEVICE,
	MPU6050_EMPTY_HANDLE
}mpu6050_status_t;

typedef enum {
	MPU6050_DEVICE_0  = 0xD2,
	MPU6050_DEVICE_1  = 0xD0
}mpu6050_device_addr_t;

typedef enum {
	GYRO_250S  = 0x00,
	GYRO_500S  = 0x01,
	GYRO_1000S = 0x02,
	GYRO_2000S = 0x03
}mpu6050_gyroscope_range_t;

typedef enum {
	ACCEL_2G  = 0x00,
	ACCEL_4G  = 0x01,
	ACCEL_8G  = 0x02,
	ACCEL_16G = 0x03
}mpu6050_accelerometer_range_t;

typedef struct {
	uint8_t DataReady;
	uint8_t I2C_Master_Ready;
	uint8_t FIFO_Overflow;
	uint8_t MotionDetected;
}mpu6050_interrupt_t;

typedef struct {
	I2C_HandleTypeDef *hi2c;
	uint8_t device_addr;
	float temperature;
	float gyro_factor;
	float accel_factor;
	int16_t accelerometer_x;
	int16_t accelerometer_y;
	int16_t accelerometer_z;
	int16_t gyroscope_x;
	int16_t gyroscope_y;
	int16_t gyroscope_z;
}mpu6050_t;

mpu6050_status_t mpu6050_init(mpu6050_t *hmpu6050, mpu6050_device_addr_t device_addr, mpu6050_accelerometer_range_t accel_sens, mpu6050_gyroscope_range_t gyro_sens);
mpu6050_status_t mpu6050_set_gyroscope_sensitivity(mpu6050_t *hmpu6050, mpu6050_gyroscope_range_t gyro_sens);
mpu6050_status_t mpu6050_set_accelerometer_sensitivity(mpu6050_t *hmpu6050, mpu6050_accelerometer_range_t accel_sens);
mpu6050_status_t mpu6050_set_sample_rate(mpu6050_t *hmpu6050, uint8_t sample_rate);
mpu6050_status_t mpu6050_enable_interrupts(mpu6050_t *hmpu6050);
mpu6050_status_t mpu6050_disable_interrupts(mpu6050_t *hmpu6050);
mpu6050_status_t mpu6050_get_interrupts(mpu6050_t *hmpu6050, mpu6050_interrupt_t *mpu6050_irq);
mpu6050_status_t mpu6050_read_gyroscope(mpu6050_t *hmpu6050);
mpu6050_status_t mpu6050_read_accelerometer(mpu6050_t *hmpu6050);
mpu6050_status_t mpu6050_read_temperature(mpu6050_t *hmpu6050);
mpu6050_status_t mpu6050_read_all(mpu6050_t *hmpu6050);

#endif /* INC_MPU6050_H_ */
