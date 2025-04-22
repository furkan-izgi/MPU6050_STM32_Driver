/*
 * MPU6050.c
 *
 *  Created on: Apr 22, 2025
 *      Author: Furkan
 */

#include "MPU6050.h"

/*
 * @brief		inits the sensor driver
 * @param		*mpu6050 pointer of mpu6050_t struct
 * @param		device_addr i2c address of the device
 * @param		accel_sens sensitivity of accelerometer
 * @param		gyro_sens sensitivity of gyroscope
 * @return		mpu6050_status_t
 * 				- MPU6050_OK on success
 *				- MPU6050_ERROR on error
 */
mpu6050_status_t mpu6050_init(mpu6050_t *hmpu6050, mpu6050_device_addr_t device_addr, mpu6050_accelerometer_range_t accel_sens, mpu6050_gyroscope_range_t gyro_sens) {

	if(hmpu6050 == NULL) {
		return MPU6050_EMPTY_HANDLE;
	}

	if(hmpu6050->hi2c == NULL) {
		return MPU6050_EMPTY_HANDLE;
	}

	hmpu6050->device_addr = device_addr;

	/* Check I2C Comm. is working or not */
	if(HAL_I2C_IsDeviceReady(hmpu6050->hi2c, hmpu6050->device_addr, 1, 100) != 0) {
		return MPU6050_ERROR;
	}

	/* Check Device ID which can be accessed from register document */
	uint8_t device_id = 0;
	if(HAL_I2C_Mem_Read(hmpu6050->hi2c, hmpu6050->device_addr, MPU6050_WHO_AM_I, I2C_MEMADD_SIZE_8BIT, &device_id, sizeof(device_id), 100) != 0) {
		return MPU6050_ERROR;
	}

	if(device_id != MPU6050_WHO_I_AM) {
		return MPU6050_ERROR;
	}

	/* Set Power Management Settings of the sensor */
	if(HAL_I2C_Mem_Write(hmpu6050->hi2c, hmpu6050->device_addr, MPU6050_PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, 0x00, 1, 100) != 0) {
		return MPU6050_ERROR;
	}

	/* Set Sample Rate */
	mpu6050_set_sample_rate(hmpu6050, MPU6050_SAMPLE_RATE_2KZ);

	/* Set Accelerometer and Gyroscope Settings */
	mpu6050_set_gyroscope_sensitivity(hmpu6050, gyro_sens);
	mpu6050_set_accelerometer_sensitivity(hmpu6050, accel_sens);

	return MPU6050_OK;
}

/*
 * @brief		sets the gyroscope sensitivity of sensor
 * @param		*mpu6050 pointer of mpu6050_t struct
 * @param		gyro_sens sensitivity of gyroscope
 * @return		mpu6050_status_t
 * 				- MPU6050_OK on success
 *				- MPU6050_ERROR on error
 */
mpu6050_status_t mpu6050_set_gyroscope_sensitivity(mpu6050_t *hmpu6050, mpu6050_gyroscope_range_t gyro_sens) {
	uint8_t gyro_conf = 0;

	/* Read Current Config */
	if(HAL_I2C_Mem_Read(hmpu6050->hi2c, hmpu6050->device_addr, MPU6050_GYRO_CONFIG, I2C_MEMADD_SIZE_8BIT, &gyro_conf, 1, 100) != 0) {
		return MPU6050_ERROR;
	}

	/* Clear Sesnsivity Bits */
	gyro_conf &= 0xE7;

	/* Set New Gyroscope Sensivitiy to Config */
	gyro_conf |= ((uint8_t)gyro_sens) << 3;

	/* Write New Config to Sensor */
	if(HAL_I2C_Mem_Write(hmpu6050->hi2c, hmpu6050->device_addr, MPU6050_GYRO_CONFIG, I2C_MEMADD_SIZE_8BIT, &gyro_conf, 1, 100) != 0) {
		return MPU6050_ERROR;
	}

	/* Calculate the Accelerometer Factor */
	switch (gyro_sens) {
		case GYRO_250S:
			hmpu6050->gyro_factor = (float)1/MPU6050_GYRO_SENS_250S;
			break;
		case GYRO_500S:
			hmpu6050->gyro_factor = (float)1/MPU6050_GYRO_SENS_500S;
			break;
		case GYRO_1000S:
			hmpu6050->gyro_factor = (float)1/MPU6050_GYRO_SENS_1000S;
			break;
		case GYRO_2000S:
			hmpu6050->gyro_factor = (float)1/MPU6050_GYRO_SENS_2000S;
			break;
		default:
			break;
	}

	return MPU6050_OK;
}

/*
 * @brief		sets the accelerometer sensitivity of sensor
 * @param		*mpu6050 pointer of mpu6050_t struct
 * @param		accel_sens sensitivity of accelerometer
 * @return		mpu6050_status_t
 * 				- MPU6050_OK on success
 *				- MPU6050_ERROR on error
 */
mpu6050_status_t mpu6050_set_accelerometer_sensitivity(mpu6050_t *hmpu6050, mpu6050_accelerometer_range_t accel_sens) {
	uint8_t accel_conf = 0;

	/* Read Current Config */
	if(HAL_I2C_Mem_Read(hmpu6050->hi2c, hmpu6050->device_addr, MPU6050_ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT, &accel_conf, 1, 100) != 0) {
		return MPU6050_ERROR;
	}

	/* Clear Sesnsivity Bits */
	accel_conf &= 0xE7;

	/* Set New Gyroscope Sensitivity to Config */
	accel_conf |= ((uint8_t)accel_sens) << 3;

	/* Write New Config to Sensor */
	if(HAL_I2C_Mem_Write(hmpu6050->hi2c, hmpu6050->device_addr, MPU6050_ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT, &accel_conf, 1, 100) != 0) {
		return MPU6050_ERROR;
	}

	/* Calculate the Accelerometer Factor */
	switch (accel_sens) {
		case ACCEL_2G:
			hmpu6050->accel_factor = (float)1/MPU6050_ACCEL_SENS_2G;
			break;
		case ACCEL_4G:
			hmpu6050->accel_factor = (float)1/MPU6050_ACCEL_SENS_4G;
			break;
		case ACCEL_8G:
			hmpu6050->accel_factor = (float)1/MPU6050_ACCEL_SENS_8G;
			break;
		case ACCEL_16G:
			hmpu6050->accel_factor = (float)1/MPU6050_ACCEL_SENS_16G;
			break;
		default:
			break;
	}

	return MPU6050_OK;
}

/*
 * @brief		sets the sample rate of sensor
 * @param		*mpu6050 pointer of mpu6050_t struct
 * @param		sample_rate user's sample rate preference
 * @return		mpu6050_status_t
 * 				- MPU6050_OK on success
 *				- MPU6050_ERROR on error
 */
mpu6050_status_t mpu6050_set_sample_rate(mpu6050_t *hmpu6050, uint8_t sample_rate) {
	/* Write New Sample Rate to Sensor */
	if(HAL_I2C_Mem_Write(hmpu6050->hi2c, hmpu6050->device_addr, MPU6050_SMPLRT_DIV, I2C_MEMADD_SIZE_8BIT, &sample_rate, 1, 100) != 0) {
		return MPU6050_ERROR;
	}
	return MPU6050_OK;
}

/*
 * @brief		enables the interrupts of sensor
 * @param		*mpu6050 pointer of mpu6050_t struct
 * @return		mpu6050_status_t
 * 				- MPU6050_OK on success
 *				- MPU6050_ERROR on error
 */
mpu6050_status_t mpu6050_enable_interrupts(mpu6050_t *hmpu6050) {
	uint8_t enabled_irq = 0x41;
	uint8_t irq_pin_conf = 0;

	/* Enable Data Ready and Motion Detection Interrupts */
	if(HAL_I2C_Mem_Write(hmpu6050->hi2c, hmpu6050->device_addr, MPU6050_INT_ENABLE, I2C_MEMADD_SIZE_8BIT, &enabled_irq, 1, 100) != 0) {
		return MPU6050_ERROR;
	}

	/* Clear Interrupt Status Bits */
	if(HAL_I2C_Mem_Read(hmpu6050->hi2c, hmpu6050->device_addr, MPU6050_INT_PIN_CFG, I2C_MEMADD_SIZE_8BIT, &irq_pin_conf, 1, 100) != 0) {
		return MPU6050_ERROR;
	}

	/* Activate INT_RD_CLEAR pin */
	irq_pin_conf |= 0x10;

	if(HAL_I2C_Mem_Write(hmpu6050->hi2c, hmpu6050->device_addr, MPU6050_INT_PIN_CFG, I2C_MEMADD_SIZE_8BIT, &irq_pin_conf, 1, 100) != 0) {
		return MPU6050_ERROR;
	}

	return MPU6050_OK;
}

/*
 * @brief		disables the interrupts of sensor
 * @param		*mpu6050 pointer of mpu6050_t struct
 * @return		mpu6050_status_t
 * 				- MPU6050_OK on success
 *				- MPU6050_ERROR on error
 */
mpu6050_status_t mpu6050_disable_interrupts(mpu6050_t *hmpu6050) {
	uint8_t disable_irq = 0x00;

	/* Disable All Interrupts */
	if(HAL_I2C_Mem_Write(hmpu6050->hi2c, hmpu6050->device_addr, MPU6050_INT_ENABLE, I2C_MEMADD_SIZE_8BIT, &disable_irq, 1, 100) != 0) {
		return MPU6050_ERROR;
	}

	return MPU6050_OK;
}

/*
 * @brief		gets the interrupt statues of sensor
 * @param		*mpu6050 pointer of mpu6050_t struct
 * @param		*mpu6050_irq pointer of mpu6050_interrupt_t struct
 * @return		mpu6050_status_t
 * 				- MPU6050_OK on success
 *				- MPU6050_ERROR on error
 */
mpu6050_status_t mpu6050_get_interrupts(mpu6050_t *hmpu6050, mpu6050_interrupt_t *mpu6050_irq) {
	uint8_t all_irqs = 0;

	/* Get All Interrupt Statues */
	if(HAL_I2C_Mem_Read(hmpu6050->hi2c, hmpu6050->device_addr, MPU6050_INT_STATUS, I2C_MEMADD_SIZE_8BIT, &all_irqs, 1, 100) != 0) {
		return MPU6050_ERROR;
	}

	/* Parse the Interrupt Status Byte */
	mpu6050_irq->DataReady = (all_irqs & 0x01) ? 1 : 0;
	mpu6050_irq->I2C_Master_Ready = (all_irqs & 0x08) ? 1 : 0;
	mpu6050_irq->FIFO_Overflow = (all_irqs & 0x10) ? 1 : 0;
	mpu6050_irq->MotionDetected = (all_irqs & 0x40) ? 1 : 0;

	return MPU6050_OK;
}

/*
 * @brief		reads the gyroscope's values from sensor
 * @param		*mpu6050 pointer of mpu6050_t struct
 * @return		mpu6050_status_t
 * 				- MPU6050_OK on success
 *				- MPU6050_ERROR on error
 */
mpu6050_status_t mpu6050_read_gyroscope(mpu6050_t *hmpu6050) {
	uint8_t gyro_raw_x_data[2] = {0, 0};
	uint8_t gyro_raw_y_data[2] = {0, 0};
	uint8_t gyro_raw_z_data[2] = {0, 0};

	/* Read X Axis */
	if(HAL_I2C_Mem_Read(hmpu6050->hi2c, hmpu6050->device_addr, MPU6050_GYRO_XOUT_H, I2C_MEMADD_SIZE_8BIT, gyro_raw_x_data, 2, 100) != 0) {
		return MPU6050_ERROR;
	}

	/* Read Y Axis */
	if(HAL_I2C_Mem_Read(hmpu6050->hi2c, hmpu6050->device_addr, MPU6050_GYRO_YOUT_H, I2C_MEMADD_SIZE_8BIT, gyro_raw_y_data, 2, 100) != 0) {
		return MPU6050_ERROR;
	}

	/* Read Z Axis */
	if(HAL_I2C_Mem_Read(hmpu6050->hi2c, hmpu6050->device_addr, MPU6050_GYRO_ZOUT_H, I2C_MEMADD_SIZE_8BIT, gyro_raw_z_data, 2, 100) != 0) {
		return MPU6050_ERROR;
	}

	/* Save Raw Datas to Struct */
	hmpu6050->gyroscope_x = (int16_t)gyro_raw_x_data[0] << 8 | (int16_t)gyro_raw_x_data[1];
	hmpu6050->gyroscope_y = (int16_t)gyro_raw_y_data[0] << 8 | (int16_t)gyro_raw_y_data[1];
	hmpu6050->gyroscope_z = (int16_t)gyro_raw_z_data[0] << 8 | (int16_t)gyro_raw_z_data[1];

	return MPU6050_OK;
}

/*
 * @brief		reads the accelerometer's values from sensor
 * @param		*mpu6050 pointer of mpu6050_t struct
 * @return		mpu6050_status_t
 * 				- MPU6050_OK on success
 *				- MPU6050_ERROR on error
 */
mpu6050_status_t mpu6050_read_accelerometer(mpu6050_t *hmpu6050) {
	uint8_t accel_raw_x_data[2] = {0, 0};
	uint8_t accel_raw_y_data[2] = {0, 0};
	uint8_t accel_raw_z_data[2] = {0, 0};

	/* Read X Axis */
	if(HAL_I2C_Mem_Read(hmpu6050->hi2c, hmpu6050->device_addr, MPU6050_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, accel_raw_x_data, 2, 100) != 0) {
		return MPU6050_ERROR;
	}

	/* Read Y Axis */
	if(HAL_I2C_Mem_Read(hmpu6050->hi2c, hmpu6050->device_addr, MPU6050_ACCEL_YOUT_H, I2C_MEMADD_SIZE_8BIT, accel_raw_y_data, 2, 100) != 0) {
		return MPU6050_ERROR;
	}

	/* Read Z Axis */
	if(HAL_I2C_Mem_Read(hmpu6050->hi2c, hmpu6050->device_addr, MPU6050_ACCEL_ZOUT_H, I2C_MEMADD_SIZE_8BIT, accel_raw_z_data, 2, 100) != 0) {
		return MPU6050_ERROR;
	}

	/* Save Raw Datas to Struct */
	hmpu6050->accelerometer_x = (int16_t)accel_raw_x_data[0] << 8 | (int16_t)accel_raw_x_data[1];
	hmpu6050->accelerometer_y = (int16_t)accel_raw_y_data[0] << 8 | (int16_t)accel_raw_y_data[1];
	hmpu6050->accelerometer_z = (int16_t)accel_raw_z_data[0] << 8 | (int16_t)accel_raw_z_data[1];

	return MPU6050_OK;
}

/*
 * @brief		reads the temperature value from sensor
 * @param		*mpu6050 pointer of mpu6050_t struct
 * @return		mpu6050_status_t
 * 				- MPU6050_OK on success
 *				- MPU6050_ERROR on error
 */
mpu6050_status_t mpu6050_read_temperature(mpu6050_t *hmpu6050) {
	uint8_t temp_raw_data[2] = {0, 0};
	int16_t raw_temp = 0;

	/* Read Raw Temperature */
	if(HAL_I2C_Mem_Read(hmpu6050->hi2c, hmpu6050->device_addr, MPU6050_TEMP_OUT_H, I2C_MEMADD_SIZE_8BIT, temp_raw_data, 2, 100) != 0) {
		return MPU6050_ERROR;
	}

	raw_temp = (int16_t)temp_raw_data[0] << 8 | (int16_t)temp_raw_data[1];

	/* Save the Temperature Data to Struct */
	hmpu6050->temperature = (float)(raw_temp / (float)340.0 + (float)36.53);

	return MPU6050_OK;
}

/*
 * @brief		reads the all sensor datas
 * @param		*mpu6050 pointer of mpu6050_t struct
 * @return		mpu6050_status_t
 * 				- MPU6050_OK on success
 *				- MPU6050_ERROR on error
 */
mpu6050_status_t mpu6050_read_all(mpu6050_t *hmpu6050) {
	/* Get Gyroscope's Values */
	if(mpu6050_read_gyroscope(hmpu6050) != 0) {
		return MPU6050_ERROR;
	}

	/* Get Accelerometer's Values */
	if(mpu6050_read_accelerometer(hmpu6050) != 0) {
		return MPU6050_ERROR;
	}

	if(mpu6050_read_temperature(hmpu6050) != 0) {
		return MPU6050_ERROR;
	}

	return MPU6050_OK;
}
