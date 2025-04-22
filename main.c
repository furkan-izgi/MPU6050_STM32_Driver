#include "main.h"
#include "MPU6050.h"

I2C_HandleTypeDef hi2c1;
mpu6050_t mpu;

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_I2C1_Init();

    mpu.hi2c = &hi2c1;
    if (mpu6050_init(&mpu, MPU6050_DEVICE_1, ACCEL_2G, GYRO_250S) != MPU6050_OK) {
        // Handle initialization failure
        Error_Handler();
    }

    while (1) {
        mpu6050_read_all(&mpu);

        float acc_x = mpu.accelerometer_x * mpu.accel_factor;
        float gyro_z = mpu.gyroscope_z * mpu.gyro_factor;
        float temp = mpu.temperature;

        HAL_Delay(500);
    }
}
