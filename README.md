
# MPU6050 Driver for STM32 (HAL Based)

This is a lightweight driver for the MPU6050 IMU sensor using STM32 HAL I2C communication.

>NOTE: This driver has not been tested on any simulation or circuit.
However, all the logic and algorithms have been compared with those of other working drivers.

## 📦 Features

- ✅ Initialization and sensor verification
- ✅ Accelerometer and gyroscope sensitivity configuration
- ✅ Sample rate configuration
- ✅ Raw data read from accelerometer and gyroscope
- ✅ Temperature reading
- ✅ Interrupt support (motion detection, data ready, etc.)
- ✅ Easy-to-integrate `mpu6050_t` structure

## 🔧 Dependencies

- STM32Cube HAL Library
- I2C Peripheral Enabled

## 📁 File Structure

```
📂 MPU6050_STM32_Driver
├── Core/
│   ├── Inc/
│   │   └── MPU6050.h
│   └── Src/
│       └── MPU6050.c
└── main.c
```

## 🚀 Quick Start

### 1️⃣ Add Driver Files

Place `MPU6050.h` and `MPU6050.c` into your `Inc/` and `Src/` directories.

### 2️⃣ Include Header

```c
#include "MPU6050.h"
```

### 3️⃣ Example: `main.c`

```c
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
```

## 📄 License

MIT License
