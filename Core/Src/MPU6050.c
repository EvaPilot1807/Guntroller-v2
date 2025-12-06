#include "MPU6050.h"
#define MPU6050_ADDR (0x68 << 1)

#define SMPLRT_DIV_REG     0x19
#define GYRO_CONFIG_REG    0x1B
#define ACCEL_CONFIG_REG   0x1C
#define ACCEL_XOUT_H_REG   0x3B
#define TEMP_OUT_H_REG     0x41
#define GYRO_XOUT_H_REG    0x43
#define PWR_MGMT_1_REG     0x6B
#define WHO_AM_I_REG       0x75

int16_t Accel_X_RAW, Accel_Y_RAW, Accel_Z_RAW;
int16_t Gyro_X_RAW, Gyro_Y_RAW, Gyro_Z_RAW;

extern I2C_HandleTypeDef hi2c1;

uint8_t DMA_Buffer[14];

float Ax, Ay, Az, Gx, Gy, Gz;


void MPU6050_init(void)
{
    uint8_t check, data;

    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 1000);

    if (check == 104)
    {
        data = 0;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &data, 1, 1000);
        HAL_Delay(100);

        data = 0x07;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &data, 1, 1000);

        data = 0x00;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &data, 1, 1000);

        data = 0x00;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &data, 1, 1000);

        data = 0x03;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1A, 1, &data, 1, 1000);

    }
}

void MPU6050_Read_Values(void)
{

    HAL_I2C_Mem_Read_DMA(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, DMA_Buffer, 14);

}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1)
    {
    Accel_X_RAW = (int16_t)(DMA_Buffer[0] << 8 | DMA_Buffer[1]);
    Accel_Y_RAW = (int16_t)(DMA_Buffer[2] << 8 | DMA_Buffer[3]);
    Accel_Z_RAW = (int16_t)(DMA_Buffer[4] << 8 | DMA_Buffer[5]);
    Gyro_X_RAW = (int16_t)(DMA_Buffer[8] << 8 | DMA_Buffer[9]);
    Gyro_Y_RAW = (int16_t)(DMA_Buffer[10] << 8 | DMA_Buffer[11]);
    Gyro_Z_RAW = (int16_t)(DMA_Buffer[12] << 8 | DMA_Buffer[13]);

    Ax = Accel_X_RAW * 100.0 / 16384.0;
    Ay = Accel_Y_RAW * 100.0 / 16384.0;
    Az = Accel_Z_RAW * 100.0 / 16384.0;
    Gx = Gyro_X_RAW / 131.0;
    Gy = Gyro_Y_RAW / 131.0;
    Gz = Gyro_Z_RAW / 131.0;
    }
}


