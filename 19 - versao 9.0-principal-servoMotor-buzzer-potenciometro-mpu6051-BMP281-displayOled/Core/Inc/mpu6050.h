//Declarações de funções, variáveis e macros. (Cabeçalho da biblioteca do mpu6050)

#ifndef MPU6050_H
#define MPU6050_H

#include "stm32f4xx_hal.h" // Ou o arquivo correto da HAL

// Endereço padrão do MPU6050
#define MPU6050_ADDR 0xD0

// Registradores do MPU6050
#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75

// Estrutura para armazenar os dados do sensor
typedef struct {
    float Ax, Ay, Az; // Aceleração em g
    float Gx, Gy, Gz; // Velocidade angular em °/s
} MPU6050_Data;

// Protótipos das funções públicas
void MPU6050_Init(I2C_HandleTypeDef *hi2c);
void MPU6050_Read_Accel(I2C_HandleTypeDef *hi2c, MPU6050_Data *data);
void MPU6050_Read_Gyro(I2C_HandleTypeDef *hi2c, MPU6050_Data *data);

#endif /* MPU6050_H */
