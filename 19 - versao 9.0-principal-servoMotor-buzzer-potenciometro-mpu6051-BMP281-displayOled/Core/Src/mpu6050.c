//Implementação das funções (da biblioteca do mpu6050)

#include "mpu6050.h"


int16_t Accel_X_RAW = 0;
int16_t Accel_Y_RAW = 0;
int16_t Accel_Z_RAW = 0;

int16_t Gyro_X_RAW = 0;
int16_t Gyro_Y_RAW = 0;
int16_t Gyro_Z_RAW = 0;



// Função para inicializar o MPU6050
void MPU6050_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t check, Data;

    // Verifica(check) o ID do dispositivo ID WHO_AM_I
    HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 1000);
    if (check == 0x68) { // 0x68 will be returned by the sensor if everything goes well
        // Configuração inicial
    	// power management register 0X6B we should write all 0's to wake the sensor up
        Data = 0; // Wake up
        HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, 1000);

        // Set DATA RATE of 1KHz by writing SMPLRT_DIV register
        Data = 0x07; // Sample rate 1 kHz
        HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);

        // Set Gyroscopic configuration in GYRO_CONFIG Register
        Data = 0x00; // XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> ± 250 dps
        HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);

        // Set accelerometer configuration in ACCEL_CONFIG Register
        Data = 0x00; // XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> Accel ± 2g
        HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);
    }
}

// Função para ler a aceleração
void MPU6050_Read_Accel(I2C_HandleTypeDef *hi2c, MPU6050_Data *data) {
    uint8_t Rec_Data[6];
    int16_t Accel_X_RAW, Accel_Y_RAW, Accel_Z_RAW;

    // Read 6 BYTES of data starting from ACCEL_XOUT_H (0x3B) register
    HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);

    Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    /*** convert the RAW values into acceleration in 'g'
    	     we have to divide according to the Full scale value set in FS_SEL
    	     I have configured FS_SEL = 0. So I am dividing by 16384.0
    	     for more details check ACCEL_CONFIG Register              ****/

    data->Ax = (float)Accel_X_RAW / 16384.0;
    data->Ay = (float)Accel_Y_RAW / 16384.0;
    data->Az = (float)Accel_Z_RAW / 16384.0;
}

// Função para ler os valores do giroscópio
void MPU6050_Read_Gyro(I2C_HandleTypeDef *hi2c, MPU6050_Data *data) {
    uint8_t Rec_Data[6];
    int16_t Gyro_X_RAW, Gyro_Y_RAW, Gyro_Z_RAW;

    // Read 6 BYTES of data starting from GYRO_XOUT_H register
    HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);

    Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    /*** convert the RAW values into dps (ｰ/s)
    	     we have to divide according to the Full scale value set in FS_SEL
    	     I have configured FS_SEL = 0. So I am dividing by 131.0
    	     for more details check GYRO_CONFIG Register              ****/

    data->Gx = (float)Gyro_X_RAW / 131.0;
    data->Gy = (float)Gyro_Y_RAW / 131.0;
    data->Gz = (float)Gyro_Z_RAW / 131.0;
}





