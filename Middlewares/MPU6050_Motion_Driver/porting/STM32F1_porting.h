
#ifndef MPU6050_MOTION_DRIVER_PORTING_STM32F1_PORTING_H_
#define MPU6050_MOTION_DRIVER_PORTING_STM32F1_PORTING_H_

#include "i2c.h"
#include "main.h"

#define MPU6050_I2C_Handle hi2c1 //硬件I2C结构体

#define I2Cx_FLAG_TIMEOUT ((uint32_t)1000) // 超时时间 Timeout duration
#define I2Cx_LONG_TIMEOUT ((uint32_t)(300 * I2Cx_FLAG_TIMEOUT))

int Sensors_I2C_ReadRegister(unsigned char slave_addr, unsigned char reg_addr, unsigned short len,
                             unsigned char *data_ptr);
int Sensors_I2C_WriteRegister(unsigned char slave_addr, unsigned char reg_addr, unsigned short len,
                              unsigned char *data_ptr);

void get_ms_user(unsigned long *count);

#endif /* MPU6050_MOTION_DRIVER_PORTING_STM32F1_PORTING_H_ */
