/*
 * mpu6050_SL.h
 *
 *  Created on: Feb 13, 2022
 *      Author: Huffer
 */

#ifndef MPU6050_MOTION_DRIVER_PORTING_MPU6050_SL_H_
#define MPU6050_MOTION_DRIVER_PORTING_MPU6050_SL_H_

#include "main.h"

#define USE_PRINTF_DEBUG 1
#define USE_PYTHON_CLIENT 0 //官方python上位机 使用<===>1  不使用<===>0
#define MPU_SELF_TEST 0
#define MOTION (0)
#define NO_MOTION (1)

#define ACCEL_ON (0x01)
#define GYRO_ON (0x02)
#define COMPASS_ON (0x04)

/* Starting sampling rate. */
#define DEFAULT_MPU_HZ (20)

#define FLASH_SIZE (512)
#define FLASH_MEM_START ((void *)0x1800)

#define PEDO_READ_MS (1000)
#define TEMP_READ_MS (500)
#define COMPASS_READ_MS (100)

struct rx_s
{
    unsigned char header[3];
    unsigned char cmd;
};
struct hal_s
{
    unsigned char lp_accel_mode;
    unsigned char sensors;
    unsigned char dmp_on;
    unsigned char wait_for_tap;
    volatile unsigned char new_gyro;
    unsigned char motion_int_mode;
    unsigned long no_dmp_hz;
    unsigned long next_pedo_ms;
    unsigned long next_temp_ms;
    unsigned long next_compass_ms;
    unsigned int report;
    unsigned short dmp_features;
    struct rx_s rx;
};

struct platform_data_s
{
    signed char orientation[9];
};

extern struct hal_s hal;
extern struct platform_data_s gyro_pdata;
uint8_t MPU6050_mpu_init(void);
uint8_t MPU6050_mpl_init(void);
uint8_t MPU6050_config(void);
void MPU6050_data_ready_cb(void);

void run_self_test(void);
void setup_gyro(void);

#ifdef COMPASS_ENABLED
void send_status_compass();
#endif

#if USE_PYTHON_CLIENT
void android_orient_cb(unsigned char orientation);
void tap_cb(unsigned char direction, unsigned char count);
void handle_input(void);
#endif

#endif /* MPU6050_MOTION_DRIVER_PORTING_MPU6050_SL_H_ */
