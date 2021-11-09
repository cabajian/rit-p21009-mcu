/*
 * File    : imu.h
 * Author  : Chris Abajian (cxa6282@rit.edu)
 * Created : November 11, 2021
 *
 * Description:
 *   IMU header file.
 */

#pragma once

#ifndef __IMU_INCLUDE_H__
#define __IMU_INCLUDE_H__

#include <stdint.h>

// LSM6DSOX Addresses.
#define LSM6DSOX_ADDR_A             (0x6A<<1)
#define LSM6DSOX_ADDR_B             (0x6B<<1)
#define LSM6DSOX_WHO_AM_I_ADDR      (0x0F)
#define LSM6DSOX_CTRL1_XL_ADDR      (0x10)
#define LSM6DSOX_CTRL2_G_ADDR       (0x11)
#define LSM6DSOX_CTRL3_C_ADDR       (0x12)
#define LSM6DSOX_CTRL8_XL_ADDR      (0x17)
#define LSM6DSOX_CTRL9_XL_ADDR      (0x18)
#define LSM6DSOX_OUT_TEMP_L_ADDR    (0x20)
#define LSM6DSOX_OUTX_L_G_ADDR      (0x22)
#define LSM6DSOX_OUTX_L_A_ADDR      (0x28)

int IMU_init(uint8_t addr);
void IMU_get_temp(uint8_t addr, double *temp);
void IMU_get_gyro(uint8_t addr, double *x, double *y, double *z);
void IMU_get_accel(uint8_t addr, double *x, double *y, double *z);

#endif