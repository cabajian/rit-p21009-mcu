/*
 * File    : orientation_board.h
 * Author  : Chris Abajian (cxa6282@rit.edu)
 * Created : November 11, 2021
 *
 * Description:
 *   Orientation Board header file.
 */

#pragma once

#ifndef __OB_INCLUDE_H__
#define __OB_INCLUDE_H__

#include "Adafruit_BNO055.h"

#define OB_HEAD     (0)
#define OB_BODY     (1)

int OB_init(int instance);
Adafruit_BNO055* OB_get(int instance);
void OB_get_cal(int instance, uint8_t* sys, uint8_t* gyro, uint8_t* accel, uint8_t* mag);
void OB_get_accel(int instance, double *x, double *y, double *z);
void OB_get_euler(int instance, double *x, double *y, double *z);

#endif