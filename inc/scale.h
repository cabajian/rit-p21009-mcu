/*
 * File    : scale.cpp
 * Author  : Chris Abajian (cxa6282@rit.edu)
 * Created : November 11, 2021
 *
 * Description:
 *   Scale header file.
 */

#pragma once

#ifndef __SCALE_INCLUDE_H__
#define __SCALE_INCLUDE_H__

#include "HX711_mbed.h"

#define SCALE_CLK               (PTC3)
#define SCALE_DATA              (PTC2)
#define SCALE_READ_ITERS        (3)
#define SCALE_CAL_ITERS         (20)
#define SCALE_CAL_TIMEOUT_MS    (10000)
#define SCALE_CAL_EPSILON       (0.05)

int scale_init(long offset = 0);
long scale_zero();
void scale_set_offset(long offset);
long scale_get_offset();
float scale_get_factor();
float scale_read();
bool scale_calibrate(float pounds);

#endif
