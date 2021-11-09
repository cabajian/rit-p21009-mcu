/*
 * File    : i2c.h
 * Author  : Chris Abajian (cxa6282@rit.edu)
 * Created : November 9, 2021
 *
 * Description:
 *   I2C communication header.
 */

#pragma once

#ifndef __I2C_INCLUDE_H__
#define __I2C_INCLUDE_H__

#include "mbed.h"

/* I2C Read/Write Function Prototypes */
void I2C_init();
I2C* I2C_get();
void I2C_read(int address, uint8_t subaddress, char *data, int length);
void I2C_write(int address, uint8_t subaddress, uint8_t command);
void I2C_setBits(int address, uint8_t subaddress, uint8_t mask);
void I2C_clearBits(int address, uint8_t subaddress, uint8_t mask);

#endif