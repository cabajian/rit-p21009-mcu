/*
 * File    : i2c.cpp
 * Author  : Chris Abajian (cxa6282@rit.edu)
 * Created : November 9, 2021
 *
 * Description:
 *   I2C communication functions.
 */

#include "i2c.h"

static I2C i2c(I2C_SDA, I2C_SCL);

/**
 * Initializes the I2C instance.
 */
void I2C_init() {
    i2c.frequency(400000);
}

/**
 * Gets the I2C instance pointer.
 */
I2C* I2C_get() {
    return &i2c;
}

/**
 * I2C Read helper function.   
 */
void I2C_read(int address, uint8_t subaddress, char* data, int length = 1) {
    // Set register to read
    char subaddr[1] = {subaddress};
    i2c.write(address, subaddr, 1);
    // Read register
    i2c.read(address, data, length);
}

/**
 * I2C Write helper function.    
 */
void I2C_write(int address, uint8_t subaddress, uint8_t command) {
    // Write register
    char wdata[2] = {subaddress, command};
    i2c.write(address, wdata, 2);
}

/**
 * I2C Set bits function.    
 */
void I2C_setBits(int address, uint8_t subaddress, uint8_t mask) {
    char data[1];
    I2C_read(address, subaddress, data, 1);
    data[0] |= mask;
    I2C_write(address, subaddress, data[0]);
}

/**
 * I2C Clear bits function.    
 */
void I2C_clearBits(int address, uint8_t subaddress, uint8_t mask) {
    char data[1];
    I2C_read(address, subaddress, data, 1);
    data[0] &= ~mask;
    I2C_write(address, subaddress, data[0]);
}
