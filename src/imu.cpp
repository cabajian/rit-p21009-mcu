/*
 * File    : imu.cpp
 * Author  : Chris Abajian (cxa6282@rit.edu)
 * Created : September 7, 2021
 *
 * Description:
 *   Handles initialization and data retrieval of the IMU sensors.
 */

#include "imu.h"
#include "Adafruit_Sensor.h"
#include "i2c.h"

static double acc_sens = 0;
static double gyr_sens = 0;
static char data[14];

/**
 * Initializes the IMU at the specified I2C address.
 */
int IMU_init(uint8_t addr) {
    /* Initialize the LSM6DSOX */
    // CTRL3_C (Control register 3)
    //   set self-clearing reset bit
    I2C_setBits(addr, LSM6DSOX_CTRL3_C_ADDR, 0x01);
    I2C_read(addr, LSM6DSOX_CTRL3_C_ADDR, data, 1);
    int iters = 100;
    while(iters-- > 0) {
        // Read until reset bit clears
        I2C_read(addr, LSM6DSOX_CTRL3_C_ADDR, data, 1);
        if (data[0] & 0x01) {
            break;
        }
        wait_us(100);
    }
    if (iters == 0) {
        // Failure.
        return 0;
    }
    // CTRL3_C (Control register 3)
    //   set BDU to only update output regs after reading
    I2C_setBits(addr, LSM6DSOX_CTRL3_C_ADDR, 0x40);
    /* Accelerometer config */
    // CTRL1_XL (Accelerometer control register 1)
    //   416Hz, +-8g
    I2C_write(addr, LSM6DSOX_CTRL1_XL_ADDR, 0x6A);
    acc_sens = 0.122; // 0.244 milli-g per bit in +-8g scale
    // CTRL8_XL (Control register 8)
    //   clear XL_FS_MODE to retain full +-16g scale
    I2C_clearBits(addr, LSM6DSOX_CTRL8_XL_ADDR, 0x02);
    // CTRL9_XL (Control register 9)
    //   set "I3C disable" bit since we aren't using it
    I2C_setBits(addr, LSM6DSOX_CTRL9_XL_ADDR, 0x01);
    /* Gyroscope config */
    // CTRL2_G (Gyroscope control register 2)
    //   416Hz, 2000dps
    I2C_write(addr, LSM6DSOX_CTRL2_G_ADDR, 0x6C);
    gyr_sens = 70.0; // 70.0 milli-dps per bit in 2000dps scale
    // Success
    return 1;
}

void IMU_zero(DeviceInstance* sensor) {
    // Obtain 5 acceleration samples.
    double data[3] = {0};
    double tmp[3];
    for (int i = 0; i < 5; i++) {
        IMU_collect(sensor, tmp);
        data[0] += tmp[0];
        data[1] += tmp[1];
        data[2] += tmp[2];
    }
    // Save the average offset (inverse of the reading).
    sensor->offsets[0] = data[0] / (-5);
    sensor->offsets[1] = data[1] / (-5);
    sensor->offsets[2] = data[2] / (-5);
}

bool IMU_collect(DeviceInstance* sensor, double* data) {
    // Get correct IMU (LSM6DSOX) I2C address.
    uint8_t addr;
    if (sensor->loc == LEG_LEFT)
        addr = LSM6DSOX_ADDR_A;
    else if (sensor->loc == LEG_RIGHT)
        addr = LSM6DSOX_ADDR_B;
    else
        return false;
    // Retrieve acceleration or gyroscope vector and store.
    if (sensor->func == ACCELERATION)
        IMU_get_accel(addr, &data[0], &data[1], &data[2]);
    else if (sensor->func == GYROSCOPE)
        IMU_get_gyro(addr, &data[0], &data[1], &data[2]);
    else
        return false;

    return true;
}

/**
 * Gets the temperature from the IMU at the specified address.
 */
void IMU_get_temp(uint8_t addr, double *temp) {
    I2C_read(addr, LSM6DSOX_OUT_TEMP_L_ADDR, data, 2);
    int16_t temp_i = (data[1] << 8 | data[0]);
    *temp = (temp_i/256.0) + 25.0;
}

/**
 * Gets the gyroscope vector from the IMU at the specified address.
 */
void IMU_get_gyro(uint8_t addr, double *x, double *y, double *z) {
    I2C_read(addr, LSM6DSOX_OUTX_L_G_ADDR, data, 6);
    int16_t gyr_ix = (data[1] << 8 | data[0]);
    int16_t gyr_iy = (data[3] << 8 | data[2]);
    int16_t gyr_iz = (data[5] << 8 | data[4]);
    *x = gyr_ix * gyr_sens * SENSORS_DPS_TO_RADS / 1000;
    *y = gyr_iy * gyr_sens * SENSORS_DPS_TO_RADS / 1000;
    *z = gyr_iz * gyr_sens * SENSORS_DPS_TO_RADS / 1000;
}

/**
 * Gets the acceleration vector from the IMU at the specified address.
 */
void IMU_get_accel(uint8_t addr, double *x, double *y, double *z) {
    I2C_read(addr, LSM6DSOX_OUTX_L_A_ADDR, data, 6);
    int16_t acc_ix = (data[1] << 8 | data[0]);
    int16_t acc_iy = (data[3] << 8 | data[2]);
    int16_t acc_iz = (data[5] << 8 | data[4]);
    *x = acc_ix * acc_sens * SENSORS_GRAVITY_STANDARD / 1000;
    *y = acc_iy * acc_sens * SENSORS_GRAVITY_STANDARD / 1000;
    *z = acc_iz * acc_sens * SENSORS_GRAVITY_STANDARD / 1000;
}
