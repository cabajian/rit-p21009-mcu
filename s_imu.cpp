/*
 * File    : s_imu.cpp
 * Author  : Chris Abajian (cxa6282@rit.edu)
 * Created : September 7, 2021
 *
 * Description:
 *   Handles initialization and data retrieval of the IMU
 *   sensors on the test mannequin.
 */

#include "msd.h"

double acc_sens = 0;
double gyr_sens = 0;
char data[14];


int imu_init(uint8_t addr) {
    /* Initialize the LSM6DSOX */
    // CTRL3_C (Control register 3)
    //   set self-clearing reset bit
    setBits(addr, LSM6DSOX_CTRL3_C_ADDR, 0x01);
    readReg(addr, LSM6DSOX_CTRL3_C_ADDR, data, 1);
    while(data[0] & 0x01) {
        wait_us(100);
        // Read until reset bit clears
        readReg(addr, LSM6DSOX_CTRL3_C_ADDR, data, 1);
    }
    // CTRL3_C (Control register 3)
    //   set BDU to only update output regs after reading
    setBits(addr, LSM6DSOX_CTRL3_C_ADDR, 0x40);
    /* Accelerometer config */
    // CTRL1_XL (Accelerometer control register 1)
    //   416Hz, +-8g
    writeReg(addr, LSM6DSOX_CTRL1_XL_ADDR, 0x6A);
    acc_sens = 0.244; // 0.244 milli-g per bit in +-8g scale
    // CTRL8_XL (Control register 8)
    //   clear XL_FS_MODE to retain full +-16g scale
    clearBits(addr, LSM6DSOX_CTRL8_XL_ADDR, 0x02);
    // CTRL9_XL (Control register 9)
    //   set "I3C disable" bit since we aren't using it
    setBits(addr, LSM6DSOX_CTRL9_XL_ADDR, 0x01);
    /* Gyroscope config */
    // CTRL2_G (Gyroscope control register 2)
    //   416Hz, 2000dps
    writeReg(addr, LSM6DSOX_CTRL2_G_ADDR, 0x6C);
    gyr_sens = 70.0; // 70.0 milli-dps per bit in 2000dps scale
    printf("IMU initialized\r\n");
    // Success
    return 1;
}

void imu_get_temp(uint8_t addr, double *temp) {
    readReg(addr, LSM6DSOX_OUT_TEMP_L_ADDR, data, 2);
    int16_t temp_i = (data[1] << 8 | data[0]);
    *temp = (temp_i/256.0) + 25.0;
    // printf("Temp: %f\n", temp_d);
}

void imu_get_gyro(uint8_t addr, double *x, double *y, double *z) {
    readReg(addr, LSM6DSOX_OUTX_L_G_ADDR, data, 6);
    int16_t gyr_ix = (data[1] << 8 | data[0]);
    int16_t gyr_iy = (data[3] << 8 | data[2]);
    int16_t gyr_iz = (data[5] << 8 | data[4]);
    *x = gyr_ix * gyr_sens * SENSORS_DPS_TO_RADS / 1000;
    *y = gyr_iy * gyr_sens * SENSORS_DPS_TO_RADS / 1000;
    *z = gyr_iz * gyr_sens * SENSORS_DPS_TO_RADS / 1000;
    // printf("Gyro\tX: %f Y: %f Z: %f\n",   gyr_dx, gyr_dy, gyr_dz);
}

void imu_get_accel(uint8_t addr, double *x, double *y, double *z) {
    readReg(addr, LSM6DSOX_OUTX_L_A_ADDR, data, 6);
    int16_t acc_ix = (data[1] << 8 | data[0]);
    int16_t acc_iy = (data[3] << 8 | data[2]);
    int16_t acc_iz = (data[5] << 8 | data[4]);
    *x = acc_ix * acc_sens * SENSORS_GRAVITY_STANDARD / 1000;
    *y = acc_iy * acc_sens * SENSORS_GRAVITY_STANDARD / 1000;
    *z = acc_iz * acc_sens * SENSORS_GRAVITY_STANDARD / 1000;
    // printf("Accel\tX: %f Y: %f Z: %f\n",   acc_dx, acc_dy, acc_dz);
}
