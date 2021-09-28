/*
 * File    : main.cpp
 * Author  : Chris Abajian (cxa6282@rit.edu)
 * Created : September 5, 2021
 *
 * Description:
 *   Main header for RIT MSD Team P21009 microcontroller software.
 */

#ifndef MSD_INCLUDE_H
#define MSD_INCLUDE_H

#include "Adafruit_BNO055.h"

/* LSM6DSOX macros */
#define LSM6DSOX_ADDR_A         (0x6A<<1)
#define IMU_LEG_L_ADDR          LSM6DSOX_ADDR_A
#define LSM6DSOX_ADDR_B         (0x6B<<1)
#define IMU_LEG_R_ADDR          LSM6DSOX_ADDR_B
#define LSM6DSOX_WHO_AM_I_ADDR  (0x0F)
#define LSM6DSOX_CTRL1_XL_ADDR  (0x10)
#define LSM6DSOX_CTRL2_G_ADDR   (0x11)
#define LSM6DSOX_CTRL3_C_ADDR   (0x12)
#define LSM6DSOX_CTRL8_XL_ADDR  (0x17)
#define LSM6DSOX_CTRL9_XL_ADDR  (0x18)
#define LSM6DSOX_OUT_TEMP_L_ADDR (0x20)
#define LSM6DSOX_OUTX_L_G_ADDR  (0x22)
#define LSM6DSOX_OUTX_L_A_ADDR  (0x28)

typedef enum {
    IDLE,
    SETUP,
    RUN,
    CALIBRATE
} systemMode_t;

typedef enum {
    OB_HEAD,
    OB_BODY,
    CHEST_SCALE,
    FSR_RIB_L,
    FSR_RIB_R,
    FSR_HIP_L,
    FSR_HIP_R,
    IMU_LEG_L,
    IMU_LEG_R
} sensorNames_t;

typedef struct {
    bool enabled;
    char shortname[8];
    char datatype[8];
    int datanum;
    double data[3];
} sensorSample_t;

/* Event functions */
void ob_collect(int id);
void scale_collect();
void fsr_collect();
void imu_collect(int id);
void calibrate();
void send_data();
void post_events();

/*  */
void send_datum(sensorSample_t *sample);

/* I2C read/write functions */
void readReg(int address, uint8_t subaddress, char *data, int length);
void writeReg(int address, uint8_t subaddress, uint8_t command);
void setBits(int address, uint8_t subaddress, uint8_t mask);
void clearBits(int address, uint8_t subaddress, uint8_t mask);

/* Orientation board functions */
int ob_init(Adafruit_BNO055 *ob);
void ob_get_accel(Adafruit_BNO055 *ob, double *x, double *y, double *z);
void ob_get_euler(Adafruit_BNO055 *ob, double *x, double *y, double *z);

/* IMU functions */
int imu_init(uint8_t addr);
void imu_get_temp(uint8_t addr, double *temp);
void imu_get_gyro(uint8_t addr, double *x, double *y, double *z);
void imu_get_accel(uint8_t addr, double *x, double *y, double *z);
#endif