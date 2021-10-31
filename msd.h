/*
 * File    : main.cpp
 * Author  : Chris Abajian (cxa6282@rit.edu)
 * Created : September 5, 2021
 *
 * Description:
 *   Main header for RIT MSD Team P21009 microcontroller software.
 */

#ifndef __MSD_INCLUDE_H__
#define __MSD_INCLUDE_H__

#include "Adafruit_BNO055.h"
#include <iostream>

/* LSM6DSOX macros */
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

// Number of ADC samples per datum collection.
#define ADC_SAMPLES                 (10)

// Sensor periods
#define OB_PERIOD_MS                (10)
#define IMU_PERIOD_MS               (10)
#define FSR_PERIOD_MS               (50)
#define SCALE_PERIOD_MS             (12)
#define SEND_PERIOD_MS              (10)

typedef enum {
    IDLE,
    SETUP,
    RUN,
    CALIBRATE
} systemMode;
static const char * mode_to_string(systemMode mode) {
    switch (mode) {
        case IDLE:
            return "IDLE";
        case SETUP:
            return "SETUP";
        case RUN:
            return "RUN";
        case CALIBRATE:
            return "CALIBRATE";
    }
}
#define MODE_STR(x)    (mode_to_string(x))

typedef enum {
    ORIENTATION_BOARD,
    SCALE,
    FSR,
    IMU
} sensorType;
static const char * stype_to_string(sensorType type) {
    switch (type) {
        case ORIENTATION_BOARD:
            return "OB";
        case SCALE:
            return "SCAL";
        case FSR:
            return "FSR";
        case IMU:
            return "IMU";
    }
}
#define STYPE_STR(x)    (stype_to_string(x))

typedef enum {
    HEAD,
    BODY,
    CHEST,
    RIB_LEFT,
    RIB_RIGHT,
    FOREARM_LEFT,
    FOREARM_RIGHT,
    HIP_LEFT,
    HIP_RIGHT,
    LEG_LEFT,
    LEG_RIGHT,
    KNEE_LEFT,
    KNEE_RIGHT
} sensorLocation;
static const char * slocation_to_string(sensorLocation loc) {
    switch (loc) {
        case HEAD:
            return "H";
        case BODY:
            return "B";
        case CHEST:
            return "CH";
        case RIB_LEFT:
            return "RL";
        case RIB_RIGHT:
            return "RR";
        case FOREARM_LEFT:
            return "FL";
        case FOREARM_RIGHT:
            return "FR";
        case HIP_LEFT:
            return "HL";
        case HIP_RIGHT:
            return "HR";
        case LEG_LEFT:
            return "LL";
        case LEG_RIGHT:
            return "LR";
        case KNEE_LEFT:
            return "KL";
        case KNEE_RIGHT:
            return "KR";
    }
}
#define SLOC_STR(x)    (slocation_to_string(x))

typedef enum {
    ACCELERATION,
    EULER,
    FORCE,
    GYROSCOPE,
    CALIBRATION
} sensorDataType;
static const char * sdatatype_to_string(sensorDataType mode) {
    switch (mode) {
        case ACCELERATION:
            return "ACC";
        case EULER:
            return "EUL";
        case FORCE:
            return "FRC";
        case GYROSCOPE:
            return "GYR";
        case CALIBRATION:
            return "CAL";
    }
}
#define SDTYPE_STR(x)    (sdatatype_to_string(x))

typedef struct {
    bool enabled;
    sensorType type;
    sensorLocation location;
    sensorDataType datatype;
    int datanum;
    double data[4];
    bool newdata;
} sensorInstance;
static sensorInstance sensors[] = {
    // Enabled  Type               Location       Data Type     Data Num   Data       New Data
    {false,     ORIENTATION_BOARD, HEAD,          ACCELERATION, 3,         {0,0,0,0}, false},
    {false,     ORIENTATION_BOARD, HEAD,          EULER,        3,         {0,0,0,0}, false},
    {false,     ORIENTATION_BOARD, BODY,          ACCELERATION, 3,         {0,0,0,0}, false},
    {false,     ORIENTATION_BOARD, BODY,          EULER,        3,         {0,0,0,0}, false},
    {false,     SCALE,             CHEST,         FORCE,        1,         {0,0,0,0}, false},
    {false,     FSR,               RIB_LEFT,      FORCE,        1,         {0,0,0,0}, false},
    {false,     FSR,               RIB_RIGHT,     FORCE,        1,         {0,0,0,0}, false},
    {false,     FSR,               FOREARM_LEFT,  FORCE,        1,         {0,0,0,0}, false}, 
    {false,     FSR,               FOREARM_RIGHT, FORCE,        1,         {0,0,0,0}, false},
    {false,     FSR,               HIP_LEFT,      FORCE,        1,         {0,0,0,0}, false},
    {false,     FSR,               HIP_RIGHT,     FORCE,        1,         {0,0,0,0}, false},
    {false,     FSR,               KNEE_LEFT,     FORCE,        1,         {0,0,0,0}, false},
    {false,     FSR,               KNEE_RIGHT,    FORCE,        1,         {0,0,0,0}, false},
    {false,     IMU,               LEG_LEFT,      ACCELERATION, 3,         {0,0,0,0}, false},
    {false,     IMU,               LEG_LEFT,      GYROSCOPE,    3,         {0,0,0,0}, false},
    {false,     IMU,               LEG_RIGHT,     ACCELERATION, 3,         {0,0,0,0}, false},
    {false,     IMU,               LEG_RIGHT,     GYROSCOPE,    3,         {0,0,0,0}, false},
    {false,     ORIENTATION_BOARD, HEAD,          CALIBRATION,  4,         {0,0,0,0}, false},
    {false,     ORIENTATION_BOARD, BODY,          CALIBRATION,  4,         {0,0,0,0}, false}
};

/* Event functions */
void collect_ob(int start_idx, int end_idx);
void collect_scale(int start_idx, int end_idx);
void collect_fsr(int start_idx, int end_idx);
void collect_imu(int start_idx, int end_idx);
void calibrate();
void print_startstop();
void post_events();
void print_datum(sensorInstance *sensor);
void send();


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