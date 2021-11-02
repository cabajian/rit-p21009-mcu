/*
 * File    : main.cpp
 * Author  : Chris Abajian (cxa6282@rit.edu)
 * Created : September 5, 2021
 *
 * Description:
 *   Main header for RIT MSD Team P21009 microcontroller software.
 */

#pragma once

#ifndef __MSD_INCLUDE_H__
#define __MSD_INCLUDE_H__

#include "Adafruit_BNO055.h"
#include "msd_comms.h"

// UART definitions.
#ifdef DEBUG
    #define UART_TX         (USBTX)     
    #define UART_RX         (USBRX)
    #define UART_BAUD       (576000)
#else
    #define UART_TX         (D1)     
    #define UART_RX         (D0)
    #define UART_BAUD       (576000)
#endif
// Pin mappings.
#define PCB_ANALOG_IN   (A0)
#define PCB_S1_OUT      (PTD0)
#define PCB_S0_OUT      (PTC4)

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
// Number of ADC samples per datum collection.
#define ADC_SAMPLES                 (10)
// Sensor sampling periods.
#define OB_PERIOD_MS                (10)
#define IMU_PERIOD_MS               (10)
#define FSR_PERIOD_MS               (50)
#define SCALE_PERIOD_MS             (12)
#define SEND_PERIOD_MS              (10)
#define POLL_CMD_PERIOD_MS          (100)

/* Sensor Structures */
#define OB_FIRST_IDX                (1)
#define OB_LAST_IDX                 (4)
#define SCALE_FIRST_IDX             (5)
#define SCALE_LAST_IDX              (5)
#define FSR_FIRST_IDX               (6)
#define FSR_LAST_IDX                (13)
#define IMU_FIRST_IDX               (14)
#define IMU_LAST_IDX                (17)
typedef struct {
    bool enabled;
    Device dev;
    Location loc;
    Function func;
    int numdata;
    double data[4];
    int numoffsets;
    double offsets[3];
} DeviceInstance;
static DeviceInstance devices[] = {
    // Enabled  Device             Location       Function      Num Data  Data       Num Offsets  Offsets
    {false,     SYSTEM,            GLOBAL,        LOGGING,      0,        {0,0,0,0}, 0,           {0,0,0}},
    {false,     ORIENTATION_BOARD, HEAD,          ACCELERATION, 3,        {0,0,0,0}, 3,           {0,0,0}},
    {false,     ORIENTATION_BOARD, HEAD,          EULER,        3,        {0,0,0,0}, 3,           {0,0,0}},
    {false,     ORIENTATION_BOARD, BODY,          ACCELERATION, 3,        {0,0,0,0}, 3,           {0,0,0}},
    {false,     ORIENTATION_BOARD, BODY,          EULER,        3,        {0,0,0,0}, 3,           {0,0,0}},
    {false,     SCALE,             CHEST,         FORCE,        1,        {0,0,0,0}, 1,           {0,0,0}},
    {false,     FSR,               RIB_LEFT,      FORCE,        1,        {0,0,0,0}, 1,           {0,0,0}},
    {false,     FSR,               RIB_RIGHT,     FORCE,        1,        {0,0,0,0}, 1,           {0,0,0}},
    {false,     FSR,               FOREARM_LEFT,  FORCE,        1,        {0,0,0,0}, 1,           {0,0,0}}, 
    {false,     FSR,               FOREARM_RIGHT, FORCE,        1,        {0,0,0,0}, 1,           {0,0,0}},
    {false,     FSR,               HIP_LEFT,      FORCE,        1,        {0,0,0,0}, 1,           {0,0,0}},
    {false,     FSR,               HIP_RIGHT,     FORCE,        1,        {0,0,0,0}, 1,           {0,0,0}},
    {false,     FSR,               KNEE_LEFT,     FORCE,        1,        {0,0,0,0}, 1,           {0,0,0}},
    {false,     FSR,               KNEE_RIGHT,    FORCE,        1,        {0,0,0,0}, 1,           {0,0,0}},
    {false,     IMU,               LEG_LEFT,      ACCELERATION, 3,        {0,0,0,0}, 1,           {0,0,0}},
    {false,     IMU,               LEG_LEFT,      GYROSCOPE,    3,        {0,0,0,0}, 1,           {0,0,0}},
    {false,     IMU,               LEG_RIGHT,     ACCELERATION, 3,        {0,0,0,0}, 1,           {0,0,0}},
    {false,     IMU,               LEG_RIGHT,     GYROSCOPE,    3,        {0,0,0,0}, 1,           {0,0,0}}
};

/* Communication Function Prototypes */
// Send sensor data.
void send_data(DeviceInstance *instance, DeviceInstance *system);
// Send sensor status.
void send_status(DeviceInstance *instance, Function status);
// Send system status.
void send_status(Function status);
// Polls for a command.
//   Returns 1 if a command was received and successfully parsed.
//   Returns 0 if no command was received.
//   Returns -1 if a command was detected but incomplete or parsing otherwise failed. 
int poll_cmd(BufferedSerial* ser);
// Parse a command.
//   Returns TRUE if the command was successfully parsed.
//   Returns FALSE if the command was unsuccessfully parsed.
bool parse_cmd(std::string cmd, Device* dev, Location* loc, Function* func, double* args, int numargs);
// Splits the provided string by the delimeter and returns the vector.
std::vector<std::string> split_string(std::string str, std::string delimeter);

/* Event Function Prototypes */
void collect_ob();
void collect_scale();
void collect_fsr();
void collect_imu();
void post_events();
void suspend_events();

/* I2C Read/Write Function Prototypes */
void readReg(int address, uint8_t subaddress, char *data, int length);
void writeReg(int address, uint8_t subaddress, uint8_t command);
void setBits(int address, uint8_t subaddress, uint8_t mask);
void clearBits(int address, uint8_t subaddress, uint8_t mask);

/* Orientation Board Function Prototypes */
int ob_init(Adafruit_BNO055 *ob);
void ob_get_accel(Adafruit_BNO055 *ob, double *x, double *y, double *z);
void ob_get_euler(Adafruit_BNO055 *ob, double *x, double *y, double *z);

/* IMU Function Prototypes */
int imu_init(uint8_t addr);
void imu_get_temp(uint8_t addr, double *temp);
void imu_get_gyro(uint8_t addr, double *x, double *y, double *z);
void imu_get_accel(uint8_t addr, double *x, double *y, double *z);

#endif