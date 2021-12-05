/*
 * File    : msd.cpp
 * Author  : Chris Abajian (cxa6282@rit.edu)
 * Created : September 5, 2021
 *
 * Description:
 *   Main header for RIT MSD Team P21009 microcontroller software.
 */

#pragma once

#ifndef __MSD_INCLUDE_H__
#define __MSD_INCLUDE_H__

#include "mbed.h"
#include "communications.h"

// UART definitions.
#ifdef DEBUG
    #define UART_TX         (USBTX)     
    #define UART_RX         (USBRX)
    #define UART_BAUD       (115200)
#else
    #define UART_TX         (PTB11)//must not colllide with ETH
    #define UART_RX         (PTB10)//must not colllide with ETH
    #define UART_BAUD       (115200)
#endif

// Sensor sampling periods.
#ifdef DEBUG
    #define OB_PERIOD_MS                (1000)
    #define IMU_PERIOD_MS               (1000)
    #define FSR_PERIOD_MS               (5000)
    #define SCALE_PERIOD_MS             (1200)
    #define SEND_PERIOD_MS              (1000)
    #define POLL_CMD_PERIOD_MS          (1000)
#else
    #define OB_PERIOD_MS                (10)
    #define IMU_PERIOD_MS               (10)
    #define FSR_PERIOD_MS               (50)
    #define SCALE_PERIOD_MS             (12)
    #define SEND_PERIOD_MS              (10)
    #define POLL_CMD_PERIOD_MS          (100)
#endif

// Indices of the device array.
#define OB_FIRST_IDX                (1)
#define OB_LAST_IDX                 (4)
#define SCALE_FIRST_IDX             (5)
#define SCALE_LAST_IDX              (5)
#define FSR_FIRST_IDX               (6)
#define FSR_LAST_IDX                (13)
#define IMU_FIRST_IDX               (14)
#define IMU_LAST_IDX                (17)
#define NUM_DEVICES                 (18)

// A structure of a single device instance.
typedef struct {
    bool enabled;
    Device dev;
    Location loc;
    Function func;
    double offsets[3];
} DeviceInstance;

/* Communication Function Prototypes */
// Send sensor data.
void send_data(bool* loggingEn, char* data, int* size, BufferedSerial* ser);
// Send sensor status.
void send_status(DeviceInstance *instance, Function status, BufferedSerial* ser);
void handle_cmd(Device dev, Location loc, Function cmd, double* args, BufferedSerial* ser);
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

DeviceInstance* get_device_instance(Device dev, Location loc, Function func, int* index);
DeviceInstance* get_device_instance(int index);

#endif