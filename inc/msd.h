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

// Number of ADC samples per datum collection.
#define ADC_SAMPLES                 (1)
// Sensor sampling periods.
#define OB_PERIOD_MS                (10)
#define IMU_PERIOD_MS               (10)
#define FSR_PERIOD_MS               (50)
#define SCALE_PERIOD_MS             (12)
#define SEND_PERIOD_MS              (10)
#define POLL_CMD_PERIOD_MS          (100)

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
    double offset;
} DeviceInstance;

/* Communication Function Prototypes */
// Send sensor data.
void send_data(bool loggingEn, char* data, int* size);
// Send sensor status.
void send_status(DeviceInstance *instance, Function status, BufferedSerial* ser);
void handle_cmd(Device dev, Location loc, Function cmd, BufferedSerial* ser);
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

DeviceInstance* get_device_instance(Device dev, Location loc, Function func);
DeviceInstance* get_device_instance(int index);

#endif