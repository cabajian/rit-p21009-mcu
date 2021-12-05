/*
 * File    : comms.cpp
 * Author  : Chris Abajian (cxa6282@rit.edu)
 * Created : September 5, 2021
 *
 * Description:
 *   Main program entry point for RIT MSD Team P21009 microcontroller software.
 *   NXT FRDM-K64F MCU target.
 */
 
#include <mbed.h>
#include <stdlib.h>
#include "msd.h"
#include "ethernet.h"
#include "orientation_board.h"
#include "fsr.h"
#include "scale.h"
#include "imu.h"

static char cmd_buff[MAX_STR_LEN];
static int cmd_idx = 0;

/* 
 * Sends the status contained in the specified device via UART.
 */
void send_status(DeviceInstance *instance, Function status, BufferedSerial* ser) {
    std::string str = to_string(instance->dev) + DELIM + to_string(instance->loc) + DELIM + to_string(status);
    switch (status) {
        case ENABLE:
            str += DELIM + std::to_string(instance->enabled);
            break;
        case CALIBRATION:
            if (instance->dev == ORIENTATION_BOARD) {
                // Get current calibration status from this sensor.
                int instanceNum = (instance->loc == HEAD) ? OB_HEAD : OB_BODY;
                uint8_t s, g, a, m;
                OB_get_cal(instanceNum, &s, &g, &a, &m);
                // Construct the string.
                str += DELIM + std::to_string(g) + DELIM + std::to_string(a) +  DELIM + std::to_string(m) +  DELIM + std::to_string(s);
            } else {
                str += DELIM + "DEVICE_MISMATCH_ERROR";
            }
            break;
        case OFFSET:
            // OB/IMU have multiple offsets
            if ((instance->dev == ORIENTATION_BOARD) || (instance->dev == IMU)) {
                if (instance->func == ACCELERATION) {
                    str += DELIM + "ACC";
                } else if (instance->func == EULER) {
                    str += DELIM + "EUL";
                } else if (instance->func == GYROSCOPE) {
                    str += DELIM + "GYR";
                }
                str += DELIM + std::to_string(instance->offsets[0]) + DELIM + std::to_string(instance->offsets[1]) + DELIM + std::to_string(instance->offsets[2]);
            } else {
                str += DELIM + std::to_string(instance->offsets[0]);
            }
            break;
        default:
            str += DELIM + "INVALID_STATUS_ERROR";
    }
    // Send the status.
    str += "\n\0";
    ser->write(str.c_str(), str.length());
}


/**
 * Sends the sensor data contained in data_buff to be 
 * transmitted via Ethernet if system logging is enabled.
 */
void send_data(bool* loggingEn, char* data, int* size, BufferedSerial* ser) {
    if (*loggingEn) {
        data[*size] = '\0';
        #ifdef DEBUG
            // DEBUG: write data over serial
            ser->write(data, *size);
        #else
            // Non-debug: write data over ethernet
            ethernet_send(data, *size);
        #endif
    }
    // Restart the buffer regardless.
    *size = 0;
}

void handle_cmd(Device dev, Location loc, Function cmd, double* args, BufferedSerial* ser) {
    DeviceInstance* instance;
    int i;
    switch (cmd) {
        case RESTART:
            NVIC_SystemReset();//TODO: need to verify this works and doesn't halt
            break;
        case ENABLE:
            // Enable the device.
            instance = get_device_instance(dev, loc, NO_FUNCTION, &i);
            instance->enabled = (args[0] > 0);
            // OB/IMU have two data types.
            if ((dev == ORIENTATION_BOARD) || (dev == IMU)) {
                instance = get_device_instance(i+1);
                instance->enabled = (args[0] > 0);
            }
            break;
        case OFFSET:
            instance = get_device_instance(dev, loc, NO_FUNCTION, &i);
            if ((dev == ORIENTATION_BOARD) || (dev == IMU)) {
                // Load the 3 offsets.
                if (args[0] == EUL_PARAM_CODE || args[0] == GYR_PARAM_CODE) {
                    instance = get_device_instance(i+1);
                }
                instance->offsets[0] = args[1];
                instance->offsets[1] = args[2];
                instance->offsets[2] = args[3];
            } else {
                // Load the offset.
                instance->offsets[0] = args[0];
            }            
            break;
        case ZERO:
            // Retrieve the device instance and get new offsets.
            instance = get_device_instance(dev, loc, NO_FUNCTION, &i);
            switch (dev) {
                case ORIENTATION_BOARD:
                    if (args[0]) {
                        OB_zero(instance);
                    }
                    if (args[1]) {
                        instance = get_device_instance(i+1);
                        OB_zero(instance);
                    }
                    break;
                case SCALE:
                    instance->offsets[0] = scale_zero();
                    break;
                case FSR:
                    FSR_zero(instance);
                    break;
                case IMU:
                    if (args[0]) {
                        IMU_zero(instance);
                    }
                    if (args[1]) {
                        instance = get_device_instance(i+1);
                        IMU_zero(instance);
                    }               
                    break;
                default:
                    const char tmp[] = {"INVALID_ZERO_COMMAND\n\0"};
                    ser->write(tmp, sizeof(tmp));
            }
            // Zero this sensor and store the offsets.
            instance = get_device_instance(dev, loc, NO_FUNCTION, &i);
            // TODO: zero the sensor
            // OB/IMU have two data types
            if ((dev == ORIENTATION_BOARD) || (dev == IMU)) {
                instance = get_device_instance(i+1);
            }
            break;
        default:
            const char tmp[] = {"INVALID_COMMAND_ERROR\n\0"};
            ser->write(tmp, sizeof(tmp));
    }
}

/* 
 * Polls for a command.
 *   Returns 1 if a command was received and successfully parsed.
 *   Returns 0 if no command was received.
 *   Returns -1 if a command was detected but incomplete or parsing otherwise failed.
 */
int poll_cmd(BufferedSerial* ser) {   
    std::string str;
    // Read from the serial buffer.
    int bytes = ser->read(cmd_buff+cmd_idx, MAX_STR_LEN);
    if (bytes <= 0) {
        // No data.
        return 0;
    } else {
        // Data exists. Check if the command is complete.
        cmd_idx += bytes;
        char last = cmd_buff[cmd_idx-1];
        if ((last == '\r') || (last == '\n') || (last == '\0')) {
            str = cmd_buff;
            str.erase(cmd_idx-1);
            cmd_idx = 0;
        } else {
            return 0;
        }
    }

    // Parse the command.
    Device dev;
    Location loc;
    Function func;
    double args[4];
    bool success = parse_cmd(str, &dev, &loc, &func, args, 4);

    // Act on the command.
    if (success) {
        if (args[3] == STATUS_CODE) {
            // Status request.
            int i;
            DeviceInstance* instance = get_device_instance(dev, loc, NO_FUNCTION, &i);
            if (args[0] == EUL_PARAM_CODE || args[0] == GYR_PARAM_CODE) {
                instance = get_device_instance(i+1);
            }
            if (instance == NULL) {
                const char tmp[] = {"INVALID_DEVICE_ERROR\n\0"};
                ser->write(tmp, sizeof(tmp));
            } else {
                send_status(instance, func, ser);
            }
        } else {
            // Act on the command.
            printf("%s,%s,%s,%f,%f,%f,%f\r\n", to_string(dev).c_str(), to_string(loc).c_str(), to_string(func).c_str(), args[0], args[1], args[2], args[3]);
            handle_cmd(dev, loc, func, args, ser);
        }
        return 1;
    } else {
        // Parsing error.
        return -1;
    }
}

/* 
 * Parse a command.
 *   Returns TRUE if the command was successfully parsed.
 *   Returns FALSE if the command was unsuccessfully parsed.
 */
bool parse_cmd(std::string cmd, Device* dev, Location* loc, Function* func, double* args, int numargs) {
    // Return variable.
    bool valid_cmd = false;
    bool status = false;
    // Split the command.
    std::vector<std::string> vec = split_string(cmd, DELIM);
    int len = vec.size();
    if (len >= 3) {
        // Get the device.
        to_enum(vec.at(0), dev);
        if (*dev == NO_DEVICE) return false;
        // Get the location.
        to_enum(vec.at(1), loc);
        if (*loc == NO_LOCATION) return false;
        // Get the command.
        to_enum(vec.at(2), func);
        if (*func == NO_FUNCTION) return false;
        // Check for status functions (CAL, OFF, EN)
        if (*func == CALIBRATION || *func == OFFSET || *func == ENABLE) {
            status = true;
        }
        // Process the arguments.
        for (int i = 3; (i < len) && (i-3 < numargs); i++) {
            // Convert the argument to a double.
            char* ptr;
            args[i-3] = strtod(vec.at(i).c_str(), &ptr);
            if (ptr == vec.at(i).c_str()) {
                // Conversion unsuccessful, check if parameter descriptor.
                if (vec.at(i) == "ACC") {
                    args[i-3] = ACC_PARAM_CODE;
                } else if (vec.at(i) == "EUL") {
                    args[i-3] = EUL_PARAM_CODE;
                } else if (vec.at(i) == "GYR") {
                    args[i-3] = GYR_PARAM_CODE;
                }
            } else {
                // Clear status flag if conversion successful.
                status = false;
            }
        }
        valid_cmd = true;
    }
    // Return.
    if (status) args[numargs-1] = STATUS_CODE;
    return valid_cmd;
}

/*
 * Splits the provided string by the delimeter and returns the vector.
 * Function courtesy of https://stackoverflow.com/users/14073678/rikupotato
 */
std::vector<std::string> split_string(std::string str, std::string delimeter) {
    // Declare variables.
    std::vector<std::string> splitStrings = {};
    size_t pos = 0;
    // Iterate through the string and split on the delimiter.
    while ((pos = str.find(delimeter)) != string::npos) {
        std::string token = str.substr(0, pos);
        if (token.length() > 0)
            splitStrings.push_back(token);
        str.erase(0, pos + delimeter.length());
    }
    // Add the rest of the string to the vector.
    if (str.length() > 0)
        splitStrings.push_back(str);
    // Return.
    return splitStrings;
}
