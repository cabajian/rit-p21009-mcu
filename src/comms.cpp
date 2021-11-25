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
#include "msd.h"
#include "orientation_board.h"

/* 
 * Sends the status contained in the specified device via UART.
 */
void send_status(DeviceInstance *instance, Function status, BufferedSerial* ser) {
    std::string str = to_string(instance->dev) + DELIM + to_string(instance->location) + DELIM + to_string(instance->status);
    switch (status) {
        case ENABLE:
            str += DELIM + std::to_string(instance->enabled);
            break;
        case CALIBRATION:
            if (instance->dev == ORIENTATION_BOARD) {
                // Get current calibration status from this sensor.
                int instanceNum = (instance->location == HEAD) ? OB_HEAD : OB_BODY;
                uint8_t s, g, a, m;
                OB_get_cal(instanceNum, &s, &g, &a, &m);
                // Construct the string.
                str += DELIM + std::to_string(g) + DELIM + std::to_string(a) +  DELIM + std::to_string(m) +  DELIM + std::to_string(s);
            } else {
                str += DELIM + "DEVICE_MISMATCH_ERROR";
            }
            break;
        case OFFSET:
            str += DELIM + std::to_string(instance->offset);
            int i = get_device_instance(instance->dev, instance->loc, NO_FUNCTION, NULL);
            // OB/IMU have two data types
            if ((instance->dev == ORIENTATION_BOARD) || (instance->dev == IMU)) {
                get_device_instance(i+1, instance);
                str += DELIM + std::to_string(instance->offset);
            }
            break;
        default:
            str += DELIM + "INVALID_STATUS_ERROR";
    }
    // Send the status.
    ser->write(str + "\n\0");
}


/**
 * Sends the sensor data contained in data_buff to be 
 * transmitted via Ethernet if system logging is enabled.
 */
void send_data(bool loggingEn, char* data, int* size) {
    if (loggingEn) {
        data[*size] = '\0';
        ethernet_send(data, *size);
        *size = 0;
    }
}

void handle_cmd(Device dev, Location loc, Function cmd, BufferedSerial* ser) {
    switch (cmd) {
        case RESTART:
            NVIC_SystemReset();//TODO: need to verify this works and doesn't halt
            break;
        case ENABLE:
            // Enable the device.
            DeviceInstance* instance;
            int i = get_device_instance(dev, loc, NO_FUNCTION, instance);
            instance->enabled = (args[0] > 0);
            // OB/IMU have two data types
            if ((dev == ORIENTATION_BOARD) || (dev == IMU)) {
                get_device_instance(i+1, instance);
                instance->enabled = (args[0] > 0);
            }
            break;
        case OFF:
            // Load the offsets.
            // Special handling for OB's/IMU's
            DeviceInstance* instance;
            int j = get_device_instance(dev, loc, NO_FUNCTION, instance);
            instance->offset = args[0];
            // OB/IMU have two data types
            if ((dev == ORIENTATION_BOARD) || (dev == IMU)) {
                get_device_instance(i+1, instance);
                instance->offset = args[1];
            }
            break;
        case ZERO:
            //TODO
            break;
        default:
            ser->write("INVALID_COMMAND_ERROR\n\0");
    }
}

/* 
 * Polls for a command.
 *   Returns 1 if a command was received and successfully parsed.
 *   Returns 0 if no command was received.
 *   Returns -1 if a command was detected but incomplete or parsing otherwise failed.
 */
int poll_cmd(BufferedSerial* ser) {
    char buffer[MAX_STR_LEN];
    std::string str;
    // Read from the serial buffer.
    int len = ser->read(buffer, MAX_STR_LEN);
    if (len <= 0) {
        // No data.
        return 0;
    } else {
        // Data exists. Check if the command is complete.
        char last = buffer[len-1];
        if ((last == '\n') || (last == '\0')) {
            str = buffer;
            str.erase(len-1);
        } else {
            // Command not complete. Repeatedly check for more data until 1ms timeout.
            int iters = 10;
            while (iters-- > 0) {
                len += ser->read(buffer+len, MAX_STR_LEN-len);
                last = buffer[len-1];
                if ((last == '\n') || (last == '\0')) {
                    str = buffer;
                    str.erase(len-1);
                    break;
                }
            }
        }
    }

    // Parse the command.
    Device dev;
    Location loc;
    Function func;
    double args[3];
    bool success = parse_cmd(str, &dev, &loc, &func, args, 3);

    // Act on the command.
    if (success) {
        if (args[0] == -9999) {
            // Status request.
            DeviceInstance* instance;
            get_device_instance(dev, loc, NO_FUNCTION, instance);
            if (instance == NULL) {
                ser->write("INVALID_DEVICE_ERROR\n\0");
            } else {
                send_status(instance, func, ser);
            }
        } else {
            // Act on the command.
            handle_cmd(dec, loc, func, ser);
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
        // Get the arguments.
        for (int i = 3; (i < len) && (i-3 < numargs); i++) {
            if (vec.at(i) == "STAT") {
                args[i-3] = -9999;
                break;
            } else {
                args[i-3] = stoi(vec.at(i));
            }
        }
        valid_cmd = true;
    }
    // Return.
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
