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


/* 
 * Send sensor data.
 */
void send_data(DeviceInstance *instance, DeviceInstance *system) {
    if (system->enabled) {
        switch (instance->numdata) {
            case 1:
                printf("%s%s%s%s%s%s%f\n", to_char(instance->dev), DELIM.c_str(), to_char(instance->loc), DELIM.c_str(), to_char(instance->func), DELIM.c_str(), instance->data[0]);
                break;
            case 2:
                printf("%s%s%s%s%s%s%f%s%f\n", to_char(instance->dev), DELIM.c_str(), to_char(instance->loc), DELIM.c_str(), to_char(instance->func), DELIM.c_str(), instance->data[0], DELIM.c_str(), instance->data[1]);
                break;
            case 3:
                printf("%s%s%s%s%s%s%f%s%f%s%f\n", to_char(instance->dev), DELIM.c_str(), to_char(instance->loc), DELIM.c_str(), to_char(instance->func), DELIM.c_str(), instance->data[0], DELIM.c_str(), instance->data[1], DELIM.c_str(), instance->data[2]);
                break;
            default:
                printf("%s%s%s%s%s\n", to_char(instance->dev), DELIM.c_str(), to_char(instance->loc), DELIM.c_str(), to_char(instance->func));
        }
    }
}

/* 
 * Send sensor status.
 */
void send_status(DeviceInstance *instance, Function status) {

}

/* 
 * Send system status.
 */
void send_status(Function status) {

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
        // TODO
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
            args[i-3] = stoi(vec.at(i));
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
