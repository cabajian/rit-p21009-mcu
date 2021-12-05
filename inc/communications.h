/*
 * File    : communications.h
 * Author  : Chris Abajian (cxa6282@rit.edu)
 * Created : October 31, 2021
 *
 * Description:
 *   Communications header file that provides common sensor types,
 *   locations, data types, commands, and other standardized printouts
 *   to or from the MCU platform.
 */

#pragma once

#ifndef __COMMUNICATIONS_INCLUDE_H__
#define __COMMUNICATIONS_INCLUDE_H__

#include <iostream>
#include <string>
#include <vector>

#define MAX_STR_LEN     (1024)
#define STATUS_CODE     (-999)
#define ACC_PARAM_CODE  (-1000)
#define EUL_PARAM_CODE  (-1001)
#define GYR_PARAM_CODE  (-1002)

static const std::string DELIM = ">";

/* Mannequin Locations */
typedef enum {
    NO_LOCATION,
    GLOBAL,
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
} Location;
static const std::string locationStrings[] = {
    "\0",
    "GLBL",
    "H",
    "B",
    "CH",
    "LR",
    "RR",
    "LF",
    "RF",
    "LH",
    "RH",
    "LL",
    "RL",
    "LK",
    "RK"
};
inline std::string to_string(Location loc) {
    return locationStrings[loc];
}
inline const char* to_char(Location loc) {
    return locationStrings[loc].c_str();
}
inline bool to_enum(std::string str, Location* loc) {
    for (int i = 1; i < 15; i++) {
        auto e = Location(i);
        if (str.compare(to_string(e)) == 0) {
            *loc = e;
            return true;
        }
    }
    *loc = NO_LOCATION;
    return false;
}

/* Device Types */
typedef enum {
    NO_DEVICE,
    SYSTEM,
    ORIENTATION_BOARD,
    SCALE,
    FSR,
    IMU
} Device;
static const std::string deviceStrings[] = {
    "\0",
    "SYS",
    "OB",
    "SCAL",
    "FSR",
    "IMU"
};
inline std::string to_string(Device dev) {
    return deviceStrings[dev];
}
inline const char* to_char(Device dev) {
    return deviceStrings[dev].c_str();
}
inline bool to_enum(std::string str, Device* dev) {
    for (int i = 1; i < 6; i++) {
        auto e = Device(i);
        if (str.compare(to_string(e)) == 0) {
            *dev = e;
            return true;
        }
    }
    *dev = NO_DEVICE;
    return false;
}

/* Data/Status/Command Functions */
typedef enum {
    NO_FUNCTION,
    ACCELERATION,
    EULER,
    FORCE,
    GYROSCOPE,
    RESTART,
    ENABLE,
    CALIBRATION,
    OFFSET,
    ZERO
} Function;
static const std::string functionStrings[] = {
    "\0",
    "ACC",
    "EUL",
    "FRC",
    "GYR",
    "RESTART",
    "EN",
    "CAL",
    "OFF",
    "ZERO"
};
inline std::string to_string(Function func) {
    return functionStrings[func];
}
inline const char* to_char(Function func) {
    return functionStrings[func].c_str();
}
inline bool to_enum(std::string str, Function* func) {
    for (int i = 1; i < 11; i++) {
        auto e = Function(i);
        if (str.compare(to_string(e)) == 0) {
            *func = e;
            return true;
        }
    }
    *func = NO_FUNCTION;
    return false;
}

#endif
