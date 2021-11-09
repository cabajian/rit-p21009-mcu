/*
 * File    : orientation.cpp
 * Author  : Chris Abajian (cxa6282@rit.edu)
 * Created : September 7, 2021
 *
 * Description:
 *   Handles initialization and data retrieval of the orientation board sensors.
 */

#include "orientation_board.h"
#include "i2c.h"

Adafruit_BNO055 ob_head = Adafruit_BNO055(OB_HEAD, BNO055_ADDRESS_A, I2C_get());
Adafruit_BNO055 ob_body = Adafruit_BNO055(OB_BODY, BNO055_ADDRESS_B, I2C_get());

/**
 * Initialize the Orientation Board instance.
 */
int OB_init(int instance = OB_HEAD) {
    Adafruit_BNO055* ob;
    if (instance == OB_HEAD) {
        ob = &ob_head;
    } else if (instance == OB_BODY) {
        ob = &ob_body;
    } else {
        return -1;
    }
    if (!ob->begin()) {
        return -1;
    }
    ThisThread::sleep_for(1s);
    ob->setMode(Adafruit_BNO055::OPERATION_MODE_NDOF);
    I2C_write(BNO055_ADDRESS_A<<1, Adafruit_BNO055::BNO055_OPR_MODE_ADDR, Adafruit_BNO055::OPERATION_MODE_NDOF);
    ob->setExtCrystalUse(true);
    // Success
    return 1;
}

/**
 * Get the specified Orientation Board instance.
 */
Adafruit_BNO055* OB_get(int instance) {
    return ((instance == OB_HEAD) ? &ob_head : &ob_body);
}

/**
 * Get the linear acceleration vector from the specified Orientation Board instance.
 */
void OB_get_accel(int instance, double *x, double *y, double *z) {
    imu::Vector<3> vec;
    if (instance == OB_HEAD) {
        vec = ob_head.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    } else if (instance == OB_BODY) {
        vec = ob_body.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    }
    *x = vec.x();
    *y = vec.y();
    *z = vec.z();
}

/**
 * Get the euler vector from the specified Orientation Board instance.
 */
void OB_get_euler(int instance, double *x, double *y, double *z) {
    imu::Vector<3> vec;
    if (instance == OB_HEAD) {
        vec = ob_head.getVector(Adafruit_BNO055::VECTOR_EULER);
    } else if (instance == OB_BODY) {
        vec = ob_body.getVector(Adafruit_BNO055::VECTOR_EULER);
    }
    *x = vec.x();
    *y = vec.y();
    *z = vec.z();
}
