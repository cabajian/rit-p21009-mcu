/*
 * File    : s_orientation.cpp
 * Author  : Chris Abajian (cxa6282@rit.edu)
 * Created : September 7, 2021
 *
 * Description:
 *   Handles initialization and data retrieval of the orientation
 *   board sensors on the test mannequin.
 */

#include "msd.h"

int ob_init(Adafruit_BNO055 *ob) {
    if(!ob->begin()) {
        printf("Orientation Board not detected!\r\n");
        return -1;
    } else {
        printf("Orientation Board was detected!\r\n");
    }
    ThisThread::sleep_for(1s);
    ob->setMode(Adafruit_BNO055::OPERATION_MODE_NDOF);
    writeReg(BNO055_ADDRESS_A<<1, Adafruit_BNO055::BNO055_OPR_MODE_ADDR, Adafruit_BNO055::OPERATION_MODE_NDOF);
    ob->setExtCrystalUse(true);
    // Success
    return 1;
}

void ob_get_accel(Adafruit_BNO055 *ob, double *x, double *y, double *z) {
    imu::Vector<3> vec = ob->getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    *x = vec.x();
    *y = vec.y();
    *z = vec.z();
}

void ob_get_euler(Adafruit_BNO055 *ob, double *x, double *y, double *z) {
    imu::Vector<3> vec = ob->getVector(Adafruit_BNO055::VECTOR_EULER);
    *x = vec.x();
    *y = vec.y();
    *z = vec.z();
}