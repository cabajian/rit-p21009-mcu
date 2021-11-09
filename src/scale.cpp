/*
 * File    : scale.cpp
 * Author  : Chris Abajian (cxa6282@rit.edu)
 * Created : November 11, 2021
 *
 * Description:
 *   Handles initialization and data retrieval of the scale sensor.
 */

#include "scale.h"

// The scale.
static HX711 scale;
// Calibration factor to convert scale readout to 1 lb of force.
static float calibration_factor = -7050;

int scale_init(long offset = 0) {
    scale.begin(SCALE_CLK, SCALE_DATA);
    scale.set_scale(calibration_factor);
    scale_set_offset(offset);
    return 1;
}

long scale_zero() {
    scale.tare();
    return scale_get_offset();
}

void scale_set_offset(long offset) {
    scale.set_offset(offset);
}

long scale_get_offset() {
    return scale.get_offset();
}

float scale_read() {
    return scale.get_units();
}

bool scale_calibrate(float pounds) {
    Timer t;
    auto start = timer_read_ms(t);
    while (timer_read_ms(t) - start < SCALE_CAL_TIMEOUT_MS) {
        // Determine the difference between the actual and desired values.
        float delta = scale.get_units(SCALE_CAL_ITERS) - pounds;
        if (abs(delta) < SCALE_CAL_EPSILON) {
            // Scale reading within margin. Exit calibration.
            return true;
        } else if (delta > 0) {
            // Scale reading too high. Reduce calibration factor.
            calibration_factor += 10;
        } else {
            // Scale reading too low. Increase calibration factor.
            calibration_factor -= 10;
        }
        // Update the scale's calibration factor.
        scale.set_scale(calibration_factor);
    }
    return false;
}