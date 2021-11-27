/*
 * File    : fsr.cpp
 * Author  : Chris Abajian (cxa6282@rit.edu)
 * Created : November 27, 2021
 */
 
#include <mbed.h>
#include "fsr.h"

AnalogIn pcb_in(PCB_ANALOG_IN);
DigitalOut pcb_s1(PCB_S1_OUT);
DigitalOut pcb_s0(PCB_S0_OUT);

void FSR_zero(DeviceInstance* sensor) {
    // Obtain 5 FSR samples.
    double data = 0;
    for (int i = 0; i < 5; i++) {
        data += FSR_collect(sensor);
    }
    // Save the average offset (inverse of the reading).
    sensor->offsets[0] = data / (-5);
}

double FSR_collect(DeviceInstance* sensor) {
    // Set mux selects going to the PCB.
    switch (sensor->loc) {
        case FOREARM_LEFT:
        case FOREARM_RIGHT:
            pcb_s1 = 0;
            pcb_s0 = 0;
            break;
        case RIB_LEFT:
        case RIB_RIGHT:
            pcb_s1 = 0;
            pcb_s0 = 1;
            break;
        case HIP_LEFT:
        case HIP_RIGHT:
            pcb_s1 = 1;
            pcb_s0 = 0;
            break;
        case KNEE_LEFT:
        case KNEE_RIGHT:
            pcb_s1 = 1;
            pcb_s0 = 1;
            break;
        default:
            pcb_s1 = 0;
            pcb_s0 = 0;
    }
    // Read the analog value (average from ADC_SAMPLES).
    float samples[ADC_SAMPLES];
    samples[0] = pcb_in.read();
    for (int i = 1; i < ADC_SAMPLES; i++) {
        samples[i] = (pcb_in.read() + samples[i-1])/2.0;
    }
    // Store the data.
    return samples[ADC_SAMPLES-1];
}
