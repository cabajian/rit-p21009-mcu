/*
 * File    : fsr.h
 * Author  : Chris Abajian (cxa6282@rit.edu)
 * Created : November 27, 2021
 */
#pragma once

#ifndef __FSR_INCLUDE_H__
#define __FSR_INCLUDE_H__

#include "mbed.h"
#include "msd.h"

// Pin mappings.
#define PCB_ANALOG_IN   (A0)
#define PCB_S1_OUT      (PTD0)
#define PCB_S0_OUT      (PTC4)

// Number of ADC samples per datum collection.
#define ADC_SAMPLES     (2)

void FSR_zero(DeviceInstance* sensor);
double FSR_collect(DeviceInstance* sensor);

#endif