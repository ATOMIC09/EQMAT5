/*
 * peaking_filter.c
 *
 *  Created on: May 12, 2024
 *      Author: phuta
 */

#define _USE_MATH_DEFINES
#include "peaking_filter.h"
//#include <stdio.h>


void peaking_filter_init(peaking_filter_data *filt) {
    // Initialize filter memory
    filt -> x[0] = 0.0f;
    filt -> x[1] = 0.0f;
    filt -> x[2] = 0.0f;
    filt -> y[0] = 0.0f;
    filt -> y[1] = 0.0f;
    filt -> y[2] = 0.0f;
}

float dt = 1 / 96E3;

void peaking_filter_cal_coeff(peaking_filter_data *filt, float gain, float centerFreq_hz, float q_width) {
    // Convert Hz to rad/s
    float centerFreq_rad = 2 * M_PI * centerFreq_hz;

    // Convert Hz to rad/s (Pre-warping cutoff frequency for bilinear transform)
    // float wcT = 2.0f * tan(M_PI * center_freq * filt -> sample_time);
    
    // Compute Quality factor (Q = fc / BW)
    // float Q = center_freq / bandwidth;

    // Guide
    float wcT = centerFreq_rad * dt;
    
    // Compute filter coefficients
    filt -> a[0] = 4.0f + (2.0f * (gain / q_width) * wcT) + powf((wcT),2);
    filt -> a[1] = 2.0f * powf((wcT),2) - 8.0f;
    filt -> a[2] = 4.0f - (2.0f * (gain/q_width) * wcT) + powf((wcT),2);
    filt -> b[0] = 1.0f / (4.0f + 2.0f / q_width * wcT + wcT * wcT);
    filt -> b[1] = -(2.0f * wcT * wcT - 8.0f);
    filt -> b[2] = -(4.0f - 2.0f / q_width * wcT + wcT * wcT);
}

void peaking_filter_set_param(peaking_filter_data *filt, float a0, float a1, float a2, float b0, float b1, float b2) {
    filt -> a[0] = a0;
    filt -> a[1] = a1;
    filt -> a[2] = a2;
    filt -> b[0] = b0;
    filt -> b[1] = b1;
    filt -> b[2] = b2;
}

float peaking_filter_update(peaking_filter_data *filt, float in) {
    // Update filter memory
    filt -> x[2] = filt -> x[1];
    filt -> x[1] = filt -> x[0];
    filt -> x[0] = in;
    filt -> y[2] = filt -> y[1];
    filt -> y[1] = filt -> y[0];

    // Compute filter output
    filt -> y[0] = ((filt -> a[0] * filt -> x[0] + filt -> a[1] * filt -> x[1] + filt -> a[2] * filt -> x[2]) + (filt -> b[1] * filt -> y[1] + filt -> b[2] * filt -> y[2])) * filt -> b[0]; // / filt -> b[0];
    return filt -> y[0];
}