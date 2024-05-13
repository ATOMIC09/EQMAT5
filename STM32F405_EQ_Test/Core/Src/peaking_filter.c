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

void peaking_filter_set_params(peaking_filter_data *filt, float gain, float centerFreq_hz, float q_width) {
    // Convert Hz to rad/s
    float centerFreq_rad = 2 * M_PI * centerFreq_hz;

    // Convert Hz to rad/s (Pre-warping cutoff frequency for bilinear transform)
    // float wcT = 2.0f * tan(M_PI * center_freq * filt -> sample_time);
    
    // Compute Quality factor (Q = fc / BW)
    // float Q = center_freq / bandwidth;

    // Compute filter coefficients
    filt -> a[0] = 4.0f + (2.0f * (gain / q_width) * centerFreq_rad * dt) + powf((centerFreq_rad * dt),2);
    filt -> a[1] = 2.0f * powf((centerFreq_rad * dt),2) - 8;
    filt -> a[2] = 4.0f - (2.0f * (gain/q_width) * centerFreq_rad * dt) + powf((centerFreq_rad * dt),2);
    filt -> b[0] = 4.0f + (2.0f / q_width * centerFreq_rad * dt) + powf((centerFreq_rad * dt),2);
    filt -> b[1] = -1.0f * (2.0f * powf((centerFreq_rad * dt),2) - 8);
    filt -> b[2] = -1.0f * (4.0f - (2.0f / q_width * centerFreq_rad * dt) + powf((centerFreq_rad * dt),2));
}

float peaking_filter_update(peaking_filter_data *filt, float in) {
    // Update filter memory
    filt -> x[2] = filt -> x[1];
    filt -> x[1] = filt -> x[0];
    filt -> x[0] = in;
    filt -> y[2] = filt -> y[1];
    filt -> y[1] = filt -> y[0];

    // Compute filter output
    filt -> y[0] = ((filt -> a[0] * filt -> x[0] + filt -> a[1] * filt -> x[1] + filt -> a[2] * filt -> x[2]) + (filt -> b[1] * filt -> y[1] + filt -> b[2] * filt -> y[2])) / filt -> b[0]; // / filt -> b[0];
    
    // Print coefficients
//    printf("a0: %f, a1: %f, a2: %f, b0: %f, b1: %f, b2: %f\n", filt -> a[0], filt -> a[1], filt -> a[2], filt -> b[0], filt -> b[1], filt -> b[2]);
    return filt -> y[0];
}
