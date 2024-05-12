/*
 * peaking_filter.c
 *
 *  Created on: May 12, 2024
 *      Author: phuta
 */

#define _USE_MATH_DEFINES
#include "peaking_filter.h"
//#include <stdio.h>

void peaking_filter_init(peaking_filter *filt, float sample_rate) {
    // Compute the sample time
    filt -> sample_time = 1.0f / sample_rate;

    // Clear filter memory
    for (uint8_t n = 0; n < 3; n++) {
        filt -> x[n] = 0.0f;
        filt -> y[n] = 0.0f;
    }

    // Calculate default filter coefficients (All-pass filter)
    peaking_filter_set_params(filt, 1.0f, 0.0f, 1.0f);
}

void peaking_filter_set_params(peaking_filter *filt, float center_freq, float bandwidth, float boostcut) {
    // Convert Hz to rad/s (Pre-warping cutoff frequency for bilinear transform)
    float wcT = 2.0f * tan(M_PI * center_freq * filt -> sample_time);
    
    // Compute Quality factor (Q = fc / BW)
    float Q = center_freq / bandwidth;

    // Compute filter coefficients
    filt -> a[0] = 4.0f + 2.0f * (boostcut / Q) * wcT + wcT * wcT;
    filt -> a[1] = 2.0f * wcT * wcT - 8.0f;
    filt -> a[2] = 4.0f - 2.0f * (boostcut / Q) * wcT + wcT * wcT;
    filt -> b[0] = 1.0f / (4.0f + 2.0f / Q * wcT + wcT * wcT);
    filt -> b[1] = -(2.0f * wcT * wcT - 8.0f);
    filt -> b[2] = -(4.0f - 2.0f / Q * wcT + wcT * wcT);
}

float peaking_filter_update(peaking_filter *filt, float in) {
    // Update filter memory
    filt -> x[2] = filt -> x[1];
    filt -> x[1] = filt -> x[0];
    filt -> x[0] = in;
    filt -> y[2] = filt -> y[1];
    filt -> y[1] = filt -> y[0];

    // Compute filter output
    filt -> y[0] = (filt -> a[0] * filt -> x[0] + filt -> a[1] * filt -> x[1] + filt -> a[2] * filt -> x[2] + (filt -> b[1] * filt -> y[1] + filt -> b[2] * filt -> y[2])) * filt -> b[0];
    
    // Print coefficients
//    printf("a0: %f, a1: %f, a2: %f, b0: %f, b1: %f, b2: %f\n", filt -> a[0], filt -> a[1], filt -> a[2], filt -> b[0], filt -> b[1], filt -> b[2]);
    return filt -> y[0];
}
