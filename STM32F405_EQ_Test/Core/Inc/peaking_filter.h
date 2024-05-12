/*
 * peaking_filter.h
 *
 *  Created on: May 12, 2024
 *      Author: phuta
 */

#ifndef INC_PEAKING_FILTER_H_
#define INC_PEAKING_FILTER_H_

#include <math.h>
#include <stdint.h>

typedef struct {
    float sample_time;
    float x[3];
    float y[3];
    float a[3];
    float b[3];
} peaking_filter;

void peaking_filter_init(peaking_filter *filt, float sample_rate);
void peaking_filter_set_params(peaking_filter *filt, float center_freq, float bandwidth, float boostcut);
float peaking_filter_update(peaking_filter *filt, float in);

#endif /* INC_PEAKING_FILTER_H_ */
