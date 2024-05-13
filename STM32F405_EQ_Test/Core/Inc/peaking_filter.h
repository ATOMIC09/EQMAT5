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
} peaking_filter_data;

void peaking_filter_init(peaking_filter_data *filt);
void peaking_filter_set_params(peaking_filter_data *filt, float center_freq, float bandwidth, float boostcut);
float peaking_filter_update(peaking_filter_data *filt, float in);

#endif /* INC_PEAKING_FILTER_H_ */
