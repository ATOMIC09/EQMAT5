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
    float x[3];
    float y[3];
    float a[3];
    float b[3];

} peaking_filter_data;

void peaking_filter_init(peaking_filter_data *filt);
void peaking_filter_cal_coeff(peaking_filter_data *filt, float gain, float centerFreq_hz, float q_width);
void peaking_filter_set_param(peaking_filter_data *filt, float a0, float a1, float a2, float b0, float b1, float b2);
float peaking_filter_update(peaking_filter_data *filt, float in);

#endif /* INC_PEAKING_FILTER_H_ */
