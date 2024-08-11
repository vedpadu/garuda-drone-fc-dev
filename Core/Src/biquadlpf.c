/*
 * biquadlpf.c
 * Biquad low pass filter in C. Used for gyro measurements
 *
 *  Created on: Aug 2, 2024
 *      Author: vedpadu
 */

#include "biquadlpf.h"

void biquadLPFInit(BiquadLPF* filter, float cutoffFreq, float sampleFreq){
	// initialize variables
	float32_t empty[3] = {0};
	assignVector(filter->x_n1, empty);
	assignVector(filter->x_n2, empty);
	assignVector(filter->y_n1, empty);
	assignVector(filter->y_n2, empty);

	filter->cutoff_freq = cutoffFreq;
	filter->sample_freq = sampleFreq;

	// compute coefficients from cutoff frequency
	float32_t Q = 1.0/sqrt(2.0);
	float32_t omega = 2 * M_PI * cutoffFreq / sampleFreq;
	float32_t alpha = sin(omega) / (2 * Q);
	float32_t a0 = 1 + alpha;

	filter->b0 = (1 - cos(omega)) / 2.0;
	filter->b1 = 1 - cos(omega);
	filter->b2 = filter->b0;
	filter->a1 = -2.0 * cos(omega);
	filter->a2 = 1 - alpha;

	filter->b0 /= a0;
	filter->b1 /= a0;
	filter->b2 /= a0;
	filter->a1 /= a0;
	filter->a2 /= a0;
}

void biquadLPFApply(BiquadLPF* filter, float32_t x_n[3], float32_t y_n[3]){
	y_n[0] = filter->b0 * x_n[0] + filter->b1 * filter->x_n1[0] + filter->b2 * filter->x_n2[0] - filter->a1 * filter->y_n1[0] - filter->a2 * filter->y_n2[0];
	y_n[1] = filter->b0 * x_n[1] + filter->b1 * filter->x_n1[1] + filter->b2 * filter->x_n2[1] - filter->a1 * filter->y_n1[1] - filter->a2 * filter->y_n2[1];
	y_n[2] = filter->b0 * x_n[2] + filter->b1 * filter->x_n1[2] + filter->b2 * filter->x_n2[2] - filter->a1 * filter->y_n1[2] - filter->a2 * filter->y_n2[2];

	assignVector(filter->y_n2, filter->y_n1);
	assignVector(filter->y_n1, y_n);
	assignVector(filter->x_n2, filter->x_n1);
	assignVector(filter->x_n1, x_n);
}

// TODO: put in util
void assignVector(float32_t* out, float32_t* in){
	out[0] = in[0];
	out[1] = in[1];
	out[2] = in[2];
}
