/*
 * biquadlpf.h
 *
 *  Created on: Aug 2, 2024
 *      Author: vedpa
 */

#ifndef INC_BIQUADLPF_H_
#define INC_BIQUADLPF_H_

#include "arm_math.h"
#include "math.h"
//float32_t gyroLpfDelay[4][3] = {0.0};


typedef struct {

	/* Controller gains */
	float32_t x_n1[3];
	float32_t x_n2[3];
	float32_t y_n1[3];
	float32_t y_n2[3];

	/* Derivative low-pass filter time constant */
	float32_t cutoff_freq;
	float32_t sample_freq;

	float32_t b0;
	float32_t b1;
	float32_t b2;
	float32_t a1;
	float32_t a2;

} BiquadLPF;

void assignVector(float32_t* out, float32_t* in);
void biquadLPFApply(BiquadLPF* filter, float32_t x_n[3], float32_t y_n[3]);
void biquadLPFInit(BiquadLPF* filter, float cutoff_freq, float sample_freq);

#endif /* INC_BIQUADLPF_H_ */
