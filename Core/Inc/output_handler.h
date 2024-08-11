/*
 * outputHandler.h
 *
 *  Created on: Aug 8, 2024
 *      Author: vedpadu
 */

#ifndef INC_OUTPUT_HANDLER_H_
#define INC_OUTPUT_HANDLER_H_

#include <motor_mixer.h>
#include "math_util.h"

void initOutputHandler(float32_t maxVel, float32_t maxAccel);
void outputUpdate(outRates_t* out);

#endif /* INC_OUTPUT_HANDLER_H_ */
