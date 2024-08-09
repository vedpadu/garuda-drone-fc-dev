/*
 * outputHandler.h
 *
 *  Created on: Aug 8, 2024
 *      Author: vedpadu
 */

#ifndef INC_OUTPUTHANDLER_H_
#define INC_OUTPUTHANDLER_H_

#include "motorMixer.h"
#include "math_util.h"

void initOutputHandler(float32_t maxVel, float32_t maxAccel);
void outputUpdate(outRates_t* out);

#endif /* INC_OUTPUTHANDLER_H_ */
