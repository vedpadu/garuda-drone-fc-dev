/*
 * com_debugging.h
 *
 *  Created on: Aug 12, 2024
 *      Author: vedpa
 */

#ifndef INC_COM_DEBUGGING_H_
#define INC_COM_DEBUGGING_H_

#include "usbd_cdc_if.h"
#include "stdlib.h"

char *convertUint8(uint8_t *a);
char *convertUint16(uint16_t *a);
void displayInt(char* desc, int val);
void displayInts(char* desc, int val, char* desc2, int val2);
void displayInts3(char* desc, int val, char* desc2, int val2, char* desc3, int val3);
void displayInts4(char* desc, int val, char* desc2, int val2, char* desc3, int val3, char* desc4, int val4);
void displayFloats4(char* desc, float32_t val, char* desc2, float32_t val2, char* desc3, float32_t val3, char* desc4, float32_t val4);
void dispImu(float32_t* gyr, float32_t* acc, float32_t timeDelt);
void dispEst(quaternion_t est);
void dispMatrixDebug(float32_t* mat);
void dispEuler(float32_t* eul);



#endif /* INC_COM_DEBUGGING_H_ */
