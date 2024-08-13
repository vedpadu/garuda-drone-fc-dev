/*
 * com_debugging.c
 *
 *  Created on: Aug 12, 2024
 *      Author: vedpa
 */

#include "com_debugging.h"

void displayInt(char* desc, int val)
{
	int len = snprintf(NULL, 0, "%s,%d\n", desc, val);
	char *str = malloc(len + 2);
	snprintf(str, len + 2, "%s,%d\n", desc, val);
	// do stuff with result
	CDC_Transmit_FS((uint8_t*)str, strlen(str));
	free(str);
}

/* USER CODE BEGIN 4 */
char *convertUint8(uint8_t *a)
{
  char* buffer2;
  int i;

  buffer2 = malloc(9);
  if (!buffer2)
    return NULL;

  buffer2[8] = 0;
  for (i = 0; i <= 7; i++)
    buffer2[7 - i] = (((*a) >> i) & (0x01)) + '0';

  puts(buffer2);

  return buffer2;
}

char *convertUint16(uint16_t *a)
{
  char* buffer2;
  int i;

  buffer2 = malloc(17);
  if (!buffer2)
    return NULL;

  buffer2[16] = 0;
  for (i = 0; i <= 15; i++)
    buffer2[15 - i] = (((*a) >> i) & (0x01)) + '0';

  puts(buffer2);

  return buffer2;
}

void displayInts(char* desc, int val, char* desc2, int val2)
{
	int len = snprintf(NULL, 0, "%s: %d %s: %d\n", desc, val, desc2, val2);
	char *str = malloc(len + 2);
	snprintf(str, len + 2, "%s: %d %s: %d\n", desc, val, desc2, val2);
	CDC_Transmit_FS((uint8_t*)str, strlen(str));
	free(str);
}

void displayInts3(char* desc, int val, char* desc2, int val2, char* desc3, int val3)
{
	int len = snprintf(NULL, 0, "%s: %d %s: %d %s: %d\n", desc, val, desc2, val2, desc3, val3);
	char *str = malloc(len + 2);
	snprintf(str, len + 2, "%s: %d %s: %d %s: %d\n", desc, val, desc2, val2, desc3, val3);
	CDC_Transmit_FS((uint8_t*)str, strlen(str));
	free(str);
}

void displayInts4(char* desc, int val, char* desc2, int val2, char* desc3, int val3, char* desc4, int val4)
{
	int len = snprintf(NULL, 0, "%s: %d %s: %d %s: %d %s: %d\n", desc, val, desc2, val2, desc3, val3, desc4, val4);
	char *str = malloc(len + 2);
	snprintf(str, len + 2, "%s: %d %s: %d %s: %d %s: %d\n", desc, val, desc2, val2, desc3, val3, desc4, val4);
	CDC_Transmit_FS((uint8_t*)str, strlen(str));
	free(str);
}

void displayFloats4(char* desc, float32_t val, char* desc2, float32_t val2, char* desc3, float32_t val3, char* desc4, float32_t val4)
{
	int len = snprintf(NULL, 0, "%s: %f %s: %f %s: %f %s: %f\n", desc, val, desc2, val2, desc3, val3, desc4, val4);
	char *str = malloc(len + 2);
	snprintf(str, len + 2, "%s: %f %s: %f %s: %f %s: %f\n", desc, val, desc2, val2, desc3, val3, desc4, val4);
	CDC_Transmit_FS((uint8_t*)str, strlen(str));
	free(str);
}

void dispImu(float32_t* gyr, float32_t* acc, float32_t timeDelt)
{
	int len = snprintf(NULL, 0, "imu,%f,%f,%f,%f,%f,%f,%f\n", gyr[0], gyr[1], gyr[2], acc[0], acc[1], acc[2], timeDelt);
	char *str = malloc(len + 2);
	snprintf(str, len + 2, "imu,%f,%f,%f,%f,%f,%f,%f\n", gyr[0], gyr[1], gyr[2], acc[0], acc[1], acc[2], timeDelt);
	CDC_Transmit_FS((uint8_t*)str, strlen(str));
	free(str);
}

void dispEuler(float32_t* eul)
{
	int len = snprintf(NULL, 0, "euler,%f,%f,%f\n", eul[0], eul[1], eul[2]);
	char *str = malloc(len + 2);
	snprintf(str, len + 2, "euler,%f,%f,%f\n", eul[0], eul[1], eul[2]);
	CDC_Transmit_FS((uint8_t*)str, strlen(str));
	free(str);
}

void dispEst(quaternion_t est)
{
	int len = snprintf(NULL, 0, "est,%f,%f,%f,%f\n", est.w, est.vec[0], est.vec[1], est.vec[2]);
	char *str = malloc(len + 2);
	snprintf(str, len + 2, "est,%f,%f,%f,%f\n", est.w, est.vec[0], est.vec[1], est.vec[2]);
	CDC_Transmit_FS((uint8_t*)str, strlen(str));
	free(str);
}

void dispMatrixDebug(float32_t* mat)
{
	int len = snprintf(NULL, 0, "mat,%f,%f,%f,%f,%f,%f,%f\n", mat[0], mat[1], mat[2], mat[3], mat[4], mat[5], mat[6]);
	char *str = malloc(len + 2);
	snprintf(str, len + 2, "mat,%f,%f,%f,%f,%f,%f,%f\n", mat[0], mat[1], mat[2], mat[3], mat[4], mat[5], mat[6]);
	CDC_Transmit_FS((uint8_t*)str, strlen(str));
	free(str);
}
