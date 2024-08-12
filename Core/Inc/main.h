/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// should prolly just include tim.h
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim9;
extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi3;
extern uint8_t doBlink;
extern int countMicroTemp;

#include "arm_math.h"
#include "kalman.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RADIO_EXTI_Pin GPIO_PIN_13
#define RADIO_EXTI_GPIO_Port GPIOC
#define RADIO_EXTI_EXTI_IRQn EXTI15_10_IRQn
#define LED_0_Pin GPIO_PIN_14
#define LED_0_GPIO_Port GPIOC
#define LED_0_PLEASE_Pin GPIO_PIN_15
#define LED_0_PLEASE_GPIO_Port GPIOC
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOA
#define BIND_Pin GPIO_PIN_2
#define BIND_GPIO_Port GPIOB
#define BIND_EXTI_IRQn EXTI2_IRQn
#define USB_VBUS_Pin GPIO_PIN_9
#define USB_VBUS_GPIO_Port GPIOA
#define RX_SPI_BUSY_Pin GPIO_PIN_13
#define RX_SPI_BUSY_GPIO_Port GPIOA
#define SPI3_CS_Pin GPIO_PIN_15
#define SPI3_CS_GPIO_Port GPIOA
#define GYRO_EXTI_Pin GPIO_PIN_6
#define GYRO_EXTI_GPIO_Port GPIOB
#define RADIO_RST_Pin GPIO_PIN_9
#define RADIO_RST_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define CLOCK_FRQ 48000000
#define KALMAN_FILTER_NVIC_PRIO 5
#define KALMAN_FILTER_SAMPLE_RATE 100

char *convert(uint8_t *a);
void displayInt(char* desc, int val);
void displayInts(char* desc, int val, char* desc2, int val2);
void displayInts3(char* desc, int val, char* desc2, int val2, char* desc3, int val3);
void displayInts4(char* desc, int val, char* desc2, int val2, char* desc3, int val3, char* desc4, int val4);
void displayFloats4(char* desc, float32_t val, char* desc2, float32_t val2, char* desc3, float32_t val3, char* desc4, float32_t val4);
void dispImu(float32_t* gyr, float32_t* acc, float32_t timeDelt);
void dispEst(quaternion_t est);
void dispMatrixDebug(float32_t* mat);
void dispEuler(float32_t* eul);


uint32_t micros();
uint32_t getDeltaTime(uint32_t greater, uint32_t lesser);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
