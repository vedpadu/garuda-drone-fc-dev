/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include <flash_memory_handler.h>
#include <motor_mixer.h>
#include "main.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "string.h"
#include "math.h"
#include "imu.h"
#include "expresslrs.h"
#include "sx1280.h"
#include "esc.h"
#include "arm_math.h"
#include "kalman.h"
#include "button_handler.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
char *convert(uint8_t *a);
char *convert16(uint16_t *a);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t radioDmaBuffer[8] = {0x00};

uint16_t rcData[8] = {0,0,0,0,0,0,0,0};

int countMicros = 0;
int countMicroTemp = 0;
int countMotor = 0;

uint16_t mot_buf[4] = {0};

uint32_t lastKalmanTick = 0;

uint32_t lastTimePrint = 0;
int16_t motorCount = 0;
int gyroCtr = 0;
int kalmanCtr = 0;

void dispImuAndPID(float32_t* gyr, float32_t* acc, outRates_t pidRate, uint16_t* mot){
	int len = snprintf(NULL, 0, "imu,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%d,%d,%d\n", gyr[0], gyr[1], gyr[2], acc[0], acc[1], acc[2], pidRate.roll, pidRate.pitch, pidRate.yaw, mot[0], mot[1], mot[2], mot[3]);
	//int len2 = snprintf(NULL, 0, "%u", val2);
	char *str = malloc(len + 2);
	snprintf(str, len + 2, "imu,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%d,%d,%d\n", gyr[0], gyr[1], gyr[2], acc[0], acc[1], acc[2], pidRate.roll, pidRate.pitch, pidRate.yaw, mot[0], mot[1], mot[2], mot[3]);
	// do stuff with result
	CDC_Transmit_FS((uint8_t*)str, strlen(str));
	free(str);
}

// figure out where to put these, these are specific to the motor and the gyro.. not necessary to be here
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim5) {
		if(!motors_armed){
			arm_ESC();
		}else{
			float32_t currTick = micros();
			  float32_t deltTime = (float32_t)(getDeltaTime(currTick, lastKalmanTick)) * 0.000001;
			  lastKalmanTick = currTick;
			  updateKalman(gyro, accel, deltTime);
			  kalmanCtr++;
			  motorMixerOuterUpdate(estimate, accel);
//			  float32_t vec[3] = {0.0, 0.0, -1.0};
//			  quaternion_t inverseEst = quatInverse(estimate);
//				rotateVector3ByQuaternion(vec, inverseEst); // verify this is working
//				float32_t dot = vec[0] * accel[0] + vec[1] * accel[1] + vec[2] * accel[2];
				//displayFloats4("dot,", outThrott, "accel1", velEst, "accel1", accel[1], "accel2", accel[2]);
			  //dispImu(gyro, accel, deltTime);
			 // dispEst(estimate);
			  //dispImuAndPID(gyro, kalman_gyro, motorSetpoints, mot_buf);
		}

	  }else if(htim == &htim9){
		  // keep the internal clock in phase
		  setLastClockTime(micros());
		  clockPhaseUpdate(0);
		  doFhssIrq();
	  }else if(htim == &htim10){
		  if(bmi270_ready && kalman_initialized){
			  motorCount++;
			  if(!isDisconnected()){
				  // fast loop, gets inputs and does rate control
				  //TODO: inputs do not have to be here
				  expressLrsSetRcDataFromPayload(rcData);
				  motorMixerUpdate(rcData, mot_buf, gyro, accel, estimate);
				  set_esc_outputs(mot_buf);

			  }
		  }
	  }else if(htim == &htim11){
		  if(bmi270_ready && kalman_initialized){
			  readIMUData();
			  gyroCtr++;
		  }
	  }else if(htim == &htim3){
		  if(bmi270_ready && kalman_initialized){
			  //displayInts4("rc0", rcData[0], "rc1", rcData[1], "rc2", rcData[2], "rc3", rcData[3]);

				//displayFloats4("00", estimate.w, "01", estimate.vec[0], "02", estimate.vec[1], "03", estimate.vec[2]);
			  	uint32_t currTime = micros();
//			  	float32_t vec[3] = {0.0, 0.0, -1.0};
//				quaternion_t inverseEst = quatInverse(estimate);
//				rotateVector3ByQuaternion(vec, inverseEst); // verify this is working
//				float32_t dot = vec[0] * accel[0] + vec[1] * accel[1] + vec[2] * accel[2];
				//displayFloats4("dot", dot, "accel1", accel[0], "accel1", accel[1], "accel2", accel[2]);
			  	//dispImu(gyro, accel, 0.1);
			  	//displayFloats4("r", motorSetpoints.roll, "p", motorSetpoints.pitch, "y", motorSetpoints.yaw, "t", motorSetpoints.throttle);
			  	//displayFloats4("gyro", (float)countGyros, "motorUpdate", gyro[0], "kalman", gyroPreFilt[0], "delta", (float)(getDeltaTime(currTime, lastTimePrint))/1000);
				//displayInts4("gyro", gyroCtr, "motorUpdate", motorCount, "kalman", kalmanCtr, "delta", (getDeltaTime(currTime, lastTimePrint))/1000);
				lastTimePrint = currTime;
				if(!isBindingMode()){
					HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_15);
				}

				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_14);
		  }

	  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
   if (GPIO_Pin == GPIO_PIN_13) {
	  // use dma?
	  // switch to a function in the class
	  // atleast use interrupt mode man..
	  setLastPacketTime(micros());
	  sx1280_read_interrupt(radioDmaBuffer);

	  processRFPacket((uint8_t *)radioDmaBuffer, micros());

  } else if(GPIO_Pin == GPIO_PIN_2){
	  // add erase flash sector functionality on multiple presses as well as clearing bind mode
	  char* data4 = "BIND\n";
	  CDC_Transmit_FS((uint8_t *)data4, strlen(data4));
	  processButtonPress(micros());


  } else {
      __NOP();
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_USB_DEVICE_Init();
  MX_SPI1_Init();
  MX_SPI3_Init();
  MX_TIM9_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim3); // debug loop and LEDs
  HAL_TIM_Base_Start_IT(&htim9); // clock express lrs phase // 250 hz atm
  HAL_TIM_Base_Start_IT(&htim5); // esc // 4000 hz
  HAL_Delay(1); // Delays are to make sure the timers are not in phase.
  HAL_TIM_Base_Start_IT(&htim11); // motor // trying to get 1000 hz // this clock only psc work?
  HAL_Delay(1);
  HAL_TIM_Base_Start_IT(&htim10); // imu 100 hz

  // initialize peripherals
  initExpressLRS();
  IMUInit();
  motorMixerInit();
  quaternion_t initEst = {1.0, {0.0, 0.0, 0.0}};
  lastKalmanTick = micros();
  initKalman(initEst, 0.0, 0.004 , 0.000005, 0.005, 0.0, 0.01);

  // start the microsecond clock
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= 1;
  DWT->CYCCNT = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
char *convert(uint8_t *a)
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

char *convert16(uint16_t *a)
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

void displayInt(char* desc, int val){
	int len = snprintf(NULL, 0, "%s,%d\n", desc, val);
	//int len2 = snprintf(NULL, 0, "%u", val2);
	char *str = malloc(len + 2);
	snprintf(str, len + 2, "%s,%d\n", desc, val);
	// do stuff with result
	CDC_Transmit_FS((uint8_t*)str, strlen(str));
	free(str);
}

void displayInts(char* desc, int val, char* desc2, int val2){
	int len = snprintf(NULL, 0, "%s: %d %s: %d\n", desc, val, desc2, val2);
	//int len2 = snprintf(NULL, 0, "%u", val2);
	char *str = malloc(len + 2);
	snprintf(str, len + 2, "%s: %d %s: %d\n", desc, val, desc2, val2);
	// do stuff with result
	CDC_Transmit_FS((uint8_t*)str, strlen(str));
	free(str);
}

void displayInts3(char* desc, int val, char* desc2, int val2, char* desc3, int val3){
	int len = snprintf(NULL, 0, "%s: %d %s: %d %s: %d\n", desc, val, desc2, val2, desc3, val3);
	//int len2 = snprintf(NULL, 0, "%u", val2);
	char *str = malloc(len + 2);
	snprintf(str, len + 2, "%s: %d %s: %d %s: %d\n", desc, val, desc2, val2, desc3, val3);
	// do stuff with result
	CDC_Transmit_FS((uint8_t*)str, strlen(str));
	free(str);
}

void displayInts4(char* desc, int val, char* desc2, int val2, char* desc3, int val3, char* desc4, int val4){
	int len = snprintf(NULL, 0, "%s: %d %s: %d %s: %d %s: %d\n", desc, val, desc2, val2, desc3, val3, desc4, val4);
	//int len2 = snprintf(NULL, 0, "%u", val2);
	char *str = malloc(len + 2);
	snprintf(str, len + 2, "%s: %d %s: %d %s: %d %s: %d\n", desc, val, desc2, val2, desc3, val3, desc4, val4);
	// do stuff with result
	CDC_Transmit_FS((uint8_t*)str, strlen(str));
	free(str);
}

void displayFloats4(char* desc, float32_t val, char* desc2, float32_t val2, char* desc3, float32_t val3, char* desc4, float32_t val4){
	int len = snprintf(NULL, 0, "%s: %f %s: %f %s: %f %s: %f\n", desc, val, desc2, val2, desc3, val3, desc4, val4);
	//int len2 = snprintf(NULL, 0, "%u", val2);
	char *str = malloc(len + 2);
	snprintf(str, len + 2, "%s: %f %s: %f %s: %f %s: %f\n", desc, val, desc2, val2, desc3, val3, desc4, val4);
	// do stuff with result
	CDC_Transmit_FS((uint8_t*)str, strlen(str));
	free(str);
}

void dispImu(float32_t* gyr, float32_t* acc, float32_t timeDelt){
	int len = snprintf(NULL, 0, "imu,%f,%f,%f,%f,%f,%f,%f\n", gyr[0], gyr[1], gyr[2], acc[0], acc[1], acc[2], timeDelt);
	//int len2 = snprintf(NULL, 0, "%u", val2);
	char *str = malloc(len + 2);
	snprintf(str, len + 2, "imu,%f,%f,%f,%f,%f,%f,%f\n", gyr[0], gyr[1], gyr[2], acc[0], acc[1], acc[2], timeDelt);
	// do stuff with result
	CDC_Transmit_FS((uint8_t*)str, strlen(str));
	free(str);
}



void dispEuler(float32_t* eul){
	int len = snprintf(NULL, 0, "euler,%f,%f,%f\n", eul[0], eul[1], eul[2]);
	//int len2 = snprintf(NULL, 0, "%u", val2);
	char *str = malloc(len + 2);
	snprintf(str, len + 2, "euler,%f,%f,%f\n", eul[0], eul[1], eul[2]);
	// do stuff with result
	CDC_Transmit_FS((uint8_t*)str, strlen(str));
	free(str);
}

void dispEst(quaternion_t est){
	int len = snprintf(NULL, 0, "est,%f,%f,%f,%f\n", est.w, est.vec[0], est.vec[1], est.vec[2]);
	//int len2 = snprintf(NULL, 0, "%u", val2);
	char *str = malloc(len + 2);
	snprintf(str, len + 2, "est,%f,%f,%f,%f\n", est.w, est.vec[0], est.vec[1], est.vec[2]);
	// do stuff with result
	CDC_Transmit_FS((uint8_t*)str, strlen(str));
	free(str);
}

void dispMatrixDebug(float32_t* mat){
	int len = snprintf(NULL, 0, "mat,%f,%f,%f,%f,%f,%f,%f\n", mat[0], mat[1], mat[2], mat[3], mat[4], mat[5], mat[6]);
	//int len2 = snprintf(NULL, 0, "%u", val2);
	char *str = malloc(len + 2);
	snprintf(str, len + 2, "mat,%f,%f,%f,%f,%f,%f,%f\n", mat[0], mat[1], mat[2], mat[3], mat[4], mat[5], mat[6]);
	// do stuff with result
	CDC_Transmit_FS((uint8_t*)str, strlen(str));
	free(str);
}

uint32_t micros(){
	return (DWT->CYCCNT/48);
}

uint32_t getDeltaTime(uint32_t greater, uint32_t lesser){
	if(greater < lesser){
		greater += (0xFFFFFFFF/48);
	}
	return greater - lesser;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
