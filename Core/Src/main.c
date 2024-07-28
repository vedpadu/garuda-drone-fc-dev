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
#include "flashMemoryConfig.h"
#include "esc.h"
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
void dshot600(uint32_t *motor, uint16_t value);
char *convert(uint8_t *a);
char *convert16(uint16_t *a);
float lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t last_tick_start = 0;

uint8_t radioDmaBuffer[8] = {0x00};
uint8_t packet_pointer_buf[4] = {0x00};
uint8_t receive_packet_status[4] = {0x00};

uint16_t rcData[8] = {0};

int countMicros = 0;
int packetArrived = 0;
int countMotor = 0;
uint16_t mot_buf[4] = {0};
uint8_t doBlink = 0;

int16_t oldRCVal = 48;
int startTick = 0;

// figure out where to put these, these are specific to the motor and the gyro.. not necessary to be here
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim5) {
		updateESC();
	  }else if(htim == &htim9){
		  setLastClockTime(micros());
		  clockPhaseUpdate(0);
		  doFhssIrq();

		  // Timer for radio
		  countMicros++;
	  }else if(htim == &htim10){
		  if(packetArrived){


			  expressLrsSetRcDataFromPayload(rcData);
				int16_t tempDat = (rcData[2] - 989)*2;
				if(tempDat < 50){
					tempDat = 48;
				}else if(tempDat > 2047){
					tempDat = 2047;
				}
				if(oldRCVal < tempDat){
					if(tempDat - oldRCVal > 30){
						tempDat = oldRCVal + 30;
					}
				}else{
					if(oldRCVal - tempDat > 30){
						tempDat = oldRCVal - 30;
					}
				}
				if(tempDat < 50){
					tempDat = 48;
				}else if(tempDat > 2047){
					tempDat = 2047;
				}
				oldRCVal = tempDat;
				//tempDat = 1200;
				//valMot = (uint16_t)tempDat;
				mot_buf[0] = tempDat;
				mot_buf[1] = tempDat;
				mot_buf[2] = tempDat;
				mot_buf[3] = tempDat;
				setMotorOutputs(mot_buf);

		  }

		  //countMicros++;
		  //handleConnectionState(micros());
	  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == GPIO_PIN_6) {
	  readIMUData();
  } else if (GPIO_Pin == GPIO_PIN_13) {
	  // use dma?
	  setLastPacketTime(micros());
	  uint8_t clear_irq[3] = {0x97, 0xFF, 0xFF};
	  HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);
	  HAL_SPI_Transmit(&hspi3, clear_irq, 3, 10);
	  HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);

	  HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET); // get pointer
	  uint8_t transmit_buf[4] = {0x17, 0x00, 0x00};
	  HAL_SPI_Transmit(&hspi3, transmit_buf, 4, 10);
	  HAL_SPI_Receive(&hspi3, packet_pointer_buf, 4, 10);
	  HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);

	  HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET); // read buffer data
	  uint8_t transmit_buf_2[11] = {0x1B, packet_pointer_buf[0] - 16, 0x00};
	  HAL_SPI_Transmit(&hspi3, transmit_buf_2, 11, 1);
	  HAL_SPI_Receive(&hspi3, transmit_buf_2, 11, 10);
	  HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);
	  memcpy((uint8_t *) radioDmaBuffer, (uint8_t *) transmit_buf_2, 8);

	  processRFPacket((uint8_t *)radioDmaBuffer, micros());
	  packetArrived = 1;

  } else if(GPIO_Pin == GPIO_PIN_2){
	  char* data4 = "BIND\n";
	  CDC_Transmit_FS((uint8_t *)data4, strlen(data4));
	  setBindingMode();

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
  /* USER CODE BEGIN 2 */
  uint32_t startingTick = HAL_GetTick();
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  HAL_TIM_Base_Start_IT(&htim9);
  HAL_TIM_Base_Start_IT(&htim5);
  HAL_TIM_Base_Start_IT(&htim10);


  // tim 2 channel 1 only

  // circular mode notes
  /*__HAL_RCC_TIM2_CLK_ENABLE();
  TIM2->PSC = (uint16_t)3;
  TIM2->ARR = (uint32_t)19;
  TIM2->CCMR1 = TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE;
  //TIM2->CCMR1 |= TIM_CCMR1_OC1PE;

  TIM2->CCER = TIM_CCER_CC1E;

  TIM2->DIER = TIM_DIER_CC1DE;

  TIM2->CR1 = TIM_CR1_ARPE;

  //TIM2->CCER |

  __HAL_RCC_DMA1_CLK_ENABLE();


  // dma 1 stream 5
  DMA1_Stream5->CR = DMA_SxCR_MSIZE_1 | DMA_SxCR_PSIZE_1 | DMA_SxCR_MINC | DMA_SxCR_CIRC| DMA_SxCR_DIR_0 | DMA_CHANNEL_3 | DMA_SxCR_PL_1;


  //DMA1_Stream5->CMAR = (uint32_t)motorDMABuf;
  //DMA1_Channel5->CNDTR = (uint16_t)18;
  DMA1_Stream5->M0AR = (uint32_t)motorDMABuf;
  DMA1_Stream5->NDTR |= (uint32_t)18;
  DMA1_Stream5->PAR = (uint32_t)(&TIM2->CCR1);

  //dshot600(motorDMABuf, 48);

  //DMA1_Stream5->CR |= DMA_IT_TC | DMA_IT_TE | DMA_IT_DME;

  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  //DMA1_Stream5->PAR = (uint32_t)(&TIM2->CCR1);*/

  //HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, motorDMABuf, 18);


  startTick = HAL_GetTick();

  initExpressLRS();
  IMUInit();

  // clock micros init
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= 1;
  DWT->CYCCNT = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // all just debug here, can be cleaned. Tomorrow write gyro class, so that switching up the gyro can be relatively easy.
  while (1)
  {
	uint32_t currentTick = HAL_GetTick();
	if(currentTick - last_tick_start > 1000)
	{
		displayInts4("test1", countGyros, "rc", rcData[2], "test2", countMotor, "offset", getOffset());
		if(doBlink){
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_15);
		}
		//uint8_t busy = (uint8_t)HAL_GPIO_ReadPin(RX_SPI_BUSY_GPIO_Port, RX_SPI_BUSY_Pin);

		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_14);
		last_tick_start = currentTick;
	}
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
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
	int len = snprintf(NULL, 0, "%u", val);
	char *str = malloc(len + 2 + strlen(desc) + 6);
	snprintf(str, len + 2, "%u", val);
	strcat(str, ": ");
	strcat(str, desc);
	strcat(str, "\n");
	// do stuff with result
	CDC_Transmit_FS((uint8_t*)str, strlen(str));
	free(str);
}

void displayInts(char* desc, int val, char* desc2, int val2){
	int len = snprintf(NULL, 0, "%s: %u %s: %u\n", desc, val, desc2, val2);
	//int len2 = snprintf(NULL, 0, "%u", val2);
	char *str = malloc(len + 2);
	snprintf(str, len + 2, "%s: %u %s: %u\n", desc, val, desc2, val2);
	// do stuff with result
	CDC_Transmit_FS((uint8_t*)str, strlen(str));
	free(str);
}

void displayInts3(char* desc, int val, char* desc2, int val2, char* desc3, int val3){
	int len = snprintf(NULL, 0, "%s: %u %s: %u %s: %u\n", desc, val, desc2, val2, desc3, val3);
	//int len2 = snprintf(NULL, 0, "%u", val2);
	char *str = malloc(len + 2);
	snprintf(str, len + 2, "%s: %u %s: %u %s: %u\n", desc, val, desc2, val2, desc3, val3);
	// do stuff with result
	CDC_Transmit_FS((uint8_t*)str, strlen(str));
	free(str);
}

void displayInts4(char* desc, int val, char* desc2, int val2, char* desc3, int val3, char* desc4, int val4){
	int len = snprintf(NULL, 0, "%s: %d %s: %u %s: %u %s: %d\n", desc, val, desc2, val2, desc3, val3, desc4, val4);
	//int len2 = snprintf(NULL, 0, "%u", val2);
	char *str = malloc(len + 2);
	snprintf(str, len + 2, "%s: %d %s: %u %s: %u %s: %d\n", desc, val, desc2, val2, desc3, val3, desc4, val4);
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
