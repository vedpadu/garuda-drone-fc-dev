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
uint8_t gyroBuf[2];
float accelXFloat = (float)0;
uint16_t accelRawX;
uint8_t readData = 0;

uint8_t radioDmaBuffer[8] = {0x00};
uint8_t packet_pointer_buf[4] = {0x00};
uint8_t receive_packet_status[4] = {0x00};

uint16_t rcData[8] = {0};

int packyCount = 0;
int countMicros = 0;
int packetArrived = 0;




// figure out where to put these, these are specific to the motor and the gyro.. not necessary to be here
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim5) {
			updateESC();
	  }else if(htim == &htim9){
		  setLastClockTime(micros(), &countMicros);
		  clockPhaseUpdate(0);
		  doFhssIrq();

		  /*if(doFhssIrq()){
			  //char* data4 = "FHSS\n";
			  //CDC_Transmit_FS((uint8_t *)data4, strlen(data4));

		  }*/

		  // Timer for radio
		  //countMicros++;
		  //countMicros++;
	  }else if(htim == &htim10){
		  //countMicros++;
		  if(packetArrived){

			  expressLrsSetRcDataFromPayload(rcData);
			  /*HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);
			  uint8_t transmit_buf_3[2] = {0x1D, 0x00};
			  HAL_SPI_Transmit(&hspi3, transmit_buf_3, 2, 10);
			  HAL_SPI_Receive(&hspi3, receive_packet_status, 4, 10); // get pin id n stuff <- huh
			  HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);*/

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
	  packyCount+=1;
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
	uint16_t buf[4] = {0};
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
  TIM3->CCR4 = 50;
  packyCount = 0;

//  HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);
//  HAL_Delay(1);
//  HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);
  //uint8_t *data = "SPI WORKIG\n";

  //uint8_t CHIP_ID_READ[2] = {0x80 | 0x00, 0x00};
  initExpressLRS();
  IMUInit();
  //initSX1280();
  //initSX1280();
//  uint8_t k = 0;
//
//  uint8_t fhssSeq[256] = {0};
//  int32_t freqOffset;
//  uint32_t currentFreq;
//  uint8_t nonceRX;
  /*HAL_FLASH_Unlock();
	FLASH_Erase_Sector(FLASH_SECTOR_7, VOLTAGE_RANGE_3);
	HAL_FLASH_Lock();*/
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
		/*uint8_t getStatus = 0xC0;
		uint8_t status_buf[1] = {0x00};
		HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi3, &getStatus, 1, 10);
		HAL_SPI_Receive(&hspi3, status_buf, 1, 10);
		HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);*/
		//return status_buf[0];
		//displayInt("rateIndex", getRateIndex());
		/*uint8_t rateInd = getRateIndex();
		char* rateIndString = convert(&rateInd);
		char* toSend = malloc(10);
		toSend = strcat(toSend, rateIndString);
		toSend = strcat(toSend, "\n");
		CDC_Transmit_FS((uint8_t*)toSend, strlen(toSend));
		free(toSend);
		free(rateIndString);*/
		//displayInt("packetCount", packetCount);
		displayInts4("packetCount", packyCount, "timClicks", rcData[2], "phaseDoff", getPhaseDiff(), "offset", getOffset());
		//displayInt("micros", countMicros);
		//uint8_t val = initFlashMemoryConfig(3);
		/*char* dataMem = convert(&out_buf[4]);
		char* test3 = malloc(10);
		test3 = strcat(test3, dataMem);
		test3 = strcat(test3,"\n");*/
		//CDC_Transmit_FS((uint8_t*)test3, strlen(test3));
		//k++;
		if(spiWorking){
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_15);
		}
		//uint8_t busy = (uint8_t)HAL_GPIO_ReadPin(RX_SPI_BUSY_GPIO_Port, RX_SPI_BUSY_Pin);

		/*uint8_t clear_irq[3] = {0x97, 0xFF, 0xFF};
	    HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);
	    HAL_SPI_Transmit(&hspi3, clear_irq, 3, 10);
	    HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);




		// read 0x901
		uint8_t read_buf_2[5] = {0x00};
		uint8_t read_packet_len_buff[5] = {0x19, 0x09, 0x01, 0x00};
		HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi3, read_packet_len_buff, 5, 10);
		HAL_SPI_Receive(&hspi3, read_buf_2, 5, 10);
		HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);
		readData = receive_packet_status[0];*/
		//01011000 (01100000) -> 01000000 (01001000)


		/*char* data = convert(&radioDmaBuffer[0]);
		char* data2 = convert(&radioDmaBuffer[1]);
		char* data3 = convert(&radioDmaBuffer[2]);
		char* data4 = convert(&radioDmaBuffer[3]);
		char* data5 = convert(&radioDmaBuffer[4]);
		char* data6 = convert(&radioDmaBuffer[5]);
		char* data7 = convert(&radioDmaBuffer[6]);
		char* data8 = convert(&radioDmaBuffer[7]);
		char* data9 = convert(&radioDmaBuffer[0]);
		char* data10 = convert(&radioDmaBuffer[1]);
		char* data11 = convert(&radioDmaBuffer[2]);
		char* data12 = convert(&receive_packet_status[0]);
		char* data13 = convert(&receive_packet_status[1]);
		char* data14 = convert(&receive_packet_status[2]);
		char* data15 = convert(&receive_packet_status[3]);
		char* data16 = convert(&packet_pointer_buf[0]);
		char* data17 = convert(&packet_pointer_buf[1]);
		char* test = malloc(8 * 17 + 17 + 1 + 1);
		strcat(test, data);
		strcat(test, " ");
		strcat(test, data2);
		strcat(test, " ");
		strcat(test, data3);
		strcat(test, " ");
		strcat(test, data4);
		strcat(test, " ");
		strcat(test, data5);
		strcat(test, " ");
		strcat(test, data6);
		strcat(test, " ");
		strcat(test, data7);
		strcat(test, " ");
		strcat(test, data8);
		strcat(test, " ");
		strcat(test, data9);
		strcat(test, " ");
		strcat(test, data10);
		strcat(test, " ");
		strcat(test, data11);
		strcat(test, " ");
		strcat(test, data12);
		strcat(test, " ");
		strcat(test, data13);
		strcat(test, " ");
		strcat(test, data14);
		strcat(test, " ");
		strcat(test, data15);
		strcat(test, " ");
		strcat(test, data16);
		strcat(test, " ");
		strcat(test, data17);
		strcat(test, " ");
		strcat(test, "\n");

//test = malloc(9);

		free(test);
		free(data);
		free(data2);
		free(data3);
		free(data4);
		free(data5);
		free(data6);
		free(data7);
		free(data8);
		free(data9);
		free(data10);
		free(data11);
		free(data12);
		free(data13);
		free(data14);
		free(data15);
		free(data16);
		free(data17);*/

		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_14);
		last_tick_start = currentTick;
	}
		uint16_t temp = rcData[2] - 989;
		if(temp < 11 || temp > 2000){
			temp = 0;
		}
		//temp = 300;
		buf[0] = temp;
		buf[1] = temp;
		buf[2] = temp;
		buf[3] = temp;
		setMotorOutputs(buf);
		/*if(currentTick - startingTick > 13000){
			value = 200;
		}else{
			value = 200;
		}*/

	/*HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
	HAL_Delay(1000);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
	HAL_Delay(1000);*/

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
