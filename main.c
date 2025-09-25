/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "adc.h"
#include "i2c.h"
#include "i2s.h"
#include "spi.h"
#include "usart.h"
#include "usb_host.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Audio_Drivers.h"
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include "main.h"
#include "spi.h"
#include "gpio.h"
#include "rgbColourSensor.h"
#include "adc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PBSIZE 4096
#define SYNTHSIZE 2048
#define PI 3.141592653589793


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int16_t PlayBuff[PBSIZE];
int16_t SineBuff[SYNTHSIZE];
int16_t RampBuff[SYNTHSIZE];
int16_t SquareBuff[SYNTHSIZE];
int16_t SoundBuff[SYNTHSIZE];
float currentPhase = 0.0;
float frequency = 0;
float phaseInc;
int16_t colours[3] = {0,0,0};
//enum eNoteStatus{ready, going, finish} noteStatus = ready;
enum eBufferStatus{empty, finished, firstHalfReq, firstHalfDone,secondHalfReq, secondHalfDone} bufferStatus = firstHalfReq;


//variables for ADC output, including the msg variable for serial output
int16_t amplitude_pot;
float amplitude_ratio=1;
char msg[20];


// Audio callbacks:
void myAudioHalfTransferCallback(void) {
	bufferStatus = firstHalfReq;
}
void myAudioTransferCompleteCallback(void) {
	myAudioChangeBuffer(PlayBuff, PBSIZE);
 	bufferStatus = secondHalfReq;
}
void setFrequencyCallback(int f){ //Constantly sets the frequency and phase increment.
	 currentPhase = 0.0;
	 frequency = f;
	 phaseInc = SYNTHSIZE * (frequency/AUDIO_FREQUENCY_44K);
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void PB_FTDI_Send(char *bytes, int howMany) {
 // Sends howMany bytes, starting at the address pointed to by
 // bytes, or until it reaches the end of the string
 // (whichever happens first).
 unsigned int next = 0;
 while (next++ < howMany && *bytes) {
 while (!(USART2->SR & USART_SR_TXE));
 USART2->DR = *bytes++;
 }
}
void PB_FTDI_SendNewLine() {
 // Does what it says on the tin... sends a CR and LF:
 while (!(USART2->SR & USART_SR_TXE)); // Wait until ready
 USART2->DR = '\r';
 while (!(USART2->SR & USART_SR_TXE));
 USART2->DR = '\n';
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
  myAudioSpeedUpTheSystemClock();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_USB_HOST_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  // Speed up the clock, then configure the audio timer (required for checking timeouts):
  initAudioTimer();

  // Initialise the audio driver:
  myAudioInitialisePeripherals(OUTPUT_DEVICE_AUTO, 80, AUDIO_FREQUENCY_44K);

  // Initialise the audio buffer with silence:
  for(int i=0; i <= PBSIZE; i++) {
	  PlayBuff[i] = 0;
  }

  //Set up light sensitivity of colour sensor
  setupDevice(SET_INTEGRATION_TIME_24_ms, SET_ANALOGUE_GAIN_X64); //integration time 24ms, gain
  enableDevice();

  //Set up wave-tables
  for (int x = 0; x < SYNTHSIZE; x++) {
	  float sineWav = 32760 * sin(x * 2.0 * PI / SYNTHSIZE);
	  SineBuff[x] = (int16_t)sineWav;
	  float rampWav = 32760 * (1.0 - x / (1.0 * 1024));
	  RampBuff[x] = (int16_t)rampWav;
	  float squareWav = 32760 * (x < 1024 ? 1.0 : -1.0 );
	  SquareBuff[x] = (int16_t)squareWav;
  }


  // Start the audio driver play routine:
  myAudioStartPlaying(PlayBuff, PBSIZE);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	getColourData((int16_t *)colours);
	PB_FTDI_Send((int16_t *)colours[0], 2);
	PB_FTDI_Send((int16_t *)colours[1], 2);
	PB_FTDI_Send((int16_t *)colours[2], 2);
	PB_FTDI_SendNewLine();
	HAL_Delay(1000);

	 //A HAL library routine to get values from the ADC and output them onto the serial terminal every cycle of the while loop
	 HAL_ADC_Start(&hadc1);
	 amplitude_pot = HAL_ADC_GetValue(&hadc1);
	 //It also calculates a ratio, a float between 0 and 1, that is used as a multiplier for the amplitude earlier in the program
	 amplitude_ratio = amplitude_pot/4095.0;

	colours[0] = colours[0]/256;
	colours[1] = colours[1]/256;
	colours[2] = colours[2]/256;
	//int16_t colours[3] = {0, 255, 0} ;  //array of 3 RGB values 0-255

	//Colour average(frequency)is calculated and multiplied by a value which customises the max frequency of the system to ~1000Hz)
	int16_t colourAvg = ((colours[0] + colours[1] + colours[2])/3) * 3.92156863 ;

	setFrequencyCallback(colourAvg);

	for(int k = 0; k < SYNTHSIZE; k++){
			//ratio of each colour:255 is used to calculate respective level(amp)
			float r = (colours[0]/255.0) * SineBuff[k];
			float g = (colours[1]/255.0) * RampBuff[k];
			float b = (colours[2]/255.0) * SquareBuff[k];

	//Divisor value, n keeps the output amplitude at a constant level for all variations in the colour array
	int n = (colours[0]!=0?1:0)+(colours[1]!=0?1:0)+(colours[2]!=0?1:0);
		float sum = (r + g + b)/ n * (amplitude_ratio);
		SoundBuff[k] = (int16_t) sum; //SoundBuff is where the final wave-table is stored
	}

	uint32_t startFill = 0, endFill = 0;

	if (bufferStatus == firstHalfReq) {
		startFill = 0;
		endFill = PBSIZE / 2;
		bufferStatus = firstHalfDone;
	}
	else if (bufferStatus == secondHalfReq){
		startFill = PBSIZE / 2;
		endFill = PBSIZE;
		bufferStatus = secondHalfDone;
	}

	if (startFill != endFill) {
		for (int i = startFill; i < endFill; i+=2) {
			currentPhase += phaseInc;
			if (currentPhase > SYNTHSIZE) {
				currentPhase -= SYNTHSIZE;
			}
			uint16_t nextSample = SoundBuff[(uint16_t)(currentPhase)];
			PlayBuff[i] = nextSample;
			PlayBuff[i+1] = nextSample;
		}
	}

    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */
  }



      //This part it just for debugging purposes so can be commented out and the audio program will still run: it prints the 12 bit value from the ADC to a serial console
      sprintf(msg, "Slider: %hu \r\n", amplitude_pot);
      HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
    }
  /* USER CODE END 3 */


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
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
