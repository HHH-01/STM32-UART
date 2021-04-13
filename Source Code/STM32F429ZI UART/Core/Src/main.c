/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
/*Info
*Developer: HHH
*Email: harry.hoa.huynh.01@gmail.com
*Date: 4/2021
*Version: 001
*Main goal: 	1. Collect temperature data using STM32F401RE
*							2. Send data to STM32F429ZI via UART using Polling
*							3. Display data on the LCD on the STM32F429ZI
*
*STM32F401RE Set up: 
*1. Temperature MCP9700A: PA0	[ADC 10bits with 13 clock cycles]
*2. UART: TX = PC6 | RX = PC7 [Baud rate of 9600bs, Word length of 8b]
*
*STM32F429ZI Set up
*1. UART: TX = PC6 | RX = PC7 [Baud rate of 9600bs, Word length of 8b]
*2. Display the temperature via LCD
*		*	RS	PD1
*			EN	PD7
*			DB4	PD6
*			DB5	PD5
*			DB6	PD4
*			DB7	PD3
*		
*		Component:
*		1. STM Nucleo STM32F429ZI
*		2. STM Nucleo STM32F401RE
*		3. Temperature MCP9700A
*		4. LCD HC16102

*/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
// LCD related functions
void LCD_init(void);
void LCD_write(unsigned char data);
void place_lcd_cursor(unsigned char lineno);
void LCD_write_word(char data[], int length);
void LCD_write_decimal(double data, int length);
void LCD_Clear(void);
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
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart6_rx;

/* USER CODE BEGIN PV */

uint32_t ADC_value;
float temperature_value;


uint8_t RX_buffer[5]={0};
uint8_t RX_buffer_temp[5]={0};

char *endptr;
int RX_int = 0;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define RS GPIO_PIN_1
#define EN GPIO_PIN_7

#define DB4 GPIO_PIN_6
#define DB5 GPIO_PIN_5
#define DB6 GPIO_PIN_4
#define DB7 GPIO_PIN_3
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
	
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//HAL_UART_Receive_DMA(&huart6, RX_buffer, 4);
}	
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
  MX_ADC1_Init();
  MX_USART6_UART_Init();
	//HAL_UART_Receive_DMA(&huart6, RX_buffer, 4);
  /* USER CODE BEGIN 2 */
 // Start the LCD
	LCD_init();
	place_lcd_cursor(1);	
	LCD_write_word("Starting", 8);
	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    LCD_Clear();		
		//UART**************************************
		HAL_UART_Receive (&huart6, RX_buffer, 5, 1000);	//Using Pooling
		//Shift by 8 bit
		RX_buffer_temp[0] = (RX_buffer[1]);
		RX_buffer_temp[1] = (RX_buffer[2]);
		RX_buffer_temp[2] = (RX_buffer[3]);
		RX_buffer_temp[3] =	(RX_buffer[4]);
		//convert buffer to string
		RX_int = (int)strtol(RX_buffer_temp, &endptr, 10);
		if (*endptr != '\0') RX_int=9999;	//check to see if there's an error

		//Temperature Conversion and display to LCD**************************************
		if (RX_int==9999){LCD_write_word("ERR", 3);}	//display on the LCD if there's error
		else{
			ADC_value=RX_int;
			temperature_value = ((ADC_value*3.3)/1024 -0.5)/0.01;		//Input of 3.3V to the Sensor | Sensor Output Calibrated with  0.5V offset (MCP9700/9700A datasheet)
			LCD_write_word("T:", 2);
			LCD_write_decimal(temperature_value,4);
			LCD_write_word(" C", 2);
		}
		HAL_Delay(1000);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_10B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Green_Pin|LD3_Red_Pin|LD2_Blue_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LCD_RS_Pin|LCD_PD3_Pin|LCD_DB6_Pin|LCD_DB5_Pin
                          |LCD_DB4_Pin|LCD_En_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LD1_Green_Pin LD3_Red_Pin LD2_Blue_Pin */
  GPIO_InitStruct.Pin = LD1_Green_Pin|LD3_Red_Pin|LD2_Blue_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_RS_Pin LCD_PD3_Pin LCD_DB6_Pin LCD_DB5_Pin
                           LCD_DB4_Pin LCD_En_Pin */
  GPIO_InitStruct.Pin = LCD_RS_Pin|LCD_PD3_Pin|LCD_DB6_Pin|LCD_DB5_Pin
                          |LCD_DB4_Pin|LCD_En_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/*******************************
 * LCD_init()
 * Inputs: NONE
 * Outputs: NONE
 * LCD Initialization
 * Read the manual carefully
 * We are doing initialization by instruction
 * Don't rush it.
 *******************************
 */

void LCD_init(){

// STEP 1: Wait for 100ms for power-on-reset to take effect
	HAL_Delay(100);
// STEP 2: Set RS pin LOW to send instructions
	HAL_GPIO_WritePin(GPIOD, RS, 0);
// Send instructions using following format:
// Set EN=HIGH; Send 4-bit instruction; Set EN=low; delay 20ms;
	
// STEP 3a-3d: Set 4-bit mode (takes a total of 4 steps)
	HAL_GPIO_WritePin(GPIOD, EN, 1);
	
	HAL_GPIO_WritePin(GPIOD, DB7, 0);
	HAL_GPIO_WritePin(GPIOD, DB6, 0);
	HAL_GPIO_WritePin(GPIOD, DB5, 1);
	HAL_GPIO_WritePin(GPIOD, DB4, 1);
	
	HAL_GPIO_WritePin(GPIOD, EN, 0);
	HAL_Delay(20);
	
	HAL_GPIO_WritePin(GPIOD, EN, 1);
	
	HAL_GPIO_WritePin(GPIOD, DB7, 0);
	HAL_GPIO_WritePin(GPIOD, DB6, 0);
	HAL_GPIO_WritePin(GPIOD, DB5, 1);
	HAL_GPIO_WritePin(GPIOD, DB4, 1);
	
	HAL_GPIO_WritePin(GPIOD, EN, 0);
	HAL_Delay(20);
	
	HAL_GPIO_WritePin(GPIOD, EN, 1);
	
	HAL_GPIO_WritePin(GPIOD, DB7, 0);
	HAL_GPIO_WritePin(GPIOD, DB6, 0);
	HAL_GPIO_WritePin(GPIOD, DB5, 1);
	HAL_GPIO_WritePin(GPIOD, DB4, 1);
	
	HAL_GPIO_WritePin(GPIOD, EN, 0);
	HAL_Delay(20);
	
	HAL_GPIO_WritePin(GPIOD, EN, 1);
	
	HAL_GPIO_WritePin(GPIOD, DB7, 0);
	HAL_GPIO_WritePin(GPIOD, DB6, 0);
	HAL_GPIO_WritePin(GPIOD, DB5, 1);
	HAL_GPIO_WritePin(GPIOD, DB4, 0);
	
	HAL_GPIO_WritePin(GPIOD, EN, 0);
	HAL_Delay(20);
// STEP 4: Set 2 line display -- treats 16 char as 2 lines
	HAL_GPIO_WritePin(GPIOD, EN, 1);
	
	HAL_GPIO_WritePin(GPIOD, DB7, 0);
	HAL_GPIO_WritePin(GPIOD, DB6, 0);
	HAL_GPIO_WritePin(GPIOD, DB5, 1);
	HAL_GPIO_WritePin(GPIOD, DB4, 0);
	
	HAL_GPIO_WritePin(GPIOD, EN, 0);
	HAL_Delay(20);
	
	HAL_GPIO_WritePin(GPIOD, EN, 1);
	
	HAL_GPIO_WritePin(GPIOD, DB7, 1);
	HAL_GPIO_WritePin(GPIOD, DB6, 0);
	HAL_GPIO_WritePin(GPIOD, DB5, 0);
	HAL_GPIO_WritePin(GPIOD, DB4, 0);
	
	HAL_GPIO_WritePin(GPIOD, EN, 0);
	HAL_Delay(20);
// STEP 5: Set DISPLAY to OFF
	HAL_GPIO_WritePin(GPIOD, EN, 1);
	
	HAL_GPIO_WritePin(GPIOD, DB7, 0);
	HAL_GPIO_WritePin(GPIOD, DB6, 0);
	HAL_GPIO_WritePin(GPIOD, DB5, 0);
	HAL_GPIO_WritePin(GPIOD, DB4, 0);
	
	HAL_GPIO_WritePin(GPIOD, EN, 0);
	HAL_Delay(20);
	
	HAL_GPIO_WritePin(GPIOD, EN, 1);
	
	HAL_GPIO_WritePin(GPIOD, DB7, 1);
	HAL_GPIO_WritePin(GPIOD, DB6, 0);
	HAL_GPIO_WritePin(GPIOD, DB5, 0);
	HAL_GPIO_WritePin(GPIOD, DB4, 0);
	
	HAL_GPIO_WritePin(GPIOD, EN, 0);
	HAL_Delay(20);
// STEP 6: CLEAR DISPLAY
	HAL_GPIO_WritePin(GPIOD, EN, 1);
	
	HAL_GPIO_WritePin(GPIOD, DB7, 0);
	HAL_GPIO_WritePin(GPIOD, DB6, 0);
	HAL_GPIO_WritePin(GPIOD, DB5, 0);
	HAL_GPIO_WritePin(GPIOD, DB4, 0);
	
	HAL_GPIO_WritePin(GPIOD, EN, 0);
	HAL_Delay(20);
	
	HAL_GPIO_WritePin(GPIOD, EN, 1);
	
	HAL_GPIO_WritePin(GPIOD, DB7, 0);
	HAL_GPIO_WritePin(GPIOD, DB6, 0);
	HAL_GPIO_WritePin(GPIOD, DB5, 0);
	HAL_GPIO_WritePin(GPIOD, DB4, 1);
	
	HAL_GPIO_WritePin(GPIOD, EN, 0);
	HAL_Delay(20);
// STEP 7: SET ENTRY MODE - Auto increment; no scrolling
	HAL_GPIO_WritePin(GPIOD, EN, 1);
	
	HAL_GPIO_WritePin(GPIOD, DB7, 0);
	HAL_GPIO_WritePin(GPIOD, DB6, 0);
	HAL_GPIO_WritePin(GPIOD, DB5, 0);
	HAL_GPIO_WritePin(GPIOD, DB4, 0);
	
	HAL_GPIO_WritePin(GPIOD, EN, 0);
	HAL_Delay(20);
	
	HAL_GPIO_WritePin(GPIOD, EN, 1);
	
	HAL_GPIO_WritePin(GPIOD, DB7, 0);
	HAL_GPIO_WritePin(GPIOD, DB6, 1);
	HAL_GPIO_WritePin(GPIOD, DB5, 1);
	HAL_GPIO_WritePin(GPIOD, DB4, 0);
	
	HAL_GPIO_WritePin(GPIOD, EN, 0);
	HAL_Delay(20);
// STEP 8: Set Display to ON with Cursor and Blink.
	HAL_GPIO_WritePin(GPIOD, EN, 1);
	
	HAL_GPIO_WritePin(GPIOD, DB7, 0);
	HAL_GPIO_WritePin(GPIOD, DB6, 0);
	HAL_GPIO_WritePin(GPIOD, DB5, 0);
	HAL_GPIO_WritePin(GPIOD, DB4, 0);
	
	HAL_GPIO_WritePin(GPIOD, EN, 0);
	HAL_Delay(20);
	
	HAL_GPIO_WritePin(GPIOD, EN, 1);
	
	HAL_GPIO_WritePin(GPIOD, DB7, 1);
	HAL_GPIO_WritePin(GPIOD, DB6, 1);
	HAL_GPIO_WritePin(GPIOD, DB5, 1);
	HAL_GPIO_WritePin(GPIOD, DB4, 1);
	
	HAL_GPIO_WritePin(GPIOD, EN, 0);
	HAL_Delay(20);
	
	HAL_GPIO_WritePin(GPIOD, RS, 1);
}

/*******************************
 * place_lcd_cursor()
 * Inputs: unsigned character
 * Outputs: NONE
 * sets Cursor position to
 * Line 1, character 1 (hex address 0x80)
 * or Line 2, character 1 (hex addres 0xC0)
 *
 *******************************
 */

void place_lcd_cursor(unsigned char lineno){
	HAL_GPIO_WritePin(GPIOD, RS, 0);
	if(lineno == 1){					//To place at line 1, send address 0x80
		HAL_GPIO_WritePin(GPIOD, EN, 1);
		
		HAL_GPIO_WritePin(GPIOD, DB7, 1);
		HAL_GPIO_WritePin(GPIOD, DB6, 0);
		HAL_GPIO_WritePin(GPIOD, DB5, 0);
		HAL_GPIO_WritePin(GPIOD, DB4, 0);
		
		HAL_GPIO_WritePin(GPIOD, EN, 0);
		HAL_Delay(20);
		
		HAL_GPIO_WritePin(GPIOD, EN, 1);
		
		HAL_GPIO_WritePin(GPIOD, DB7, 0);
		HAL_GPIO_WritePin(GPIOD, DB6, 0);
		HAL_GPIO_WritePin(GPIOD, DB5, 0);
		HAL_GPIO_WritePin(GPIOD, DB4, 0);
		
		HAL_GPIO_WritePin(GPIOD, EN, 0);
		HAL_Delay(1);
	}
	else{											//Place at line two, send address 0xC0
		HAL_GPIO_WritePin(GPIOD, EN, 1);
		
		HAL_GPIO_WritePin(GPIOD, DB7, 1);
		HAL_GPIO_WritePin(GPIOD, DB6, 1);
		HAL_GPIO_WritePin(GPIOD, DB5, 0);
		HAL_GPIO_WritePin(GPIOD, DB4, 0);
		
		HAL_GPIO_WritePin(GPIOD, EN, 0);
		HAL_Delay(20);
		
		HAL_GPIO_WritePin(GPIOD, EN, 1);
		
		HAL_GPIO_WritePin(GPIOD, DB7, 0);
		HAL_GPIO_WritePin(GPIOD, DB6, 0);
		HAL_GPIO_WritePin(GPIOD, DB5, 0);
		HAL_GPIO_WritePin(GPIOD, DB4, 0);
		
		HAL_GPIO_WritePin(GPIOD, EN, 0);
		HAL_Delay(1);
	}
	HAL_GPIO_WritePin(GPIOD, RS, 1);
}



/*******************************
 * LCD_write()
 * Inputs: unsigned character data (8-bit)
 * Outputs: NONE
 * writes the character to LCD.
 *
 *******************************
 */

void LCD_write(unsigned char data)
{
	HAL_GPIO_WritePin(GPIOD, RS, 1);
	
	HAL_GPIO_WritePin(GPIOD, EN, 1);
		//For each bit check if it is a one or zero and send output accordingly
		if(data & 0x80){
			HAL_GPIO_WritePin(GPIOD, DB7, 1);
		}
		else{
			HAL_GPIO_WritePin(GPIOD, DB7, 0);
		}
		if(data & 0x40){
			HAL_GPIO_WritePin(GPIOD, DB6, 1);
		}
		else{
			HAL_GPIO_WritePin(GPIOD, DB6, 0);
		}
		if(data & 0x20){
			HAL_GPIO_WritePin(GPIOD, DB5, 1);
		}
		else{
			HAL_GPIO_WritePin(GPIOD, DB5, 0);
		}
		if(data & 0x10){
			HAL_GPIO_WritePin(GPIOD, DB4, 1);
		}
		else{
			HAL_GPIO_WritePin(GPIOD, DB4, 0);
		}
		
		HAL_GPIO_WritePin(GPIOD, EN, 0);
		HAL_Delay(1);
		
		HAL_GPIO_WritePin(GPIOD, EN, 1);
		
		if(data & 0x08){
			HAL_GPIO_WritePin(GPIOD, DB7, 1);
		}
		else{
			HAL_GPIO_WritePin(GPIOD, DB7, 0);
		}
		if(data & 0x04){
			HAL_GPIO_WritePin(GPIOD, DB6, 1);
		}
		else{
			HAL_GPIO_WritePin(GPIOD, DB6, 0);
		}
		if(data & 0x02){
			HAL_GPIO_WritePin(GPIOD, DB5, 1);
		}
		else{
			HAL_GPIO_WritePin(GPIOD, DB5, 0);
		}
		if(data & 0x01){
			HAL_GPIO_WritePin(GPIOD, DB4, 1);
		}
		else{
			HAL_GPIO_WritePin(GPIOD, DB4, 0);
		}
		
		HAL_GPIO_WritePin(GPIOD, EN, 0);
		HAL_Delay(1);
		
	HAL_GPIO_WritePin(GPIOD, RS, 0);
}


/*******************************
 * LCD_write_word()
 * Inputs: string data and int length
 * Outputs: NONE
 * writes the string to LCD.
 *
 *******************************
 */

void LCD_write_word(char data[], int length){
	int i;
	//Send first 8 characters
	for(i = 0; i<length && i<8; i++){
		LCD_write(data[i]);
	}
	//If it is longer than 8 characters then change the line
	if(length > 8){
		place_lcd_cursor(2);
	}
	//Place remaining characters
	for(i = 8; i<length; i++){
		LCD_write(data[i]);
	}
	
}

/*******************************
 * LCD_write_decimal()
 * Inputs: double data and int length
 * Outputs: NONE
 * writes the data to LCD.
 *
 *******************************
 */

void LCD_write_decimal(double data, int length){
	char string[17];
	snprintf(string,length+1,"%f",data);
	LCD_write_word(string,length);
}

/*******************************
 * LCD_Clear()
 * Inputs: NONE
 * Outputs: NONE
 * clears the LCD and resets cursor
 *
 *******************************
 */
void LCD_Clear(void){
	//Send clear command, same as when initializing
	HAL_GPIO_WritePin(GPIOD, RS, 0);
	
	HAL_GPIO_WritePin(GPIOD, EN, 1);
	
	HAL_GPIO_WritePin(GPIOD, DB7, 0);
	HAL_GPIO_WritePin(GPIOD, DB6, 0);
	HAL_GPIO_WritePin(GPIOD, DB5, 0);
	HAL_GPIO_WritePin(GPIOD, DB4, 0);
	
	HAL_GPIO_WritePin(GPIOD, EN, 0);
	HAL_Delay(20);
	
	HAL_GPIO_WritePin(GPIOD, EN, 1);
	
	HAL_GPIO_WritePin(GPIOD, DB7, 0);
	HAL_GPIO_WritePin(GPIOD, DB6, 0);
	HAL_GPIO_WritePin(GPIOD, DB5, 0);
	HAL_GPIO_WritePin(GPIOD, DB4, 1);
	
	HAL_GPIO_WritePin(GPIOD, EN, 0);
	HAL_Delay(20);
	HAL_GPIO_WritePin(GPIOD, RS, 1);
}
/*******************************
 * UART Communication()
 * Inputs: NONE
 * Outputs: NONE
 * Send UART
 *
 *******************************
 */



/*
 * @brief Function to convert and send data over UART
 * @param XL_Data, UART Handle Typedef
 */

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
