/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
 ******************************************************************************
 ** This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * COPYRIGHT(c) 2018 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "lcd.h"
#include "delay.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart2;
TextLCDType lcd;
/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

//Keypad och diverse
char str[50], string[50];
int svar = -1;
int check;
int channel_0;
int knapp;

//Alarm/Activate/Deactivate
int ARM_DISARM_flag = 1;
int activate_deactivate = 0;
int alarm_off = 0;
int alarm = 0;
int volym = 0;

//timer/PIR
int PIR = 0;
int sec = 0, check_sek = 0;
int sec10 = 3;
int timer = 0;
char lcdTime[3];

// temprature
int channel_0, temprature, temp, set_Temp_Flag = 0, check_set;
int temp10 = 0, temp1 = 0;
int set_Temp = 30;
float recistance, vOut;

//Thermometer
uint8_t data_rec[6];
int16_t x, y, z;
int xg, yg, zg;
int set_Threshold = 100, set_Threshold_Flag = 0;
int threshold1 = 0, threshold2 = 0, threshold3 = 0; //check value when setting threshold
int set_x, set_y, set_z;

typedef enum {
	IDLE,
	SET_TEMP,
	SET_THERMOMETER,
	TURN_OFF1,
	TURN_OFF2,
	LARM_OFF,
	ARM_DISARM1,
	ARM_DISARM2,
	ARMING,
	ARMED,
	WRONG_PIN1,
	WRONG_PIN2,
	WRONG_PIN3,
	WRONG_PIN
} state_t;

volatile state_t state = IDLE;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM10_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM11_Init(void);
static void MX_I2C1_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

//recieve the input from keypad and check if code is right
void check_Code(int keyVal) {

	switch (state) {
	case IDLE:

		TextLCD_Position(&lcd, 1, 0);
		TextLCD_Puts(&lcd, "PIN:");
		if (alarm == 0) {
			//shows temprature on display
			sprintf(str, "T: %d C", temprature);
			TextLCD_Position(&lcd, 1, 8);
			TextLCD_Puts(&lcd, str);

			//A to set TEMP
			if (keyVal == -3) {
				state = SET_TEMP;
				TextLCD_Clear(&lcd);
			}
			//B to set THERMOMETER
			if (keyVal == -4) {
				set_Threshold = 0;
				state = SET_THERMOMETER;
				TextLCD_Clear(&lcd);
			}

		}
		if (keyVal >= 0) {
			TextLCD_Clear(&lcd);
			TextLCD_Position(&lcd, 1, 0);
			TextLCD_Puts(&lcd, "PIN:*");
			if (keyVal == 6) {

				state = TURN_OFF1;
			} else if (keyVal == 7) {

				state = ARM_DISARM1;
			} else {

				state = WRONG_PIN1;
			}

		}
		break;

	case SET_THERMOMETER:
		sprintf(str, "X:%d", xg);
		TextLCD_Position(&lcd, 1, 0);
		TextLCD_Puts(&lcd, str);
		sprintf(str, "Y:%d", yg);
		TextLCD_Position(&lcd, 1,7);
		TextLCD_Puts(&lcd, str);
		sprintf(str, "Z:%d", zg);
		TextLCD_Position(&lcd, 2, 0);
		TextLCD_Puts(&lcd, str);

		sprintf(str, "Th:+- %d", set_Threshold);
		TextLCD_Position(&lcd, 2, 7);
		TextLCD_Puts(&lcd, str);

		if (keyVal >= 0) {
			check_set = keyVal;
			set_Threshold_Flag++;

			if (set_Threshold_Flag == 1) {
				threshold1 = check_set;
				set_Threshold = threshold1;

			}

			else if (set_Threshold_Flag == 2) {
				threshold2 = check_set;
				set_Threshold = (threshold1 * 10) + threshold2;

			} else if (set_Threshold_Flag == 3) {
				threshold3 = check_set;
				set_Threshold = (threshold1 * 100) + (threshold2 * 10)
						+ threshold3;

			}
		}

		if (keyVal == -5)   //C to CLEAR
				{
			threshold1 = 0;
			threshold2 = 0;
			threshold3 = 0;
			set_Threshold = 0;
			set_Threshold_Flag = 0;
			TextLCD_Clear(&lcd);

		}
		if (keyVal == -4) {  //B to set threshold
			set_Threshold_Flag = 0;
			TextLCD_Clear(&lcd);
			state = IDLE;

		}

		break;

	case SET_TEMP:

		sprintf(str, "T: %d C", temprature);
		TextLCD_Position(&lcd, 1, 0);
		TextLCD_Puts(&lcd, str);
		sprintf(str, "Set T: %d C", set_Temp);
		TextLCD_Position(&lcd, 2, 0);
		TextLCD_Puts(&lcd, str);

		if (keyVal >= 0) {
			check_set = keyVal;

			if (set_Temp_Flag == 0) {
				temp10 = check_set;
				temp1 = 0;
				set_Temp = (temp10 * 10) + temp1;
				set_Temp_Flag = 1;
			}

			else if (set_Temp_Flag == 1) {
				temp1 = check_set;
				set_Temp = (temp10 * 10) + temp1;
				set_Temp_Flag = 0;
			}
		}
		if (keyVal == -5)   // C to CLEAR
				{
			temp10 = 0;
			temp1 = 0;
			set_Temp = 0;
			TextLCD_Clear(&lcd);
		}

		if (keyVal == -3) {  // A to set temp
			state = IDLE;
			TextLCD_Clear(&lcd);
		}

		break;

	case TURN_OFF1:

		if (keyVal >= 0) {

			TextLCD_Position(&lcd, 1, 0);
			TextLCD_Puts(&lcd, "PIN:**");
			if (keyVal == 5)
				state = TURN_OFF2;
			else
				state = WRONG_PIN2;

		}
		break;
	case TURN_OFF2:

		if (keyVal >= 0) {

			TextLCD_Position(&lcd, 1, 0);
			TextLCD_Puts(&lcd, "PIN:***");
			if (keyVal == 1)
				state = LARM_OFF;
			else
				state = WRONG_PIN3;

		}
		break;
	case LARM_OFF:

		if (keyVal >= 0) {

			TextLCD_Position(&lcd, 1, 0);
			TextLCD_Puts(&lcd, "PIN:****");
			if (keyVal == 0) {
				alarm = 0;
				HAL_TIM_PWM_Stop(&htim10, TIM_CHANNEL_1);
				HAL_TIM_Base_Stop_IT(&htim11);
				ARM_DISARM_flag = 1;
				TextLCD_Clear(&lcd);
				TextLCD_Position(&lcd, 2, 3);
				TextLCD_Puts(&lcd, "LARM OFF!");
				activeBuzzer();
				HAL_Delay(1000);

				sprintf(str, "larm off \n\r ");
				HAL_UART_Transmit(&huart2, (uint8_t*) str, strlen(str), 100);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 1);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);
				TextLCD_Clear(&lcd);
				state = IDLE;
			} else
				state = WRONG_PIN;

		}
		break;
	case ARM_DISARM1:

		if (keyVal >= 0) {

			TextLCD_Position(&lcd, 1, 0);
			TextLCD_Puts(&lcd, "PIN:**");
			if (keyVal == 3)
				state = ARM_DISARM2;
			else
				state = WRONG_PIN2;

		}
		break;
	case ARM_DISARM2:

		if (keyVal >= 0) {

			TextLCD_Position(&lcd, 1, 0);
			TextLCD_Puts(&lcd, "PIN:***");
			if (keyVal == 9)
				state = ARMING;

			else
				state = WRONG_PIN3;

		}
		break;
	case ARMING:

		if (keyVal >= 0) {

			TextLCD_Position(&lcd, 1, 0);
			TextLCD_Puts(&lcd, "PIN:****");

			if (keyVal == 2) {



				//check if you want to activate or deactivate the alarm-system
				if (ARM_DISARM_flag == 0)
					ARM_DISARM_flag = 1;
				else if (ARM_DISARM_flag == 1)
					ARM_DISARM_flag = 0;

				//When activating the alarm-system
				if (ARM_DISARM_flag == 0) {

					TextLCD_Clear(&lcd);
					TextLCD_Position(&lcd, 1, 4);
					TextLCD_Puts(&lcd, "ARMING!");
					activeBuzzer();

					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 0);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);
					timer = 1;
					HAL_TIM_Base_Start_IT(&htim11);

				}

				// when deactivate the alarm-system, you send flag = 1 to activate_deactivate() function
				else
					activate_deactivate = 1;


			}


			else
				state = WRONG_PIN;
		}

		break;

	case ARMED:

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 0);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1);
		if (alarm == 0 && PIR == 0) {
			TextLCD_Position(&lcd, 2, 4);
			TextLCD_Puts(&lcd, "ARMED");
			activate_deactivate = 1;
		}
		if (alarm == 1)
			HAL_TIM_Base_Start_IT(&htim11); // starts the timer for sirens when alarm = 1

		if (keyVal == -2) // press D to set a new pin
			state = IDLE;

		break;
	case WRONG_PIN1:

		if (keyVal >= 0) {
			TextLCD_Clear(&lcd);
			TextLCD_Position(&lcd, 1, 0);
			TextLCD_Puts(&lcd, "PIN:**");
			state = WRONG_PIN2;
		}
		break;
	case WRONG_PIN2:

		if (keyVal >= 0) {
			TextLCD_Clear(&lcd);
			TextLCD_Position(&lcd, 1, 0);
			TextLCD_Puts(&lcd, "PIN:***");
			state = WRONG_PIN3;

		}
		break;
	case WRONG_PIN3:

		if (keyVal >= 0) {
			TextLCD_Clear(&lcd);
			TextLCD_Position(&lcd, 1, 0);
			TextLCD_Puts(&lcd, "PIN:****");
			HAL_Delay(1000);
			state = WRONG_PIN;

		}
		break;
	case WRONG_PIN:
		TextLCD_Clear(&lcd);
		TextLCD_Position(&lcd, 1, 3);
		TextLCD_Puts(&lcd, "WRONG PIN!");
		sprintf(str, "wrong code \n\r ");
		HAL_UART_Transmit(&huart2, (uint8_t*) str, strlen(str), 100);
		HAL_Delay(1500);
		TextLCD_Clear(&lcd);
		state = IDLE;

		break;

	default:
		state = IDLE;
		break;
	}
}

//Temprature
int16_t Read_Analog_Temp() {

	HAL_ADC_Start(&hadc1);
	if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK) {
		channel_0 = HAL_ADC_GetValue(&hadc1);
	}
	HAL_ADC_Stop(&hadc1);

	vOut = (3.3f * channel_0) / 4096.0f;

	/* Calculate Thermistor Resistance. Code Begin*/

	recistance = ((10000 * 5.0F) / vOut) - 10000; // Rt = R0 * (( Vs / Vo ) - 10000) ,    Löste ut R ur (V = 2.5 * (R/(10000 + R))

	/* Calculate Thermistor Resistance. Code End*/

	/* Calculate Temprature. Code Begin*/
	temp = 1.0f / ((log(recistance / 10000) / 3450.0f) + (1.0f / 298.15f));
	temprature = temp - 273.15f;
	/* Calculate Temprature. Code End*/

	//sprintf(str, "vOut: %d\n\r", temprature);
	//HAL_UART_Transmit(&huart2, (uint8_t*) str, strlen(str), 100);
	return temprature;
}

// timer
void countDown() {


	if (PIR == 0) {
			TextLCD_Position(&lcd, 2, 0);
			TextLCD_Puts(&lcd, "Timer:");

			TextLCD_Position(&lcd, 2, 7);
			TextLCD_Puts(&lcd, lcdTime);
		}



	lcdTime[0] = sec10 + 0x30;
	lcdTime[1] = sec + 0x30;
	lcdTime[2] = '\0';

	if (check_sek != sekTick) {
		sec--;
		check_sek = sekTick;
	}

	if (sec < 0) {
		sec = 9;
		sec10--;
		activeBuzzer(); // var tioende sekund körs buzzer
	}

	if (sec == 0 && sec10 == 0 && PIR == 0) { // 30 ec
		TextLCD_Clear(&lcd);
		sec = 0;
		sec10 = 3;
		activeBuzzer();
		set_x = xg;// set the current position on thermometer
		set_y = yg;
		set_z = zg;
		state = ARMED;
		HAL_TIM_Base_Stop_IT(&htim11);
		check_sek = 0;
		timer = 0;

	}
	if (sec == 0 && sec10 == 1 && PIR == 1) { // 20 sec
		sec = 0;
		sec10 = 3;
		alarm = 1;
		HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
		TextLCD_Clear(&lcd);
		HAL_TIM_Base_Stop_IT(&htim11);
		check_sek = 0;
		timer = 0;
	}

	if (svar == -2) {
			TextLCD_Clear(&lcd);
			state = IDLE;

		}

}

// Activate or deactivate the alarm-system
void activate_Deactivate() {
	if (ARM_DISARM_flag == 0) //ACTIVATED
			{

		//red light on
		// all sensors on
		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == 1) { // PIR

			PIR = 1;
			timer = 1;
			HAL_TIM_Base_Start_IT(&htim11);

			TextLCD_Position(&lcd, 2, 1);
			TextLCD_Puts(&lcd, "!PIR!DETECTED!");

		}

		if ((set_Temp + 2) <= temprature) { // alarm on if set_temp + 2 is lower than the temprature.
			alarm = 1;
			HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
			HAL_TIM_Base_Start_IT(&htim11);
		}
		if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2) == 0) { // alarm on if the diamond is being lifted
			alarm = 1;
			HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
			HAL_TIM_Base_Start_IT(&htim11);
		}

		if ((set_x + set_Threshold) <= xg || (set_x - set_Threshold) >= xg
				||	// alarm on if an activty is generating higher than the threshold
				(set_y + set_Threshold) <= yg || (set_y - set_Threshold) >= yg
				|| (set_z + set_Threshold) <= zg
				|| (set_z - set_Threshold) >= zg)

				{
			alarm = 1;
			HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
			HAL_TIM_Base_Start_IT(&htim11);
		}

		if (svar == -2)
			state = IDLE;
		// If a sensor is tripped, the passive buzzer will go on

	}

	else if (ARM_DISARM_flag == 1) // DEACTIVATED
			{
		HAL_TIM_PWM_Stop(&htim10, TIM_CHANNEL_1);
		HAL_TIM_Base_Stop_IT(&htim11);
		PIR = 0;
		timer = 0;
		sec = 0;
		sec10 = 3;

		TextLCD_Clear(&lcd);

		TextLCD_Position(&lcd, 2, 3);
		TextLCD_Puts(&lcd, "DEACTIVATED!");
		HAL_Delay(2000);
		TextLCD_Clear(&lcd);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 1);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);

		activate_deactivate = 0;

		state = IDLE;

	}
}

//Alarm ON
void sirenON() {

	activate_deactivate = 0;
	PIR = 0;

	TextLCD_Position(&lcd, 2, 3);
	TextLCD_Puts(&lcd, "!ALARM!");

	if (volym == 1) {
		__HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 1260); // 1260 / 4200 = 30% duty cycle

	}

	else if (volym == 2) {
		__HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 2100); // 2100/4200 = 50% duty cycle

		volym = 0;
	}

	if (check_sek != sekTick) {
		volym++;
		check_sek = sekTick;

	}

}
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

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
	MX_USART2_UART_Init();
	MX_TIM10_Init();
	MX_ADC1_Init();
	MX_TIM11_Init();
	MX_I2C1_Init();

	/* USER CODE BEGIN 2 */

	TextLCD_Init(&lcd, LCD_RS_GPIO_Port, LCD_RS_Pin, LCD_RW_Pin, LCD_E_Pin,
	LCD_D0_GPIO_Port,
			LCD_D1_Pin | LCD_D2_Pin | LCD_D3_Pin | LCD_D4_Pin | LCD_D5_Pin
					| LCD_D6_Pin | LCD_D7_Pin);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 1); // Start with green light

	ADXL345_Init();  // initiate thermometer

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	while (1) {
		check = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);
		Read_Analog_Temp();  // checking temprature from thermomistor
		ADXL345_Run(); // check thermometer

		scanna_knapp(); // check which button is being clicked from keypad

		check_Code(svar); // sends output from keypad and check

		sprintf(str, "CHECK: %d \n\r ", check);
		HAL_UART_Transmit(&huart2, (uint8_t*) str, strlen(str), 100);

		sprintf(str, "KEYVAL: %d \n\r ", svar);
		HAL_UART_Transmit(&huart2, (uint8_t*) str, strlen(str), 100);

		/*	sprintf(str, "THRESHOLD: %d \n\r ", set_Threshold);
		 HAL_UART_Transmit(&huart2, (uint8_t*) str, strlen(str), 100);*/

		if (timer == 1) {
			countDown();  // starts counting down

		}

		if (activate_deactivate == 1) {
			activate_Deactivate(); // check if you want to activate or deactivate the alarm-system
		}

		if (alarm == 1) {

			sirenON();   // the siren will sound
		}

	}

	/* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */

	/* USER CODE END 3 */

}

/** System Clock Configuration
 */
void SystemClock_Config(void) {

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	/**Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE()
	;

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 16;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void) {

	ADC_ChannelConfTypeDef sConfig;

	/**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* I2C1 init function */
static void MX_I2C1_Init(void) {

	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* TIM10 init function */
static void MX_TIM10_Init(void) {

	TIM_OC_InitTypeDef sConfigOC;

	htim10.Instance = TIM10;
	htim10.Init.Prescaler = 20;
	htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim10.Init.Period = 4200;
	htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	if (HAL_TIM_Base_Init(&htim10) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	if (HAL_TIM_PWM_Init(&htim10) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 4199;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	HAL_TIM_MspPostInit(&htim10);

}

/* TIM11 init function */
static void MX_TIM11_Init(void) {

	htim11.Instance = TIM11;
	htim11.Init.Prescaler = 1343;
	htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim11.Init.Period = 62499;
	htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	if (HAL_TIM_Base_Init(&htim11) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* USART2 init function */
static void MX_USART2_UART_Init(void) {

	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/** Configure pins as 
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 */
static void MX_GPIO_Init(void) {

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE()
	;
	__HAL_RCC_GPIOH_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;
	__HAL_RCC_GPIOD_CLK_ENABLE()
	;

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC,
			LCD_D0_Pin | LCD_D1_Pin | LCD_D2_Pin | LCD_D3_Pin | LCD_D4_Pin
					| LCD_D5_Pin | LCD_D6_Pin | LCD_D7_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, C4_Pin | LD2_Pin | C3_Pin | C2_Pin | C1_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB,
			LCD_RS_Pin | LCD_RW_Pin | LCD_E_Pin | green_Pin | red_Pin
					| yellow_Pin | buzzer_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LCD_D0_Pin LCD_D1_Pin LCD_D2_Pin LCD_D3_Pin
	 LCD_D4_Pin LCD_D5_Pin LCD_D6_Pin LCD_D7_Pin */
	GPIO_InitStruct.Pin = LCD_D0_Pin | LCD_D1_Pin | LCD_D2_Pin | LCD_D3_Pin
			| LCD_D4_Pin | LCD_D5_Pin | LCD_D6_Pin | LCD_D7_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : PIR_Pin */
	GPIO_InitStruct.Pin = PIR_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(PIR_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : C4_Pin LD2_Pin C3_Pin C2_Pin
	 C1_Pin */
	GPIO_InitStruct.Pin = C4_Pin | LD2_Pin | C3_Pin | C2_Pin | C1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : LCD_RS_Pin LCD_RW_Pin LCD_E_Pin green_Pin
	 red_Pin yellow_Pin buzzer_Pin */
	GPIO_InitStruct.Pin = LCD_RS_Pin | LCD_RW_Pin | LCD_E_Pin | green_Pin
			| red_Pin | yellow_Pin | buzzer_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : R4_Pin R3_Pin R2_Pin R1_Pin */
	GPIO_InitStruct.Pin = R4_Pin | R3_Pin | R2_Pin | R1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : switch_Pin */
	GPIO_InitStruct.Pin = switch_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(switch_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

//Buzzer
void activeBuzzer() {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 1);
	HAL_Delay(800);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 0);
}

//thermometer
void ADXL345_WriteRegister(uint8_t reg, uint8_t value) // To write to the internal registers in the device
{
	uint8_t data[2];
	data[0] = reg;
	data[1] = value;
	HAL_I2C_Master_Transmit(&hi2c1, 0x53 << 1, data, 2, 10);
}

void ADXL345_readRegisters(uint8_t reg, uint8_t numberofbytes) //to read from the internal registers in the device
{

	HAL_I2C_Mem_Read(&hi2c1, 0x53 << 1, reg, 1, data_rec, numberofbytes, 100);
}

void ADXL345_Init(void) {
	ADXL345_readRegisters(0x00, 1);

	ADXL345_WriteRegister(0x2d, 0); //reset
	ADXL345_WriteRegister(0x2d, 0x08); //measure bit
	ADXL345_WriteRegister(0x31, 0x01); // +4 range
}

void ADXL345_Run(void) {

	ADXL345_readRegisters(0x32, 6);
	x = data_rec[1] << 8 | data_rec[0];
	y = data_rec[3] << 8 | data_rec[2];
	z = data_rec[5] << 8 | data_rec[4];

	xg = x * 7.8;
	yg = y * 7.8;
	zg = z * 7.8;

}

//KEYPAD
void knapp1() {

	svar = 1;
	HAL_Delay(300);

}

void knapp4() {

	svar = 4;
	HAL_Delay(300);

}

void knapp7() {

	svar = 7;
	HAL_Delay(300);

}

void knapp0() {

	svar = 0;
	HAL_Delay(300);

}

void knapp2() {

	svar = 2;
	HAL_Delay(300);

}

void knapp5() {

	svar = 5;
	HAL_Delay(300);

}

void knapp8() {

	svar = 8;
	HAL_Delay(300);

}

void knapp3() {

	svar = 3;
	HAL_Delay(300);

}

void knapp6() {

	svar = 6;
	HAL_Delay(300);

}

void knapp9() {

	svar = 9;
	HAL_Delay(300);

}
void knappD() {
	svar = -2;
	HAL_Delay(300);
}

void knappA() {
	svar = -3;
	HAL_Delay(300);
}

void knappB() {
	svar = -4;
	HAL_Delay(300);
}

void knappC() {
	svar = -5;
	HAL_Delay(300);
}

void setcolum() {

	HAL_GPIO_WritePin(GPIOA, C1_Pin, 1);
	HAL_GPIO_WritePin(GPIOA, C2_Pin, 1);
	HAL_GPIO_WritePin(GPIOA, C3_Pin, 1);
	HAL_GPIO_WritePin(GPIOA, C4_Pin, 1);
}

void setcolum1() {

	HAL_GPIO_WritePin(GPIOA, C1_Pin, 0);
	HAL_GPIO_WritePin(GPIOA, C2_Pin, 1);
	HAL_GPIO_WritePin(GPIOA, C3_Pin, 1);
	HAL_GPIO_WritePin(GPIOA, C4_Pin, 1);
}

void setcolum2() {

	HAL_GPIO_WritePin(GPIOA, C1_Pin, 1);
	HAL_GPIO_WritePin(GPIOA, C2_Pin, 0);
	HAL_GPIO_WritePin(GPIOA, C3_Pin, 1);
	HAL_GPIO_WritePin(GPIOA, C4_Pin, 1);
}

void setcolum3() {

	HAL_GPIO_WritePin(GPIOA, C1_Pin, 1);
	HAL_GPIO_WritePin(GPIOA, C2_Pin, 1);
	HAL_GPIO_WritePin(GPIOA, C3_Pin, 0);
	HAL_GPIO_WritePin(GPIOA, C4_Pin, 1);
}

void setcolum4() {

	HAL_GPIO_WritePin(GPIOA, C1_Pin, 1);
	HAL_GPIO_WritePin(GPIOA, C2_Pin, 1);
	HAL_GPIO_WritePin(GPIOA, C3_Pin, 1);
	HAL_GPIO_WritePin(GPIOA, C4_Pin, 0);
}

void scana_rad1() {
	setcolum1();

	if ((HAL_GPIO_ReadPin( GPIOA, R1_Pin)) == 0) {

		// nummer 1
		knapp1();

	}

	if ((HAL_GPIO_ReadPin( GPIOA, R2_Pin)) == 0) {

		// nummer 4
		knapp4();
	}

	if ((HAL_GPIO_ReadPin( GPIOA, R3_Pin)) == 0) {

		// nummer 7
		knapp7();
	}

	if ((HAL_GPIO_ReadPin( GPIOA, R4_Pin)) == 0) {

		// nummer " * "
		//knapp0();
	}

}

void scana_rad2() {

	setcolum2();

	if ((HAL_GPIO_ReadPin( GPIOA, R1_Pin)) == 0) {

		// nummer 2
		knapp2();
	}

	if ((HAL_GPIO_ReadPin( GPIOA, R2_Pin)) == 0) {

		// nummer 5
		knapp5();
	}

	if ((HAL_GPIO_ReadPin( GPIOA, R3_Pin)) == 0) {

		// nummer 8
		knapp8();
	}

	if ((HAL_GPIO_ReadPin( GPIOA, R4_Pin)) == 0) {

		// nummer 0
		knapp0();
	}

}

void scana_rad3() {

	setcolum3();

	if ((HAL_GPIO_ReadPin( GPIOA, R1_Pin)) == 0) {

		// nummer 3
		knapp3();

	}

	if ((HAL_GPIO_ReadPin( GPIOA, R2_Pin)) == 0) {

		// nummer 6
		knapp6();
	}

	if ((HAL_GPIO_ReadPin( GPIOA, R3_Pin)) == 0) {

		// nummer 9
		knapp9();
	}

	if ((HAL_GPIO_ReadPin( GPIOA, R4_Pin)) == 0) {

		// nummer " # "
		//knapp0();
	}

}

void scana_rad4() {

	setcolum4();

	if ((HAL_GPIO_ReadPin( GPIOA, R1_Pin)) == 0) {

		// nummer A
		knappA();
	}

	if ((HAL_GPIO_ReadPin( GPIOA, R2_Pin)) == 0) {

		// nummer B
		knappB();
	}

	if ((HAL_GPIO_ReadPin( GPIOA, R3_Pin)) == 0) {

		// nummer C
		knappC();
	}

	if ((HAL_GPIO_ReadPin( GPIOA, R4_Pin)) == 0) {

		// nummer " D "
		knappD();
	}

}

int scanna_knapp() {

	//setcolum();
	svar = -1;
	scana_rad1();
	scana_rad2();
	scana_rad3();
	scana_rad4();

	return svar;

}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void _Error_Handler(char * file, int line) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */

}

#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
