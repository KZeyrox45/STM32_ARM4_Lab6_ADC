/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"
#include "fsmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "software_timer.h"
#include "led_7seg.h"
#include "button.h"
#include "lcd.h"
#include "picture.h"
#include "ds3231.h"
#include "sensor.h"
#include "buzzer.h"
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
void system_init();
void test_LedDebug();
void test_Buzzer();
void test_Adc();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM2_Init();
  MX_SPI1_Init();
  MX_FSMC_Init();
  MX_I2C1_Init();
  MX_TIM13_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  system_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
 lcd_Clear(BLACK);
  while (1)
  {
	  while(!flag_timer2);
	  flag_timer2 = 0;
	  button_Scan();
	  test_LedDebug();
	  test_Adc();
	  test_Buzzer();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void system_init(){
	timer_init();
	button_init();
	lcd_init();
	sensor_init();
	buzzer_init();
	setTimer2(50);
	led7_init();
	ds3231_init();
}

uint8_t count_led_debug = 0;

void test_LedDebug(){
	count_led_debug = (count_led_debug + 1)%20;
	if(count_led_debug == 0){
		HAL_GPIO_TogglePin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin);
	}
}

uint8_t isButtonUp()
{
    if (button_count[3] == 1)
        return 1;
    else
        return 0;
}

uint8_t isButtonDown()
{
    if (button_count[7] == 1)
        return 1;
    else
        return 0;
}

uint8_t isButtonRight()
{
    if (button_count[11] == 1)
        return 1;
    else
        return 0;
}

uint8_t count_adc = 0;
uint8_t alarm_state = 0;  // 0: No alarm, 1: Alarm active
uint32_t last_buzzer_toggle_time = 0;  // To track the last time buzzer was toggled

float get_PowerConsumption() {
	return sensor_GetVoltage() * sensor_GetCurrent() * 1000;
}

float get_Humidity() {
	return ((float)sensor_GetPotentiometer() / 4095.0) * 100;
}

void display_time() {
    // Read the current time from DS3231
    ds3231_ReadTime();

    // Extract hours and minutes
    uint8_t hours = ds3231_hours - 6;
    uint8_t minutes = ds3231_min;

    // Set the digits on the 7-segment display
    led7_SetDigit(hours / 10, 0, 0);  // Display the tens place of hours
    led7_SetDigit(hours % 10, 1, 0);  // Display the ones place of hours
    led7_SetDigit(minutes / 10, 2, 0); // Display the tens place of minutes
    led7_SetDigit(minutes % 10, 3, 0); // Display the ones place of minutes

    // Set the colon (:) between hours and minutes
    led7_SetColon(1); // Turn on colon
}

void test_Adc() {
	count_adc = (count_adc + 1) % 20;
	if(count_adc == 0){
		sensor_Read();
		lcd_ShowStr(10, 100, "Power consumption:", RED, BLACK, 16, 0);
		lcd_ShowIntNum(160, 100, get_PowerConsumption(), 4, RED, BLACK, 16);
		lcd_ShowStr(10, 130, "Light:", RED, BLACK, 16, 0);
		if (sensor_GetLight() >= 1750) {
			lcd_ShowStr(160, 130, "Strong", RED, BLACK, 16, 0);
		} else {
			lcd_ShowStr(160, 130, "Weak  ", RED, BLACK, 16, 0);
		}
		lcd_ShowStr(10, 160, "Temperature:", RED, BLACK, 16, 0);
		lcd_ShowFloatNum(160, 160, sensor_GetTemperature(), 4, RED, BLACK, 16);
		lcd_ShowStr(10, 190, "Humidity:", RED, BLACK, 16, 0);
		lcd_ShowFloatNum(160, 190, get_Humidity(), 4, RED, BLACK, 16);

		// Check condition to alarm if humidity is larger than 70%
		if (get_Humidity() > 70) {
			if (alarm_state == 0) {
				alarm_state = 1;
				last_buzzer_toggle_time = HAL_GetTick(); // Store the current time for buzzer toggle
			}
		} else {
			alarm_state = 0;
			buzzer_SetVolume(0);  // Ensure the buzzer is off if no alarm
		}

		// If in alarm state, toggle the buzzer every second
		if (alarm_state == 1) {
			uint32_t current_time = HAL_GetTick();
		    if (current_time - last_buzzer_toggle_time >= 1000) {  // Check if 1 second has passed
		    	last_buzzer_toggle_time = current_time;  // Update the last toggle time
		        static uint8_t buzzer_on = 0;  // Track the buzzer state (on/off)
		        if (buzzer_on) {
		        	buzzer_SetVolume(0);  // Turn off buzzer
		        } else {
		        	buzzer_SetVolume(25);  // Turn on buzzer (25% volume)
		        }
		            buzzer_on = !buzzer_on;  // Toggle buzzer state
		    }
		}
	}
	display_time();
}

void test_Buzzer(){
	if(isButtonUp()){
		buzzer_SetVolume(50);
	}

	if(isButtonDown()){
		buzzer_SetVolume(0);
	}

	if(isButtonRight()){
		buzzer_SetVolume(25);
	}
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
