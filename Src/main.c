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
  * COPYRIGHT(c) 2017 STMicroelectronics
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
#include "stm32l0xx_hal.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include <accelerometer.h>
#include <gyroscope.h>
#include <x_nucleo_iks01a2_accelero.h>
#include <x_nucleo_iks01a2_gyro.h>
#include "orientation_estimation.h"
#include <math.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define FIFO_WATERMARK 25
#define LSM6DSL_SAMPLE_ODR	ODR_LOW
#define LSM6DSL_FIFO_MAX_ODR 26

#define PATTERN_GYR_X_AXIS 0
#define PATTERN_GYR_Y_AXIS 1
#define PATTERN_GYR_Z_AXIS 2

DrvContextTypeDef *Accelero;
DrvContextTypeDef *Gyro;
unsigned char dataAvailable = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static DrvStatusTypeDef LSM6DSL_FIFO_Config(void);
static DrvStatusTypeDef LSM6DSL_Read_All_FIFO_Data(void);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

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
  MX_I2C1_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();

  /* USER CODE BEGIN 2 */
  // Initialize the Board support packages for accelero and gyro
  BSP_ACCELERO_Init(LSM6DSL_X_0, (void**)&Accelero);
  BSP_GYRO_Init(LSM6DSL_G_0, (void**)&Gyro);

  BSP_ACCELERO_Check_WhoAmI(Accelero);
  // And enable both sensors
  BSP_ACCELERO_Sensor_Enable(Accelero);
  BSP_GYRO_Sensor_Enable(Gyro);

  // Configure the sensors in FIFO mode,
  LSM6DSL_FIFO_Config();
  // And initialize the estimator with 0 radians, 1g estimation
  orient_est_init(0, 1.0);

  // Finally turn off the LED
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  // When data are available
	  if (dataAvailable)
	      {
		  	dataAvailable=0;
		  	// read the FIFO data (and do the estimation)
	      	LSM6DSL_Read_All_FIFO_Data();

	      }
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/** NVIC Configuration
*/
static void MX_NVIC_Init(void)
{
  /* EXTI4_15_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
}

/* USER CODE BEGIN 4 */

/**
 * @brief Configure the sensor in FIFO mode with a sampling rate of 26Hz
 */
static DrvStatusTypeDef LSM6DSL_FIFO_Config(void)
{
	// Configure the IMU sampling rate
	if (BSP_GYRO_Set_ODR_Value(Gyro, LSM6DSL_FIFO_MAX_ODR) == COMPONENT_ERROR)
		return COMPONENT_ERROR;

	if (BSP_ACCELERO_Set_ODR_Value(Accelero, LSM6DSL_FIFO_MAX_ODR) == COMPONENT_ERROR)
		return COMPONENT_ERROR;


	/** Set Full Scale **/
	if (BSP_GYRO_Set_FS(Gyro, FS_LOW) == COMPONENT_ERROR)
			return COMPONENT_ERROR;

	if (BSP_ACCELERO_Set_FS(Accelero, FS_MID) == COMPONENT_ERROR)
			return COMPONENT_ERROR;

	/** Set FIFO Decimation */
	if (BSP_GYRO_FIFO_Set_Decimation_Ext(Gyro, LSM6DSL_ACC_GYRO_DEC_FIFO_G_NO_DECIMATION) == COMPONENT_ERROR)
		return COMPONENT_ERROR;

	if (BSP_ACCELERO_FIFO_Set_Decimation_Ext(Accelero, LSM6DSL_ACC_GYRO_DEC_FIFO_XL_NO_DECIMATION) == COMPONENT_ERROR)
		return COMPONENT_ERROR;

	/** Set ODR to highest value **/
	if (BSP_GYRO_FIFO_Set_ODR_Value_Ext(Gyro, LSM6DSL_FIFO_MAX_ODR) == COMPONENT_ERROR)
		return COMPONENT_ERROR;

	if (BSP_ACCELERO_FIFO_Set_ODR_Value_Ext(Accelero, LSM6DSL_FIFO_MAX_ODR) == COMPONENT_ERROR)
		return COMPONENT_ERROR;

	/** Set FIFO FULL on INT1 **/
	if (BSP_GYRO_FIFO_Set_INT1_FIFO_Full_Ext(Gyro, LSM6DSL_ACC_GYRO_INT1_FULL_FLAG_ENABLED) == COMPONENT_ERROR)
		return COMPONENT_ERROR;


	/** Set FIFO Watermark **/
	if (BSP_GYRO_FIFO_Set_Watermark_Level_Ext(Gyro, FIFO_WATERMARK) == COMPONENT_ERROR)
		return COMPONENT_ERROR;

	/** Set FIFO depth to be limited to watermark thershold level **/
	if (BSP_GYRO_FIFO_Set_Stop_On_Fth_Ext(Gyro, LSM6DSL_ACC_GYRO_STOP_ON_FTH_ENABLED) == COMPONENT_ERROR)
		return COMPONENT_ERROR;


	if (BSP_GYRO_FIFO_Set_Mode_Ext(Gyro, LSM6DSL_ACC_GYRO_FIFO_MODE_DYN_STREAM_2) == COMPONENT_ERROR)
		return COMPONENT_ERROR;

	return COMPONENT_OK;
}

/**
 * @brief Read all the FIFO data and feed them to the EKF
 */
static DrvStatusTypeDef LSM6DSL_Read_All_FIFO_Data(void)
{
	uint16_t samplesToRead = 0;
	int i =0, j=0;
	uint16_t pattern;
	int32_t velocity;

	int32_t data[6]={0,0,0,0,0,0};
	static int32_t offset=0;
	static uint8_t offset_count = 0;

	/* get samples count */
	if (BSP_GYRO_FIFO_Get_Num_Of_Samples_Ext(Gyro, &samplesToRead) == COMPONENT_ERROR)
		return COMPONENT_ERROR;

	/* Each sample is a 6 component info */
	samplesToRead /= 6;


	// For each sample
	for (i=0;i<samplesToRead;i++)
	{
		// Clear the memory
		memset(data, 0, sizeof(data));
		// Get the data
		for (j=0;j<6;j++)
		{
			if (BSP_GYRO_FIFO_Get_Pattern_Ext(Gyro, &pattern) == COMPONENT_ERROR)
				return COMPONENT_ERROR;

			if (pattern>2)
			{
				if (BSP_ACCELERO_FIFO_Get_Axis_Ext(Accelero, &velocity) == COMPONENT_ERROR)
					return COMPONENT_ERROR;
			}
			else
			{
				if (BSP_GYRO_FIFO_Get_Axis_Ext(Gyro, &velocity) == COMPONENT_ERROR)
					return COMPONENT_ERROR;
			}

			if (pattern<6 && pattern>=0)
				data[pattern] = velocity;
		}

		// Use the first 100 points to estimate the gyroscope bias
		if (offset_count < 100)
		{
			offset += data[1];
			offset_count ++;
		}
		// When offset if known, go for the pose estimation
		else
		{
			// For algorithm performances measurement purpose, the LED flashed during the estimation time
	      	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
	      	float gyro_rate_rad = -((float)(data[1]-offset/100))*2*M_PI/360000.0; // The gyro rate in rad/s
	      	float ax_g = ((float)data[3])/1000.0f; // The X acceleration in g
	      	float az_g = ((float)data[5])/1000.0f; // The Z acceleration in g

			if (orient_est_evolve(gyro_rate_rad) == MATH_ERROR)
				while(1);
			if (orient_est_correct(ax_g, az_g) == MATH_ERROR)
				while(1);
	      	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
		}
	}
	// Print the results for the user
	printf("%d, %d\n", (int)(orient_est_get_theta()*360.0f/(2.0f*M_PI)), (int)(orient_est_get_g()*1000.0f));

	return COMPONENT_OK;
}

// Handle External interrupt on FIFO Watermark reached for the IMU
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_5)
	{
		dataAvailable=1;
	}
}

/**
 * @brief Override the _write function to write data to the USART
 */
int _write(_PTR file, const char *buffer, _READ_WRITE_BUFSIZE_TYPE len)
{
	HAL_UART_Transmit(&huart2, buffer, len, 100);
	return len;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
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
