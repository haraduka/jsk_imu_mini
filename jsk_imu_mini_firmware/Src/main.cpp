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
#include "adc.h"
#include "spi.h"
#include "usart.h"
#include "dma.h"
#include "iwdg.h"
#include "gpio.h"
#include "stm32f4xx_hal.h"
/* USER CODE BEGIN Includes */
#include "common.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/


/* flag for the start about the process in HAL_SYSTICK_Callback  */
bool start_process_flag_ = false;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
static int counter = 0;
static int LEDcounter = 0;

// attitude estimate and  control is at sys timer it callback
// update interrupt
void HAL_SYSTICK_Callback(void)
{
	static uint32_t last_time = HAL_GetTick();
	uint32_t now_time = HAL_GetTick();


	/* ros spin func, mainly subscribing  data from ROS network */
	nh_.spinOnce();

	if(!start_process_flag_)
		return;
	/* please check whether connection between ros and mcu is build */
	if(nh_.connected())
	{
		/* publish message */
			/* state estimate */
		if(counter++>8)
		{
			counter = 0;
			imu_.update();
//			estimator_.attitude_estimator_.imu_msg_.stamp = estimator_.attitude_estimator_.nh_->now();
//			for(int i=0;i<3;i++)
//			{
//				estimator_.attitude_estimator_.imu_msg_.gyro_data[i] = estimator_.attitude_estimator_.imu_->getGyro()[i];
//				estimator_.attitude_estimator_.imu_msg_.mag_data[i] = estimator_.attitude_estimator_.imu_->getMag()[i];
//				estimator_.attitude_estimator_.imu_msg_.acc_data[i] = estimator_.attitude_estimator_.imu_->getAcc()[i];
//			}
//			estimator_.attitude_estimator_.imu_pub_->publish(&estimator_.attitude_estimator_.imu_msg_);
			estimator_.update();

		}
	}



}
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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  //MX_IWDG_Init();

  /* USER CODE BEGIN 2 */
  /**********************************/
  {
	  nh_.initNode(&huart1);
	  /* ugv ros node */
	  testnode = new RosNode(&nh_);
	  /* Sensors */
#if IMU_FLAG
	  imu_.init(&hspi1, &nh_);
	  /* State Estimation */
	  estimator_.init(&imu_, NULL, NULL, &nh_);  // imu  => att
#endif

	  /* all process can start right now! */
	  start_process_flag_ = true;
	 // HAL_IWDG_Init(&hiwdg);

  }
  /* USER CODE END 2 */
  uint16_t ADC_value[4];

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  if(start_process_flag_){
		  if(LEDcounter >= 10){
			  HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
			  LEDcounter = 0;
		  }
		  LEDcounter++;
	  }
	  HAL_Delay(9);
	  //uint8_t c[5]={1,2,3,4,5};

	  HAL_ADC_Start(&hadc1);
	  for(int i=0; i<4; i++){
		  HAL_ADC_PollForConversion(&hadc1, 1);
		  ADC_value[i] = HAL_ADC_GetValue(&hadc1);
	  }
	  testnode->publish(ADC_value);
	  HAL_ADC_Stop(&hadc1);

		  //imu_.update();
	 // HAL_IWDG_Refresh(&hiwdg);

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
   RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
   RCC_OscInitStruct.HSEState = RCC_HSE_ON;
   RCC_OscInitStruct.LSIState = RCC_LSI_ON;
   RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
   RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
   RCC_OscInitStruct.PLL.PLLM = 4;
   RCC_OscInitStruct.PLL.PLLN = 100;
   RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
   RCC_OscInitStruct.PLL.PLLQ = 4;
   RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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

/* USER CODE BEGIN 4 */

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
