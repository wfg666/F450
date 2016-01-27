/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
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
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "./global.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

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

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();

  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim6);
	
//	uint8_t aaa[]={0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
//	HAL_UART_Transmit_IT(&huart3,aaa, 16);

//	for(__IO int i=0;i<10000;i++);
	
	HAL_UART_Receive_IT(&huart3,USART3RxBuffer,25);
	HAL_UART_Receive_IT(&huart2,USART2RxBuffer,sizeof(AHRSData)+2);

	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		__IO int s=sizeof(AHRSData);
		
		if(AHRSReceived)
		{
			uint8_t cs=0;
			for(int8_t i=1;i<=sizeof(AHRSData)+1;i++)
				cs^=USART2RxBuffer[i];
			if(USART2RxBuffer[0]==0xaa && cs==0)
			{
				AHRSData = *((struct AHRSDataStruct *)(USART2RxBuffer+1));
			}
			else
			{
				int32_t receiveTime = sysClock;
				while(sysClock - receiveTime < 10 && sysClock >= receiveTime);//等1ms，重新收
			}
			AHRSReceived = 0;
			HAL_UART_Receive_IT(&huart2,USART2RxBuffer,sizeof(AHRSData)+2);
		}
		
		if(rcReceived)
		{
			if(USART3RxBuffer[0]==0x0f)
			{
				//todo
					uint8_t *p_origin=USART3RxBuffer;
					uint16_t *p_result=rcChannels;
					int16_t countOrigin=0,countResult=0,i;
					uint8_t filter=0x01;

					(*p_result)=0;	//p_result置零
					p_result--;		//p_result从int[0]开始，p_origin从char[1]开始
					for(i=0; i<176; i++)
					{	
						countOrigin--;countResult--;
						if(countOrigin<0)
						{
							countOrigin=7;	//Origin每8位指针移动
							p_origin++;
						}
						if(countResult<0)
						{
							countResult=10;					//Result每11位指针移动
							(*p_result)=(*p_result)<<1;		//撤销最后一次移位
							p_result++;
							(*p_result)=0;
						}
						(*p_result)|=(((*p_origin)&(filter))<<10);
						(*p_result)=(*p_result)>>1;
						(*p_origin)=(*p_origin)>>1;
					}
				
					p_origin++;
//					wireLessLost=(*p_origin)&0x04;
//					digitalChannel2=(*p_origin)&0x02;//两个数字通道好像有一个有问题
//					digitalChannel1=(*p_origin)&0x01;
	
				;
			}
			else
			{
				int32_t receiveTime = sysClock;
				while(sysClock - receiveTime < 40 && sysClock >= receiveTime);//等1ms，重新收
			}
			rcReceived = 0;
			HAL_UART_Receive_IT(&huart3,USART3RxBuffer,25);
		}
		
		if(sysClock-lastUpdateControlTime >= 25)
		{
			
			pitchAngleSP = (rcChannels[1]-1024)/1000.0*400;
			rollAngleSP = (rcChannels[0]-1024)/1000.0*400;
			
			yawAngleSP += (rcChannels[3]-1024)/1000.0*0.2;
			if(yawAngleSP > 3600)
				yawAngleSP -=3600;
			if(yawAngleSP < 0)
				yawAngleSP +=3600;
			
			throttle = 0.900 + rcChannels[2]/2000.0;
			
			
			
			pitchAngleErr = AHRSData.pitch - pitchAngleSP;
			pitchAngleAdjust = -(pitchAngleP * pitchAngleErr + pitchAngleD * AHRSData.gyroY);
			rollAngleErr = AHRSData.roll - rollAngleSP;
			rollAngleAdjust = -(rollAngleP * rollAngleErr + rollAngleD * AHRSData.gyroX);
			
			yawAngleErr = AHRSData.yaw - yawAngleSP;
			if(yawAngleErr>1800)
				yawAngleErr-=3600;
			if(yawAngleErr<-1800)
				yawAngleErr+=3600;
			
			yawAngleAdjust = -(yawAngleP * yawAngleErr + yawAngleD * AHRSData.gyroZ);
			
			constrain(pitchAngleAdjust,-0.4f,0.4f);
			constrain(rollAngleAdjust,-0.4f,0.4f);
			constrain(yawAngleAdjust,-0.4f,0.4f);
			
			
			pwm1 = throttle - pitchAngleAdjust - rollAngleAdjust + yawAngleAdjust;
			pwm2 = throttle + pitchAngleAdjust + rollAngleAdjust + yawAngleAdjust;
			pwm3 = throttle - pitchAngleAdjust + rollAngleAdjust - yawAngleAdjust;
			pwm4 = throttle + pitchAngleAdjust - rollAngleAdjust - yawAngleAdjust;
			
			if(throttle<1.1f)
			{
				pwm1=0.9;pwm2=0.9;pwm3=0.9;pwm4=0.9;yawAngleSP=AHRSData.yaw;
			}
			
			constrain(pwm1,0.9f,2.1f);
			constrain(pwm2,0.9f,2.1f);
			constrain(pwm3,0.9f,2.1f);
			constrain(pwm4,0.9f,2.1f);
				
			
			
			
			TIM_OC_InitTypeDef sConfigOC;
			sConfigOC.OCMode = TIM_OCMODE_PWM1;
			sConfigOC.Pulse = 0;
			sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
			sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
			
			sConfigOC.Pulse = pwm1*84003;
			HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);
			HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
			sConfigOC.Pulse = pwm2*84003;
			HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2);
			HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
			sConfigOC.Pulse = pwm3*84003;
			HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3);
			HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
			sConfigOC.Pulse = pwm4*84003;
			HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4);
			HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
			
			lastUpdateControlTime = sysClock;

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

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* TIM2 init function */
void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 210000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim2);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);

  HAL_TIM_PWM_Init(&htim2);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 70000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);

  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2);

  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3);

  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4);

}

/* TIM6 init function */
void MX_TIM6_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 8400;
  HAL_TIM_Base_Init(&htim6);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig);

}

/* USART2 init function */
void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 256000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart2);

}

/* USART3 init function */
void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 100000;
  huart3.Init.WordLength = UART_WORDLENGTH_9B;
  huart3.Init.StopBits = UART_STOPBITS_2;
  huart3.Init.Parity = UART_PARITY_EVEN;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_8;
  HAL_UART_Init(&huart3);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOH_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART2)
	{
		AHRSReceived = 1;
	}
	
	if(huart->Instance == USART3)
	{
		rcReceived = 1;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance==TIM6)
	{
		sysClock = (sysClock+1)%1000000000;
		HAL_GPIO_WritePin(GPIOD, LD6_Pin,(GPIO_PinState)!HAL_GPIO_ReadPin(GPIOD, LD6_Pin));
	}
}
/* USER CODE END 4 */

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
