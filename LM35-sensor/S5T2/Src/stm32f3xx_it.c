/**
  ******************************************************************************
  * @file    stm32f3xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2020 STMicroelectronics
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
#include "stm32f3xx_hal.h"
#include "stm32f3xx.h"
#include "stm32f3xx_it.h"

/* USER CODE BEGIN 0 */
void numbertobcd1(int i){
int x1=i&1;
int x2=i&2;
int x3=i&4;
int x4=i&8;
if(x1>0)
	x1=1;
if(x2>0)
	x2=1;
if(x3>0)
	x3=1;
if(x4>0)
	x4=1;
HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1,x1);
HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,x2);
HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,x3);
HAL_GPIO_WritePin(GPIOD,GPIO_PIN_4,x4);

}
int h;
int ha;
extern int count;
extern int counter;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern TIM_HandleTypeDef htim2;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F3xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f3xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles ADC1 and ADC2 interrupts.
*/
void ADC1_2_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_2_IRQn 0 */

  /* USER CODE END ADC1_2_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  HAL_ADC_IRQHandler(&hadc2);
  /* USER CODE BEGIN ADC1_2_IRQn 1 */
  int x=HAL_ADC_GetValue(&hadc1);
	float fx=((float)x*3300/4095);
	h=(int)fx/10;
	
	int a=HAL_ADC_GetValue(&hadc2);
	ha=((float)a*50/4095);
	
	HAL_TIM_Base_Start_IT(&htim2);
	
	HAL_ADC_Start_IT(&hadc1);
	HAL_ADC_Start_IT(&hadc2);
	
  /* USER CODE END ADC1_2_IRQn 1 */
}

/**
* @brief This function handles TIM2 global interrupt.
*/
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */
	
	if(counter>70&&counter<140&&h<ha){
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_11,1);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_12,1);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_0,1);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_10,1);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_5,0);
}
	
	else{
		
		if(h>=ha) HAL_GPIO_WritePin(GPIOD,GPIO_PIN_5,0);
		else HAL_GPIO_WritePin(GPIOD,GPIO_PIN_5,1);
		
	if(count%4==0){
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_11,1);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_12,1);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_0,1);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_10,0);
		numbertobcd1 (h/10);		
	}
	if(count%4==1){
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_12,1);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_0,1);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_10,1);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_11,0);
		numbertobcd1 (h%10);
	}
	if(count%4==2){
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_11,1);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_12,0);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_0,1);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_10,1);
		numbertobcd1 (ha/10);
	}
	if(count%4==3){
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_11,1);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_12,1);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_0,0);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_10,1);
		numbertobcd1 (ha%10);
	}
	
	count++;
}
	
if(counter==140) counter=0;
	counter++;	

  /* USER CODE END TIM2_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
