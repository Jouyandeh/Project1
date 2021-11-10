
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *45454
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
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "sun_sensor.h" 
#include "sun_sensor_param.h" 
//#define test //for rs485
//#define Error
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
__IO uint32_t SH_period = 20;
__IO uint32_t ICG_period = 500000;
int test2;
uint16_t aTxBuffer[2 * CCDSize+100]= {0};
uint32_t avgBuffer[2 * CCDSize] = {0};
__IO uint8_t aRxBuffer[RxDataSize] = {0};
__IO uint8_t nRxBuffer[RxDataSize] = {0};

__IO uint8_t change_exposure_flag = 0;
__IO uint8_t data_flag = 0;
__IO uint8_t pulse_counter = 0;
__IO uint8_t CCD_flushed = 0;
__IO uint8_t avg_exps = 0;
__IO uint8_t exps_left = 0;

extern DMA_HandleTypeDef hdma_usart1_tx;

volatile phase_t sensor_1 , sensor_2;

#define flush_pulse_counter 3
#define adc_pulse_counter 6
#define max_pulse_counter 32


//-----------------------------------------------------------------------------------------------
uint8_t BASE_BYTE = 0 ;
const uint8_t SH_period_shift = 2,ICG_period_shift = 6 ,h4_1_g_shift =10,h5_1_g_shift =11 ,delta_1_g_shift=12, h4_2_g_shift=13
	, h5_2_g_shift=14,delta_2_g_shift=15, centpixl_1_shift=16 ,centpixl_2_shift=19 , avg_req_shift =23;
//----------------------------------------------------------------------------------------------
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void flush_CCD(void);
//@!Notice!
extern void sort_aRxBuffer(void);
//@!Notice!
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

//void init(){
//	BASE_BYTE = 0 ;
//	BASE_BYTE += 2; 
//	SH_period_shift = BASE_BYTE;// 4 Byte
//	BASE_BYTE += 4; 
//	ICG_period_shift = BASE_BYTE ;// 4 Byte
//	BASE_BYTE += 4; 
//	h4_1_g_shift = BASE_BYTE;// 1 Byte 
//	BASE_BYTE += 1; 
//	h5_1_g_shift = BASE_BYTE;// 1 Byte 
//	BASE_BYTE += 1; 
//	delta_1_g_shift = BASE_BYTE;// 1 Byte 
//	BASE_BYTE += 1; 
//	h4_2_g_shift = BASE_BYTE;// 1 Byte 
//	BASE_BYTE += 1; 
//	h5_2_g_shift = BASE_BYTE;// 1 Byte 
//	BASE_BYTE += 1; 
//	delta_2_g_shift = BASE_BYTE;// 1 Byte 
//	BASE_BYTE += 1;
//	centpixl_shift = BASE_BYTE;// 1 Byte 
//	BASE_BYTE += 3;		
//	BASE_BYTE += 1;//dummy 
//	avg_req_shift = BASE_BYTE;// 1 Byte 
////	BASE_BYTE += 1;
//	
//}
void update_param(){

	ICG_period = 1000000;//nRxBuffer[ICG_period_shift]<<24|nRxBuffer[ICG_period_shift + 1]<<16|nRxBuffer[ICG_period_shift + 2]<<8|nRxBuffer[ICG_period_shift + 3];
	SH_period = nRxBuffer[SH_period_shift]<<24|nRxBuffer[SH_period_shift + 1 ]<<16|nRxBuffer[SH_period_shift + 2]<<8|nRxBuffer[SH_period_shift + 3];
	h4_1_g = 12.0/1000;//nRxBuffer[h4_1_g_shift ] /1000.0;//* 1e-3 ;  
	h5_1_g =  17.0/10000;//nRxBuffer[h5_1_g_shift] /10000.0;//* 1e-4;  
	delta_1_g = 8.0/1000000;//nRxBuffer[delta_1_g_shift] /1000000.0;//* 1e-6;  
	h4_2_g =  12.0/1000;//nRxBuffer[h4_2_g_shift] /1000.0;//* 1e-3;  
	h5_2_g =  17.0/10000;//nRxBuffer[h5_2_g_shift] /10000.0;//* 1e-4;  
	delta_2_g = 8.0/1000000;//nRxBuffer[delta_2_g_shift] /1000000.0;//* 1e-6;  
	centpixl_1 = 1919;//(nRxBuffer[centpixl_1_shift]<<8|nRxBuffer[centpixl_1_shift + 1]) + (nRxBuffer[centpixl_1_shift + 2] / 100.0);
	centpixl_2 = 1864;//(nRxBuffer[centpixl_2_shift]<<8|nRxBuffer[centpixl_2_shift + 1]) + (nRxBuffer[centpixl_2_shift + 2] / 100.0);

}
void RS485_direction(int dir){ // dir 0 recive , dir 1 transmit
const uint8_t RS485_Delay = 20	;	

	if (dir){
			while(__HAL_UART_GET_FLAG (&huart1 ,UART_FLAG_IDLE)); // wait for recive finished
			HAL_Delay(RS485_Delay);//IC DELAY	
			HAL_GPIO_WritePin(RS485__RE_GPIO_Port,RS485__RE_Pin,GPIO_PIN_SET);
	}else{
			while(! (__HAL_UART_GET_FLAG (&huart1 ,UART_FLAG_TXE) && __HAL_UART_GET_FLAG (&huart1 ,UART_FLAG_TC))); // wait for transmit finished
			HAL_Delay(RS485_Delay);//IC DELAY	
			HAL_GPIO_WritePin(RS485__RE_GPIO_Port,RS485__RE_Pin,GPIO_PIN_RESET);
	}

}


uint16_t findMax(const uint16_t arr[],uint16_t len){
  uint16_t max=0;
  int index=0;
  int i=0;

  for(i=0;i<len;i++){
      if(arr[i]>max){
        max=arr[i];
        index=i;
//        printf("%d\t,%d\n,",max,i);
      }
  }
  return max;
}

void invertData(uint16_t data[],uint16_t len){
	
	int i=0;
	//uint16_t max = findMax(data,len)+10;
	for(i=0;i<len;i++)
	
	{
		data[i] = 3300 - data[i];
	}
}

void Send_DATA(){
	RS485_direction(1);//transmit
	
	//HAL_Delay(1000);
//		HAL_UART_Transmit(&huart1,(uint8_t *)&(nRxBuffer[0]), RxDataSize,100);//command
	//sensor_1 = calculateSignal((uint16_t*)signal,CCDSize,centpixl_1,h4_1_g,h5_1_g,delta_1_g);
	//sensor_1 = calculateSignal((uint16_t*)signal,CCDSize,centpixl_1,12e-3,1e-3,8e-6);
	//memcpy((uint8_t*)signal2,(uint8_t*)&aTxBuffer[0],CCDSize*2);
	
	invertData(&aTxBuffer[0 * CCDSize],CCDSize);
	sensor_1 = calculateSignal((uint16_t*)&aTxBuffer[0 * CCDSize],CCDSize,centpixl_1,h4_1_g,h5_1_g,delta_1_g);
	
	invertData(&aTxBuffer[1 * CCDSize],CCDSize);
	sensor_2 = calculateSignal((uint16_t*)&aTxBuffer[1 * CCDSize],CCDSize,centpixl_2,h4_2_g,h5_2_g,delta_2_g); 

	//char result[200];
	char x,buff[200];
	x= sprintf((char *)&aTxBuffer[2 * CCDSize],\
		"\nsensor_1.peak = %05.4f\nsensor_1.val = %05.4f\nsensor_2.peak = %05.4f\nsensor_3.val = %05.4f \n",\
		(sensor_1.peak),				(sensor_1.val),						(sensor_2.peak),			sensor_2.val);
	
	//HAL_Delay(1);
	//x= sprintf((char *)buff,"\nsensor_1.peak = %.4f\nsensor_1.val = %.4f\nsensor_2.peak = %.4f\nsensor_2.val = %.4f \n",sensor_1.peak,sensor_1.val,sensor_2.peak,sensor_2.val);
	//memcpy((void *)&aTxBuffer[2 * CCDSize],(const void *)buff,strlen((const char *)buff));
		
	HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&(aTxBuffer[0 * CCDSize]), (2 * 2 * CCDSize) + 104);//The data is 16-bit, but UART is 8-bit
	while(HAL_DMA_GetState(&hdma_usart1_tx) == HAL_DMA_STATE_BUSY);

//	HAL_UART_Transmit_DMA(&huart1,(uint8_t *)&result[0], x);
//	while(HAL_DMA_GetState(&hdma_usart1_tx) == HAL_DMA_STATE_BUSY);
	HAL_Delay(10);
	RS485_direction(0);//recive

}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM5)
	{
		if (pulse_counter == adc_pulse_counter)
		{
			/* Restart TIM4 as this gets the ADC running again */
			//__HAL_TIM_ENABLE(&htim4);
			//TIM4->CR1 |= TIM_CR1_CEN;
			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
		}
		else if (pulse_counter == flush_pulse_counter)
		{
			CCD_flushed = 1;
		}
		pulse_counter++;
		/* prevent overflow */
		if (pulse_counter > max_pulse_counter)
			pulse_counter = max_pulse_counter;
		/* Flash the led to the beat of ICG */
		//HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
	}
}
#ifdef Error
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
	RS485_direction(1);//RECIVE
	uint32_t error_Code = HAL_UART_GetError(huart);
	char err[16];
	sprintf(err,"USART ERROR\n%2u\n",error_Code);
	HAL_UART_Transmit(&huart1,(uint8_t*)err, 15,100);//for test
	RS485_direction(0);//RECIVE
_Error_Handler(__FILE__, __LINE__);
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef* hadc){
	RS485_direction(1);//RECIVE
	HAL_UART_Transmit(&huart1, "ADC ERROR\n", 10,100);//for test
	RS485_direction(0);//RECIVE
}
#endif
#ifndef test
 void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	
	if(huart->Instance==USART1)
	{
		/* Wait a short while - to let DMA finish ..or something */
		int i = 0;
		for (i=0;i<8400;i++);

		/* Sort aRxBuffer into nRxBuffer */	
		sort_aRxBuffer();

		/* Check the key before doing anything */
	 	if ((nRxBuffer[0]==69)&&(nRxBuffer[1]==82))
		{ 
			/* reset the key */
			nRxBuffer[0] = 0;
			nRxBuffer[1] = 0;

			/* set flags for main-loop */
			change_exposure_flag = 1;
			data_flag = 0;

			/* disable averaging by default */
			avg_exps = 1;

			/* check if user averaging-request is valid */
			if ((nRxBuffer[avg_req_shift]<16)&&(nRxBuffer[avg_req_shift]>0))
				avg_exps = nRxBuffer[avg_req_shift];
			avg_exps = 4;
			exps_left = avg_exps;
		}
//		GPIOA->ODR ^= GPIO_Pin_5;
	}
}
#endif
#ifdef test
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	
	RS485_direction(1);//Transmit
	HAL_UART_Transmit_DMA(&huart1,(uint8_t *)&(aRxBuffer[0]), RxDataSize);
	//HAL_UART_Transmit(&huart1, "recieve\n", 8,100);//for test
	RS485_direction(0);//RECIVE
	
	
}
#endif
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	
	if(hadc->Instance == ADC1)
	{
		//HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);//for test
		/* Stop TIM4 and thus the ADC */
		//__HAL_TIM_DISABLE(&htim4);
		//TIM4->CR1 &= (uint16_t)~TIM_CR1_CEN;
		HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
		
		/* Keep track of the number of integrations performed */
		/* Are we collecting just once? */
		if (avg_exps == 1)
		{
			/* Set the data_flag to transmit */
			data_flag = 1;
		} 
		else if (avg_exps > 1)
		{
			/* Is this the first collection of several? */
			if (exps_left == avg_exps)
			{
//			exps_left--;
				/* Set the pulse counter to 6 to start the ADC again
				   at next ICG-pulse. */
				pulse_counter = adc_pulse_counter;
				/* Set the data_flag to overwrite avgBuffer */
				data_flag = 2;
			}

			/* Is this a collection in the middle? */
			else if ((exps_left < avg_exps)&&(exps_left > 1))
			{
//			exps_left--;
				/* Set the pulse counter to 6 to start the ADC again
  			   at next ICG-pulse. */
				pulse_counter = adc_pulse_counter;
				/* Set the data_flag to sum integrations */
				data_flag = 3;
			}
		
			/* Is this the last collection of several? */
			else if (exps_left == 1)
			{
//				exps_left--;
				/* Set the data_flag to average integrations and tx */
				data_flag = 4;
			}
			exps_left--;
		}
	}
}
void TIM_ICG_SH_conf(void){
	MX_TIM5_Init();
	MX_TIM2_Init();
	
	__HAL_TIM_CLEAR_FLAG(&htim5, TIM_FLAG_UPDATE);
	//__HAL_TIM_ENABLE_IT(&htim5, TIM_IT_UPDATE);
	HAL_TIM_Base_Start_IT(&htim5);
	
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);
	
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	
	htim2.Instance->CNT = SH_period - SH_delay;
	htim5.Instance->CNT = ICG_period - ICG_delay;
	htim3.Instance->CNT =  fm_delay;
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	
//	if(huart->Instance==USART1){
//			RS485_direction(0);//recive
//	}
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint32_t i = 0;

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
//	init();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
	//HAL_GPIO_WritePin(RS485__RE_GPIO_Port,RS485__RE_Pin,GPIO_PIN_RESET);
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	TIM_ICG_SH_conf();

	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&(aTxBuffer[0 * CCDSize]), 1 * CCDSize);
	HAL_ADC_Start_DMA(&hadc2, (uint32_t *)&(aTxBuffer[1 * CCDSize]), 1 * CCDSize);
	
	//Send_DATA();
	
	RS485_direction(0);//RECIVE
	
	
	HAL_UART_Receive_DMA(&huart1,(uint8_t *)&(aRxBuffer[0]), RxDataSize);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	if (change_exposure_flag == 1)
		{
			/* reset flag */
			change_exposure_flag = 0;

			flush_CCD();

			/* set new integration time */
//			ICG_period = nRxBuffer[6]<<24|nRxBuffer[7]<<16|nRxBuffer[8]<<8|nRxBuffer[9];
//			SH_period = nRxBuffer[2]<<24|nRxBuffer[3]<<16|nRxBuffer[4]<<8|nRxBuffer[5];
			update_param();
			/*	Disable ICG (TIM5) and SH (TIM2) before reconfiguring*/
			//TIM_Cmd(TIM2, DISABLE);
			//TIM_Cmd(TIM5, DISABLE);
			//HAL_TIM_Base_Stop(&htim2);
			//HAL_TIM_Base_Stop(&htim5);
			HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
			HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_4);
			
			/* 	Reconfigure TIM2 and TIM5 */
			TIM_ICG_SH_conf();
		}

		switch (data_flag){
		case 1:
			/* reset flags */
			data_flag = 0;
			/* Transmit data in aTxBuffer */
			//Uart1_Tx_DMA();
//			for (i = 0; i < CCDSize; i++)
//			{
//				aTxBuffer[2 * CCDSize + i] = aTxBuffer[2 * i];
//				aTxBuffer[3 * CCDSize + i] = aTxBuffer[2 * i + 1];
//			}
			Send_DATA();
//			HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&(aTxBuffer[0 * CCDSize]), 2 * 4 * CCDSize);//The data is 16-bit, but UART is 8-bit
//			while(HAL_DMA_GetState(&hdma_usart1_tx) == HAL_DMA_STATE_BUSY);
			break;		

		case 2:
			/* reset flags */
			data_flag = 0;

			/* This is the first integration of several so overwrite avgBuffer */
			for (i=0; i<2 * CCDSize; i++)
				avgBuffer[i] = aTxBuffer[i];	
			break;

		case 3:
			/* reset flags */
			data_flag = 0;

			/* Add new to previous integrations.
			   This loop takes 3-4ms to complete. */		
			for (i=0; i<2 * CCDSize; i++)
				avgBuffer[i] = avgBuffer[i] + aTxBuffer[i];		
			break;

		case 4:
			/* reset flags */
			data_flag = 0;

			/* Add new to previous integrations.
			   This loop takes 3-4ms to complete. */		
			for (i=0; i<2 * CCDSize; i++)
				avgBuffer[i] = avgBuffer[i] + aTxBuffer[i];		

			/* Store average values in aTxBuffer */
			for (i=0; i<2 * CCDSize; i++)
				aTxBuffer[i] = avgBuffer[i]/(avg_exps);			

			/* Transmit data in aTxBuffer */
//			Uart1_Tx_DMA();
//			for (i = 0; i < CCDSize; i++)
//			{
//				aTxBuffer[2 * CCDSize + i] = aTxBuffer[2 * i];
//				aTxBuffer[3 * CCDSize + i] = aTxBuffer[2 * i + 1];
//			}
			Send_DATA();
//			HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&(aTxBuffer[0 * CCDSize]), 2 * 4 * CCDSize);//The data is 16-bit, but UART is 8-bit
//			while(HAL_DMA_GetState(&hdma_usart1_tx) == HAL_DMA_STATE_BUSY);
			break;
		}
  }
  /* USER CODE END 3 */


}

/**
  * @brief System Clock Configuration
  * @retval None
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
void flush_CCD()
{
	/* Set exposure very low */
	ICG_period = 15000;
	SH_period = 20;

	/*	Disable ICG (TIM5) and SH (TIM2) before reconfiguring*/
	//TIM_Cmd(TIM2, DISABLE);
	//TIM_Cmd(TIM5, DISABLE);
	//HAL_TIM_Base_Stop(&htim2);
	//HAL_TIM_Base_Stop(&htim5);
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_4);
	
	/*	Reset flags and counters */
	CCD_flushed = 0;
	pulse_counter = 0;

	/* 	Reconfigure TIM2 and TIM5 */
	TIM_ICG_SH_conf();

	/*	Block until CCD is properly flushed */
	while(CCD_flushed == 0);
}

void sort_aRxBuffer()
{
	int shift = -1;	
	int i = 0;
	
	/* Examine for byte-shifts in aRxBuffer */
	for (i = 0; i < RxDataSize; i++)
	{
		if ((aRxBuffer[i]==69)&&(aRxBuffer[i+1]==82))
			shift = i;
	}
	if ((aRxBuffer[RxDataSize-1]==69)&&(aRxBuffer[0]==82))
			shift = RxDataSize-1;

	/* If necessary permutate data */
	if (shift != -1){
		for (i = 0; i < RxDataSize - shift; i++) 
			nRxBuffer[i] = aRxBuffer[shift+i]; 
		for (i = 0; i < shift; i++)
			nRxBuffer[RxDataSize-shift+i] = aRxBuffer[i];
	}

	/* Clear aRxBuffer */
	for (i = 0; i < RxDataSize; i++)
	{
		aRxBuffer[i] = 0;
	}

}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	HAL_Delay(100);
	HAL_NVIC_SystemReset();
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
