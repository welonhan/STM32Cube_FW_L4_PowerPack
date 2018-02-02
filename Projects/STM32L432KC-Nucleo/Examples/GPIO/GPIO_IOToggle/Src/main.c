/**
  ******************************************************************************
  * @file    I2C/I2C_WakeUpFromStop/Src/main.c 
  * @author  MCD Application Team
  * @brief   This sample code shows how to use STM32L4xx I2C HAL API to transmit 
  *          and receive a data buffer with a communication process in stop mode
  *          based on IT transfer. 
  *          The communication is done using 2 Boards.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
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


//#define I2C_TIMING      0x0020098E

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* I2C handler declaration */
I2C_HandleTypeDef I2cHandle;

ADC_HandleTypeDef             AdcHandle;
ADC_ChannelConfTypeDef        sConfig;
uint16_t   aADCxConvertedData[4];

uint32_t usb_in_voltage, dc_in_voltage, chg_out_voltage;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void Error_Handler(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void ADC_Data_Handle(void);

uint8_t p9221_id[2];
uint8_t	p9221_fw_revision[4];
uint8_t smb1381_id[4];

uint8_t key_count=0;
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* STM32L4xx HAL library initialization:
       - Configure the Flash prefetch
       - Systick timer is configured by default as source of time base, but user 
         can eventually implement his proper time base source (a general purpose 
         timer for example or other time source), keeping in mind that Time base 
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4
       - Low Level Initialization
     */
  HAL_Init();

  /* Configure the system clock to 80 MHz */
  SystemClock_Config();
  		
  BSP_POWER_PACK_Init();
	
	BSP_SMB_Enable();
	BSP_SMB_Get_ID((uint8_t*)smb1381_id);
	BSP_SMB_Init();
	
	
	
  while(1)
  {
	  //HAL_Delay(1000);
	  //BSP_LED_Toggle(LED1);
		//BSP_LED_Toggle(LED2);
		//BSP_LED_Toggle(LED3);
		//BSP_LED_Toggle(LED4);
		ADC_Data_Handle();
  }

}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follows :
  *            System Clock source            = PLL (MSI)
  *            SYSCLK(Hz)                     = 80000000
  *            HCLK(Hz)                       = 80000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            MSI Frequency(Hz)              = 4000000
  *            PLL_M                          = 1
  *            PLL_N                          = 40
  *            PLL_R                          = 2
  *            PLL_P                          = 7
  *            PLL_Q                          = 4
  *            Flash Latency(WS)              = 4
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};

  /* MSI is enabled after System reset, activate PLL with MSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLP = 7;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* Error if LED3 is slowly blinking (1 sec. period) */
  while(1)
  {    
    BSP_LED_Toggle(LED3); 
    HAL_Delay(1000);
  } 
}

/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == SMB_CC_STS_PIN)
  {
    /* Toggle LED2 */
    BSP_LED_Toggle(LED2);
  }

  else if (GPIO_Pin == SMB_STAT_PIN)
  {
    /* Toggle LED2 */
    BSP_LED_Toggle(LED2);
  }

  else if (GPIO_Pin == SMB_SYS_PIN)
  {
      /* Toggle LED2 */
      BSP_LED_Toggle(LED2);
  }

  else if (GPIO_Pin == BOOST_OCP_INT_PIN)
  {
      /* Toggle LED2 */
      BSP_LED_Toggle(LED2);
  }

  else if (GPIO_Pin == KEY_PIN)
  {
    //HAL_Delay(10);
		key_count=key_count+1;
    BSP_LED_Light(key_count);
		if(key_count==1)
			BSP_WLC2SMB();
		else if(key_count==2)
			BSP_USB2SMB();
		else if(key_count==3)
			BSP_USB2PHONE();
		else if(key_count==4)
			BSP_WLC2PHONE();
		else if(key_count==5)
			BSP_BOOST2PHONE(0);
		else if(key_count==6)
		{
			key_count=0;
			BSP_BOOST2PHONE(1);
		}			
  }

  else if (GPIO_Pin == PHONE_ATTACHED_PIN)
  {
       /* Toggle LED2 */
      BSP_LED_Toggle(LED2);
  }

  else if (GPIO_Pin == FG_INT_PIN)
  {
       /* Toggle LED2 */
       BSP_LED_Toggle(LED2);
  }

  else if (GPIO_Pin == WLC_INT_PIN)
  {
        /* Toggle LED2 */
        BSP_LED_Toggle(LED2);
  }
}


void ADC_Data_Handle(void)
{
	uint32_t vrefint_data,vrefint_cal,vdda;
	vrefint_cal=(uint32_t)(*VREFINT_CAL_ADDR);
	
	vrefint_data=aADCxConvertedData[3];
	vdda=(3000*vrefint_cal)/vrefint_data;
	

	usb_in_voltage=(vdda*aADCxConvertedData[0]*11)/4096;
	dc_in_voltage=(vdda*aADCxConvertedData[0]*11)/4096;
	chg_out_voltage=(vdda*aADCxConvertedData[0]*11)/4096;



}



#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
