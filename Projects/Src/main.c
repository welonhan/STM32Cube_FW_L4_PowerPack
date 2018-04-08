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
#include <stdio.h>
#include "cmsis_os.h"

//#define I2C_TIMING      0x0020098E

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* I2C handler declaration */
I2C_HandleTypeDef I2cHandle;

ADC_HandleTypeDef             AdcHandle;
ADC_ChannelConfTypeDef        sConfig;
uint16_t   										aADCxConvertedData[4];

UART_HandleTypeDef 						UartHandle;
__IO ITStatus UartReady = RESET;

DMA_HandleTypeDef         		DmaHandle;

uint32_t usb_in_voltage, dc_in_voltage, chg_out_voltage,vdda;

uint32_t I2c1Timeout = BSP_I2C1_TIMEOUT_MAX;    /*<! Value of Timeout when I2C1 communication fails */
uint32_t I2c2Timeout = BSP_I2C2_TIMEOUT_MAX;    /*<! Value of Timeout when I2C1 communication fails */
I2C_HandleTypeDef powerpack_I2c1,powerpack_I2c2;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void ADC_Data_Handle(void);

uint8_t p9221_id[2];
uint8_t	p9221_fw_revision[4];
uint8_t smb1381_id[4];

uint8_t key_count=0;
uint8_t reg_add_h,reg_add_l,reg_data,prefix;
uint16_t reg_add16;

uint8_t aRxBuffer[5];

osMailQDef (mail_pool_q, 5, PACK_INFO);  // Declare mail queue
osMailQId  (mail_pool_q_id);              // Mail queue ID

PACK_INFO pack_info;

typedef enum
{
	STATUS_STANDBY=0,
	STATUS_USB2PACK,
	STATUS_USB2BOTH,
	STATUS_USB2PHONE,
	STATUS_DCIN2PACK,
	STATUS_DCIN2BOTH,
	STATUS_DCIN2PHONE,
	STATUS_BOOST2PHONE,
	STATUS_PHONE2PACK
}PACK_STATUS;
	
uint8_t status_pack=STATUS_STANDBY;

typedef enum
{
  THREAD_1 = 0,
  THREAD_2,
	THREAD_3
} Thread_TypeDef;


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
osThreadId UART3Thread1_id, ADCThread2_id, CHARGINGThread3_id;
/* Private function prototypes -----------------------------------------------*/
static void UART3_Thread1(void const *argument);
static void ADC_Thread2(void const *argument);
static void CHARGING_Thread3(void const *argument);
PACK_STATUS get_pack_status(PACK_INFO *ptr);

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
	
	//HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_0);

  /* Configure the system clock to 80 MHz */
  SystemClock_Config();
  		
  BSP_POWER_PACK_Init();
	printf("Power pack hardware init ok!\n\r");
	
	BSP_SMB_Enable();
	BSP_SMB_Init();
	
	//BSP_USB2SMB();
	pack_info.ATTACH=0;
	pack_info.USB=0;
	pack_info.DCIN=0;
	pack_info.PACK_SOC=0;
	pack_info.PHONE_SOC=0;	
	
	//ADC_Data_Handle();
	//printf("*****\n\rVDDA:%dmV\n\rUSB VOLTAGE:%dmV\n\rDC VOLTAGE:%dmV\n\rCHG VOLTAGE:%dmV\n\r",vdda,usb_in_voltage,dc_in_voltage,chg_out_voltage);
	
  /* Thread 1 definition */
  osThreadDef(THREAD_1, UART3_Thread1, osPriorityNormal, 0, 1024);
  
  /* Thread 2 definition */
  osThreadDef(THREAD_2, ADC_Thread2, osPriorityNormal, 0, 1024);
  
	/* Thread 3 definition */
  osThreadDef(THREAD_3, CHARGING_Thread3, osPriorityNormal, 0, 1024);
	
  /* Start thread 1 */
  UART3Thread1_id = osThreadCreate(osThread(THREAD_1), NULL);

  /* Start thread 2 */
  ADCThread2_id = osThreadCreate(osThread(THREAD_2), NULL);  

	/* Start thread 3 */
	CHARGINGThread3_id = osThreadCreate(osThread(THREAD_3), NULL); 
  
	mail_pool_q_id = osMailCreate(osMailQ(mail_pool_q), NULL);
	
  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  for (;;);

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
  * @brief  Tx Transfer completed callback
  * @param  UartHandle: UART handle. 
  * @note   This example shows a simple way to report end of IT Tx transfer, and 
  *         you can add your own implementation. 
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: transfer complete */
  UartReady = SET;

  
}

/**
  * @brief  Rx Transfer completed callback
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report end of DMA Rx transfer, and 
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: transfer complete */
  UartReady = SET; 
	osSignalSet (UART3Thread1_id, 0);
}


/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  PACK_INFO *mptr;
	
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
		BSP_LED_Toggle(LED2);
  }

  else if (GPIO_Pin == PHONE_ATTACHED_PIN)
  {
    /* Toggle LED2 */
    BSP_LED_Toggle(LED2);
		if(HAL_GPIO_ReadPin(PHONE_ATTACHED_PIN_GPIO_PORT,PHONE_ATTACHED_PIN))
			pack_info.ATTACH=1;
		else
			pack_info.ATTACH=0;
		mptr = osMailAlloc(mail_pool_q_id, osWaitForever);       	// Allocate memory  
	
		mptr->ATTACH		=pack_info.ATTACH;
		mptr->USB				=pack_info.USB;
		mptr->DCIN			=pack_info.DCIN;
		mptr->PACK_SOC	=pack_info.PACK_SOC;
		mptr->PHONE_SOC	=pack_info.PHONE_SOC;
		mptr->PHONE_USB_VOLTAGE	=pack_info.PHONE_USB_VOLTAGE;
		osMailPut(mail_pool_q_id, mptr);                         	// Send Mail
		
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

/**
  * @brief  UART3 thread 1
  * @param  thread not used
  * @retval None
  */
static void UART3_Thread1(void const *argument)
{
  //uint32_t count = 0;
  (void) argument;
  PACK_INFO *mptr;
	osEvent evt;
	
  for (;;)
  {
		if(HAL_UART_Receive_IT(&UartHandle, (uint8_t *)aRxBuffer, 5) != HAL_OK)
		{
			printf("UART Error\n\r");
		}
		evt=osSignalWait(0,osWaitForever );
		/*##-3- Wait for the end of the transfer ###################################*/   
		
//		while (UartReady != SET)
//		{
//		}
		if(evt.status==osEventSignal){
		/* Reset transmission flag */
		UartReady = RESET;
		//osSignalClear(UART3Thread1_id,0);
		if((aRxBuffer[0]==0xAA)&&aRxBuffer[1]==0x0E)//write registor
		{
			reg_add16=(uint16_t)(aRxBuffer[2]<<8)+(uint16_t)aRxBuffer[3];
			printf("Write reg_add16:0x%x,reg_data=0x%x\n\r",reg_add16,aRxBuffer[4]);
			if((BSP_I2C2_Write(SMB_ADDRESS, reg_add16, I2C_MEMADD_SIZE_16BIT, reg_data))!=HAL_OK)
				printf("SBM1381 setting error!\n\r");		
		}
		else if((aRxBuffer[0]==0xAA)&&aRxBuffer[1]==0x0F)//read registor
		{				
			reg_add16=(uint16_t)(aRxBuffer[2]<<8)+(uint16_t)aRxBuffer[3];
			reg_data=BSP_I2C2_Read(SMB_ADDRESS, reg_add16, I2C_MEMADD_SIZE_16BIT);
			printf("Read reg_add16:0x%x,reg_data=0x%x\n\r",reg_add16,reg_data);		
		}
		else if((aRxBuffer[0]==0xAA)&&aRxBuffer[1]==0x00)//firmware version
		{				
			printf("PACK firmware version:%x\n\r",__PACK_BSP_VERSION);	
			
		}
		else if((aRxBuffer[0]==0xAA)&&aRxBuffer[1]==0x01)//read pack soc
		{				
			printf("PACK soc:%d\n\r",pack_info.PACK_SOC);		
		}
		else if((aRxBuffer[0]==0xAA)&&aRxBuffer[1]==0x02)//info phone soc
		{				
			pack_info.PHONE_SOC=((uint16_t)(aRxBuffer[2]<<8)+aRxBuffer[3]);
			printf("PHONE SOC:%d\n\r",pack_info.PHONE_SOC);	
			mptr = osMailAlloc(mail_pool_q_id, osWaitForever);       	// Allocate memory  
			mptr->ATTACH		=pack_info.ATTACH;
			mptr->USB				=pack_info.USB;
			mptr->DCIN			=pack_info.DCIN;
			mptr->PACK_SOC	=pack_info.PACK_SOC;
			mptr->PHONE_SOC	=pack_info.PHONE_SOC;	
			osMailPut(mail_pool_q_id, mptr);                         	// Send Mail
			osThreadYield();
		}
		else if((aRxBuffer[0]==0xAA)&&aRxBuffer[1]==0x03)//info phone usb voltage
		{				
			pack_info.PHONE_USB_VOLTAGE=((uint16_t)(aRxBuffer[2]<<8)+aRxBuffer[3]);
			if(pack_info.PHONE_USB_VOLTAGE)
				pack_info.PHONE_USB=1;
			else
				pack_info.PHONE_USB=0;
			printf("PHONE USB VOLTAGE:%dmV\n\r",pack_info.PHONE_USB_VOLTAGE);	
			mptr = osMailAlloc(mail_pool_q_id, osWaitForever);       	// Allocate memory  
			mptr->ATTACH		=pack_info.ATTACH;
			mptr->USB				=pack_info.USB;
			mptr->DCIN			=pack_info.DCIN;
			mptr->PACK_SOC	=pack_info.PACK_SOC;
			mptr->PHONE_USB	=pack_info.PHONE_USB;
			mptr->PHONE_USB_VOLTAGE	=pack_info.PHONE_USB_VOLTAGE;
			osMailPut(mail_pool_q_id, mptr);  
			osThreadYield();			
		}    
  }
}
}

static void ADC_Thread2(void const *argument)
{
	uint32_t vrefint_data,vrefint_cal;
	vrefint_cal=(uint32_t)(*VREFINT_CAL_ADDR);
	
	for(;;)
	{
		vrefint_data=aADCxConvertedData[3];
		vdda=(3000*vrefint_cal)/vrefint_data;	

		usb_in_voltage=(vdda*aADCxConvertedData[0])/4096;
		dc_in_voltage=(vdda*aADCxConvertedData[1])/4096;
		chg_out_voltage=(vdda*aADCxConvertedData[2])/4096;
		printf("vdda=%dmV\n\r",vdda);
		osDelay(2000);
		osThreadYield();
	}
}

static void CHARGING_Thread3(void const *argument)
{
	PACK_INFO *rptr;
	osEvent  evt;
	PACK_STATUS status;
  for(;;)
	{
		evt = osMailGet(mail_pool_q_id, 1000); 
		if (evt.status == osEventMail) 
		{
			rptr=evt.value.p;
			printf("Attach:%d\n\r",rptr->ATTACH);
			printf("USB:%d\n\r",rptr->USB);
			printf("DCIN:%d\n\r",rptr->DCIN);
			printf("PHONE_USB:%d\n\r",rptr->PHONE_USB);
			printf("USB_VOL:%d\n\r",rptr->PHONE_USB_VOLTAGE);
			printf("PHONE SOC:%d\n\r",rptr->PHONE_SOC);
			printf("PACK SOC:%d\n\r",rptr->PACK_SOC);
			status=get_pack_status(rptr);
			osMailFree(mail_pool_q_id, rptr);
		
		
		switch(status){
			case STATUS_STANDBY:
			{
				BSP_STANDBY();
				printf("standby\n\r");
				break;
			}
			case STATUS_USB2PACK:
			{
				BSP_USB2PACK();
				printf("usb to pack\n\r");
				break;
			}
			case STATUS_USB2BOTH:
			{
				BSP_USB2BOTH();
				printf("usb to both\n\r");
				break;
			}
			case STATUS_USB2PHONE:
			{
				BSP_USB2PHONE();
				printf("dcin to phone\n\r");
				break;
			}
			case STATUS_DCIN2PACK:
			{
				BSP_DCIN2PACK();
				printf("dcin to pack\n\r");
				break;
			}
			case STATUS_DCIN2BOTH:
			{
				BSP_DCIN2BOTH();
				printf("dc int to phone\n\r");
				break;
			}
			case STATUS_DCIN2PHONE:
			{
				BSP_DCIN2PHONE();
				printf("dcin to phone\n\r");
				break;
			}
			case STATUS_BOOST2PHONE:
			{
				BSP_BOOST2PHONE(1);
				printf("boost to phone\n\r");
				break;
			}
			case STATUS_PHONE2PACK:
			{
				BSP_PHONE2PACK();
				printf("phone to pack\n\r");
				break;
			}
			default:
			{
				BSP_STANDBY();
				printf("default\n\r");
				break;
			}
		}			
		}
	}
}

int fputc(int ch, FILE *f)
{
	UartHandle.Instance->TDR = ((uint8_t)ch & (uint16_t)0x01FF);
	while (!(UartHandle.Instance->ISR	& USART_ISR_TXE));
	return (ch);
}

int fgetc(FILE *f)
{
	uint16_t uhMask;
	
	UART_WaitOnFlagUntilTimeout(&UartHandle,USART_ISR_RXNE,RESET,0,10);
	
	UART_MASK_COMPUTATION(&UartHandle);
  
	uhMask = UartHandle.Mask;
	return (int)(UartHandle.Instance->RDR & uhMask);
}

PACK_STATUS get_pack_status(PACK_INFO *ptr)
{
	PACK_STATUS status;
	if((ptr->ATTACH==0)&&	(ptr->USB==0)&&(ptr->DCIN==0))
		status=STATUS_STANDBY;
	else if((ptr->ATTACH==0)&&	(ptr->USB==1))
		status=STATUS_USB2PACK;
	else if((ptr->ATTACH==1)&&	(ptr->USB==1)&&(ptr->PHONE_USB==1))
		status=STATUS_USB2PACK;
	else if((ptr->ATTACH==1)&&	(ptr->USB==1)&&(ptr->PHONE_USB==0)&&(ptr->PHONE_SOC>=90))
		status=STATUS_USB2BOTH;
	else if((ptr->ATTACH==1)&&	(ptr->USB==1)&&(ptr->DCIN==0)&&(ptr->PHONE_USB==0)&&(ptr->PHONE_SOC<90))
		status=STATUS_USB2PHONE;
	else if((ptr->ATTACH==0)&&	(ptr->USB==1)&&(ptr->DCIN==1))
		status=STATUS_DCIN2PACK;
	else if((ptr->ATTACH==1)&&	(ptr->USB==0)&&(ptr->DCIN==1)&&(ptr->PHONE_USB==0)&&(ptr->PHONE_SOC>=90))
		status=STATUS_DCIN2BOTH;
	else if((ptr->ATTACH==1)&&	(ptr->USB==0)&&(ptr->DCIN==1)&&(ptr->PHONE_USB==0)&&(ptr->PHONE_SOC<90))
		status=STATUS_DCIN2PHONE;
	else if((ptr->ATTACH==1)&&	(ptr->USB==0)&&(ptr->DCIN==0)&&(ptr->PHONE_USB==0)&&(ptr->PACK_SOC>2))
		status=STATUS_BOOST2PHONE;
	else if((ptr->ATTACH==1)&&	(ptr->USB==0)&&(ptr->DCIN==0)&&(ptr->PHONE_USB==1)&&(ptr->PHONE_SOC>90))
		status=STATUS_PHONE2PACK;
	return status;
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
