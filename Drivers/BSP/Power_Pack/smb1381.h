/**
  ******************************************************************************
  * @file    p9221.h
  * @author  
  * @version V1.0.0
  * @date    31-January-2018
  * @brief   
  *          
  *
  ******************************************************************************
  */ 
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SMB1381_H
#define __SMB1381_H

#ifdef __cplusplus
 extern "C" {
#endif


#include "stm32l4xx_hal.h"
	 
#define SMB_ADDRESS 				0x10
	 



void BSP_SMB_Enable(void);
void BSP_SMB_Disable(void);	 
void BSP_SMB_Init(void);
uint8_t BSP_SMB_Get_ID(uint8_t *smb_revid);
    
#ifdef __cplusplus
}
#endif

#endif /* __SMB1381_H*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

