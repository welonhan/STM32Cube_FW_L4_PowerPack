/* Includes ------------------------------------------------------------------*/
#include "smb1381.h"
#include "power_pack.h"

/**
  * @brief  Turns selected LED On.
  * @param  Led: Specifies the Led to be set on. 
  *   This parameter can be one of following parameters:
  *            @arg  LED3
  * @retval None
  */
void BSP_SMB_Enable(void)
{
	//HAL_GPIO_WritePin(SMB_RESET_PIN_GPIO_PORT, SMB_RESET_PIN, GPIO_PIN_RESET); 
	//HAL_Delay(10);
	//HAL_GPIO_WritePin(SMB_RESET_PIN_GPIO_PORT, SMB_RESET_PIN, GPIO_PIN_SET); 
  //HAL_GPIO_WritePin(SMB_LPMODE_EN_PIN_GPIO_PORT, SMB_LPMODE_EN_PIN, GPIO_PIN_SET); 
	//HAL_GPIO_WritePin(SMB_SUSP_PIN_GPIO_PORT, SMB_SUSP_PIN, GPIO_PIN_SET); 
}

void BSP_SMB_Init(void)
{
	uint8_t temp;
	temp=BSP_I2C2_Read(SMB_ADDRESS, 0x1042,I2C_MEMADD_SIZE_16BIT);
	temp=temp|0x01;
	BSP_I2C2_Write(SMB_ADDRESS, 0x1042, I2C_MEMADD_SIZE_16BIT, temp);
	
	BSP_I2C2_Write(SMB_ADDRESS, 0x1061, I2C_MEMADD_SIZE_16BIT, 0x14);
	
	BSP_I2C2_Write(SMB_ADDRESS, 0x1070, I2C_MEMADD_SIZE_16BIT, 0xB4);
	
}

void BSP_SMB_Disable(void)
{
  HAL_GPIO_WritePin(SMB_RESET_PIN_GPIO_PORT, SMB_RESET_PIN, GPIO_PIN_RESET); 
}

uint8_t BSP_SMB_Get_ID(uint8_t *smb_revid)
{
	HAL_StatusTypeDef status = HAL_OK;
	
	status=BSP_I2C2_ReadBuffer(SMB_ADDRESS, 0x102, I2C_MEMADD_SIZE_16BIT, smb_revid, 4);
	if(status == HAL_OK)
	{
			return 0;
	}
	else 
		return 1;
}

void BSP_SMB_WLC_Charging(void)
{
  //HAL_GPIO_WritePin(SMUX_SEL_PIN_GPIO_PORT, SMUX_SEL_PIN, GPIO_PIN_SET);
	//HAL_GPIO_WritePin(PMUX1_MODE_PIN_GPIO_PORT, PMUX1_MODE_PIN, GPIO_PIN_RESET);	
}





/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
