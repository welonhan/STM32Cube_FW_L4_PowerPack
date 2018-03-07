/* Includes ------------------------------------------------------------------*/
#include "smb1381.h"
#include "power_pack.h"
#include "smb-reg.h"

#include <stdio.h>

HAL_StatusTypeDef smb138x_init_hw(void);


void BSP_SMB_Enable(void)
{
	HAL_GPIO_WritePin(SMB_RESET_PIN_GPIO_PORT, SMB_RESET_PIN, GPIO_PIN_RESET); 
	HAL_Delay(10);
	HAL_GPIO_WritePin(SMB_RESET_PIN_GPIO_PORT, SMB_RESET_PIN, GPIO_PIN_SET); 
  HAL_GPIO_WritePin(SMB_LPMODE_EN_PIN_GPIO_PORT, SMB_LPMODE_EN_PIN, GPIO_PIN_SET); 
	HAL_GPIO_WritePin(SMB_SUSP_PIN_GPIO_PORT, SMB_SUSP_PIN, GPIO_PIN_SET); 
}

void BSP_SMB_Init(void)
{
	uint8_t temp;
	HAL_StatusTypeDef status;
	
	status=smb138x_init_hw();
	if(status!=HAL_OK)
		printf("smb138x init hw error!\n\r");
		
	//charging enable
	temp=BSP_I2C2_Read(SMB_ADDRESS, 0x1042,I2C_MEMADD_SIZE_16BIT);
	temp=temp|0x01;
	if((BSP_I2C2_Write(SMB_ADDRESS, 0x1042, I2C_MEMADD_SIZE_16BIT, temp))!=HAL_OK)
		printf("SBM1381 charging enable setting error!\n\r");		
	
	//charge current
	if((BSP_I2C2_Write(SMB_ADDRESS, 0x1061, I2C_MEMADD_SIZE_16BIT, 0x14))!=HAL_OK)
		printf("SBM1381 charge current setting error!\n\r");
	
	//float voltage 4.35V
	if((BSP_I2C2_Write(SMB_ADDRESS, 0x1070, I2C_MEMADD_SIZE_16BIT, 0xB4))!=HAL_OK)
		printf("SBM1381 float voltage setting error!\n\r");
}

HAL_StatusTypeDef smblib_masked_write(uint16_t reg, uint8_t bits, uint8_t data)
{
	uint8_t temp;
	HAL_StatusTypeDef status = HAL_OK;
	
	temp=BSP_I2C2_Read(SMB_ADDRESS, reg, I2C_MEMADD_SIZE_16BIT);
	if(data == 0)
	{
		temp=temp&(~(1<<bits));
	}
	else if (data==1)
	{
		temp=temp|(1<<bits);
	}
	
	status=BSP_I2C2_Write(SMB_ADDRESS, reg, I2C_MEMADD_SIZE_16BIT, temp);	
	
	return status;	
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


HAL_StatusTypeDef smb138x_init_hw(void)
{
	HAL_StatusTypeDef rc = HAL_OK;

	/* Disable OTG */
	rc = smblib_masked_write(CMD_OTG_REG, OTG_EN_BIT, 0);
	if (rc != HAL_OK) {
		//pr_err("Couldn't disable OTG rc=%d\n", rc);
		return rc;
	}

	/* Unsuspend USB input */
	rc = smblib_masked_write(USBIN_CMD_IL_REG, USBIN_SUSPEND_BIT, 0);
	if (rc != HAL_OK) {
		//pr_err("Couldn't unsuspend USB, rc=%d\n", rc);
		return rc;
	}

	#if 0
	/* configure to a fixed 700khz freq to avoid tdie errors */
	rc = smblib_set_charge_param(chg, &chg->param.freq_buck, 700);
	if (rc != HAL_OK) {
		//pr_err("Couldn't configure 700Khz switch freq rc=%d\n", rc);
		return rc;
	}
	#endif
	
	/* configure charge enable for software control; active high */
	rc = smblib_masked_write(CHGR_CFG2_REG,CHG_EN_POLARITY_BIT , 0);
	rc = smblib_masked_write(CHGR_CFG2_REG,CHG_EN_SRC_BIT, 0);
	if (rc != HAL_OK) {
		//pr_err("Couldn't configure charge enable source rc=%d\n", rc);
		return rc;
	}

	#if 0
	/* enable the charging path */
	rc = vote(chg->chg_disable_votable, DEFAULT_VOTER, false, 0);
	if (rc != HAL_OK) {
		//pr_err("Couldn't enable charging rc=%d\n", rc);
		return rc;
	}
	#endif
	
	/*
	 * trigger the usb-typec-change interrupt only when the CC state
	 * changes, or there was a VBUS error
	 */
	rc = smblib_masked_write(TYPE_C_INTRPT_ENB_REG,TYPEC_CCSTATE_CHANGE_INT_EN_BIT,1);
	rc = smblib_masked_write(TYPE_C_INTRPT_ENB_REG,TYPEC_VBUS_ERROR_INT_EN_BIT,1);
	if (rc != HAL_OK) {
		//pr_err("Couldn't configure Type-C interrupts rc=%d\n", rc);
		return rc;
	}

	/* configure VCONN for software control */
	rc = smblib_masked_write(TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
				 VCONN_EN_SRC_BIT ,
				 1);
	rc = smblib_masked_write(TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
				 VCONN_EN_VALUE_BIT,
				 0);
	if (rc != HAL_OK) {
		//pr_err("Couldn't configure VCONN for SW control rc=%d\n", rc);
		return rc;
	}

	/* configure VBUS for software control */
	rc = smblib_masked_write(OTG_CFG_REG, OTG_EN_SRC_CFG_BIT, 0);
	if (rc != HAL_OK) {
		//pr_err("Couldn't configure VBUS for SW control rc=%d\n", rc);
		return rc;
	}

	/* configure power role for dual-role */
	rc = smblib_masked_write(TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
				 UFP_EN_CMD_BIT, 0);
	rc = smblib_masked_write(TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
				 DFP_EN_CMD_BIT, 0);
	rc = smblib_masked_write(TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
				 TYPEC_DISABLE_CMD_BIT, 0);
	if (rc != HAL_OK) {
		//pr_err("Couldn't configure power role for DRP rc=%d\n", rc);
		return rc;
	}

	#if 0
	if (chip->wa_flags & OOB_COMP_WA_BIT) {
		rc = smblib_masked_write(chg, SMB2CHG_MISC_ENG_SDCDC_CFG2,
					ENG_SDCDC_SEL_OOB_VTH_BIT,
					ENG_SDCDC_SEL_OOB_VTH_BIT);
		if (rc != HAL_OK) {
			//pr_err("Couldn't configure the OOB comp threshold rc = %d\n",
									rc);
			return rc;
		}

		rc = smblib_masked_write(chg, SMB2CHG_MISC_ENG_SDCDC_CFG6,
				DEAD_TIME_MASK, HIGH_DEAD_TIME_MASK);
		if (rc != HAL_OK) {
			//pr_err("Couldn't configure the sdcdc cfg 6 reg rc = %d\n",
									rc);
			return rc;
		}
	}
	#endif
	
	return rc;
}



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
