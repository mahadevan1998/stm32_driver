/*
 * stm32f446xx_i2c_driver.c
 *
 *  Created on: 28-Jan-2020
 *      Author: MAHADEVAN
 */

#include"stm32f446xx_i2c_driver.h"

uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};

uint8_t APB1_PreScaler[4] = { 2, 4 , 8, 16};

/***************************** API to calculate clock frequency of APB1 **********************************/

uint32_t RCC_GetPClk1Value(void)
{
	uint32_t pclk1,SystemClk;

		uint8_t clksrc,temp,ahbp,apb1p;

		clksrc = ((RCC->CFGR >> 2) & 0x3);

		if(clksrc == 0 )
		{
			SystemClk = 16000000;
		}else if(clksrc == 1)
		{
			SystemClk = 8000000;
		}else if (clksrc == 2)
		{
			SystemClk = RCC_GetPLLOutputClock();
		}

		//for ahb
		temp = ((RCC->CFGR >> 4 ) & 0xF);

		if(temp < 8)
		{
			ahbp = 1;
		}else
		{
			ahbp = AHB_PreScaler[temp-8];
		}



		//apb1
		temp = ((RCC->CFGR >> 10 ) & 0x7);

		if(temp < 4)
		{
			apb1p = 1;
		}else
		{
			apb1p = APB1_PreScaler[temp-4];
		}

		pclk1 =  (SystemClk / ahbp) /apb1p;

		return pclk1;

}






/************** ENABLE OR DISABLE THE I2C PERIPHERAL**********************/
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)

{

	if(EnOrDi == ENABLE)

	{
        pI2Cx->CR1 |= (1 << I2C_CR1_PE);

		//pI2cBaseAddress->CR1 |= I2C_CR1_PE_Bit_Mask;

	}else

	{

		pI2Cx->CR1 &= ~(1 << 0);

	}
}



/*
 	 This API enables or disables the clock foe I2C peripheral
*/

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if( pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}

		if( pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}
		if( pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}
	}

	else
	{
		if( pI2Cx == I2C1)
		{
			I2C1_PCLK_DI();
		}

		if( pI2Cx == I2C2)
		{
			I2C2_PCLK_DI();
		}
		if( pI2Cx == I2C3)
		{
			I2C3_PCLK_DI();
		}

	}

}
/********************************* API FOR I2C INITIALISATION ***************************************/

void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg = 0;

//1.configure the ACK control

	tempreg |= (pI2CHandle->I2C_Config.I2C_AckControl << 10) ;
	pI2CHandle->pI2Cx->CR1 = tempreg;

//2.configure the freq field

	tempreg = 0;
	tempreg |= RCC_GetPClk1Value()/1000000U;
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F);

//3.configure the device address in the OAR register
	tempreg = 0;
	tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress <<1;
	tempreg |= 1 << 14;
	pI2CHandle->pI2Cx->OAR1 = tempreg;

//4.ccr calculation

	uint16_t ccr_value = 0;
	tempreg = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
		{
			//mode is standard mode
			ccr_value = (RCC_GetPCLK1Value() / ( 2 * pI2CHandle->I2C_Config.I2C_SCLSpeed ) );
			tempreg |= (ccr_value & 0xFFF);
		}else
		{
			//mode is fast mode
			tempreg |= ( 1 << 15);
			tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);
			if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
			{
				ccr_value = (RCC_GetPCLK1Value() / ( 3 * pI2CHandle->I2C_Config.I2C_SCLSpeed ) );
			}else
			{
				ccr_value = (RCC_GetPCLK1Value() / ( 25 * pI2CHandle->I2C_Config.I2C_SCLSpeed ) );
			}
			tempreg |= (ccr_value & 0xFFF);
		}
		pI2CHandle->pI2Cx->CCR = tempreg;

//5. TRISE Configuration



}



/*
*************************This API resets the I2C peripheral registers******************************
*/
void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
	if(pI2Cx == I2C1)
	{
		I2C1_REG_RESET();
	}
	if(pI2Cx == I2C1)
	{
	    I2C2_REG_RESET();
	}
	if(pI2Cx == I2C1)
	{
		I2C3_REG_RESET();
	}
}


/*
  * IRQ configuration and ISR handling
  */

  void I2C_IRQInterruptConfig(uint8_t IRQNumber , uint8_t EnorDi)
  {
	  if(EnorDi == ENABLE)
	  	 	{
	  	 		if(IRQNumber <= 31)
	  	 		{
	  	 			//program ISER0 register
	  	 			*NVIC_ISER0 |= ( 1 << IRQNumber );

	  	 		}else if(IRQNumber > 31 && IRQNumber < 64 ) //32 to 63
	  	 		{
	  	 			//program ISER1 register
	  	 			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
	  	 		}
	  	 		else if(IRQNumber >= 64 && IRQNumber < 96 )
	  	 		{
	  	 			//program ISER2 register //64 to 95
	  	 			*NVIC_ISER3 |= ( 1 << (IRQNumber % 64) );
	  	 		}
	  	 	}else
	  	 	{
	  	 		if(IRQNumber <= 31)
	  	 		{
	  	 			//program ICER0 register
	  	 			*NVIC_ICER0 |= ( 1 << IRQNumber );
	  	 		}else if(IRQNumber > 31 && IRQNumber < 64 )
	  	 		{
	  	 			//program ICER1 register
	  	 			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
	  	 		}
	  	 		else if(IRQNumber >= 6 && IRQNumber < 96 )
	  	 		{
	  	 			//program ICER2 register
	  	 			*NVIC_ICER3 |= ( 1 << (IRQNumber % 64) );
	  	 		}
	  	 	}
	  }
  void I2C_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
  {
	  //1. first lets find out the ipr register
	  	 	uint8_t iprx = IRQNumber / 4;
	  	 	uint8_t iprx_section  = IRQNumber %4 ;

	  	 	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

	  	 	*(  NVIC_PR_BASE_ADDR + iprx ) |=  ( IRQPriority << shift_amount );
  }


