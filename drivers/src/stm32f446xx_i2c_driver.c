/*
 * stm32f446xx_i2c_driver.c
 *
 *  Created on: 28-Jan-2020
 *      Author: MAHADEVAN
 */

#include"stm32f446xx_i2c_driver.h"


static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx);

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
 {
	 pI2Cx->CR1 |= (1 << I2C_CR1_START);
 }

static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	 pI2Cx->CR1 |= (1 << I2C_CR1_STOP);

}


static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1); //SlaveAddr is Slave address + r/nw bit=0
	pI2Cx->DR = SlaveAddr;
}


static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx)
{
	uint32_t dummyRead = pI2Cx->SR1;
	dummyRead = pI2Cx->SR2;
	(void)dummyRead;
}



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


/****************************** IRQ configuration and ISR handling ************************/

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

// Interrupt Priority Configuration

 void I2C_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
  {
	  //1. first lets find out the ipr register
	  	 	uint8_t iprx = IRQNumber / 4;
	  	 	uint8_t iprx_section  = IRQNumber %4 ;

	  	 	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

	  	 	*(  NVIC_PR_BASE_ADDR + iprx ) |=  ( IRQPriority << shift_amount );
 }


 void I2C_MasterSendData(I2C_Handle_t *pI2CHandle , uint8_t *pTxbuffer , uint32_t Len , uint8_t SlaveAddr)
 {

	 // 1. Generate the START condition
	 	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	 	//2. confirm that start generation is completed by checking the SB flag in the SR1
	 	//   Note: Until SB is cleared SCL will be stretched (pulled to LOW)
	 	while( !  I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_SB)   );

	 	//3. Send the address of the slave with r/nw bit set to w(0) (total 8 bits )
	 	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx,SlaveAddr);

	 	//4. Confirm that address phase is completed by checking the ADDR flag in teh SR1
	 	while( !  I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_ADDR)   );

	 	//5. clear the ADDR flag according to its software sequence
	 	//   Note: Until ADDR is cleared SCL will be stretched (pulled to LOW)
	 	I2C_ClearADDRFlag(pI2CHandle);

	 	//6. send the data until len becomes 0

	 	while(Len > 0)
	 	{
	 		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_TXE) ); //Wait till TXE is set
	 		pI2CHandle->pI2Cx->DR = *pTxbuffer;
	 		pTxbuffer++;
	 		Len--;
	 	}

	 	//7. when Len becomes zero wait for TXE=1 and BTF=1 before generating the STOP condition
	 	//   Note: TXE=1 , BTF=1 , means that both SR and DR are empty and next transmission should begin
	 	//   when BTF=1 SCL will be stretched (pulled to LOW)

	 	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_TXE) );

	 	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_BTF) );


	 	//8. Generate STOP condition and master need not to wait for the completion of stop condition.
	 	//   Note: generating STOP, automatically clears the BTF

	 	I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

}



 	 	 	 	 	 	 	 	 	 	 // API for checking I2C Flags

 uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx , uint32_t FlagName)
 {
	 if(pI2Cx->SR1 & FlagName)
	 	{
	 		return FLAG_SET;
	 	}
	 	return FLAG_RESET;
 }









