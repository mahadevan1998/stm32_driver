/*
 * stm32f446xx_i2c_driver.c
 *
 *  Created on: 28-Jan-2020
 *      Author: MAHADEVAN
 */

#include"stm32f446xx_i2c_driver.h"

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


