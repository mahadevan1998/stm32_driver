/*
 * stm32f446xx_spi_driver.c
 *
 *  Created on: 28-Sep-2019
 *      Author: MAHADEVAN
 */

#include"stm32f446xx_spi_driver.h"

/******************************
 * 	Peripheral Clock Setup APIs
 ******************************/

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		{
			if(pSPIx == SPI1)
			{
				SPI1_PCLK_EN();
			}else if (pSPIx == SPI2)
			{
				SPI2_PCLK_EN();
			}else if (pSPIx == SPI3)
			{
				SPI3_PCLK_EN();
			}else if (pSPIx == SPI4)
			{
				SPI4_PCLK_EN();
			}

		}
		else
		{
			if(pSPIx == SPI1)
			{
				SPI1_PCLK_DI();

			}else if(pSPIx==SPI2)
			{
				SPI2_PCLK_DI();

			}else if(pSPIx==SPI3)
			{
				SPI3_PCLK_DI();

			}else if(pSPIx==SPI4)
			{
				SPI4_PCLK_DI();

			}

		}
	}

/***************************************
 *  Initialisation and de-initialisation
 ***************************************/

void SPI_Init(SPI_Handle_t *pSPIHandle)
{

}

void SPI_DeInit(SPI_RegDef_t *pSPIx)
{

}
/*
 * Send and Receive Data
*/
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *TxBuffer, uint32_t Len)
{

}
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *RxBuffer, uint32_t Len)
{

}
/*
 * IRQ configuration and ISR handling
 */

 void SPI_IRQInterruptConfig(uint8_t IRQNumber , uint8_t EnorDi)
 {

 }
 void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint32_t Priority)
 {

 }
 void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
 {

 }



