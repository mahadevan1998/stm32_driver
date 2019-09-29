/*
 * stm32f446xx_spi_driver.h
 *
 *  Created on: 28-Sep-2019
 *      Author: MAHADEVAN
 */

#ifndef INC_STM32F446XX_SPI_DRIVER_H_
#define INC_STM32F446XX_SPI_DRIVER_H_

#include"stm32f446xx.h"

typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;

}SPI_Config_t;

typedef struct
{
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPIConfig;
}SPI_Handle_t;


/******************************************************
  	  	  APIs supported by SPI Driver
  For more information refer the function definitions
 *****************************************************/

/******************************
 * 	Peripheral Clock Setup APIs
 ******************************/

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);


/***************************************
 *  Initialisation and de-initialisation
 ***************************************/
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * Send and Receive Data
*/
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *TxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *RxBuffer, uint32_t Len);

/*
 * IRQ configuration and ISR handling
 */

 void SPI_IRQInterruptConfig(uint8_t IRQNumber , uint8_t EnorDi);
 void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint32_t Priority);
 void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);

#endif /* INC_STM32F446XX_SPI_DRIVER_H_ */
