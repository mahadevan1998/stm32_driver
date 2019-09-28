/*
 * stm32f446xx_gpio_drivers.h
 *
 *  Created on: 11-Aug-2019
 *      Author: MAHADEVAN
 */

#ifndef INC_STM32F446XX_GPIO_DRIVERS_H_
#define INC_STM32F446XX_GPIO_DRIVERS_H_

#include"stm32f446xx.h"

/*
 * This is a configuration structure for a GPIO pin
 */
   typedef struct
 {
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;      /*!< Possible values from @PIN_MODES > */
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;

  } GPIO_PinConfig_t;

/*
 * This is a handle structure for a GPIO pin
 */
   typedef struct
 {
  GPIO_RegDef_t *pGPIOx;                   //this pointer holds the base address of a GPIO port to which the pin belongs
  GPIO_PinConfig_t GPIO_PinConfig;         //this holds pin configuration settings

 } GPIO_Handle_t;


 /*
  * GPIO Pin Number Macros
  */

 #define GPIO_PIN_NO_0              0
 #define GPIO_PIN_NO_1              1
 #define GPIO_PIN_NO_2              2
 #define GPIO_PIN_NO_3              3
 #define GPIO_PIN_NO_4              4
 #define GPIO_PIN_NO_5              5
 #define GPIO_PIN_NO_6              6
 #define GPIO_PIN_NO_7              7
 #define GPIO_PIN_NO_8              8
 #define GPIO_PIN_NO_9              9
 #define GPIO_PIN_NO_10             10
 #define GPIO_PIN_NO_11             11
 #define GPIO_PIN_NO_12             12
 #define GPIO_PIN_NO_13             13
 #define GPIO_PIN_NO_14             14
 #define GPIO_PIN_NO_15             15



/* @PIN_MODES
 * Pin modes macros of GPIO pin
 */

 #define GPIO_MODE_IN               0
 #define GPIO_MODE_OUT              1
 #define GPIO_MODE_ALTFN            2
 #define GPIO_MODE_ANALOG           3
 #define GPIO_MODE_IT_FT            4
 #define GPIO_MODE_IT_RT            5
 #define GPIO_MODE_IT_RFT           6

 /*
  * GPIO pin possible output types
  */
 #define GPIO_OP_TYPE_PP             0
 #define GPIO_OP_TYPE_OD		     1

/*
 * GPIO pin possible speeds
 */
 #define GPIO_SPEED_LOW               0
 #define GPIO_SPEED_MEDIUM            1
 #define GPIO_SPEED_FAST              2
 #define GPIO_SPEED_HIGH              3

 /*
  * GPIO pin pull up and pull down configuration macros
  */
 #define GPIO_NO_PUPD                 0
 #define GPIO_PIN_PU                  1
 #define GPIO_PIN_PD                  2




/************************************************************************************
 * 		             APIS SUPPORTED BY THIS DRIVER
 * 					CHECK API DEFINITION FOR MORE DETAILS
 ************************************************************************************/

/*
 * Peripheral clock setup
 */
   void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
* Initialize and De - initialize
*/

   void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
   void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Data read and write
*/

 uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
 uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
 void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
 void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
 void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ configuration and ISR handling
 */

 void GPIO_IRQInterruptConfig(uint8_t IRQNumber , uint8_t EnorDi);
 void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint32_t Priority);
 void GPIO_IRQHandling(uint8_t PinNumber);




#endif /* INC_STM32F446XX_GPIO_DRIVERS_H_ */
