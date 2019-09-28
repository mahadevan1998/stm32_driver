/*
 * stm32f446xx.h
 * MCU SPECIFIC HEADER FILE
 *  Created on: 10-Aug-2019
 *      Author: MAHADEVAN
 */

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_

#define __vo volatile

/**********************************START:Processor Specific Details **********************************/
/*
 * ARM Cortex Mx Processor NVIC ISERx register Addresses
 */

#define NVIC_ISER0          ( (__vo uint32_t*)0xE000E100 )
#define NVIC_ISER1          ( (__vo uint32_t*)0xE000E104 )
#define NVIC_ISER2          ( (__vo uint32_t*)0xE000E108 )
#define NVIC_ISER3          ( (__vo uint32_t*)0xE000E10c )


/*
 * ARM Cortex Mx Processor NVIC ICERx register Addresses
 */
#define NVIC_ICER0 			((__vo uint32_t*)0XE000E180)
#define NVIC_ICER1			((__vo uint32_t*)0XE000E184)
#define NVIC_ICER2  		((__vo uint32_t*)0XE000E188)
#define NVIC_ICER3			((__vo uint32_t*)0XE000E18C)


/*
 * ARM Cortex Mx Processor Priority Register Address Calculation
 */
#define NVIC_PR_BASE_ADDR 	((__vo uint32_t*)0xE000E400)

/*
 * ARM Cortex Mx Processor number of priority bits implemented in Priority Register
 */
#define NO_PR_BITS_IMPLEMENTED  4

#define NVIC_IRQ_PRI15	15


#include<stdint.h>

/**********************************START:Microcontroller Specific Details **********************************/
/*

/*
 * base addresses of flash , SRAM and system memory(ROM)
 */
#define FLASH_BASEADDR		        0x08000000U
#define SRAM1_BASEADDR		        0x20000000U
#define SRAM                        SRAM1_BASEADDR
#define SRAM2_BASEADDR		        0x2001C000U
#define ROM                         0x1FFF0000U

/*
 * base address of bus domains (AHBx/APBx)peripherals
 */

#define PERIPH_BASEADDR				0x40000000U
#define APB1PERIPH_BASEADDR         PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR         0x40010000U
#define AHB1PERIPH_BASEADDR         0x40020000U
#define AHB2PERIPH_BASEADDR         0x50000000U

/*
 * base addresses of all peripherals hanging on AHB1
 */
#define GPIO_A_BASEADDR             (AHB1PERIPH_BASEADDR + 0x0000)
#define GPIO_B_BASEADDR             (AHB1PERIPH_BASEADDR + 0x0400)
#define GPIO_C_BASEADDR             (AHB1PERIPH_BASEADDR + 0x0800)
#define GPIO_D_BASEADDR             (AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIO_E_BASEADDR             (AHB1PERIPH_BASEADDR + 0x1000)
#define GPIO_F_BASEADDR             (AHB1PERIPH_BASEADDR + 0x1400)
#define GPIO_G_BASEADDR             (AHB1PERIPH_BASEADDR + 0x1800)
#define GPIO_H_BASEADDR             (AHB1PERIPH_BASEADDR + 0x0C00)
#define RCC_BASEADDR                (AHB1PERIPH_BASEADDR + 0x3800)

/*
 *  base addresses of peripherals hanging on APB1
 */
#define SPI2_BASEADDR               (APB1PERIPH_BASEADDR + 3800)
#define SPI3_BASEADDR               (APB1PERIPH_BASEADDR + 3C00)
#define USART2_BASEADDR             (APB1PERIPH_BASEADDR + 4400)
#define USART3_BASEADDR             (APB1PERIPH_BASEADDR + 4800)
#define UART4_BASEADDR              (APB1PERIPH_BASEADDR + 4C00)
#define UART5_BASEADDR              (APB1PERIPH_BASEADDR + 5000)
#define I2C1_BASEADDR               (APB1PERIPH_BASEADDR + 5400)
#define I2C2_BASEADDR               (APB1PERIPH_BASEADDR + 5800)
#define I2C3_BASEADDR               (APB1PERIPH_BASEADDR + 5C00)

/*
 * base addresses of peripherals hanging on APB2
 */
#define EXTI_BASEADDR               (APB2PERIPH_BASEADDR + 0x3C00)
#define SYSCFG_BASEADDR             (APB2PERIPH_BASEADDR + 0x3800)
#define SPI1_BASEADDR               (APB2PERIPH_BASEADDR + 0x3000)
#define USART1_BASEADDR             (APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR             (APB2PERIPH_BASEADDR + 0x1400)

/*
 *  peripheral register definition structures
 */

typedef struct
{
	__vo uint32_t MODER ;     //
	__vo uint32_t OTYPER ;
	__vo uint32_t OSPEEDER ;
	__vo uint32_t PUPDR ;
	__vo uint32_t IDR ;
	__vo uint32_t ODR ;
	__vo uint32_t BSRR ;
	__vo uint32_t LCKR ;
	__vo uint32_t AFR[2] ;
}GPIO_RegDef_t;

typedef struct
{   __vo uint32_t CR;
    __vo uint32_t PLLCFGR;
    __vo uint32_t CFGR;
    __vo uint32_t CIR;
    __vo uint32_t AHB1RSTR;
    __vo uint32_t AHB2RSTR;
    __vo uint32_t AHB3RSTR;
         uint32_t RESERVED0;
    __vo uint32_t APB1RSTR;
    __vo uint32_t APB2RSTR;
         uint32_t RESERVED1[2];
    __vo uint32_t AHB1ENR;
    __vo uint32_t AHB2ENR;
    __vo uint32_t AHB3ENR;
         uint32_t RESERVED2;
    __vo uint32_t APB1ENR;
    __vo uint32_t APB2ENR;
         uint32_t RESERVED3[2];
    __vo uint32_t AHB1LPENR;
    __vo uint32_t AHB2LPENR;
    __vo uint32_t AHB3LPENR;
         uint32_t RESERVED4;
    __vo uint32_t APB1LPENR;
    __vo uint32_t APB2LPENR;
         uint32_t RESERVED5[2];
    __vo uint32_t BDCR;
    __vo uint32_t CSR;
         uint32_t RESERVED6[2];
    __vo uint32_t SSCGR;
    __vo uint32_t PLLI2SCFGR;
    __vo uint32_t PLLSAICFGR;
    __vo uint32_t DCKCFGR;
    __vo uint32_t CKGATEENR;
    __vo uint32_t DCKCFGR2;
}RCC_RegDef_t;

/*
 * EXTI register structure
 */

  typedef struct
 {

	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;

 }EXTI_RegDef_t;

/*
* SYSCFG register structure
*/

  typedef struct
  {
	  __vo uint32_t MEMRMP;
	  __vo uint32_t PMC;
	  __vo uint32_t EXTICR[4];
	       uint32_t RESERVED1[2];
	  __vo uint32_t CMPCR;
	       uint32_t RESERVED2[2];
	  __vo uint32_t CFGR;
  }SYSCFG_RegDef_t;


/*
 * peripheral definition (peripheral base address type casted to *xxx_RegDef_t)
 */
#define GPIOA                       ((GPIO_RegDef_t*)GPIO_A_BASEADDR)
#define GPIOB                       ((GPIO_RegDef_t*)GPIO_B_BASEADDR)
#define GPIOC                       ((GPIO_RegDef_t*)GPIO_C_BASEADDR)
#define GPIOD                       ((GPIO_RegDef_t*)GPIO_D_BASEADDR)
#define GPIOE                       ((GPIO_RegDef_t*)GPIO_E_BASEADDR)
#define GPIOF                       ((GPIO_RegDef_t*)GPIO_F_BASEADDR)
#define GPIOG                       ((GPIO_RegDef_t*)GPIO_G_BASEADDR)
#define GPIOH                       ((GPIO_RegDef_t*)GPIO_H_BASEADDR)

#define RCC                         ((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI                        ((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG						((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)
/*
 * CLOCK ENABLE MACROS FOR GPIOx peripherals
 */

#define GPIOA_PCLK_EN()              (RCC->AHB1ENR |= (1<<0))
#define GPIOB_PCLK_EN()              (RCC->AHB1ENR |= (1<<1))
#define GPIOC_PCLK_EN()              (RCC->AHB1ENR |= (1<<2))
#define GPIOD_PCLK_EN()              (RCC->AHB1ENR |= (1<<3))
#define GPIOE_PCLK_EN()              (RCC->AHB1ENR |= (1<<4))
#define GPIOF_PCLK_EN()              (RCC->AHB1ENR |= (1<<5))
#define GPIOG_PCLK_EN()              (RCC->AHB1ENR |= (1<<6))
#define GPIOH_PCLK_EN()              (RCC->AHB1ENR |= (1<<7))

/*
 *  Clock enable macros for I2Cx peripherals
 */
#define I2C1_PCLK_EN()               (RCC->APB1ENR |= (1<<21))
#define I2C2_PCLK_EN()               (RCC->APB1ENR |= (1<<22))
#define I2C3_PCLK_EN()               (RCC->APB1ENR |= (1<<23))

/*
 * Clock enable macros for SPIx peripherals
 */
#define SPI1_PCLK_EN()              (RCC->APB2ENR |= (1<<12))
#define SPI2_PCLK_EN()              (RCC->APB1ENR |= (1<<14))
#define SPI3_PCLK_EN()              (RCC->APB1ENR |= (1<<15))
#define SPI4_PCLK_EN()              (RCC->APB2ENR |= (1<<13))

/*
 * Clock enable macros for USARTx peripherals
 */
#define USART1_PCLK_EN()            (RCC->APB2ENR |= (1<<4))
#define USART2_PCLK_EN()            (RCC->APB1ENR |= (1<<17))
#define USART3_PCLK_EN()            (RCC->APB1ENR |= (1<<18))
#define UART4_PCLK_EN()             (RCC->APB1ENR |= (1<<19))
#define UART5_PCLK_EN()             (RCC->APB1ENR |= (1<<20))
#define USART6_PCLK_EN()            (RCC->APB1ENR |= (1<<5))

/*
 * Clock enable macros for SYSCFG peripheral
 */
#define SYSCFG_PCLK_EN()            (RCC->APB2ENR |= (1<<14))

/*
 * Clock disable macro for GPIOx
 */
#define GPIOA_PCLK_DI()             (RCC->AHB1ENR &= ~(1<<0))
#define GPIOB_PCLK_DI()             (RCC->AHB1ENR &= ~(1<<1))
#define GPIOC_PCLK_DI()             (RCC->AHB1ENR &= ~(1<<2))
#define GPIOD_PCLK_DI()             (RCC->AHB1ENR &= ~(1<<3))
#define GPIOE_PCLK_DI()             (RCC->AHB1ENR &= ~(1<<4))
#define GPIOF_PCLK_DI()             (RCC->AHB1ENR &= ~(1<<5))
#define GPIOG_PCLK_DI()             (RCC->AHB1ENR &= ~(1<<6))
#define GPIOH_PCLK_DI()             (RCC->AHB1ENR &= ~(1<<7))

/*
 *  Clock disable macros for I2Cx peripherals
 */
#define I2C1_PCLK_DI()             (RCC->APB1ENR &= ~(1<<21))
#define I2C2_PCLK_DI()             (RCC->APB1ENR &= ~(1<<22))
#define I2C3_PCLK_DI()             (RCC->APB1ENR &= ~(1<<23))


/*
 * Clock disable macros for SPIx
 */
#define SPI1_PCLK_DI()             (RCC->APB2ENR &= ~(1<<12))
#define SPI2_PCLK_DI()             (RCC->APB1ENR &= ~(1<<14))
#define SPI3_PCLK_DI()             (RCC->APB1ENR &= ~(1<<15))
#define SPI4_PCLK_DI()             (RCC->APB2ENR &= ~(1<<13))

/*
 * Clock disable macros for USARTx peripherals
 */
#define USART1_PCLK_DI()            (RCC->APB2ENR &= ~(1<<4))
#define USART2_PCLK_DI()            (RCC->APB1ENR &= ~(1<<17))
#define USART3_PCLK_DI()            (RCC->APB1ENR &= ~(1<<18))
#define UART4_PCLK_DI()             (RCC->APB1ENR &= ~(1<<19))
#define UART5_PCLK_DI()             (RCC->APB1ENR &= ~(1<<20))
#define USART6_PCLK_DI()            (RCC->APB1ENR &= ~(1<<5))

/*
 * Clock disable macros for SYSCFG peripheral
 */
#define SYSCFG_PCLK_DI()            (RCC->APB2ENR &= ~(1<<14))

/*
 *  Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOF_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); }while(0)
#define GPIOG_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); }while(0)
#define GPIOH_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); }while(0)
#define GPIOI_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8)); }while(0)

  /*
   *  returns port code for given GPIOx base address
   */
  /*
   * This macro returns a code( between 0 to 7) for a given GPIO base address(x)
   */
  #define GPIO_BASEADDR_TO_CODE(x)      ( (x == GPIOA)?0:\
  										(x == GPIOB)?1:\
  										(x == GPIOC)?2:\
  										(x == GPIOD)?3:\
  								        (x == GPIOE)?4:\
  								        (x == GPIOF)?5:\
  								        (x == GPIOG)?6:\
  								        (x == GPIOH)?7:0)

  /*
   * IRQ(Interrupt Request) Numbers of STM32F407x MCU
   */

  #define IRQ_NO_EXTI0 		6
  #define IRQ_NO_EXTI1 		7
  #define IRQ_NO_EXTI2 		8
  #define IRQ_NO_EXTI3 		9
  #define IRQ_NO_EXTI4 		10
  #define IRQ_NO_EXTI9_5 	23
  #define IRQ_NO_EXTI15_10 	40
  #define IRQ_NO_SPI1		35
  #define IRQ_NO_SPI2       36
  #define IRQ_NO_SPI3       51
  #define IRQ_NO_SPI4       84
  #define IRQ_NO_I2C1_EV    31
  #define IRQ_NO_I2C1_ER    32
  #define IRQ_NO_USART1	    37
  #define IRQ_NO_USART2	    38
  #define IRQ_NO_USART3	    39
  #define IRQ_NO_UART4	    52
  #define IRQ_NO_UART5	    53
  #define IRQ_NO_USART6	    71





/*
 * Some generic macros
*/

#define ENABLE                      1
#define DISABLE                     0
#define SET                         ENABLE
#define RESET                       DISABLE
#define GPIO_PIN_SET				SET
#define GPIO_PIN_RESET				RESET

#include"stm32f446xx_gpio_drivers.h"

#endif /* INC_STM32F446XX_H_ */
