/*
 * 001led_toggle.c
 *
 *  Created on: 13-Aug-2019
 *      Author: MAHADEVAN
 */

#include"stm32f446xx.h"

void delay()
{
	uint32_t i;
	for(i=0;i<=30000;i++);
}

int main(void)
{

	GPIO_Handle_t GpioLed;
	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
  //GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA,ENABLE);
	GPIO_Init(&GpioLed);

	while(1)
	{
		GPIO_ToggleOutputPin(GPIOA,GPIO_PIN_NO_5);
		delay();

	}









}
