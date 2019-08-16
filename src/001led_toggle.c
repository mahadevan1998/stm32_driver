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
	for(i=0;i<=500000/2;i++);
}

int main(void)
{

	GPIO_Handle_t GpioLed;
	GPIO_Handle_t Btn;

	Btn.pGPIOx = GPIOC;
    Btn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
    Btn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	//GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
    //Btn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	Btn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	Btn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;



	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
  //GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA,ENABLE);
	GPIO_PeriClockControl(GPIOC,ENABLE);
	GPIO_Init(&GpioLed);
	GPIO_Init(&Btn);

	while(1)
	{
		if(!GPIO_ReadFromInputPin(GPIOC,13))
	{
		delay();
		GPIO_ToggleOutputPin(GPIOA,GPIO_PIN_NO_5);

		}
		//GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NO_5, !GPIO_ReadFromInputPin(GPIOC,13));
	}


    return 0;

}
