/**
 ******************************************************************************
 * @file           : main.c
 * @author         : josefholub, https://github.com/josefholub/MMIA
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
#include "stm32f0xx.h"

#define LED_TIME_BLINK 300		//casova konstanta
#define BUTTON_DEBOUNCE 40
#define BUTTON_DEBOUNCE_SHORT 5
#define LED_TIME_SHORT 100
#define LED_TIME_LONG 1000

static volatile uint32_t Tick;	//promenna casovace

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

void EXTI0_1_IRQHandler(void)
{
	if (EXTI->PR & EXTI_PR_PR0)		// check line 0 has triggered the IT
	{
		EXTI->PR |= EXTI_PR_PR0; 	// clear the pending bit
		GPIOB->ODR ^= (1<<0);
	}
}

void SysTick_Handler(void)			//vektor preruseni
{
	Tick++;							//kazdou ms se zvysi o 1
}

void blikac(void)
{
	static uint32_t delay;

	if (Tick > delay + LED_TIME_BLINK)
	{
		GPIOA->ODR ^= (1<<4);
		delay = Tick;
	}
}

void tlacitka(void)
{
	static uint32_t debounce1 = 0;
	static uint32_t debounce2 = 0;
	static uint32_t off_time1;
	static uint32_t off_time2;

	if (Tick > debounce1 + BUTTON_DEBOUNCE)
	{
//		static uint32_t old_s2;					//puvodoni hodnota stavu vstpniho pinu S2
//		uint32_t new_s2  = GPIOC->IDR& (1<<0);	//aktualni hodnota stavu vstpniho pinu S2
		debounce1 = Tick;						//prirazeni aktualniho casu pomoci Tick

/*		if(old_s2 && !new_s2)					//falling edge
		{
			off_time2 = Tick + LED_TIME_SHORT;	//nastaveni vypinaciho casu
			GPIOB->BSRR = (1<<0);				//rozsviceni LED2
		}
		old_s2 = new_s2;
*/

		static uint32_t old_s1;					//puvodoni hodnota stavu vstpniho pinu S1
		uint32_t new_s1  = GPIOC->IDR& (1<<1);	//aktualni hodnota stavu vstpniho pinu S1
		if(old_s1 && !new_s1)					//falling edge
		{
			off_time1 = Tick + LED_TIME_LONG;	//nastaveni vypinaciho casu
			GPIOA->BSRR = (1<<4);				//rozsviceni LED1
		}
		old_s1 = new_s1;
	}

	if (Tick > debounce2 + BUTTON_DEBOUNCE_SHORT)
	{
		static uint16_t debounce = 0xFFFF;				//nastaveni pomocne promenne na 1111.....11111

		debounce <<= 1;									//posunuti po jednom bitu 111....110
		if(GPIOC->IDR & (1<<1)) debounce |= 0x0001;		//stisk => 1111...1110; nestist => 111...111;
		if(debounce == 0x8000)							//pokud bude stiskle tlacitko tak debounce = 100...000;
		{
			off_time2 = Tick + LED_TIME_LONG;
			GPIOB->BSRR = (1<<0);						//rozsviceni LED2
		}
	}

	if(Tick > off_time2)
	{
		GPIOB->BRR = (1<<0);					//vypnuti LED2
	}
	if(Tick > off_time1)
	{
		GPIOA->BRR = (1<<4);					//vypnuti LED1
	}
}

int main(void)
{
	 RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN; // enable
	 GPIOA->MODER |= GPIO_MODER_MODER4_0; // LED1 = PA4, output
	 GPIOB->MODER |= GPIO_MODER_MODER0_0; // LED2 = PB0, output
	 GPIOC->PUPDR |= GPIO_PUPDR_PUPDR0_0; // S2 = PC0, pullup
	 GPIOC->PUPDR |= GPIO_PUPDR_PUPDR1_0; // S1 = PC1, pullup

	 RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	 SysTick_Config(8000); // 1ms

	 SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PC; // select PC0 for EXTI0
	 EXTI->IMR |= EXTI_IMR_MR0; // mask
	 EXTI->FTSR |= EXTI_FTSR_TR0; // trigger on falling edge
	 NVIC_EnableIRQ(EXTI0_1_IRQn); // enable EXTI0_1


    /* Loop forever */
	for(;;)
	{
		blikac();
		tlacitka();
	}
}
