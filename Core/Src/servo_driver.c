
#include "stm32f4xx_hal.h"
#include "servo_driver.h"


static void servo_pin_configure();

void servo_init(void)
{
	servo_pin_configure();

	TIM2->PSC = 8-1; // Timer clock = 16 mhz / 16 = 1Mhz (1 us)
	TIM2->ARR = 20000-1;  // Period ==> 20 ms
	TIM2->CCR1 = 2000;  // Duty Cycle
	TIM2->CCR2 = 2000;  // Duty Cycle
	TIM2->CCR3 = 2000;  // Duty Cycle
	TIM2->CCR4 = 2000;  // Duty Cycle

	// CH-1 PWM MODE
	TIM2->CCMR1 |= TIM_CCMR1_OC1M_2;
	TIM2->CCMR1 |= TIM_CCMR1_OC1M_1;
	TIM2->CCMR1 |= TIM_CCMR1_OC1PE;

	// CH-2 PWM MODE
	TIM2->CCMR1 |= TIM_CCMR1_OC2M_2;
	TIM2->CCMR1 |= TIM_CCMR1_OC2M_1;
	TIM2->CCMR1 |= TIM_CCMR1_OC2PE;

	// CH-3 PWM MODE
	TIM2->CCMR2 |= TIM_CCMR2_OC3M_2;
	TIM2->CCMR2 |= TIM_CCMR2_OC3M_1;
	TIM2->CCMR2 |= TIM_CCMR2_OC3PE;

	// CH-4 PWM MODE
	TIM2->CCMR2 |= TIM_CCMR2_OC4M_2;
	TIM2->CCMR2 |= TIM_CCMR2_OC4M_1;
	TIM2->CCMR2 |= TIM_CCMR2_OC4PE;

	// Enable OC1REF and OC2REF OUTPUTS
	TIM2->CCER |= TIM_CCER_CC1E;
	TIM2->CCER |= TIM_CCER_CC2E;
	TIM2->CCER |= TIM_CCER_CC3E;
	TIM2->CCER |= TIM_CCER_CC4E;
}

void servo_enable(void)
{
	// Enable Timer
	TIM2->CR1 |= TIM_CR1_CEN;
	TIM2->EGR |= TIM_EGR_UG;
}

void servo_disable(void)
{
	// Disable Timer
	TIM2->CR1 &= ~(TIM_CR1_CEN);
}

void servo_set_duty_cycle(uint32_t duty, Servo_Channels_e channel)
{
	switch (channel) {
	case ServoCHANNEL1:
		TIM2->CCR1 = duty;
		break;

	case ServoCHANNEL2:
		TIM2->CCR2 = duty;
		break;

	case ServoCHANNEL3:
		TIM2->CCR3 = duty;
		break;

	case ServoCHANNEL4:
		TIM2->CCR4 = duty;
		break;
	}
}

void servo1 (double angel){
	servo_set_duty_cycle((2000), ServoCHANNEL1); //PA0
}
void servo2 (double angel){
	servo_set_duty_cycle(((angel/180)+1000), ServoCHANNEL2); //PA1
}
void servo3 (double angel){
	servo_set_duty_cycle(((angel/180)*1000+1000), ServoCHANNEL3); //PA2
}
void servo4 (double angel){
	servo_set_duty_cycle(((angel/180)+1000), ServoCHANNEL4); //PA3
}

static void servo_pin_configure()
{
	//DOLDURULACAK
		GPIO_InitTypeDef GPIO_InitStruct;

		__HAL_RCC_TIM2_CLK_ENABLE();
		__HAL_RCC_GPIOB_CLK_ENABLE();

		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = GPIO_PIN_0;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;

		GPIO_InitStruct.Pin = GPIO_PIN_1;
		GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		// PB0 - TIMER3 - CH3 - A3
		GPIO_InitStruct.Pin = GPIO_PIN_2;
		GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		// PB1 - TIMER3 - CH4
		GPIO_InitStruct.Pin = GPIO_PIN_3;
		GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}/*
 * servodriver.c
 *
 *  Created on: Jul 17, 2023
 *      Author: CASPER
 */


