/*
 * buzzer.c
 *
 *  Created on: Jul 28, 2023
 *      Author: CASPER
 */

#include "stm32f4xx_hal.h"
#include "buzzer_driver.h"


static void buzzer_pin_configure();

void buzzer_init(void)
{
	buzzer_pin_configure();

	TIM5->PSC = 8-1; // Timer clock = 8 mhz /
	TIM5->ARR = 20000-1;  // Period ==> 20 ms
	TIM5->CCR1 = 2000;  // Duty Cycle  //Bu kısım servo için geçerli
	TIM5->CCR2 = 2000;  // Duty Cycle
	TIM5->CCR3 = 2000;  // Duty Cycle
	TIM5->CCR4 = 2000;  // Duty Cycle

	// CH-1 PWM MODE
	TIM5->CCMR1 |= TIM_CCMR1_OC1M_2;
	TIM5->CCMR1 |= TIM_CCMR1_OC1M_1;
	TIM5->CCMR1 |= TIM_CCMR1_OC1PE;

	// CH-2 PWM MODE
	TIM5->CCMR1 |= TIM_CCMR1_OC2M_2;
	TIM5->CCMR1 |= TIM_CCMR1_OC2M_1;
	TIM5->CCMR1 |= TIM_CCMR1_OC2PE;

	// CH-3 PWM MODE
	TIM5->CCMR2 |= TIM_CCMR2_OC3M_2;
	TIM5->CCMR2 |= TIM_CCMR2_OC3M_1;
	TIM5->CCMR2 |= TIM_CCMR2_OC3PE;

	// CH-4 PWM MODE
	TIM5->CCMR2 |= TIM_CCMR2_OC4M_2;
	TIM5->CCMR2 |= TIM_CCMR2_OC4M_1;
	TIM5->CCMR2 |= TIM_CCMR2_OC4PE;

	// Enable OC1REF and OC2REF OUTPUTS
	TIM5->CCER |= TIM_CCER_CC1E;
	TIM5->CCER |= TIM_CCER_CC2E;
	TIM5->CCER |= TIM_CCER_CC3E;
	TIM5->CCER |= TIM_CCER_CC4E;
}

void buzzer_enable(void)
{
	// Enable Timer
	TIM5->CR1 |= TIM_CR1_CEN;
	TIM5->EGR |= TIM_EGR_UG;
}

void buzzer_disable(void)
{
	// Disable Timer
	TIM5->CR1 &= ~(TIM_CR1_CEN);
}

void buzzer_set_duty_cycle(uint32_t duty, Channels_e channel)
{
	switch (channel) {
	case CHANNEL1:
		TIM5->CCR1 = duty;
		break;

	case CHANNEL2:
		TIM5->CCR2 = duty;
		break;

	case CHANNEL3:
		TIM5->CCR3 = duty;
		break;

	case CHANNEL4:
		TIM5->CCR4 = duty;
		break;
	}
}

void buzzer1 (double ses_siddeti){
	buzzer_set_duty_cycle((ses_siddeti), CHANNEL1); //PE9
}
void buzzer2 (double ses_siddeti){
	buzzer_set_duty_cycle((ses_siddeti), CHANNEL2); //PE11
}
void buzzer3 (double ses_siddeti){
	buzzer_set_duty_cycle((ses_siddeti), CHANNEL3); //PE13
}
void buzzer4 (double ses_siddeti){
	buzzer_set_duty_cycle((ses_siddeti), CHANNEL4); //P14
}

void begining_song(void){
	buzzer_set_duty_cycle(19000, CHANNEL1);
	HAL_Delay(500);
	buzzer_set_duty_cycle(0, CHANNEL1);
	HAL_Delay(250);
	buzzer_set_duty_cycle(19000, CHANNEL1);
	HAL_Delay(500);
	buzzer_set_duty_cycle(0, CHANNEL1);
	HAL_Delay(250);
	buzzer_set_duty_cycle(19000, CHANNEL1);
	HAL_Delay(500);
	buzzer_set_duty_cycle(0, CHANNEL1);
	HAL_Delay(250);
	buzzer_set_duty_cycle(5000, CHANNEL1);
	HAL_Delay(250);
	buzzer_set_duty_cycle(0, CHANNEL1);
	HAL_Delay(250);
	buzzer_set_duty_cycle(10000, CHANNEL1);
	HAL_Delay(125);
	buzzer_set_duty_cycle(0, CHANNEL1);
	HAL_Delay(125);
	buzzer_set_duty_cycle(19000, CHANNEL1);
	HAL_Delay(500);
	buzzer_set_duty_cycle(0, CHANNEL1);
	HAL_Delay(250);
	buzzer_set_duty_cycle(5000, CHANNEL1);
	HAL_Delay(250);
	buzzer_set_duty_cycle(0, CHANNEL1);
	HAL_Delay(250);
	buzzer_set_duty_cycle(10000, CHANNEL1);
	HAL_Delay(125);
	buzzer_set_duty_cycle(0, CHANNEL1);
	HAL_Delay(125);
	buzzer_set_duty_cycle(19000, CHANNEL1);
	HAL_Delay(500);
	buzzer_set_duty_cycle(0, CHANNEL1);
	HAL_Delay(250);
  	buzzer_set_duty_cycle(19000, CHANNEL1);
    HAL_Delay(500);
	buzzer_set_duty_cycle(0, CHANNEL1);
	HAL_Delay(125);
	buzzer_set_duty_cycle(10000, CHANNEL1);
	HAL_Delay(125);
	buzzer_set_duty_cycle(0, CHANNEL1);
	HAL_Delay(125);
	buzzer_set_duty_cycle(10000, CHANNEL1);
	HAL_Delay(125);
	buzzer_set_duty_cycle(0, CHANNEL1);
	HAL_Delay(125);
	buzzer_set_duty_cycle(19000, CHANNEL1);
	HAL_Delay(500);
	buzzer_set_duty_cycle(0, CHANNEL1);
	HAL_Delay(125);
	buzzer_set_duty_cycle(10000, CHANNEL1);
	HAL_Delay(125);
	buzzer_set_duty_cycle(0, CHANNEL1);
	HAL_Delay(125);
	buzzer_set_duty_cycle(10000, CHANNEL1);
	HAL_Delay(125);
	buzzer_set_duty_cycle(0, CHANNEL1);
	HAL_Delay(500);
	buzzer_set_duty_cycle(15000, CHANNEL1);
	HAL_Delay(125);
	buzzer_set_duty_cycle(0, CHANNEL1);
	HAL_Delay(125);
	buzzer_set_duty_cycle(17500, CHANNEL1);
	HAL_Delay(125);
	buzzer_set_duty_cycle(0, CHANNEL1);
	HAL_Delay(10);
	buzzer_set_duty_cycle(19000, CHANNEL1);
	HAL_Delay(500);
	buzzer_set_duty_cycle(0, CHANNEL1);
	HAL_Delay(10);
}

static void buzzer_pin_configure()
{
	//DOLDURULACAK

		GPIO_InitTypeDef GPIO_InitStruct;

		__HAL_RCC_TIM5_CLK_ENABLE();
		__HAL_RCC_GPIOE_CLK_ENABLE();

		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = GPIO_PIN_0;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;

		GPIO_InitStruct.Pin = GPIO_PIN_1;
		GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		// PB0 - TIMER3 - CH3 - A3
		GPIO_InitStruct.Pin = GPIO_PIN_2;
		GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		// PB1 - TIMER3 - CH4
		GPIO_InitStruct.Pin = GPIO_PIN_3;
		GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}



