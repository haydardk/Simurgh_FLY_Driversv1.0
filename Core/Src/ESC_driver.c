/*
 * ESC_driver.c
 *
 *  Created on: Aug 3, 2023
 *      Author: CASPER
 */

#include "ESC_driver.h"
#include "stm32f4xx.h"

extern uint32_t ratio ;
// Duty 1000-2000

static void ESC_PIN_Conf();

void ESC_init(void){
	ESC_PIN_Conf();

	TIM9->PSC = 8 - 1 ; // Önbölücü ayarı 8MHZ'lik kristale göre
	TIM9->ARR = 20000 ; // Periyot ayarı
	TIM9->CCR1 = 2000; // Duty


	TIM9->CCMR1 |= TIM_CCMR1_OC1M_2 ;
	TIM9->CCMR1 |= TIM_CCMR1_OC1M_1 ;
	TIM9->CCMR1 |= TIM_CCMR1_OC1PE ;

	TIM9->CCER |= TIM_CCER_CC1E;

	TIM9->CCMR2 |= TIM_CCMR1_OC2M_2;
	TIM9->CCMR2 |= TIM_CCMR1_OC2M_1;
	TIM9->CCMR2 |= TIM_CCMR1_OC2PE;

	TIM9->CCER |= TIM_CCER_CC2E;
}

void ESC_enable(void)
{
	TIM9->CR1 |= TIM_CR1_CEN;
	TIM9->EGR |= TIM_EGR_UG;
}

void ESC_disable(void)
{
	TIM9->CR1 &= ~(TIM_CR1_CEN);
}

void ESC_set_duty_cycle(uint32_t duty, int channel)
{
	if (channel==0){
		TIM9->CCR1 = duty;
	}
	if (channel==1){
		TIM9->CCR2 = duty;
	}
}

void ESC1_Run(uint32_t ratio){  // Yüzelik bir değer gir exp : 20

	ratio = ratio*10 + 1000 ;
	//ESC_set_duty_cycle(ratio, 0);
	TIM9->CCR1 = ratio;
	TIM9->CCR2 = ratio;
}

void ESC_Calibrate(void){
	int i = 0;
	for (i=0;i<100;i++){
		TIM9->CCR1 = i*10 + 1000;
		TIM9->CCR2 = i*10 + 1000;
	}
}

static void ESC_PIN_Conf()
{
	//DOLDURULACAK
		GPIO_InitTypeDef GPIO_InitStruct;

		__HAL_RCC_TIM2_CLK_ENABLE();
		__HAL_RCC_GPIOB_CLK_ENABLE();

		HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = GPIO_PIN_5;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF3_TIM9;

		HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
		GPIO_InitStruct.Pin = GPIO_PIN_6;
		GPIO_InitStruct.Alternate = GPIO_AF3_TIM9;

}

