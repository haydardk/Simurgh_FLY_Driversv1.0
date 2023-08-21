/*
 * buzzer_driver.h
 *
 *  Created on: Jul 28, 2023
 *      Author: CASPER
 */

#ifndef INC_BUZZER_DRIVER_H_
#define INC_BUZZER_DRIVER_H_

typedef enum CHANNELS{
	CHANNEL1 =0, // Buzzer numaraları
	CHANNEL2 =1,
	CHANNEL3 =2,
	CHANNEL4 =3,
}Channels_e;

void buzzer_init(void);

void buzzer_enable(void);
void buzzer_disable(void);

void buzzer_set_duty_cycle(uint32_t duty, Channels_e channel);

void buzzer1 (double ses_siddeti);
void buzzer2 (double ses_siddeti);
void buzzer3 (double ses_siddeti);
void buzzer4 (double ses_siddeti);

void begining_song(void);

#endif /* INC_buzzer_DRIVER_H_ */
