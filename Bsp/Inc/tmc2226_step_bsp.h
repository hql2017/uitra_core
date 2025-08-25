/*
 *tmc2226_step_bsp.h
 *
 *  Created on: Apr 10, 2025
 *      Author: Hql2017
 */
#ifndef TMC2226_STEP_BSP_H_
#define TMC2226_STEP_BSP_H_

#define CONTINUOUS_STEPS_COUNT   0xFFFFFFFFUL //


void tmc2226_init(void);
void app_steps_pulse(int steps);
void tmc2226_start(unsigned char dir,unsigned short int spdLevel);
unsigned int tmc2226_stop(void);

 void app_tmc2226_sped_set(unsigned char spdLevel);

#endif /* TMC2226_STEP_BSP_H_ */

