/*
 *tmc2226_step_bsp.h
 *
 *  Created on: Apr 10, 2025
 *      Author: Hql2017
 */
#ifndef TMC2226_STEP_BSP_H_
#define TMC2226_STEP_BSP_H_

#define CONTINUOUS_STEPS_COUNT   0xFFFFFFFFUL //
#define TMC_WATER_OUT_DIR_VALUE   0//

void tmc2226_init(void);
void app_steps_pulse(unsigned int steps);
void tmc2226_start(unsigned char dir,unsigned short int spdLevel);
unsigned int tmc2226_stop(void);

 void app_tmc2226_speed_set(unsigned char spdLevel);

#endif /* TMC2226_STEP_BSP_H_ */

