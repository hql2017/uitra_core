/*
 * air_gzp6816d_bsp.h
 *
 *  Created on: Apr 10, 2025
 *      Author: Hql2017
 */

#ifndef AIR_GZP6816D_BSP_H_
#define AIR_GZP6816D_BSP_H_

void GZP6816D_init(void);
unsigned char GZP6816D_IsBusy(void); 
void GZP6816D_start_sampling(void); 
void GZP6816D_get_cal(float *preKpa,float *enTemprature); 
unsigned char  app_gzp6816d_listen(unsigned int sysTick,float *envirPresure,float *envirTemprature);
#endif /* AIR_GZP6816D_BSP_H_ */

