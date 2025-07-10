/*
 *m117z_bsp.h
 *
 *  Created on: Apr 10, 2025
 *      Author: Hql2017
 */

#ifndef M117Z_BSP_H_
#define M117Z_BSP_H_
#include "main.h"
unsigned short int M117Z_get_status(void);
void M117Z_init(void);
short int  M117Z_get_temprature(void) ; //读取传感器的温度测量值
void M117Z_start_sampling(void); //启动测量
#endif /*  M117Z_BSP_BSP_H_ */

