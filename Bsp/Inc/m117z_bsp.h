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
short int  M117Z_get_temprature(void) ; //��ȡ���������¶Ȳ���ֵ
void M117Z_start_sampling(void); //��������
#endif /*  M117Z_BSP_BSP_H_ */

