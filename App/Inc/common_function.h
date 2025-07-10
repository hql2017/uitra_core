/*
 * common_function.h
 *
 *  Created on: Oct 21, 2021
 *      Author: Administrator
 */

#ifndef COMMON_FUNCTION_H_
#define COMMON_FUNCTION_H_

#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include "stdio.h"
#include "main.h"
// 2字节大小端
#define SWAP2BYTES(num) do{\
	u16 temp = num;\
	num = (((temp) >> 8) | (((temp) & 0xFF) << 8));\
	}while(0);
	
	// 4字节大小端
	#define SWAP4BYTES(num) do{\
	u32 temp = num;\
	num = ((((temp) >> 24) & 0xFF) | (((temp) >> 8) & 0xFF00) | (((temp) << 8) & 0xFF0000) | (((temp) << 24) & 0xFF000000));\
	}while(0);
	
	
extern  unsigned short int  crc16Num(unsigned char *pData,unsigned  int length);
extern void DWT_Init(void);
extern void delay_us(volatile uint32_t nus);
unsigned int sumCheck(unsigned char *pData,unsigned  int length);

HAL_StatusTypeDef compare_buff_no_change(unsigned char *buff1,unsigned char *buff2,unsigned int length);
#endif /* COMMON_FUNCTION */

