/*
 *GE2117_GP_BSP_H_.h
 *
 *  Created on: Apr 10, 2025
 *      Author: Hql2017
 */

#ifndef GE2117_GP_BSP_H_
#define GE2117_GP_BSP_H_

#include <stdint.h>
#include "main.h"

#define GE2117_PACKAGE_HEAD    0xAA 
#define GE2117_PACKAGE_END     0x55  
#define GE2117_PACKAGE_LENGTH   16
#define GE2117_UART_BAUTE    9600

#define  GE2117_START_CMD         0x03//启动
#define  GE2117_STOP_CMD    		0x04//停止

///压缩机
#define  GE2117_SLAVE_ADD           0x01//默认地址 1~32
#define  GE2117_REDA_CODE     		0x03//读
#define  GE2117_SINGLE_WRITE_CODE   0x06//写单个寄存器

#define  GE2117_REG_MODBUS_ADDR  	0x1000//modbus地址1~32
#define  GE2117_REG_RE1  	        0x1001//保留寄存器1
#define  GE2117_REG_RE2  	        0x1002//保留寄存器2
#define  GE2117_REG_RE3 	        0x1003//保留寄存器3
#define  GE2117_REG_START_STOP	    0x1004//通信寄存器地址 3压缩机启动，4压缩机停止
#define  GE2117_REG_RE4	                0x1005//保留寄存器3
#define  GE2117_REG_SPD                0x1006//速度：0~7000

#define  GE2117_REG_SPD_STRAT                0x6000//速度启动指令
//0x1006~0x103F  保持默认

#define  GE2117_REG_STATUS1	        0x2000//0待机；!0工作
#define  GE2117_REG_STATUS2_SPD	        0x2001//当前速度
#define  GE2117_REG_STATUS3_REV	        0x2002//保留
#define  GE2117_REG_STATUS4_VOLTAGE	        0x2003//当前电压  0.01V
#define  GE2117_REG_STATUS5_CURRENT        0x2004//当前电流 0.01A
#define  GE2117_REG_STATUS6_ERR       0x2005//错误编码
#define  GE2117_REG_STATUS7_TEMPRATURE       0x2006//驱动器温度

void ge2117_speed_set(unsigned short int spd);
void ge2117_start_up_set(unsigned short int startFlag);
void app_ge2117_gp_ctr(float circleWaterTmprature,unsigned int sysTimeS);
void app_ge2117_receive_data_handle(void);
void ge2117_UART_Init(uint32_t BaudRate);

#endif /*  GE2117_GP_BSP_H_ */




