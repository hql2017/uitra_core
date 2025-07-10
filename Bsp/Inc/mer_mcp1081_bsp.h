/*
 *MER_MCP1081_BSP_.h
 *
 *  Created on: Apr 10, 2025
 *      Author: Hql2017
 */

#ifndef MER_MCP1081_BSP_H_
#define MER_MCP1081_BSP_H_

#include <stdint.h>
#include "main.h"

#define  MER_SLAVE_ADD          0x01//默认地址
#define  MER_REDA_CODE     		0x03//读
#define  MER_SINGLE_WRITE_CODE  0x06//写单个寄存器


#define  MER_REG_SMAMLING_SPD  	0x0003//采集平均次数寄存器，
#define  MER_REG_WATER_DEPTH   	0x0004//液位挡位，
#define  MER_REG_ID_NOT_USED    0x0005//未使用的地址，
#define  MER_REG_CALIBRATION  	0x0006//校准指令

//只读
#define  MER_REG_TEMPRATURE  	0x0007//温度
#define  MER_REG_C0_VALUE   	0x0008//C0电容值(1000倍，单位pF)  0x08~0x0E,C0~C6
#define  MER_REG_C_FREQ     	0x000F//参比频率（1000倍，单位mHz）
#define  MER_REG_CF0_FREQ     	0x0010//参比频率F0（1000倍，单位mHz）0x0010~0x16,CF0~CF6
#define  MER_REG_I_CHANEL_VA   	0x0017//内部参比通道count10
#define  MER_REG_I_CH0_VA      	0x0018//内部参比通道count0   0x0018~0x001E
#define  MER_REG_I_CH6_VA      	0x001E//内部参比通道count0   0x0018~0x001E

#define  MER_MAX_REG_NUM    	    MER_REG_I_CH6_VA-MER_REG_SMAMLING_SPD+1//寄存器个数
#define  MER_MAX_READ_READNUM    	28//MER_REG_I_CH6_VA-MER_REG_SMAMLING_SPD+1//最多连读28个

#ifndef LLS_MCP61
#define LLS_MCP61
#define  MCP61_REG_SINGGLE_C_VALUE  	0x0016//电容寄存器
uint16_t  app_mcp61_get_singgle_c_value_req(void);
void  app_mcp61_calibration(uint8_t min_max);
float  app_mcp61_c_value(void);

#endif

#ifdef LLM //治疗水
#define LLM
#define LLM_PACKAGE_HEAD    0x0A //'\n'
#define LLM_PACKAGE_END     0x20  //' '
#define LLM_PACKAGE_LENGTH   44
#define LLM_UART_BAUTE    38400

HAL_StatusTypeDef  app_fresh_llm_data(float *value);
void LLM_UART_Init(uint32_t BaudRate);
void  app_llm_min_calib_value_record(unsigned char min_max ,float value);
unsigned char  app_get_llm_depth(void);
#endif 

HAL_StatusTypeDef app_mer_write_req_frame(uint16_t regStart,uint16_t data);
HAL_StatusTypeDef app_mer_read_req_frame(uint16_t regStart,uint16_t regOffset);
void app_mer_lisen(uint16_t listenReg,uint16_t dataLen);
void app_mer_prin(void);
uint16_t  app_mer_get_ID(void);
uint16_t app_get_mer_lisen_status(void);
///CLM液位计
#define  CLM_SLAVE_ADD          0x01//默认地址
#define  CLM_REDA_CODE     		0x03//读
#define  CLM_SINGLE_WRITE_CODE  0x06//写单个寄存器

#define  CLM_REG_ALERT_FLAG  	0x0001//报警标志，0，正常，1报警
#define  CLM_REG_485_ADDR   	0x0002//485节点地址，
#define  CLM_REG_CALIBRATION  	0x0007//校准指令,0,默认；1空载校准
#define  CLM_REG_HAN_  	        0x0014//硬件版本号，高字节，主版本号，低字节为子版本号
#define  CLM_REG_GJ_  	        0x0016//固件版本号，高字节，主版本号，低字节为子版本号
#define  CLM_REG_IDENTIFER_  	0x0017//设备(96bit UID)唯一标识符0x0017~0x001C

#endif /*  MER_MCP1081_BSP_H */

