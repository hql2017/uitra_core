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
 
 #ifndef LLS_MCP61
 #define LLS_MCP61
 #define  MCP61_REG_CALI_VALUE  	0x0006//校准指令
 #define  MCP61_REG_SINGGLE_C_VALUE  	0x0016//电容寄存器
 #define  MCP61_REG_ALART_SET_C_VALUE  	0x0022//设置报警阈值
 #define  MCP61_REG_ALART_CLEAR_C_VALUE  	0x0023//设置解除报警阈值
 uint16_t  app_mcp61_get_singgle_c_value_req(void);
 HAL_StatusTypeDef  app_mcp61_calibration(uint8_t min_max);
 float  app_mcp61_c_value(void);
 unsigned short int app_mcp61_package_check(void);
 void app_mcp61_init(void);
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
 
 void app_mcp61_lisen(uint16_t listenReg,uint16_t dataLen);
 uint16_t app_get_mcp61_lisen_status(void);
 HAL_StatusTypeDef  app_mcp61_set_alart_value_high(unsigned short int  high_value);
 HAL_StatusTypeDef  app_mcp61_set_alart_value_low(unsigned short int   low_value);
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
 
 