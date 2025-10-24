/*
 * 
 *
 *  Created on: Apr 10, 2025
 *      Author: Hql2017
 */

#ifndef USER_CAN1_H_
#define USER_CAN1_H_

#include "main.h"
#define HMI_CMD_ACK_MASK         0x80 //命令响应
#define HMI_BROADCAST_ADDR       0
#define HMI_CAN_FRAME_HEADER     0x7E //7E
#define HMI_CAN_FRAME_DELAY_TIME  100//50 //最小数据帧间隔150

typedef enum {
	HMI_NONE_CMD=0,
	HMI_CODE_STATUS_QUERY,//1
	HMI_CODE_ENERGE_PARAM,//2
	HMI_CODE_TREATMENT_WATER_LEVEL,//3治疗水
	HMI_CODE_AIR_LEVEL,//4
	HMI_CODE_LED_AUDIO_LEVEL,//5
	HMI_CODE_CTR_TEST_MODE,//6光纤激活
	HMI_CODE_PRO_HOT,//7
	HMI_CODE_PULSE_COUNT_AND_TIME,//8
	HMI_CODE_PULSE_WIDTH_US,//9
	HMI_CODE_JT_CONFIG,//0A
	HMI_CODE_LIQUID_DEPTH,//0B液位
	HMI_CODE_PHOTODIOD,//0C光电二极管
	HMI_CODE_ENERGE_CALIBRATION,//0D
	HMI_CODE_CLEAN_AND_DISINFECTION_TIME,//0E清洗消毒时间
	HMI_CODE_COOL_TEMPRATURE,//0F冷却系统温度
	HMI_CODE_WATER_MIST_PREPARE,//10水雾准备
	HMI_CODE_START_TIMER_LASER,//11启动定时出光	,倒计时
	HMI_CODE_rev2,//0x12
	HMI_CODE_rev3,//0x13
	HMI_CODE_rev4,//0x14
	HMI_CODE_rev5,//0x15
	HMI_CODE_rev6,//0x16
	HMI_CODE_rev7,//0x17
	HMI_CODE_rev8,//0x18
	HMI_CODE_rev9,//0x19
	HMI_CODE_rev0A,//0x1A
	HMI_CODE_rev0B,//0x1B
	HMI_CODE_rev0C,//0x1C
	HMI_CODE_rev0D,//0x1D
	HMI_CODE_rev0E,//0x1E
	HMI_CODE_rev0F,//0x1F
	HMI_CODE_CONFIG_PARAM_SYNCHRONOUS,//0x20先0参数同步
}hmi_cmd;

void  HMI_Parse_Data(unsigned char  *data, unsigned int  length) ;
void app_hmi_report_status(sys_genaration_status *sys_status) ;
void app_hmi_report_pulse_count(void) ;
void app_hmi_sysnc_req(void) ;
unsigned short int app_hmi_package_check(unsigned char* pBuff,unsigned short int buffLen) ;
#endif /* CAN_CAN1_H_ */

