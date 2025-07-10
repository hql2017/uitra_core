/*
 * 
 *
 *  Created on: Apr 10, 2025
 *      Author: Hql2017
 */

#ifndef USER_CAN1_H_
#define USER_CAN1_H_

#include "main.h"
#define HMI_CMD_ACK_MASK        0x80 //命令响应
#define HMI_BROADCAST_ADDR       0
#define HMI_CAN_FRAME_HEADER  0x7E
#define HMI_CAN_FRAME_DELAY_TIME  50 //最小数据帧间隔150

typedef enum {
	HMI_NONE_CMD=0,
	HMI_CODE_STATUS_QUERY,//1
	HMI_CODE_POWER_PARAM,//2
	HMI_CODE_TREATMENT_WATER_LEVEL,//3治疗水
	HMI_CODE_AIR_LEVEL,//4
	HMI_CODE_LED_LEVEL,//5
	HMI_CODE_CTR_TEST_MODE,//6光纤激活
	HMI_CODE_PRO_HOT,//7
	HMI_CODE_PULSE_COUNT,//8
}hmi_cmd;

void  HMI_Parse_Data(unsigned char  *data, unsigned int  length) ;
void app_hmi_report_status(sys_genaration_status *sys_status) ;
void app_hmi_report_pulse_count(unsigned int pulseCount) ;
#endif /* CAN_CAN1_H_ */

