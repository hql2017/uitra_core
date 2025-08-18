
#include <stdio.h>
#include <stdbool.h>
#include <string.h> 

#include "user_can1.h"
#include "fdcan.h"



uint8_t Calculate_Checksum_Rx(uint8_t *data, size_t length) {
	uint8_t checksum = 0;
	for (size_t i = 0; i < length; i++) {
			checksum += data[i];		
	}
	return checksum; 
}
/***************************************************************************//**
 * @brief 报告状态数据
 * @param sys_status 状态结构体指针
 * @note  
 * @return 
*******************************************************************************/
void app_hmi_report_status(sys_genaration_status *sys_status) 
{
		uint8_t  can_tx_data[8];
		can_tx_data[0]=HMI_CAN_FRAME_HEADER;
		can_tx_data[1]=HMI_CODE_STATUS_QUERY|HMI_CMD_ACK_MASK;
		can_tx_data[2]= (uint8_t) sys_status->circle_water_box_temprature;
		can_tx_data[3]=(sys_status->treatment_water_level_status&0x0F)|(sys_status->air_level_status<<4);
		can_tx_data[4]=sys_status->laser_run_B0_pro_hot_status|sys_status->laser_run_B1_laser_out_status<<1|sys_status->laser_run_B2_gx_test_status<<2|sys_status->laser_run_B3_laser_pilot_lamp_status<<3|sys_status->laser_run_B4_laser_980_out_status<<4;
		can_tx_data[5]=sys_status->laser_param_B01_energe_status|sys_status->laser_param_B23_air_pump_pressure_status<<2|sys_status->laser_param_B456_jt_status<<4;
		can_tx_data[6]=sys_status->genaration_io_status&0xFF;
		can_tx_data[7]=(sys_status->genaration_io_status>>8)&0xFF;
		APP_CAN_SEND_DATA(can_tx_data,8,HMI_BROADCAST_ADDR);	
}
/***************************************************************************//**
* @brief 脉冲数据上报
 * @param pulseCount 脉冲数
 * @note  
 * @return 
*******************************************************************************/
void app_hmi_report_pulse_count(unsigned int pulseCount) 
{
	uint8_t  can_tx_data[8];
	can_tx_data[0]=HMI_CAN_FRAME_HEADER;
	can_tx_data[1]=HMI_CODE_PULSE_COUNT|HMI_CMD_ACK_MASK;
	can_tx_data[2]=pulseCount&0xFF;
	can_tx_data[3]=(pulseCount>>8)&0xFF;
	can_tx_data[4]=(pulseCount>>16)&0xFF;
	can_tx_data[5]=(pulseCount>>24)&0xFF;
	can_tx_data[6]=0;
	can_tx_data[7]=0;		
	APP_CAN_SEND_DATA(can_tx_data,8,HMI_BROADCAST_ADDR);	
}
/***************************************************************************//**
 * @brief 报告状态数据
 * @param sys_status 状态结构体指针
 * @note  
 * @return 
*******************************************************************************/
void app_hmi_cmd_ack(unsigned char code) 
{
	uint8_t  can_tx_data[8];
	can_tx_data[0]=HMI_CAN_FRAME_HEADER;
	can_tx_data[1]=code|HMI_CMD_ACK_MASK;		
	can_tx_data[2]=0;	
	can_tx_data[3]=0;	
	can_tx_data[4]=0;	
	can_tx_data[5]=0;	
	can_tx_data[6]=0;	
	can_tx_data[7]=0;
	APP_CAN_SEND_DATA(can_tx_data,8,HMI_BROADCAST_ADDR);
}
/***************************************************************************//**
 * @brief 屏幕数据解析
 * @param *data 数据 length长度
 * @note  
 * @return 返回ID
*******************************************************************************/
extern  void app_laser_preapare_semo(void);
void HMI_Parse_Data(unsigned char  *data, unsigned int  length) 
{		
	unsigned char can_tx_data[8]={0};
	unsigned char hmi_code=data[1];
	uint32_t timeout;
	if(hmi_code>HMI_CODE_PULSE_COUNT||hmi_code==HMI_NONE_CMD) 	 return ;
	can_tx_data[0]=HMI_CAN_FRAME_HEADER;
	can_tx_data[1]=hmi_code|HMI_CMD_ACK_MASK;
	switch(hmi_code)
	{		
		case HMI_CODE_STATUS_QUERY:
			app_hmi_report_status(&sGenSta);					
			break;
		case HMI_CODE_POWER_PARAM:
			data[2]%=2;
			laser_config_param.laserType=data[2];
			laser_config_param.laserEnerge=(data[3]|(data[4]<<8));						
			laser_config_param.laserFreq=data[5];						
			break;
		case HMI_CODE_TREATMENT_WATER_LEVEL:
			laser_config_param.treatmentWaterLevel=data[2];				
			break;
		case HMI_CODE_AIR_LEVEL:
			laser_config_param.airPressureLevel=data[2];			
			break;
		case HMI_CODE_LED_LEVEL:
			laser_config_param.ledLightLevel=data[2];	 	 		
			break;
		case HMI_CODE_CTR_TEST_MODE:
			laser_config_param.ctrTestMode=data[2];		
			//app_laser_preapare_semo();	//本设备无980	
			break;
		case HMI_CODE_PRO_HOT:				
			//laser_config_param.laserType=data[2];	
			laser_config_param.proHotCtr = data[3];
			app_laser_preapare_semo();
			break;
		case HMI_CODE_PULSE_COUNT:			
			can_tx_data[2]=sGenSta.laser1064PulseCount&0xFF;
			can_tx_data[3]=(sGenSta.laser1064PulseCount>>8)&0xFF;
			can_tx_data[4]=(sGenSta.laser1064PulseCount>>16)&0xFF;
			can_tx_data[5]=(sGenSta.laser1064PulseCount>>24)&0xFF;	
			can_tx_data[6]=0;
			can_tx_data[7]=0;
			APP_CAN_SEND_DATA(can_tx_data,8,HMI_BROADCAST_ADDR);
			break;
		default:			
			break;
	}		
	if(hmi_code>HMI_CODE_STATUS_QUERY&&hmi_code!=HMI_CODE_PULSE_COUNT)	APP_CAN_SEND_DATA(can_tx_data,8,HMI_BROADCAST_ADDR);//ACK no DATA
}
	

