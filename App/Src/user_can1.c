
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
	can_tx_data[4]=sys_status->laser_run_B0_pro_hot_status|sys_status->laser_run_B1_laser_out_status<<1|sys_status->laser_run_B2_gx_test_status<<2|sys_status->laser_run_B3_laser_pilot_lamp_status<<3\
	|sys_status->laser_run_B4_laser_980_out_status<<4|sys_status->laser_run_B5_timer_status<<5|sys_status->laser_run_B6_t_water_prepare_status<<6;
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
void app_hmi_report_pulse_count(void) 
{
	uint8_t  can_tx_data[8];
	can_tx_data[0]=HMI_CAN_FRAME_HEADER;
	can_tx_data[1]=HMI_CODE_PULSE_COUNT_AND_TIME|HMI_CMD_ACK_MASK;
	if(laser_ctr_param.laserType==0)//pulse 
	{
		can_tx_data[2]=u_sys_param.sys_config_param.laser_config_param.laser_pulse_count&0xFF;
		can_tx_data[3]=(u_sys_param.sys_config_param.laser_config_param.laser_pulse_count>>8)&0xFF;
		can_tx_data[4]=(u_sys_param.sys_config_param.laser_config_param.laser_pulse_count>>16)&0xFF;
		can_tx_data[5]=(u_sys_param.sys_config_param.laser_config_param.laser_pulse_count>>24)&0xFF;
	}
	else //980
	{
		can_tx_data[2]=u_sys_param.sys_config_param.laser_config_param.laser_use_timeS&0xFF;
		can_tx_data[3]=(u_sys_param.sys_config_param.laser_config_param.laser_use_timeS>>8)&0xFF;
		can_tx_data[4]=(u_sys_param.sys_config_param.laser_config_param.laser_use_timeS>>16)&0xFF;
		can_tx_data[5]=(u_sys_param.sys_config_param.laser_config_param.laser_use_timeS>>24)&0xFF;
	}
	can_tx_data[6]=(u_sys_param.sys_config_param.laser_config_param.RDB_use_timeS/360)&0xFF;//小时
	can_tx_data[7]=((u_sys_param.sys_config_param.laser_config_param.RDB_use_timeS/360)>>8)&0xFF;		
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
extern  void app_treatment_water_cleaN_ctr(unsigned char *cleaflag);
void HMI_Parse_Data(unsigned char  *data, unsigned int  length) 
{		
	unsigned char can_tx_data[8]={0};
	unsigned char hmi_code=data[1];
	unsigned short int dw;
	uint32_t timeout;
	if(hmi_code>HMI_CODE_CONFIG_PARAM_SYNCHRONOUS||hmi_code==HMI_NONE_CMD) 	 return ;
	can_tx_data[0]=HMI_CAN_FRAME_HEADER;
	can_tx_data[1]=hmi_code|HMI_CMD_ACK_MASK;
	switch(hmi_code)
	{		
		case HMI_CODE_STATUS_QUERY:
			app_hmi_report_status(&sGenSta);					
			break;
		case HMI_CODE_ENERGE_PARAM:		
			laser_ctr_param.laserType=data[2];
			laser_ctr_param.laserEnerge=(data[3]|(data[4]<<8));						
			laser_ctr_param.laserFreq=data[5];						
			break;
		case HMI_CODE_TREATMENT_WATER_LEVEL:
			laser_ctr_param.treatmentWaterLevel=data[2];				
			break;
		case HMI_CODE_AIR_LEVEL:
			laser_ctr_param.airPressureLevel=data[2];			
			break;
		case HMI_CODE_LED_AUDIO_LEVEL:
			laser_ctr_param.ledLightLevel=data[2];
			u_sys_param.sys_config_param.laser_config_param.rgb_light=data[3];	
			u_sys_param.sys_config_param.laser_config_param.beep=data[4];	 		
			break;
		case HMI_CODE_CTR_TEST_MODE:
			laser_ctr_param.ctrTestMode=data[2];		
			//app_laser_preapare_semo();	
			break;
		case HMI_CODE_PRO_HOT:				
			laser_ctr_param.laserType=data[2];	
			laser_ctr_param.proHotCtr = data[3];
			app_laser_preapare_semo();
			break;
		case HMI_CODE_PULSE_COUNT_AND_TIME:	
			app_hmi_report_pulse_count();
			break;
		case HMI_CODE_PULSE_WIDTH_US:
			u_sys_param.sys_config_param.laser_config_param.laser_pulse_width_us=data[2]|data[3]<<8;			
			memcpy(&can_tx_data[2],&data[2],6);
			break;
		case HMI_CODE_JT_CONFIG:  
			if(data[6]==0)
			{
				u_sys_param.sys_config_param.laser_config_param.jtId=0;
				u_sys_param.sys_config_param.laser_config_param.jt_status=0;				
				memset(&can_tx_data[2],0,6);
			}
			else if(data[6]==1)
			{
				u_sys_param.sys_config_param.laser_config_param.jtId=data[2]|data[3]<<8|data[4]<<16|data[5]<<24;
				u_sys_param.sys_config_param.laser_config_param.jt_status=1;
				memcpy(&can_tx_data[2],&data[2],6);
			}
			else if(data[6]==2)
			{
				u_sys_param.sys_config_param.laser_config_param.jtId = sEnvParam.JT_ID;
				u_sys_param.sys_config_param.laser_config_param.jt_status=2;
				can_tx_data[2]=u_sys_param.sys_config_param.laser_config_param.jtId&0xFF;
				can_tx_data[3]=(u_sys_param.sys_config_param.laser_config_param.jtId>>8)&0xFF;
				can_tx_data[4]=(u_sys_param.sys_config_param.laser_config_param.jtId>>16)&0xFF;
				can_tx_data[5]=(u_sys_param.sys_config_param.laser_config_param.jtId>>24)&0xFF;
				can_tx_data[6]=2;				
			}
			break;
		case HMI_CODE_LIQUID_DEPTH:
			u_sys_param.sys_config_param.laser_config_param.treatment_water_depth_high=data[2];
			u_sys_param.sys_config_param.laser_config_param.treatment_water_depth_low=data[4];
			u_sys_param.sys_config_param.laser_config_param.cool_water_depth_high=data[5];
			u_sys_param.sys_config_param.laser_config_param.cool_water_depth_low=data[7];
			memcpy(&can_tx_data[2],&data[2],6);
			can_tx_data[3]=sEnvParam.treatment_water_depth;
			can_tx_data[6]=sEnvParam.cool_water_depth;
			break;
		case HMI_CODE_PHOTODIOD:
			u_sys_param.sys_config_param.laser_config_param.photodiod_low=data[2]|data[3]<<8;
			u_sys_param.sys_config_param.laser_config_param.photodiod_mid=data[4]|data[5]<<8;
			u_sys_param.sys_config_param.laser_config_param.photodiod_high=data[6]|data[7]<<8;
			memcpy(&can_tx_data[2],&data[2],6);
			break;
		case HMI_CODE_ENERGE_CALIBRATION:			
			dw=((data[2]|data[3]<<8)-20)/5;
			dw%=36;
			u_sys_param.sys_config_param.laser_config_param.e_cali[dw].energe_dw=data[2]|data[3]<<8;
			u_sys_param.sys_config_param.laser_config_param.e_cali[dw].energe_cali=data[4]|data[5]<<8;
			u_sys_param.sys_config_param.laser_config_param.e_cali[dw].power_cali=data[6]|data[7]<<8;
			memcpy(&can_tx_data[2],&data[2],6);
			break;
		case HMI_CODE_CLEAN_AND_DISINFECTION_TIME:
			u_sys_param.sys_config_param.laser_config_param.clean_time_min=data[2];
			u_sys_param.sys_config_param.laser_config_param.dit_time_min=data[3];
			laser_ctr_param.cleanCtr=data[4];			
			memcpy(&can_tx_data[2],&data[2],6);
			app_treatment_water_cleaN_ctr(&laser_ctr_param.cleanCtr);
			break;
		case HMI_CODE_COOL_TEMPRATURE:
			u_sys_param.sys_config_param.laser_config_param.cool_temprature_low=data[2]|data[3]<<8;
			u_sys_param.sys_config_param.laser_config_param.cool_temprature_target=data[4]|data[5]<<8;
			u_sys_param.sys_config_param.laser_config_param.cool_temprature_high=data[6]|data[7]<<8;
			memcpy(&can_tx_data[2],&data[2],6);
			break;
		case HMI_CODE_WATER_MIST_PREPARE:
			laser_ctr_param.air_water_prepare_ctr = data[2];		
			can_tx_data[2]=laser_ctr_param.air_water_prepare_ctr;
			break;
		case HMI_CODE_CONFIG_PARAM_SYNCHRONOUS:
			if(data[2]==0)
			{
				u_sys_param.sys_config_param.synchronousFlag = data[3];
				if(data[3]==1) u_sys_param.sys_config_param.laser_config_param.equipmentId=data[4]|data[5]<<8|data[6]<<16|data[7]<<24;
				else if(data[3]==3)  
				{
					can_tx_data[2]=0;
					can_tx_data[3]=3;
					can_tx_data[4]= u_sys_param.sys_config_param.laser_config_param.equipmentId&0xFF;
					can_tx_data[5]=(u_sys_param.sys_config_param.laser_config_param.equipmentId>>8)&0xFF;
					can_tx_data[6]=(u_sys_param.sys_config_param.laser_config_param.equipmentId>>16)&0xFF;
					can_tx_data[7]=(u_sys_param.sys_config_param.laser_config_param.equipmentId>>24)&0xFF;
				}
			}
			else 
			{				
				memcpy(&u_sys_param.data[data[2]*5],&data[3],5);
			}
			break;			
		default:			
			break;
	}		
	if(hmi_code>HMI_CODE_STATUS_QUERY&&hmi_code!=HMI_CODE_PULSE_COUNT_AND_TIME)	APP_CAN_SEND_DATA(can_tx_data,8,HMI_BROADCAST_ADDR);//ACK no DATA
}
	

