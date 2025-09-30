
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
 * @brief 同步请求
 * @param sys_status 状态结构体指针
 * @note  
 * @return 
*******************************************************************************/
void app_hmi_sysnc_req(void) 
{
	uint8_t  can_tx_data[16];
	can_tx_data[0]=0x7E;
	can_tx_data[1]=0x7E;	
	can_tx_data[2]=13;
	can_tx_data[3]=HMI_CODE_CONFIG_PARAM_SYNCHRONOUS|HMI_CMD_ACK_MASK;
	can_tx_data[4]= u_sys_param.sys_config_param.laser_config_param.synchronousFlag;
	can_tx_data[5]= u_sys_param.sys_config_param.laser_config_param.equipmentId&0xFF;
	can_tx_data[6]=(u_sys_param.sys_config_param.laser_config_param.equipmentId>>8)&0xFF;
	can_tx_data[7]=(u_sys_param.sys_config_param.laser_config_param.equipmentId>>16)&0xFF;
	can_tx_data[8]=(u_sys_param.sys_config_param.laser_config_param.equipmentId>>24)&0xFF;	
	can_tx_data[10]=(crc16Num(can_tx_data,9)>>8)&0xFF;
	can_tx_data[9]=crc16Num(can_tx_data,9)&0xFF;
	can_tx_data[11]=0x0D;
	can_tx_data[12]=0x0A;		
	APP_CAN_SEND_DATA(can_tx_data,13,HMI_BROADCAST_ADDR);
}
/***************************************************************************//**
 * @brief 报告状态数据
 * @param sys_status 状态结构体指针
 * @note  
 * @return 
*******************************************************************************/
void app_hmi_report_status(sys_genaration_status *sys_status) 
{
	uint8_t  can_tx_data[26];
	can_tx_data[0]=0x7E;
	can_tx_data[1]=0x7E;	
	can_tx_data[2]=26;
	can_tx_data[3]=HMI_CODE_STATUS_QUERY|HMI_CMD_ACK_MASK;
	can_tx_data[4]=25;// (uint8_t) sys_status->circle_water_box_temprature;
	can_tx_data[5]=(sys_status->treatment_water_level_status&0x0F)|(sys_status->air_level_status<<4);
	can_tx_data[6]=sys_status->laser_run_B0_pro_hot_status|sys_status->laser_run_B1_laser_out_status<<1|sys_status->laser_run_B2_gx_test_status<<2|sys_status->laser_run_B3_laser_pilot_lamp_status<<3\
	|sys_status->laser_run_B4_laser_980_out_status<<4|sys_status->laser_run_B5_timer_status<<5;
	can_tx_data[7]=sys_status->laser_param_B01_energe_status|sys_status->laser_param_B23_air_pump_pressure_status<<2|sys_status->laser_param_B456_jt_status<<4;
	can_tx_data[8]=sys_status->genaration_io_status&0xFF;
	can_tx_data[9]=(sys_status->genaration_io_status>>8)&0xFF;
	can_tx_data[10]=u_sys_param.sys_config_param.laser_config_param.laser_pulse_count&0xFF;
	can_tx_data[11]=(u_sys_param.sys_config_param.laser_config_param.laser_pulse_count>>8)&0xFF;
	can_tx_data[12]=(u_sys_param.sys_config_param.laser_config_param.laser_pulse_count>>16)&0xFF;
	can_tx_data[13]=(u_sys_param.sys_config_param.laser_config_param.laser_pulse_count>>24)&0xFF;
	can_tx_data[14]=u_sys_param.sys_config_param.laser_config_param.laser_use_timeS&0xFF;
	can_tx_data[15]=(u_sys_param.sys_config_param.laser_config_param.laser_use_timeS>>8)&0xFF;
	can_tx_data[16]=(u_sys_param.sys_config_param.laser_config_param.laser_use_timeS>>16)&0xFF;
	can_tx_data[17]=(u_sys_param.sys_config_param.laser_config_param.laser_use_timeS>>24)&0xFF;		
	can_tx_data[18]=(u_sys_param.sys_config_param.laser_config_param.RDB_use_timeS)&0xFF;//s
	can_tx_data[19]=(u_sys_param.sys_config_param.laser_config_param.RDB_use_timeS>>8)&0xFF;	
	can_tx_data[20]=(u_sys_param.sys_config_param.laser_config_param.RDB_use_timeS>>16)&0xFF;
	can_tx_data[21]=(u_sys_param.sys_config_param.laser_config_param.RDB_use_timeS>>24)&0xFF;	
	can_tx_data[23]=(crc16Num(can_tx_data,22)>>8)&0xFF;
	can_tx_data[22]=crc16Num(can_tx_data,22)&0xFF;
	can_tx_data[24]=0x0D;
	can_tx_data[25]=0x0A;
	APP_CAN_SEND_DATA(can_tx_data,26,HMI_BROADCAST_ADDR);//304 bytes(38package) use 75ms  t=75/38;2ms

}
/***************************************************************************//**
* @brief 脉冲数据上报
 * @param pulseCount 脉冲数
 * @note  
 * @return 
*******************************************************************************/
void app_hmi_report_pulse_count(void) 
{
	uint8_t  can_tx_data[20];
	can_tx_data[0]=0x7E;
	can_tx_data[1]=0x7E;
	can_tx_data[2]=20;
	can_tx_data[3]=HMI_CODE_PULSE_COUNT_AND_TIME|HMI_CMD_ACK_MASK;	
	can_tx_data[4]=u_sys_param.sys_config_param.laser_config_param.laser_pulse_count&0xFF;
	can_tx_data[5]=(u_sys_param.sys_config_param.laser_config_param.laser_pulse_count>>8)&0xFF;
	can_tx_data[6]=(u_sys_param.sys_config_param.laser_config_param.laser_pulse_count>>16)&0xFF;
	can_tx_data[7]=(u_sys_param.sys_config_param.laser_config_param.laser_pulse_count>>24)&0xFF;
	can_tx_data[8]=u_sys_param.sys_config_param.laser_config_param.laser_use_timeS&0xFF;
	can_tx_data[9]=(u_sys_param.sys_config_param.laser_config_param.laser_use_timeS>>8)&0xFF;
	can_tx_data[10]=(u_sys_param.sys_config_param.laser_config_param.laser_use_timeS>>16)&0xFF;
	can_tx_data[11]=(u_sys_param.sys_config_param.laser_config_param.laser_use_timeS>>24)&0xFF;		
	can_tx_data[12]=(u_sys_param.sys_config_param.laser_config_param.RDB_use_timeS)&0xFF;//s
	can_tx_data[13]=(u_sys_param.sys_config_param.laser_config_param.RDB_use_timeS>>8)&0xFF;	
	can_tx_data[14]=(u_sys_param.sys_config_param.laser_config_param.RDB_use_timeS>>16)&0xFF;
	can_tx_data[15]=(u_sys_param.sys_config_param.laser_config_param.RDB_use_timeS>>24)&0xFF;
	can_tx_data[17]=(crc16Num(can_tx_data,16)>>8)&0xFF;
	can_tx_data[16]=crc16Num(can_tx_data,16)&0xFF;
	can_tx_data[18]	=0x0D;	
	can_tx_data[19]	=0x0A;	
	APP_CAN_SEND_DATA(can_tx_data,20,HMI_BROADCAST_ADDR);//304 bytes(38package) use 75ms  t=75/38;2ms		
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
	can_tx_data[0]=0x7E;
	can_tx_data[1]=0x7E;
	can_tx_data[2]=8;
	can_tx_data[3]=code|HMI_CMD_ACK_MASK;	
	can_tx_data[5]=(crc16Num(can_tx_data,4)>>8)&0xFF;
	can_tx_data[4]=crc16Num(can_tx_data,4)&0xFF;
	can_tx_data[6]=0x0D;
	can_tx_data[7]=0x0A;
	APP_CAN_SEND_DATA(can_tx_data,8,HMI_BROADCAST_ADDR);
}
/***************************************************************************//**
 * @brief 屏幕数据解析
 * @param *data 数据 length长度
 * @note  
 * @return 返回ID
*******************************************************************************/
extern  void app_laser_preapare_semo(void);
extern  void app_treatment_water_clean_ctr(unsigned char *cleaflag);
void HMI_Parse_Data(unsigned char  *data, unsigned int  length) 
{		
	unsigned char can_tx_data[256]={0};
	unsigned char hmi_code=data[3];
	unsigned short int dw;
	uint32_t timeout;
	if(hmi_code>HMI_CODE_CONFIG_PARAM_SYNCHRONOUS||hmi_code==HMI_NONE_CMD) return ;
	can_tx_data[0]=0x7E;
	can_tx_data[1]=0x7E;
	can_tx_data[3]=hmi_code|HMI_CMD_ACK_MASK;
	switch(hmi_code)
	{		
		case HMI_CODE_STATUS_QUERY:
			app_hmi_report_status(&sGenSta);					
			break;
		case HMI_CODE_ENERGE_PARAM:		
			laser_ctr_param.laserType=data[4];			
			laser_ctr_param.laserEnerge=(data[5]|(data[6]<<8));						
			laser_ctr_param.laserFreq=data[7];//只取低位		
			app_hmi_cmd_ack(hmi_code) ;				
			break;
		case HMI_CODE_TREATMENT_WATER_LEVEL:
			laser_ctr_param.treatmentWaterLevel=data[4];
			sGenSta.treatment_water_level_status=data[4];
			app_hmi_cmd_ack(hmi_code) ;					
			break;
		case HMI_CODE_AIR_LEVEL:
			laser_ctr_param.airPressureLevel=data[4];
			sGenSta.air_level_status=data[4];
			app_hmi_cmd_ack(hmi_code) ;			
			break;
		case HMI_CODE_LED_AUDIO_LEVEL:
			laser_ctr_param.ledLightLevel=data[4];
			u_sys_param.sys_config_param.laser_config_param.rgb_light=data[5];	
			u_sys_param.sys_config_param.laser_config_param.beep=data[6];
			app_hmi_cmd_ack(hmi_code) ;	 		
			break;			
		case HMI_CODE_CTR_TEST_MODE:
			laser_ctr_param.ctrTestMode=data[4];		
			app_laser_preapare_semo();	
			app_hmi_cmd_ack(hmi_code) ;	
			break;
		case HMI_CODE_PRO_HOT:
			laser_ctr_param.laserType=data[4];	
			laser_ctr_param.proHotCtr = data[5];
			laser_ctr_param.proCali=data[6];
			app_laser_preapare_semo();		
			break;
		case HMI_CODE_PULSE_COUNT_AND_TIME:	
			app_hmi_report_pulse_count();
			break;
		case HMI_CODE_PULSE_WIDTH_US:
			u_sys_param.sys_config_param.laser_config_param.laser_pulse_width_us=data[4]|data[5]<<8;				
			can_tx_data[2]=8;	
			can_tx_data[4]=(crc16Num(can_tx_data,4)>>8)&0xFF;
			can_tx_data[5]=crc16Num(can_tx_data,4)&0xFF;
			can_tx_data[6]=0x0A;
			can_tx_data[7]=0x0D;
			APP_CAN_SEND_DATA(can_tx_data,8,HMI_BROADCAST_ADDR);
			break;
		case HMI_CODE_JT_CONFIG: 			 
			if(data[9]==0)
			{
				u_sys_param.sys_config_param.laser_config_param.jtId=0;
				u_sys_param.sys_config_param.laser_config_param.jt_status=0;		
			}
			else if(data[9]==1)
			{
				u_sys_param.sys_config_param.laser_config_param.jtId=data[4]|data[5]<<8|data[6]<<16|data[7]<<24;
				u_sys_param.sys_config_param.laser_config_param.jt_status=1;			
			}
			else if(data[9]==2)//search
			{
				u_sys_param.sys_config_param.laser_config_param.jtId = sEnvParam.JT_ID;
				u_sys_param.sys_config_param.laser_config_param.jt_status=2;							
			}			
			can_tx_data[2]=13;//len			
			can_tx_data[4]=u_sys_param.sys_config_param.laser_config_param.jtId&0xFF;
			can_tx_data[5]=(u_sys_param.sys_config_param.laser_config_param.jtId>>8)&0xFF;
			can_tx_data[6]=(u_sys_param.sys_config_param.laser_config_param.jtId>>16)&0xFF;
			can_tx_data[7]=(u_sys_param.sys_config_param.laser_config_param.jtId>>24)&0xFF;
			can_tx_data[8]=u_sys_param.sys_config_param.laser_config_param.jt_status;
			can_tx_data[9]=(crc16Num(can_tx_data,9)>>8)&0xFF;
			can_tx_data[10]=crc16Num(can_tx_data,9)&0xFF;	
			can_tx_data[11]=0x0A;
			can_tx_data[12]=0x0D;			
			APP_CAN_SEND_DATA(can_tx_data,13,HMI_BROADCAST_ADDR);
			break;
		case HMI_CODE_LIQUID_DEPTH:
			u_sys_param.sys_config_param.laser_config_param.treatment_water_depth_high=data[4];
			u_sys_param.sys_config_param.laser_config_param.treatment_water_depth_low=data[7];
			u_sys_param.sys_config_param.laser_config_param.cool_water_depth_high=data[8];
			u_sys_param.sys_config_param.laser_config_param.cool_water_depth_low=data[9];		
			can_tx_data[2]=14;			
			can_tx_data[4]=u_sys_param.sys_config_param.laser_config_param.treatment_water_depth_high;
			can_tx_data[5]=(uint8_t)(sEnvParam.treatment_water_depth*10);
			can_tx_data[6]=u_sys_param.sys_config_param.laser_config_param.treatment_water_depth_low;		
			can_tx_data[7]=u_sys_param.sys_config_param.laser_config_param.cool_water_depth_high;
			can_tx_data[8]=(uint8_t)(sEnvParam.cool_water_depth*10);
			can_tx_data[9]=	u_sys_param.sys_config_param.laser_config_param.cool_water_depth_low;
			can_tx_data[10]=(crc16Num(can_tx_data,10)>>8)&0xFF;
			can_tx_data[11]=crc16Num(can_tx_data,10)&0xFF;
			can_tx_data[12]=0x0A;
			can_tx_data[13]=0x0D;
			APP_CAN_SEND_DATA(can_tx_data,14,HMI_BROADCAST_ADDR);
			break;
		case HMI_CODE_PHOTODIOD:
			u_sys_param.sys_config_param.laser_config_param.photodiod_low=data[4]|data[5]<<8;
			u_sys_param.sys_config_param.laser_config_param.photodiod_mid=data[6]|data[7]<<8;
			u_sys_param.sys_config_param.laser_config_param.photodiod_high=data[8]|data[9]<<8;
			can_tx_data[2]=14;
			memcpy(&can_tx_data[4],&data[4],6);			
			can_tx_data[10]=(crc16Num(can_tx_data,10)>>8)&0xFF;
			can_tx_data[11]=crc16Num(can_tx_data,10)&0xFF;
			can_tx_data[12]=0x0A;
			can_tx_data[13]=0x0D;
			APP_CAN_SEND_DATA(can_tx_data,14,HMI_BROADCAST_ADDR);
			break;
		case HMI_CODE_ENERGE_CALIBRATION:			
			dw=(data[4]|data[5]<<8)/5;
			dw%=40;			
			u_sys_param.sys_config_param.laser_config_param.e_cali[dw].energe_cali=data[6]|data[7]<<8;
			u_sys_param.sys_config_param.laser_config_param.e_cali[dw].power_cali=data[8]|data[9]<<8;
			can_tx_data[2]=14;
			memcpy(&can_tx_data[4],&data[4],6);			
			can_tx_data[10]=(crc16Num(can_tx_data,10)>>8)&0xFF;
			can_tx_data[11]=crc16Num(can_tx_data,10)&0xFF;
			can_tx_data[12]=0x0A;
			can_tx_data[13]=0x0D;
			APP_CAN_SEND_DATA(can_tx_data,14,HMI_BROADCAST_ADDR);
			break;
		case HMI_CODE_CLEAN_AND_DISINFECTION_TIME:
			u_sys_param.sys_config_param.laser_config_param.clean_time_min=data[4];
			u_sys_param.sys_config_param.laser_config_param.dit_time_min=data[5];
			laser_ctr_param.cleanCtr=data[6];	
			can_tx_data[2]=9;
			memcpy(&can_tx_data[4],&data[4],3);
			can_tx_data[7]=(crc16Num(can_tx_data,7)>>8)&0xFF;
			can_tx_data[8]=crc16Num(can_tx_data,7)&0xFF;
			can_tx_data[9]=0x0A;
			can_tx_data[10]=0x0D;
			APP_CAN_SEND_DATA(can_tx_data,11,HMI_BROADCAST_ADDR);
			app_treatment_water_clean_ctr(&laser_ctr_param.cleanCtr);
			break;
		case HMI_CODE_COOL_TEMPRATURE:
			u_sys_param.sys_config_param.laser_config_param.cool_temprature_low=data[4]|data[5]<<8;
			u_sys_param.sys_config_param.laser_config_param.cool_temprature_target=data[6]|data[7]<<8;
			u_sys_param.sys_config_param.laser_config_param.cool_temprature_high=data[8]|data[9]<<8;
			can_tx_data[2]=14;
			memcpy(&can_tx_data[4],&data[4],6);			
			can_tx_data[10]=(crc16Num(can_tx_data,10)>>8)&0xFF;
			can_tx_data[11]=crc16Num(can_tx_data,10)&0xFF;
			can_tx_data[12]=0x0A;
			can_tx_data[13]=0x0D;
			APP_CAN_SEND_DATA(can_tx_data,14,HMI_BROADCAST_ADDR);
			break;
		case HMI_CODE_WATER_MIST_PREPARE:
			if(data[4]!=0)	laser_ctr_param.air_water_prepare_ctr = data[4];		
			can_tx_data[2]=9;
			can_tx_data[4]=data[4];			
			can_tx_data[5]=(crc16Num(can_tx_data,5)>>8)&0xFF;
			can_tx_data[6]=crc16Num(can_tx_data,5)&0xFF;
			can_tx_data[7]=0x0A;
			can_tx_data[8]=0x0D;
			APP_CAN_SEND_DATA(can_tx_data,9,HMI_BROADCAST_ADDR);
			break;
		case HMI_CODE_CONFIG_PARAM_SYNCHRONOUS:
			if(data[4]==1)//load param			
			{
				memcpy(&u_sys_param.data[data[4]*5],&data[4],length-8);	
				u_sys_param.sys_config_param.laser_config_param.synchronousFlag=3;		
				can_tx_data[2]=9;
				can_tx_data[4]=3;//syncFlag				
				can_tx_data[5]=(crc16Num(can_tx_data,5)>>8)&0xFF;
				can_tx_data[6]=crc16Num(can_tx_data,5)&0xFF;
				can_tx_data[7]=0x0A;
				can_tx_data[8]=0x0D;
				APP_CAN_SEND_DATA(can_tx_data,9,HMI_BROADCAST_ADDR);//ack			
			}
			else if(data[6]==2)//load param				 
			{				
				can_tx_data[2]=235;
				//can_tx_data[4]=3;//syncFlag
				u_sys_param.sys_config_param.laser_config_param.synchronousFlag=3;
				//...	
				memcpy(&can_tx_data[4],&u_sys_param.data[12],227);				
				can_tx_data[231]=crc16Num(can_tx_data,231>>8)&0xFF;
				can_tx_data[232]=crc16Num(can_tx_data,231)&0xFF;
				can_tx_data[233]=0x0A;
				can_tx_data[234]=0x0D;
				APP_CAN_SEND_DATA(can_tx_data,235,HMI_BROADCAST_ADDR);	//ack				
				//APP_CAN_SEND_DATA(can_tx_data,8,HMI_BROADCAST_ADDR);	//ack			
				//APP_CAN_SEND_DATA(&u_sys_param.data[12],245,HMI_BROADCAST_ADDR);	//data
			}
			
			break;			
		default:
			break;
	}	
}
	

