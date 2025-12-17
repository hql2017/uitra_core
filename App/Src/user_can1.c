
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
	can_tx_data[4]= 0;//u_sys_param.sys_config_param.synchronousFlag;
	can_tx_data[5]= u_sys_param.sys_config_param.equipmentId&0xFF;
	can_tx_data[6]=(u_sys_param.sys_config_param.equipmentId>>8)&0xFF;
	can_tx_data[7]=(u_sys_param.sys_config_param.equipmentId>>16)&0xFF;
	can_tx_data[8]=(u_sys_param.sys_config_param.equipmentId>>24)&0xFF;	
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
	uint8_t  can_tx_data[32];
	uint8_t send_buff_jt;
	can_tx_data[0]=0x7E;
	can_tx_data[1]=0x7E;	
	can_tx_data[2]=26;
	can_tx_data[3]=HMI_CODE_STATUS_QUERY|HMI_CMD_ACK_MASK;
	can_tx_data[4]=(uint8_t) sys_status->circle_water_box_temprature;
	can_tx_data[5]=(sys_status->treatment_water_level_status&0x0F)|(sys_status->air_level_status<<4);
	if(laser_ctr_param.laserType!=0)
	{
		sys_status->laser_run_B4_laser_980_out_status= sys_status->laser_run_B1_laser_out_status;
	}
	can_tx_data[6]=sys_status->laser_run_B0_pro_hot_status|sys_status->laser_run_B1_laser_out_status<<1|sys_status->laser_run_B2_gx_test_status<<2|sys_status->laser_run_B3_laser_pilot_lamp_status<<3\
	|sys_status->laser_run_B4_laser_980_out_status<<4|sys_status->laser_run_B5_timer_status<<5|sys_status->laser_run_B6_close_device_status<<6;	
	if(sys_status->laser_param_B456_jt_status==IO_KEY_IDLE)
	{
		if(sEnvParam.JT_bat<30)	sys_status->laser_param_B456_jt_status=KEY_LOW_POWER;	
		else send_buff_jt=1;
	}
	else send_buff_jt=(sys_status->laser_param_B456_jt_status%5);	
	can_tx_data[7]=sys_status->laser_param_B01_energe_status|sys_status->laser_param_B23_air_pump_pressure_status<<2|send_buff_jt<<4;//sys_status->laser_param_B456_jt_status<<4;
	can_tx_data[8]=sys_status->genaration_io_status&0xFF;
	can_tx_data[9]=(sys_status->genaration_io_status>>8)&0xFF;
	can_tx_data[10]=u_sys_param.sys_config_param.laser_pulse_count&0xFF;
	can_tx_data[11]=(u_sys_param.sys_config_param.laser_pulse_count>>8)&0xFF;
	can_tx_data[12]=(u_sys_param.sys_config_param.laser_pulse_count>>16)&0xFF;
	can_tx_data[13]=(u_sys_param.sys_config_param.laser_pulse_count>>24)&0xFF;	
	can_tx_data[14]=u_sys_param.sys_config_param.laser_use_timeS&0xFF;
	can_tx_data[15]=(u_sys_param.sys_config_param.laser_use_timeS>>8)&0xFF;
	can_tx_data[16]=(u_sys_param.sys_config_param.laser_use_timeS>>16)&0xFF;
	can_tx_data[17]=(u_sys_param.sys_config_param.laser_use_timeS>>24)&0xFF;
	can_tx_data[18]=(u_sys_param.sys_config_param.RDB_use_timeS)&0xFF;//s
	can_tx_data[19]=(u_sys_param.sys_config_param.RDB_use_timeS>>8)&0xFF;	
	can_tx_data[20]=(u_sys_param.sys_config_param.RDB_use_timeS>>16)&0xFF;
	can_tx_data[21]=(u_sys_param.sys_config_param.RDB_use_timeS>>24)&0xFF;	
	can_tx_data[23]=(crc16Num(can_tx_data,22)>>8)&0xFF;
	can_tx_data[22]=crc16Num(can_tx_data,22)&0xFF;
	can_tx_data[24]=0x0D;
	can_tx_data[25]=0x0A;
	can_tx_data[26]	=0;
	can_tx_data[27]	=0;
	can_tx_data[28]	=0;
	can_tx_data[29]	=0;
	can_tx_data[30]	=0;
	can_tx_data[31]	=0;
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
	uint8_t  can_tx_data[24];	
	can_tx_data[0]=0x7E;
	can_tx_data[1]=0x7E;
	can_tx_data[2]=20;
	can_tx_data[3]=HMI_CODE_PULSE_COUNT_AND_TIME|HMI_CMD_ACK_MASK;	
	can_tx_data[4]=u_sys_param.sys_config_param.laser_pulse_count&0xFF;
	can_tx_data[5]=(u_sys_param.sys_config_param.laser_pulse_count>>8)&0xFF;
	can_tx_data[6]=(u_sys_param.sys_config_param.laser_pulse_count>>16)&0xFF;
	can_tx_data[7]=(u_sys_param.sys_config_param.laser_pulse_count>>24)&0xFF;	
	can_tx_data[8]=u_sys_param.sys_config_param.laser_use_timeS&0xFF;
	can_tx_data[9]=(u_sys_param.sys_config_param.laser_use_timeS>>8)&0xFF;
	can_tx_data[10]=(u_sys_param.sys_config_param.laser_use_timeS>>16)&0xFF;
	can_tx_data[11]=(u_sys_param.sys_config_param.laser_use_timeS>>24)&0xFF;		
	can_tx_data[12]=(u_sys_param.sys_config_param.RDB_use_timeS)&0xFF;//s
	can_tx_data[13]=(u_sys_param.sys_config_param.RDB_use_timeS>>8)&0xFF;	
	can_tx_data[14]=(u_sys_param.sys_config_param.RDB_use_timeS>>16)&0xFF;
	can_tx_data[15]=(u_sys_param.sys_config_param.RDB_use_timeS>>24)&0xFF;
	can_tx_data[17]=(crc16Num(can_tx_data,16)>>8)&0xFF;
	can_tx_data[16]=crc16Num(can_tx_data,16)&0xFF;
	can_tx_data[18]	=0x0D;	
	can_tx_data[19]	=0x0A;	
	can_tx_data[20]	=0;
	can_tx_data[21]	=0;
	can_tx_data[22]	=0;
	can_tx_data[23]	=0;
	APP_CAN_SEND_DATA(can_tx_data,20,HMI_BROADCAST_ADDR);//304 bytes(38package) use 75ms  t=75/38;2ms		
}
/***************************************************************************//**
 * @brief ack
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
extern void app_t_clean_run_timer(unsigned char *runflag);
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
			laser_ctr_param.laserEnerge=(data[4]|(data[5]<<8));						
			laser_ctr_param.laserFreq=data[6];	//(data[6]|(data[7]<<8));
			laser_ctr_param.laserType=data[8];							
			app_hmi_cmd_ack(hmi_code) ;				
			break;
		case HMI_CODE_TREATMENT_WATER_LEVEL:
			laser_ctr_param.treatmentWaterLevel=data[4];			
			sGenSta.treatment_water_level_status=data[4];
			u_sys_param.sys_config_param.t_water_low=data[5];
			u_sys_param.sys_config_param.t_water_mid=data[6];
			u_sys_param.sys_config_param.t_water_high=data[7];
			app_hmi_cmd_ack(hmi_code) ;					
			break;
		case HMI_CODE_AIR_LEVEL:
			laser_ctr_param.airPressureLevel=data[4];
			sGenSta.air_level_status=data[4];
			u_sys_param.sys_config_param.air_low_pressure=data[5];
			u_sys_param.sys_config_param.air_mid_pressure=data[6];
			u_sys_param.sys_config_param.air_high_pressure=data[7];
			app_hmi_cmd_ack(hmi_code) ;			
			break;
		case HMI_CODE_LED_AUDIO_LEVEL:
			laser_ctr_param.ledLightLevel=data[4];
			u_sys_param.sys_config_param.rgb_light=data[5];	
			u_sys_param.sys_config_param.beep=50;//data[6];//固定音量
			laser_ctr_param.beep=data[6];
			app_hmi_cmd_ack(hmi_code);				 		
			break;			
		case HMI_CODE_CTR_TEST_MODE:
			laser_ctr_param.ctrTestMode=data[4];
			sGenSta.laser_run_B2_gx_test_status=0;//1;		
			app_laser_preapare_semo();	
			app_hmi_cmd_ack(hmi_code) ;	
			break;
		case HMI_CODE_PRO_HOT:
		laser_ctr_param.proHotCtr = data[4];
			laser_ctr_param.laserType=data[5];
			laser_ctr_param.proCali=data[6];			
			app_laser_preapare_semo();	
			app_hmi_cmd_ack(hmi_code) ;	
			break;
		case HMI_CODE_PULSE_COUNT_AND_TIME:	
			if(data[4]!=0)	
			{	
				u_sys_param.sys_config_param.laser_pulse_count=0;				
			}
			if(data[5]!=0)	
			{
				u_sys_param.sys_config_param.laser_use_timeS=0;				
			}
			if(data[6]!=0)	
			{	
				u_sys_param.sys_config_param.RDB_use_timeS=0;				
			}
			app_hmi_report_pulse_count();
			break;
		case HMI_CODE_PULSE_WIDTH_US:
			u_sys_param.sys_config_param.charge_width_us=data[6]|(data[7]<<8);			
			//u_sys_param.sys_config_param.charge_width_us=data[6];		
			u_sys_param.sys_config_param.laser_pulse_width_us=data[4]|(data[5]<<8);
			app_hmi_cmd_ack(hmi_code) ;
			break;
		case HMI_CODE_JT_CONFIG: 			 
			if(data[8]==0)
			{
				u_sys_param.sys_config_param.jtId=0;
				u_sys_param.sys_config_param.jt_status=0;		
			}
			else if(data[8]==1)
			{
				u_sys_param.sys_config_param.jtId=data[4]|data[5]<<8|data[6]<<16|data[7]<<24;
				u_sys_param.sys_config_param.jt_status=1;			
			}
			else if(data[8]==2)//search
			{
				u_sys_param.sys_config_param.jtId = sEnvParam.JT_ID;
				DEBUG_PRINTF("JT ID%d",sEnvParam.JT_ID);											
			}			
			can_tx_data[2]=13;//len					
			can_tx_data[4]=u_sys_param.sys_config_param.jtId&0xFF;
			can_tx_data[5]=(u_sys_param.sys_config_param.jtId>>8)&0xFF;
			can_tx_data[6]=(u_sys_param.sys_config_param.jtId>>16)&0xFF;
			can_tx_data[7]=(u_sys_param.sys_config_param.jtId>>24)&0xFF;
			can_tx_data[8]=data[8];//cmd
			can_tx_data[10]=(crc16Num(can_tx_data,9)>>8)&0xFF;
			can_tx_data[9]=crc16Num(can_tx_data,9)&0xFF;	
			can_tx_data[11]=0x0D;
			can_tx_data[12]=0x0A;			
			APP_CAN_SEND_DATA(can_tx_data,13,HMI_BROADCAST_ADDR);			
			break;
		case HMI_CODE_LIQUID_DEPTH:
			u_sys_param.sys_config_param.treatment_water_depth_high=data[4];
			u_sys_param.sys_config_param.treatment_water_depth_low=data[6];
			u_sys_param.sys_config_param.cool_water_depth_high=data[7];
			u_sys_param.sys_config_param.cool_water_depth_low=data[9];		
			can_tx_data[2]=14;			
			can_tx_data[4]=u_sys_param.sys_config_param.treatment_water_depth_high;
			can_tx_data[5]=(uint8_t)(sEnvParam.treatment_water_depth*10);
			can_tx_data[6]=u_sys_param.sys_config_param.treatment_water_depth_low;		
			can_tx_data[7]=u_sys_param.sys_config_param.cool_water_depth_high;
			can_tx_data[8]=(uint8_t)(sEnvParam.cool_water_depth*10);
			can_tx_data[9]=	u_sys_param.sys_config_param.cool_water_depth_low;
			can_tx_data[11]=(crc16Num(can_tx_data,10)>>8)&0xFF;
			can_tx_data[10]=crc16Num(can_tx_data,10)&0xFF;
			can_tx_data[12]=0x0D;
			can_tx_data[13]=0x0A;
			can_tx_data[14]=0;
			can_tx_data[15]=0;
			APP_CAN_SEND_DATA(can_tx_data,14,HMI_BROADCAST_ADDR);
			break;
		case HMI_CODE_PHOTODIOD:
			u_sys_param.sys_config_param.photodiod_low=data[4]|(data[5]<<8);
			u_sys_param.sys_config_param.photodiod_mid=data[6]|(data[7]<<8);
			u_sys_param.sys_config_param.photodiod_high=data[8]|(data[9]<<8);
			app_hmi_cmd_ack(hmi_code) ;
			break;
		case HMI_CODE_ENERGE_CALIBRATION:			
			dw=((data[6]|data[7]<<8)/5);			
			dw%=40;
			if(dw>0) dw-=1;
			if(data[4]==0)	
			{
				u_sys_param.sys_config_param.e_cali[dw].energe_cali=data[8]|(data[9]<<8);
			}		
			else 
			{
				u_sys_param.sys_config_param.e_cali[dw].power_cali=data[8]|(data[9]<<8);
			}
			laser_ctr_param.laserEnerge=(data[6]|(data[7]<<8));						
			laser_ctr_param.laserFreq=10;//固定					
			app_hmi_cmd_ack(hmi_code);
			break;
		case HMI_CODE_CLEAN_AND_DISINFECTION_TIME:
			u_sys_param.sys_config_param.clean_time_min=data[4];
			u_sys_param.sys_config_param.dit_time_min=data[5];
			laser_ctr_param.cleanCtr = data[6];	
			app_t_clean_run_timer(&laser_ctr_param.cleanCtr);
			app_hmi_cmd_ack(hmi_code);
			break;			
		case HMI_CODE_COOL_TEMPRATURE:
			u_sys_param.sys_config_param.cool_temprature_low=data[4]|(data[5]<<8);
			u_sys_param.sys_config_param.cool_temprature_target=data[6]|(data[7]<<8);
			u_sys_param.sys_config_param.cool_temprature_high=data[8]|(data[9]<<8);
			app_hmi_cmd_ack(hmi_code) ;
			break;
		case HMI_CODE_WATER_MIST_PREPARE:
			laser_ctr_param.air_water_prepare_ctr = data[4];		
			app_hmi_cmd_ack(hmi_code);
			break;
		case HMI_CODE_START_TIMER_LASER:
			laser_ctr_param.timerCtr = data[4];	
			laser_ctr_param.timerEnableFlag = data[5];						
			app_hmi_cmd_ack(hmi_code);
			break;
		case HMI_CODE_CONFIG_PARAM_SYNCHRONOUS:		
			if(data[4]==1)//load param			
			{
				memcpy(u_sys_param.data,&data[4],SYS_LASER_CONFIG_PARAM_LENGTH);
				can_tx_data[2]=13;
				u_sys_param.sys_config_param.synchronousFlag=3;
				can_tx_data[4]=3;//syncFlag
				//check param
				if(u_sys_param.sys_config_param.cool_temprature_target>280||u_sys_param.sys_config_param.cool_temprature_target<210)
				{
				  u_sys_param.sys_config_param.cool_temprature_target=240;
				} 
				unsigned int idBuff=data[5]|(data[6]<<8)|(data[7]<<16)|(data[8]<<24);				
				if(idBuff!=0)
				{
					u_sys_param.sys_config_param.equipmentId=data[5]|(data[6]<<8)|(data[7]<<16)|(data[8]<<24);					
				}
				can_tx_data[5]=u_sys_param.sys_config_param.equipmentId&0xFF;
				can_tx_data[6]=(u_sys_param.sys_config_param.equipmentId>>8)&0xFF;
				can_tx_data[7]=(u_sys_param.sys_config_param.equipmentId>>16)&0xFF;
				can_tx_data[8]=(u_sys_param.sys_config_param.equipmentId>>24)&0xFF;
				can_tx_data[10]=(crc16Num(can_tx_data,9)>>8)&0xFF;
				can_tx_data[9]=crc16Num(can_tx_data,9)&0xFF;
				can_tx_data[10]=0x0D;
				can_tx_data[11]=0x0A;
				can_tx_data[12]=0;
				can_tx_data[13]=0;
				can_tx_data[14]=0;
				can_tx_data[15]=0;
				APP_CAN_SEND_DATA(can_tx_data,12,HMI_BROADCAST_ADDR);//ack      		
			}
			else if(data[4]==2)//up param				 
			{		
				can_tx_data[2]=SYS_LASER_CONFIG_PARAM_LENGTH+8;
				u_sys_param.sys_config_param.synchronousFlag=2;									
				memcpy(&can_tx_data[4],&u_sys_param.data[0],SYS_LASER_CONFIG_PARAM_LENGTH);	
				can_tx_data[4]=2;//;//syncFlag											
				can_tx_data[SYS_LASER_CONFIG_PARAM_LENGTH+5]=(crc16Num(can_tx_data,SYS_LASER_CONFIG_PARAM_LENGTH+4)>>8)&0xFF;
				can_tx_data[SYS_LASER_CONFIG_PARAM_LENGTH+4]=crc16Num(can_tx_data,SYS_LASER_CONFIG_PARAM_LENGTH+4)&0xFF;
				can_tx_data[SYS_LASER_CONFIG_PARAM_LENGTH+6]=0x0D;
				can_tx_data[SYS_LASER_CONFIG_PARAM_LENGTH+7]=0x0A;
				can_tx_data[223]=0;
				APP_CAN_SEND_DATA(can_tx_data,SYS_LASER_CONFIG_PARAM_LENGTH+8,HMI_BROADCAST_ADDR);	//ack
			}
			else if(data[4]==3)//up param			
			{	
				u_sys_param.sys_config_param.synchronousFlag=3;	
				u_sys_param.sys_config_param.equipmentId=data[5]|(data[6]<<8)|(data[7]<<16)|(data[8]<<24);
			}
			break;			
		default:
			break;
	}	

}
	

