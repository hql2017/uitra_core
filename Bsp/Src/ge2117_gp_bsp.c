/*
 *ge2117_gp_bsp.c
 *
 *  Created on: Apr 10, 2025
 *      Author: Hql2017
 */
#include "main.h"
#include "ge2117_gp_bsp.h" 
#include "usart.h" 
#include <stdio.h>

#define MAX_GE_UART_BUFF_LENTH 64
static unsigned char UART_GE_TX_BUFF[MAX_GE_UART_BUFF_LENTH]={0};
static unsigned char UART_GE_RX_BUFF[MAX_GE_UART_BUFF_LENTH]={0};

typedef struct {
	unsigned short int idle;//0空闲：1正处于应带侦听端口
	unsigned char frame_length;// 侦听长度
	unsigned char frame_err;// 数据报错
	unsigned char frame_bus_timeOut;// 等待超时
}ge2117_modbus_status;
static ge2117_modbus_status  ge_run_sta;

typedef struct {
	unsigned int  		geTimeS;// 驱动器运行时间	//
	unsigned short int  compressorSetSpd;//压缩机设定转速
	unsigned char 		startUpStaus;// 启动标志0关机；1开机器
	unsigned char 		wkTimeOut;// 串口周期时间,超时时间
	unsigned char 		ge_seriel_err;// 串口周期时间,超时时间0 正常；1超时；

	unsigned short int  workStaus;// 工作状态1工作；0待机	//	reg0x2000	
	unsigned short int  compressorRunSpd;//压缩机运行转速
	unsigned short int  reverse;//保留
	unsigned short int  compressorCurrent;//电流
	unsigned short int  compressorVoltage;//压缩机母线电压	
	unsigned short int  errrCode;// 故障代码
	unsigned short int 	driverTemprature;// 驱动器温度	     //	reg0x2006
}ge2117_work_status;
static ge2117_work_status geWksta;
static unsigned short int ge2117_reg_sta[7];//addr 0x2000~0x2006
#if 1
unsigned char GPSend_data[16]={0xAA,0x0,0x00,0xB8,0x0B,0x00,0x00,0x00,0x00,0x00,\
	0x00,0x00,0x00,0x00,0xDA,0x55};
//发送变量
unsigned char GPReceive_data[16]={0xAA,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x55};
#endif
//空调制冷压缩机/
 /***************************************************************************//**
 * @brief MODBUS/CRC-16
 * @param *data缓存，长度len
 * @note 多项式0xA001 
 * @return 返回crc16结果
*******************************************************************************/
static uint16_t ge_crc16_modbus(const  uint8_t *data, uint16_t len)
{  
	uint16_t crc = 0xFFFF;  // 初始值
	uint16_t polynomial=0xA001;	
	for (size_t pos = 0; pos < len; pos++) 
	{
		crc ^= (uint16_t)data[pos];  
		for (uint8_t i = 8; i != 0; i--) 
		{  
			if ((crc & 0x0001) != 0) 
			{  // 如果最低位是1
				crc >>= 1;
				crc ^= polynomial;
			} 
			else 
			{
				crc >>= 1;
			}
		}
	}
	return  crc;
}  
 /***************************************************************************//**
 * @brief 应答帧处理
 * @param listenReg侦听端口，datalen，侦听数据长度
 * @note 
 * @return 
*******************************************************************************/
void app_ge2117_receive_data_handle(void)
{
	uint16_t crcValue,regStart,regNum;
	uint8_t readDataLen;  
	if(UART_GE_RX_BUFF[0]==GE2117_PACKAGE_HEAD)//私有控制协议else 
	{
		ge_run_sta.idle=0;
		ge_run_sta.frame_length = 0;
		ge_run_sta.frame_bus_timeOut=0;
		ge_run_sta.frame_err=0;
		geWksta.wkTimeOut=0;
		/*		if(UART_GE_RX_BUFF[1]==GE2117_SLAVE_ADD)
		{
			if(UART_GE_RX_BUFF[15]==GE2117_PACKAGE_END)
		   {
				geWksta.compressorRunSpd=UART_GE_RX_BUFF[2]|UART_GE_RX_BUFF[3]<<8;
				geWksta.compressorCurrent=UART_GE_RX_BUFF[4]|UART_GE_RX_BUFF[5]<<8;
				geWksta.compressorVoltage=UART_GE_RX_BUFF[6]|UART_GE_RX_BUFF[7]<<8;
				geWksta.driverTemprature=UART_GE_RX_BUFF[8];
				geWksta.errrCode=UART_GE_RX_BUFF[9];
				geWksta.workStaus=UART_GE_RX_BUFF[10];//工作状态
				geWksta.wkTimeOut=0;		
		   }
		}		*/
	}
	else
	{		
		if(UART_GE_RX_BUFF[0]==GE2117_SLAVE_ADD)//modbus  data
		{		
			if(UART_GE_RX_BUFF[1]==GE2117_SINGLE_WRITE_CODE)//code，写回应,固定8字节，只写单个寄存器
			{   
				regStart=(UART_GE_RX_BUFF[2]<<8)|UART_GE_RX_BUFF[3];
				crcValue=UART_GE_RX_BUFF[6]|(UART_GE_RX_BUFF[7]<<8);
				if(crcValue==ge_crc16_modbus(UART_GE_RX_BUFF, 6))
				{	 
					DEBUG_PRINTF("regWriteOk\r\n");
					ge_run_sta.idle=0;
					ge_run_sta.frame_length = 0;
					ge_run_sta.frame_bus_timeOut=0;
					ge_run_sta.frame_err=0;
					geWksta.wkTimeOut=0;
				}
				else ge_run_sta.frame_err=1;
			}
			else 
			{	
				if(ge_run_sta.idle<0x2000)//配置寄存器
				{
					#if 1
					ge_run_sta.frame_err=0;
					ge_run_sta.idle=0;
					ge_run_sta.frame_length =0;
					ge_run_sta.frame_err=0;	
					ge_run_sta.frame_bus_timeOut=0;
					geWksta.wkTimeOut=0;					
					#endif	
					/*暂时不读配置寄存器
					regStart=ge_run_sta.idle-GE2117_REG_MODBUS_ADDR;//ge_run_sta.idle-GE2117_REG_MODBUS_ADDR;	//读状态		
					readDataLen=UART_GE_RX_BUFF[2];				
					crcValue=UART_GE_RX_BUFF[readDataLen+3]| (UART_GE_RX_BUFF[readDataLen+4]<<8);			
					if(crcValue==ge_crc16_modbus(UART_GE_RX_BUFF, readDataLen+3))
					{ 
						regNum=readDataLen>>1;	
						//DEBUG_PRINTF("regRead:");
						for(uint8_t i=0;i<regNum;i++)
						{
							ge2117_reg_sta[regStart+i]=UART_GE_RX_BUFF[3+2*i]<<8|UART_GE_RX_BUFF[4+2*i];						
							//DEBUG_PRINTF("  %04x",ge_run_sta[regStart+i]);	
						}
						//DEBUG_PRINTF(" regend\r\n");
						ge_run_sta.idle=0;
						ge_run_sta.frame_length = 0;
						ge_run_sta.frame_err=0;	
						ge_run_sta.frame_bus_timeOut=0;			
					}
					else ge_run_sta.frame_err=1;
					*/		
				}
				else //状态寄存器
				{
					regStart=ge_run_sta.idle-GE2117_REG_STATUS1;//ge_run_sta.idle-GE2117_REG_MODBUS_ADDR;	//读状态		
					readDataLen=UART_GE_RX_BUFF[2];	//14			
					crcValue=UART_GE_RX_BUFF[readDataLen+3]|(UART_GE_RX_BUFF[readDataLen+4]<<8);			
					if(crcValue==ge_crc16_modbus(UART_GE_RX_BUFF, readDataLen+3))
					{ 
						regNum=readDataLen>>1;	
						//DEBUG_PRINTF("regRead:");
						for(uint8_t i=0;i<regNum;i++)
						{
							ge2117_reg_sta[regStart+i]=UART_GE_RX_BUFF[3+2*i]<<8|UART_GE_RX_BUFF[4+2*i];						
							//DEBUG_PRINTF("  %04x",ge2117_reg_sta[regStart+i]);	
						}
						geWksta.workStaus      =    ge2117_reg_sta[0];//工作状态
						geWksta.compressorRunSpd  = ge2117_reg_sta[1];						
						geWksta.compressorVoltage = ge2117_reg_sta[3];
						geWksta.compressorCurrent = ge2117_reg_sta[4];
						geWksta.errrCode       =    ge2117_reg_sta[5];
						geWksta.driverTemprature  = ge2117_reg_sta[6];				
						geWksta.wkTimeOut=0;
						//DEBUG_PRINTF(" regend\r\n");
						ge_run_sta.idle=0;
						ge_run_sta.frame_length =0;
						ge_run_sta.frame_err=0;	
						ge_run_sta.frame_bus_timeOut=0;			
					}
					else ge_run_sta.frame_err=1;
				}
			}
		}
	    else 
		{
			ge_run_sta.idle=0;
			ge_run_sta.frame_length = 0;
			ge_run_sta.frame_bus_timeOut=0;
			ge_run_sta.frame_err=0;
		}
	}		
}
/***************************************************************************//**
 * @brief 压缩机串口通讯
 * @param void
 * @note 
 * @return 
*******************************************************************************/
void ge2117_UART_Init(uint32_t BaudRate)
{
	huart5.Instance = UART5;
	huart5.Init.BaudRate = BaudRate;
	huart5.Init.WordLength = UART_WORDLENGTH_8B;
	huart5.Init.StopBits = UART_STOPBITS_1;
	huart5.Init.Parity = UART_PARITY_NONE;
	huart5.Init.Mode = UART_MODE_TX_RX;
	huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart5.Init.OverSampling = UART_OVERSAMPLING_16;
	huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart5.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart5) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart5, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart5, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart5) != HAL_OK)
	{
		Error_Handler();
	}	 

}
 /***************************************************************************//**
 * @brief 压缩机初始化
 * @param void
 * @note 
 * @return 
*******************************************************************************/
void ge2117_gp_init(void)
{
	ge2117_UART_Init(9600);	
	
}
  /***************************************************************************//**
 * @brief 进入侦听状态
 * @param listenReg,侦听端口，datalen，侦听数据长度
 * @note 
 * @return 
*******************************************************************************/
void app_ge_lisen(uint16_t listenReg,uint16_t dataLen)
{	
	ge_run_sta.idle=listenReg;
	ge_run_sta.frame_length =dataLen;


	ge_run_sta.frame_bus_timeOut=0;		
	HAL_UART_Receive_IT(&huart5,UART_GE_TX_BUFF, dataLen);//切换进入侦听状态	
}
/***************************************************************************//**
 * @brief 问询帧
 * @param regStart 寄存器起始；
 * @note  只允许写单个寄存器
 * @return 返回操作状态，如果成功等待从机应答
*******************************************************************************/
HAL_StatusTypeDef app_ge_write_req_frame(uint16_t regStart,uint16_t data)
{ 
	HAL_StatusTypeDef err=HAL_OK;
	
	if(ge_run_sta.idle!=0) 
	{
		ge_run_sta.frame_bus_timeOut++;
		if(ge_run_sta.frame_bus_timeOut>2)
		{
			ge_run_sta.idle=0;
			ge_run_sta.frame_bus_timeOut=0;
			return HAL_BUSY;
		}		
	} 
	UART_GE_TX_BUFF[0]=GE2117_SLAVE_ADD; 
	UART_GE_TX_BUFF[1]=GE2117_SINGLE_WRITE_CODE;
	UART_GE_TX_BUFF[2]=(regStart>>8)&0xFF;
	UART_GE_TX_BUFF[3]=regStart&0xFF;		
	if(regStart==GE2117_REG_SPD||regStart==GE2117_REG_SPD_STRAT) 
	{
		geWksta.compressorSetSpd=data;
		if(geWksta.compressorSetSpd<1600) geWksta.compressorSetSpd=1600;
		if(geWksta.compressorSetSpd>7000) geWksta.compressorSetSpd=7000;
	}
	UART_GE_TX_BUFF[4]=(data>>8)&0xFF;
	UART_GE_TX_BUFF[5]=data&0xFF;  
	UART_GE_TX_BUFF[6]=ge_crc16_modbus(UART_GE_TX_BUFF,6)&0xFF;
	UART_GE_TX_BUFF[7]=(ge_crc16_modbus(UART_GE_TX_BUFF,6)>>8)&0xFF;
	err = HAL_UART_Transmit(&huart5,UART_GE_TX_BUFF, 8, 100);	
	if(err==HAL_OK) app_ge_lisen(regStart,8);
	else ge_run_sta.idle=0;
	return err;
}
/***************************************************************************//**
 * @brief 问询帧
 * @param regStart 寄存器起始；regOffset寄存器偏移量>=1
 * @note 
 * @return 返回操作状态，如果成功等待从机应答
*******************************************************************************/
HAL_StatusTypeDef app_ge_read_req_frame(uint16_t regStart,uint16_t regOffset)
{ 
	HAL_StatusTypeDef err=HAL_OK;
		
	if(ge_run_sta.idle!=0) 
	{
		ge_run_sta.frame_bus_timeOut++;
		if(ge_run_sta.frame_bus_timeOut>2)
		{
			ge_run_sta.idle = 0;
			ge_run_sta.frame_bus_timeOut = 0;
			return HAL_BUSY;
		}		 
	}	
	UART_GE_TX_BUFF[0]=GE2117_SLAVE_ADD;	
	UART_GE_TX_BUFF[1]=GE2117_REDA_CODE;
	UART_GE_TX_BUFF[2]=(regStart>>8)&0xFF;
	UART_GE_TX_BUFF[3]=regStart&0xFF;	
	UART_GE_TX_BUFF[4]=(regOffset>>8)&0xFF;
	UART_GE_TX_BUFF[5]=regOffset&0xFF;
	UART_GE_TX_BUFF[6]=ge_crc16_modbus(UART_GE_TX_BUFF,6)&0xFF;
	UART_GE_TX_BUFF[7]=(ge_crc16_modbus(UART_GE_TX_BUFF,6)>>8)&0xFF;
	err = HAL_UART_Transmit(&huart5,UART_GE_TX_BUFF, 8, 100);	
	if(err==HAL_OK) app_ge_lisen(regStart,regOffset*2+5);
	else ge_run_sta.idle=0;
	return err;
}
 /***************************************************************************//**
 * @brief 压缩机启停
 * @param startFlag 启停标志，3启动；4停止
 * @note 
 * @return 
*******************************************************************************/
void ge2117_start_up_set(unsigned short int startFlag)
{	
	if(startFlag==3)	
	{
		//app_ge_write_req_frame(GE2117_REG_START_STOP,3);//启动	
		if(geWksta.compressorSetSpd<2500)		geWksta.compressorSetSpd=2500;
		if(geWksta.compressorSetSpd>7000) geWksta.compressorSetSpd=7000;
		app_ge_write_req_frame(GE2117_REG_SPD_STRAT,geWksta.compressorSetSpd);//启动
	}
	else app_ge_write_req_frame(GE2117_REG_START_STOP,4);//停止
	//geWksta.startUpStaus=startFlag;
}
 /***************************************************************************//**
 * @brief 压缩机速度设置
 * @param spd 速度》1600压缩机开启
 * @note 
 * @return 
*******************************************************************************/
void ge2117_speed_set(unsigned short int spd)
{	
	app_ge_write_req_frame(GE2117_REG_SPD,spd);
	//geWksta.compressorSetSpd=spd;
}

 /***************************************************************************//**
 * @brief 发送控制帧
 * @param void
 * @note 
 * @return 
*******************************************************************************/
void app_ge2117_gp_ctr_frame(void)
{		
	#if  1		
	app_ge_read_req_frame(GE2117_REG_STATUS1,7);//读状态
	#else 
	uint8_t i;
	uint8_t temp;
	UART_GE_TX_BUFF[0]=GE2117_PACKAGE_HEAD;
	UART_GE_TX_BUFF[1]=0x00;//addr,广播地址
	UART_GE_TX_BUFF[2]=geWksta.startUpStaus;//开关机
	UART_GE_TX_BUFF[3]=geWksta.compressorSetSpd&0x00FF;//spdL
	UART_GE_TX_BUFF[4]=geWksta.compressorSetSpd>>8;//spdH
	for(i=0;i<9;i++)
	{
		UART_GE_TX_BUFF[5+i]=0x00;
	}
	UART_GE_TX_BUFF[14]=0;
	for(i=0;i<14;i++)
	{
		if(UART_GE_TX_BUFF[i]==0xAA) UART_GE_TX_BUFF[i]=0xAB;
		if(UART_GE_TX_BUFF[i]==0x55) UART_GE_TX_BUFF[i]=0x56;
		UART_GE_TX_BUFF[14]+=UART_GE_TX_BUFF[i];//校验和
	}
	UART_GE_TX_BUFF[15]=GE2117_PACKAGE_END;
	HAL_StatusTypeDef err = HAL_UART_Transmit(&huart5,UART_GE_TX_BUFF, 16, 100);	
	if(err==HAL_OK) app_ge_lisen(GE2117_PACKAGE_HEAD,16);
	#endif
}
 /***************************************************************************//**
 * @brief 压缩机控制函数，1秒调用一次
 * @param circleWaterTmprature 冷却水温度，
 * @note 循环水需要维持24~26摄氏度，25℃最佳
 * @return 
*******************************************************************************/
void app_ge2117_gp_ctr(float  circleWaterTmprature,unsigned int sysTimeS)
{	
	unsigned char bus_idle_flag;
	static unsigned char  runFlag=0;		
	static unsigned int geRunTime=0;//压缩机持续时间
	geWksta.geTimeS++;	
	if(geWksta.geTimeS>10)//10秒后开始通讯
	{ 	
		if(runFlag==0)	
		{
			if(circleWaterTmprature>MAX_TEMPRATURE_LASER)
			{
				runFlag=1;
				ge2117_start_up_set(GE2117_START_CMD);
				bus_idle_flag=1;
			}
		}
		else
		{
			geRunTime++;	
			if(circleWaterTmprature<MID_TEMPRATURE_LASER) 
			{				
				if(geWksta.workStaus!=0)
				{
					runFlag=0;
					ge2117_start_up_set(GE2117_STOP_CMD);
					bus_idle_flag=1;
				}							
			}
			if(geWksta.workStaus!=0)	
			{		
				if(geRunTime>120&&(circleWaterTmprature>MAX_TEMPRATURE_LASER))	//2分钟还没降下来，增加制冷功率	 
				{
					geRunTime=0;
					geWksta.compressorSetSpd=geWksta.compressorRunSpd+500;
					geWksta.compressorSetSpd%=7000;					
					ge2117_start_up_set(GE2117_START_CMD);//改变速度
					//ge2117_speed_set(geWksta.compressorSetSpd);
					bus_idle_flag=1;
				}	
			}
			else 
			{
				if(geRunTime>5)//5秒重启
				{
					ge2117_start_up_set(GE2117_START_CMD);
					bus_idle_flag=1;
				}
			}			
		}
		if(bus_idle_flag!=0) 	 bus_idle_flag=0;
		else app_ge2117_gp_ctr_frame();//status req			
		if(geWksta.wkTimeOut>10 ) //10秒没数据报错
		{
			geWksta.ge_seriel_err=1;			
		} 			
		geWksta.wkTimeOut++;
	}

}


	