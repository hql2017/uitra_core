/*
 * mer_mcp1081_bsp.c
 *
 *  Created on: Apr 10, 2025
 *      Author: Hql2017
 */
#include "main.h"
#include "mer_mcp1081_bsp.h" 
#include "usart.h" 
#include <stdio.h>

#define MAX_USART1_BUFF_LENTH 64
static unsigned char USART1_TX_BUFF[MAX_USART1_BUFF_LENTH]={0};
static unsigned char USART1_RX_BUFF[MAX_USART1_BUFF_LENTH]={0};
typedef struct{
	uint16_t mer_sampling_value;//采集平均次数
	uint16_t mer_deph_value;//液挡位
	uint16_t mer_calibration_value;//calibrati
	uint16_t no_reverse_val;//保留
	int16_t mer_temprature;
	uint16_t mer_c0_value;
	uint16_t mer_c1_value;
	uint16_t mer_c2_value;
	uint16_t mer_c3_value;
	uint16_t mer_c4_value;
	uint16_t mer_c5_value;
	uint16_t mer_c6_value;
	uint16_t mer_c_freq_value;
	uint16_t mer_c_fq0_value;
	uint16_t mer_c_fq1_value;
	uint16_t mer_c_fq2_value;
	uint16_t mer_c_fq3_value;
	uint16_t mer_c_fq4_value;
	uint16_t mer_c_fq5_value;
	uint16_t mer_c_fq6_value;
	uint16_t mer_i_ch10_value;
	uint16_t mer_i_ch0_value;
	uint16_t mer_i_ch1_value;
	uint16_t mer_i_ch2_value;
	uint16_t mer_i_ch3_value;
	uint16_t mer_i_ch4_value;
	uint16_t mer_i_ch5_value;
	uint16_t mer_i_ch6_value;
}mer_reg_local_value;//sieof==28
typedef struct {
	unsigned char idle;//0空闲：1正处于应带侦听端口
	unsigned char frame_length;// 侦听长度
	unsigned char frame_err;// 数据报错
	unsigned char frame_bus_timeOut;// 等待超时
}mer_mcp_modbus_status;
static mer_mcp_modbus_status  mer_run_sta; 
//static mer_reg_local_value  mer_reg_value;

static uint16_t  mer_reg_value[MER_MAX_REG_NUM]={0};
#ifdef LLS_MCP61
static uint16_t  mcp61_reg_value=0;
static uint16_t  mcp61_min_capacitance=0;//
static uint16_t  mcp61_max_capacitance=0;
static uint16_t  mcp61_water_depth=0;//0~10;百分比10个等级
#endif
#ifdef LLM
typedef struct{
	float min_max_capacitanceValue[10];//pF（0~100pF）等级表
	float enviromentTmprature;//℃
	float freq;//mHz
	float capacitanceValue;//pF（0~100pF）
}LLM_REC_DATA;
static LLM_REC_DATA llm_rec_data;

#endif
 /***************************************************************************//**
 * @brief MODBUS/CRC-16
 * @param *data缓存，长度len
 * @note 多项式0xA001 
 * @return 返回crc16结果
*******************************************************************************/
static uint16_t crc16_modbus(const  uint8_t *data, uint16_t len)
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
 * @brief 液位传感器，从机应答帧处理
 * @param listenReg侦听端口，datalen，侦听数据长度
 * @note 
 * @return 
*******************************************************************************/

void app_mer_receive_data_handle(void)
{
	uint16_t crcValue,regStart,regNum;
	uint8_t readDataLen;	
	#ifdef LLM
		uint8_t i;
		uint32_t valueH,valuePo;
		float temp;
		if( USART1_RX_BUFF[0]==LLM_PACKAGE_HEAD&&USART1_RX_BUFF[LLM_PACKAGE_LENGTH-1]==LLM_PACKAGE_END)	
		{	
			//DEBUG_PRINTF(" llm rec[");
			//for(i=0;i<44;i++)
			//{
			//	DEBUG_PRINTF("%02x ",USART1_RX_BUFF[i]);
			//}
			//DEBUG_PRINTF(" ]");	
			for(i=0;i<5;i++)//找小数点
			{
				if(USART1_RX_BUFF[35+i]==0x2E)//小数点
				{
				break;
				}
			}					
			if(i==0)//1.0000
			{
				temp=(USART1_RX_BUFF[34]-0x30)+ //整数部分
				(USART1_RX_BUFF[36]-0x30)*0.1000+(USART1_RX_BUFF[37]-0x30)*0.0100+(USART1_RX_BUFF[38]-0x30)*0.0010+(USART1_RX_BUFF[39]-0x30)*0.0001;//小数部分			
			}	
			else if (i==1)//10.000
			{
				temp=(USART1_RX_BUFF[34]-0x30)*10.000+(USART1_RX_BUFF[35]-0x30)*1.0000+ //整数部分
				(USART1_RX_BUFF[37]-30)*0.1000+(USART1_RX_BUFF[38]-30)*0.0100+(USART1_RX_BUFF[39]-0x30)*0.0010;//小数部分			
			}
			else if (i==2)//100.00
			{
				temp=(USART1_RX_BUFF[34]-0x30)*100.00+(USART1_RX_BUFF[35]-0x30)*10.000+(USART1_RX_BUFF[36]-0x30)*1.0000+ //整数部分
				(USART1_RX_BUFF[38]-0x30)*0.1000+(USART1_RX_BUFF[39]-30)*0.0100;//小数部分
			}	
			else //if (i==4)//1000.0
			{
				temp=(USART1_RX_BUFF[34]-0x30)*1000.0+(USART1_RX_BUFF[35]-0x30)*100.00+(USART1_RX_BUFF[36]-0x30)*10.000+(USART1_RX_BUFF[37]-0x30)*1.0000+  //整数部分
				(USART1_RX_BUFF[39]-0x30)*0.1;//小数部分
			}	
			llm_rec_data.capacitanceValue = temp;
			
			//temprature
			temp=(USART1_RX_BUFF[8]-0x30)*10.0+(USART1_RX_BUFF[9]-0x30)*1.0+  //整数部分
				(USART1_RX_BUFF[11]-0x30)*0.1+(USART1_RX_BUFF[12]-0x30)*0.01;//小数部分
			llm_rec_data.enviromentTmprature=temp;
			//freq
			temp=(USART1_RX_BUFF[20]-0x30)*10.0+(USART1_RX_BUFF[21]-0x30)*1.0+  //整数部分
				(USART1_RX_BUFF[23]-0x30)*0.1+(USART1_RX_BUFF[24]-0x30)*0.01+(USART1_RX_BUFF[25]-0x30)*0.001;//小数部分
			llm_rec_data.freq=temp;
			mer_run_sta.idle=0;
			mer_run_sta.frame_length = 0;
			mer_run_sta.frame_bus_timeOut=0;
			mer_run_sta.frame_err=0;
			//DEBUG_PRINTF("llm T=%.2f F=%.3f C=%.2f",llm_rec_data.enviromentTmprature,llm_rec_data.freq,llm_rec_data.capacitanceValue);
		}	
		else
		{
			mer_run_sta.idle=0;
			mer_run_sta.frame_length = 0;
			mer_run_sta.frame_bus_timeOut=0;
			mer_run_sta.frame_err=0;		
		}
		
	#else 
	if(USART1_RX_BUFF[0]==MER_SLAVE_ADD)
	{		
		if(USART1_RX_BUFF[1]==MER_SINGLE_WRITE_CODE)//code，写回应,固定8字节，只写单个寄存器
		{   
			regStart=(USART1_RX_BUFF[2]<<8)|USART1_RX_BUFF[3];
			crcValue=(USART1_RX_BUFF[6]>>8)| (USART1_RX_BUFF[7]<<8);
			if(crcValue==crc16_modbus(USART1_RX_BUFF, 6))
			{	 
				DEBUG_PRINTF("regWriteOk\r\n");
				mer_run_sta.idle=0;
				mer_run_sta.frame_length = 0;
				mer_run_sta.frame_bus_timeOut=0;
				mer_run_sta.frame_err=0;
			}
			else mer_run_sta.frame_err=1;
		}
		else 
		{			
			#ifdef LLS_MCP61
			regStart=MCP61_REG_SINGGLE_C_VALUE;			
			readDataLen=USART1_RX_BUFF[2];
			crcValue=USART1_RX_BUFF[readDataLen+3]| (USART1_RX_BUFF[readDataLen+4]<<8);
			if(crcValue==crc16_modbus(USART1_RX_BUFF, readDataLen+3))
			{
				
				 if(mer_run_sta.frame_err==1)
				{
					mcp61_min_capacitance=	USART1_RX_BUFF[3]<<8|USART1_RX_BUFF[4];
					DEBUG_PRINTF(" cali min=%04x",mcp61_min_capacitance);	
					mer_run_sta.frame_err=0;	
				}
				else if(mer_run_sta.frame_err==2)
				{
					mcp61_max_capacitance=	USART1_RX_BUFF[3]<<8|USART1_RX_BUFF[4];
					DEBUG_PRINTF(" cali max=%04x",mcp61_max_capacitance);	
					mer_run_sta.frame_err=0;
				}
				else 
				{
					mcp61_reg_value =	USART1_RX_BUFF[3]<<8|USART1_RX_BUFF[4];
					//DEBUG_PRINTF("  mcp61C=%04x =%.3f",mcp61_reg_value ,mcp61_reg_value*0.001);		
				}		
				mer_run_sta.idle=0;
				mer_run_sta.frame_length =0;		
				mer_run_sta.frame_bus_timeOut=0;			
			}
			
			#else 
				regStart=mer_run_sta.idle-MER_REG_SMAMLING_SPD;			
				readDataLen=USART1_RX_BUFF[2];				
				crcValue=USART1_RX_BUFF[readDataLen+3]| (USART1_RX_BUFF[readDataLen+4]<<8);			
				if(crcValue==crc16_modbus(USART1_RX_BUFF, readDataLen+3))
				{ 
					regNum=readDataLen>>1;	
					//DEBUG_PRINTF("regRead:");
					for(uint8_t i=0;i<regNum;i++)
					{
						mer_reg_value[regStart+i]=USART1_RX_BUFF[3+2*i]<<8|USART1_RX_BUFF[4+2*i];						
						//DEBUG_PRINTF("  %04x",mer_reg_value[regStart+i]);	
					}
					//DEBUG_PRINTF(" regend\r\n");
					mer_run_sta.idle=0;
					mer_run_sta.frame_length =0;
					mer_run_sta.frame_err=0;	
					mer_run_sta.frame_bus_timeOut=0;			
				}
				else mer_run_sta.frame_err=1;
			#endif	
		}
	}	
	#endif
	
}
  /***************************************************************************//**
 * @brief 进入侦听状态
 * @param listenReg,侦听端口，datalen，侦听数据长度
 * @note 
 * @return 
*******************************************************************************/
void app_mer_lisen(uint16_t listenReg,uint16_t dataLen)
{	
	mer_run_sta.idle=listenReg;
	mer_run_sta.frame_length =dataLen;
	mer_run_sta.frame_bus_timeOut=0;	
	#ifdef LLS_MCP61
	mer_run_sta.frame_err=0;// mcp61 用来指示是否校准状态0，正常；1校准空载；2校准满载	
	#endif
	HAL_UART_Receive_IT(&huart1,USART1_RX_BUFF, dataLen);//切换进入侦听状态	
}
  /***************************************************************************//**
 * @brief 获取总线侦听状态
 * @param 
 * @note 
 * @return 侦听状态，0空闲 !0等待应答帧
*******************************************************************************/
uint16_t app_get_mer_lisen_status(void)
{			
	return mer_run_sta.idle;
}
/***************************************************************************//**
 * @brief 问询帧
 * @param regStart 寄存器起始；
 * @note  只允许写单个寄存器
 * @return 返回操作状态，如果成功等待从机应答
*******************************************************************************/
HAL_StatusTypeDef app_mer_write_req_frame(uint16_t regStart,uint16_t data)
{ 
	HAL_StatusTypeDef err=HAL_OK;
	USART1_TX_BUFF[0]=MER_SLAVE_ADD;
	if(mer_run_sta.idle!=0) 
	{
		mer_run_sta.frame_bus_timeOut++;
		if(mer_run_sta.frame_bus_timeOut>2)//超过3次未回复
		{
			mer_run_sta.idle=0;
			mer_run_sta.frame_bus_timeOut=0;
		}
		return HAL_BUSY;
	}
	if(regStart<MER_REG_SMAMLING_SPD||regStart>MER_REG_I_CH6_VA)
	{	
		mer_run_sta.idle=0;
		return HAL_ERROR;		
	}
	USART1_TX_BUFF[1]=MER_SINGLE_WRITE_CODE;
	USART1_TX_BUFF[2]=(regStart>>8)&0xFF;
	USART1_TX_BUFF[3]=regStart&0xFF;		
	USART1_TX_BUFF[4]=(data>>8)&0xFF;
	USART1_TX_BUFF[5]=data&0xFF;  
	USART1_TX_BUFF[6]=crc16_modbus(USART1_TX_BUFF,6)&0xFF;
	USART1_TX_BUFF[7]=(crc16_modbus(USART1_TX_BUFF,6)>>8)&0xFF;
	err = HAL_UART_Transmit(&huart1,USART1_TX_BUFF, 8, 100);	
	if(err==HAL_OK) app_mer_lisen(regStart,8);
	else mer_run_sta.idle=0;
	return err;
}
/***************************************************************************//**
 * @brief 问询帧
 * @param regStart 寄存器起始；regOffset寄存器偏移量>=1
 * @note 
 * @return 返回操作状态，如果成功等待从机应答
*******************************************************************************/
HAL_StatusTypeDef app_mer_read_req_frame(uint16_t regStart,uint16_t regOffset)
{ 
	HAL_StatusTypeDef err=HAL_OK;
	USART1_TX_BUFF[0]=MER_SLAVE_ADD;
	if(mer_run_sta.idle!=0) 
	{
		mer_run_sta.frame_bus_timeOut++;
		if(mer_run_sta.frame_bus_timeOut>2)
		{
			mer_run_sta.idle = 0;
			mer_run_sta.frame_bus_timeOut = 0;			
		} 
		return HAL_BUSY;
	}
	if(regStart<MER_REG_SMAMLING_SPD||regStart>MER_REG_I_CH6_VA)
	{	
		mer_run_sta.idle=0;
		return HAL_ERROR;		
	}
	if(regOffset>MER_MAX_READ_READNUM)
	{
		regOffset=MER_MAX_READ_READNUM;
	}   
	USART1_TX_BUFF[1]=MER_REDA_CODE;
	USART1_TX_BUFF[2]=(regStart>>8)&0xFF;
	USART1_TX_BUFF[3]=regStart&0xFF;	
	USART1_TX_BUFF[4]=(regOffset>>8)&0xFF;
	USART1_TX_BUFF[5]=regOffset&0xFF;
	USART1_TX_BUFF[6]=crc16_modbus(USART1_TX_BUFF,6)&0xFF;
	USART1_TX_BUFF[7]=(crc16_modbus(USART1_TX_BUFF,6)>>8)&0xFF;
	err = HAL_UART_Transmit(&huart1,USART1_TX_BUFF, 8, 100);	
	if(err==HAL_OK) app_mer_lisen(regStart,regOffset*2+5);
	else mer_run_sta.idle=0;
	return err;
}

/***************************************************************************//**
 * @brief 初始化
 * @param 
 * @note 
 * @return 返回操作状态，如果成功等待从机应答
*******************************************************************************/
void app_mer_init(void)
{ 
	HAL_StatusTypeDef err=HAL_OK;
	err=app_mer_write_req_frame(MER_REG_CALIBRATION,1);//校准
}
/***************************************************************************//**
 * @brief 获取设备ID
 * @param 
 * @note 用来确认通信正常
 * @return 返回ID
*******************************************************************************/
uint16_t  app_mer_get_ID(void)
{ 
	uint16_t ret;
	ret=mer_reg_value[2];
	return ret;
}
#ifdef LLS_MCP61
/***************************************************************************//**
 * @brief 获取mcp61读到的电容值
 * @param 
 * @note 
 * @return 返回深度值
*******************************************************************************/
float  app_mcp61_c_value(void)
{ 
	float ret ;
	ret=mcp61_reg_value*0.001;	//还原
	return ret;
}
/***************************************************************************//**
 * @brief 校准值
 * @param  min_max ，选择空载值和满载值
 * @note 
 * @return 返回无
*******************************************************************************/
void  app_mcp61_calibration(uint8_t min_max)
{
	if(min_max==1)
	{
		mer_run_sta.frame_err=1;
	}
	else  if(min_max==2)
	{
		mer_run_sta.frame_err=2;
	}
	app_mcp61_get_singgle_c_value_req();
}
#endif
/***************************************************************************//**
 * @brief 数据输出
 * @param 
 * @note 
 * @return 返回操作状态，如果成功等待从机应答
*******************************************************************************/
void app_mer_prin(void)
{ 
	//DEBUG_PRINTF("mer: smp=%d  depth=%d id=%d cali=%d\r\n",mer_reg_value[0],mer_reg_value[1],mer_reg_value[2],mer_reg_value[3]	);
	//DEBUG_PRINTF("c: T=%.1f c0=%.2f c1=%.2f c2=%.2f c3=%.2f c4=%.2f c5=%.2f c6=%.2f\r\n",mer_reg_value[4]*0.10,mer_reg_value[5]*0.001,\
		mer_reg_value[6]*0.001,mer_reg_value[7]*0.001,mer_reg_value[8]*0.001,mer_reg_value[9]*0.001	,mer_reg_value[10]*0.001,mer_reg_value[11]*0.001);
	//DEBUG_PRINTF("ct:ct=%.2f  ct0=%.2f ct1=%.2f ct2=%.2f ct3=%.2f ct4=%.2f ct5=%.2f ct6=%.2f\r\n",mer_reg_value[21]*0.10,mer_reg_value[22]*0.001,\
		mer_reg_value[23]*0.001,mer_reg_value[24]*0.001,mer_reg_value[25]*0.001,mer_reg_value[26]*0.001	,mer_reg_value[27]*0.001,mer_reg_value[28]*0.001);
	//深度计算
}
/***************************************************************************//**
 * @brief 获取mcp61单通道模块电容值
 * @param 
 * @note 用来确认通信正常
 * @return 返回ID
*******************************************************************************/
uint16_t  app_mcp61_get_singgle_c_value_req(void)
{ 
	uint16_t ret;
	#ifdef LLS_MCP61	
	HAL_StatusTypeDef err=HAL_OK;
	USART1_TX_BUFF[0]=MER_SLAVE_ADD;
	if(mer_run_sta.idle!=0) 
	{
		mer_run_sta.frame_bus_timeOut++;
		if(mer_run_sta.frame_bus_timeOut>2)
		{
			mer_run_sta.idle=0;
			mer_run_sta.frame_bus_timeOut=0;
		}
		return HAL_BUSY;
	}	
	USART1_TX_BUFF[1]=MER_REDA_CODE;
	USART1_TX_BUFF[2]=(MCP61_REG_SINGGLE_C_VALUE>>8)&0xFF;
	USART1_TX_BUFF[3]=MCP61_REG_SINGGLE_C_VALUE&0xFF;		
	USART1_TX_BUFF[4]=0;
	USART1_TX_BUFF[5]=1;  
	USART1_TX_BUFF[6]=crc16_modbus(USART1_TX_BUFF,6)&0xFF;
	USART1_TX_BUFF[7]=(crc16_modbus(USART1_TX_BUFF,6)>>8)&0xFF;
	err = HAL_UART_Transmit(&huart1,USART1_TX_BUFF, 8, 100);	
	if(err==HAL_OK) app_mer_lisen(MCP61_REG_SINGGLE_C_VALUE,8);
	else mer_run_sta.idle=0;	
	#endif
	return ret;
}
#ifdef LLM
//***************************LLM液位计**********************************************/
void LLM_UART_Init(uint32_t BaudRate)
{
	huart1.Instance = UART5;
	huart1.Init.BaudRate = BaudRate;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
	{
		Error_Handler();
	}	  
  	app_mer_lisen(0x0A,LLM_PACKAGE_LENGTH);
}
/***************************************************************************//**
 * @brief 获取数据值
 * @param 
 * @note  该模块1秒间隔返回固定长度格式数据 包含，参考温度(f摄氏度);频率(mHz)；电容值（pF）
 * @return 返回ID
*******************************************************************************/

HAL_StatusTypeDef  app_fresh_llm_data(float *value)
{
	HAL_StatusTypeDef ret=HAL_OK;
	
	if(mer_run_sta.idle==0)
	{
		*value=llm_rec_data.capacitanceValue;//上一次采集值			
		mer_run_sta.frame_bus_timeOut=0;
		app_mer_lisen(LLM_PACKAGE_HEAD,LLM_PACKAGE_LENGTH); 
	}
	else 	
	{	
		mer_run_sta.frame_bus_timeOut++;
		if(mer_run_sta.frame_bus_timeOut>2)	//3s
		{
			mer_run_sta.frame_bus_timeOut=0;
			*value=0;//err
			app_mer_lisen(LLM_PACKAGE_HEAD,LLM_PACKAGE_LENGTH);
		}		
		else *value	= llm_rec_data.capacitanceValue;
	}	
	return ret;
}
/***************************************************************************//**
 * @brief 记录当前值为空载校准值
 * @param 
 * @note  该模块1秒间隔返回固定长度格式数据 包含，参考温度(f摄氏度);频率(mHz)；电容值（pF）
 * @return 返回ID
*******************************************************************************/

void  app_llm_min_calib_value_record(unsigned char min_max ,float value)
{
	uint8_t i;
	float temp;
	if(min_max==1)
	{
		llm_rec_data.min_max_capacitanceValue[0]=value;
		temp=(llm_rec_data.min_max_capacitanceValue[9]-llm_rec_data.min_max_capacitanceValue[0])/9;
        for(i=0;i<10;i++)
		{
			llm_rec_data.min_max_capacitanceValue[i]=llm_rec_data.min_max_capacitanceValue[0]+temp*i;
		}
		
	}
	else if(min_max==2)
	{
		llm_rec_data.min_max_capacitanceValue[9]=value;
		temp=(llm_rec_data.min_max_capacitanceValue[9]-llm_rec_data.min_max_capacitanceValue[0])/9;
        for(i=0;i<10;i++)
		{
			llm_rec_data.min_max_capacitanceValue[i]=llm_rec_data.min_max_capacitanceValue[0]+temp*i;
		}
	}
	else 
	{
		llm_rec_data.capacitanceValue=value;		
	}
}
/***************************************************************************//**
 * @brief 获取治疗水液位深度
 * @param 
 * @note  深度等级百分比;
 * @return 返回深度值 
*******************************************************************************/
unsigned char  app_get_llm_depth(void)
{
	unsigned char depth,i;
	float temp1;
	if(llm_rec_data.min_max_capacitanceValue[0]<llm_rec_data.min_max_capacitanceValue[9])
	{
		depth=0;
	}
	else
	{
		for(i=0;i<9;i++)
		{
			if(llm_rec_data.capacitanceValue>=llm_rec_data.min_max_capacitanceValue[i]&&llm_rec_data.capacitanceValue<llm_rec_data.min_max_capacitanceValue[i+1])
			{
				depth=i*10;
				break;
			}
		}		
		if(llm_rec_data.capacitanceValue<=llm_rec_data.min_max_capacitanceValue[0]) depth=0;
		if(llm_rec_data.capacitanceValue>=llm_rec_data.min_max_capacitanceValue[9]) depth=100;
	} 
  	return depth;
}
#endif
//***************************clm液位计**********************************************/
