/*
 * mer_mcp1081_bsp.c
 *
 *  Created on: Jan 01, 2026
 *      Author: Hql2017
 */
#include "main.h"
#include "mer_mcp1081_bsp.h" 
#include "usart.h"
#include <stdio.h>
#include  "lwrb.h"

//暂定用调试串口hlpuart1


#define MAX_LWRB_MCP61_QUEUE_LEN 64
static unsigned char lwrb_mcp_rx_buff[MAX_LWRB_MCP61_QUEUE_LEN+1]={0};  // 
static unsigned char mcp61_app_data[MAX_LWRB_MCP61_QUEUE_LEN+1]={0};  // 
static unsigned char  uart_mcp_rx_byte;  // 
static lwrb_t  u_mcp_lwrb;//
static unsigned char mcp61_send_buff[MAX_LWRB_MCP61_QUEUE_LEN]={0};  // 

typedef struct {
	unsigned char idle;//0空闲：1正处于应带侦听端口
	unsigned char frame_length;// 侦听长度
	unsigned char frame_err;// 数据报错
	unsigned char frame_bus_timeOut;// 等待超时
}mer_mcp_modbus_status;
static mer_mcp_modbus_status  mcp_run_sta; 

typedef struct {	
   unsigned char addr;  
   unsigned char code;//code
   unsigned char dataLen;//data Len
   unsigned char *data;//data
   unsigned char crcL;// 校验
   unsigned char crcH;// 校验
}__attribute__ ((packed)) MCP61_APP_PACKAGE;//应用数据解析
MCP61_APP_PACKAGE *mcp_app_package;

#ifdef LLS_MCP61
typedef struct {	
	unsigned short int  c_value1000pf;  
	unsigned short int alertFlag;
	unsigned short int vbemv;//温度相关	
 }__attribute__ ((packed)) MCP61_STATUIS;//状态数据
 MCP61_STATUIS mcp_sta;

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
static uint16_t mcp61_crc16_modbus(const  uint8_t *data, uint16_t len)
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
 * @brief app_mcp61_uart_receive
 * @param 
 * @note 液位模块接收数据
 * @return 
*******************************************************************************/
void app_mcp61_uart_receive( void )
{
	lwrb_write(&u_mcp_lwrb, &uart_mcp_rx_byte, 1);//
	HAL_UART_Receive_IT(&hlpuart1,&uart_mcp_rx_byte,1);	
}
  /***************************************************************************//**
 * @brief 液位传感器，从机应答帧处理
 * @param listenReg侦听端口，datalen，侦听数据长度
 * @note   
 * @return 
*******************************************************************************/
void app_mecp61_receive_data_handle(unsigned char code,unsigned char *data,unsigned char len)
{	
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
		#if 1
		regStart=mer_run_sta.idle-MER_REG_SMAMLING_SPD;			
				readDataLen=UART5_RX_BUFF[2];				
				crcValue=UART5_RX_BUFF[readDataLen+3]| (UART5_RX_BUFF[readDataLen+4]<<8);			
				if(crcValue==crc16_modbus(USART1_RX_BUFF, readDataLen+3))
				{ 
					regNum=readDataLen>>1;	
					//DEBUG_PRINTF("regRead:");
					for(uint8_t i=0;i<regNum;i++)
					{
						mer_reg_value[regStart+i]=UART5_RX_BUFF[3+2*i]<<8|UART5_RX_BUFF[4+2*i];						
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
	#else 
	if(code==MER_SINGLE_WRITE_CODE)//code，写回应,固定8字节，只写单个寄存器
	{   
		unsigned short int ack=data[2]<<8|data[3];
		if(mcp_run_sta.idle==(data[0]<<8|data[1])&&ack!=0)
		{
			DEBUG_PRINTF("reg%04x WriteOk\r\n",mcp_run_sta.idle);
			mcp_run_sta.idle=0;
			mcp_run_sta.frame_length = 0;
			mcp_run_sta.frame_bus_timeOut=0;
			mcp_run_sta.frame_err=0;
		}
		else 
		{
			DEBUG_PRINTF("reg%04x Write fail\r\n",mcp_run_sta.idle);
			mcp_run_sta.idle=0;
			mcp_run_sta.frame_length = 0;
			mcp_run_sta.frame_bus_timeOut=0;
			mcp_run_sta.frame_err=1;		
		}
	}
	else //read
	{	
		mcp_sta.c_value1000pf =	data[0]<<8|data[1];
		mcp_sta.alertFlag=data[2]<<8|data[3];
		mcp_sta.vbemv=data[4]<<8|data[5];
		//DEBUG_PRINTF("  mcp61C=%04x =%.3f",mcp61_reg_value ,mcp61_reg_value*0.001);	
		mcp_run_sta.idle=0;
		mcp_run_sta.frame_length =0;		
		mcp_run_sta.frame_bus_timeOut=0;		
	}
		
	#endif
	
}

/***************************************************************************//**
 * @brief app_mcp61_package_check
 * @param 
 * @note  20ms轮询
 * @return 
*******************************************************************************/
unsigned short int app_mcp61_package_check(void) 
{
	static unsigned int peekLen = 8, skipLen = 0;
    unsigned char packLen = 0;
    unsigned int readLen = lwrb_peek(&u_mcp_lwrb, skipLen, mcp61_app_data, peekLen);
    if(readLen>5)
	{	
		#if 0       
		DEBUG_PRINTF("uart5_rec:\r\n");
		for(unsigned int i=0;i<peekLen;i++)
		{
		DEBUG_PRINTF(" %02x",mcp61_app_data[skipLen+i]);
		}
		DEBUG_PRINTF(" Len=%d\r\n",peekLen); 
		#endif	
        while (skipLen < readLen) {
            if (mcp61_app_data[skipLen] == MER_SLAVE_ADD) 
			{
                unsigned char code = mcp61_app_data[skipLen + 1];
                if (code == MER_REDA_CODE) 
				{
                    packLen = mcp61_app_data[skipLen + 2] + 5;
                    if (packLen > readLen) 
					{
                        peekLen = packLen;
                        break;
                    } 
					else 
					{
                        uint16_t crcValue = (mcp61_app_data[skipLen + packLen - 1] << 8) | mcp61_app_data[skipLen + packLen - 2];
                        if (crcValue == mcp61_crc16_modbus(&mcp61_app_data[skipLen], packLen - 2)) {
							peekLen = packLen;
                            app_mecp61_receive_data_handle(code, &mcp61_app_data[skipLen + 3], mcp61_app_data[skipLen + 2]);
                        }
                        skipLen += packLen;
                        break;
                    }
                } 
				else if (code == MER_SINGLE_WRITE_CODE) 
				{
                    packLen = 8;
                    if (packLen > readLen) 
					{
                        peekLen = packLen;
                        break;
                    } 
					else 
					{
                        uint16_t crcValue = (mcp61_app_data[skipLen + packLen - 1] << 8) | mcp61_app_data[skipLen + packLen - 2];
                        if (crcValue == mcp61_crc16_modbus(&mcp61_app_data[skipLen], packLen - 2)) {
							peekLen = packLen;
                            app_mecp61_receive_data_handle(code, &mcp61_app_data[skipLen + 2], 4);
                        }
                        skipLen += packLen;
                        break;
                    }
                }
            }
            skipLen++;
        }
        lwrb_skip(&u_mcp_lwrb, skipLen);
        skipLen = 0;
    }

    return packLen;
}
  /***************************************************************************//**
 * @brief 进入侦听状态
 * @param listenReg,侦听端口，datalen，侦听数据长度
 * @note 
 * @return 
*******************************************************************************/
void app_mcp61_lisen(uint16_t listenReg,uint16_t dataLen)
{	
	mcp_run_sta.idle=listenReg;
	mcp_run_sta.frame_length =dataLen;
	mcp_run_sta.frame_bus_timeOut=0;	
	#ifdef LLS_MCP61
	mcp_run_sta.frame_err=0;// mcp61 用来指示是否校准状态0，正常；1校准空载；2校准满载	
	#endif	
	HAL_UART_Receive_IT(&hlpuart1,&uart_mcp_rx_byte,1);
}
  /***************************************************************************//**
 * @brief 获取总线侦听状态
 * @param 
 * @note 
 * @return 侦听状态，0空闲 !0等待应答帧
*******************************************************************************/
uint16_t app_get_mcp61_lisen_status(void)
{			
	return mcp_run_sta.idle;
}
/***************************************************************************//**
 * @brief 初始化
 * @param 
 * @note 
 * @return 返回操作状态，如果成功等待从机应答
*******************************************************************************/
void app_mcp61_init(void)
{
	lwrb_init(&u_mcp_lwrb,lwrb_mcp_rx_buff,MAX_LWRB_MCP61_QUEUE_LEN);
	HAL_UART_Receive_IT(&hlpuart1,&uart_mcp_rx_byte,1);
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
	ret=mcp_sta.c_value1000pf*0.001;	//还原

	return ret;
}
/***************************************************************************//**
 * @brief 空载校准指令
 * @param  min_max ，无用
 * @note 
 * @return 返回无
*******************************************************************************/
HAL_StatusTypeDef  app_mcp61_calibration(uint8_t min_max)
{
	HAL_StatusTypeDef err=HAL_OK;
	mcp61_send_buff[0]=MER_SLAVE_ADD;
	if(mcp_run_sta.idle!=0) 
	{
		mcp_run_sta.frame_bus_timeOut++;
		if(mcp_run_sta.frame_bus_timeOut>2)//超过3次未回复
		{
			mcp_run_sta.idle=0;
			mcp_run_sta.frame_bus_timeOut=0;
		}
		return HAL_BUSY;
	}
	mcp61_send_buff[1]=MER_SINGLE_WRITE_CODE;
	mcp61_send_buff[2]=(MCP61_REG_CALI_VALUE>>8)&0xFF;
	mcp61_send_buff[3]=MCP61_REG_CALI_VALUE&0xFF;	
	mcp61_send_buff[4]=0;
	mcp61_send_buff[5]=1;
	mcp61_send_buff[6]=mcp61_crc16_modbus(mcp61_send_buff,6)&0xFF;
	mcp61_send_buff[7]=(mcp61_crc16_modbus(mcp61_send_buff,6)>>8)&0xFF;
	err = HAL_UART_Transmit(&hlpuart1,mcp61_send_buff, 8, 100);	
	if(err==HAL_OK) app_mcp61_lisen(MCP61_REG_CALI_VALUE+1,8);
	else mcp_run_sta.idle=0;
	return err;
}

/***************************************************************************//**
 * @brief 设置报警高值
 * @param  max_value，(扩大1000倍)
 * @note 
 * @return 返回无
*******************************************************************************/
HAL_StatusTypeDef  app_mcp61_set_alart_value_high(unsigned short int  high_value)
{
	HAL_StatusTypeDef err=HAL_OK;
	mcp61_send_buff[0]=MER_SLAVE_ADD;
	if(mcp_run_sta.idle!=0) 
	{
		mcp_run_sta.frame_bus_timeOut++;
		if(mcp_run_sta.frame_bus_timeOut>2)//超过3次未回复
		{
			mcp_run_sta.idle=0;
			mcp_run_sta.frame_bus_timeOut=0;
		}
		return HAL_BUSY;
	}
	mcp61_send_buff[1]=MER_SINGLE_WRITE_CODE;
	mcp61_send_buff[2]=(MCP61_REG_ALART_SET_C_VALUE>>8)&0xFF;
	mcp61_send_buff[3]=MCP61_REG_ALART_SET_C_VALUE&0xFF;	
	mcp61_send_buff[4]=(high_value>>8)&0xFF;
	mcp61_send_buff[5]=high_value&0xFF;
	mcp61_send_buff[6]=mcp61_crc16_modbus(mcp61_send_buff,6)&0xFF;
	mcp61_send_buff[7]=(mcp61_crc16_modbus(mcp61_send_buff,6)>>8)&0xFF;
	err = HAL_UART_Transmit(&hlpuart1,mcp61_send_buff, 8, 100);	
	if(err==HAL_OK) app_mcp61_lisen(MCP61_REG_ALART_SET_C_VALUE,8);
	else mcp_run_sta.idle=0;
	return err;
}
/***************************************************************************//**
 * @brief 设置解除报警值
 * @param  min，水位低限(扩大1000倍)
 * @note  >max 水量充足 <min 水量不足
 * @return 返回无
*******************************************************************************/
HAL_StatusTypeDef  app_mcp61_set_alart_value_low(unsigned short int   low_value)
{
	HAL_StatusTypeDef err=HAL_OK;
	mcp61_send_buff[0]=MER_SLAVE_ADD;
	if(mcp_run_sta.idle!=0) 
	{
		mcp_run_sta.frame_bus_timeOut++;
		if(mcp_run_sta.frame_bus_timeOut>2)//超过3次未回复
		{
			mcp_run_sta.idle=0;
			mcp_run_sta.frame_bus_timeOut=0;
		}
		return HAL_BUSY;
	}
	mcp61_send_buff[1]=MER_SINGLE_WRITE_CODE;
	mcp61_send_buff[2]=(MCP61_REG_ALART_CLEAR_C_VALUE>>8)&0xFF;
	mcp61_send_buff[3]=MCP61_REG_ALART_CLEAR_C_VALUE&0xFF;	
	mcp61_send_buff[4]=(low_value>>8)&0xFF;
	mcp61_send_buff[5]=low_value&0xFF;;
	mcp61_send_buff[6]=mcp61_crc16_modbus(mcp61_send_buff,6)&0xFF;
	mcp61_send_buff[7]=(mcp61_crc16_modbus(mcp61_send_buff,6)>>8)&0xFF;
	err = HAL_UART_Transmit(&hlpuart1,mcp61_send_buff, 8, 100);	
	if(err==HAL_OK) app_mcp61_lisen(MCP61_REG_ALART_CLEAR_C_VALUE,8);
	else mcp_run_sta.idle=0;
	return err;
}
#endif
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
	mcp61_send_buff[0]=MER_SLAVE_ADD;
	if(mcp_run_sta.idle!=0) 
	{
		mcp_run_sta.frame_bus_timeOut++;
		if(mcp_run_sta.frame_bus_timeOut>2)
		{
			mcp_run_sta.idle=0;
			mcp_run_sta.frame_bus_timeOut=0;
		}
		//return HAL_BUSY;
	}	
	mcp61_send_buff[1]=MER_REDA_CODE;
	mcp61_send_buff[2]=(MCP61_REG_SINGGLE_C_VALUE>>8)&0xFF;
	mcp61_send_buff[3]=MCP61_REG_SINGGLE_C_VALUE&0xFF;		
	mcp61_send_buff[4]=0;
	mcp61_send_buff[5]=3;  
	mcp61_send_buff[6]=mcp61_crc16_modbus(mcp61_send_buff,6)&0xFF;
	mcp61_send_buff[7]=(mcp61_crc16_modbus(mcp61_send_buff,6)>>8)&0xFF;
	#if 0       
	DEBUG_PRINTF("uart5_send 8:\r\n");
	for(unsigned int i=0;i<8;i++)
	{
		DEBUG_PRINTF(" %02x",mcp61_send_buff[i]);
	}
	DEBUG_PRINTF("\r\n");
	#endif		
	err = HAL_UART_Transmit(&hlpuart1,mcp61_send_buff, 8, 100);	
	if(err==HAL_OK) app_mcp61_lisen(MCP61_REG_SINGGLE_C_VALUE,11);
	else mcp_run_sta.idle = 0;	
	#endif
	return ret;
}
//***************************clm液位计**********************************************/

