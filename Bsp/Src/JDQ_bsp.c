/*
 * JDQ_bsp.c
 *  only write
 *  Created on: Apr 10, 2025
 *      Author: Hql2017
 */
#include "main.h"
#include "JDQ_bsp.h" 
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "stdio.h"
#include "gpio.h"
#include "adc.h"


#define JDQ_DAC_CS_ENABLE  HAL_GPIO_WritePin(DAC_CS_GPIO_Port, DAC_CS_Pin, GPIO_PIN_RESET)
#define JDQ_DAC_CS_DISABLE  HAL_GPIO_WritePin(DAC_CS_GPIO_Port, DAC_CS_Pin, GPIO_PIN_SET)
#define JDQ_LDAC_ENABLE  HAL_GPIO_WritePin(DAC_LD_GPIO_Port, DAC_LD_Pin, GPIO_PIN_RESET)
#define JDQ_LDAC_DISABLE  HAL_GPIO_WritePin(DAC_LD_GPIO_Port, DAC_LD_Pin, GPIO_PIN_SET)

#define HV_PULSE_TIM_RELOAD_VALUE 1000//10ms  100HZ

extern TIM_HandleTypeDef htim3;

//JDQ POWER
#define  STS_1200_OUT_VOLTAGE_LIMIT	220.0f//最大输出220V（MAX220）10mV分辨率
#define  STS_1200_OUT_CURRENT_LIMIT	2.7f//最大输出2.7A（MAX220）
#define  STS_1200_DEVICE_ADDV	0x01//电源通信地址
typedef struct{	
 	unsigned short int frame_regStart;//侦听起始地址
	unsigned short int frame_len;//侦听长度
	unsigned  int frame_timeout;//超时
}JDQ_rs485_status;
static JDQ_rs485_status jdq_rs485_sta;

/*
typedef struct{	
	unsigned short int set_sts_voltage;//设置电压
	unsigned short int set_sts_current;//设置电流
	unsigned short int real_sts_voltage;//读取的实际电压
	unsigned short int real_sts_current;//读取的实际电压
	unsigned short int rerve_1;//保留1
	unsigned short int rerve_2;//保留2
	unsigned short int jdq_run_stop;//电源开关
}JDQ_sts_local_param;
static JDQ_sts_local_param jdq_sts_param;
*/
//0~200mJ =5*40
static float laser_target_energe_list[40]={
	0,0,2.0,2.5,3.0,3.5,4.0,4.5,5.0,5.5,6.0,

};
#ifdef JDQ_PWR_GWB_3200W
 typedef struct {
	unsigned short int outVoltage;
	unsigned short int outCurrent;
	unsigned short int setVoltage;
	unsigned short int setCurrent;
	unsigned char pwr_status;//on/off
	
	unsigned char emergencyErrorFlag;//严重
	unsigned char generationErrorFlag;//一般

}jdq_sts_reg_struct;

static jdq_sts_reg_struct jdq_sts_reg;
#else
static uint16_t jdq_sts_reg_value[8];
#endif
static unsigned char  jdq_rs485_receiv_len,rs485_rec_byte;
static unsigned char UART1_TX_BUFF[MAX_UART1_BUFF_LENTH+1]={0};
static unsigned char UART1_RX_BUFF[MAX_UART1_BUFF_LENTH+1]={0};
#define JDQ_RS485_TX  HAL_GPIO_WritePin(RS485_DIR_out_GPIO_Port, RS485_DIR_out_Pin, GPIO_PIN_SET)
#define JDQ_RS485_RX  HAL_GPIO_WritePin(RS485_DIR_out_GPIO_Port, RS485_DIR_out_Pin, GPIO_PIN_RESET)


static HAL_StatusTypeDef app_jdq_read_req_frame(uint16_t regStart,uint16_t regOffset);
static HAL_StatusTypeDef app_jdq_write_req_frame(uint16_t regStart,uint16_t data);
/****************ADS1110**IO 模拟I2C****0.4M~3.4M************************/
#ifdef ADS1110_JDQ_USED
#if 0
	#include "soft_i2C_bsp.h"
#else
	#include "i2c.h"
	#define TRUE1	1
	#define FALSE1 0
#endif
static unsigned char ads1110_i2C_receive_buff[3];
static unsigned char ads1110_reg_config;
//static unsigned short int ads1110_reg_data;//16bit
/***************************************************************************//**
 * @brief ADS1110_初始化
 * @param 默认参数并启动连续转换
 * @note jdq电源反馈AD，高速模式要多发一个特殊地址00001xxx字节，本次采用400KHZ速度
 * @return 
*******************************************************************************/
void app_jdq_ads1110_init(void)
{
	#if 0
 	soft_I2C_GPIO_Init();
	#else 
	app_I2C_start(&hi2c1);
	#endif
	ads1110_reg_config=0x8C;
	ads1110_reg_config=ADS1110_CONFIG_15SPS|ADS1110_CONFIG_CONTINUS_SAMPLING|ADS1110_CONFIG_1PGA|ADS1110_CONFIG_START_SAMP;
	//ads1110_reg_data=0;
}

/***************************************************************************//**
 * @brief ADS1110_读数据
 * @param 
 * @note jdq电源反馈AD
 * @return 
*******************************************************************************/
/***************************************************************************************************************/
uint8_t ads1110_read( uint8_t *data)
{
	//if(soft_I2C_ReadBuffer(data,3,ADS1110_I2C_SLAVE_ADDR)==TRUE1)
	if(HAL_I2C_Master_Receive(&hi2c1,ADS1110_I2C_SLAVE_ADDR|0x01,data,3,100)==HAL_OK)
	{
		ads1110_reg_config=data[2];
		//ads1110_reg_data=(data[0]<<8)|data[1];
		return TRUE1;
	}
	return FALSE1; 
}
/***************************************************************************************************************/
uint8_t ads1110_write( uint8_t data)
{
	//if( soft_I2C_WriteByte(ADS1110_I2C_SLAVE_ADDR, data)==TRUE1)
	if(HAL_I2C_Master_Transmit(&hi2c1,ADS1110_I2C_SLAVE_ADDR,&data,1,100)==HAL_OK)
	{
		ads1110_reg_config = data;
		return TRUE1;
	}
	return FALSE1;
}
/***************************************************************************************************************/
uint8_t ads1110_available(void)//读取转换状态
{
  uint8_t data[3];   
  //if(soft_I2C_ReadBuffer(data,3,ADS1110_I2C_SLAVE_ADDR)==TRUE1)
  if(ads1110_read(data)==TRUE1)
  {
    if((data[2]&ADS1110_CONFIG_START_SAMP) == 0)
    return TRUE1;
  }
  return TRUE1;
}
/***************************************************************************************************************/
uint8_t ads1110_read_raw( int16_t *adc_raw)
{
  uint8_t data[3];    
  if(ads1110_read(data)==TRUE1)
  {
    *adc_raw = (data[0] << 8)|data[1];    
    return TRUE1;    
  }
  return FALSE1; 
}
/***************************************************************************************************************/
uint8_t ads1110_read_mv( float *adc_mv)
{
  int16_t adc_raw;
  float res_div;
  float gain;
  unsigned char tempValue;
  if (ads1110_read_raw(&adc_raw) == FALSE1)
    return FALSE1; 
	tempValue=ads1110_reg_config&ADS1110_CONFIG_8PGA;//PGA
  if (tempValue == ADS1110_CONFIG_1PGA)
    gain = 1.0f;
  else if (tempValue== ADS1110_CONFIG_2PGA)
    gain = 2.0f;
  else if (tempValue == ADS1110_CONFIG_4PGA)
    gain = 4.0f;
  else
    gain = 8.0f; 

	tempValue=ads1110_reg_config&ADS1110_CONFIG_15SPS;//SPS
  if (tempValue == ADS1110_CONFIG_15SPS)
    res_div = 32768.0f;
  else if (tempValue == ADS1110_CONFIG_30SPS)
    res_div = 16384.0f;
  else if (tempValue == ADS1110_CONFIG_60SPS)
    res_div = 8192.0f;
  else
    res_div = 2048.0f;    
  *adc_mv = ((float)adc_raw * (ADS1110_REF / res_div)) / gain;
  return TRUE1;
}
#endif
 

/**
 * @brief jdq_reley_charge 
 * @param  void
 * @note   init,常闭
 * @retval None
 */
void jdq_reley_charge(unsigned char onOff)
{
	if(onOff==0) HAL_GPIO_WritePin(JDQ_STAND_GPIO_Port, JDQ_STAND_Pin, GPIO_PIN_RESET);
	else HAL_GPIO_WritePin(JDQ_STAND_GPIO_Port, JDQ_STAND_Pin, GPIO_PIN_SET); 
}
/**
 * @brief jdq_reley_charge_ready 
 * @param  void
 * @note   init，常闭
 * 
 * @retval None
 */
void jdq_reley_charge_ready(unsigned char onOff)
{
	if(onOff==0) HAL_GPIO_WritePin(JDQ_READY_GPIO_Port, JDQ_READY_Pin, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(JDQ_READY_GPIO_Port, JDQ_READY_Pin, GPIO_PIN_RESET); 
}

/**
  * @brief jdq_init 
  * @param  voidvoid
  * @note   init
  * @retval None
  */ 
void jdq_init(void)
{
	//spi only send
	app_jdq_ads1110_init();//电压反馈	
	JDQ_LDAC_DISABLE; 	
	jdq_reley_charge_ready(0);//负载断开
	HAL_Delay(500);		
	jdq_reley_charge(1);//	
	HAL_Delay(500);	
	JDQ_RS485_RX;
}
//***************************AD5541ABRMZ********����ʽdac***************************************//
#define  AD5541_VREF   4.096f//2.5//2.5V
/***************************************************************************//**
 * @brief ͨspi发送寄存器设置值
 * 

 *
 * @param registerValue - 设置值
 *
 * @return 无
*******************************************************************************/
void AD5541A_SetRegisterValue(unsigned short registerValue)
{
	HAL_StatusTypeDef err;
	unsigned char TXdata[3];
	unsigned short int dLength;	
	TXdata[0] = (registerValue>>8)&0xFF; //high
	TXdata[1] = registerValue&0xFF;  
	dLength=2;
	JDQ_DAC_CS_ENABLE;
	delay_us(1);
	err = HAL_SPI_Transmit(&hspi1, TXdata, dLength, 100);//HAL_MAX_DELAY);
	JDQ_DAC_CS_DISABLE; 
	delay_us(1);
}
/***************************************************************************//**
 * @brief 设置输出电压
 *
 * @param outVoltage - 输出电压
 * @param vRef - 参考电压4.096V
 *
 * @return 返回设置电压值
*******************************************************************************/
float AD5541A_SetVoltage(float outVoltage, float vRef)
{
    unsigned short registerValue = 0;
    float          actualVoltage = 0;
    float          code          = 0;    
    if (vRef == 0)
    {
    	vRef = 4.096;//2.5;
    }		
    /* Get raw data from the user's desired voltage value. */
    code = (outVoltage / vRef) * 65536;
    /* Round to the nearest integer. */
    registerValue = (unsigned short)(code + 0.5);
		
		
    /* Write to DAC register. */
    AD5541A_SetRegisterValue(registerValue);
    /* Calculate the voltage value that can be outtputed by the device. */
    actualVoltage = vRef * ((float)registerValue / 65536);
	//**application new voltage
	JDQ_LDAC_ENABLE;
	delay_us(1);
	JDQ_LDAC_DISABLE;    
   return actualVoltage;
}
/***************************************************************************//**
 * @brief 设置输出电压,不更新
 *
 * @param outVoltage - 输出电压
 * @param vRef - 参考电压4.096V
 *
 * @return 
*******************************************************************************/

void AD5541A_SetVoltage_noLoad(float outVoltage, float vRef)
{	
    unsigned short registerValue = 0;
    float          actualVoltage = 0;
    float          code          = 0;   
	JDQ_LDAC_DISABLE;
    if (vRef == 0)
    {
    	vRef = 4.096;//2.5;
    }		
    /* Get raw data from the user's desired voltage value. */
    code = (outVoltage / vRef) * 65536;
    /* Round to the nearest integer. */
    registerValue = (unsigned short)(code + 0.5);
		
		
    /* Write to DAC register. */
    AD5541A_SetRegisterValue(registerValue);
    /* Calculate the voltage value that can be outtputed by the device. */
    actualVoltage = vRef * ((float)registerValue / 65536);
	//**application new voltage
	
}
/***************************************************************************//**
 * @brief 使能设定值
 * @param 
 * @param vRef - 参考电压4.096V
 *
 * @return 
*******************************************************************************/
void AD5541A_SetVoltage_Load_enable(void)
{
	JDQ_LDAC_ENABLE;
}
void AD5541A_SetVoltage_Load_disable(void)
{
	JDQ_LDAC_DISABLE;
}
//***************************AD5541ABRMZ********dac*********end******************************//

#ifdef JDQ_PWR_GWB_3200W
//******GWB3200-14420系列*，通过485读取**9600波特率*********//
/***************************************************************************//**
 * @brief 通讯包校验
 * @param 16进制数
 * @note 
 * @return 
*******************************************************************************/
uint16_t DEC2BCD(uint16_t ui16hexcode)
{
	uint16_t ui16bcd, ui16temp1, ui16temp2;
	if (ui16hexcode > 9999)
	{
		return (NULL);
	}
	ui16temp1 = ui16hexcode / 1000; //取千位数
	ui16bcd = ui16temp1 << 12; //左移 12 位
	ui16temp1 = (ui16hexcode % 1000) / 100; //取百位数
	ui16temp2 = ui16temp1 << 8; //左移 8 位
	ui16bcd = ui16bcd | ui16temp2; //将百位数并入 BCD 码
	ui16temp1 = (ui16hexcode % 100) / 10; //取十位数
	ui16temp2 = ui16temp1 << 4; //左移 4 位
	ui16bcd = ui16bcd | ui16temp2; //将百位数并入 BCD 码
	ui16temp1 = ui16hexcode % 10; //取个位数
	ui16bcd = ui16bcd | ui16temp1; //两个数合并为一个 BCD 码
	return (ui16bcd);
}

/***************************************************************************//**
 * @brief 侦听数据处理
 * @param listenReg,侦听缓存，datalen，侦听数据长度
 * @note 
 * @return 
*******************************************************************************/
void app_jdq_gwb3200_receive_handle(unsigned char *buff,unsigned char Len)
{ 
	uint16_t crc,datalen,i;
	uint8_t regS;	
	regS=buff[3]&0x7F;
	switch(regS)
	{
		case GWB_3200_REG_VOLTAGE_CURRENT_DISPLAY:		
			jdq_sts_reg.outVoltage=(buff[5]<<8)|buff[6];
			if(jdq_sts_reg.outVoltage!=0&&jdq_sts_reg.setVoltage!=0)
			{
				jdq_sts_reg.pwr_status=1;
			}
			else jdq_sts_reg.pwr_status=0;
			jdq_sts_reg.outCurrent=(buff[7]<<8)|buff[8];
			jdq_sts_reg.emergencyErrorFlag=buff[14];
			jdq_sts_reg.generationErrorFlag=buff[15];			
			break;
			case GWB_3200_REG_RUN_STOP:
			if(buff[4]==0x00)
			{
				jdq_sts_reg.pwr_status=1;
			}
			else jdq_sts_reg.pwr_status=0;		
		break;
		case GWB_3200_REG_SET_VOLTAGE_CURRENT  :
			jdq_sts_reg.setVoltage=(buff[4]<<8)|buff[5];
			jdq_sts_reg.setCurrent=(buff[6]<<8)|buff[7];
		break;
		case GWB_3200_REG_READ_ALL_PARAM:

		break;
		case GWB_3200_REG_READ_TMPEATURE:
		break;
		default:
		break;
	}
		
}
/***************************************************************************//**
 * @brief 数据包检查
 * @param 
 * @note  最小包长8字节
 * @return 
*******************************************************************************/
unsigned short int app_rs485_package_check(unsigned char* pBuff,unsigned short int buffLen) 
{
	unsigned short int retLen=0,i=0,j=0,pLen,decBcd;  
	unsigned short int sum;
	while(i<buffLen)
	{   
		if(pBuff[i]==0x7E)
		{
			pLen=(pBuff[i+2]>>4)*10+(pBuff[i+2]&0x0F);						      
			if(i+pLen+5>buffLen) 
			{
				retLen = i;//end
				break;
			} 
			else    
			{ 
				if(pBuff[i+pLen+4]==0x0D)
				{
					sum=0;
					for(j=1;j<pLen+3;j++)
					{
						sum += pBuff[i+j];
					}
					decBcd = pBuff[i+pLen+3]; 					  
					if(decBcd == DEC2BCD((sum&0x00FF)%100))
					{						
						app_jdq_gwb3200_receive_handle(&pBuff[i],pLen+5);
						retLen = pLen+i+5; 
						i=retLen;    
						jdq_rs485_sta.frame_regStart=0;
						jdq_rs485_sta.frame_len=0;
					} 

				}				
			} 
    	}
		i++; 
	}
  	if(i==buffLen) retLen=i;
	return retLen;
}

/***************************************************************************//**
 * @brief check receiveBuff
 * @param 
 * @note 
 * @return 
*******************************************************************************/ 
unsigned char  app_jdq_rs485_check_gwb3200_rec_package(void)
{
	unsigned char i=0,packgeLen=0;
	static unsigned char readLen;
	if(readLen<jdq_rs485_receiv_len) 
    {
		packgeLen = jdq_rs485_receiv_len-readLen;      
      #if 0   
	  if(packgeLen>5) 
	  {    
		DEBUG_PRINTF("rs485_receive_pack:\r\n");
		for(int i=0;i<packgeLen;i++)
		{
			DEBUG_PRINTF(" %02x",UART1_RX_BUFF[i+readLen]);
		}
		DEBUG_PRINTF(" Len=%d\r\n",packgeLen);    
		}  
      #endif   
      packgeLen = app_rs485_package_check(&UART1_RX_BUFF[readLen],packgeLen);	 
      if(packgeLen!=0)
      {
        readLen+=packgeLen;		
      }   
    }
    if(readLen>=jdq_rs485_receiv_len&&readLen!=0)//full
    {
      readLen=0;
      jdq_rs485_receiv_len=0;
    }  
	return  packgeLen;
}
#else 
//******CA-IS1306M25G**5MHz~24MHZ，通过485读取***********//
/***************************************************************************//**
 * @brief MODBUS/CRC-16
 * @param *data缓存，长度len
 * @note 多项式0xA001 
 * @return 返回crc16结果
*******************************************************************************/
uint16_t jdq_crc16_modbus(const  uint8_t *data, uint16_t len)
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
 * @brief 侦听数据处理
 * @param listenReg,侦听缓存，datalen，侦听数据长度
 * @note 
 * @return 
*******************************************************************************/
void app_jdq_sts_1200_receive_handle(void)
{
	uint16_t crc,datalen,i,regS;
	if(jdq_rs485_receiv_len>=jdq_rs485_sta.frame_len&&jdq_rs485_receiv_len!=0)	
	{
		#if  0
		printf("485_rec=%d",jdq_rs485_receiv_len);
		for(int i=0;i<jdq_rs485_receiv_len;i++)
		{
			printf(" %02x",UART1_RX_BUFF[i]);		
		}
		printf("\r\n");
		#endif	
		if(UART1_RX_BUFF[1]==0x03)
		{
			datalen=UART1_RX_BUFF[2];
			crc=UART1_RX_BUFF[3+datalen]|(UART1_RX_BUFF[4+datalen]<<8);		
			if(crc==jdq_crc16_modbus(UART1_RX_BUFF,datalen+3))//
			{
				if(jdq_rs485_sta.frame_regStart>=STS_1200_REG_SET_VOLTAGE&&jdq_rs485_sta.frame_regStart<=STS_1200_REG_RUN_STOP)
				{					
					regS=jdq_rs485_sta.frame_regStart-STS_1200_REG_SET_VOLTAGE;
					for(i=0;i<(datalen>>1);i++)
					{
						jdq_sts_reg_value[i+regS]=(UART1_RX_BUFF[3+i*2]<<8)|UART1_RX_BUFF[4+i*2];
					}
				}	
			} 		
		}
		else if(UART1_RX_BUFF[1]==0x06)//写单个寄存器
		{     
			regS=(UART1_RX_BUFF[2]<<8)|UART1_RX_BUFF[3];
			if(regS==jdq_rs485_sta.frame_regStart)
			{
				crc=UART1_RX_BUFF[6]|(UART1_RX_BUFF[7]<<8);		
				if(crc==jdq_crc16_modbus(UART1_RX_BUFF,6))//
				{
					jdq_sts_reg_value[regS-STS_1200_REG_SET_VOLTAGE]=(UART1_RX_BUFF[4]<<8)|UART1_RX_BUFF[5];	
				}          
			}
		}
		else if(UART1_RX_BUFF[1]==0x10)//写多个寄存器
		{
			regS=(UART1_RX_BUFF[2]<<8)|UART1_RX_BUFF[3];
			datalen=(UART1_RX_BUFF[4]<<8)|UART1_RX_BUFF[5];//offeset
			if(regS==jdq_rs485_sta.frame_regStart)
			{
				crc=UART1_RX_BUFF[6+datalen*2]|(UART1_RX_BUFF[7+datalen*2]<<8);		
				if(crc==jdq_crc16_modbus(UART1_RX_BUFF,6))//
				{
					for(i=0;i<(datalen);i++)
					{
						//jdq_sts_reg_value[i+regS]=(UART1_RX_BUFF[6+i*2]<<8)|UART1_RX_BUFF[7+i*2];
					}					
				}          
			}
		}
		jdq_rs485_sta.frame_len=0;
		jdq_rs485_sta.frame_regStart=0;
		jdq_rs485_sta.frame_timeout=0;
		jdq_rs485_receiv_len=0;
	}	
}
  /************************************************************************//**
  * @brief 设置总电源电流值
  * @param powerCurrent电流值 
  * @note   0.01A分辨率
  * @retval 
  *****************************************************************************/
 void app_jdq_bus_current_set(float  powerCurrent)
 {
	HAL_StatusTypeDef err;	
	unsigned short int sendBuff;
	if(powerCurrent<1.0) sendBuff=100;
	else if(powerCurrent>2.7) sendBuff=270; //max2.7A
	else sendBuff=(int)(powerCurrent*100);
	err= app_jdq_write_req_frame(STS_1200_REG_SET_CURRENT,(uint16_t)sendBuff);	
 }
  /************************************************************************//**
  * @brief 设置总电源电压值
  * @param powerCurrent电流值
  * @note   0.01V分辨率
  * @retval 
  *****************************************************************************/
 void app_jdq_bus_voltage_set(float  voltage)
 {
	HAL_StatusTypeDef err;	
	int sendBuff;
	if(voltage<12.0) sendBuff=1200;
	else if(voltage>220.0) sendBuff=22000; 
	else sendBuff=(int)(voltage*100);//100倍
	err= app_jdq_write_req_frame(STS_1200_REG_SET_VOLTAGE,(uint16_t)sendBuff);		
 }
 
 /***************************************************************************//**
 * @brief 写参数
 * @param regStart 寄存器起始；
 * @note  只允许写单个寄存器
 * @return 返回操作状态，如果成功等待从机应答
*******************************************************************************/
HAL_StatusTypeDef app_jdq_write_req_frame(uint16_t regStart,uint16_t data)
{ 
	HAL_StatusTypeDef err=HAL_OK;
	UART1_TX_BUFF[0]=STS_1200_DEVICE_ADDV;
	UART1_TX_BUFF[1]=0x06;
	UART1_TX_BUFF[2]=(regStart>>8)&0xFF;
	UART1_TX_BUFF[3]=regStart&0xFF;		
	UART1_TX_BUFF[4]=(data>>8)&0xFF;
	UART1_TX_BUFF[5]=data&0xFF;  
	UART1_TX_BUFF[6]=jdq_crc16_modbus(UART1_TX_BUFF,6)&0xFF;
	UART1_TX_BUFF[7]=(jdq_crc16_modbus(UART1_TX_BUFF,6)>>8)&0xFF;
	JDQ_RS485_TX;
	err = HAL_UART_Transmit(&huart1,UART1_TX_BUFF, 8, 100);	
	JDQ_RS485_RX;		
	if(err==HAL_OK) app_jdq_lisen(regStart,8);
	else jdq_rs485_sta.frame_regStart=0;	
	return err;
}
/***************************************************************************//**
 * @brief 问询帧
 * @param regStart 寄存器起始；regOffset寄存器偏移量>=1
 * @note 
 * @return 返回操作状态，如果成功等待从机应答
*******************************************************************************/
HAL_StatusTypeDef app_jdq_read_req_frame(uint16_t regStart,uint16_t regOffset)
{ 
	HAL_StatusTypeDef err=HAL_OK;	
	if(jdq_rs485_sta.frame_regStart!=0)
	{
		jdq_rs485_sta.frame_timeout++;
		if(jdq_rs485_sta.frame_timeout>2) 
		{
			jdq_rs485_sta.frame_regStart=0;
			jdq_rs485_sta.frame_timeout =0;
		}
		return HAL_BUSY;
	}	
	UART1_TX_BUFF[0]=STS_1200_DEVICE_ADDV;  
	UART1_TX_BUFF[1]=0x03;
	UART1_TX_BUFF[2]=(regStart>>8)&0xFF;
	UART1_TX_BUFF[3]=regStart&0xFF;	
	UART1_TX_BUFF[4]=(regOffset>>8)&0xFF;
	UART1_TX_BUFF[5]=regOffset&0xFF;
	UART1_TX_BUFF[6]=jdq_crc16_modbus(UART1_TX_BUFF,6)&0xFF;
	UART1_TX_BUFF[7]=(jdq_crc16_modbus(UART1_TX_BUFF,6)>>8)&0xFF;
	JDQ_RS485_TX;
	err = HAL_UART_Transmit(&huart1,UART1_TX_BUFF, 8, 100);	
	JDQ_RS485_RX;	
	if(err==HAL_OK) app_jdq_lisen(regStart,regOffset*2+5);
	else jdq_rs485_sta.frame_regStart=0;
	return err;
} 
#endif
/***************************************************************************//**
 * @brief 进入侦听状态
 * @param listenReg,侦听端口，datalen，侦听数据长度
 * @note 
 * @return 
*******************************************************************************/
void app_jdq_lisen(uint16_t listenReg,uint16_t dataLen)
{
	#ifdef JDQ_PWR_GWB_3200W
	jdq_rs485_sta.frame_regStart=listenReg|0x80;
	#else
	jdq_rs485_receiv_len=0;
	jdq_rs485_sta.frame_regStart=listenReg;
	#endif
	JDQ_RS485_RX;
	jdq_rs485_sta.frame_len=dataLen;	
	HAL_UART_Receive_IT(&huart1,&rs485_rec_byte, 1);//切换进入侦听状态
}

/************************************************************************//**
* @brief 设置总电源电压、电流
* @param   powerVolotage 电压值   
*          powerCurrent 电流值
* @note   
* @retval 
*****************************************************************************/
void app_jdq_bus_vol_current_set(float powerVolotage,float  powerCurrent)
{
   HAL_StatusTypeDef err;		
#ifdef JDQ_PWR_GWB_3200W
   unsigned short int u16Checksum=0;
   UART1_TX_BUFF[0]=0x7E;
   UART1_TX_BUFF[1]=DEC2BCD(0x01);
   UART1_TX_BUFF[2]=DEC2BCD(0x05);
   UART1_TX_BUFF[3]=GWB_3200_REG_SET_VOLTAGE_CURRENT;
   UART1_TX_BUFF[4]=((uint16_t)powerVolotage*100)>>8;	
   UART1_TX_BUFF[5]=((uint16_t)powerVolotage*100)&0xFF;
   UART1_TX_BUFF[6]=((uint16_t)powerCurrent*100)>>8;	
   UART1_TX_BUFF[7]=((uint16_t) powerCurrent*100)&0xFF; 
   for(uint8_t i=1; i<8; i++)
   {
	   u16Checksum += UART1_TX_BUFF[i];
   }
   UART1_TX_BUFF[8] = DEC2BCD((u16Checksum & 0x00FF) % 100);//校验码
   UART1_TX_BUFF[9] = 0x0D;
   JDQ_RS485_TX;
   err = HAL_UART_Transmit(&huart1,UART1_TX_BUFF, 10, 100);
   if(err==HAL_OK) app_jdq_lisen(GWB_3200_REG_SET_VOLTAGE_CURRENT,11);	
   else jdq_rs485_sta.frame_regStart=0;
#else
   unsigned short int regStart;
   int volBuff,currentBuff;
   if(powerVolotage<12.0) volBuff=1200;
   else if(powerVolotage>STS_1200_OUT_VOLTAGE_LIMIT) volBuff=(int)(STS_1200_OUT_VOLTAGE_LIMIT*100);
   else volBuff=(int) (powerVolotage*100);

   if(powerCurrent<1.0) currentBuff=0;
   else if(powerCurrent>STS_1200_OUT_CURRENT_LIMIT) currentBuff=(int)(STS_1200_OUT_CURRENT_LIMIT*100);
   else currentBuff=(int) (powerCurrent*100);	
   if(jdq_rs485_sta.frame_regStart!=0) 
   {
	   jdq_rs485_sta.frame_timeout++;
	   if(jdq_rs485_sta.frame_timeout>2)
	   {
		   jdq_rs485_sta.frame_regStart=0;
		   jdq_rs485_sta.frame_timeout=0;
	   }
   }
   UART1_TX_BUFF[0]=STS_1200_DEVICE_ADDV;
   regStart=STS_1200_REG_SET_VOLTAGE;
   UART1_TX_BUFF[1]=0x10;//写多路
   UART1_TX_BUFF[2]=(regStart>>8)&0xFF;
   UART1_TX_BUFF[3]=regStart&0xFF;	
   UART1_TX_BUFF[4]=0;	//lenH
   UART1_TX_BUFF[5]=2;	//lenL	
   UART1_TX_BUFF[6]=(((uint16_t)volBuff)>>8)&0xFF; 
   UART1_TX_BUFF[7]=((uint16_t)volBuff)&0xFF; 
   UART1_TX_BUFF[8]=(((uint16_t)currentBuff)>>8)&0xFF;
   UART1_TX_BUFF[9]=((uint16_t)currentBuff)&0xFF; 
   UART1_TX_BUFF[10]=jdq_crc16_modbus(UART1_TX_BUFF,10)&0xFF;
   UART1_TX_BUFF[11]=(jdq_crc16_modbus(UART1_TX_BUFF,10)>>8)&0xFF;
   JDQ_RS485_TX;
   err = HAL_UART_Transmit(&huart1,UART1_TX_BUFF, 12, 100);		
   if(err==HAL_OK) app_jdq_lisen(regStart,8);
   else jdq_rs485_sta.frame_regStart=0;
#endif
}
  /************************************************************************//**
  * @brief 读电源输出状态
  * @param *sta  状态值指针
  * @note   
  * @retval 
  *****************************************************************************/
 unsigned short int  app_jdq_get_vbus_sta(void)
 {		
	#ifdef JDQ_PWR_GWB_3200W	
	//if(jdq_sts_reg.emergencyErrorFlag!=0)	
	//{//严重错误
		//	return JDQ_PWR_GWB_3200W_ERROR_FLAG;//err
	//}
	if(jdq_sts_reg.outVoltage+10>jdq_sts_reg.setVoltage&&jdq_sts_reg.outVoltage+10<jdq_sts_reg.setVoltage)
	{
		return jdq_sts_reg.setVoltage;
	}
	else 
	{
		return jdq_sts_reg.outVoltage;  //err
	}
	#else 
	return jdq_sts_reg_value[6];
	#endif	
 }
   /************************************************************************//**
  * @brief 读总电源电压电流值请求
  * @param powerCurrent电流值
  * @note   
  * @retval 
  *****************************************************************************/
 void app_jdq_bus_get_v_c_req(void)
 {
	HAL_StatusTypeDef err;	
	#ifdef JDQ_PWR_GWB_3200W
	unsigned short int u16Checksum=0;
	UART1_TX_BUFF[0]=0x7E;
	UART1_TX_BUFF[1]=DEC2BCD(0x01);
	UART1_TX_BUFF[2]=DEC2BCD(0x01);
	UART1_TX_BUFF[3]=GWB_3200_REG_VOLTAGE_CURRENT_DISPLAY;	
	for(uint8_t i=1; i<4; i++)
	{
		u16Checksum += UART1_TX_BUFF[i];
	}
	UART1_TX_BUFF[4] = DEC2BCD((u16Checksum & 0x00FF) % 100);//校验码
	UART1_TX_BUFF[5] = 0x0D;
	JDQ_RS485_TX;
	err = HAL_UART_Transmit(&huart1,UART1_TX_BUFF, 6, 100);
	
	if(err==HAL_OK) app_jdq_lisen(GWB_3200_REG_SET_VOLTAGE_CURRENT,17);	
	else jdq_rs485_sta.frame_regStart=0;
	#else
		HAL_StatusTypeDef err;	
		err = app_jdq_read_req_frame(STS_1200_REG_SET_VOLTAGE,4);
	#endif
	
 }
   /************************************************************************//**
  * @brief 读总电源电压输出状态请求
  * @param 
  * @note   
  * @retval 
  *****************************************************************************/
 void app_jdq_bus_power_onoff_sta_req(void)
 {
	#ifdef JDQ_PWR_GWB_3200W	
	app_jdq_bus_get_v_c_req();
	#else	
	HAL_StatusTypeDef err;		
	err= app_jdq_read_req_frame(STS_1200_REG_RUN_STOP,1);
	#endif
 }
   /************************************************************************//**
  * @brief 读总电源电流设定值
  * @param  *voltage 电压设定, *current 电流设定
  * @note   
  * @retval 
  *****************************************************************************/
 void app_jdq_bus_get_set_v_c(float *voltage,float *current)
 {	
	#ifdef JDQ_PWR_GWB_3200W
	*voltage=jdq_sts_reg.setVoltage*0.01;
	*current=jdq_sts_reg.setCurrent*0.01;
	#else
	*voltage=jdq_sts_reg_value[0]*0.01;
	*current=jdq_sts_reg_value[1]*0.01;
	#endif
 }
  /************************************************************************//**
  * @brief 开启电源输出
  * @param 0；关闭 1：开启
  * @note   
  * @retval 
  *****************************************************************************/
 void app_jdq_bus_power_on_off(unsigned char flag)
 {
	HAL_StatusTypeDef err;
	#ifdef JDQ_PWR_GWB_3200W
	unsigned short int u16Checksum=0;
	UART1_TX_BUFF[0]=0x7E;
	UART1_TX_BUFF[1]=DEC2BCD(0x01);
	UART1_TX_BUFF[2]=DEC2BCD(0x03);
	UART1_TX_BUFF[3]=GWB_3200_REG_RUN_STOP;	
	if(flag==0)
	{
		UART1_TX_BUFF[4]=0x01; //close
		UART1_TX_BUFF[5]=0x00; 
	}
	else
	{
		UART1_TX_BUFF[4]=0x00; //open
		UART1_TX_BUFF[5]=0x00;
	}
	for(uint8_t i=1; i<6; i++)
	{
		u16Checksum += UART1_TX_BUFF[i];
	}
	UART1_TX_BUFF[6] = DEC2BCD((u16Checksum & 0x00FF) % 100);//校验码
	UART1_TX_BUFF[7] = 0x0D;
	JDQ_RS485_TX;
	err = HAL_UART_Transmit(&huart1,UART1_TX_BUFF, 8, 100);
	if(err==HAL_OK) app_jdq_lisen(GWB_3200_REG_RUN_STOP,7);	
	else jdq_rs485_sta.frame_regStart=0;
#else		
	if(flag==0)
	{
		err = app_jdq_write_req_frame(STS_1200_REG_RUN_STOP,0);
	}
	else //if ()
	{
		err = app_jdq_write_req_frame(STS_1200_REG_RUN_STOP,1);
	}
	#endif		
 }
//JDQ 继电器 逻辑 (LL HL  LH LL) (stand ,ready)
//***************************laser pulse (100us~500us) timer3***************************************//
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance ==TIM2)
	{
		if(htim->Channel ==	HAL_TIM_ACTIVE_CHANNEL_1)
		{  
			HAL_GPIO_WritePin(GPIOC, HV_ONE_PULSE_out_Pin, GPIO_PIN_SET); 		
		 	pulse_adc_start(MAX_AD2_ENERGE_BUFF_LENGTH);
		}
	}
}
  /***************************************************************************//**
 * @brief app_laser_pulse_width_set
 * @param 
 * @note  100~200us
 * @return 
*******************************************************************************/
//(1.5V~4.0V)(0.1us)
//static unsigned short int  jdq_pulse_cali_time01Us[26]={\
	218,222,226,230,234,238,252,256,250,254,\
	258,262,266,270,274,278,282,286,290,\
	294,298,302,306,310,314,320,\
};
static unsigned short int  jdq_pulse_cali_time01Us[27]={//(1.4V~4.0V)(0.1us)//T+9
	196,200,204,208,212, 216,220,224,228,\
	232,236,240,\
	244,248,252,256,260,264,268,\
	272,276,280,284,288,292,296,300//+8
};
void app_laser_pulse_width_set(unsigned short int pulse100ns,float energeVoltage)
{	  
	unsigned short int num100ns;	
	unsigned short int lowVoltage100ns;	
	//num100ns=(unsigned short int)(((energeVoltage-1.4)*4+19.6-9)/2)*10+65;
	//num100ns=(unsigned short int)((energeVoltage*4+14-9)*5+65);
	if(energeVoltage<2.20)
	{
		lowVoltage100ns=(2.2-energeVoltage)*450;//1.85V  15us
	}
	else lowVoltage100ns=0;
	if(energeVoltage<DAC_MIN_VOLTAGE_F) num100ns=158;
	else if(energeVoltage>DAC_MAX_VOLTAGE_F) num100ns = 210;
	else num100ns=(unsigned short int)(energeVoltage*20)+130;
	//100~200us	 
	if(pulse100ns<800) __HAL_TIM_SetAutoreload(&htim3,799-num100ns-lowVoltage100ns);//100us	
	else if(pulse100ns>2000) __HAL_TIM_SetAutoreload(&htim3,1999-num100ns-lowVoltage100ns);//200us
	else __HAL_TIM_SetAutoreload(&htim3,pulse100ns-1-num100ns-lowVoltage100ns);  
}
/**
  * @brief app_laser_pulse_start 
  * @param  time100ns=0.1us;
  * @note  timeUs: laser pulse  100us~200us ；frq:pulse frequency; energeVoltage:DAC 1.5V~4.0V
  * @retval None
  */
//static unsigned short int  jdq_pulse_pro_timeUs[27]={//(1.4,1.5V~4.0V)  
//	350,275,200,160,150,130,118,108,99,90,\
//	84,80, 76,72,68,64,62,60,58,\
	//56,54,52,50,48,46,45,44
//};
static unsigned short int  jdq_pulse_pro_timeUs[27]={//(1.4,1.5V~4.0V)  
	350,269,204,169,150,138,128,106,93,84,\
	73,71, 69,67,65,64,62,60,58,\
	56,54,52,50,48,46,45,44
};
 void app_laser_pulse_start(unsigned short int timeUs,unsigned short int freq,float energeVoltage)
 { 
	unsigned  int counter;	
	unsigned short int timeLoad;
	if(timeUs!=0&&freq!=0)	
	{
		//check param
		float ev=energeVoltage;
		uint16_t timeus;
		uint16_t timeus2;		
		if(ev>DAC_MAX_VOLTAGE_F) ev=DAC_MAX_VOLTAGE_F;
		if(ev<DAC_MIN_VOLTAGE_F) ev=DAC_MIN_VOLTAGE_F;
		if(energeVoltage<1.5)
		{
			timeus2=1200*(1.617-energeVoltage);
		}
		else if(energeVoltage<1.55)
		{
			timeus2=500*(1.876-energeVoltage)+13;
		}
		else if(energeVoltage<1.6)
		{
			timeus2=500*(1.876-energeVoltage);
		}
		else if(energeVoltage<1.65)
		{
			timeus2=500*(1.856-energeVoltage);
		}
		else if(energeVoltage<1.7)
		{
			timeus2=500*(1.882-energeVoltage);
		}
		else if(energeVoltage<1.75)
		{
			timeus2=300*(2.0500-energeVoltage);
		}	
		else if(energeVoltage<1.8)
		{
			timeus2=300*(2.0667-energeVoltage);
		}
		else if(energeVoltage<1.85)
		{
			timeus2=300*(2.09337-energeVoltage);
		}
		else if(energeVoltage<2.0)
		{
			timeus2=100*(2.6-energeVoltage);
		}
		else if(energeVoltage<2.3)
		{
			timeus2=65*(2.923-energeVoltage);
		}
		else if(energeVoltage<2.9)
		{
			timeus2=30*(3.6-energeVoltage);
		}
		else if(energeVoltage<DAC_MAX_VOLTAGE_F+1)
		{
			timeus2=10*(5.0-energeVoltage);
		}
		else 
		{
			timeus2=(unsigned short int)(288/(ev*ev));
		}		
		if(energeVoltage<1.85)
		{
		 	//timeus=(unsigned short int)(94.0/ev)+timeUs+timeus2-(unsigned short int)((energeVoltage*4+14)+6.5);			
			timeus=(unsigned short int)(94.0/ev)+timeUs+timeus2-(unsigned short int)(energeVoltage*4+20.5);
			timeLoad=timeus;
		}
		else
		{
			timeus=(unsigned short int)((94.0/ev)+13)+timeUs+timeus2;	
			timeLoad=timeus;		
			app_laser_pulse_width_set(timeUs*10,ev);
		} 
		if( timeLoad > JDQ_MAX_CONTROL_PULSE_US_WIDTH)  timeLoad = JDQ_MAX_CONTROL_PULSE_US_WIDTH;//check pulse timeUs
		if( timeLoad <JDQ_MIN_CONTROL_PULSE_US_WIDTH )  timeLoad = JDQ_MIN_CONTROL_PULSE_US_WIDTH;//check pulse timeUs		
		//if( freq > 60 ) counter = 16667;//(1000000/60);
		if( freq > 100 ) counter = 10000;//(1000000/100);
		else if( freq < 1 )  counter = 1000000; //(1000000/1)
		else counter=(1000000/freq);
		__HAL_TIM_SetAutoreload(&htim2,counter-1);//1~100HZ	
		__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,timeLoad-1);	
		HAL_TIM_OC_Start_IT(&htim2,TIM_CHANNEL_1);	
		HAL_TIM_Base_Start_IT(&htim2);	
	}
	else 
	{		
		HAL_TIM_OC_Stop_IT(&htim2,TIM_CHANNEL_1);
		HAL_TIM_Base_Stop_IT(&htim2);	
	}	
 }
  /************************************************************************//**
  * @brief 激光电源输出端电压反馈
  * @param 无
  * @note   大电容输出端电压检测,voltage range((min)80v ~160v~(max)180v)
  * @retval 键值
  *****************************************************************************/
 float app_jdq_voltage_monitor(void)
 {
	float retVoltage=0,cvolt;		
	if(ads1110_available()==TRUE1)
	{		 
		if(ads1110_read_mv(&cvolt)==TRUE1)
		{		
			retVoltage = cvolt*0.0919+12.0;//91.9*0.001+12.0;//12.0V隔离地偏差			  	
		}	
	}	
	return retVoltage;
 } 
  /************************************************************************//**
  * @brief 激光电源限流充电
  * @param 无
  * @note   
  * @retval 
  *****************************************************************************/
 void app_jdq_current_limit_charge(void)
 {	
	jdq_reley_charge_ready(0);
	HAL_Delay(1);
	jdq_reley_charge(1);		
 }
 
/***************************************************************************//**
 * @brief 电源数据接收
 * @param 
 * @note 
 * @return 
*******************************************************************************/ 
void app_jdq_rs485_receive_data(void)
{	
	jdq_rs485_receiv_len%=(MAX_UART1_BUFF_LENTH+1);
	UART1_RX_BUFF[jdq_rs485_receiv_len]=rs485_rec_byte;
	jdq_rs485_receiv_len++;		
	if(HAL_UART_Receive_IT(&huart1, &rs485_rec_byte,1)!=HAL_OK)
	{
	 /*Transfer error in reception process */
		//Error_Handler();			
	}	
}
/***************************************************************************//**
 * @brief 获取总线状态
 * @param 
 * @note 
 * @return 
*******************************************************************************/ 
unsigned short int  app_get_jdq_rs485_bus_statu(void)
{
	#ifdef JDQ_PWR_GWB_3200W	
	app_jdq_rs485_check_gwb3200_rec_package();	
	#else 	
	app_jdq_sts_1200_receive_handle();
	#endif
	return jdq_rs485_sta.frame_regStart;
}





