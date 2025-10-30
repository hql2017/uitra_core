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

#define JDQ_DAC_CS_ENABLE  HAL_GPIO_WritePin(DAC_CS_GPIO_Port, DAC_CS_Pin, GPIO_PIN_RESET)
#define JDQ_DAC_CS_DISABLE  HAL_GPIO_WritePin(DAC_CS_GPIO_Port, DAC_CS_Pin, GPIO_PIN_SET)
#define JDQ_LDAC_ENABLE  HAL_GPIO_WritePin(DAC_LD_GPIO_Port, DAC_LD_Pin, GPIO_PIN_RESET)
#define JDQ_LDAC_DISABLE  HAL_GPIO_WritePin(DAC_LD_GPIO_Port, DAC_LD_Pin, GPIO_PIN_SET)

#define HV_PULSE_TIM_RELOAD_VALUE 1000//10ms  100HZ

extern TIM_HandleTypeDef htim3;

//JDQ POWER
#define  STS_1200_OUT_VOLTAGE_LIMIT	220.0f//最大输出220V（MAX220）100mV分辨率
#define  STS_1200_OUT_CURRENT_LIMIT	100.0f//最大输出100A（MAX220）
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
static uint16_t jdq_sts_reg_value[8];

static unsigned char  jdq_rs485_receiv_len,rs485_rec_byte;
static unsigned char UART1_TX_BUFF[MAX_UART1_BUFF_LENTH+1]={0};
static unsigned char UART1_RX_BUFF[MAX_UART1_BUFF_LENTH+1]={0};

#define JDQ_RS485_TX  HAL_GPIO_WritePin(RS485_DIR_out_GPIO_Port, RS485_DIR_out_Pin, GPIO_PIN_SET)
#define JDQ_RS485_RX  HAL_GPIO_WritePin(RS485_DIR_out_GPIO_Port, RS485_DIR_out_Pin, GPIO_PIN_RESET)

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

/**
 * @brief jdq_reley_charge 
 * @param  void
 * @note   init
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
 * @note   init
 * @retval None
 */
void jdq_reley_charge_ready(unsigned char onOff)
{
	if(onOff==0) HAL_GPIO_WritePin(JDQ_READY_GPIO_Port, JDQ_READY_Pin, GPIO_PIN_RESET);
	else HAL_GPIO_WritePin(JDQ_READY_GPIO_Port, JDQ_READY_Pin, GPIO_PIN_SET); 
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
	jdq_reley_charge(0);//限流负载A端接地
	jdq_reley_charge_ready(0);//限流负载B端接电容正极	
	JDQ_RS485_RX;	
	HAL_Delay(1500);		
}
//***************************AD5541ABRMZ********����ʽdac***************************************//
#define  AD5541_VREF   4.096f//2.5//2.5V
/***************************************************************************//**
 * @brief ͨspi发送寄存器设置值
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
	err=HAL_SPI_Transmit(&hspi1, TXdata, dLength, 100);//HAL_MAX_DELAY);
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
//***************************AD5541ABRMZ********dac***************************************//

//******CA-IS1306M25G**5MHz~24MHZ，通过485读取***********//

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
/***************************************************************************//**
 * @brief 进入侦听状态
 * @param listenReg,侦听端口，datalen，侦听数据长度
 * @note 
 * @return 
*******************************************************************************/
void app_jdq_lisen(uint16_t listenReg,uint16_t dataLen)
{
	jdq_rs485_receiv_len=0;
	jdq_rs485_sta.frame_regStart=listenReg;
	jdq_rs485_sta.frame_len=dataLen;	
	HAL_UART_Receive_IT(&huart1,&rs485_rec_byte, 1);//切换进入侦听状态
}
/***************************************************************************//**
 * @brief 问询帧
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
	unsigned short int regStart;
	int volBuff,currentBuff;
	if(powerVolotage<12.0) volBuff=120;
	else if(powerVolotage>STS_1200_OUT_VOLTAGE_LIMIT) volBuff=(int)(STS_1200_OUT_VOLTAGE_LIMIT*10);
	else volBuff=(int) (powerVolotage*10);

	if(powerCurrent<1.0) currentBuff=10;
	else if(powerCurrent>STS_1200_OUT_CURRENT_LIMIT) currentBuff=(int)(STS_1200_OUT_CURRENT_LIMIT*10);
	else currentBuff=(int) (powerCurrent*10);	
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
 }
  /************************************************************************//**
  * @brief 设置总电源电流值
  * @param powerCurrent电流值 
  * @note   0.1A分辨率
  * @retval 
  *****************************************************************************/
 void app_jdq_bus_current_set(float  powerCurrent)
 {
	HAL_StatusTypeDef err;	
	unsigned short int sendBuff;
	if(powerCurrent<1.0) sendBuff=10;
	else if(powerCurrent>100.0) sendBuff=1000; //max100A
	else sendBuff=(int)(powerCurrent*10);
	err= app_jdq_write_req_frame(STS_1200_REG_SET_CURRENT,(uint16_t)sendBuff);	
 }
  /************************************************************************//**
  * @brief 设置总电源电压值
  * @param powerCurrent电流值
  * @note   0.1V分辨率
  * @retval 
  *****************************************************************************/
 void app_jdq_bus_voltage_set(float  voltage)
 {
	HAL_StatusTypeDef err;	
	int sendBuff;
	if(voltage<12.0) sendBuff=120;
	else if(voltage>220.0) sendBuff=2200; 
	else sendBuff=(int)(voltage*100);//100倍
	err= app_jdq_write_req_frame(STS_1200_REG_SET_VOLTAGE,(uint16_t)sendBuff);		
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
	err= app_jdq_read_req_frame(STS_1200_REG_SET_VOLTAGE,4);
 }
   /************************************************************************//**
  * @brief 读总电源电压输出状态请求
  * @param 
  * @note   
  * @retval 
  *****************************************************************************/
 void app_jdq_bus_power_onoff_sta_req(void)
 {
	HAL_StatusTypeDef err;	
	err= app_jdq_read_req_frame(STS_1200_REG_RUN_STOP,1);
 }
   /************************************************************************//**
  * @brief 读总电源电流设定值
  * @param  *voltage 电压设定, *current 电流设定
  * @note   
  * @retval 
  *****************************************************************************/
 void app_jdq_bus_get_set_v_c(float *voltage,float *current)
 {
	*voltage=jdq_sts_reg_value[0]*0.1;
	*current=jdq_sts_reg_value[1]*0.1;
 }
 /************************************************************************//**
  * @brief 读电容端电压
  * @param  *outVoltage 电压, 
  * @note   
  * @retval 
  *****************************************************************************/
float app_jdq_get_laser_v(void)
 {
	float outVoltage=jdq_sts_reg_value[2]*0.1;
	return outVoltage;
 }
  /************************************************************************//**
  * @brief 读电源输出状态
  * @param *sta  状态值指针
  * @note   
  * @retval 
  *****************************************************************************/
 unsigned short int  app_jdq_get_vbus_sta(void )
 {		
	return jdq_sts_reg_value[6];
 }
  /************************************************************************//**
  * @brief 开启总电源
  * @param 0；关闭 1：开启
  * @note   
  * @retval 
  *****************************************************************************/
 void app_jdq_bus_power_on_off(unsigned char flag)
 {
	HAL_StatusTypeDef err;
	if(flag==0)
	{
		err = app_jdq_write_req_frame(STS_1200_REG_RUN_STOP,0);
	}
	else //if ()
	{
		err = app_jdq_write_req_frame(STS_1200_REG_RUN_STOP,1);
	}		
 }
//JDQ 继电器 逻辑 (LL HL  LH LL) (stand ,ready)
//***************************laser pulse (100us~500us) timer3***************************************//

/**
  * @brief app_laser_pulse_start 
  * @param  
  * @note   laser pulse width(l_pulse) 100us~230us   ；voltage pulse width (v_pulse)  v_pulse ≈ l_pulse+40;（140~270us）
  * @retval None
  */
 void app_laser_pulse_start(unsigned short int timeUs,unsigned short int freq)
 { 
	unsigned short int counter,timeload;
	if(timeUs!=0)
	{
		//if( freq > 60 )  countor =3332;// (200000/60);
		if( freq > 100 )  counter =2000;// (200000/60);
		else if( freq < 5 )  counter = 40000; //(200000/5)
		else counter=(200000/freq);
		if( timeUs > 300 )  timeload = 60;//check pulse timeUs
		else if( timeUs < 120 )  timeload = 24;//check pulse timeUs		
		else timeload=timeUs/5;
		__HAL_TIM_SetAutoreload(&htim3,counter-1);
		__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_2,timeload-1);
		HAL_TIM_PWM_Start_IT(&htim3,TIM_CHANNEL_2);		
		HAL_TIM_Base_Start_IT(&htim3);
	}
	else 
	{
		HAL_TIM_PWM_Stop_IT(&htim3,TIM_CHANNEL_2);	
		HAL_TIM_Base_Stop_IT(&htim3);
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
	jdq_reley_charge(1);
	jdq_reley_charge_ready(0);
		
 }
   /************************************************************************//**
  * @brief 激光充电完成，电源直连
  * @param 无
  * @note   
  * @retval 
  *****************************************************************************/
 void app_jdq_direct_160v(void)
 {
	jdq_reley_charge(1);
	jdq_reley_charge_ready(1);
 }
 
/***************************************************************************//**
 * @brief 电源数据接收
 * @param 
 * @note 
 * @return 
*******************************************************************************/ 
void app_jdq_rs485_receive_data(void)
{	
	jdq_rs485_receiv_len%=MAX_UART1_BUFF_LENTH;
	UART1_RX_BUFF[jdq_rs485_receiv_len]=rs485_rec_byte;
	jdq_rs485_receiv_len++;		
	if(HAL_UART_Receive_IT(&huart1, &rs485_rec_byte,1)!=HAL_OK)
	{
	 /*Transfer error in reception process */
		Error_Handler();			
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
	//if(app_jdq_rs485_check_rec_len()!=0)
	if(jdq_rs485_receiv_len!=0)
	{
		app_jdq_sts_1200_receive_handle();
	}
	return jdq_rs485_sta.frame_regStart;
}
/***************************************************************************//**
 * @brief 获取数据长度
 * @param 
 * @note 
 * @return 
*******************************************************************************/ 
unsigned char  app_jdq_rs485_check_rec_len(void)
{		
	return jdq_rs485_receiv_len;
}


