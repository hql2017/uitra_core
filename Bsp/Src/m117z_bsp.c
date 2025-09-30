/*
 * m117z_bsp.c
 *
 *  Created on: Apr 10, 2025
 *      Author: Hql2017
 */
#include "main.h"
#include "m117z_bsp.h" 
#include "i2c.h" 

#define  M117Z_REAL_ADDR   0x44<<1//

#define  IS31FL3193_REAL_ADDR   0xD0//only write

#define M117Z_MIN_TEMPRATURE   -10
#define M117Z_MAX_TEMPRATURE   60  

#define M117Z_I2C_TIMEOUT     100
#define M117Z_TPU_TIME   100//100ms  
#define M117Z_CMD_WAIT_TIME   1//��������ִ�еȴ�ʱ��

#define  M117Z_CMD_TEMPRATURE_CODE  0xCC44//启动测量
#define  M117Z_WRITE_ALERT_HI_SET   0x611d//设置报警高阈值
#define  M117Z_WRITE_ALERT_HI_UNSET 0x6116//取消设置报警高阈值
#define M117Z_WRITE_ALERT_LO_SET    0x6100//设置报警低阈值
#define M117Z_WRITE_ALERT_LO_UNSET  0x610b//取消设置报警低阈值
#define M117Z_READ_ALERT_HI_SET     0xe11f//读报警高阈值
#define M117Z_READ_ALERT_HI_UNSET   0xe114//读取消报警高阈值
#define M117Z_READ_ALERT_LO_SET     0xe102//读低报警阈值
#define M117Z_READ_ALERT_LO_UNSET   0xe109//读取消低报警阈值
#define M117Z_CONFIG                0x5206//配置
#define M117Z_READ_STATUS           0xf32d//读状态
#define M117Z_CLEAR_STATUS          0x3041//清除转台
#define M117Z_BREAK                 0x3093//
#define SOFT_RST                    0x30a2//软件复位
#define M117Z_COPY_PAGE0            0xcc48 //����Page0��EEROM
#define M117Z_RECALL_EE             0xccb8//�ָ�EE����
#define M117Z_RECALL_PAGE0_RES      0xccb6//�ָ�Page0����չ����

// CRCУ�麯��
#define CRC_POLYNOMIAL_8    0x0C
#include <stdint.h>
#include <stdbool.h>
 
// CRC-8/MAXIM�Ķ���
#define CRC8_POLY 0x31  // ����ʽ����ĳЩʵ������0x07
#define CRC8_INIT 0xFF  // ��ʼֵ
#define CRC8_XOROUT 0x00  // �������ֵ
#define REFLECT_INPUT false  // ���������Ƿ���
#define REFLECT_OUTPUT false  // ���CRC�Ƿ���
static void M117Z_IIC_Read(unsigned short int reg,unsigned char *buff,unsigned short int len);
static void M117Z_IIC_Write(unsigned short int reg,unsigned char *data,unsigned short int len);
/**
  * @brief crc8
  * @param void
  * @note   CRCУ�� 
  * @retval None
  */
uint8_t crc8(uint8_t *data, size_t length) {
    uint8_t crc = CRC8_INIT;
    uint8_t i, j;
    uint8_t c;
    uint8_t poly = CRC8_POLY;
    if (REFLECT_INPUT) {
        for (i = 0; i < length; i++) {
            c = data[i];
            c = (c ^ (crc << (8 - 1))) & 0xFF;
            for (j = 0; j < 8; j++) {
                if (c & 0x80) {
                    c = (c << 1) ^ poly;
                } else {
                    c = (c << 1);
                }
            }
            crc = c & 0xFF;
        }
    } else {
        for (i = 0; i < length; i++) {
            c = data[i];
            crc ^= c;
            for (j = 0; j < 8; j++) {
                if (crc & 0x80) {
                    crc = (crc << 1) ^ poly;
                } else {
                    crc = (crc << 1);
                }
            }
        }
    }
    if (REFLECT_OUTPUT) {
        uint8_t ref_out = 0;
        uint8_t bit = 1;
        for (i = 0; i < 8; i++) {
            if (crc & bit) ref_out |= (1 << (7 - i));
            bit <<= 1;
        }
        crc = ref_out;
    }
    return crc ^ CRC8_XOROUT;
}

/**-----------------------------------------------------------------------
  * @brief  唤醒
  * @param  none
  * @retval none
-------------------------------------------------------------------------*/
void Quick_wakeup(void)
{
  HAL_GPIO_WritePin(TEMPRATURE_ALERT_I2C1_SCL_GPIO_Port, TEMPRATURE_ALERT_I2C1_SCL_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(TEMPRATURE_ALERT_I2C1_SDA_GPIO_Port, TEMPRATURE_ALERT_I2C1_SDA_Pin, GPIO_PIN_RESET); 
  delay_us(50);  
  HAL_GPIO_WritePin(TEMPRATURE_ALERT_I2C1_SDA_GPIO_Port, TEMPRATURE_ALERT_I2C1_SDA_Pin, GPIO_PIN_SET); 
  delay_us(10);
  HAL_GPIO_WritePin(TEMPRATURE_ALERT_I2C1_SCL_GPIO_Port, TEMPRATURE_ALERT_I2C1_SCL_Pin, GPIO_PIN_RESET);	   
  delay_us(20); 	
  HAL_GPIO_WritePin(TEMPRATURE_ALERT_I2C1_SDA_GPIO_Port, TEMPRATURE_ALERT_I2C1_SDA_Pin, GPIO_PIN_RESET); 	   
  delay_us(25);	
  HAL_GPIO_WritePin(TEMPRATURE_ALERT_I2C1_SCL_GPIO_Port, TEMPRATURE_ALERT_I2C1_SCL_Pin, GPIO_PIN_SET);
  delay_us(10);
  HAL_GPIO_WritePin(TEMPRATURE_ALERT_I2C1_SDA_GPIO_Port, TEMPRATURE_ALERT_I2C1_SDA_Pin, GPIO_PIN_SET); 
  delay_us(15);
}
/**
  * @brief M117Z_init
  * @param void
  *         
  * @note   M117Z_init
  * @retval None
  */
 void M117Z_init(void)
{  
  unsigned char buff[3],conf;  
  app_I2C_start(&hi2c1);
  HAL_Delay(M117Z_TPU_TIME);
  Quick_wakeup();    
  M117Z_IIC_Read(M117Z_CONFIG,buff,3); 
  //conf=buff[0];
  //printf("m117Z_t conf=0x%x%x%X\r\n",buff[0],buff[1],buff[2]);
  //conf=0x0D;//每秒测量两次  0x11每秒测4次
  conf=0x0D;//
  buff[0]=conf;  
  buff[1]=0xFF;
  M117Z_IIC_Write(M117Z_CONFIG, buff, 2);  //config 
}
  /************************************************************************//**
  * @brief M117Z_IIC_Read
  * @param   uint16_t addr:device IIC address��
            uint8_t* rBuf��receive data buff 
            uint16_t len:data length
  * @note   M117Z_IIC_Read 
  * @retval None
  *****************************************************************************/
static void M117Z_IIC_Read(unsigned short int reg,unsigned char *buff,unsigned short int len)
{	
  HAL_StatusTypeDef err; 
  err=HAL_I2C_Mem_Read(&hi2c1,M117Z_REAL_ADDR,reg,I2C_MEMADD_SIZE_16BIT,buff,len,M117Z_I2C_TIMEOUT);   
}

/**
  * @brief M117Z_IIC_write
  * @param   uint16_t unsigned short int reg
            unsigned char *data
            uint16_t len:data length
  * @note   M117Z_IIC_write
  * @retval None
  */
static void M117Z_IIC_Write(unsigned short int reg,unsigned char *data,unsigned short int len)
{
  HAL_StatusTypeDef err; 
	unsigned char buff[5];
  if(len==0)//no data
  {
    buff[0]	=(reg>>8)&0xFF;
	  buff[1]	=reg&0xFF;
    buff[2]=crc8(buff, 2);
    err= HAL_I2C_Master_Transmit(&hi2c1,M117Z_REAL_ADDR,buff,3,100); 
  }
  else 
  {
    buff[0]	=reg>>8;
	  buff[1]	=reg&0xFF;
    buff[2]	=data[0];
	  buff[3]	=data[1];
    
	  buff[4]	=crc8(buff, 4);//�̶�����5bytes
    err=HAL_I2C_Mem_Write(&hi2c1,M117Z_REAL_ADDR,reg,I2C_MEMADD_SIZE_16BIT,&buff[2],3,M117Z_I2C_TIMEOUT);
  }   
}
//Read the status of the sensor and judge whether IIC is busy
unsigned short int M117Z_get_status(void) 
{
  unsigned char recBuff[3];
	unsigned short int ret;	
  recBuff[0]=(SOFT_RST>>8)&0xff;
  recBuff[1]=SOFT_RST&0xff;  
  M117Z_IIC_Read(M117Z_READ_STATUS, recBuff, 2);  
	ret=recBuff[1];
  return ret;	
}
  /************************************************************************//**
  * @brief 读取温度值
  * @param 无
  * @note   
  * @retval 温度值
  *****************************************************************************/
 short int M117Z_get_temprature(void) 
{
  unsigned char recBuff[3];
  short int  temp;
  HAL_StatusTypeDef err; 
  err=HAL_I2C_Master_Receive(&hi2c1,M117Z_REAL_ADDR,recBuff,3,100); 
  if(err==HAL_OK)
  {
    temp = (short int)(recBuff[0]<<8|recBuff[1]); 
    temp=40+(temp/256);
  } 
  else temp=ERR_T_SHORT_INT_VALUE
  
  ;//0xFFFE;//err  	
  return temp;
}
  /************************************************************************//**
  * @brief 启动一次测量
  * @param 无
  * @note   
  * @retval 无
  *****************************************************************************/
void M117Z_start_sampling(void) 
{
	unsigned char buff[4];
  unsigned char conf;  
  //M117Z_IIC_Read(M117Z_CONFIG,buff,3); 
 // conf=buff[0];
  //printf("m117Z_t conf=0x%x%x%X\r\n",buff[0],buff[1],buff[2]);
  //conf=0x11; 
  //buff[0]=conf;  
  //buff[1]=0xFF;
 // M117Z_IIC_Write(M117Z_CONFIG, buff, 2);  //config
 // HAL_Delay(2);
  M117Z_IIC_Write(M117Z_CMD_TEMPRATURE_CODE, buff, 0);   
}


