/*
 * air_gzp6816d_bsp.c
 *
 *  Created on: Apr 10, 2025
 *      Author: Hql2017
 */
#include "main.h"
#include "air_gzp6816d_bsp.h" 
#include "i2c.h" 

//The 7-bit IIC address of the sensor is 0x78 ������ 7 λ IIC ���ߵ�ַ
#define  GZP6816D_REAL_ADDR   0x78

#define PMIN 30.0 // The minimum range pressure value for example 30Kpa �������������
#define PMAX 110.0 //The full scale pressure value, for example 110Kpa �������������
#define DMIN 1677722.0 //AD value corresponding to The minimum range pressure, // for example 10%AD=2^24*0.1 ������̶�Ӧ�� AD ֵ
#define DMAX 15099494.0 //AD Value Corresponding to The full scale pressure value, // for example 90%AD=2^24*0.9 ������̶�Ӧ�� AD ֵ

float pressure_kpa = 0.0; //���������ڱ���ѹ��ֵ����λΪ KPa
unsigned long pressure_pa = 0; //���������ڱ���ѹ��ֵ����λΪ Pa
float temperature = 0.0; //���������ڱ����¶�ֵ����λΪ��

static unsigned char Device_Address = GZP6816D_REAL_ADDR << 1;//(0x78<<1)=0xA0;
static void GZP6816D_IIC_Write(unsigned short int addr,unsigned char *buff,unsigned short int len);

/**
  * @brief GZP6816D_init
  * @param void
  *         
  * @note   GZP6816D_init
  * @retval None
  */
 void GZP6816D_init(void)
{ 
  unsigned char buff; 
  app_I2C_start(&hi2c2);
  buff=0xAC;
  GZP6816D_IIC_Write(Device_Address, &buff, 1); //oxAC，默认配置
}
/**
  * @brief GZP6816D_IIC_Read
  * @param   uint16_t addr:device IIC address
            uint8_t* rBuf��receive data buff 
            uint16_t len:data length
  * @note   GZP6816D_IIC_Read 
  * @retval None
  */
static void GZP6816D_IIC_Read(unsigned short int addr,unsigned char *buff,unsigned short int len)
{	
  HAL_StatusTypeDef err;	
	err=HAL_I2C_Master_Receive(&hi2c2,addr,buff,len,100);  
}

/**
  * @brief GZP6816D_IIC_write
  * @param   uint16_t addr:device IIC address
            uint8_t* rBuf receive data buff 
            uint16_t len:data length
  * @note   GZP6816D_IIC_write
  * @retval None
  */
static void GZP6816D_IIC_Write(unsigned short int addr,unsigned char *buff,unsigned short int len)
{
  HAL_StatusTypeDef err;  
  err=HAL_I2C_Master_Transmit(&hi2c2,addr,buff,len,100);   
}
//Read the status of the sensor and judge whether IIC is busy
unsigned char GZP6816D_IsBusy(void) 
{
    unsigned char status;				
    GZP6816D_IIC_Read(Device_Address, &status, 1);
    status = (status >> 5) & 0x01;
    return status;	
}
/**
  * @brief  GZP6816D_start_sampling
  * @param  none
  * @note   启动一次测量
  * @retval None
  */
void GZP6816D_start_sampling(void) 
{
	unsigned char buff;	
	unsigned char buff2[4];	
	buff2[0]=0x34;
	buff2[1]=0x35;
	buff2[2]=0x74;
	buff2[3]=0x94;
  buff= 0xAC; //Send 0xAC command and read the returned six-byte data		 
  GZP6816D_IIC_Write(Device_Address, &buff, 1); // 0XAC 启动一次采样 
}
/**
  * @brief GZP6816D_get_cal
  * @param  none
  * @note   GZP6816D_IIC_write
  * @retval None
  */
void GZP6816D_get_cal(float *preKpa,float *enTemprature) //The function of reading pressure and temperature from the sensor
{
    unsigned char buffer[6] = {0}; //Temp variables used to restoring bytes from the sensor

    unsigned long Dtest = 0;
    unsigned int temp_raw = 0;  
    GZP6816D_IIC_Read(Device_Address, buffer, 6); 

    //Computing the calibrated pressure and temperature values
    Dtest = (unsigned long)((((unsigned long)buffer[1]) << 16) | (((unsigned int)buffer[2]) << 8) | ((unsigned char)buffer[3]));
    temp_raw = ((unsigned int)buffer[4] << 8) | (buffer[5] << 0);
    //The calibrated pressure value is converted into actual values
    if (Dtest != 0)
    { 
			
        //pressure_kpa = (float) ((PMAX-PMIN)/(DMAX-DMIN)*(Dtest-DMIN)+PMIN); //
        *preKpa = (float) ((PMAX-PMIN)/(DMAX-DMIN)*(Dtest-DMIN)+PMIN); //
        pressure_pa = (unsigned long) (pressure_kpa * 1000.0); //
    }
    else
    {
       *preKpa=0;
        pressure_kpa = 0.0; //pressure value, its unit is KPa 
        pressure_pa = 0; //pressure value, its unit is Pa 
    }
    temperature = (float) temp_raw / 65536; //The calibrated temperature value is converted into actual values
    temperature = ((float) temperature * 19000 - 4000)/100; // its unit is 
    *enTemprature=temperature;
}

/*****************************api**********************************************/
/***************************************************************************//**
 * @brief 环境气压状态监测
 * @param sysTick,系统时间
 * @note 
 * @return 
*******************************************************************************/
unsigned char  app_gzp6816d_listen(unsigned int sysTick,float *envirPresure,float *envirTemprature)
{	
  unsigned char  status=0;
  static unsigned int localTick=0;
  if(sysTick>=localTick+500)
  {
    localTick=sysTick;
    status=GZP6816D_IsBusy();			
    if(status==0)	
    {      
      GZP6816D_get_cal(envirPresure,envirTemprature);       
    }    
    GZP6816D_start_sampling();
  } 
  else localTick=sysTick;
  return status;
}