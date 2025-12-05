/*
 * lS31FL3193_bsp.c
 *  only write
 *  Created on: Apr 10, 2025
 *      Author: Hql2017
 */
#include "main.h"
#include "IS31FL3193_bsp.h" 
#include "i2c.h" 
#include "cmsis_os2.h"

#define  IS3_I2C_TIMEOUT  100
void RGB_shutdown(unsigned char status );


#ifdef  FL3236_USED
static uint8_t rgbBreathCountMs=0;//
#else
#define shutdown_chip    0
#define start_chip       1

#define  IS3_REAL_ADDR   0xD0//only write,0x78<<1

#define IS3_REG_SOFT_SHUTDOWN  0x00   //soft shutdown������ֹͣ���  0x20 ��ͨģʽ 0x21 ����ֹͣģʽ
#define IS3_REG_HEATH_CONTROL  0x01  //��������
#define IS3_REG_OPYION_MODE    0x02  //led����ģʽ,0x00PWM mode;0x20 one shot programming mode
#define IS3_REG_CURRENT_SET    0x03  //�����������Imax��{0x00��0x40,0x80, 0x0C,0x1C(>=0x10) }-{42MA ,10mA,5mA,30mA,17.5mA}
#define IS3_REG_CHANNEL1_PWM   0x04  //����ͨ��1���PWM(0~255),Iout=Imax(pwm1)/256
#define IS3_REG_CHANNEL2_PWM   0x05  //����ͨ��2���PWM
#define IS3_REG_CHANNEL3_PWM   0x06 //����ͨ��3���PWM
#define IS3_APPLAY_PWM_CONTROL 0x07  //Ӧ��pwm��led����ģʽ д 0x00���� �������ݡ�
#define IS3_T0_SET         0x0A  //����T0ʱ��  value1=T0<<4 value2=T0<<4 value3=T0<<4   0.13*(0,1,2,4,8,16,32,64,128,256,512) 2^n
#define IS3_T1_T2_SET      0x10  //����ͨ������T1(bit7~bit5)��T2(bit4~bit1)ʱ�� value=(T1<<5)|(T2<<1)  T1-0.13*(0,1,2,3,4,5,6,7,8)2^n(128) T2-0.13*(0+����0,1,2,4,8,16,32,64,128)2^n��
#define IS3_T3_T4_SET      0x16  //����ͨ������T3(bit7~bit5)��T4(bit4~bit1)ʱ��   value=(T3<<5)|(T4<<1) 0.13*(0,1,2,43,4,5,6,7,8��9��10)2^n
#define IS3_T0_4_UPDATA     0x1C  //Ӧ��T0~T4ʱ��  д 0x00����  0x0A~0x0C���ݡ�

#define IS3_RGB_ENABLE     0x1D  //led���ʹ��λ��bit2~bit0����out1~out3��
#define IS3_SOFT_RESTOREGE 0x2F  //�������ûָ�Ĭ��ֵ,д0x00ִ�в���

static unsigned short int  T0_list[12]={0,1,2,4,6,8,16,32,64,128,256,512};
//out1 R  out0 G out2 B
static const unsigned char   rgb_color_value[3]={255,0,255};//Ĭ����ɫRGB��255,0,255��
static const unsigned char   rgb_light_value[5]={0x80,0x40,0x1C,0x0C,0x00};////Imax����0~5 ��5ma 10mA ,17.5mA,30mA,42mA��
typedef struct 
{
  unsigned char soft_shutdown;
  unsigned char breath1_ctrl;
  unsigned char breath2_ctrl;
  unsigned char breath3_ctrl;
  unsigned char led_ctl_mode;
  unsigned char imax_value;//������,����
  unsigned char pwm_value[3];
  unsigned char T0_value[3];//t0ʱ�� 
  unsigned char t1_value[3];
  unsigned char t2_value[3];
  unsigned char t3_value[3];
  unsigned char t4_value[3];
}RGB_IS3_PARAM;
static RGB_IS3_PARAM  rgb_config_param;
static void is3_rgb_default_param(void);
static void ISI3_IIC_Write(unsigned char reg,unsigned char *data,unsigned short int len);
/**
  * @brief is3_rgb_default_param
  * @param  void
  * @note   Ĭ������
  * @retval None
  */
static void is3_rgb_default_param(void)
 {  
    unsigned char buff[8]; 
    RGB_shutdown(0);//Ӳ���ر�   
    buff[0]=0x00;
    ISI3_IIC_Write(IS3_SOFT_RESTOREGE,buff,1);//reset  
    HAL_Delay(5); 
    rgb_config_param.soft_shutdown=0x20;//0x21
    buff[0]=rgb_config_param.soft_shutdown;
    ISI3_IIC_Write(IS3_REG_SOFT_SHUTDOWN,    buff,1);
    rgb_config_param.breath1_ctrl=0x24;//B
    buff[0]=rgb_config_param.breath1_ctrl;
    ISI3_IIC_Write(IS3_REG_HEATH_CONTROL,buff,1);
    rgb_config_param.breath2_ctrl=0x25;//R
    buff[0]=rgb_config_param.breath2_ctrl;
    ISI3_IIC_Write(IS3_REG_HEATH_CONTROL,buff,1);
    rgb_config_param.breath3_ctrl=0x26;//G
    buff[0]=rgb_config_param.breath3_ctrl;
    ISI3_IIC_Write(IS3_REG_HEATH_CONTROL,buff,1);
    rgb_config_param.led_ctl_mode=0x00;//0x20 one shut program 0x00:PWM   
    rgb_config_param.imax_value=rgb_light_value[2]<<2;   
    rgb_config_param.pwm_value[1]=rgb_color_value[0];//0x20;//50%R
    rgb_config_param.pwm_value[0]=rgb_color_value[1];//0xC8;//50%G
    rgb_config_param.pwm_value[2]=rgb_color_value[2];//0x80;//50%B
    buff[0]=rgb_config_param.led_ctl_mode;
    buff[1]=rgb_config_param.imax_value;
    buff[2]=rgb_config_param.pwm_value[0];
    buff[3]=rgb_config_param.pwm_value[1];
    buff[4]=rgb_config_param.pwm_value[2];
    buff[5]=0x00;//updata
    ISI3_IIC_Write(IS3_REG_OPYION_MODE,buff,6);
    rgb_config_param.T0_value[0]=1;//1.04s
    rgb_config_param.T0_value[1]=1;//1.04s
    rgb_config_param.T0_value[2]=1;//1.04s
    rgb_config_param.t1_value[0]=3;//1.04s
    rgb_config_param.t2_value[0]=4;//1.04s
    rgb_config_param.t3_value[0]=3;//1.04s
    rgb_config_param.t4_value[0]=4;//1.04s
    rgb_config_param.t1_value[0]=3;//1.04s
    rgb_config_param.t2_value[1]=4;//1.04s
    rgb_config_param.t3_value[2]=3;//1.04s
    rgb_config_param.t4_value[3]=4;//1.04s
    rgb_config_param.t1_value[0]=3;//1.04s
    rgb_config_param.t2_value[1]=4;//1.04s
    rgb_config_param.t3_value[2]=3;//1.04s
    rgb_config_param.t4_value[3]=4;//1.04s
    buff[0]=rgb_config_param.T0_value[0]<<4;
    buff[1]=rgb_config_param.T0_value[1]<<4;
    buff[2]=rgb_config_param.T0_value[2]<<4;
    ISI3_IIC_Write(IS3_T0_SET,buff,3);
    buff[0]=(rgb_config_param.t1_value[0]<<5)|(rgb_config_param.t2_value[0]<<1);
    buff[1]=(rgb_config_param.t1_value[1]<<5)|(rgb_config_param.t2_value[1]<<1);
    buff[2]=(rgb_config_param.t1_value[2]<<5)|(rgb_config_param.t2_value[2]<<1);
    ISI3_IIC_Write(IS3_T1_T2_SET,buff,3);
    buff[0]=(rgb_config_param.t3_value[0]<<5)|(rgb_config_param.t4_value[0]<<1); 
    buff[1]=(rgb_config_param.t3_value[1]<<5)|(rgb_config_param.t4_value[1]<<1);   
    buff[2]=(rgb_config_param.t3_value[2]<<5)|(rgb_config_param.t4_value[2]<<1); 
    ISI3_IIC_Write(IS3_T3_T4_SET,buff,3);    
    buff[0]=0x00;
    ISI3_IIC_Write(IS3_T0_4_UPDATA,buff,1);//updata tim   
    buff[0]=0x07;
    ISI3_IIC_Write(IS3_RGB_ENABLE,buff,1);//enable
    RGB_shutdown(1);
 }
 /**
  * @brief ISI3_IIC_errhandle
  * @param   
  * @note   ISI3_IIC_errhandle
  * @retval None
  */
 static void ISI3_IIC_err_handle(void)
{
  IS3_init();//restart
}
/**
  * @brief ISI3_IIC_Write
  * @param   uint16_t unsigned short int reg
            unsigned char *data
            uint16_t len:data length
  * @note   ISI3_IIC_Write
  * @retval None
  */
static void ISI3_IIC_Write(unsigned char reg,unsigned char *data,unsigned short int len)
{
  HAL_StatusTypeDef err; 	
	err=HAL_I2C_Mem_Write(&hi2c5,IS3_REAL_ADDR,reg,I2C_MEMADD_SIZE_8BIT,data,len,IS3_I2C_TIMEOUT);
  if(err!=HAL_OK)
  {
    ISI3_IIC_err_handle();
    //error
  }	 
}

/**
  * @brief ISI3_IIC_Write
  * @param   uint16_t unsigned short int reg
            unsigned char *data
            uint16_t len:data length
  * @note   ISI3_IIC_Write
  * @retval None
  */
  void app_is3_rgb_config(unsigned char reg,unsigned char *data)
  {   
    if(reg==IS3_RGB_ENABLE) 
    {
      ISI3_IIC_Write(reg,data,1);
    }
    else if(reg==IS3_APPLAY_PWM_CONTROL) 
    { 
      ISI3_IIC_Write(reg,data,1);
    }   	
    else if(reg==IS3_T0_4_UPDATA) 
    {
      ISI3_IIC_Write(reg,data,1);
    }
  }
#endif

/**
  * @brief RGB_shutdown 
  * @param  unsigned char status��0 shutdown ��1 on
  * @note   RGB 
  * @retval None
  */
 void RGB_shutdown(unsigned char status )
 {
  if(status==0)  HAL_GPIO_WritePin(S31FL3193_SDB_out_GPIO_Port,S31FL3193_SDB_out_Pin, GPIO_PIN_RESET);
  else HAL_GPIO_WritePin(S31FL3193_SDB_out_GPIO_Port,S31FL3193_SDB_out_Pin, GPIO_PIN_SET);
 }
 
/**
  * @brief IS3_init
  * @param  void
  * @note   RGB 
  * @retval None
  */
void IS3_init(void)
{ 
  RGB_shutdown(0);
	#ifdef  FL3236_USED
	app_I2C_start(&hi2c5);
	IS31FL3236A_Init();
	RGB_shutdown(1);
	#else
	app_I2C_start(&hi2c5);
  is3_rgb_default_param();
	RGB_shutdown(1);
	#endif
}
/************************************************************************//**
  * @brief   rgb颜色常亮
  * @param  rgb     
    @param  
  * @note   
  * @retval None
 ******************************************************************************/
void rgb_color_all(unsigned short int color)
{ 
	uint8_t rgb_buff[8];
  if(color==1)//绿色常亮
	{
		#ifdef  FL3236_USED//12路
		is_12_all_rgb(0x07E0);//长亮不闪烁
		#else 
			rgb_buff[1]=0;
			rgb_buff[0]=(unsigned char)(u_sys_param.sys_config_param.rgb_light*2.55);//255;//++;   
			rgb_buff[2]=0;			
			rgb_buff[3]=0;//updata
			ISI3_IIC_Write(IS3_REG_CHANNEL1_PWM,rgb_buff,4); 
			rgb_buff[0]=0x00;
			ISI3_IIC_Write(IS3_T0_4_UPDATA,rgb_buff,1);//updata tim 
		#endif
	}
	else if(color==2)//紫色常亮
	{
		#ifdef  FL3236_USED//12路
		is_12_all_rgb(0xF81F);//长亮不闪烁
		#else 
		rgb_buff[1]=(unsigned char)(u_sys_param.sys_config_param.rgb_light*2.55);//255;
		rgb_buff[0]=0;;   
		rgb_buff[2]=(unsigned char)(u_sys_param.sys_config_param.rgb_light*2.55);//255;		
		rgb_buff[3]=0;//updata
		ISI3_IIC_Write(IS3_REG_CHANNEL1_PWM,rgb_buff,4); 
		rgb_buff[0]=0x00;
		ISI3_IIC_Write(IS3_T0_4_UPDATA,rgb_buff,1);//updata tim 
		#endif
	}
	else if(color==0)//关闭
	{
		#ifdef  FL3236_USED//12路
		is_12_all_rgb(0);
		#else 
		rgb_buff[1]=0;
    rgb_buff[0]=0;//++;   
    rgb_buff[2]=0;			
    rgb_buff[3]=0;//updata
    ISI3_IIC_Write(IS3_REG_CHANNEL1_PWM,rgb_buff,4); 
    rgb_buff[0]=0x00;
    ISI3_IIC_Write(IS3_T0_4_UPDATA,rgb_buff,1);//updata tim 
		#endif
	}		
}
/************************************************************************//**
  * @brief   rgb呼吸控制
  * @param   breathFreq freq     
    @param   rgbValue颜色值
  * @note   高电平有效
  * @retval None
 ******************************************************************************/
 void app_rgb_breath_ctl(unsigned char breathFreq,unsigned short int rgbValue)//tim callback
 { 
		#ifdef  FL3236_USED//12路
    if(breathFreq==0)
    {   					
      is_12_all_rgb(0);//关闭
      rgbBreathCountMs=0;
    }
    else //if(breathFreq==0xFF)
    {  
      if(rgbBreathCountMs>100) rgbBreathCountMs=100;	//10ms	
      rgbBreathCountMs = 1000/breathFreq;
      High_Breath();	//刷一次465ms    
    }		
		#else	
	//刷一次1ms
		static unsigned char rgb_buff[8]={0},flag=0;
#if 0		
		if(rgb_config_param.led_ctl_mode==0x00)//PWM
		{ 
      if(flag==0)  
      {      
        if(rgb_buff[2]>(rgb_color_value[2]/17)*16)    
        {        
          rgb_buff[1]-=(rgb_color_value[0]/17);//breath time =31*(17*2-1)=1023ms;
          rgb_buff[0]-=(rgb_color_value[1]/17);   
          rgb_buff[2]-=(rgb_color_value[2]/17);      
          flag=1;
        }
        else
        {      
          rgb_buff[1]+=(rgb_color_value[0]/17);
          rgb_buff[0]+=(rgb_color_value[1]/17);
          rgb_buff[2]+=(rgb_color_value[2]/17);
				}
			}
			else
			{      
				if(rgb_buff[2]*17<rgb_color_value[2])    
				{
					flag=0;        
					rgb_buff[1]+=(rgb_color_value[0]/17);
					rgb_buff[0]+=(rgb_color_value[1]/17);
					rgb_buff[2]+=(rgb_color_value[2]/17); 
				}
				else
				{     
					rgb_buff[1]-=(rgb_color_value[0]/17);	
					rgb_buff[0]-=(rgb_color_value[1]/17);
					rgb_buff[2]-=(rgb_color_value[2]/17);         
				}
			}
			rgb_buff[3]=0;//updata
			ISI3_IIC_Write(IS3_REG_CHANNEL1_PWM,rgb_buff,4);  
		}
		else
		{   
			rgb_buff[0]=0x00;
			ISI3_IIC_Write(IS3_T0_4_UPDATA,rgb_buff,1);//updata tim 
		}
#else 
			if(flag==0)
			{
        rgb_buff[1]++;
        rgb_buff[0]=0;//++;   
        rgb_buff[2]++; 
				if(rgb_buff[1]==(unsigned char)(u_sys_param.sys_config_param.rgb_light*2.55)) flag=1;//255) flag=1;
			}
			else 
			{
				if(rgb_buff[1]==0) flag=0;
					if(rgb_buff[1]!=0)rgb_buff[1]--;
							rgb_buff[0]=0;//++;   
						if(rgb_buff[2]!=0)	rgb_buff[2]--;
			}
			rgb_buff[3]=0;//updata
			ISI3_IIC_Write(IS3_REG_CHANNEL1_PWM,rgb_buff,4); 
			rgb_buff[0]=0x00;
			ISI3_IIC_Write(IS3_T0_4_UPDATA,rgb_buff,1);//updata tim 
			
#endif
		
		#endif
	}
 
#ifdef  FL3236_USED//12路
 
#define SDB_H   HAL_GPIO_WritePin(S31FL3193_SDB_GPIO_Port,S31FL3193_SDB_Pin, GPIO_PIN_SET)
#define SDB_L   HAL_GPIO_WritePin(S31FL3193_SDB_GPIO_Port,S31FL3193_SDB_Pin, GPIO_PIN_RESET)
uint8_t fl3236_data[64];
uint8_t volatile Mode_Num;
uint8_t PWM_64_table[128]=
{
    0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,
    0x08,0x09,0x0b,0x0d,0x0f,0x11,0x13,0x16,
    0x1a,0x1c,0x1d,0x1f,0x22,0x25,0x28,0x2e,
    0x34,0x38,0x3c,0x40,0x44,0x48,0x4b,0x4f,
    0x55,0x5a,0x5f,0x64,0x69,0x6d,0x72,0x77,
    0x7d,0x80,0x88,0x8d,0x94,0x9a,0xa0,0xa7,
    0xac,0xb0,0xb9,0xbf,0xc6,0xcb,0xcf,0xd6,
    0xe1,0xe9,0xed,0xf1,0xf6,0xfa,0xfe,0xff,

    0xff,0xfe,0xfa,0xf6,0xf1,0xed,0xe9,0xe1,
    0xd6,0xcf,0xcb,0xc6,0xbf,0xb9,0xb0,0xac,
    0xa7,0xa0,0x9a,0x94,0x8d,0x88,0x80,0x7d,
    0x77,0x72,0x6d,0x69,0x64,0x5f,0x5a,0x55,
    0x4f,0x4b,0x48,0x44,0x40,0x3c,0x38,0x34,
    0x2e,0x28,0x25,0x22,0x1f,0x1d,0x1c,0x1a,
    0x16,0x13,0x11,0x0f,0x0d,0x0b,0x09,0x08,
    0x07,0x06,0x05,0x04,0x03,0x02,0x01,0x00
};
unsigned char  PWM_RGB[512]=
{
    0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,
    0x08,0x09,0x0b,0x0d,0x0f,0x11,0x13,0x16,
    0x1a,0x1c,0x1d,0x1f,0x22,0x25,0x28,0x2e,
    0x34,0x38,0x3c,0x40,0x44,0x48,0x4b,0x4f,
    0x55,0x5a,0x5f,0x64,0x69,0x6d,0x72,0x77,
    0x7d,0x80,0x88,0x8d,0x94,0x9a,0xa0,0xa7,
    0xac,0xb0,0xb9,0xbf,0xc6,0xcb,0xcf,0xd6,
    0xe1,0xe9,0xed,0xf1,0xf6,0xfa,0xfe,0xff,

    0xff,0xfe,0xfa,0xf6,0xf1,0xed,0xe9,0xe1,
    0xd6,0xcf,0xcb,0xc6,0xbf,0xb9,0xb0,0xac,
    0xa7,0xa0,0x9a,0x94,0x8d,0x88,0x80,0x7d,
    0x77,0x72,0x6d,0x69,0x64,0x5f,0x5a,0x55,
    0x4f,0x4b,0x48,0x44,0x40,0x3c,0x38,0x34,
    0x2e,0x28,0x25,0x22,0x1f,0x1d,0x1c,0x1a,
    0x16,0x13,0x11,0x0f,0x0d,0x0b,0x09,0x08,
    0x07,0x06,0x05,0x04,0x03,0x02,0x01,0x00,

    0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,

    0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,
    0x08,0x09,0x0b,0x0d,0x0f,0x11,0x13,0x16,
    0x1a,0x1c,0x1d,0x1f,0x22,0x25,0x28,0x2e,
    0x34,0x38,0x3c,0x40,0x44,0x48,0x4b,0x4f,
    0x55,0x5a,0x5f,0x64,0x69,0x6d,0x72,0x77,
    0x7d,0x80,0x88,0x8d,0x94,0x9a,0xa0,0xa7,
    0xac,0xb0,0xb9,0xbf,0xc6,0xcb,0xcf,0xd6,
    0xe1,0xe9,0xed,0xf1,0xf6,0xfa,0xfe,0xff,

    0xff,0xfe,0xfa,0xf6,0xf1,0xed,0xe9,0xe1,
    0xd6,0xcf,0xcb,0xc6,0xbf,0xb9,0xb0,0xac,
    0xa7,0xa0,0x9a,0x94,0x8d,0x88,0x80,0x7d,
    0x77,0x72,0x6d,0x69,0x64,0x5f,0x5a,0x55,
    0x4f,0x4b,0x48,0x44,0x40,0x3c,0x38,0x34,
    0x2e,0x28,0x25,0x22,0x1f,0x1d,0x1c,0x1a,
    0x16,0x13,0x11,0x0f,0x0d,0x0b,0x09,0x08,
    0x07,0x06,0x05,0x04,0x03,0x02,0x01,0x00,

    0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,

    0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,
    0x08,0x09,0x0b,0x0d,0x0f,0x11,0x13,0x16,
    0x1a,0x1c,0x1d,0x1f,0x22,0x25,0x28,0x2e,
    0x34,0x38,0x3c,0x40,0x44,0x48,0x4b,0x4f,
    0x55,0x5a,0x5f,0x64,0x69,0x6d,0x72,0x77,
    0x7d,0x80,0x88,0x8d,0x94,0x9a,0xa0,0xa7,
    0xac,0xb0,0xb9,0xbf,0xc6,0xcb,0xcf,0xd6,
    0xe1,0xe9,0xed,0xf1,0xf6,0xfa,0xfe,0xff,

    0xff,0xfe,0xfa,0xf6,0xf1,0xed,0xe9,0xe1,
    0xd6,0xcf,0xcb,0xc6,0xbf,0xb9,0xb0,0xac,
    0xa7,0xa0,0x9a,0x94,0x8d,0x88,0x80,0x7d,
    0x77,0x72,0x6d,0x69,0x64,0x5f,0x5a,0x55,
    0x4f,0x4b,0x48,0x44,0x40,0x3c,0x38,0x34,
    0x2e,0x28,0x25,0x22,0x1f,0x1d,0x1c,0x1a,
    0x16,0x13,0x11,0x0f,0x0d,0x0b,0x09,0x08,
    0x07,0x06,0x05,0x04,0x03,0x02,0x01,0x00
};
#if 1
uint8_t I2C_WriteByte(int DeviceAddress, int WriteAddress, int SendByte)
{
	HAL_StatusTypeDef err; 	
	err=HAL_I2C_Mem_Write(&hi2c5,Addr_GND_GND,(uint16_t)WriteAddress,I2C_MEMADD_SIZE_8BIT,(uint8_t*)&SendByte,1,IS3_I2C_TIMEOUT);	
	return err;
}
uint8_t I2C_Write2Byte(int WriteAddress, int SendByte)
{	
	HAL_StatusTypeDef err; 	
	err=HAL_I2C_Mem_Write(&hi2c5,Addr_GND_GND,(uint16_t)WriteAddress,I2C_MEMADD_SIZE_16BIT,(uint8_t*)&SendByte,1,IS3_I2C_TIMEOUT);
	return err;	
}
uint8_t I2C_WriteBuffer(uint8_t* pBuffer,int length,int DeviceAddress ,int WriteAddress)
{
	HAL_StatusTypeDef err; 	
	err=HAL_I2C_Mem_Write(&hi2c5,Addr_GND_GND,(uint16_t)WriteAddress,I2C_MEMADD_SIZE_8BIT,pBuffer,length,IS3_I2C_TIMEOUT);
return err;	
}
#endif
void IC_Write_Pro(void)
{
    uint8_t i, j = 0, succeed = 1;
    if (fl3236_data[62] == 0x02)
    {
       succeed = I2C_Write2Byte(fl3236_data[4], fl3236_data[5]);
    }
    else
    {
      if (fl3236_data[62] > 2 && fl3236_data[62] % 2 == 1)
      {
        for (i = 0; i < (fl3236_data[62] - 1) / 2; i++)
        {
          succeed = I2C_WriteByte(fl3236_data[4], fl3236_data[5 + j], fl3236_data[6 + j]);
          if (succeed != 1)
          {
            break;
          }
          j += 2;
        }
      }
    }
    fl3236_data[4] = succeed;
}

void IC_WriteBuff_Pro(void)
{
  if (fl3236_data[62] > 2)
  {
    fl3236_data[4] = I2C_WriteBuffer(&fl3236_data[6], fl3236_data[62] - 2, fl3236_data[4], fl3236_data[5]);
  }
}
void Green_Breath(void )//呼吸一次，约耗时465ms，单色约600/128=4.7ms
{
  uint8_t i; 
  static uint8_t j;
  //for (j=0; j<128; j++)	  
  j++;
  j%=128; 
  {
      for (i=0; i<12; i++) //R
      { // R  G  B  
        I2C_WriteByte(Addr_GND_GND,1+i*3,0);   //B//PWM  
        if(u_sys_param.sys_config_param.rgb_light!=0) 
        {
          if(j<5||j>123) I2C_WriteByte(Addr_GND_GND,2+i*3,(unsigned char )(PWM_64_table[5]*(u_sys_param.sys_config_param.rgb_light*0.01))); //low 5%
          else I2C_WriteByte(Addr_GND_GND,2+i*3,(unsigned char )(PWM_64_table[j]*(u_sys_param.sys_config_param.rgb_light*0.01)));   //PWM
        }          
        else I2C_WriteByte(Addr_GND_GND,2+i*3,(unsigned char )(PWM_64_table[j]*(u_sys_param.sys_config_param.rgb_light*0.01)));   //PWM 
        I2C_WriteByte(Addr_GND_GND,3+i*3,0);  //R
      }      
      I2C_WriteByte(Addr_GND_GND,0x25,0x00);//update
      I2C_WriteByte(Addr_GND_GND,0x4B,0x01);//all channel out  freq
      I2C_WriteByte(Addr_GND_GND,0x00,0x01);// 
		//osDelay(8);//1K//看起来比较累
    //osDelay(16);//2K	
  }
}
void High_Breath(void )//呼吸一次，
{
  uint8_t i,j; 
	static uint32_t local_stick;
	static uint8_t breath_falg;
	if(osKernelGetTickCount()> local_stick+rgbBreathCountMs)
	{
		local_stick=osKernelGetTickCount();
		if(breath_falg==0)
		{
			breath_falg=1;
			for (i=0; i<12; i++) //R
			{ // R  G  B  
				I2C_WriteByte(Addr_GND_GND,1+i*3,(unsigned char )(u_sys_param.sys_config_param.rgb_light*2.55));   //B//PWM       
				I2C_WriteByte(Addr_GND_GND,2+i*3,0);   //PWM 
				I2C_WriteByte(Addr_GND_GND,3+i*3,(unsigned char )(u_sys_param.sys_config_param.rgb_light*2.55)); //R
			}    
		}
		else
		{
			breath_falg=0;
			for (i=0; i<12; i++) //R
			{ // R  G  B  
				I2C_WriteByte(Addr_GND_GND,1+i*3,0);   //B//PWM       
				I2C_WriteByte(Addr_GND_GND,2+i*3,0);   //PWM 
				I2C_WriteByte(Addr_GND_GND,3+i*3,0);  //R
			} 
    }     
    I2C_WriteByte(Addr_GND_GND,0x25,0x00);//update
    I2C_WriteByte(Addr_GND_GND,0x4B,0x01);//all channel out  freq
    I2C_WriteByte(Addr_GND_GND,0x00,0x01);//		
	}
}
void IS31FL3236A_Init(void)
{
    uint8_t i;
    //for (i=0x01; i<0x25; i++)
    //{
     // I2C_WriteByte(Addr_GND_GND,i,0xFF);   //PWM      
    //}
    is_12_all_gLED();
    HAL_Delay(10);
    for(i=0x26; i<0x4A; i++)
    {
      I2C_WriteByte(Addr_GND_GND,i,0x01);//on LED      
    }
    HAL_Delay(10);
    I2C_WriteByte(Addr_GND_GND,0x25,0x00);//update
    I2C_WriteByte(Addr_GND_GND,0x4B,0x01);
    I2C_WriteByte(Addr_GND_GND,0x00,0x01);//
    HAL_Delay(10);
}
//all rgb 
void is_12_all_rgb(unsigned short int rgbValue)//混合色
{
   uint8_t   i,R,G,B;
    R=((rgbValue>>11)&0x1F)*(u_sys_param.sys_config_param.rgb_light*0.01);
    G=((rgbValue>>5)&0x3F)*(u_sys_param.sys_config_param.rgb_light*0.01);
    B=(rgbValue&0x1F)*(u_sys_param.sys_config_param.rgb_light*0.01);
    for (i=0; i<12; i++)
    {			//PWM
			I2C_WriteByte(Addr_GND_GND,0x01+i*3,B);
			I2C_WriteByte(Addr_GND_GND,0x02+i*3,G);
			I2C_WriteByte(Addr_GND_GND,0x03+i*3,R);		
    }		
    I2C_WriteByte(Addr_GND_GND,0x25,0x00);//update
    I2C_WriteByte(Addr_GND_GND,0x4B,0x01);
    I2C_WriteByte(Addr_GND_GND,0x00,0x01);//
}
//all g 
void is_12_all_gLED(void)
{
    uint8_t   i;
    for (i=0; i<12; i++)
    {			//PWM
			I2C_WriteByte(Addr_GND_GND,0x01+i*3,0x00);
			I2C_WriteByte(Addr_GND_GND,0x02+i*3,u_sys_param.sys_config_param.rgb_light*2.55);//0x41);
			I2C_WriteByte(Addr_GND_GND,0x03+i*3,0x00);		
    }
		I2C_WriteByte(Addr_GND_GND,0x25,0x00);//update
}
//all r
void is_12_all_rLED(void)
{
	uint8_t   i;  
	for (i=0; i<12; i++)
	{			//PWM
		I2C_WriteByte(Addr_GND_GND,0x01+i*3,0x00);
		I2C_WriteByte(Addr_GND_GND,0x02+i*3,0x00);
		I2C_WriteByte(Addr_GND_GND,0x03+i*3,u_sys_param.sys_config_param.rgb_light*2.55);//0x41);		
	}
	I2C_WriteByte(Addr_GND_GND,0x25,0x00);//update
}
//all b
void is_12_all_bLED(void)
{
	uint8_t   i;
	for (i=0; i<12; i++)
	{			//PWM
		I2C_WriteByte(Addr_GND_GND,0x01+i*3,u_sys_param.sys_config_param.rgb_light*2.55);//0x41);
		I2C_WriteByte(Addr_GND_GND,0x02+i*3,0x00);
		I2C_WriteByte(Addr_GND_GND,0x03+i*3,0x00);		
	}
	I2C_WriteByte(Addr_GND_GND,0x25,0x00);//update	
}
void Reset_Register(void)
{
	I2C_WriteByte(Addr_GND_GND,0x4F,0x00);//reset
}
 #endif