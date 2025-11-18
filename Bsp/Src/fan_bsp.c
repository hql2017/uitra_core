/************************************ 
 * @file fan_bsp.c *
 * @brief 
 * @warning This fan Drivers
 */
#include "fan_bsp.h"
#include "main.h"
#include "tim.h"

#define FAN_CLK_FREQ  10000//10M 9000//9M
#define FAN_SPD_FREQ  25// 10//10K//1~20K
 typedef struct
 {
	unsigned char runflag;//0 全部关闭；1 开FAN1；2开FAN2；3全开
	unsigned short int  fan_set_speed[2]; 
	unsigned short int  fan_speed[2]; 
	unsigned short int  fan_count_freq[2]; 
	unsigned short int  fan_duty_count[2];           //占空比计数
	unsigned char fantimeOut[2];//丢计数 
 }fan_param;//
static  fan_param fanParam;
typedef enum{  
	FAN_SPD_0RPM=0,
	FAN_SPD_1000RPM,//1
	FAN_SPD_2000RPM,//2
	FAN_SPD_3000RPM,
	FAN_SPD_4000RPM,
	FAN_SPD_5000RPM,
	FAN_SPD_6000RPM,
	FAN_SPD_7000RPM,
	FAN_SPD_8000RPM,
	FAN_SPD_9000RPM,
	FAN_SPD_10000RPM,		
	FAN_SPD_11000RPM,	 //11
	FAN_SPD_MAX,	 //12
}fan_spd;// 
 //static unsigned short int fan_duty_buff[13]={0,20,112,148,192,254,324,390,479,559,640,710,890};//0,0,2000~11000RPM,890 ，10k
 
 //static const unsigned short int fan_duty_buff[13]={0,40,80,120,160,200,240,280,320,360,360,360,360};//0,1,500~6400RPM,25K
   static unsigned short int fan_duty_buff[13]={0,10, 17, 40, 65, 90, 98, 98, 98, 98, 98, 99, 100 };//%fan25
   static unsigned short int fan2_38_duty_buff[13]={0, 20, 32, 57, 78, 97, 98, 98, 98, 98, 99, 100,100 };//%fan38
 /************************************************************************//**
  * @brief 
  * @param 无
  * @note   
  * @retval 
  *****************************************************************************/
void app_fan_feed_count(unsigned char fan_number)
{	
	static unsigned int local_cnt[2];
	unsigned int temp;
	unsigned char fan_num;
	if(fan_number==1||fan_number==2)	
	{	
		fan_num=fan_number%2;
		if(DWT->CYCCNT>local_cnt[fan_num])
		{
			temp=DWT->CYCCNT-local_cnt[fan_num];
			fanParam.fan_count_freq[fan_num]=SystemCoreClock/temp;//	freq	
		}
		else if(DWT->CYCCNT<local_cnt[fan_num])
		{
			temp=DWT->CYCCNT+(0xFFFFFFFF-local_cnt[fan_num]);
			fanParam.fan_count_freq[fan_num]=SystemCoreClock/temp;//	freq
		}	
		local_cnt[fan_num]  = DWT->CYCCNT;
		fanParam.fantimeOut[fan_num]=0;	
	}
	else  
	{
		local_cnt[1] =	DWT->CYCCNT;
		local_cnt[0] =	DWT->CYCCNT;
	}	

}
  /************************************************************************//**
  * @brief 
  * @param 无
  * @note   
  * @retval 
  *****************************************************************************/
static unsigned int get_fan_count(unsigned char fan_num)
{	
	if(fan_num==1)
	{
		return fanParam.fan_count_freq[1];		
	}
	else
	{
		return fanParam.fan_count_freq[0];
	}	
}
/************************************************************************//**
  * @brief 微调
  * @param 无
  * @note   
  * @retval 
  *****************************************************************************/
 static void fan_micro_pwm_pid(unsigned char fanNum ,unsigned short int  timeCount)
 {	
	if(fanNum==1) 
    {
      __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,timeCount-1);      
    }
    else //if(fanNum==2) 
    {
      __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3,timeCount-1);
    }     
 }
/************************************************************************//**
* @brief 
* @param 无
* @note   
* @retval 
*****************************************************************************/
void fan_spd_pid(unsigned char runFlag,unsigned int P)
{
	unsigned char temp1,temp2,spd_num,duty;
	static unsigned int d1_count,d2_count;
	temp1=runFlag&0x01;
	temp2=runFlag&0x02;	
	fanParam.fantimeOut[1]++;
	fanParam.fantimeOut[0]++;
	if(fanParam.fan_count_freq[1]>0)
	{
		if(fanParam.fantimeOut[1]>2) fanParam.fan_count_freq[1]=0;
	}
	if(fanParam.fan_count_freq[0]>0)
	{
		if(fanParam.fantimeOut[0]>2) fanParam.fan_count_freq[0]=0;
	}
	if(temp1!=0)//fan1
	{
		spd_num=fanParam.fan_set_speed[1]/1000;	
		if(fanParam.fan_speed[1]<fanParam.fan_set_speed[1]&&fanParam.fan_duty_count[1]<(fan_duty_buff[spd_num]*4))		
		{//slow start 
			fanParam.fan_duty_count[1]=(fanParam.fan_duty_count[1]+fan_duty_buff[spd_num]*4)>>1;
			duty=0xFF&(fanParam.fan_duty_count[1]>>2);
			app_fan_pwm_set(duty,1);	
		}
		else 
		{
			if(fanParam.fan_speed[1]!=0)
			{
				if(fanParam.fan_speed[1]+50<fanParam.fan_set_speed[1])
				{
					d1_count++;
					fan_micro_pwm_pid(1 ,fanParam.fan_duty_count[1]+d1_count);
				}
				else if(fanParam.fan_speed[1]>50+fanParam.fan_set_speed[1])
				{
					if(d1_count>0) d1_count--;
					fan_micro_pwm_pid(1 ,fanParam.fan_duty_count[1]+d1_count);
				}
			}
		}
	}	
	if(temp2!=0)//fan2
	{
		spd_num=fanParam.fan_set_speed[0]/1000;	
		if(fanParam.fan_speed[0]<fanParam.fan_set_speed[0]&&fanParam.fan_duty_count[0]<(fan2_38_duty_buff[spd_num]*4))		
		{//slow start 
			fanParam.fan_duty_count[0]=(fanParam.fan_duty_count[0]+fan2_38_duty_buff[spd_num]*4)>>1;
			duty=0xFF&(fanParam.fan_duty_count[0]>>2);
			app_fan_pwm_set(duty,2);	
		}
		else 
		{
			if(fanParam.fan_speed[1]!=0)
			{
				if(fanParam.fan_speed[0]+50<fanParam.fan_set_speed[0])
				{
					d2_count++;
					fan_micro_pwm_pid(2,fanParam.fan_duty_count[0]+d2_count);
				}
				else if(fanParam.fan_speed[0]>50+fanParam.fan_set_speed[0])
				{
					if(d2_count>0) d2_count--;
					fan_micro_pwm_pid(2 ,fanParam.fan_duty_count[0]+d2_count);
				}
			}
		}
	}	
}
/************************************************************************//**
* @brief 
* @param 无
* @note   
* @retval 
*****************************************************************************/
static void get_fan_spd_calcu(unsigned int run_tick)
{
	fanParam.fan_speed[1]= get_fan_count( FAN25_NUM )*30;	//spd=count_freq*30;
	fanParam.fan_speed[0]= get_fan_count(FAN38_COMPRESSOR_NUM )*30; //spd=count_freq*30
	if(fanParam.runflag!=0)
	{ 
		fan_spd_pid(fanParam.runflag,30);		
		//DEBUG_PRINTF("fan_spd1=%d duty=%d\r\n",fanParam.fan_speed[1],fanParam.fan_duty_count[1]);
		//DEBUG_PRINTF("fan_spd2=%d duty=%d\r\n",fanParam.fan_speed[0],fanParam.fan_duty_count[0]);
	}
}
/************************************************************************//**
* @brief 
* @param 无
* @note   
* @retval 
*****************************************************************************/
void fan_spd_set(unsigned char fanNumber,unsigned int spd)
{	
	unsigned short int spd_num;	
	unsigned char  fan_num1,fan_num2;	
	if(spd==0)
	{
		fan_stop(fanNumber);	
	}
	else 
	{	
		fan_num1=fanNumber&0x01;
		fan_num2=fanNumber&0x02;
		spd_num=spd/1000;
		if(fan_num1!=0)
		{
			if(spd<2000)			fanParam.fan_set_speed[1]=2000;
			else if(spd>6000)		fanParam.fan_set_speed[1]=6000;
			else fanParam.fan_set_speed[1]=spd;					
		}
		else 
		{
			fan_stop(1);
		}		
		if(fan_num2!=0)
		{
			if(spd<2000)			fanParam.fan_set_speed[0]=2000;
			else if(spd>6000)		fanParam.fan_set_speed[0]=6000;
			else fanParam.fan_set_speed[0]=spd;						
		}
		else 
		{
			fan_stop(2);
		}					
	}
}
/************************************************************************//**
* @brief 
* @param 无
* @note   
* @retval 
*****************************************************************************/
void fan_start(unsigned char runflag)
{		
	unsigned int  period,timeDuty;
	unsigned char startDuty=10;
	unsigned char  fan_num1,fan_num2;
	if(runflag==0||runflag>3)
	{
		fan_stop(3);
		fanParam.runflag=0;		
	}
	else 
	{		
		fan_num1 =	runflag&0x01;
		fan_num2 =	runflag&0x02;
        if(fan_num1!=0)
		{
			app_fan_feed_count(0);
			app_fan_pwm_set(startDuty,1);
		}
		else 
		{
			fan_stop(1);
		}
		if(fan_num2!=0)
		{
			app_fan_feed_count(0);
			app_fan_pwm_set(startDuty,2);
		}
		else 
		{
			fan_stop(2);
		}
		fanParam.runflag=runflag;
	}
}
/************************************************************************//**
* @brief 
* @param 无
* @note   
* @retval 
*****************************************************************************/
void fan_stop(unsigned char stopFlag)
{	
	if(stopFlag==1)
	{
		app_fan_pwm_set(0,1);
		fanParam.fan_speed[1]=0;
		fanParam.fan_count_freq[1]=0;
		fanParam.runflag&=0x02;			
	}
	else if(stopFlag==2)
	{	
		app_fan_pwm_set(0,2);
		fanParam.fan_speed[0]=0;
		fanParam.fan_count_freq[0]=0;
		fanParam.runflag&=0x01;
	}
	else 
	{
		app_fan_pwm_set(0,1);
		app_fan_pwm_set(0,2);	
		fanParam.fan_speed[1]=0;
		fanParam.fan_speed[0]=0;
		fanParam.fan_count_freq[1]=0;
		fanParam.fan_count_freq[0]=0;		
		fanParam.runflag=0;
	}	
	delay_us(5);	
}
/************************************************************************//**
* @brief 
* @param 无
* @note   
* @retval run speed
*****************************************************************************/
unsigned short int fan_get_run_spd(unsigned char fanNumber)
{	
	if(fanNumber==1)
	{
		return fanParam.fan_speed[1];
	}
	else
	{		
		return fanParam.fan_speed[0];
	} 
}
/************************************************************************//**
* @brief 
* @param 无
* @note   
* @retval 
*****************************************************************************/
void fan_init(void)
{
	fan_spd_set(FAN25_NUM,3000);
	fan_spd_set(FAN38_COMPRESSOR_NUM,3000);
	fan_start(3);
}
/************************************************************************//**
* @brief 
* @param 无
* @note   
* @retval 
*****************************************************************************/
void app_fan_manage(unsigned int systick)
{		
	get_fan_spd_calcu(systick);			
}
