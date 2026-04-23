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
#define FAN_MAX_FREQ  167//FAN_MAX_FREQ*30=5000rpm
 typedef struct
 {
	unsigned char runflag;//0 全部关闭；1 开FAN1；2开FAN2；3全开
	unsigned short int  fan_set_speed[2]; //0rpm~5000rpm
	unsigned short int  fan_speed[2]; 
	unsigned short int  fan_pulse_count[2]; //max 166HZ 6ms~1000ms
	unsigned short int  fan_duty_count[2];           //占空比计数
	unsigned short int  fanFreq[2];//
 }fan_param;//
static  fan_param fanParam;
typedef enum{  
	FAN_SPD_0RPM=0,
	FAN_SPD_1000RPM,//1
	FAN_SPD_2000RPM,//2
	FAN_SPD_3000RPM,
	FAN_SPD_4000RPM,
	FAN_SPD_MAX,
	FAN_SPD_5000RPM,		 
	FAN_SPD_6000RPM,
	FAN_SPD_7000RPM,
	FAN_SPD_8000RPM,
	FAN_SPD_9000RPM,
	FAN_SPD_10000RPM,		
	FAN_SPD_11000RPM,	 //11	
}fan_spd;// 
 //static unsigned short int fan_duty_buff[13]={0,20,112,148,192,254,324,390,479,559,640,710,890};//0,0,2000~11000RPM,890 ，10k 
 //static const unsigned short int fan_duty_buff[13]={0,40,76,120,160,200,240,280,320,360,360,360,360};//0,1500~5100RPM,25K
	static unsigned short int fan_duty_buff[13]=     {0,10, 14, 38, 65, 86, 86, 86, 86, 86, 86, 86, 86 };//%fan25
	#if 1
   	static unsigned short int fan2_38_duty_buff[13]= {0,20, 38, 53, 74, 90, 90, 90, 90, 90, 90, 90, 90 };//%fan38
	 #else
	 //通电就转最低1710rpm duty0~100；
	 static unsigned short int fan2_38_duty_buff[13]={0,10, 15,38, 65,  86, 86, 86, 86, 86, 86, 86, 86};//%fan38
	 #endif
#ifdef  FAN_PID_ENABLE 
#define  FAN_PID_ENABLE 
	/*****************PID-start************************************ */
  // PID控制器结构体
typedef struct {
    double kp;              // 比例增益
    double ki;              // 积分增益
    double kd;              // 微分增益
    double setpoint;        // 设定值
    double integral;        // 积分累积值
    double previous_error;  // 上次误差值
    double output_min;      // 输出最小值
    double output_max;      // 输出最大值
} PIDController;

static PIDController fan_pid[2];
//static   double target_speed = 2000.0;  // 目标转速 2000 RPM
//static    double current_speed = 0.0;    // 当前转速
static   double pwm_output = 0.0;       // PWM输出值
static   double time_step = 0.1;        // 时间步长 100ms
/************************************************************************//**
  * @brief 初始化PID控制器
  * @param 无
  * @note   
  * @retval 
  *****************************************************************************/
void pid_init(PIDController *pid, double kp, double ki, double kd, double setpoint) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->setpoint = setpoint;
    pid->integral = 0.0;
    pid->previous_error = 0.0;
    pid->output_min = 0.0;
    pid->output_max = 400.0;//100.0;  // PWM占空比最大值为100%
}

/************************************************************************//**
  * @brief 计算PID输出
  * @param 无
  * @note   
  * @retval 
  *****************************************************************************/
 //pid_init(&fan_pid[1], 0.08, 0.2, 0.05, fanParam.fan_set_speed[1]*1.0);
double pid_compute(PIDController *pid, double feedback, double dt) {
    // 计算当前误差
    double error = pid->setpoint - feedback;
    
    // 比例项
    double proportional = pid->kp * error;
    
    // 积分项
    pid->integral += error * dt;
    double integral = pid->ki * pid->integral;
    
    // 微分项
    double derivative = pid->kd * (error - pid->previous_error) / dt;
    
    // 更新上次误差
    pid->previous_error = error;
    
    // 计算总输出
    double output = proportional + integral + derivative;
    
    // 输出限幅
    if (output > pid->output_max) {
        output = pid->output_max;
        // 抗积分饱和
        if (error * pid->output_max > 0) {
            pid->integral -= error * dt;
        }
    } else if (output < pid->output_min) {
        output = pid->output_min;
        // 抗积分饱和
        if (error * pid->output_min < 0) {
            pid->integral -= error * dt;
        }
    }
    
    return output;
}


/************************************************************************//**
  * @brief 重置PID控制器
  * @param 无
  * @note   
  * @retval 
  *****************************************************************************/
void pid_reset(PIDController *pid) {
    pid->integral = 0.0;
    pid->previous_error = 0.0;
}

/************************************************************************//**
  * @brief 设置输出限制
  * @param 无
  * @note   
  * @retval 
  *****************************************************************************/
void pid_set_output_limits(PIDController *pid, double min, double max) {
    if (min >= max) return;
    pid->output_min = min;
    pid->output_max = max;
}


/************************************************************************//**
  * @brief // 设置PID参数
  * @param 无
  * @note   
  * @retval 
  *****************************************************************************/
void pid_set_tunings(PIDController *pid, double kp, double ki, double kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}
#endif
/*****************PID-END************************************ */
 /************************************************************************//**
  * @brief 
  * @param 无
  * @note   
  * @retval 
  *****************************************************************************/
void app_fan_feed_count(unsigned char fan_number)
{	
	unsigned char fan_index;
	if(fan_number==1||fan_number==2)	
	{	
		fan_index=fan_number&0x01;//fan_number%2;	
		fanParam.fan_pulse_count[fan_index]++;	
	}		
	else {	
		fanParam.fan_pulse_count[1]=0;//clear
		fanParam.fan_pulse_count[0]=0;
	}
		
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
 void cacul_fan_freq(unsigned int timeMs)
{
	unsigned char ret_freq;
	if(fanParam.fan_pulse_count[1]>200)
	{
		fanParam.fanFreq[1]=200;
	}
	else fanParam.fanFreq[1]=fanParam.fan_pulse_count[1];//*1000/timeMs;
	if(fanParam.fan_pulse_count[0]>200)
	{
		fanParam.fanFreq[0]=200;
	}
	else fanParam.fanFreq[0]=fanParam.fan_pulse_count[0];//*1000/timeMs;
	fanParam.fan_pulse_count[1]=0;//clear
	fanParam.fan_pulse_count[0]=0;//clear
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
       if(timeCount<2) timeCount=2;
      __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,timeCount-1);      
    }
    else //if(fanNum==2) 
    {
		if(timeCount<2) timeCount=2;
      __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3,timeCount-1);
    }     
 } 
/************************************************************************//**
* @brief 
* @param 无
* @note   
* @retval 
*****************************************************************************/
#ifdef  FAN_PID_ENABLE
void fan_spd_pid(unsigned char fanNum,unsigned int timeMs)
{
    // PID输出
	unsigned char Index=(fanNum%2);	
	//float output = pid_compute(&fan_pid[Index],fanParam.fan_speed[Index]*1.0,timeMs*0.001);
	//DEBUG_PRINTF("fan_pidout=%.1f\r\n",output);		
	//fanParam.fan_duty_count[Index]=(unsigned short int)output;
	if(fanParam.fan_speed[Index]>50+fanParam.fan_set_speed[Index])
	{
		if(fanParam.fan_duty_count[Index]>16)fanParam.fan_duty_count[Index]-=2;
		app_fan_pwm_set(fanParam.fan_duty_count[Index],fanNum);
	}
	else if(fanParam.fan_speed[Index]+50<fanParam.fan_set_speed[Index])
	{
		if(fanParam.fan_duty_count[Index]<400)fanParam.fan_duty_count[Index]+=2;
		app_fan_pwm_set(fanParam.fan_duty_count[Index],fanNum);
	}
}
#else 
void fan_spd_pid(unsigned char runFlag,unsigned int P)
{
	unsigned char Index=(runFlag%2);		
	if(fanParam.fan_speed[Index]>30+fanParam.fan_set_speed[Index])
	{
		if(fanParam.fan_speed[Index]>510+fanParam.fan_set_speed[Index])
		{
			if(fanParam.fan_duty_count[Index]>32)fanParam.fan_duty_count[Index]-=16;
			app_fan_pwm_set(fanParam.fan_duty_count[Index],runFlag);
		}
		if(fanParam.fan_speed[Index]>240+fanParam.fan_set_speed[Index])
		{
			if(fanParam.fan_duty_count[Index]>8)fanParam.fan_duty_count[Index]-=8;
			app_fan_pwm_set(fanParam.fan_duty_count[Index],runFlag);
		}
		else 
		{
			if(fanParam.fan_duty_count[Index]>2)fanParam.fan_duty_count[Index]--;
			app_fan_pwm_set(fanParam.fan_duty_count[Index],runFlag);
		}
	}
	else if(fanParam.fan_speed[Index]+30<fanParam.fan_set_speed[Index])
	{
		if(fanParam.fan_speed[Index]+510<fanParam.fan_set_speed[Index])
		{
			if(fanParam.fan_duty_count[Index]<384)
			{
				fanParam.fan_duty_count[Index]+=16;
			}
			app_fan_pwm_set(fanParam.fan_duty_count[Index],runFlag);
		}
		else if(fanParam.fan_speed[Index]+240<fanParam.fan_set_speed[Index])
		{
			if(fanParam.fan_duty_count[Index]<384)
			{
				fanParam.fan_duty_count[Index]+=8;
			}
			app_fan_pwm_set(fanParam.fan_duty_count[Index],runFlag);
		}
		else {
			if(fanParam.fan_duty_count[Index]<400)fanParam.fan_duty_count[Index]++;
			app_fan_pwm_set(fanParam.fan_duty_count[Index],runFlag);
		}
	}	
}
	
#endif
/************************************************************************//**
* @brief 
* @param 无
* @note   
* @retval 
*****************************************************************************/
static void get_fan_spd_calcu(unsigned int timeMs)
{
	unsigned char fanNum1;
	unsigned char fanNum2;
	fanNum1=fanParam.runflag&0x01;
	fanNum2=fanParam.runflag&0x02;
	if(fanNum1!=0)
	{ 
		fanParam.fan_speed[1]=30*fanParam.fanFreq[1];//get_fan_freq(1,1000);	//spd=count_freq*30;
		fan_spd_pid(1,timeMs);
		//DEBUG_PRINTF("fan_spd1=%d duty=%d\r\n",fanParam.fan_speed[1],__HAL_TIM_GET_COMPARE(&htim8,TIM_CHANNEL_2));		
	}
	if(fanNum2!=0)
	{ 
		fanParam.fan_speed[0]=30*fanParam.fanFreq[0];//get_fan_freq(2,1000);;	//spd=count_freq*30;
		fan_spd_pid(2,timeMs);		
		//DEBUG_PRINTF("fan_spd2=%d duty=%d\r\n",fanParam.fan_speed[0],__HAL_TIM_GET_COMPARE(&htim8,TIM_CHANNEL_3));
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
	unsigned char fan_index;
		
	  // 输入参数规范化处理
	  if (spd == 0) {
        fan_stop(fanNumber);
        return;
    }
    // 风扇目标转速限制在有效范围内
    if (spd < 1000) spd = 1000;
    else if (spd > 5000) spd = 5000;
	unsigned char spd_index=spd/1000;
    switch (fanNumber) {
        case 1:
            fanParam.fan_set_speed[1] = spd;
			
            break;
        case 2:
            fanParam.fan_set_speed[0] = spd;
            break;
        default:
            // 设置全部风扇的速度
            fanParam.fan_set_speed[0] = spd;
            fanParam.fan_set_speed[1] = spd;
            break;
    }
	#ifdef  FAN_PID_ENABLE     
		if(fanNumber==1)	{
			pid_set_output_limits(&fan_pid[1], 0, fan_duty_buff[spd_index]*4.0);
		}
		else if(fanNumber==2){
			pid_set_output_limits(&fan_pid[0], 0, fan2_38_duty_buff[spd_index]*4.0);
		}
		else {
			pid_set_output_limits(&fan_pid[1], 0, fan_duty_buff[spd_index]*4.0);
			pid_set_output_limits(&fan_pid[0], 0, fan2_38_duty_buff[spd_index]*4.0);
		}	
		#endif
}
/************************************************************************//**
* @brief 
* @param 无
* @note   
* @retval 
*****************************************************************************/
void fan_start(unsigned char fanNum)
{		
	unsigned int  period,timeDuty;	
	if(fanNum==FAN25_NUM)
	{
		fanParam.runflag|=0x01;
		HAL_TIMEx_PWMN_Start(&htim8,TIM_CHANNEL_2); 
	}
	else if(fanNum==FAN38_COMPRESSOR_NUM)
	{
		fanParam.runflag|=0x02;
		HAL_TIMEx_PWMN_Start(&htim8,TIM_CHANNEL_3); 
	}
	else 
	{
		fanParam.runflag=3;
		HAL_TIMEx_PWMN_Start(&htim8,TIM_CHANNEL_2); 
		HAL_TIMEx_PWMN_Start(&htim8,TIM_CHANNEL_3); 		
	}
	HAL_TIM_Base_Start_IT(&htim23);	
}
/************************************************************************//**
* @brief 
* @param 无
* @note   
* @retval 
*****************************************************************************/
void fan_stop(unsigned char fanNum)
{	
	if(fanNum==FAN25_NUM)
	{
		app_fan_pwm_set(0,1);
		fanParam.fan_set_speed[1]=0;
		fanParam.fan_duty_count[1]=0;	
		fanParam.runflag&=0x02;	//only stop fan1,keep fan2 run		
	}
	else if(fanNum==FAN38_COMPRESSOR_NUM)
	{	
		app_fan_pwm_set(0,2);
		fanParam.fan_set_speed[0]=0;
		fanParam.fan_duty_count[0]=0;		
		fanParam.runflag&=0x01;//only stop fan2,keep fan1 run
	}
	else 
	{
		app_fan_pwm_set(0,1);
		app_fan_pwm_set(0,2);	
		fanParam.fan_set_speed[1]=0;
		fanParam.fan_set_speed[0]=0;
		fanParam.fan_duty_count[1]=0;
		fanParam.fan_duty_count[0]=0;
		fanParam.runflag=0;
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
	fan_spd_set(FAN25_NUM,1000);
	app_fan_pwm_set(20,1);	
	fan_spd_set(FAN38_COMPRESSOR_NUM,1000);	
	app_fan_pwm_set(20,2);	
	app_fan_feed_count(3);  
	#ifdef  FAN_PID_ENABLE 
    // 初始化PID控制器	
   	pid_init(&fan_pid[1], 0.08, 0.02, 0.005, fanParam.fan_set_speed[1]*1.0);
	pid_init(&fan_pid[0], 0.08, 0.02, 0.005, fanParam.fan_set_speed[0]*1.0);	
	pid_set_output_limits(&fan_pid[1], 0, fan_duty_buff[2]*4.0);
	pid_set_output_limits(&fan_pid[0], 0, fan2_38_duty_buff[2]*4.0);
	#endif
	fan_start(3);
}
/************************************************************************//**
* @brief 
* @param 无
* @note   1sHandle
* @retval 
*****************************************************************************/
void app_fan_manage(unsigned int handleTimeMs)
{		
	get_fan_spd_calcu(handleTimeMs);			
}
