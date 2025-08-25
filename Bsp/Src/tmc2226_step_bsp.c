/**
 * @file tmc2226_step_bsp.c
 *
 * @brief 
 * @warning This software tmc2226 Drivers
 *  KCM΢���䶯��
 *
 */
#include "tmc2226_step_bsp.h"
#include "main.h"
#include "usart.h"

#ifndef TMC_USART_USED
/******************LSB**************/
#define TMC_USART_USED 
#define TMC_USART_FRAME_HEAD  0x05  
#define TMC_SLAVE_ADDR        0x00   // MS1(bit0) MS2(bit1)(0x00,0x01,0x02 0x03)(0~3)
#define TMC_USART_WRITE_MASK  0x80
#define TMC_USART_READ_MASK   0x00

//GENERAL CONFIGURATION REGISTERS (0X00��0X0F)
#define TMC_USART_GCONF_REG         0x00
#define TMC_USART_GSTAT_REG         0x01
#define TMC_USART_IFCNT_REG         0x02
#define TMC_USART_SLAVECONF_REG     0x03
#define TMC_USART_OTP_PROG_REG      0x04
#define TMC_USART_OTP_READ_REG      0x05
#define TMC_USART_IOIN_REG          0x06
#define TMC_USART_FACTORY_CONF_REG  0x07
//VELOCITY DEPENDENT DRIVER FEATURE CONTROL REGISTER SET (0X10��0X1F)
#define TMC_USART_FIHOLD_IRUN_REG   0x10
#define TMC_USART_TPOWER DOWN_REG   0x11
#define TMC_USART_TSTEP_REG         0x12
#define TMC_USART_TPWMTHRS_REG      0x13
#define TMC_USART_VACTUAL_REG       0x14
//COOLSTEP AND STALLGUARDCONTROL REGISTER SET (0X14,0X40��0X42)
#define TMC_USART_TCOOLTHRS_REG     0x14
#define TMC_USART_SGTHRS_REG        0x40
#define TMC_USART_SG_RESULT_REG     0x41
#define TMC_USART_COOLCONF_REG      0x42
//MICROSTEPPING CONTROL REGISTER SET (0X60��0X6B)
#define TMC_USART_MSCNT_REG         0x6A
#define TMC_USART_MSCURACT_REG      0x6B
//DRIVER REGISTER SET (0X6C��0X7F)
#define TMC_USART_HOPCONF_REG       0x6C
#define TMC_USART_DRV_STATUS_REG    0x6F
#define TMC_USART_PWMCONF_REG       0x70
#define TMC_USART_PWM_SCALE_REG     0x71
#define TMC_USART_PWM_AUTO_REG      0x72

typedef struct {
  unsigned char frameHead;
  unsigned char SlaveAddr;
  unsigned char reg;  
  unsigned char data[4];
  unsigned char crc;
}TMC_USART_FRAME;
#endif

#define TMC_ONE_CIRCLE_STEPS  200// �����1.8��
typedef struct {
  unsigned char run;//0，stop;1 low;2,mid,3high;
  unsigned char dir;  
  unsigned char errStatus;
  unsigned char  rdb_speed;
  unsigned int  pulse_count;//
}TMC_INFO;
static TMC_INFO tmc2226_rdb_info;
static unsigned char tmc_usart_config[8]={0};
static unsigned short int tmc_speed_list6[6]={100,150,200,250,300,350};//rpm  ,50ml/Min ~200ml/min
extern TIM_HandleTypeDef htim7;
//extern UART_HandleTypeDef huart10;;
// usart change speed
//??? = ?8 +?2 + ?1 + ?0
/**
  * @brief swuart_calcCRC
  * 
  * @param  unsigned char* datagram, unsigned char datagramLength
  * @note   crc 
  * @retval None
  */
 void swuart_calcCRC(unsigned char* datagram, unsigned char datagramLength)
 {
  int i,j;
  unsigned char* crc = datagram + (datagramLength-1); // CRC located in last byte of message
  unsigned char currentByte;
  *crc = 0;
  for (i=0; i<(datagramLength-1); i++) 
  { // Execute for all bytes of a message
    currentByte = datagram[i]; // Retrieve a byte to be sent from Array
    for (j=0; j<8; j++) 
    {
      if ((*crc >> 7) ^ (currentByte&0x01)) // update CRC based result of XOR operation
      {
        *crc = (*crc << 1) ^ 0x07;
      }
      else
      {
        *crc = (*crc << 1);
      }
      currentByte = currentByte >> 1;
    } // for CRC bit
  } // for message byte  
 }

/**
  * @brief tmc2226_en
  * @param  void
  * @note   蠕动泵使能
  * @retval None
  */
 void tmc2226_en(unsigned  char en)
 {
  if(en!=0)  
  {
    HAL_GPIO_WritePin(TMC2226_EN_out_GPIO_Port, TMC2226_EN_out_Pin, GPIO_PIN_RESET);    
  } 
  else 
  {
    HAL_GPIO_WritePin(TMC2226_EN_out_GPIO_Port, TMC2226_EN_out_Pin, GPIO_PIN_SET);   
  }  
 }
 /**
  * @brief  void tmc2226_param_default(void)

  * @param  void
  * @note  蠕动泵默认参数
  * @retval None
  */
 void tmc2226_param_default(void)
 {
  tmc2226_rdb_info.run=0;
  tmc2226_rdb_info.dir =0;
  tmc2226_rdb_info.errStatus=0;
  tmc2226_rdb_info.rdb_speed = u_sys_param.sys_config_param.laser_config_param.t_water_low;
  if(tmc2226_rdb_info.rdb_speed<5) tmc2226_rdb_info.rdb_speed=5;
  if(tmc2226_rdb_info.rdb_speed>35) tmc2226_rdb_info.rdb_speed=35;
  tmc2226_rdb_info.pulse_count=0;//用于脉冲计数
 }
/**
  * @brief tmc2226_dir
  * @param  void
  * @note  蠕动泵方向
  * @retval None
  */
 void tmc2226_dir(unsigned  char dir)
 {
//  if(dir==0)  HAL_GPIO_WritePin(TMC2226_DIR_out_GPIO_Port, TMC2226_DIR_out_Pin, GPIO_PIN_RESET);
//  else HAL_GPIO_WritePin(TMC2226_DIR_out_GPIO_Port, TMC2226_DIR_out_Pin, GPIO_PIN_SET); 
	//固定	 
	 HAL_GPIO_WritePin(TMC2226_DIR_out_GPIO_Port, TMC2226_DIR_out_Pin, GPIO_PIN_RESET);
 }
 
/**
  * @brief temc2226_init
  * @param  void
  * @note   蠕动泵电机驱动初始化
  * @retval None
  */
 void tmc2226_init(void)
 {
    tmc2226_param_default(); 
    tmc2226_stop();//art(0,100);     
 }
/**
  * @brief app_steps_pulse
  * @param  void
  * @note   蠕动泵步数
  * @retval None
  */
void app_steps_pulse(int steps)
 { 
  static unsigned int timeout;
  if(tmc2226_rdb_info.run!=0)
  {    
    if(steps==CONTINUOUS_STEPS_COUNT)
    {
      HAL_GPIO_TogglePin(TMC2226_STEP_out_GPIO_Port, TMC2226_STEP_out_Pin);       
      tmc2226_rdb_info.pulse_count++;
    }
    else if(steps==0)
    {
      tmc2226_stop(); 
    }
    else 
    {      
      if(tmc2226_rdb_info.run==0) 
      {
        tmc2226_start(0,100); 
        tmc2226_rdb_info.run=1;
      }
      //if(tmc2226_rdb_info.A_phase_count*200<steps)
      if(tmc2226_rdb_info.pulse_count*200<steps)
      {
        HAL_GPIO_TogglePin(TMC2226_STEP_out_GPIO_Port, TMC2226_STEP_out_Pin); 
      }
      else tmc2226_stop();          
    }   
  }
 }

 /**
  * @brief tmc2226_start
  * @param  unsigned char dir,unsigned short int speed
  * @note   启动蠕动泵电机 ,MAX（ (timeUs=62.5)8K) 35ml/min;）
  *   timeUs=500;5ml/min;timeUs=100;20ml/min; timeUs=62;35ml/min;
  * @retval None
  */
void tmc2226_start(unsigned char dir,unsigned short int spdLevel)
{
	unsigned short int timeUs;
	
	//check status ,error status	
	if(spdLevel==0)
	{
		tmc2226_stop();
	}
	else 
	{
		tmc2226_dir(dir);
    if(tmc2226_rdb_info.rdb_speed<0) tmc2226_rdb_info.rdb_speed=5;// (5~35)
    if(tmc2226_rdb_info.rdb_speed>35)  tmc2226_rdb_info.rdb_speed=35;
		if(spdLevel==1)//5~20ml/min (1K~4.5k)(111~500)
		{	 
      //timeUs=500;//1K,5ml/min   
      //timeUs=500/((8/35)*tmc2226_rdb_info.rdb_speed);
      tmc2226_rdb_info.rdb_speed=u_sys_param.sys_config_param.laser_config_param.t_water_low;//15;//ml/min
		}
		else if(spdLevel==2)//20~30ml/min(4.5k~6.83k)(73~111)
		{
			//timeUs=125;//4k//250/2k  (4K)17.5ml/min 
      tmc2226_rdb_info.rdb_speed=u_sys_param.sys_config_param.laser_config_param.t_water_mid;//20;//ml/min
		}
		else //if(spdLevel==3)//30~40ml/min   35ml/min (6.83k~8K)
		{			
      //timeUs=65;//8K 35ml/min
      tmc2226_rdb_info.rdb_speed=u_sys_param.sys_config_param.laser_config_param.t_water_high;//35;//ml/min
		}
    timeUs=2275/tmc2226_rdb_info.rdb_speed;
		//1000·100 freq =1K ~8K
   // timeUs 500~62;//8K 		
		__HAL_TIM_SetAutoreload(&htim7,timeUs-1);
		tmc2226_rdb_info.run=spdLevel;
		HAL_TIM_Base_Start_IT(&htim7);		
		tmc2226_en(1);
	}
}
  /**
  * @brief 停止蠕动泵
  * @param  void
  * @note   停止蠕动泵电机
  * @retval 本次运行时间单位S
  */
  unsigned int  tmc2226_stop(void)
  {
    unsigned int retS;
    HAL_TIM_Base_Stop_IT(&htim7); 
    tmc2226_rdb_info.run=0;	
    tmc2226_en(0);
    //timeus=2275/tmc2226_rdb_info.rdb_speed;    
    retS=(tmc2226_rdb_info.pulse_count*0.002275)/tmc2226_rdb_info.rdb_speed;//S 
    tmc2226_rdb_info.pulse_count=tmc2226_rdb_info.pulse_count-retS*1000000;
    u_sys_param.sys_config_param.laser_config_param.RDB_use_timeS+=retS;
    HAL_Delay(1);
    return retS;
  } 
 /************************************************************************//**
  * @brief 设置蠕动泵速度等级
 * @param spdLevel: 0,1,2,3
  * @note   0关闭  3最快
   
  * @retval 无
  *****************************************************************************/
 void app_tmc2226_sped_set(unsigned char spdLevel)
 {
		 if(spdLevel==0)
		 {
				tmc2226_stop();
		 }
		 else 
		 {
			 if(spdLevel==1)//低速
			 {        
				 tmc2226_rdb_info.rdb_speed=u_sys_param.sys_config_param.laser_config_param.t_water_low;//20;//ml/min         
			 }
			 else if(spdLevel==2)
			 {
				  tmc2226_rdb_info.rdb_speed=u_sys_param.sys_config_param.laser_config_param.t_water_mid;//25;//ml/min 
			 }
			 else //if(spdLevel==3)//高速
			 {
				  tmc2226_rdb_info.rdb_speed=u_sys_param.sys_config_param.laser_config_param.t_water_high;//32;//ml/min 
			 }
		 }
 }
