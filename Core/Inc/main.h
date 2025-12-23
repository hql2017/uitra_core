/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "common_function.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define adtest adtest
#define ADS1118_SPI4_SCK_Pin GPIO_PIN_2
#define ADS1118_SPI4_SCK_GPIO_Port GPIOE
#define ADS1118_CS_out_Pin GPIO_PIN_3
#define ADS1118_CS_out_GPIO_Port GPIOE
#define beep_pwm_Pin GPIO_PIN_4
#define beep_pwm_GPIO_Port GPIOE
#define ADS1118_SPI4_MISO_Pin GPIO_PIN_5
#define ADS1118_SPI4_MISO_GPIO_Port GPIOE
#define ADS1118_SPI4_MOSI_Pin GPIO_PIN_6
#define ADS1118_SPI4_MOSI_GPIO_Port GPIOE
#define CIRCLE_WATER_DEPTH_in_Pin GPIO_PIN_13
#define CIRCLE_WATER_DEPTH_in_GPIO_Port GPIOC
#define PWR_SYS_ON_Pin GPIO_PIN_14
#define PWR_SYS_ON_GPIO_Port GPIOC
#define water_cycle_ok_Pin GPIO_PIN_15
#define water_cycle_ok_GPIO_Port GPIOC
#define FAN1_COUNT_in_Pin GPIO_PIN_0
#define FAN1_COUNT_in_GPIO_Port GPIOC
#define FAN1_COUNT_in_EXTI_IRQn EXTI0_IRQn
#define FAN2_COUNT_in_Pin GPIO_PIN_1
#define FAN2_COUNT_in_GPIO_Port GPIOC
#define FAN2_COUNT_in_EXTI_IRQn EXTI1_IRQn
#define AD_OCP_CHANNEL_Pin GPIO_PIN_0
#define AD_OCP_CHANNEL_GPIO_Port GPIOA
#define AD_VBUS_24V_CHANNEL_Pin GPIO_PIN_1
#define AD_VBUS_24V_CHANNEL_GPIO_Port GPIOA
#define AD_TMC_NTC_CHANNEL_Pin GPIO_PIN_2
#define AD_TMC_NTC_CHANNEL_GPIO_Port GPIOA
#define AD_LASER_CHANNEL_Pin GPIO_PIN_3
#define AD_LASER_CHANNEL_GPIO_Port GPIOA
#define AD_AIR_PRESSURE_CHANNEL_Pin GPIO_PIN_4
#define AD_AIR_PRESSURE_CHANNEL_GPIO_Port GPIOA
#define RF24_SP6_SCK_Pin GPIO_PIN_5
#define RF24_SP6_SCK_GPIO_Port GPIOA
#define RF24_SP6_MISO_Pin GPIO_PIN_6
#define RF24_SP6_MISO_GPIO_Port GPIOA
#define RF24_SP6_MOSI_Pin GPIO_PIN_7
#define RF24_SP6_MOSI_GPIO_Port GPIOA
#define RF24_SP6_CS_out_Pin GPIO_PIN_4
#define RF24_SP6_CS_out_GPIO_Port GPIOC
#define RF24_SDN_out_Pin GPIO_PIN_5
#define RF24_SDN_out_GPIO_Port GPIOC
#define FAN1_TIM8CH2N_PWM_Pin GPIO_PIN_0
#define FAN1_TIM8CH2N_PWM_GPIO_Port GPIOB
#define FAN2_TIM8CH3N_PWM_Pin GPIO_PIN_1
#define FAN2_TIM8CH3N_PWM_GPIO_Port GPIOB
#define H_AIR_ERROR_Pin GPIO_PIN_2
#define H_AIR_ERROR_GPIO_Port GPIOB
#define LASER_PULSE_COUNT_in_Pin GPIO_PIN_7
#define LASER_PULSE_COUNT_in_GPIO_Port GPIOE
#define LASER_PULSE_COUNT_in_EXTI_IRQn EXTI9_5_IRQn
#define LASER_PULSE_AD_RESET_out_Pin GPIO_PIN_8
#define LASER_PULSE_AD_RESET_out_GPIO_Port GPIOE
#define auxiliary_bulb_pwm_Pin GPIO_PIN_9
#define auxiliary_bulb_pwm_GPIO_Port GPIOE
#define TREATMENT_WATER_DEPTH_in_Pin GPIO_PIN_10
#define TREATMENT_WATER_DEPTH_in_GPIO_Port GPIOE
#define REMOTE_ON_OFF_in_Pin GPIO_PIN_11
#define REMOTE_ON_OFF_in_GPIO_Port GPIOE
#define Solenoid_STATUS2_Pin GPIO_PIN_12
#define Solenoid_STATUS2_GPIO_Port GPIOE
#define Solenoid_STATUS3_Pin GPIO_PIN_13
#define Solenoid_STATUS3_GPIO_Port GPIOE
#define Solenoid_EN2_Pin GPIO_PIN_14
#define Solenoid_EN2_GPIO_Port GPIOE
#define Solenoid_EN3_Pin GPIO_PIN_15
#define Solenoid_EN3_GPIO_Port GPIOE
#define AIR_I2C2_SCL_Pin GPIO_PIN_10
#define AIR_I2C2_SCL_GPIO_Port GPIOB
#define AIR_I2C2_SDA_Pin GPIO_PIN_11
#define AIR_I2C2_SDA_GPIO_Port GPIOB
#define compressor_UART5_RX_Pin GPIO_PIN_12
#define compressor_UART5_RX_GPIO_Port GPIOB
#define compressor_UART5_TX_Pin GPIO_PIN_13
#define compressor_UART5_TX_GPIO_Port GPIOB
#define RS485_USART1_TX_Pin GPIO_PIN_14
#define RS485_USART1_TX_GPIO_Port GPIOB
#define RS485_USART1_RX_Pin GPIO_PIN_15
#define RS485_USART1_RX_GPIO_Port GPIOB
#define RS485_DIR_out_Pin GPIO_PIN_8
#define RS485_DIR_out_GPIO_Port GPIOD
#define H_AIR_EN_out_Pin GPIO_PIN_9
#define H_AIR_EN_out_GPIO_Port GPIOD
#define treatment_water_ready_ok_in_Pin GPIO_PIN_10
#define treatment_water_ready_ok_in_GPIO_Port GPIOD
#define TMC2226_DIR_out_Pin GPIO_PIN_11
#define TMC2226_DIR_out_GPIO_Port GPIOD
#define RF24_IRQ_in_Pin GPIO_PIN_12
#define RF24_IRQ_in_GPIO_Port GPIOD
#define TMC2226_index_in_Pin GPIO_PIN_13
#define TMC2226_index_in_GPIO_Port GPIOD
#define TMC2226_ERROR_out_Pin GPIO_PIN_14
#define TMC2226_ERROR_out_GPIO_Port GPIOD
#define FOOT_SWITCH_IN_Pin GPIO_PIN_15
#define FOOT_SWITCH_IN_GPIO_Port GPIOD
#define FOOT_SWITCH_IN_EXTI_IRQn EXTI15_10_IRQn
#define KEY_PWR_SWITCH_Pin GPIO_PIN_6
#define KEY_PWR_SWITCH_GPIO_Port GPIOC
#define HV_ONE_PULSE_out_Pin GPIO_PIN_7
#define HV_ONE_PULSE_out_GPIO_Port GPIOC
#define EEROM_W_EN_out_Pin GPIO_PIN_8
#define EEROM_W_EN_out_GPIO_Port GPIOC
#define EEROM_I2C3_SDA_Pin GPIO_PIN_9
#define EEROM_I2C3_SDA_GPIO_Port GPIOC
#define EEROM_I2C3_SCL_Pin GPIO_PIN_8
#define EEROM_I2C3_SCL_GPIO_Port GPIOA
#define TMC_TCD_LPUART1_TX_Pin GPIO_PIN_9
#define TMC_TCD_LPUART1_TX_GPIO_Port GPIOA
#define TMC_TCD_LPUART1_RX_Pin GPIO_PIN_10
#define TMC_TCD_LPUART1_RX_GPIO_Port GPIOA
#define LCD_12V_ON_Pin GPIO_PIN_15
#define LCD_12V_ON_GPIO_Port GPIOA
#define S31FL3193_IIC5_SDA_Pin GPIO_PIN_10
#define S31FL3193_IIC5_SDA_GPIO_Port GPIOC
#define S31FL3193_IIC5_SCL_Pin GPIO_PIN_11
#define S31FL3193_IIC5_SCL_GPIO_Port GPIOC
#define S31FL3193_SDB_out_Pin GPIO_PIN_12
#define S31FL3193_SDB_out_GPIO_Port GPIOC
#define Hyperbaria_OFF_Signal_Pin GPIO_PIN_0
#define Hyperbaria_OFF_Signal_GPIO_Port GPIOD
#define EMERGENCY_LASER_STOP_STATUS_in_Pin GPIO_PIN_1
#define EMERGENCY_LASER_STOP_STATUS_in_GPIO_Port GPIOD
#define circulating_water_pump_status_in_Pin GPIO_PIN_2
#define circulating_water_pump_status_in_GPIO_Port GPIOD
#define circulating_water_pump_EN_Pin GPIO_PIN_3
#define circulating_water_pump_EN_GPIO_Port GPIOD
#define PTC_EN_Pin GPIO_PIN_4
#define PTC_EN_GPIO_Port GPIOD
#define JDQ_READY_Pin GPIO_PIN_5
#define JDQ_READY_GPIO_Port GPIOD
#define JDQ_STAND_Pin GPIO_PIN_6
#define JDQ_STAND_GPIO_Port GPIOD
#define DAC_DIN_Pin GPIO_PIN_7
#define DAC_DIN_GPIO_Port GPIOD
#define DAC_CLK_Pin GPIO_PIN_3
#define DAC_CLK_GPIO_Port GPIOB
#define DAC_LD_Pin GPIO_PIN_4
#define DAC_LD_GPIO_Port GPIOB
#define DAC_CS_Pin GPIO_PIN_5
#define DAC_CS_GPIO_Port GPIOB
#define V160_I2C1_SCK_Pin GPIO_PIN_6
#define V160_I2C1_SCK_GPIO_Port GPIOB
#define V160_I2C1_SDA_Pin GPIO_PIN_7
#define V160_I2C1_SDA_GPIO_Port GPIOB
#define TMC_STEP_TIM16CH1_PWM_out_Pin GPIO_PIN_8
#define TMC_STEP_TIM16CH1_PWM_out_GPIO_Port GPIOB
#define TMC2226_EN_out_Pin GPIO_PIN_9
#define TMC2226_EN_out_GPIO_Port GPIOB
#define MCU_SYS_health_LED_Pin GPIO_PIN_0
#define MCU_SYS_health_LED_GPIO_Port GPIOE
#define ADS1118_DRDY_in_Pin GPIO_PIN_1
#define ADS1118_DRDY_in_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

#define MAX_TMC2226_NTC_TEMPRATURE  85.0f//85.0
//激光工作环境温度，冷却液温度（允许温差3度）
#define MIN_TEMPRATURE_LASER  -0.2f 
#define MID_TEMPRATURE_LASER  0.0f // 24.0f//最佳工作温度
#define MAX_TEMPRATURE_LASER  0.8f 

#define ERR_LOW_TEMPRATURE_LASER    -40.0f //温度低边界
#define ERR_HIGH_TEMPRATURE_LASER   100.0f//温度高边界
//air pump 气泵 kPa,相对值
#define MIN_AIR_PUMP_PRESSURE  140.0f//210.00f
#define MID_AIR_PUMP_PRESSURE  170.0f//260.00f//最佳气压
#define MAX_AIR_PUMP_PRESSURE  190.0f//280.00f

//300.00f
#define ERR_T_FLOAT_VALUE      1000///温度浮点数错误值
#define ERR_T_SHORT_INT_VALUE  160///温度整型错误值

//laser
#define LASER_980_MIN_ENERGE_V     150//mV(210mV，1W) 
#define LASER_980_MAX_ENERGE_V     1200//mV(12W,200mJ)

#define LASER_1064_MIN_ENERGE_V    7000// 440V
#define LASER_1064_MAX_ENERGE_V  13500/// 800V

#define SYS_1_SECOND_TICKS    1000//1s
#define SYS_1_MINUTES_TICK    60000//1 minete
#define LASER_MAX_CONTINUS_WORK_TIME    5*SYS_1_MINUTES_TICK

#define MAX_IBUS_MA 10000
#define MIN_VBUS_MV 10000

#define EEROM_DATA_ERR_CHECK_FLAG  0xFF

#ifndef ONE_WIRE_BUS_JT_SLAVE 
#define ONE_WIRE_BUS_JT_SLAVE 
#endif

#ifdef IWDG_USED
#define IWDG_USED  
#endif

#ifdef DEBUG_MSG_UART 
#define DEBUG_MSG_UART  /*use printf*/
#define DEBUG_PRINTF(fmt, ...) printf(fmt, ##__VA_ARGS__)
#else
#define DEBUG_PRINTF(fmt, ...) do { } while(0)
#endif

#define  SYS_LASER_CONFIG_PARAM_LENGTH   sizeof(SYS_CONFIG_PARAM)-4// 215
typedef enum{ 
  IO_KEY_IDLE=0,
  KEY_SHORT_PRESS,//短按
  KEY_LONG_PRESS,//长按2
  KEY_LONG_RELEASE,//长按释放3
  KEY_LOW_POWER,//电量低4
  KEY_NO_CONNECT//失联
}KEY_VALUE;
typedef enum{ 
  NO_KEY_MESSAGE=0,
  key_jt_short_press,
  key_jt_long_press,
  key_jt_release,
  key_jt_no_connect,
  key_pwr_short_press,
  key_pwr_long_press,
  key_pwr_release,
//key_multifunctional_press,//全部按下
}app_key_message;


typedef struct{  
		float NTC_temprature;                 //蠕动泵电机驱动IC温度，adc 
		float laser_1064_energy;              //1064激光能量计，adc   
		float iBus;                           //24V总电源电流mA，adc
		float vBus;                           //24V总电源电压mV，adc
		float air_pump_pressure;              //气泵气压,adc
	  //float eth_cool_k0_temprature;       //环境冷端补偿温度
		float eth_k1_temprature;              //热电偶K1温度
		float eth_k2_temprature;              //热电偶K2温度
		float enviroment_temprature;          //环境温度m117z
		float air_gzp_temprature;             //环境气压计温度
		float air_gzp_enviroment_pressure_kpa;//环境气压计,kpa  //平原气压约90~100kPa，西藏约80kPa
		float treatment_water_depth;          //治疗水位
    float cool_water_depth;          //冷却水位  
		//lite增加部分
		float tec_current_out;//制冷片
    unsigned int JT_ID;
    unsigned char JT_bat;//(vbat/100)
 }sys_enviroment_assistant_param;//辅助参数
 extern sys_enviroment_assistant_param sEnvParam;

 typedef struct{  
  unsigned short int energe_cali;          //）能量校准值
  unsigned short int power_cali;           //）功率校准值
}ENERGE_CALIBRATION;
typedef struct
{  //215 bytes
  unsigned char  synchronousFlag;           // 同步标志0，未同步；1下载；2上传；3已同步；EEROM_DATA_ERR_CHECK_FLAG 未同步，无屏幕通信;0xFF未初始化 
	unsigned int   equipmentId;                //设备ID
  unsigned int   jtId;                       //脚踏ID
  unsigned char  jt_status;                  //脚踏绑定状态 （0/1）
  unsigned short int cool_temprature_high;           //）冷却系统温度高阈值*10
  unsigned short int cool_temprature_target;         //）冷却系统目标温度*10
  unsigned short int cool_temprature_low;            //）冷却系统温度低阈值*10
  unsigned short int photodiod_low;         //）光电二极管校准50mj挡
  unsigned short int photodiod_mid;         //）光电二极管校准100mJ挡
  unsigned short int photodiod_high;        //）光电二极管校准200mJ挡
  unsigned  int laser_pulse_count;          //）脉冲计数（1064）
  unsigned  int laser_use_timeS;            //）连续激光使用时间S（980脉冲激光运行时间。(MAX,连续49.7天)）
  unsigned  int RDB_use_timeS;              //）蠕动泵使用时间S
  unsigned short int laser_pulse_width_us;  //）出光脉宽
  unsigned char treatment_water_depth_high; //）治疗水位高校准
  unsigned char treatment_water_depth_low;  //）治疗水位低校准
  unsigned char cool_water_depth_high;      //）冷却液位高校准
  unsigned char cool_water_depth_low;       //）冷却液位低校准  
  unsigned char air_low_pressure;           //）气1档气压
  unsigned char air_mid_pressure;           //）气2档气压
  unsigned char air_high_pressure;          //）气3档气压
  unsigned char t_water_low;                //）水1档流量
  unsigned char t_water_mid;                //）水2档流量
  unsigned char t_water_high;               //）水3档流量
  unsigned char dit_time_min;               //）消毒时间分钟
  unsigned char clean_time_min;             //）冲洗时间分钟
  unsigned char tec_switch;                 //）制冷片开关
  unsigned char low_freq;                   //）重频模式（低重频(<20Hz)，高重频freq>1K）
  unsigned short int charge_width_us;       //）充电脉冲
  unsigned char laser_led_light;            //）激光指示灯亮度
  unsigned char rgb_light;                  //）rgb状态指示灯亮度
  unsigned char beep;                       //）音量
  ENERGE_CALIBRATION  e_cali[40];
  unsigned int  checkSum;// 校验和 (通信包不会发送)
}__attribute__ ((packed)) SYS_CONFIG_PARAM ;//系统配置参数
typedef union 
{
	SYS_CONFIG_PARAM sys_config_param;
	unsigned char data[sizeof(SYS_CONFIG_PARAM)];
}U_SYS_CONFIG_PARAM;

extern U_SYS_CONFIG_PARAM u_sys_param;
extern U_SYS_CONFIG_PARAM u_sys_default_param;
typedef enum{
	LASER_YAG_1064_TYPE=0,
	LASER_980_TYPE	
}laser_type;
typedef struct
{   
  unsigned short int  laserEnerge;          //能量值
  unsigned char   laserFreq;         			  //激光频率 1~100（100表示980激光，）
  unsigned char   laserType;         	      //0:1064（Nd：YAG）;1:980（连续激光）；2：（半导体）
	unsigned char   treatmentWaterLevel;      //治疗水流等级0~3（0,表示关闭）
	unsigned char   airPressureLevel;         //气流等级0~3（0,表示关闭）
	unsigned char   ledLightLevel;         	  //激光指示灯led光亮等级0~100（0,表示关闭）//duty
	unsigned char   ctrTestMode;         		  //测试模式（光纤激活）0x01打开，0x00关闭，
	unsigned char   proHotCtr;         			  //预燃控制0x01启动，0x00关闭
  unsigned char   proCali;         			    //是否校准0x01启动，0x00关闭
  unsigned char   timerCtr;         			  //倒计时时间
  unsigned char   timerEnableFlag;         	//倒计时启动
  unsigned char   cleanCtr;         			  //清洗消毒
  unsigned char   air_water_prepare_ctr;    //水雾准备
  unsigned char   lowEnergeMode; //DAC<1.85V
  unsigned char   beep;
}LASER_CONTROL_PARAM;//ctr param
extern LASER_CONTROL_PARAM laser_ctr_param;
typedef struct{ 
	/*******************状态*********************/
	char circle_water_box_temprature;                			  //水箱温度 （-127 ~128）
	unsigned char treatment_water_level_status;             //治疗水气流速等级 bit0~3 水等级； 
	unsigned char air_level_status;                         //bit4~7 气等级
	unsigned char laser_run_B0_pro_hot_status;              //Bit0：预燃状态；
	unsigned char laser_run_B1_laser_out_status;            //Bit1：（1064）出光状态；
	unsigned char laser_run_B2_gx_test_status;              //Bit2：光纤激活状态；
	unsigned char laser_run_B3_laser_pilot_lamp_status;     //Bit3：指示灯状态
	unsigned char laser_run_B4_laser_980_out_status;        //Bit4：（980）出光状态
  unsigned char laser_run_B5_timer_status;                //Bit5：定时器状态
  unsigned char laser_run_B6_close_device_status;         //Bit6：关机（1关机指令）
	unsigned char laser_param_B01_energe_status;            //Bit0~Bit1:激光能量计状态：
	unsigned char laser_param_B23_air_pump_pressure_status;	//Bit2~Bit3:循环气泵压力状态：
	unsigned char laser_param_B456_jt_status;               //Bit4~Bit6:脚踏状态：
	unsigned short int genaration_io_status;                //bhit0~bit15		
	//不需定时上传
	unsigned int  laser1064PulseCount;
	unsigned int  laser980TimeS;
  unsigned int  RDB_TimeS;
//	char circle_water_box_temprature;               //水箱温度 （-127 ~128）
//	unsigned char water_air_level_status;           //治疗水气流速等级 bit0~3 水等级； bit4~7 气等级
//	unsigned char laser_run_status;                	//激光运行状态Bit0：预燃状态；Bit1：出光状态；Bit2：光纤激活状态；Bit3：指示灯状态;bit4,水雾准备状态
//	unsigned char laser_param_status;               //激光参数状态 Bit0~Bit1:激光能量计状态：Bit2~Bit3:循环气泵压力状态：Bit4~Bit6:脚踏状态：
//	unsigned short int genaration_io_status;        //bhit0~bit15
	//IO 
	/*
  unsigned char high_voltage_solenoid_status;    	//bit0高压电磁阀1正常，0异常
  unsigned char deflate_air_solenoid_status;     	//bit1堵气阀1正常，0异常
  unsigned char chocke_air_solenoid_status;      	//bit2泄气阀1正常，0异常
  unsigned char enviroment_tmprature_alert_status;//bit3环境温度报警1正常，0异常
  unsigned char h_air_error_status;               //bit4气泵电磁阀电源错误状态：1正常，0异常
  unsigned char Hyperbaria_OFF_Signal_staus;     	//bit5气泵过气压报警口状态：1正常，0过高，	
  unsigned char water_ready_ok_status;            //bit6治疗水 1正常出水,0异常，
  unsigned char water_circle_ok_status;           //bit7冷却液循环，0错误，1正常，	
  unsigned char NTC_temprature_status;           	//bit8蠕动泵状态，0过热，1正常	
  unsigned char iBus_status;                     	//bit9总电源电流状态, 0异常；1正常
  unsigned char vBus_status;                     	//bit10总电源电压状态，0异常；1正常
  unsigned char water_circle_temprature_status;  	//bit11冷却箱K1温度状态：1正常，0异常
  unsigned char laser_temprature_status;         	//bit12激光K2温度状态：1正常，0异常	
	unsigned char emergency_press_status;         	//bit13急停开关状态,0异常；1正常
  unsigned char water_air_prepare_status;         //bit14水雾准备,0异常；1正常
  unsigned char clean_status;                     //bit15清洗消毒状态,0异常；1正常
*/
 }sys_genaration_status;
 extern  sys_genaration_status sGenSta;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
