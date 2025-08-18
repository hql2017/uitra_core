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
#define ADS1118_DRDY_in_Pin GPIO_PIN_4
#define ADS1118_DRDY_in_GPIO_Port GPIOE
#define ADS1118_SPI4_MISO_Pin GPIO_PIN_5
#define ADS1118_SPI4_MISO_GPIO_Port GPIOE
#define ADS1118_SPI4_MOSI_Pin GPIO_PIN_6
#define ADS1118_SPI4_MOSI_GPIO_Port GPIOE
#define PWR_SYS_ON_Pin GPIO_PIN_14
#define PWR_SYS_ON_GPIO_Port GPIOC
#define water_cycle_ok_Pin GPIO_PIN_15
#define water_cycle_ok_GPIO_Port GPIOC
#define AD_OCP_CHANNEL_Pin GPIO_PIN_0
#define AD_OCP_CHANNEL_GPIO_Port GPIOA
#define AD_VBUS_24V_CHANNEL_Pin GPIO_PIN_1
#define AD_VBUS_24V_CHANNEL_GPIO_Port GPIOA
#define NTC_14_CHANNEL_Pin GPIO_PIN_2
#define NTC_14_CHANNEL_GPIO_Port GPIOA
#define AD_LASER_1064_CHANNEL_Pin GPIO_PIN_3
#define AD_LASER_1064_CHANNEL_GPIO_Port GPIOA
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
#define RF24_IRQ_in_Pin GPIO_PIN_0
#define RF24_IRQ_in_GPIO_Port GPIOB
#define H_AIR_EN_Pin GPIO_PIN_1
#define H_AIR_EN_GPIO_Port GPIOB
#define H_AIR_ERROR_Pin GPIO_PIN_2
#define H_AIR_ERROR_GPIO_Port GPIOB
#define LASER_1064_count_in_Pin GPIO_PIN_7
#define LASER_1064_count_in_GPIO_Port GPIOE
#define LASER_1064_count_in_EXTI_IRQn EXTI9_5_IRQn
#define LASER_1064_AD_DOWN_out_Pin GPIO_PIN_8
#define LASER_1064_AD_DOWN_out_GPIO_Port GPIOE
#define auxiliary_bulb_pwm_Pin GPIO_PIN_9
#define auxiliary_bulb_pwm_GPIO_Port GPIOE
#define Solenoid_STATUS1_Pin GPIO_PIN_10
#define Solenoid_STATUS1_GPIO_Port GPIOE
#define Solenoid_STATUS2_Pin GPIO_PIN_11
#define Solenoid_STATUS2_GPIO_Port GPIOE
#define Solenoid_STATUS3_Pin GPIO_PIN_12
#define Solenoid_STATUS3_GPIO_Port GPIOE
#define Solenoid_EN1_Pin GPIO_PIN_13
#define Solenoid_EN1_GPIO_Port GPIOE
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
#define TMC2226_USART3_TX_Pin GPIO_PIN_8
#define TMC2226_USART3_TX_GPIO_Port GPIOD
#define TMC2222_USART3_RX_Pin GPIO_PIN_9
#define TMC2222_USART3_RX_GPIO_Port GPIOD
#define TMC2226_STEP_out_Pin GPIO_PIN_10
#define TMC2226_STEP_out_GPIO_Port GPIOD
#define TMC2226_DIR_out_Pin GPIO_PIN_11
#define TMC2226_DIR_out_GPIO_Port GPIOD
#define TMC2226_EN_out_Pin GPIO_PIN_12
#define TMC2226_EN_out_GPIO_Port GPIOD
#define TMC2226_index_in_Pin GPIO_PIN_13
#define TMC2226_index_in_GPIO_Port GPIOD
#define TMC2226_index_in_EXTI_IRQn EXTI15_10_IRQn
#define TMC2226_ERROR_out_Pin GPIO_PIN_14
#define TMC2226_ERROR_out_GPIO_Port GPIOD
#define FOOT_SWITCH_IN_Pin GPIO_PIN_15
#define FOOT_SWITCH_IN_GPIO_Port GPIOD
#define KEY_PWR_SWITCH_Pin GPIO_PIN_6
#define KEY_PWR_SWITCH_GPIO_Port GPIOC
#define HV_ONE_PULSE_Pin GPIO_PIN_7
#define HV_ONE_PULSE_GPIO_Port GPIOC
#define EEROM_W_EN_out_Pin GPIO_PIN_8
#define EEROM_W_EN_out_GPIO_Port GPIOC
#define EEROM_I2C3_SDA_Pin GPIO_PIN_9
#define EEROM_I2C3_SDA_GPIO_Port GPIOC
#define EEROM_I2C3_SCL_Pin GPIO_PIN_8
#define EEROM_I2C3_SCL_GPIO_Port GPIOA
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
#define VC160_I2C_SDA_out_Pin GPIO_PIN_1
#define VC160_I2C_SDA_out_GPIO_Port GPIOD
#define VC160_I2C_SCK_Pin GPIO_PIN_2
#define VC160_I2C_SCK_GPIO_Port GPIOD
#define circulating_pump_EN_Pin GPIO_PIN_3
#define circulating_pump_EN_GPIO_Port GPIOD
#define PTC_EN_Pin GPIO_PIN_4
#define PTC_EN_GPIO_Port GPIOD
#define JDQ_STAND_Pin GPIO_PIN_5
#define JDQ_STAND_GPIO_Port GPIOD
#define JDQ_READY_Pin GPIO_PIN_6
#define JDQ_READY_GPIO_Port GPIOD
#define DAC_SPI1_MOSI_Pin GPIO_PIN_7
#define DAC_SPI1_MOSI_GPIO_Port GPIOD
#define DAC_SPI1_SCK_Pin GPIO_PIN_3
#define DAC_SPI1_SCK_GPIO_Port GPIOB
#define DAC_LD_Pin GPIO_PIN_4
#define DAC_LD_GPIO_Port GPIOB
#define DAC_CS_Pin GPIO_PIN_5
#define DAC_CS_GPIO_Port GPIOB
#define TEMPRATURE_ALERT_I2C1_SCL_Pin GPIO_PIN_6
#define TEMPRATURE_ALERT_I2C1_SCL_GPIO_Port GPIOB
#define TEMPRATURE_ALERT_I2C1_SDA_Pin GPIO_PIN_7
#define TEMPRATURE_ALERT_I2C1_SDA_GPIO_Port GPIOB
#define TEMPRATURE_ALERT_Pin GPIO_PIN_8
#define TEMPRATURE_ALERT_GPIO_Port GPIOB
#define RS485_DIR_Pin GPIO_PIN_9
#define RS485_DIR_GPIO_Port GPIOB
#define MCU_SYS_health_LED_Pin GPIO_PIN_0
#define MCU_SYS_health_LED_GPIO_Port GPIOE
#define water_ready_ok_in_Pin GPIO_PIN_1
#define water_ready_ok_in_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */
//激光工作环境温度，冷却液温度（允许温差3度）
#define MIN_TEMPRATURE_LASER   22.0f
#define MID_TEMPRATURE_LASER   23.0f//最佳工作温度
#define MAX_TEMPRATURE_LASER   24.0f

#define ERR_LOW_TEMPRATURE_LASER  21.0f //工作温度低边界
#define ERR_HIGH_TEMPRATURE_LASER   28.0f//工作温度高边界

//air pump 气泵 kPa,相对值
#define MIN_AIR_PUMP_PRESSURE  160.0f//210.00f
#define MID_AIR_PUMP_PRESSURE  180.0f//260.00f//最佳气压
#define MAX_AIR_PUMP_PRESSURE  200.0f//290.00f
//300.00f
#define ERR_T_FLOAT_VALUE      1000///温度浮点数错误值
#define ERR_T_SHORT_INT_VALUE  160///温度整型错误值

//laser
#define LASER_980_MIN_ENERGE_V     150//mV(210mV，1W) 
#define LASER_980_MAX_ENERGE_V     1200//mV(12W,200mJ)

#define LASER_1064_MIN_ENERGE_V    7000// 440V
#define LASER_1064_MAX_ENERGE_V    13500/// 800V

#define SYS_1_MINUTES_TICK    60000//
#define LASER_MAX_CONTINUS_WORK_TIME    5*SYS_1_MINUTES_TICK

#define MAX_IBUS_MA 10000
#define MIN_VBUS_MV 12000

#define EEROM_DATA_ERR_CHECK_FLAG  20250707//有历史数据

#ifdef IWDG_USED
#define IWDG_USED  
#endif

#ifdef DEBUG_MSG_UART 
#define DEBUG_MSG_UART  /*use printf*/
#define DEBUG_PRINTF(fmt, ...) printf(fmt, ##__VA_ARGS__)
#else
#define DEBUG_PRINTF(fmt, ...) do { } while(0)
#endif
typedef enum{ 
  IO_KEY_IDLE=0,
  KEY_SHORT_PRESS,//短按
  KEY_LONG_PRESS,//长按2
  KEY_LONG_RELEASE//长按释放3
}KEY_VALUE;
typedef enum{ 
  NO_KEY_MESSAGE=0,
  key_jt_short_press,
  key_jt_long_press,
  key_jt_release,
  key_pwr_short_press,
  key_pwr_long_press,
  key_pwr_release,
//key_multifunctional_press,//全部按下
}app_key_message;
typedef struct { 	
  unsigned char systemParamFlag;                        //系统参数，0未加载；1默认参数；2正常参数；3通讯异常；
  unsigned char jtFlag;                                 //脚踏输入，0未加载；1待机；2开启；3异常；
  unsigned char rgbFlag;                                //rgb氛围灯，0未加载；1待机；2开启；3异常；
  unsigned char auxiliary_bulbFlag;                     //激光指示灯，0未加载；1待机；2开启；3异常；
  unsigned char hmiLcdLoadFlag;                         //显示屏，0未加载；1待机；2开启；3异常；
  unsigned char coolWaterSystemLoadFlag;                //冷却水系统，0未加载；1待机；2开启；3异常；
  unsigned char treatmentWaterSystemLoadFlag;           //治疗水系统，0未加载；1待机；2开启；3异常；  
  unsigned char eTempratureAirpressureSystemLoadFlag;   //环境气压系统，0未加载；1待机；2开启；3异常；
  unsigned char eTempratureSystemLoadFlag;   						//环境温度，0未加载；1待机；2开启；3异常；
  unsigned char tempratureSystemLoadFlag;               //温度系统，0未加载；1待机；2开启；3异常；
  unsigned char adNtCSystemLoadFlag;             				//气压系统，0未加载；1待机；2开启；3异常；
  unsigned char adLaserEnergeSystemLoadFlag;            //激光能量监测系统，0未加载；1待机；2开启；3异常；
  unsigned char adIBusSystemLoadFlag;              			//电源电流监测系统，0未加载；1待机；2开启；3异常；
  unsigned char adVBusSystemLoadFlag;                 	//电源电压检测系统，0未加载；1待机；2开启；3异常；
  unsigned char adAirPressureSystemLoadFlag;            //气压系统，0未加载；1待机； 2开启；3异常；
  unsigned char laserPowerSystemLoadFlag;               //激光电源系统，0未加载；1待机；2开启；3异常；
	
}__attribute__ ((packed)) SYS_LOAD_STATUS;							//辅助系统加载状态
 typedef struct{  

		float NTC_temprature;                 //蠕动泵电机驱动IC温度，adc 
		float laser_1064_energy;              //1064激光能量计，adc   
		float iBus;                           //24V总电源电流mA，adc
		float vBus;                           //24V总电源电压mV，adc
		float air_pump_pressure;              //气泵气压 ,adc
		//float eth_cool_k0_temprature;       //环境冷端补偿温度
		float eth_k1_temprature;              //热电偶K1温度
		float eth_k2_temprature;              //热电偶K2温度
		float enviroment_temprature;          //环境温度m117z
		float air_gzp_temprature;             //环境气压计温度
		float air_gzp_enviroment_pressure_kpa;//环境气压计,kpa  //平原气压约90~100kPa，西藏约80kPa
		float treatment_water_depth;          //治疗水位
		//lite增加部分
		float tec_current_out;//制冷片
 }sys_enviroment_assistant_param;//辅助参数
 extern  sys_enviroment_assistant_param sEnvParam;
typedef struct { 
  unsigned int  errFlag;                //异常标志0x00Ok，0xFF未初始化
  unsigned int  equipmentModel  ;       // 设备型号HSM_III1064
  unsigned int  softVersion;            // 软件版本2025063001(2025.06.30.01)
  unsigned int  equipmentId  ;          // 设备编号 
	unsigned int  laser1064PulseCount;  	// 1064总脉冲数。
	unsigned int  laser1064TimeS;  				// 1064脉冲激光运行时间。(MAX,连续49.7天)
  unsigned int  laser980TimeS;  				// 980运行时间。(MAX,连续49.7天)
  unsigned int  checkSum;               // 校验和 
}SYS_CONFIG_PARAM ;//系统配置参数
typedef union 
{
	SYS_CONFIG_PARAM sys_config_param;
	unsigned char data[sizeof(SYS_CONFIG_PARAM)];
}U_SYS_CONFIG_PARAM;
extern U_SYS_CONFIG_PARAM u_sys_param;
extern U_SYS_CONFIG_PARAM u_sys_default_param;
typedef struct
{   
	unsigned char laserType;         			  //0:1064（YAG宝石）;1:980（半导体）
  unsigned short int  laserEnerge;        //（能量值）//输出电压值
  unsigned char laserFreq;         			  //激光频率 1~100（100表示980激光，）
	unsigned char treatmentWaterLevel;      //治疗水流等级0~3（0,表示关闭）
	unsigned char airPressureLevel;         //气流等级0~3（0,表示关闭）
	unsigned char ledLightLevel;         	  //led光亮等级0~100（0,表示关闭）//duty
	unsigned char ctrTestMode;         		  //测试模式（光纤激活）0x01打开，0x00关闭，
	unsigned char  proHotCtr;         			//预燃控制0x01启动，0x00关闭
}LASER_CONFIG_PARAM;//激光参数，
 extern LASER_CONFIG_PARAM laser_config_param;
typedef enum{
	LASER_YAG_1064_TYPE=0,
	LASER_980_TYPE	
}laser_type;

typedef struct{ 
	/*******************屏幕需要*********************/
	char circle_water_box_temprature;                			//水箱温度 （-127 ~128）
	unsigned char treatment_water_level_status;           //治疗水气流速等级 bit0~3 水等级； 
	unsigned char air_level_status;                       //bit4~7 气等级
	unsigned char laser_run_B0_pro_hot_status;            //激光运行状态Bit0：预燃状态；
	unsigned char laser_run_B1_laser_out_status;          //Bit1：出光状态；
	unsigned char laser_run_B2_gx_test_status;            //Bit2：光纤激活状态；
	unsigned char laser_run_B3_laser_pilot_lamp_status;   //Bit3：指示灯状态
	unsigned char laser_run_B4_laser_980_out_status;      //Bit4：半导体出光状态
	unsigned char laser_param_B01_energe_status;            // Bit0~Bit1:激光能量计状态：
	unsigned char laser_param_B23_air_pump_pressure_status;	//	Bit2~Bit3:循环气泵压力状态：
	unsigned char laser_param_B456_jt_status;               //Bit4~Bit6:脚踏状态：
	unsigned short int genaration_io_status;                //bhit0~bit13		
	//不需定时上传
	unsigned int  laser1064PulseCount;
	unsigned int  laser980TimeMs;
	//本地需要
	float laser_box_temprature;                			//激光器内温度温度 （-127 ~128）	
	unsigned char laser_type;                			//0 :1064 1:980 2：光纤激活（980持续1秒）	
//	char circle_water_box_temprature;                //水箱温度 （-127 ~128）
//	unsigned char water_air_level_status;           //治疗水气流速等级 bit0~3 水等级； bit4~7 气等级
//	unsigned char laser_run_status;                	//激光运行状态Bit0：预燃状态；Bit1：出光状态；Bit2：光纤激活状态；Bit3：指示灯状态
//	unsigned char laser_param_status;                	//激光参数状态 Bit0~Bit1:激光能量计状态：Bit2~Bit3:循环气泵压力状态：Bit4~Bit6:脚踏状态：
//	unsigned short int genaration_io_status;        //bhit0~bit13
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
//unsigned char laser_1064_energy_status;       	//bit14激光能量计状态，0失效；1正常；2超标
//unsigned char air_pump_pressure_status;      		//bit15循环气泵压力状态，0过低，1正常、2过高
//unsigned char jt_key_status;                		//bit16脚踏,0失效异常；1正常;2按下;3松开。4无线脚踏电量低
*/
 }sys_genaration_status;
 extern  sys_genaration_status sGenSta;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
