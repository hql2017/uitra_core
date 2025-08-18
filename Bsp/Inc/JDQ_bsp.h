/*
 *JDQ_bsp.h
 *
 *  Created on: Apr 10, 2025
 *      Author: Hql2017
 */

#ifndef JDQ_BSP_H_
#define JDQ_BSP_H_
void jdq_init(void);
float AD5541A_SetVoltage(float outVoltage, float vRef);
void app_laser_pulse_start(unsigned short int timeUs,unsigned short int freq);
float app_jdq_voltage_monitor(void);
void app_jdq_current_limit_charge(void);
void app_jdq_direct_160v(void);
void app_jdq_consume_remaining_power_160v(void);

//电源模块
void app_jdq_bus_power_in(unsigned short int powerVolotage);
void app_jdq_bus_current_set(float  powerCurrent);
void app_jdq_bus_voltage_set(float  voltage);
void app_jdq_bus_get_v_c_req(void);
void app_jdq_bus_power_onoff_sta_req();
void app_jdq_bus_get_set_v_c(float *voltage,float *current);
unsigned short int  app_jdq_get_vbus_sta(void );
void app_jdq_bus_power_on_off(unsigned char flag);
void app_jdq_bus_vol_current_set(float powerVolotage,float  powerCurrent);
float app_jdq_get_laser_v(void);

void jdq_reley_charge(unsigned char onOff);
void jdq_reley_charge_ready(unsigned char onOff);

unsigned short int  app_get_jdq_rs485_bus_statu(void);
unsigned char  app_jdq_rs485_check_rec_len(void);
void app_jdq_rs485_receive_data(void);
void app_jdq_sts_1200_receive_handle(void);

#define JDQ_RS485_FRAME_MAX_DELAY_MS 1500
#define JDQ_RS485_FRAME_MIN_MS 100
 #define MAX_UART1_BUFF_LENTH 64
 
#define  STS_1200_REG_SET_VOLTAGE 	    1000//设定电压
#define  STS_1200_REG_SET_CURRENT	    1001//设定电流，
#define  STS_1200_REG_VOLTAGE_DISPLAY	1002//电压显示
#define  STS_1200_REG_CURRENT_DISPLAY	1003//电流显示
#define  STS_1200_REG_RUN_STOP	        1006//电源输出/停止 0停止；1输出。

#define LASER_PULSE_STOP  0
#define LASER_JDQ_VOLTAGE_F  150.0f
#define LASER_JDQ_VOLTAGE  150
#define  LASER_JDQ_CHARGE_TIMEOUT_MS  90000//60S

#ifndef  ADS1110_JDQ_USED
#define  ADS1110_JDQ_USED 
#define  ADS1110_I2C_SLAVE_ADDR  0x48<<1  //A0:1001000  A3:1001011

#define  ADS1110_DEFAULT_CONFIG  0x8C

#define  ADS1110_CONFIG_START_SAMP  0x80//启动采样
#define  ADS1110_CONFIG_SINGLE_SAMPLING  0x10 //单次
#define  ADS1110_CONFIG_CONTINUS_SAMPLING   0x00//连续
#define  ADS1110_CONFIG_240SPS  0x00 //采样率
#define  ADS1110_CONFIG_60SPS   0x04 //采样率
#define  ADS1110_CONFIG_30SPS   0x08 //采样率
#define  ADS1110_CONFIG_15SPS   0x0C //采样率默认
#define  ADS1110_CONFIG_1PGA   0x00 //增益1，默认
#define  ADS1110_CONFIG_2PGA   0x01 //增益2
#define  ADS1110_CONFIG_4PGA   0x02 //增益4
#define  ADS1110_CONFIG_8PGA   0x03 //增益8

#define  ADS1110_REF          2048.0f

#endif

#endif /* JDQ_BSP_H_ */

