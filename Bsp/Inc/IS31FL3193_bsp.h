/*
 *IS31FL3193_bsp.h
 *
 *  Created on: Apr 10, 2025
 *      Author: Hql2017
 */

 #ifndef IS31FL3193_BSP_H_
 #define IS31FL3193_BSP_H_
 
 #ifndef  FL3236_USED//12路
 #define  FL3236_USED
 
 #define Addr_GND_GND   0x78
 #include "stdint.h"
 #if 0
 #include "soft_i2c_bsp.h"
 #else
 uint8_t I2C_WriteByte(int DeviceAddress, int WriteAddress, int SendByte);
 uint8_t I2C_Write2Byte(int WriteAddress, int SendByte);
 uint8_t I2C_WriteBuffer(uint8_t* pBuffer,int length,int DeviceAddress ,int WriteAddress);
 #endif
 void IC_Write_Pro(void);
 void IC_WriteBuff_Pro(void);
 void IC_Read_Pro(void);
 void IC_ReadBuff_Pro(void);
 
 void SPI_Write_Pro(void);
 void SPI_WriteBuff_Pro(void);
 void SPI_Read_Pro(void);
 void SPI_ReadBuff_Pro(void);
 
 void Reset_Register(void);
 void Green_Breath(void );//单次呼吸
 void High_Breath(void );//单次呼吸
 void IS31FL3236A_Init(void);
 void is_12_all_gLED(void);
 void is_12_all_rLED(void);
 void is_12_all_bLED(void);
 void is_12_all_rgb(unsigned short int rgbValue);//混合色
 uint8_t   One_breath(void);
 #endif
 void app_is3_rgb_config(unsigned char reg,unsigned char *data);
 void IS3_init(void);
 void app_rgb_breath_ctl(unsigned char breathCount,unsigned short int rgbValue);
 void rgb_color_all(unsigned short int color);
 #endif /* IS31FL3193_BSP_H_ */
 
 