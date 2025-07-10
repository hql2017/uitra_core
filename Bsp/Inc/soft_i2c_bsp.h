#ifndef _SOFT_I2C_BSP_H
#define _SOFT_I2C_BSP_H
#include "main.h"
/*
	I2C_Speed(us)		Speed
	1					1.5MHz
	2					1MHz
	3					700KHz
	5					500KHz
	6					400KHz
	9					300KHz
	14				200KHz
	29				100KHz
	58				50KHz
*/
#define SOFT_I2C1MHz 	2
#define SOFT_I2C400KHz 	4

#define SOFT_I2C100KHz 	24

#define TRUE1	1
#define FALSE1 0


void soft_I2C_GPIO_Init(void);
uint8_t soft_I2C_WriteByte(int DeviceAddress, unsigned char SendByte);
uint8_t soft_I2C_ReadByte(int DeviceAddress);
uint8_t soft_I2C_ReadBuffer(uint8_t* pBuffer,   int length,  int DeviceAddress);
uint8_t soft_I2C_WriteBuffer(uint8_t* pBuffer,int length,int DeviceAddress );

#endif /* __SOFT_I2C_H */

