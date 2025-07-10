/*
 * eeprom_bsp.h
 *
 *  Created on: Apr 10, 2025
 *      Author: Hql2017
 */

#ifndef EEPROM_BSP_H_
#define EEPROM_BSP_H_
 void EEPROM_M24C32_init(void);

unsigned char EEPROM_M24C32_Test(void);
unsigned char EEPROM_M24C32_Read(unsigned short int ReadAddr, unsigned char *pBuffer, unsigned short int ReadNum);
unsigned char EEPROM_M24C32_Write(unsigned short int WriteAddr,unsigned char *pBuffer,unsigned short int WriteNum);
unsigned char EEPROM_M24C32_Read_WORD(unsigned short int ReadAddr, unsigned int *pBuffer, unsigned short int ReadNum);
unsigned char EEPROM_M24C32_write_WORD(unsigned short int writeAddr, unsigned int *pBuffer, unsigned short int WriteNum);

#define EEROM_SYS_PARAM_SAVE_ADDR  0

#endif /* EEROM_BSP_H_ */

