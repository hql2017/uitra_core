/**
 * @file MAX31865ATP_BSP.h
 *
 * @brief This header file contains all register map definitions for the ADS1118 device family.
 * @warning This software ads1118 Drivers
 *
 * @copyright - http://www.ti.com/
 *
 */
#ifndef MAX31865ATP_BSP_H
#define MAX31865ATP_BSP_H

extern unsigned char  MAX31865_SB_Read(unsigned char addr);
extern void MAX31865_SB_Write(unsigned char addr,unsigned char wdata);
extern float Get_pt_tempture(void);
extern void max_31865_pt1000(void);

#endif
