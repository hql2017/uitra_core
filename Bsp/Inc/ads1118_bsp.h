/**
 * @file ads1118.h
 *
 * @brief This header file contains all register map definitions for the ADS1118 device family.
 * @warning This software ads1118 Drivers
 *
 * @copyright - http://www.ti.com/
 *
 */
#ifndef ADS1118_BSP_H
#define ADS1118_BSP_H

//****************************************************************************
//
// Standard Libraries
//
//****************************************************************************
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
//****************************************************************************
//
// Custom Libraries
//
//****************************************************************************
#include "main.h"

//****************************************************************************
//
// Global variables
//
//****************************************************************************
extern const char *adcRegisterNames[];

//****************************************************************************
//
// Function prototypes
//
//****************************************************************************
#if 0
void adcStartup(void);
int16_t readData(void);
#endif
uint16_t readSingleRegister(uint8_t address);
uint16_t writeSingleRegister(uint8_t address, uint16_t data);
uint16_t startAdcConversion(void);
uint16_t stopAdcConversion(void);

float ads1118_measure_internal_temperature_example(void);

// Getter functions
uint16_t    getRegisterValue(uint8_t address);

// Helper functions
uint8_t     upperByte(uint16_t uint16_Word);
uint8_t     lowerByte(uint16_t uint16_Word);
uint16_t    combineBytes(uint8_t upperByte, uint8_t lowerByte);
int32_t     signExtend(const uint8_t dataBytes[]);

float  app_ads1118_channel_get_value(unsigned char adChannel);
void app_ads1118_channel_sampling_start(unsigned char adChannel);
void app_ads1118_startup(void);
#define ADS1118_COOL_CHANNEL 0//TC MODE
#define ADS1118_K1_CHANNEL  1//AIN0 -AIN1 GND
#define ADS1118_K2_CHANNEL  2//AIN2 -AIN3 GND
#define ADS1118_DELEY_TIME_MS  5////2//SRC860 = 2;SRC250 = 4; SRC128 = 10
//****************************************************************************
//
// Register macros
//
//****************************************************************************

#define WLENGTH     1

//**********************************************************************************
//
// Device commands
//
//**********************************************************************************

//****************************************************************************
//
// Constants
//
//****************************************************************************
/* The ADS1118 does not have addressable registers, but a numbered register concept
 * is used to maintain synchronization between the device and the firmware.
 * Register 0 can be considered the Conversion register and Register 1 can be
 * considered the Configuration register.
 *
 */
#define NUM_REGISTERS                           ((uint8_t) 2)
/* Maximum register address or address of the last register in the regmap */
#define MAX_REGISTER_ADDRESS                    ((uint8_t) 1)

//****************************************************************************
//
// Register definitions
//
//****************************************************************************

/* NOTE: Whenever possible, macro names (defined below) were derived from
 * datasheet defined names; however, updates to documentation or readability
 * may cause mismatches between names defined here in example code from those
 * shown in the device datasheet.
 */


/* Register 0x00 (CONVERSION) definition ת���Ĵ���
 * ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |  Bit 15  |  Bit 14  |  Bit 13  |  Bit 12  |  Bit 11  |  Bit 10  |   Bit 9  |   Bit 8  |   Bit 7  |   Bit 6  |   Bit 5  |   Bit 4  |   Bit 3  |   Bit 2  |   Bit 1  |   Bit 0  |
 * ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                                    CONV[15:0]                                                                                   |
 * ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 */

    /* CONVERSION register addressת���Ĵ�����ַ */
    #define CONVERSION_ADDRESS                                              ((uint16_t) 0x00)

    /* CONVERSION default (reset) valueת��Ĭ�ϣ����ã�ֵ */
    #define CONVERSION_DEFAULT                                              ((uint16_t) 0x0000)

    /* CONVERSION register field masksת���Ĵ��������� */
    #define CONVERSION_CONV_MASK                                            ((uint16_t) 0xFFFF)


/* Register 0x01 (CONFIG) definition ���üĴ���
 * ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |  Bit 15  |  Bit 14  |  Bit 13  |  Bit 12  |  Bit 11  |  Bit 10  |   Bit 9  |   Bit 8  |   Bit 7  |   Bit 6  |   Bit 5  |   Bit 4  |   Bit 3  |   Bit 2  |   Bit 1  |   Bit 0  |
 * ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |    SS    |            MUX[2:0]            |            PGA[2:0]            |   MODE   |             DR[2:0]            |  TS_MODE |PULL_UP_EN|       NOP[1:0]      | RESERVED |
 * ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 */

    /* CONFIG register address */
    #define CONFIG_ADDRESS                                                  ((uint16_t) 0x01)

    /* CONFIG default (reset) value */
    #define CONFIG_DEFAULT                                                  ((uint16_t) 0x8583)//((uint16_t) 0x02B4)

    /* CONFIG register field masks */
    #define CONFIG_SS_MASK                                                  ((uint16_t) 0x8000)
    #define CONFIG_MUX_MASK                                                 ((uint16_t) 0x7000)
    #define CONFIG_PGA_MASK                                                 ((uint16_t) 0x0E00)
    #define CONFIG_MODE_MASK                                                ((uint16_t) 0x0100)
    #define CONFIG_DR_MASK                                                  ((uint16_t) 0x00E0)
    #define CONFIG_TS_MODE_MASK                                             ((uint16_t) 0x0010)
    #define CONFIG_PULL_UP_EN_MASK                                          ((uint16_t) 0x0008)
    #define CONFIG_NOP_MASK                                                 ((uint16_t) 0x0006)
    #define CONFIG_RESERVED_MASK                                            ((uint16_t) 0x0001)

    /* SS field values */
    #define CONFIG_SS_NA                                                    ((uint16_t) 0x0000)
    #define CONFIG_SS_CONV_START                                            ((uint16_t) 0x8000)

    /* MUX field values */
    #define CONFIG_MUX_AIN0_AIN1                                            ((uint16_t) 0x0000)
    #define CONFIG_MUX_AIN0_AIN3                                            ((uint16_t) 0x1000)
    #define CONFIG_MUX_AIN1_AIN3                                            ((uint16_t) 0x2000)
    #define CONFIG_MUX_AIN2_AIN3                                            ((uint16_t) 0x3000)
    #define CONFIG_MUX_AIN0_GND                                             ((uint16_t) 0x4000)
    #define CONFIG_MUX_AIN1_GND                                             ((uint16_t) 0x5000)
    #define CONFIG_MUX_AIN2_GND                                             ((uint16_t) 0x6000)
    #define CONFIG_MUX_AIN3_GND                                             ((uint16_t) 0x7000)

    /* PGA field values */
    #define CONFIG_PGA_6p144V                                               ((uint16_t) 0x0000)
    #define CONFIG_PGA_4p096V                                               ((uint16_t) 0x0200)
    #define CONFIG_PGA_2p048V                                               ((uint16_t) 0x0400)
    #define CONFIG_PGA_1p024V                                               ((uint16_t) 0x0600)
    #define CONFIG_PGA_0p512V                                               ((uint16_t) 0x0800)
    #define CONFIG_PGA_0p256V                                               ((uint16_t) 0x0A00)

    /* MODE field values */
    #define CONFIG_MODE_CONT                                                ((uint16_t) 0x0000)
    #define CONFIG_MODE_SS                                                  ((uint16_t) 0x0100)

#ifdef ADS1018
    /* DR field values */
    #define CONFIG_DR_128SPS                                                ((uint16_t) 0x0000)
    #define CONFIG_DR_250SPS                                                ((uint16_t) 0x0020)
    #define CONFIG_DR_490SPS                                                ((uint16_t) 0x0040)
    #define CONFIG_DR_920SPS                                                ((uint16_t) 0x0060)
    #define CONFIG_DR_1600SPS                                               ((uint16_t) 0x0080)
    #define CONFIG_DR_2400SPS                                               ((uint16_t) 0x00A0)
    #define CONFIG_DR_3300_1SPS                                             ((uint16_t) 0x00C0)
    #define CONFIG_DR_3300_2SPS                                             ((uint16_t) 0x00E0)
#else
    /* DR field values */
    #define CONFIG_DR_8SPS                                                  ((uint16_t) 0x0000)
    #define CONFIG_DR_16SPS                                                 ((uint16_t) 0x0020)
    #define CONFIG_DR_32SPS                                                 ((uint16_t) 0x0040)
    #define CONFIG_DR_64SPS                                                 ((uint16_t) 0x0060)
    #define CONFIG_DR_128SPS                                                ((uint16_t) 0x0080)
    #define CONFIG_DR_250SPS                                                ((uint16_t) 0x00A0)
    #define CONFIG_DR_475SPS                                                ((uint16_t) 0x00C0)
    #define CONFIG_DR_860SPS                                                ((uint16_t) 0x00E0)
#endif

    /* TS_MODE field values */
    #define TS_MODE_ADC                                                     ((uint16_t) 0x0000)
    #define TS_MODE_TS                                                      ((uint16_t) 0x0010)

    /* PULL_UP_EN field values */
    #define PULL_UP_EN_DISABLE                                              ((uint16_t) 0x0000)
    #define PULL_UP_EN_ENABLE                                               ((uint16_t) 0x0080)

    /* NOP field values */
    #define NOP_INV_0                                                       ((uint16_t) 0x0000)
    #define NOP_VALID                                                       ((uint16_t) 0x0002)
    #define NOP_INV_2                                                       ((uint16_t) 0x0004)
    #define NOP_INV_3                                                       ((uint16_t) 0x0006)

    /* RESERVED field values */
#define RESERVED_VALUE                                                      ((uint16_t) 0x0001)

#endif
