#ifndef __ONE_WIRE_BUS_H_
#define __ONE_WIRE_BUS_H_
/*软件实现单总线协议******************************************
波特率                 ：16.4kbit/s
uitra_core_202411版本硬件不支持总线协议
********************************************************/
#include "main.h"
#ifdef ONE_WIRE_BUS_SLAVE 
unsigned short int  app_owb_get_receive_pack_len(void);
void app_owb_receive_handle(unsigned char *pData,unsigned short int length);
void owb_dq_falling_callback(void);
#else
HAL_StatusTypeDef app_owb_write_status(void);
void app_owb_write_bytes(unsigned char *pData,unsigned short int length); 
#endif
extern void one_wire_bus_intit(void);
extern void owb_tim_callback(unsigned int timeUs);
#endif