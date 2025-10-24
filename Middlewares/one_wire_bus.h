#ifndef __ONE_WIRE_BUS_H_
#define __ONE_WIRE_BUS_H_
/*软件实现单总线协议******************************************
波特率                 ：16.4kbit/s
uitra_core_202411版本硬件不支持总线协议
********************************************************/

#ifdef ONE_WIRE_BUS_JT_SLAVE 
#include "main.h"
extern unsigned short int  app_owb_get_receive_pack_len(void);
extern void  app_owb_receive_handle(unsigned char *pData,unsigned short int length);
extern void owb_dq_edge_callback(void);
extern void one_wire_bus_init(void);
#else
#include "main.h"
HAL_StatusTypeDef app_owb_write_status(void);
void app_owb_write_bytes(unsigned char *pData,unsigned short int length); 
#endif
extern void one_wire_bus_intit(void);
extern void owb_tim_callback(unsigned int timeUs);
#endif