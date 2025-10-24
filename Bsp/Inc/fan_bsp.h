/*
 *fan_bsp.h
 *
 *  Created on: Aug 14, 2025
 *      Author: Hql2017
 */

#ifndef __FAN_BSP_H__
#define __FAN_BSP_H__

extern void fan_init(void);
extern void app_fan_feed_count(unsigned char fan_number);
extern void fan_stop(unsigned char stopFlag);
extern void fan_start(unsigned char runflag);
extern void app_fan_manage(unsigned int systick);
extern void fan_spd_set(unsigned char fanNumber,unsigned int spd);
extern unsigned short int fan_get_run_spd(unsigned char fanNumber);
#define FAN38_COMPRESSOR_NUM  2//压缩机风扇
#define FAN25_NUM 1
#endif /* FAN_BSP_H_ */

