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
extern void app_fan_manage(unsigned int systick);
#endif /* FAN_BSP_H_ */

