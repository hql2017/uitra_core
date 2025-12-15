#ifndef __BUZZ_BSP_H_
#define __BUZZ_BSP_H_

typedef enum{
    MUSIC_MUTE=0,
    MUSIC_SHORT_PROMT,
    MUSIC_LONG_PROMT,
    MUSIC_TWO_TIGER,
    MUSIC_STAR,
    MUSIC_HAPPY,  
    MUSIC_SYS_ON,
    MUSIC_LASER_WORK
}music_type;
extern unsigned short int music_tab_c[];
extern  unsigned char music_star[85];
extern   unsigned char music_happy[97];
extern unsigned char music_two_tiger[65];

extern  void app_beep_pwm(unsigned short int  freq,unsigned char volume);

#endif
