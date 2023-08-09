#ifndef __MENU_H__
#define __MENU_H__

#include "sys.h"
#include "lcd.h"
#include "key.h"

#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "lcd.h"
#include "Key.h"
// #include "malloc.h"
// #include "w25qxx.h"
#include "ST7789_Init.h" //屏幕
#include "BEEPER.h"
#include "Random.h"

#include "PWM.h"

#include "pid.h"

#include "Servo.h"
#include "control.h"

extern char str[20];

typedef enum
{
    _1_reset,
    _2_bigsquare,
    _3_task3,
    _4_task4,
    _5_pro_task3,
    _6_pro_task4,
    _7_pro_self,
    _71_control,
    _8_reset_point,

    MAX_MENU
} menu_index;

// _0_start,
//      _1_gyro,
//           _11_gyroShow,

//      _2_pid,
//           _21_angSpe_P,
//           _22_angSpe_I,
//           _23_angSpe_D,
//           _24_angele_P,
//           _25_angele_I,
//           _26_angele_D,
//           _27_speed_P,
//           _28_speed_I,
//           _29_speed_D,

//      _3_TRV,
//           _31_endoder

//       _4_mpid,
//          _41_speed_P,
//          _42_speed_I,
//          _43_speed_D,
//          _44_angele_P,
//          _45_angele_I,
//          _46_angele_D,
//          _47_angSpe_p

//      _5_camera,
//          _51_cameraShow

//      _6_dir,
//           _61_1,
//           _61_2,
//           _61_3,

typedef struct
{
    unsigned char current;
    unsigned char up;    // 向上翻索引号
    unsigned char down;  // 向下翻索引号
    unsigned char enter; // 确认索引号
    unsigned char quit;  // 退出索引号
    void (*showFun)();
} menu_keyTable;

void menu_show(void);
void menu_key(void);
void menu_show(void);

#endif // __MENU_H__
