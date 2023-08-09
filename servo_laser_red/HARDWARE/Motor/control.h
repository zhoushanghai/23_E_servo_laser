#ifndef __CONTROL_H__
#define __CONTROL_H__
#include "sys.h"

#include "Random.h"

#include "Encoder.h"
#include "Motor.h"
#include "PWM.h"

#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "pid.h"
#include "gw_gray_serial.h"

#include "Servo.h"
#include "delay.h"

#include "key.h"
void servo_line(int x1, int y1, int x2, int y2);
void go_line(int x1, int y1, int x2, int y2);
void go_line_set(int x1, int y1, int x2, int y2, int min);

#endif
