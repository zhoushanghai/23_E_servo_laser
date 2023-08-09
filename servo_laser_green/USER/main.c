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
#include "menu.h"
#include "BEEPER.h"

//-----(‵▽′)/----------------------  -----------------------------//
/*
舵机  11=1°   35
servo1

上 500
中 1530  90°
下 1800   **不能超过这个值，不然会卡住
90° 1550

servo2
右边 500
中 1500  90°
左边 2500

*/

char str[20];

GLO_FLAG flag_G = {.oled = 0};
SERVO servo;

KEY key[KEY_NUMBER] = { // 按键
	{KEY_RELEASE, KEY_RELEASE}};

K210 k210 = {0};

extern CAR_PID pid;

DOT dot[4] = {0};
DOT testDot[4] = {0};

//-----(‵▽′)/---------------------- fun -----------------------------//
void data_show(void);
void key_(void);
void software_init(void);
void wave(void);

void RX_data_deal(void);
float yaw_in180(float yaw);
float abs_float(float a);
void reminder(void);
void no_reminder(void);
int abs_int(int x);
//(‵▽′)/////////////////////////////////////////////////////////////////////////////////
float angle = 90;

int seed = 0; // 由于生成随机数

//(‵▽′)/////////////////////////////////////  ////////////////////////////////////////////

int main(void)
{
	software_init();

	while (1)
	{

		if (CHECK_FLAG(&flag_G.oled))
		{
			if (flag_G.wave)
				wave();
			RX_data_deal();
			// data_show();
		}
		if (CHECK_FLAG(&flag_G.key))
		{
			key_();

			if (CHECK_FLAG(&flag_G.cm3))
			{
				reminder();
			}
			else
			{
				no_reminder();
			}
		}

		if (CHECK_FLAG(&flag_G.uart))
		{
			RX_k210data_deal();
		}

		seed++;
	}
}

void wave(void)
{
	//(‵▽′)///////////////////////////////////// k210 data ////////////////////////////////////////////
	// printf("x%.2f y%.2f out%d\r\n", k210.x, k210.y, (uint16_t)pid.sevor1.output);
	// printf("%.2f,%.2f\r\n", k210.x, k210.y);
	printf("x:%.2f y:%.2f\r\n", pid.sevor1.output, pid.sevor2.output);
}

#define FILTER 0.4f
void TIM7_IRQHandler(void)
{
	static int time = 0;
	static int servo1_last = 0, servo2_last = 0;

	if (TIM_GetITStatus(TIM7, TIM_IT_Update) == SET)
	{
		++time;
		TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
		//(‵▽′)///////////////////////////////////// 舵机PID ////////////////////////////////////////////
		if (CHECK_FLAG(&flag_G.k210))
		{
			RX_k210data_deal();

			if (flag_G.detect)
			{
				// pid cal
				PID_Incremental(&pid.sevor1, 0, k210.y);
				PID_Incremental(&pid.sevor2, 0, k210.x);
				PWM_SetServo1((int16_t)pid.sevor1.output);
				PWM_SetServo2((int16_t)pid.sevor2.output);

				printf("t%d s1:%d s2%d\r\n", time, (int16_t)pid.sevor1.output, (int16_t)pid.sevor2.output);
				time = 0;
			}

			if (abs_float(k210.x) + abs_float(k210.y) < 9)
			{
				flag_G.cm3 = 1;
			}
			else
			{
				flag_G.cm3 = 0;
			}
			// get fps
			// k210_cnt++;
			// if (time >= 1000)
			// {
			// 	// printf("\r\n\r\nk210_cnt:%d\r\n", k210_cnt);
			// 	k210_cnt = 0;
			// 	time = 0;
			// }
		}

		//(‵▽′)///////////////////////////////////// 设定输出 ////////////////////////////////////////////
		if (CHECK_FLAG(&flag_G.servo))
		{

			if (servo1_last != servo.servo1_set)
				PWM_SetServo1(servo.servo1_set);
			if (servo2_last != servo.servo2_set)
				PWM_SetServo2(servo.servo2_set);

			servo1_last = servo.servo1_set;
			servo2_last = servo.servo2_set;
		}
	}
}

void data_show(void)
{
	sprintf(str, "1:%d 2:%d", servo.servo1_set, servo.servo2_set);
	LCD_ShowString(0, 10, str, WHITE, BLACK, 16, 0);

	// 显示四个dot坐标
	sprintf(str, "x:%d y:%d  ", dot[0].x, dot[0].y);
	LCD_ShowString(0, 30, str, WHITE, BLACK, 16, 0);
	sprintf(str, "x:%d y:%d  ", dot[1].x, dot[1].y);
	LCD_ShowString(0, 50, str, WHITE, BLACK, 16, 0);
	sprintf(str, "x:%d y:%d  ", dot[2].x, dot[2].y);
	LCD_ShowString(0, 70, str, WHITE, BLACK, 16, 0);
	sprintf(str, "x:%d y:%d  ", dot[3].x, dot[3].y);
	LCD_ShowString(0, 90, str, WHITE, BLACK, 16, 0);

	// 显示当前舵机位置
	// sprintf(str, "                   ");
	// LCD_ShowString(0, 110, str, WHITE, BLACK, 16, 0);

	sprintf(str, "servo pos x:%5d y:%5d", servo.current_x, servo.current_y);
	LCD_ShowString(0, 110, str, WHITE, BLACK, 16, 0);

	sprintf(str, "k210 x:%.2f y:%.2f", k210.x, k210.y);
	LCD_ShowString(0, 130, str, WHITE, BLACK, 16, 0);
}

void software_init(void)
{
	flag_G.wave = WAVE;
	flag_G.detect = 0;
	flag_G.cm3 = 0;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); // 设置系统中断优先级分组2
	delay_init(168);								// 初始化延时函数

	ST7789_Init();						   // 屏幕初始化
	LCD_Color_Fill(0, 0, 240, 240, GREEN); // 让屏幕刷青色
	LCD_ShowString(1, 1, "hello world", BLACK, GREEN, 32, 0);

	uart_init(115200); // 初始化串口波特率为115200
	LED_Init();		   // 初始化LED

	PID_set(); // PID参数初始化

	PWM_TIM1_Init(); // 20ms 50Hz 舵机
	// PWM_SetServo1(30); // 舵机回正
	// PWM_SetServo2(-30);
	// delay_ms(800);
	// PWM_SetServo2(30);
	// delay_ms(800);
	PWM_SetServo1(0); // 舵机回正
	PWM_SetServo2(0);
	pid.sevor1.output = 0;
	pid.sevor2.output = 0;
	// delay_ms(500);

	PWM_TIM5_Init(8400 - 1, 1 - 1);

	key_init(10); // 按键初始化

	Beeper_Init(); // 初始化蜂鸣器

	//(‵▽′)/////////////////////////////////////  ////////////////////////////////////////////

	TIM4_Random_Init(1000 - 1, 84 - 1); // ms定时器
	TIM7_Random_Init(1000 - 1, 84 - 1); // 1ms定时器
}
void key_(void)
{
	key_scanner();
	for (int i = 0; i < KEY_NUMBER; i++)
	{
		key[i].state = key_get_state(i);
		if (key[i].state == KEY_SHORT_PRESS)
		{

			printf("key%d short press\r\n", i);
		}
		else if (key[i].state == KEY_LONG_PRESS)
		{

			if (key[i].lastState != KEY_LONG_PRESS)
			{
				printf("key%d long press\r\n", i);
				// flag_G.wave = flag_G.wave ^ 1;
			}
		}
		else if (key[i].state == KEY_RELEASE)
		{

			if (key[i].lastState != KEY_RELEASE)
			{
				printf("key%d KEY_RELEASE\r\n", i);
			}
		}
		key[i].lastState = key[i].state;
	}
	// menu_show();

	// 开始跟踪
	if (key[0].state == KEY_SHORT_PRESS)
	{
		// flag_G.detect = flag_G.detect ^ 1;
		flag_G.detect = 1;
		LCD_Color_Fill(0, 0, 240, 240, GREEN); // 让屏幕刷青色
		LCD_ShowString(1, 1, "tracking    ", BLACK, GREEN, 32, 0);
	}
	// 停止
	if (key[1].state == KEY_SHORT_PRESS)
	{
		// flag_G.detect = flag_G.detect ^ 1;
		flag_G.detect = 0;
		LCD_Color_Fill(0, 0, 240, 240, GREEN); // 让屏幕刷青色
		LCD_ShowString(1, 1, "stop    ", BLACK, GREEN, 32, 0);
	}

	// test
	if (key[2].state == KEY_SHORT_PRESS)
	{
		reminder();
	}
}

float yaw_in180(float yaw)
{
	if (yaw > 180)
	{
		yaw -= 360;
	}
	else if (yaw < -180)
	{
		yaw += 360;
	}
	return yaw;
}
float abs_float(float a)
{
	if (a < 0)
		return -a;
	else
		return a;
}

void reminder(void)
{
	// led
	// GPIO_ResetBits(GPIOC, GPIO_Pin_10);
	// GPIO_ResetBits(GPIOC, GPIO_Pin_12);
	GPIO_SetBits(GPIOC, GPIO_Pin_10);
	GPIO_SetBits(GPIOC, GPIO_Pin_12);
	// beeper
	Beeper_Red_Green_Traced();
}

void no_reminder(void)
{
	// led
	// GPIO_ResetBits(GPIOC, GPIO_Pin_10);
	// GPIO_ResetBits(GPIOC, GPIO_Pin_12);
	GPIO_ResetBits(GPIOC, GPIO_Pin_10);
	GPIO_ResetBits(GPIOC, GPIO_Pin_12);
}
int abs_int(int x)
{
	return x > 0 ? x : -x;
}
