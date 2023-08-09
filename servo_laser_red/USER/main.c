#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "lcd.h"
#include "Key.h"
// #include "malloc.h"
// #include "w25qxx.h"
#include "ST7789_Init.h" //��Ļ
#include "BEEPER.h"
#include "Random.h"

#include "PWM.h"

#include "pid.h"

#include "Servo.h"
#include "control.h"
#include "menu.h"

//-----(�F����)/----------------------  -----------------------------//
/*
���  11=1��   35
servo1

�� 500
�� 1530  90��
�� 1800   **���ܳ������ֵ����Ȼ�Ῠס
90�� 1550

servo2
�ұ� 500
�� 1500  90��
��� 2500

*/

char str[20];

GLO_FLAG flag_G = {.oled = 0};
SERVO servo;

KEY key[KEY_NUMBER] = { // ����
	{KEY_RELEASE, KEY_RELEASE}};

K210 k210 = {0};

extern CAR_PID pid;

DOT dot[4] = {0};
DOT testDot[4] = {0};

//-----(�F����)/---------------------- fun -----------------------------//
void data_show(void);
void key_(void);
void software_init(void);
void wave(void);

void RX_data_deal(void);
float yaw_in180(float yaw);
float abs_float(float a);
void key_goline_key(void);
//(�F����)/////////////////////////////////////////////////////////////////////////////////
float angle = 90;

int seed = 0; // �������������

//(�F����)/////////////////////////////////////  ////////////////////////////////////////////

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
	//(�F����)///////////////////////////////////// k210 data ////////////////////////////////////////////
	// printf("x%.2f y%.2f out%d\r\n", k210.x, k210.y, (uint16_t)pid.sevor1.output);
	// printf("%.2f,%.2f\r\n", k210.x, k210.y);
	printf("x:%.2f y:%.2f\r\n", pid.sevor1.output, pid.sevor2.output);
}

#define FILTER 0.4f
void TIM7_IRQHandler(void)
{
	static int time = 0, k210_cnt = 0;
	static int servo1_last = 0, servo2_last = 0;

	if (TIM_GetITStatus(TIM7, TIM_IT_Update) == SET)
	{
		++time;
		TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
		//(�F����)///////////////////////////////////// ���PID ////////////////////////////////////////////
		if (CHECK_FLAG(&flag_G.k210))
		{
			RX_k210data_deal();
			// pid cal
			PID_Incremental(&pid.sevor1, 0, k210.y);
			PID_Incremental(&pid.sevor2, 0, k210.x);
			PWM_SetServo1((int16_t)pid.sevor1.output);
			PWM_SetServo2((int16_t)pid.sevor2.output);

			// get fps
			k210_cnt++;
			if (time >= 1000)
			{
				// printf("\r\n\r\nk210_cnt:%d\r\n", k210_cnt);
				k210_cnt = 0;
				time = 0;
			}
		}

		//(�F����)///////////////////////////////////// �趨��� ////////////////////////////////////////////
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
	LCD_ShowString(90, 10, str, WHITE, GBLUE, 16, 0);

	// ��ʾ�ĸ�dot����
	sprintf(str, "x:%d y:%d  ", dot[0].x, dot[0].y);
	LCD_ShowString(90, 30, str, WHITE, GBLUE, 16, 0);	
	sprintf(str, "x:%d y:%d  ", dot[1].x, dot[1].y);
	LCD_ShowString(90, 50, str, WHITE, GBLUE, 16, 0);
	sprintf(str, "x:%d y:%d  ", dot[2].x, dot[2].y);
	LCD_ShowString(90, 70, str, WHITE, GBLUE, 16, 0);
	sprintf(str, "x:%d y:%d  ", dot[3].x, dot[3].y);
	LCD_ShowString(90, 90, str, WHITE, GBLUE, 16, 0);

	// ��ʾ��ǰ���λ��
	// sprintf(str, "                   ");
	// LCD_ShowString(0, 110, str, WHITE, GBLUE, 16, 0);

	sprintf(str, "pos x:%5d y:%5d", servo.current_x, servo.current_y);
	LCD_ShowString(90, 110, str, WHITE, GBLUE, 16, 0);

	sprintf(str, "k210 x:%.2f y:%.2f", k210.x, k210.y);
	LCD_ShowString(90, 130, str, WHITE, GBLUE, 16, 0);
}

void software_init(void)
{
	flag_G.wave = WAVE;
	flag_G.go_line = 0;
	flag_G.line_stop = 0;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); // ����ϵͳ�ж����ȼ�����2
	delay_init(168);								// ��ʼ����ʱ����

	ST7789_Init();						 // ��Ļ��ʼ��
	LCD_Color_Fill(0, 0, 240, 240, RED); // ����Ļˢ��ɫ
	LCD_ShowString(1, 1, "hello world", WHITE, GBLUE, 16, 0);

	uart_init(115200); // ��ʼ�����ڲ�����Ϊ115200
	LED_Init();		   // ��ʼ��LED

	PID_set(); // PID������ʼ��

	PWM_TIM1_Init();	// 20ms 50Hz ���
	PWM_SetServo1(150); // �������
	PWM_SetServo2(-150);
	delay_ms(800);
	PWM_SetServo2(150);
	delay_ms(800);
	PWM_SetServo1(0); // �������
	delay_ms(800);
	PWM_SetServo2(0);
	pid.sevor1.output = 0;
	pid.sevor2.output = 0;
	delay_ms(500);

	PWM_TIM5_Init(8400 - 1, 1 - 1);

	key_init(10); // ������ʼ��

	//	Beeper_Init(); // ��ʼ��������

	LCD_Color_Fill(0, 0, 240, 240, RED); // ����Ļˢ��ɫ

	//(�F����)/////////////////////////////////////  ////////////////////////////////////////////

	TIM4_Random_Init(1000 - 1, 84 - 1); // ms��ʱ��
	TIM7_Random_Init(1000 - 1, 84 - 1); // 1ms��ʱ��
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
	menu_show();

#if 0
	if (key[2].state == KEY_SHORT_PRESS)
	{
		// ��������1�� �ص�ԭ��
		// ��Ϊ��е���ص�ԭ���ռ�ձȲ�һ����
		// if (servo.current_x >= 0)
		// 	PWM_SetServo1(0);
		// else
		// 	PWM_SetServo1(10);

		// delay_ms(400);

		// if (servo.current_y >= 0)
		// 	PWM_SetServo2(0);
		// else
		// 	PWM_SetServo2(7);

		// servo.current_x = 0;
		// servo.current_y = 0;
		PWM_SetServo1(0);
		delay_ms(400);
		PWM_SetServo2(0);
		pid.sevor1.output = 0;
		pid.sevor2.output = 0;
	}

	if (key[2].state == KEY_LONG_PRESS)
	{
		// ��������2����һ������
		// �ߵ���һ����
		// PWM_SetServo1(TASK_2_UP);
		// delay_ms(700);
		// PWM_SetServo2(TASK_2_RIGHT);
		// delay_ms(700);
		// PWM_SetServo1(TASK_2_DOWN);
		// delay_ms(700);
		// PWM_SetServo2(TASK_2_LEFT);
		// delay_ms(700);
		// PWM_SetServo1(TASK_2_UP);
		// delay_ms(700);
		// PWM_SetServo2(0);

		PWM_SetServo1(145);
		PWM_SetServo2(145);
		delay_ms(800);
		PWM_SetServo1(-145);
		PWM_SetServo2(155);
		delay_ms(800);
		PWM_SetServo1(-145);
		PWM_SetServo2(-148);
		delay_ms(800);
		PWM_SetServo1(145);
		PWM_SetServo2(-155);
		delay_ms(800);
		PWM_SetServo1(145);
		PWM_SetServo2(145);
		delay_ms(800);
	}
	if (key[3].state == KEY_SHORT_PRESS)
	{
		// ��������3���̶�A4ֽ�߿�
	}
	if (key[3].state == KEY_SHORT_PRESS)
	{
		// for (int i = 0; i < 10; i++)
		// {
		// 	USART_SendData(USART1, '1');
		// 	delay_ms(1);
		// }
		printf("task3\r\n");
		USART_SendData(USART1, '1');
		int cnt = 0;
		while (!CHECK_FLAG(&flag_G.k210))
		{
			cnt++;
			if (cnt > 100000000)
			{
				printf("k210 not ready\r\n");
				break;
			}
		}
		RX_k210data_deal();
		data_show();
		// ʹ��go_line_se����
		// int setGO = 1;
		// go_line_set(dot[0].x, dot[0].y, dot[1].x, dot[1].y, setGO);
		// go_line_set(dot[1].x, dot[1].y, dot[2].x, dot[2].y, setGO);
		// go_line_set(dot[2].x, dot[2].y, dot[3].x, dot[3].y, setGO);
		// go_line_set(dot[3].x, dot[3].y, dot[0].x, dot[0].y, setGO);

		// ��������4�����A4ֽ�߿�

		PWM_SetServo1(dot[0].y);
		PWM_SetServo2(dot[0].x);
		delay_ms(1000);
		PWM_SetServo1(dot[1].y);
		PWM_SetServo2(dot[1].x);
		delay_ms(1000);
		PWM_SetServo1(dot[2].y);
		PWM_SetServo2(dot[2].x);
		delay_ms(1000);
		PWM_SetServo1(dot[3].y);
		PWM_SetServo2(dot[3].x);
		delay_ms(1000);
		PWM_SetServo1(dot[0].y);
		PWM_SetServo2(dot[0].x);
	}
	if (key[1].state == KEY_SHORT_PRESS)
	{

		PWM_SetServo1(dot[0].y);
		PWM_SetServo2(dot[0].x);
		delay_ms(1000);
		PWM_SetServo1(dot[1].y);
		PWM_SetServo2(dot[1].x);
		delay_ms(1000);
		PWM_SetServo1(dot[2].y);
		PWM_SetServo2(dot[2].x);
		delay_ms(1000);
		PWM_SetServo1(dot[3].y);
		PWM_SetServo2(dot[3].x);
		delay_ms(1000);
		PWM_SetServo1(dot[0].y);
		PWM_SetServo2(dot[0].x);
	}
	if (key[4].state == KEY_SHORT_PRESS)
	{
		// ���λ��
		// int x = seed % 142;
		// int y = seed % 143;
		// if (y % 2)
		// 	PWM_SetServo1(y);
		// else
		// 	PWM_SetServo1(-y);
		// if (x % 2)
		// 	PWM_SetServo2(x);
		// else
		// 	PWM_SetServo2(-x);
	}
	if (key[4].state == KEY_SHORT_PRESS)
	{
		testDot[0].x = 0;
		testDot[0].y = 0;

		testDot[1].x = 150;
		testDot[1].y = 150;

		testDot[2].x = -150;
		testDot[2].y = -150;

		testDot[3].x = 65;
		testDot[3].y = 145;

		int setGO = 3;
		go_line_set(testDot[0].x, testDot[0].y, testDot[1].x, testDot[1].y, setGO);
		go_line_set(testDot[1].x, testDot[1].y, testDot[2].x, testDot[2].y, setGO);
		go_line_set(testDot[2].x, testDot[2].y, testDot[3].x, testDot[3].y, setGO);
		go_line_set(testDot[3].x, testDot[3].y, testDot[0].x, testDot[0].y, setGO);

		// go_line(testDot[0].x, testDot[0].y, testDot[1].x, testDot[1].y);
		// delay_ms(800);
		// go_line(testDot[1].x, testDot[1].y, testDot[2].x, testDot[2].y);
		// delay_ms(800);
		// go_line(testDot[2].x, testDot[2].y, testDot[3].x, testDot[3].y);
		// delay_ms(800);
		// go_line(testDot[3].x, testDot[3].y, testDot[0].x, testDot[0].y);
		// delay_ms(800);
	}

	// ��б��
	// if (key[4].state == KEY_LONG_PRESS)
	// {
	// 	// PWM_SetServo1(-140);
	// 	// delay_ms(500);
	// 	// PWM_SetServo2(-110);
	// 	// delay_ms(500);

	// 	// servo_line(-110, -140, 150, 110);

	// 	PWM_SetServo1(150);
	// 	PWM_SetServo2(180);
	// 	delay_ms(1000);

	// 	// servo_line(180, 150, -150, -180);
	// 	delay_ms(500);
	// }
#endif
}

void key_goline_key(void)
{
	key_scanner();
	for (int i = 0; i < KEY_NUMBER; i++)
	{
		key[i].state = key_get_state(i);
		key[i].lastState = key[i].state;
	}

	if (key[0].state == KEY_SHORT_PRESS)
	{
		// ��ת
		flag_G.line_stop = flag_G.line_stop ^ 1;
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
