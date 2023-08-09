#include "sys.h"
#include "usart.h"
#include "pid.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "Servo.h"

extern CAR_PID pid;
extern K210 k210;
extern DOT dot[4];

u8 USART_RX_BUF[USART_REC_LEN]; // ���ջ���,���USART_REC_LEN���ֽ�.
u16 USART_RX_STA = 0;			// ����״̬���

u8 USART_RX_BUF_U1[200]; // ���ջ���,���USART_REC_LEN���ֽ�.
u16 USART_RX_STA_U1 = 0; // ����״̬���
//////////////////////////////////////////////////////////////////////////////////
// ���ʹ��ucos,����������ͷ�ļ�����.
#if SYSTEM_SUPPORT_OS
#include "includes.h" //ucos ʹ��
#endif

static void analysis_data(u8 *buf);
static void change_pid(u8 *buf);
void analysis_k210_data(u8 *buf);
void analysis_square_data(u8 *buf);

//(�F����)/////////////////////////////////////  ////////////////////////////////////////////
void camera2laser(int *dotx, int *doty)
{
	float x = (float)*dotx;
	float y = (float)*doty;
	float x1, x2;

	x1 = 0.1321f * y - 76.962f;
	x2 = -0.0814f * y + 76.618f;
	*dotx = (int)(((x - x1) / (x2 - x1)) * 300 - 150);
	*doty = (int)(0.0037f * y * y + 2.1126f * y - 36.729f);
}

void RX_k210data_deal(void)
{
	// if (USART_RX_BUF_U1[0] != '\0')
	// printf("K210Data:%s\r\n", USART_RX_BUF_U1);
	analysis_k210_data(USART_RX_BUF_U1);
	// analysis_square_data(USART_RX_BUF_U1);

	// if (USART_RX_BUF_U1[0] == '$') // ����ת��
	// {
	// 	dot[0].x = dot[0].x - 160;
	// 	dot[0].y = 120 - dot[0].y;
	// 	dot[1].x = dot[1].x - 160;
	// 	dot[1].y = 120 - dot[1].y;
	// 	dot[2].x = dot[2].x - 160;
	// 	dot[2].y = 120 - dot[2].y;
	// 	dot[3].x = dot[3].x - 160;
	// 	dot[3].y = 120 - dot[3].y;

	// 	camera2laser(&dot[0].x, &dot[0].y);
	// 	camera2laser(&dot[1].x, &dot[1].y);
	// 	camera2laser(&dot[2].x, &dot[2].y);
	// 	camera2laser(&dot[3].x, &dot[3].y);
	// }

	// ��ջ���
	for (int i = 0; i < 10; i++)
	{
		USART_RX_BUF_U1[i] = '\0';
	}
	USART_RX_STA_U1 = 0;
}

/*��������K210������
���ݸ�ʽ�ǣ�
!x��y@
*/
#define K210_FILTER 0.9f

void analysis_k210_data(u8 *buf)
{
	char *start_ptr = strchr((char *)buf, '!'); // �ҵ�'!'��λ��
	char *comma_ptr = strchr((char *)buf, ','); // �ҵ�','��λ��
	char *end_ptr = strchr((char *)buf, '@');	// �ҵ�'@'��λ��

	if (start_ptr != NULL && comma_ptr != NULL && end_ptr != NULL)
	{
		// ������ݰ���ʽ�Ƿ���ȷ
		if (comma_ptr > start_ptr && end_ptr > comma_ptr)
		{
			// ��ȡx��y
			char x_str[10];
			char y_str[10];
			memset(x_str, 0, sizeof(x_str));
			memset(y_str, 0, sizeof(y_str));

			// ����x��y���ַ�������
			strncpy(x_str, start_ptr + 1, comma_ptr - start_ptr - 1);
			strncpy(y_str, comma_ptr + 1, end_ptr - comma_ptr - 1);

			// ����ȡ����x��yת������ֵ
			int16_t x = (int16_t)(atoi(x_str));
			int16_t y = -(int16_t)(atoi(y_str));
			// ��ͨ�˲�
			k210.x += K210_FILTER * ((float)x - k210.x);
			k210.y += K210_FILTER * ((float)y - k210.y);

			// k210.x = (float)x;
			// k210.y = (float)y;

			// printf("x = %.2f, y = %.2f\n", k210.x, k210.y);
		}
	}
}

/*�������εĽǵ�����*/
// typedef struct Point
// {
// 	int x;
// 	int y;
// } Point;

void analysis_square_data(u8 *buf)
{
	char *start_ptr = strchr((char *)buf, '$'); // �ҵ�'$'��λ��
	char *end_ptr = strchr((char *)buf, '%');	// �ҵ�'%'��λ��

	if (start_ptr != NULL && end_ptr != NULL)
	{
		// ������ݰ���ʽ�Ƿ���ȷ
		if (end_ptr > start_ptr + 1)
		{
			// ��ȡ�ĸ����������Ϣ
			// Point dot[4];

			char *ptr = start_ptr + 1;
			for (int i = 0; i < 4; i++)
			{
				// �ҵ����ŵ�λ��
				char *comma_ptr = strchr(ptr, ',');
				if (comma_ptr == NULL || comma_ptr >= end_ptr)
				{
					// �������ݸ�ʽ�����˳�ѭ��
					printf("Error: Invalid data format!\n");
					break;
				}

				// ��ȡx��y���ַ�������
				char x_str[10];
				memset(x_str, 0, sizeof(x_str));
				strncpy(x_str, ptr, comma_ptr - ptr);

				// ����ȡ����xת������ֵ
				dot[i].x = atoi(x_str);

				// �ƶ�ָ�뵽y����ʼλ�ã����ź��棩
				ptr = comma_ptr + 1;

				// �ҵ���һ�����Ż��������λ��
				comma_ptr = strchr(ptr, ',');
				if (comma_ptr == NULL || comma_ptr >= end_ptr)
				{
					// �������һ����������
					if (i == 3)
					{
						// ��ȡy���ַ�������ֱ��end_ptr��
						char y_str[10];
						memset(y_str, 0, sizeof(y_str));
						strncpy(y_str, ptr, end_ptr - ptr);

						// ����ȡ����yת������ֵ
						dot[i].y = atoi(y_str);
					}
					else
					{
						// �������ݸ�ʽ�����˳�ѭ��
						printf("Error: Invalid data format!\n");
						break;
					}
				}
				else
				{
					// ��ȡy���ַ�������
					char y_str[10];
					memset(y_str, 0, sizeof(y_str));
					strncpy(y_str, ptr, comma_ptr - ptr);

					// ����ȡ����yת������ֵ
					dot[i].y = atoi(y_str);

					// �ƶ�ָ�뵽��һ���������ʼλ��
					ptr = comma_ptr + 1;
				}
			}

			// ����ĸ����������Ϣ
			for (int i = 0; i < 4; i++)
			{
				printf("Point %d: x = %d, y = %d\n", i + 1, dot[i].x, dot[i].y);
			}
		}
	}
}

void RX_data_deal(void)
{
	// printf("RX_data_deal\r\n");
	analysis_data(USART_RX_BUF);

	// ��ջ���
	for (int i = 0; i < 32; i++)
	{
		USART_RX_BUF[i] = '\0';
	}
	USART_RX_STA = 0;
}

/*
�趨�ٶȣ�v1.5
�趨ת��a90
��ʾ���Σ�wa
�趨�����1se500 2se2500   (500~2500)
*/
static void analysis_data(u8 *buf)
{

	static float speed = 0.2;
	if (buf[0] != '\0')
	{
		printf("RX data:%s\r\n", USART_RX_BUF);
		if (buf[1] != '\0')
		{
			if (buf[0] == 'v')
			{
				speed = atof((char *)buf + 1);
				servo.speed_set = speed;
				printf("//////////////////////////\r\nspeed:%f\r\n\r\n", speed);
			}
			else if (buf[0] == 'w' && buf[1] == 'a') // ��ʾ����
			{
				flag_G.wave ^= 1;
			}
			else if (buf[0] == 'a') // ת��
			{
				servo.turn_set = atof((char *)buf + 1);
				printf("//////////////////////////\r\nturn:%f\r\n\r\n", servo.turn_set);
			}
			else if (buf[1] == 's' && buf[2] == 'e') // ���
			{
				flag_G.servo = 1;
				if (buf[0] == '1')
				{
					servo.servo1_set = atoi((char *)buf + 3);
					printf("//////////////////////////\r\nservo1:%d\r\n\r\n", servo.servo1_set);
				}
				else if (buf[0] == '2')
				{
					servo.servo2_set = atoi((char *)buf + 3);
					printf("//////////////////////////\r\nservo2:%d\r\n\r\n", servo.servo2_set);
				}
			}
			else if (buf[0] == '*')
				change_pid(buf);
		}
		else
		{
			u8 speChange_flag = 0;
			switch (buf[0])
			{
			case 'G':
				servo.speed_set = speed;
				speChange_flag = 1;
				break;
			case 'S': // ֹͣ
				servo.speed_set = 0;
				speChange_flag = 1;

				servo.status = stop;
				servo.turn_set = 0;
				break;

			case 'Q': // ֹͣ
				// car.dis_set = 4280.0f;
				servo.dis_set = -3860.0f * 4;
				servo.l_distanc = 0;
				servo.r_distanc = 0;

				break;

			case 'E': // ֹͣ
				servo.turn_set = 90.0f;
				break;
				// case 'K':
				// 	car.speed_set = -speed;
				// 	break;
				// case 'D':
				// 	car.speed_set += 1;
				// 	break;
				// case 'E':
				// 	car.speed_set -= 1;
				// 	break;
				// case 'F':
				// 	car.status = CAR_STOP;
				// 	break;
				// case 'R':
				// 	// car.turnSet -= 20;
				// 	car.dirOPEN = 2;
				// 	// flag_G.buzzer = 1;
				// 	break;
				// case 'L':
				// 	// car.turnSet += 20;
				// 	car.dirOPEN = 1;
				// 	break;
				// default:
				// 	break;
			}
			if (speChange_flag)
			{
				speChange_flag = 0;
				printf("set spe:%f \r\n", servo.speed_set);
			}
		}
	}
}

/*******************************
 * �޸�PID
 *
 * ����� d
 *******************************/
static void change_pid(u8 *buf)
{
	static float *val;
	static float *val_l;

	int8_t pidSet = 20;
	PID *SetPID;

	switch (buf[1])
	{
	case 'v':
		SetPID = &pid.speed_r;
		pidSet = 0;
		switch (buf[2])
		{
		case 'p':
			val = &pid.speed_r.kp;
			// val_l = &pid.speed_l.kp;
			break;
		case 'i':
			val = &pid.speed_r.ki;
			// val_l = &pid.speed_l.ki;
			break;
		case 'd':
			val = &pid.speed_r.kd;
			// val_l = &pid.speed_l.kd;
			break;
		}
		break;

	case 'a':
		SetPID = &pid.angle;
		pidSet = 0;
		switch (buf[2])
		{
		case 'p':
			val = &pid.angle.kp;

			break;
		case 'i':
			val = &pid.angle.ki;

			break;
		case 'd':
			val = &pid.angle.kd;

			break;
		}
		break;
		// �����
	case 'd':
		SetPID = &pid.sevor1;
		pidSet = 0;
		switch (buf[2])
		{
		case 'p':
			val = &pid.sevor1.kp;
			val_l = &pid.sevor2.kp;
			break;
		case 'i':
			val = &pid.sevor1.ki;
			val_l = &pid.sevor2.ki;
			break;
		case 'd':
			val = &pid.sevor1.kd;
			val_l = &pid.sevor2.kd;
			break;
		}
		break;
	}

	if (pidSet != 20)
	{
		// buf[0] = '\0';
		// buf[1] = '\0';
		*val = atof((char *)buf + 3);
		if (val_l != NULL)
		{
			*val_l = atof((char *)buf + 3);
			val_l = NULL;
		}
		// *val_l = atof((char *)buf + 3);
		printf("//////////////////////////\r\nPID set P:%f I:%f D:%f\r\n\r\n", SetPID->kp, SetPID->ki, SetPID->kd);
	}
}

//(�F����)///////////////////////////////////// ��ʼ�� ////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////

// �������´���,֧��printf����,������Ҫѡ��use MicroLIB
#if 1
#pragma import(__use_no_semihosting)
// ��׼����Ҫ��֧�ֺ���
struct __FILE
{
	int handle;
};

FILE __stdout;
// ����_sys_exit()�Ա���ʹ�ð�����ģʽ
void _sys_exit(int x)
{
	x = x;
}
// �ض���fputc����
int fputc(int ch, FILE *f)
{
	// while ((USART1->SR & 0X40) == 0)
	// 	; // ѭ������,ֱ���������
	// USART1->DR = (u8)ch;
	// return ch;

	while ((USART3->SR & 0X40) == 0)
		; // ѭ������,ֱ���������
	USART3->DR = (u8)ch;
	return ch;
}
#endif

#if EN_USART1_RX // ���ʹ���˽���
// ����1�жϷ������
// ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���

// ��ʼ��IO ����1
// bound:������
void uart_init(u32 bound)
{
	// GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);  // ʹ��GPIOAʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); // ʹ��USART1ʱ��

	// ����1��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);  // GPIOA9����ΪUSART1
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1); // GPIOA10����ΪUSART1

	// USART1�˿�����
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; // GPIOA9��GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;			// ���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		// �ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;			// ���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;			// ����
	GPIO_Init(GPIOA, &GPIO_InitStructure);					// ��ʼ��PA9��PA10

	// USART1 ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;										// ����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;						// �ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;							// һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;								// ����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // ��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					// �շ�ģʽ
	USART_Init(USART1, &USART_InitStructure);										// ��ʼ������1

	USART_Cmd(USART1, ENABLE); // ʹ�ܴ���1

	// USART_ClearFlag(USART1, USART_FLAG_TC);

#if EN_USART1_RX
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // ��������ж�

	// Usart1 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;		  // ����1�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; // ��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		  // �����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  // IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);							  // ����ָ���Ĳ�����ʼ��VIC�Ĵ�����

#endif

	//-----(�F����)/---------------------- init UART3 -----------------------------//

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);  // ʹ��GPIOAʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); // ʹ��USART3ʱ��

	// ����1��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART2); // GPIOB11����ΪUSART3
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART2); // GPIOB10����ΪUSART3

	// USART1�˿�����
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_10; // GPIOB11��GPIOB10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;			 // ���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 // �ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;			 // ���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;			 // ����
	GPIO_Init(GPIOB, &GPIO_InitStructure);					 // ��ʼ��

	// USART1 ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;										// ����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;						// �ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;							// һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;								// ����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // ��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					// �շ�ģʽ
	USART_Init(USART3, &USART_InitStructure);										// ��ʼ������1

	USART_Cmd(USART3, ENABLE); // ʹ�ܴ���3

	// USART_ClearFlag(USART1, USART_FLAG_TC);

	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE); // ��������ж�

	// Usart1 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;		  // ����1�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; // ��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		  // �����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  // IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);							  // ����ָ���Ĳ�����ʼ��VIC�Ĵ�����
}

void USART1_IRQHandler(void) // ����1�жϷ������
{
	u8 Res;
#if SYSTEM_SUPPORT_OS // ���SYSTEM_SUPPORT_OSΪ�棬����Ҫ֧��OS.
	OSIntEnter();
#endif
	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) // �����ж�(���յ������ݱ�����0x0d 0x0a��β)
	{
		Res = USART_ReceiveData(USART1); //(USART1->DR);	//��ȡ���յ�������
		// if (Res == '@')

		if ((USART_RX_STA_U1 & 0x8000) == 0) // ����δ���
		{
			if (USART_RX_STA_U1 & 0x4000) // ���յ���0x0d
			{
				if (Res != 0x0a)
					USART_RX_STA_U1 = 0; // ���մ���,���¿�ʼ
				else
					USART_RX_STA_U1 |= 0x8000; // ���������
			}
			else // ��û�յ�0X0D
			{
				if (Res == 0x0d)
					USART_RX_STA_U1 |= 0x4000;
				else
				{
					USART_RX_BUF_U1[USART_RX_STA_U1 & 0X3FFF] = Res;
					USART_RX_STA_U1++;
					if (USART_RX_STA_U1 > (USART_REC_LEN - 1))
						USART_RX_STA_U1 = 0; // �������ݴ���,���¿�ʼ����
				}
			}
		}
		if (Res == '@' || Res == '%')
		{
			flag_G.k210 = 1;
			// printf("2ss\r\n");
		}
		// pid ����

		// if (Res == '@')
		// {
		// 	RX_k210data_deal();
		// 	// pid cal
		// 	PID_Incremental(&pid.sevor1, 0, k210.y);
		// 	PID_Incremental(&pid.sevor2, 0, k210.x);
		// 	PWM_SetServo1((uint16_t)pid.sevor1.output);
		// 	PWM_SetServo2((uint16_t)pid.sevor2.output);
		// }
	}
#if SYSTEM_SUPPORT_OS // ���SYSTEM_SUPPORT_OSΪ�棬����Ҫ֧��OS.
	OSIntExit();
#endif
}
#endif

void USART3_IRQHandler(void) // ����3�жϷ������
{
	u8 Res;
#if SYSTEM_SUPPORT_OS // ���SYSTEM_SUPPORT_OSΪ�棬����Ҫ֧��OS.
	OSIntEnter();
#endif
	if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) // �����ж�(���յ������ݱ�����0x0d 0x0a��β)
	{
		Res = USART_ReceiveData(USART3); //(USART3->DR);	//��ȡ���յ�������

		if ((USART_RX_STA & 0x8000) == 0) // ����δ���
		{
			if (USART_RX_STA & 0x4000) // ���յ���0x0d
			{
				if (Res != 0x0a)
					USART_RX_STA = 0; // ���մ���,���¿�ʼ
				else
					USART_RX_STA |= 0x8000; // ���������
			}
			else // ��û�յ�0X0D
			{
				if (Res == 0x0d)
					USART_RX_STA |= 0x4000;
				else
				{
					USART_RX_BUF[USART_RX_STA & 0X3FFF] = Res;
					USART_RX_STA++;
					if (USART_RX_STA > (USART_REC_LEN - 1))
						USART_RX_STA = 0; // �������ݴ���,���¿�ʼ����
				}
			}
		}
	}
#if SYSTEM_SUPPORT_OS // ���SYSTEM_SUPPORT_OSΪ�棬����Ҫ֧��OS.
	OSIntExit();
#endif
}
