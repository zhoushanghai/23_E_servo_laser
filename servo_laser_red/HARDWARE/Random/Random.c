#include "sys.h"
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include "BEEPER.h"
#include "Random.h"
#include "Key.h"
#include "Encoder.h"

extern void key_goline_key(void);

uint32_t Time_ms = 0; // �������

/**
 * @brief ��ʼ��TIM4(�Ѿ����ú����ȼ���)
 * @param  ARR(��װ��ֵ��//PSC(Ԥ��Ƶֵ)
 * @retval ��
 */
void TIM4_Random_Init(uint16_t ARR, uint16_t PSC)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruture;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	TIM_TimeBaseInitStruture.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStruture.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruture.TIM_Period = ARR; // 1ms
	TIM_TimeBaseInitStruture.TIM_Prescaler = PSC;
	TIM_TimeBaseInitStruture.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStruture);

	TIM_ClearFlag(TIM4, TIM_FLAG_Update);

	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);

	TIM_Cmd(TIM4, ENABLE);
}

/**
 * @brief ��ʼ��TIM7(�Ѿ����ú����ȼ���)
 * @param  ARR(��װ��ֵ��//PSC(Ԥ��Ƶֵ)
 * @retval ��
 */
void TIM7_Random_Init(uint16_t ARR, uint16_t PSC)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruture;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);

	TIM_TimeBaseInitStruture.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStruture.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruture.TIM_Period = ARR; // 1ms
	TIM_TimeBaseInitStruture.TIM_Prescaler = PSC;
	TIM_TimeBaseInitStruture.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseInitStruture);

	TIM_ClearFlag(TIM7, TIM_FLAG_Update);

	TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStructure);

	TIM_Cmd(TIM7, ENABLE);
}
/**
 * @brief ����һ��0��100�������
 * @param  ��
 * @retval ��������õ��������
 */
uint32_t RandomCreate(void)
{
	uint32_t Num = 0;

	srand(Time_ms);

	Num = rand() % 1000; // ����һ��0��100�������//�ĳ�1000��

	return Num;
}

/**
 * @brief �ٲ���һ��0��1000�������
 * @param  Number��srand������
 * @retval ��������õ��������
 */
uint32_t RandomCreateAgain(uint16_t Number)
{
	uint32_t Num = 0;
	srand(Number);
	Num = rand() % 1000;

	return Num;
}

uint8_t Key_count, oled_count;
uint16_t Encoder_count, uart_count;
int L_Encoder;
int R_Encoder;
/**
 * @brief TIM4�жϣ�ms��������
 * @param  ��
 * @retval ��
 */
void TIM4_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) == SET)
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
		Time_ms++;
		// Beeper_Proc();
		if (++Key_count > 10)
		{
			flag_G.key = 1;
			Key_count = 0;

			if (flag_G.go_line)
			{
				key_goline_key();
			}
		}

		if (++Encoder_count > 10)
		{

			// printf("L2ENCDR:%d\r\n", car.l_encoder);
			// printf("R1ENCDR:%d\r\n", car.r_encoder);
			Encoder_count = 0;
		}

		if (++oled_count > 20)
		{
			flag_G.oled = 1;
			oled_count = 0;
		}

		if (++uart_count > 1000)
		{
			flag_G.uart = 1;
			uart_count = 0;
		}
	}
}
