#include "Encoder.h"
#include "stm32f4xx.h"

/**
 * @brief Ϊ����������ö�ʱ��TIM2
 *
 */
void Encoder_TIM2_Init(void)
{
    /*��ʼ���ṹ��*/
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;

    /*����ʱ��*/
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    /*�������� PA0&PA1*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /*������ӳ��*/
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM2);

    /*��ʼ����ʱ�� TIM2*/
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 65535;    // arr
    TIM_TimeBaseInitStructure.TIM_Prescaler = 1 - 1; // psc
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);

    /*���ö�ʱ�����벶��*/
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 10; // ���벶���˲���
    TIM_ICInit(TIM2, &TIM_ICInitStructure);

    /*���ñ�����*/
    TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

    TIM_SetCounter(TIM2, 0); // ��CNT��0

    /*������ʱ��TIM2*/
    TIM_Cmd(TIM2, ENABLE);
}

/**
 * @brief Ϊ�ұ��������ö�ʱ��TIM3
 *
 */
void Encoder_TIM3_Init(void)
{
    /*��ʼ���ṹ��*/
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;

    /*����ʱ��*/
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    /*�������� PB4&PB5*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; // ��Ϊ�������������ݣ�����ѡ�񸡿�����
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /*������ӳ��*/
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_TIM3);

    /*��ʼ����ʱ�� TIM3*/
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 65535;    // arr
    TIM_TimeBaseInitStructure.TIM_Prescaler = 1 - 1; // psc
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);

    /*���ö�ʱ�����벶��*/
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 10; // ���벶���˲���
    TIM_ICInit(TIM3, &TIM_ICInitStructure);

    /*���ñ�����*/
    TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

    /*������ʱ��TIM3*/
    TIM_Cmd(TIM3, ENABLE);
}

/**
 * @brief ��������ȡ����ٶȺ���
 *
 * @param TIMx ��ʱ��x (2or3)
 * @return int ��ʱ��x ��CNT����
 */
int Encoder_Read_Speed(int TIMx)
{
    int cnt_value; // ����һ��ʱ���ڣ���ʱ���ļ���ֵ

    // ���ؼ���ֵ�����㶨ʱ������ֵ
    switch (TIMx)
    {
    case 2: // �������
        cnt_value = (short)TIM_GetCounter(TIM2);
        TIM_SetCounter(TIM2, 0);
        break;
    case 3: // �ұ�����
        cnt_value = (short)TIM_GetCounter(TIM3);
        TIM_SetCounter(TIM3, 0);
        break;
    default:
        cnt_value = 0;
    }
    return cnt_value;
}

