#include "PWM.h"
#include "stm32f4xx.h"

void PWM_TIM5_Init(uint16_t arr, uint16_t psc)
{
    /*��ʼ���ṹ��*/
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    /*����ʱ��*/
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    /*�������� PA2&PA3*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /*������ӳ��*/
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM5);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM5);

    /*��ʼ����ʱ��TIM5*/
    TIM_DeInit(TIM5);
    TIM_TimeBaseStructInit(&TIM_TimeBaseInitStructure);
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = arr;    // arr
    TIM_TimeBaseInitStructure.TIM_Prescaler = psc; // psc
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseInitStructure);
    TIM_CtrlPWMOutputs(TIM5, ENABLE);

    /*���ö�ʱ��TIM5ΪOC_PWM (oc3&oc4)*/
    TIM_OCStructInit(&TIM_OCInitStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;

    TIM_OC3Init(TIM5, &TIM_OCInitStructure);
    TIM_OC4Init(TIM5, &TIM_OCInitStructure);

    TIM_ARRPreloadConfig(TIM5, ENABLE); // ARRӰ�ӼĴ�������

    TIM_OC3PreloadConfig(TIM5, TIM_OCPreload_Enable); // ����Ƚ�Ӱ�ӼĴ�������
    TIM_OC4PreloadConfig(TIM5, TIM_OCPreload_Enable);

    /*������ʱ��TIM5*/
    TIM_Cmd(TIM5, ENABLE);
}
