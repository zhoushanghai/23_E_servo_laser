#include "Motor.h"
#include "stm32f4xx.h"  

/**
 * @brief �����ʼ������
 * 
 */
void Motor_GPIO_Init(void)
{
    /*��ʼ���ṹ��*/
    GPIO_InitTypeDef GPIO_InitStructure;

    /*����ʱ��*/
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    /*�������� PB12&PB13&PB14&PB15*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/**
 * @brief ��������PWM���޷�����
 *
 * @param motoL ���������ĵ�PWM_CCR
 * @param motoR �����ҵ����PWM_CCR
 */
int PWM_MAX = 9000, PWM_MIN = -7200; // PWM_CCR�޷�����
void Motor_PWM_Limit(int *motoL, int *motoR)
{
    if (*motoL > PWM_MAX)
    {
        *motoL = PWM_MAX;
    }
    if (*motoL < PWM_MIN)
    {
        *motoL = PWM_MIN;
    }

    if (*motoR > PWM_MAX)
    {
        *motoR = PWM_MAX;
    }
    if (*motoR < PWM_MIN)
    {
        *motoR = PWM_MIN;
    }
}

/**
 * @brief ȡ����ֵ����
 * @param Ҫȡ����ֵ����
 * @retval ȡ����ֵ�����
 */
int GFP_abs(int p) // ����ֵ����
{
    int q;
    q = p > 0 ? p : (-p); 
    return q;
}

/**
 * @brief PWM_CCR���������� // ��ڲ�����PID������ɵ�����PWMֵ
 * @param  moto1,moto2,����PWM_CCR��ֵ���ҵ��PWM_CCR��ֵ
 * @retval ��
 */
void Motor_PWM_Load(int moto1, int moto2)
{
    if (moto1 > 0) // ��������ת
    {
        Lin1 = 1, Lin2 = 0;
    }
    else // ��������ת
    {
        Lin1 = 0, Lin2 = 1;
    }                                    
    TIM_SetCompare3(TIM5, GFP_abs(moto1)); // ��Ϊ�˺���ֻ��д����������Ҫȡ����ֵ

    if (moto2 > 0) // ���ҵ����ת
    {
        Rin1 = 1, Rin2 = 0;
    }
    else // ��������ת
    {
        Rin1 = 0, Rin2 = 1;
    }
    TIM_SetCompare4(TIM5, GFP_abs(moto2)); // ��Ϊ�˺���ֻ��д����������Ҫȡ����ֵ
}
