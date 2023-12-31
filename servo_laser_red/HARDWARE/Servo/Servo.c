#include "Servo.h"
#define SERVO_LIMIT(Compare) ((Compare) = ((Compare) > 160) ? 160 : ((Compare) < -160) ? -160 \
                                                                                       : (Compare))

/**
 * @brief TIM1_PWM初始化 20ms 50Hz
 *
 */
void PWM_TIM1_Init(void)
{
    /*初始化结构体*/
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    /*开启时钟*/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    /*引脚配置 PA8&PA11*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /*引脚重映射*/
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_TIM1);

    /*定时器TIM1初始化*/
    TIM_TimeBaseStructInit(&TIM_TimeBaseInitStructure);
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 20000 - 1;  // arr
    TIM_TimeBaseInitStructure.TIM_Prescaler = 168 - 1; // psc
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);

    /*配置定时器TIM1为OC_PWM (oc1&oc4)*/
    TIM_OCStructInit(&TIM_OCInitStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;

    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    TIM_OC4Init(TIM1, &TIM_OCInitStructure);

    // TIM_ARRPreloadConfig(TIM1, ENABLE); // ARR影子寄存器配置

    // TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable); // 输出比较影子寄存器配置
    // TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

    /*清空PWM_CCR*/
    TIM_SetCompare1(TIM1, 0);
    TIM_SetCompare4(TIM1, 0);

    TIM_ARRPreloadConfig(TIM1, ENABLE);               // ARR影子寄存器配置
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable); // 输出比较影子寄存器配置
    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

    // 使能TIM1更新中断（在更新事件时触发中断）
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
    // 配置中断优先级（可选步骤）
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;  // 使用TIM1的更新中断
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0; // 抢占优先级
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;        // 子优先级
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    TIM_Cmd(TIM4, ENABLE);

    /*开启定时器TIM1*/
    TIM_Cmd(TIM1, ENABLE);
}
extern SERVO servo;

/**
 * @brief 直接设置舵机1的PWM_CCR 0.5ms-2ms（500-2000
 *
 * @param Compare
 */
void PWM_SetServo1(int16_t Compare)
{
    SERVO_LIMIT(Compare);
    // printf("Servo1 = %d\n", Compare);
    int temp = SERVO_1_DEFAULT - Compare;

    if (Compare > servo.current_y) // 误差补偿
        temp -= 7;
    servo.current_y = Compare;

    TIM_SetCompare1(TIM1, temp);
}

/**
 * @brief 直接设置舵机2的PWM_CCR 0.5ms-2ms（500-2000
 *
 * @param Compare
 */
void PWM_SetServo2(int16_t Compare)
{
    SERVO_LIMIT(Compare);
    // printf("Servo2 = %d\n", Compare);
    int temp = SERVO_2_DEFAULT - Compare;

    if (Compare > servo.current_x) // 误差补偿
        temp -= 5;
    servo.current_x = Compare;

    TIM_SetCompare4(TIM1, temp);
}

/**
 * @brief 直接设置舵机1的角度 0-180度
 *
 * @param Angle
 */
void Servo1_SetAngle(float Angle)
{
    uint16_t Compare;
    if (Angle > 0)
        Compare = (uint16_t)(Angle / 180 * 2000 + 500);
    else
        Compare = (uint16_t)(-(Angle / 180 * 2000 + 500));
    TIM_SetCompare1(TIM1, Compare);
}

/**
 * @brief 直接设置舵机1的角度 0-180度
 *
 * @param Angle
 */
void Servo2_SetAngle(float Angle)
{
    uint16_t Compare;
    if (Angle > 0)
        Compare = (uint16_t)(Angle / 180 * 2000 + 500);
    else
        Compare = (uint16_t)(-(Angle / 180 * 2000 + 500));
    TIM_SetCompare4(TIM1, Compare);
}

// void servo_limit(&int16_t Compare)
//{
//     if (*Compare > 200)
//         *Compare = 200;
//     else if (*Compare < -200)
//         *Compare = -200;
// }

void TIM1_UP_TIM10_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)
    {
        // 处理中断代码
        // ...

        // 清除中断标志
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    }
}
