#include "menu.h"

#define onlyshow_PI 1

extern KEY key[KEY_NUMBER];
extern CAR_PID pid;
extern DOT dot[4];
void data_show(void);

unsigned char current_menu = _1_reset, last_menu = MAX_MENU, exit_menu = 0;
unsigned char x = 0, y = 0, PIDchange_falg = 0, showMenu_flag = 0, DIRchange_falg = 0, ANGchange_falg = 0;

// int8_t

// void (*p)(void);

void operation_0_start(void);
void operation_1_gyro(void);
void operation_1_reset(void);
void operation_2_bigsquare(void);
void operation_3_task3(void);
void operation_4_task4(void);
void operation_5_pro_task3(void);
void operation_6_pro_task4(void);
void operation_7_pro_self(void);
void operation_8_reset_point(void);

menu_keyTable menu[MAX_MENU] = {

    {_1_reset, _8_reset_point, _2_bigsquare, _2_bigsquare, _1_reset, (*operation_1_reset)},
    {_2_bigsquare, _1_reset, _3_task3, _3_task3, _2_bigsquare, (*operation_2_bigsquare)},
    {_3_task3, _2_bigsquare, _4_task4, _4_task4, _3_task3, (*operation_3_task3)},
    {_4_task4, _3_task3, _5_pro_task3, _5_pro_task3, _4_task4, (*operation_4_task4)},
    {_5_pro_task3, _4_task4, _6_pro_task4, _6_pro_task4, _5_pro_task3, (*operation_5_pro_task3)},
    {_6_pro_task4, _5_pro_task3, _7_pro_self, _7_pro_self, _6_pro_task4, (*operation_6_pro_task4)},
    {_7_pro_self, _6_pro_task4, _8_reset_point, _8_reset_point, _7_pro_self, (*operation_7_pro_self)},
    {_8_reset_point, _7_pro_self, _1_reset, _1_reset, _8_reset_point, (*operation_8_reset_point)},
};

void menu_1(void)
{
    LCD_Color_Fill(0, 0, 240, 240, GREEN);
    // sprintf(str, "reset");
    LCD_ShowString(1, 1, "reset", BLACK, GREEN, 16, 0);
    // sprintf(str, "bigsquare");
    LCD_ShowString(1, 20, "bigsquare", BLACK, GREEN, 16, 0);
    // sprintf(str, "task3");
    LCD_ShowString(1, 40, "task3", BLACK, GREEN, 16, 0);
    // sprintf(str, "task4");
    LCD_ShowString(1, 60, "task4", BLACK, GREEN, 16, 0);
    // sprintf(str, "pro_task3");
    LCD_ShowString(1, 80, "pro_task3", BLACK, GREEN, 16, 0);
    // sprintf(str, "pro_task4");
    LCD_ShowString(1, 100, "pro_task4", BLACK, GREEN, 16, 0);
    // sprintf(str, "pro_self");
    LCD_ShowString(1, 120, "pro_self", BLACK, GREEN, 16, 0);

    LCD_ShowString(1, 140, "RES_POINT", BLACK, GREEN, 16, 0);
}

void operation_1_reset(void)
{

    if (current_menu != last_menu)
    {
        menu_1();
        LCD_ShowString(1, 1, "reset", WHITE, BLACK, 16, 0);
    }
    if (key[0].state == KEY_SHORT_PRESS)
    {
        LCD_ShowString(1, 200, "reset is going", WHITE, BLACK, 16, 0);

        PWM_SetServo1(0);
        delay_ms(400);
        PWM_SetServo2(0);
        pid.sevor1.output = 0;
        pid.sevor2.output = 0;
        LCD_ShowString(1, 200, "                           ", WHITE, GREEN, 16, 0);
    }
}
void operation_2_bigsquare(void)
{
    if (current_menu != last_menu)
    {
        menu_1();
        LCD_ShowString(1, 20, "bigsquare", WHITE, BLACK, 16, 0);
    }

    if (key[0].state == KEY_SHORT_PRESS)
    {
        LCD_ShowString(1, 200, "_bigsquar is going", WHITE, BLACK, 16, 0);
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

        LCD_ShowString(1, 200, "                           ", WHITE, GREEN, 16, 0);
    }
}

void operation_3_task3(void)
{
    if (current_menu != last_menu)
    {
        menu_1();
        LCD_ShowString(1, 40, "task3", WHITE, BLACK, 16, 0);
    }
    if (key[0].state == KEY_SHORT_PRESS)
    {
        LCD_ShowString(1, 200, "                           ", WHITE, GREEN, 16, 0);
    }
}

void operation_4_task4(void)
{
    if (current_menu != last_menu)
    {
        menu_1();
        LCD_ShowString(1, 60, "task4", WHITE, BLACK, 16, 0);
    }
    if (key[0].state == KEY_SHORT_PRESS)
    {
        LCD_ShowString(1, 200, "reset is going", WHITE, BLACK, 16, 0);

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
        // 使用go_line_se函数
        // int setGO = 1;
        // go_line_set(dot[0].x, dot[0].y, dot[1].x, dot[1].y, setGO);
        // go_line_set(dot[1].x, dot[1].y, dot[2].x, dot[2].y, setGO);
        // go_line_set(dot[2].x, dot[2].y, dot[3].x, dot[3].y, setGO);
        // go_line_set(dot[3].x, dot[3].y, dot[0].x, dot[0].y, setGO);

        // 基础任务（4）随机A4纸边框

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

        LCD_ShowString(1, 200, "                           ", WHITE, GREEN, 16, 0);
    }
}

void operation_5_pro_task3(void)
{
    if (current_menu != last_menu)
    {
        menu_1();
        LCD_ShowString(1, 80, "pro_task3", WHITE, BLACK, 16, 0);
    }
    if (key[0].state == KEY_SHORT_PRESS)
    {
        LCD_ShowString(1, 200, "reset is going", WHITE, BLACK, 16, 0);

        LCD_ShowString(1, 200, "                           ", WHITE, GREEN, 16, 0);
    }
}

void operation_6_pro_task4(void)
{
    if (current_menu != last_menu)
    {
        menu_1();
        LCD_ShowString(1, 100, "pro_task4", WHITE, BLACK, 16, 0);
    }
    if (key[0].state == KEY_SHORT_PRESS)
    {
        LCD_ShowString(1, 200, "reset is going", WHITE, BLACK, 16, 0);
        LCD_ShowString(1, 200, "                           ", WHITE, GREEN, 16, 0);
    }
}

void operation_7_pro_self(void)
{
    if (current_menu != last_menu)
    {
        menu_1();
        LCD_ShowString(1, 120, "pro_self", WHITE, BLACK, 16, 0);
    }
    if (key[0].state == KEY_SHORT_PRESS)
    {
        LCD_ShowString(1, 200, "reset is going", WHITE, BLACK, 16, 0);
        LCD_ShowString(1, 200, "                           ", WHITE, GREEN, 16, 0);
    }
}

void operation_8_reset_point(void)
{
    if (current_menu != last_menu)
    {
        menu_1();
        LCD_ShowString(1, 140, "RES_POINT", WHITE, BLACK, 16, 0);
    }
    if (key[0].state == KEY_SHORT_PRESS)
    {
        LCD_ShowString(1, 200, "reset_point is going", WHITE, BLACK, 16, 0);

        PWM_SetServo1(150); // 舵机回正
        PWM_SetServo2(-150);
        delay_ms(800);
        PWM_SetServo2(150);
        delay_ms(800);
        PWM_SetServo1(0); // 舵机回正
        delay_ms(800);
        PWM_SetServo2(0);

        LCD_ShowString(1, 200, "                           ", WHITE, GREEN, 16, 0);
    }
}

void menu_show(void)
{
    if (key[1].state == KEY_SHORT_PRESS)
    {
        current_menu = menu[current_menu].up;
    }
    if (key[2].state == KEY_SHORT_PRESS)
        current_menu = menu[current_menu].down;
    if (key[4].state == KEY_SHORT_PRESS)
        current_menu = menu[current_menu].enter;
    if (key[3].state == KEY_SHORT_PRESS)
        current_menu = menu[current_menu].quit;

    menu[current_menu].showFun();
    last_menu = current_menu;
}
