#include "control.h"
static int abs(int x);
static void delay_servo(void);

void go_line_set(int x1, int y1, int x2, int y2, int min)
{
    int DeltaY = y2 - y1;
    int DeltaX = x2 - x1;
    if (abs(DeltaY) > abs(DeltaX))
    {
        float k = ((float)DeltaX) / ((float)DeltaY);
        if (DeltaY > 0)
        {
            // 每次移动min个像素
            for (int y = y1; y <= y2; y += min)
            {
                int x = (int)(k * (y - y1) + x1);
                PWM_SetServo1(y);
                PWM_SetServo2(x);
                delay_servo();
            }
        }
        else
        {
            for (int y = y1; y >= y2; y -= min)
            {
                int x = (int)(k * (y - y1) + x1);
                PWM_SetServo1(y);
                PWM_SetServo2(x);
                delay_servo();
            }
        }
    }
    else
    {
        float k = ((float)DeltaY) / ((float)DeltaX);
        if (DeltaX > 0)
        {
            for (int x = x1; x <= x2; x += min)
            {
                int y = (int)(k * (x - x1) + y1);
                PWM_SetServo1(y);
                PWM_SetServo2(x);
                delay_servo();
            }
        }
        else
        {
            for (int x = x1; x >= x2; x -= min)
            {
                int y = (int)(k * (x - x1) + y1);
                PWM_SetServo1(y);
                PWM_SetServo2(x);
                delay_servo();
            }
        }
    }
}
void go_line(int x1, int y1, int x2, int y2)
{
    int DeltaY = y2 - y1;
    int DeltaX = x2 - x1;
    if (abs(DeltaY) > abs(DeltaX))
    {
        float k = ((float)DeltaX) / ((float)DeltaY);
        if (DeltaY > 0)
        {
            for (int y = y1; y <= y2; ++y)
            {
                int x = (int)(k * (y - y1) + x1);
                PWM_SetServo1(y);
                PWM_SetServo2(x);
                delay_servo();
            }
        }
        else
        {
            for (int y = y1; y >= y2; --y)
            {
                int x = (int)(k * (y - y1) + x1);
                PWM_SetServo1(y);
                PWM_SetServo2(x);
                delay_servo();
            }
        }
    }
    else
    {
        float k = ((float)DeltaY) / ((float)DeltaX);
        if (DeltaX > 0)
        {
            for (int x = x1; x <= x2; ++x)
            {
                int y = (int)(k * (x - x1) + y1);
                PWM_SetServo1(y);
                PWM_SetServo2(x);
                delay_servo();
            }
        }
        else
        {
            for (int x = x1; x >= x2; --x)
            {
                int y = (int)(k * (x - x1) + y1);
                PWM_SetServo1(y);
                PWM_SetServo2(x);
                delay_servo();
            }
        }
    }
}

void servo_line(int x1, int y1, int x2, int y2)
{

    char DeltaY = y2 - y1;
    char DeltaX = x2 - x1;

    if (DeltaX == 0)
    {                // 垂直运动
        if (y1 > y2) // 向下
        {
            for (int y = y1; y >= y2; --y)
            {
                PWM_SetServo1(y);
                delay_servo();
            }
        }
        else if (y1 < y2) // 向上
        {
            for (int y = y1; y <= y2; ++y)
            {
                PWM_SetServo1(y);
                delay_servo();
            }
        }
    }
    else if (DeltaY == 0)
    {                // 水平运动
        if (x1 > x2) // 向左
        {
            for (int x = x1; x >= x2; --x)
            {
                PWM_SetServo2(x);
                delay_servo();
            }
        }
        else if (x1 < x2) // 向右
        {
            for (int x = x1; x <= x2; ++x)
            {
                PWM_SetServo2(x);
                delay_servo();
            }
        }
    }

    else
    { // 斜线

        // 判断那条边长
        if (abs(DeltaX) > abs(DeltaY))
        { // X边长

            if (x1 > x2)
            { // 向左
                for (int x = x1; x >= x2; --x)
                {
                    int y = (int)((DeltaY * (x - x1)) / DeltaX + y1);
                    PWM_SetServo1(y);
                    PWM_SetServo2(x);
                    delay_servo();
                }
            }
            else if (x1 < x2)
            { // 向右
                for (int x = x1; x <= x2; ++x)
                {
                    int y = (int)((DeltaY * (x - x1)) / DeltaX + y1);
                    PWM_SetServo1(y);
                    PWM_SetServo2(x);
                    delay_servo();
                }
            }
        }
        else
        { // Y边长

            if (y1 > y2)
            { // 向下
                for (int y = y1; y >= y2; --y)
                {
                    int x = (int)((DeltaX * (y - y1)) / DeltaY + x1);
                    PWM_SetServo1(y);
                    PWM_SetServo2(x);
                    delay_servo();
                }
            }
            else if (y1 < y2)
            { // 向上
                for (int y = y1; y <= y2; ++y)
                {
                    int x = (int)((DeltaX * (y - y1)) / DeltaY + x1);
                    PWM_SetServo1(y);
                    PWM_SetServo2(x);
                    delay_servo();
                }
            }
        }

        //     float k = ((float)DeltaY) / ((float)DeltaX);
        // float b = y2 - k * x2;
        // if (k > -1 && k < 1)
        // {
        //     for (int x = x1; x <= x2; ++x)
        //     {
        //         int y = (int)(k * x + b);
        //         line_width(x, y, width, 1);
        //     }
        // }
        // else
        // { // k >= 1 or k <= -1
        //     int yStart = (y1 <= y2) ? y1 : y2;
        //     int yEnd = (y1 > y2) ? y1 : y2;
        //     for (int y = yStart; y <= yEnd; y++)
        //     {
        //         int x = (int)((y - b) / k);
        //         line_width(x, y, width, 1);
        //     }
        // }
    }
}

static int abs(int x)
{
    return x > 0 ? x : -x;
}

static void delay_servo(void)
{
    delay_ms(20);
    // delay_us(500);
}
