/*
 * @Author: hz 1281989921@qq.com
 * @Date: 2023-07-21 17:52:01
 * @LastEditors: hz 1281989921@qq.com
 * @LastEditTime: 2023-07-25 11:35:32
 * @FilePath: \USERe:\����\��ҩС����v1.2 �Ż���һ��key\SYSTEM\sys\sys.h
 * @Description: ����Ĭ������,������`customMade`, ��koroFileHeader�鿴���� ��������: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __SYS_H
#define __SYS_H
#include "stm32f4xx.h"
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>

// 0,��֧��ucos
// 1,֧��ucos
#define SYSTEM_SUPPORT_OS 0 // ����ϵͳ�ļ����Ƿ�֧��UCOS

// λ������,ʵ��51���Ƶ�GPIO���ƹ���
// ����ʵ��˼��,�ο�<<CM3Ȩ��ָ��>>������(87ҳ~92ҳ).M4ͬM3����,ֻ�ǼĴ�����ַ����.
// IO�ڲ����궨��
#define BITBAND(addr, bitnum) ((addr & 0xF0000000) + 0x2000000 + ((addr & 0xFFFFF) << 5) + (bitnum << 2))
#define MEM_ADDR(addr) *((volatile unsigned long *)(addr))
#define BIT_ADDR(addr, bitnum) MEM_ADDR(BITBAND(addr, bitnum))
// IO�ڵ�ַӳ��
#define GPIOA_ODR_Addr (GPIOA_BASE + 20) // 0x40020014
#define GPIOB_ODR_Addr (GPIOB_BASE + 20) // 0x40020414
#define GPIOC_ODR_Addr (GPIOC_BASE + 20) // 0x40020814
#define GPIOD_ODR_Addr (GPIOD_BASE + 20) // 0x40020C14
#define GPIOE_ODR_Addr (GPIOE_BASE + 20) // 0x40021014
#define GPIOF_ODR_Addr (GPIOF_BASE + 20) // 0x40021414
#define GPIOG_ODR_Addr (GPIOG_BASE + 20) // 0x40021814
#define GPIOH_ODR_Addr (GPIOH_BASE + 20) // 0x40021C14
#define GPIOI_ODR_Addr (GPIOI_BASE + 20) // 0x40022014

#define GPIOA_IDR_Addr (GPIOA_BASE + 16) // 0x40020010
#define GPIOB_IDR_Addr (GPIOB_BASE + 16) // 0x40020410
#define GPIOC_IDR_Addr (GPIOC_BASE + 16) // 0x40020810
#define GPIOD_IDR_Addr (GPIOD_BASE + 16) // 0x40020C10
#define GPIOE_IDR_Addr (GPIOE_BASE + 16) // 0x40021010
#define GPIOF_IDR_Addr (GPIOF_BASE + 16) // 0x40021410
#define GPIOG_IDR_Addr (GPIOG_BASE + 16) // 0x40021810
#define GPIOH_IDR_Addr (GPIOH_BASE + 16) // 0x40021C10
#define GPIOI_IDR_Addr (GPIOI_BASE + 16) // 0x40022010

// IO�ڲ���,ֻ�Ե�һ��IO��!
// ȷ��n��ֵС��16!
#define PAout(n) BIT_ADDR(GPIOA_ODR_Addr, n) // ���
#define PAin(n) BIT_ADDR(GPIOA_IDR_Addr, n)  // ����

#define PBout(n) BIT_ADDR(GPIOB_ODR_Addr, n) // ���
#define PBin(n) BIT_ADDR(GPIOB_IDR_Addr, n)  // ����

#define PCout(n) BIT_ADDR(GPIOC_ODR_Addr, n) // ���
#define PCin(n) BIT_ADDR(GPIOC_IDR_Addr, n)  // ����

#define PDout(n) BIT_ADDR(GPIOD_ODR_Addr, n) // ���
#define PDin(n) BIT_ADDR(GPIOD_IDR_Addr, n)  // ����

#define PEout(n) BIT_ADDR(GPIOE_ODR_Addr, n) // ���
#define PEin(n) BIT_ADDR(GPIOE_IDR_Addr, n)  // ����

#define PFout(n) BIT_ADDR(GPIOF_ODR_Addr, n) // ���
#define PFin(n) BIT_ADDR(GPIOF_IDR_Addr, n)  // ����

#define PGout(n) BIT_ADDR(GPIOG_ODR_Addr, n) // ���
#define PGin(n) BIT_ADDR(GPIOG_IDR_Addr, n)  // ����

#define PHout(n) BIT_ADDR(GPIOH_ODR_Addr, n) // ���
#define PHin(n) BIT_ADDR(GPIOH_IDR_Addr, n)  // ����

#define PIout(n) BIT_ADDR(GPIOI_ODR_Addr, n) // ���
#define PIin(n) BIT_ADDR(GPIOI_IDR_Addr, n)  // ����

// ����Ϊ��ຯ��
void WFI_SET(void);      // ִ��WFIָ��
void INTX_DISABLE(void); // �ر������ж�
void INTX_ENABLE(void);  // ���������ж�
void MSR_MSP(u32 addr);  // ���ö�ջ��ַ

///////////////////////////////////////////car data//////////////////////////////////////////////

#define WAVE 0
#define SERVO_1_DEFAULT 1490
#define SERVO_2_DEFAULT 1520

#define TASK_2_UP 145
#define TASK_2_DOWN -145
#define TASK_2_LEFT -149
#define TASK_2_RIGHT 151

typedef enum // car status
{
    straight,
    turn,
    stop
} CAR_STATUS;

typedef struct // С������
{
    short gyro_x, gyro_y, gyro_z;                    // ��������������      gyro (������)
    short acc_x, acc_y, acc_z;                       // ������ٶȼ�����    acc (accelerometer ���ٶȼ�)
    int l_encoder, r_encoder, l_distanc, r_distanc;  // ������ֵ
    float pitch, roll, yaw, speed, r_speed, l_speed; // ������ ����� ƫ����
    float yawPlus;                                   // �ޱ߽�yaw
    float fspeedBal, fspeedTurn;

    CAR_STATUS status; // С��״̬
    float speed_set, speedSet_M, turn_set, dis_set;

    short tofVal; // mm

    int8_t dirOPEN; // 0 stop   1 left    2 right

    float XgyOFFSET, YgyOFFSET, ZgyOFFSET; // ��������Ʈ
    uint8_t GrayscaleData[8];              // �Ҷ�����
    int GrayVal;

    //(�F����)///////////////////////////////////// servo ////////////////////////////////////////////

    int servo1_set, servo2_set; // ������ֵ

    int16_t current_x, current_y; // ��ǰ����

} SERVO;

typedef struct
{
    int8_t oled;
    int8_t key;
    int8_t wave;
    int8_t k210;
    int8_t servo;
    int8_t uart;
    int8_t detect;
    int8_t cm3;
} GLO_FLAG;

typedef struct
{
    // int8_t x, y;
    float x, y;

} K210;

typedef struct
{
    int x, y;

} DOT;

//(�F����)/////////////////////////////////////  ////////////////////////////////////////////

#define CHECK_FLAG(flag) (*(flag) ? (*(flag) = 0, 1) : 0)

//-----(�F����)/---------------------- global -----------------------------//
extern SERVO servo;
extern GLO_FLAG flag_G;
extern char str[20];

#define DIS_EN 0.000259f // 1/3860
#define EN2VAL DIS_EN * 100.0f

#endif
