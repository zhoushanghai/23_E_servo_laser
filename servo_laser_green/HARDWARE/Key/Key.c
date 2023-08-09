#include "Delay.h"
#include "stm32f4xx.h" // Device header
#include "BEEPER.h"
#include "key.h"
#include "led.h"

static int scanner_period = 10;				 // ������ɨ������
static int key_press_time[KEY_NUMBER] = {0}; // �����źų���ʱ��
static key_state_enum key_state[KEY_NUMBER]; // ����״̬
//(�F����)///////////////////////////////////// �ײ���� ////////////////////////////////////////////
//-------------------------------------------------------------------------------------------------------------------
// �������     ������ʼ��
// ����˵��     period          ����ɨ������ �Ժ���Ϊ��λ
// ���ز���     void
// ʹ��ʾ��     key_init(10);
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void key_init(int period)
{

	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	// ����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	// �����尴��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_5 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	scanner_period = period;
}

void get_key_gpio(char *keyVal)
{
	keyVal[1] = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_3);
	keyVal[0] = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5);
	keyVal[2] = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7);
	keyVal[3] = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_1);
	keyVal[4] = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_0);
}
//(�F����)///////////////////////////////////// Ӧ�ò� //////////////////////////////////////////////
/*�����Ǹ�ʽ*/

// void key_(void)
// {
// 	key_scanner();
// 	for (int i = 0; i < KEY_NUMBER; i++)
// 	{
// 		key[i].state = key_get_state(i);
// 		if (key[i].state == KEY_SHORT_PRESS)
// 		{
// 			static uint8_t go = 1;

// 			if (go++ % 2)
// 				Motor_PWM_Load(9000, -9000);
// 			// car.speed_set = 0.5;
// 			else
// 				Motor_PWM_Load(0, 0);
// 			// car.speed_set = 0;

// 			printf("key%d short press%d\r\n", i, go);
// 		}
// 		else if (key[i].state == KEY_LONG_PRESS)
// 		{

// 			if (key[i].lastState != KEY_LONG_PRESS)
// 			{
// 				printf("key%d long press\r\n", i);
// 				// flag_G.wave = flag_G.wave ^ 1;
// 			}
// 		}
// 		else if (key[i].state == KEY_RELEASE)
// 		{

// 			if (key[i].lastState != KEY_RELEASE)
// 			{
// 				printf("key%d KEY_RELEASE\r\n", i);
// 			}
// 		}
// 		key[i].lastState = key[i].state;
// 	}
// }

//(�F����)///////////////////////////////////// �м�� ////////////////////////////////////////////

//-------------------------------------------------------------------------------------------------------------------
// �������     ����״̬ɨ��
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     key_scanner();
// ��ע��Ϣ     �������������ѭ������ PIT �ж���
//-------------------------------------------------------------------------------------------------------------------
void key_scanner(void)
{
	char i = 0, keyVal[KEY_NUMBER];
	get_key_gpio(keyVal);
	//-----(�F����)/---------------------- ɨ�谴����ƽ -----------------------------//
	for (i = 0; i < KEY_NUMBER; i++)
	{
		if (!keyVal[i]) // ��������
		{
			key_press_time[i]++;
			// printf("t%d=%d\r\n", keyVal[0], key_press_time[i]);
			if (key_press_time[i] >= KEY_LONG_PRESS_PERIOD / scanner_period)
			{
				key_state[i] = KEY_LONG_PRESS;
			}
		}
		else // �����ͷ�
		{
			if (key_state[i] != KEY_LONG_PRESS && key_press_time[i] >= KEY_MAX_SHOCK_PERIOD / scanner_period)
			{
				key_state[i] = KEY_SHORT_PRESS;
			}
			else
			{
				key_state[i] = KEY_RELEASE;
			}
			key_press_time[i] = 0;
		}

		// printf("key%d=%d\r\n", i, key_state[i]);
	}
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡ����״̬
// ����˵��     key_n           ��������
// ���ز���     key_state_enum  ����״̬
// ʹ��ʾ��     key_get_state(KEY_1);
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
key_state_enum key_get_state(key_index_enum key_n)
{
	return key_state[key_n];
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ���ָ������״̬
// ����˵��     key_n           ��������
// ���ز���     void            ��
// ʹ��ʾ��     key_clear_state(KEY_1);
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void key_clear_state(key_index_enum key_n)
{
	key_state[key_n] = KEY_RELEASE;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ������а���״̬
// ����˵��     void            ��
// ���ز���     void            ��
// ʹ��ʾ��     key_clear_all_state();
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void key_clear_all_state(void)
{
	for (int i = 0; i < KEY_NUMBER; i++)
	{
		key_state[i] = KEY_RELEASE;
	}
}
