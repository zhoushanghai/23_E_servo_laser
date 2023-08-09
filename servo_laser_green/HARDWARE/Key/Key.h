#ifndef __KEY_H__
#define __KEY_H__

#define SHORT_PRESS 1  // ����
#define LONG_PRESS 2   // ����
#define DOUBLE_PRESS 3 // ˫��

typedef struct
{
	uint16_t Time_1;			  // Time_1�жϳ���ʱ��
	uint16_t Time_2;			  // Time_2�ж�˫��ʱ��
	uint8_t FirstPress_Flag;	  // ��һ�ΰ��±�־
	uint8_t SecondPress_Flag;	  // �ڶ��ΰ��±�־
	uint8_t LongPressOpen_Flag;	  // �Ƿ�򿪳�����־(1��/0�ر�)
	uint8_t DoublePressOpen_Flag; // �Ƿ��˫����־(1��/0�ر�)
} Key_Tag;

/*�ײ㺯��*/
void Key_Init(void);

void TIM2_Key_Init(void);
void Key_Loop(void);
uint8_t Key_GetState(void);
/*Ӧ�ò㺯��*/
uint8_t Key_GetNum(void);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define KEY_RELEASE_LEVEL (GPIO_HIGH) // ������Ĭ��״̬ Ҳ���ǰ����ͷ�״̬�ĵ�ƽ
#define KEY_MAX_SHOCK_PERIOD (20)	  // �����������ʱ�� ��λ���� �������ʱ�����źŻᱻ��Ϊ���Ӳ�����
#define KEY_LONG_PRESS_PERIOD (300)	  // ��С����ʱ�� ��λ���� �������ʱ�����źŻᱻ��Ϊ�ǳ�������
//==================================================���� ���� ��������================================================

//==================================================���� ���� �����ṹ��===============================================

typedef enum
{
	KEY_1,
	KEY_2,
	KEY_3,
	KEY_4,
	KEY_5,
	KEY_NUMBER,
} key_index_enum; // �������� ��Ӧ�Ϸ�����İ������Ÿ��� Ĭ�϶����ĸ�����

typedef enum
{
	KEY_RELEASE,	 // �����ͷ�״̬
	KEY_SHORT_PRESS, // �����̰�״̬
	KEY_LONG_PRESS,	 // ��������״̬
} key_state_enum;

typedef struct
{
	key_state_enum state;
	key_state_enum lastState;
} KEY;
//==================================================���� ���� �����ṹ��===============================================

//==================================================���� ���� ��������===============================================
void key_scanner(void);								// ����״̬ɨ��
key_state_enum key_get_state(key_index_enum key_n); // ��ȡ����״̬
void key_clear_state(key_index_enum key_n);			// ���ָ������״̬
void key_clear_all_state(void);						// ������а���״̬
void key_init(int period);							// ������ʼ��
//==================================================���� ���� ��������===============================================

#endif
