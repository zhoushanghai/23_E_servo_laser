#include "pid.h"
#define INTEGRAL_CALCULATION_H 0 // PID ���л��ּ���

static void PID_Init(PID *pid, float p, float i, float d, float maxI, float maxOut);
static void Limit(float *val, float max);

CAR_PID pid;

///////////////////////////////////init///////////////////////////////
void PID_set(void)
{
    PID_Init(&pid.speed_l, SPEED_P, SPEED_I, SPEED_D, 0.0, 10000);
    PID_Init(&pid.speed_r, SPEED_P, SPEED_I - 200.0f, SPEED_D, 0.0, 10000);
    PID_Init(&pid.dis, DIS_P, DIS_I, DIS_D, 0.0, 0.3);

    PID_Init(&pid.angle, ANGLE_P, ANGLE_I, ANGLE_D, 0.0, 0.3);

    PID_Init(&pid.angSpe, ANGSPE_P, ANGSPE_I, ANGSPE_D, 0.0, 0.6);

    PID_Init(&pid.gray, GRAY_P, GRAY_I, GRAY_D, 0.0, 0.8);

    PID_Init(&pid.sevor1, SEVOR_P, SEVOR_I, SEVOR_D, 0.0, 2500);
    PID_Init(&pid.sevor2, SEVOR_P, SEVOR_I, SEVOR_D, 0.0, 2500);
}

//(�F����)///////////////////////////////////// ���㺯�� ////////////////////////////////////////////
/*****************
����ʽPID������
******************/
float PID_Incremental(PID *pid, float target, float feedback)
{
    pid->lastError2 = pid->lastError;
    pid->lastError = pid->error;
    pid->error = target - feedback;
    switch (pid->type)
    {
    case PI:
        pid->output += (pid->error - pid->lastError) * pid->kp + pid->error * pid->ki;
        break;
    case PD:
        pid->output += (pid->error - pid->lastError) * pid->kp + (pid->error - 2 * pid->lastError + pid->lastError2) * pid->kd;
        break;
    case P:
        pid->output += (pid->error - pid->lastError) * pid->kp;
        break;
    case PIDALL:
        pid->output += (pid->error - pid->lastError) * pid->kp + pid->error * pid->ki + (pid->error - 2 * pid->lastError + pid->lastError2) * pid->kd;
        break;
    }
    Limit(&pid->output, pid->maxOutput);

    return pid->output;
}

/*****************
λ��ʽPID������
******************/
float PID_Positional(PID *pid, float target, float feedback)
{
    pid->lastError = pid->error;
    pid->error = target - feedback;
    switch (pid->type)
    {
    case PI:
        pid->integral += pid->error * pid->ki;
        Limit(&pid->integral, pid->maxIntegral);
        pid->output = pid->error * pid->kp + pid->integral;
        break;
    case PD:
        pid->output = pid->output = pid->error * pid->kp + (pid->error - pid->lastError) * pid->kd;
        break;
    case P:
        pid->output = pid->error * pid->kp;
        break;

    case PIDALL:
        pid->integral += pid->error * pid->ki;
        Limit(&pid->integral, pid->maxIntegral);
        pid->output = pid->error * pid->kp + pid->integral + (pid->error - pid->lastError) * pid->kd;
        break;
    }
    Limit(&pid->output, pid->maxOutput);

    return pid->output;
}
//(�F����)///////////////////////////////////// �ײ���� ////////////////////////////////////////////
static void Limit(float *val, float max)
{
    if (*val > max)
        *val = max;
    else if (*val < -max)
        *val = -max;
}

// ���ڳ�ʼ��pid�����ĺ���
static void PID_Init(PID *pid, float p, float i, float d, float maxI, float maxOut)
{
    pid->kp = p;
    pid->ki = i;
    pid->kd = d;
    pid->maxIntegral = maxI;
    pid->maxOutput = maxOut;
    // �ж�pid����
    if (i == 0 && d == 0)
    {
        pid->type = P;
    }
    else if (i == 0)
    {
        pid->type = PD;
    }
    else if (d == 0)
    {
        pid->type = PI;
    }
    else
    {
        pid->type = PIDALL;
    }
}

/*****************
�ٶȻ�PI��������Kp*Ek+Ki*Ek_S(Ek_S��ƫ��Ļ���)
******************/
void Velocity(PID *pid, int target, float feedback)
{
    pid->lastError2 = pid->lastError;
    pid->lastError = pid->error;
    pid->error = target - feedback;

    pid->output += (pid->error - pid->lastError) * pid->kp + (pid->error - 2 * pid->lastError + pid->lastError2) * pid->kd;
    Limit(&pid->output, pid->maxOutput);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////

// /*****************
//  ���ٶȻ� PD������
// ��ڣ�Med:��е��ֵ(�����Ƕ�)��Angle:��ʵ�Ƕȣ�gyro_Y:��ʵ���ٶ�
// ���ڣ�ֱ�������
// ******************/
// void TRV_pid_angspe(PID *pid, float reference, float feedback)
// {

//     /*�е�ͨ�˲���*/
//     // static float a = 0.7;
//     // pid->error = feedback - reference;
//     // pid->error = pid->error * a + pid->lastError * (1 - a);
//     // pid->output += (pid->error - pid->lastError) * pid->kp + pid->error * pid->ki;
//     // pid->lastError = pid->error;

//     /*�е�ͨ�˲���*/
//     pid->lastError2 = pid->lastError;
//     pid->lastError = pid->error;
//     pid->error = feedback - reference;
//     pid->output += (pid->error - pid->lastError) * pid->kp + (pid->error - 2 * pid->lastError + pid->lastError2) * pid->kd;

//     // ����޷�
//     Limit(&pid->output, pid->maxOutput);
// }

// /*****************
// ֱ����PD��������Kp*Ek+Kd*Ek_D

// ��ڣ�Med:��е��ֵ(�����Ƕ�)��Angle:��ʵ�Ƕȣ�gyro_Y:��ʵ���ٶ�
// ���ڣ�ֱ�������
// ******************/
// void Vertical(PID *pid, float Med, float Angle)
// {
//     pid->lastError2 = pid->lastError;
//     pid->lastError = pid->error;
//     pid->error = Angle - Med;
//     pid->output += (pid->error - pid->lastError) * pid->kp + (pid->error - 2 * pid->lastError + pid->lastError2) * pid->kd;
// }

// /*****************
// �ٶȻ�PI��������Kp*Ek+Ki*Ek_S(Ek_S��ƫ��Ļ���)
// ******************/
// void Velocity(PID *pid, float Target, float encoder)
// {
//     // ����ɾ�̬�����������ھ�̬�洢����ʹ�ñ���������
//     static float Encoder_Err, EnC_Err_Lowout, EnC_Err_Lowout_last;
//     float a = 0.6;

//     // 1.�����ٶ�ƫ��
//     // ��ȥ���--�ҵ���⣺�ܹ����ٶ�Ϊ"0"�ĽǶȣ����ǻ�е��ֵ��
//     Target = -Target;
//     Encoder_Err = (float)(Target - encoder);
//     // 2.���ٶ�ƫ����е�ͨ�˲�
//     // low_out = (1-a)*Ek+a*low_out_last
//     EnC_Err_Lowout = (1 - a) * Encoder_Err + a * EnC_Err_Lowout_last; // ʹ�ò��θ���ƽ�����˳���Ƶ���ţ������ٶ�ͻ��

//     pid->integral += EnC_Err_Lowout * pid->ki;
//     pid->integral = pid->integral > pid->maxIntegral ? pid->maxIntegral : (pid->integral < (-pid->maxIntegral) ? (-pid->maxIntegral) : pid->integral);

//     // 5.�ٶȻ��������
//     pid->output = pid->kp * EnC_Err_Lowout + pid->integral + pid->kd * (EnC_Err_Lowout - EnC_Err_Lowout_last);
//     EnC_Err_Lowout_last = EnC_Err_Lowout;

//     /////////////////////////////////////////ֱ�Ӹ�////////////////////////////////////////////////////
//     // Target = -Target;
//     // pid->error = Target - encoder;
//     // // 4.�����޷�
//     // pid->integral += pid->error * pid->ki;
//     // pid->integral = pid->integral > pid->maxIntegral ? pid->maxIntegral : (pid->integral < (-pid->maxIntegral) ? (-pid->maxIntegral) : pid->integral);

//     // // 5.�ٶȻ��������
//     // pid->output = pid->kp * pid->error + pid->integral + pid->kd * (pid->error - pid->lastError);
//     // pid->lastError = pid->error;
// }

// ////////////////////////////////////////////////////////////////////////////////////////////////////////
// void flypid_angspe(PID *pid, float reference, float feedback)
// {
//     static float angspe_iout;
//     //-----(�F����)/---------------------- ��ͨ�˲� -----------------------------//
//     // err = reference - feedback; // ������error
//     // pid->error = (1 - a) * err + a * pid->lastError;
//     // // lastErr = EnC_Err_Lowout; // ����error������
//     // // pid->error = EnC_Err_Lowout;

//     pid->error = reference - feedback;
//     angspe_iout = pid->error * pid->ki;
//     pid->integral += angspe_iout;
//     pid->output += (pid->error - pid->lastError) * pid->kp + angspe_iout;
//     Limit(&pid->output, pid->maxOutput);
//     pid->lastError = pid->error;
// }

// void flypid_angle(PID *pid, float reference, float feedback)
// {
//     // static float lastErr, err;
//     // err = reference - feedback;
//     // pid->output = (err)*pid->kp - (err - lastErr) * pid->kd;
//     // lastErr = err;
//     // pid->output += (pid->error - pid->lastError) * pid->kp + (gyro - lastGyro) * pid->kd;
//     pid->lastError2 = pid->lastError;
//     pid->lastError = pid->error;
//     pid->error = reference - feedback;
//     pid->output += (pid->error - pid->lastError) * pid->kp + (pid->error - 2 * pid->lastError + pid->lastError2) * pid->kd;
// }

// void flypid_Velocity(PID *pid, float Target, float encoder)
// {
//     // ����ɾ�̬�����������ھ�̬�洢����ʹ�ñ���������
//     static float Encoder_Err, EnC_Err_Lowout, EnC_Err_Lowout_last;
//     float a = 0.7;

//     // 1.�����ٶ�ƫ��
//     // ��ȥ���--�ҵ���⣺�ܹ����ٶ�Ϊ"0"�ĽǶȣ����ǻ�е��ֵ��
//     Encoder_Err = (float)(Target - encoder);
//     // 2.���ٶ�ƫ����е�ͨ�˲�
//     // low_out = (1-a)*Ek+a*low_out_last
//     EnC_Err_Lowout = (1 - a) * Encoder_Err + a * EnC_Err_Lowout_last; // ʹ�ò��θ���ƽ�����˳���Ƶ���ţ������ٶ�ͻ��
//     EnC_Err_Lowout_last = EnC_Err_Lowout;                             // ��ֹ�ٶȹ���Ӱ��ֱ��������������
//     // 3.���ٶ�ƫ����ֳ�λ��
//     pid->integral += EnC_Err_Lowout * pid->ki;
//     // Encoder_S += EnC_Err_Lowout * pid->ki;
//     // 4.�����޷�
//     pid->integral = pid->integral > pid->maxIntegral ? pid->maxIntegral : (pid->integral < (-pid->maxIntegral) ? (-pid->maxIntegral) : pid->integral);

//     // 5.�ٶȻ��������
//     pid->output = pid->kp * EnC_Err_Lowout + pid->integral;
// }

// void turn_angspe(PID *pid, float reference, float feedback)
// {
//     pid->error = reference - feedback;
//     pid->output = pid->error * pid->kp + (pid->error - pid->lastError) * pid->kd;
//     pid->lastError = pid->error;
//     Limit(&pid->output, pid->maxOutput);
// }

// void turn_midLin(PID *pid, float tra, float angle)
// {

//     pid->error = tra - angle;
//     pid->output = pid->error * pid->kp + (pid->error - pid->lastError) * pid->kd;
//     pid->lastError = pid->error;
// }

// void turn_Velocity(PID *pid, float Target, float encoder)
// {
//     // ����ɾ�̬�����������ھ�̬�洢����ʹ�ñ���������
//     static float Encoder_Err, EnC_Err_Lowout, EnC_Err_Lowout_last;
//     float a = 0.7;

//     // 1.�����ٶ�ƫ��
//     Encoder_Err = (float)(Target - encoder);
//     // 2.���ٶ�ƫ����е�ͨ�˲�
//     // low_out = (1-a)*Ek+a*low_out_last
//     EnC_Err_Lowout = (1 - a) * Encoder_Err + a * EnC_Err_Lowout_last; // ʹ�ò��θ���ƽ�����˳���Ƶ���ţ������ٶ�ͻ��
//     EnC_Err_Lowout_last = EnC_Err_Lowout;                             // ��ֹ�ٶȹ���Ӱ��ֱ��������������
//     // 3.���ٶ�ƫ����ֳ�λ��
//     // pid->integral += EnC_Err_Lowout * pid->ki;
//     // Encoder_S += EnC_Err_Lowout * pid->ki;
//     // 4.�����޷�
//     pid->integral = pid->integral > pid->maxIntegral ? pid->maxIntegral : (pid->integral < (-pid->maxIntegral) ? (-pid->maxIntegral) : pid->integral);

//     // 5.�ٶȻ��������
//     pid->output = pid->kp * EnC_Err_Lowout;
// }
