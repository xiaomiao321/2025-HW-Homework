#include "pid.hpp"

/**
 *@brief PID初始化函数
 *
 * @param T
 * @param Kp
 * @param Ki
 * @param Kd
 * @param i_max
 * @param out_max
 */
void Class_PID::Init(float t, float kp, float ki, float kd, float iMax, float outMax)
{
    T = t;
    Kp = kp;
    Ki = ki;
    Kd = kd;
    i_max = iMax;
    out_max = outMax;
}

void Class_PID::Set_K_P(float K_P)
{
    this->Kp = K_P;
}

void Class_PID::Set_K_I(float K_I)
{
    this->Ki = K_I;
}

void Class_PID::Set_K_D(float K_D)
{
    this->Kd = K_D;
}

void Class_PID::Set_I_Out_Max(float I_Out_Max)
{
    this->i_max = I_Out_Max;
}

void Class_PID::Set_Out_Max(float Out_Max)
{
    this->out_max = Out_Max;
}

void Class_PID::Set_Target(float Target)
{
    this->target = Target;
}

void Class_PID::Set_Now(float Now)
{
    this->now = Now;
}

void Class_PID::Reset_Integral(float Integral)
{
    integral = Integral;
}

float Class_PID::Get_Out()
{
    return out;
}

float Class_PID::Get_Integral()
{
    return integral;
}

void Class_PID::Calc_PID()
{
    float p = 0.0f;     //计算好的P值
    float i = 0.0f;     //计算好的I值
    float d = 0.0f;     //计算好的D值

    float err = target - now;//误差

    p = Kp * err;       //计算P值
    integral += err * T;//计算I值
    if (i_max != 0)
    {                   //积分限幅使能了
        if (integral > i_max)
        {
            integral = i_max;
        }
        else if (integral < -i_max)
        {
            integral = -i_max;
        }
    }
    i = Ki * integral;  //计算I值
    d = Kd * (err - pre_error) / T;//计算D值
    out = p + i + d;
    if (out_max != 0)
    {
        if (out > out_max)
        {
            out = out_max;
        }
        else if (out < -out_max)
        {
            out = -out_max;
        }
    }
    pre_error = err;//更新误差值
}