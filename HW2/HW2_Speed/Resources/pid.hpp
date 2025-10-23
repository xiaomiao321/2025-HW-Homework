#ifndef __PID_HPP
#define __PID_HPP
/**
 *@brief PID类
 *
 */
class Class_PID
{
public:
    void Init(float T, float Kp, float Ki, float Kd, float i_max, float out_max);

    void Set_K_P(float K_P);
    void Set_K_I(float K_I);
    void Set_K_D(float K_D);

    void Set_I_Out_Max(float I_Out_Max);
    void Set_Out_Max(float Out_Max);
    void Set_Target(float Target);
    void Set_Now(float Now);
    void Reset_Integral(float Integral);//用于积分清零

    float Get_Out();
    float Get_Integral();       //获取积分值
    void Calc_PID();
private:
    float T = 0.0f;         //计时器周期，单位s
    float Kp = 0.0f;
    float Ki = 0.0f;
    float Kd = 0.0f;
    float i_max = 0.0f;     //抗积分饱和，即积分上限值，0为不限制
    float out_max = 0.0f;   //输出限制，即输出上限值，0为不限制

    float integral = 0.0f;  //积分值
    float pre_error = 0.0f; //上一次误差值
    float error = 0.0f;     //当前误差值
    float target = 0.0f;    //目标值
    float now = 0.0f;       //当前值
    float out = 0.0f;       //输出值

};
#endif // !__PID_HPP
