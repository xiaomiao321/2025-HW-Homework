#ifndef MOTOR_GM6020_H
#define MOTOR_GM6020_H

#include "pid.hpp"
#include "stm32f4xx_hal.h"

// 控制方式枚举
enum Enum_Control_Method
{
    Control_Method_OMEGA = 1,   // 速度控制
    Control_Method_ANGLE = 2    // 角度控制
};

class Class_Motor_GM6020
{
public:
    Class_PID PID_Angle;   // 角度环 PID
    Class_PID PID_Omega;   // 速度环 PID

    // 初始化电机
    void Init(CAN_HandleTypeDef *hcan,
        uint32_t receive_can_id, // 电机回传数据的ID (0x201-0x208)
        Enum_Control_Method method = Control_Method_ANGLE,
        float max_speed_radps = 320.0f * (2.0f * 3.14159265f / 60.0f));

    // 设置控制模式
    void Set_Control_Method(Enum_Control_Method method);

    // 设置目标值
    void Set_Target_Omega(float omega_radps);      // rad/s
    void Set_Target_Angle(float angle_rad);        // rad

    // 获取当前状态
    float Get_Now_Angle();      // 当前角度 (rad)
    float Get_Now_Omega();      // 当前速度 (rad/s)
    float Get_Out();            // 输出量 (-30000 ~ 30000)
    uint32_t receive_can_id; // 电机回传ID

    // CAN 接收回调（由CAN中断调用）
    void CAN_RxCpltCallback(uint8_t *rx_data);

    // 定时器周期调用（执行PID计算）
    void TIM_PID_PeriodElapsedCallback();

private:
    // 配置参数
    CAN_HandleTypeDef *hcan;
    uint32_t send_can_id;    // 电机控制ID
    uint8_t motor_index;     // 电机在CAN ID组中的索引 (0-3)
    Enum_Control_Method control_method;

    // 编码器相关
    int32_t total_encoder = 0;
    int32_t total_round = 0;
    uint16_t prev_encoder = 0;
    float now_angle = 0.0f;
    float now_omega = 0.0f;

    // 输出
    int16_t output = 0;         // 发送给电机的值

    // 常量
    static constexpr uint16_t ENCODER_PER_ROUND = 8192;
    static constexpr float RPM_TO_RADPS = 2.0f * 3.14159265f / 60.0f;
    static constexpr int16_t OUTPUT_MAX = 30000;

    // 内部函数
    void UpdateAngleAndOmega(uint8_t *data);
    void SendOutput();
};

#endif