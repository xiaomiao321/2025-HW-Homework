#include "motor.hpp"
#include <stdint.h>
#include "can.h"
#include "pid.hpp"
#include <cmath> 

void Class_Motor_GM6020::Init(CAN_HandleTypeDef *hcan,
    uint32_t receive_can_id,
    Enum_Control_Method method,
    float max_speed_radps)
{
    this->hcan = hcan;
    this->receive_can_id = receive_can_id;
    this->control_method = method;

    // 根据接收ID判断发送ID和电机索引
    if (this->receive_can_id >= 0x201 && this->receive_can_id <= 0x204)
    {
        this->send_can_id = 0x1FE;// 电流控制
        this->motor_index = this->receive_can_id - 0x201;
    }
    else if (this->receive_can_id >= 0x205 && this->receive_can_id <= 0x208)
    {
        this->send_can_id = 0x2FE;
        this->motor_index = this->receive_can_id - 0x205;
    }

    // 初始化 PID 参数
    PID_Angle.Init(0.001f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f);  // 角度环
    PID_Omega.Init(0.001f, 250.0f, 20.0f, 0.0f, 0.0f, 0.0f); // 速度环

    // 清零输出
    output = 0;
}

void Class_Motor_GM6020::Set_Control_Method(Enum_Control_Method method)
{
    control_method = method;
}

void Class_Motor_GM6020::Set_Target_Omega(float omega_radps)
{
    PID_Omega.Set_Target(omega_radps);
}

void Class_Motor_GM6020::Set_Target_Angle(float angle_rad)
{
    PID_Angle.Set_Target(angle_rad);
}

float Class_Motor_GM6020::Get_Now_Angle() { return now_angle; }
float Class_Motor_GM6020::Get_Now_Omega() { return now_omega; }
float Class_Motor_GM6020::Get_Out() { return output; }

void Class_Motor_GM6020::UpdateAngleAndOmega(uint8_t *data)
{
    uint16_t rx_encoder = (data[0] << 8) | data[1];
    int16_t rx_omega_rpm = (data[2] << 8) | data[3];

    // 判断是否过圈
    int16_t delta = rx_encoder - prev_encoder;
    if (delta < -4096) total_round++;
    else if (delta > 4096) total_round--;

    total_encoder = total_round * ENCODER_PER_ROUND + rx_encoder;
    now_angle = (float) total_encoder / ENCODER_PER_ROUND * 2.0f * M_PI;
    now_omega = rx_omega_rpm * RPM_TO_RADPS;

    prev_encoder = rx_encoder;
}

void Class_Motor_GM6020::CAN_RxCpltCallback(uint8_t *rx_data)
{
    UpdateAngleAndOmega(rx_data);
}

void Class_Motor_GM6020::TIM_PID_PeriodElapsedCallback()
{
    switch (control_method)
    {
    case Control_Method_OMEGA: {
        PID_Omega.Set_Now(now_omega);
        PID_Omega.Calc_PID();
        output = (int16_t) PID_Omega.Get_Out();
        break;
    }
    case Control_Method_ANGLE: {
        PID_Angle.Set_Now(now_angle);
        PID_Angle.Calc_PID();

        float target_omega = PID_Angle.Get_Out();
        PID_Omega.Set_Target(target_omega);
        PID_Omega.Set_Now(now_omega);
        PID_Omega.Calc_PID();

        output = (int16_t) PID_Omega.Get_Out();
        break;
    }
    default:
        output = 0;
    }
    SendOutput();
}

void Class_Motor_GM6020::SendOutput()
{
    uint8_t tx_data[8] = { 0 };

    // 根据电机索引将输出值放入正确的位置
    tx_data[this->motor_index * 2] = output >> 8;
    tx_data[this->motor_index * 2 + 1] = output & 0xFF;

    CAN_TxHeaderTypeDef tx_header;
    tx_header.StdId = this->send_can_id; // 使用正确的发送ID
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.IDE = CAN_ID_STD;
    tx_header.DLC = 8;
    tx_header.TransmitGlobalTime = DISABLE;

    uint32_t tx_mailbox;
    HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data, &tx_mailbox);


}
