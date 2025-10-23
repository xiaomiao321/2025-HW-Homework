#include "task.hpp"
#include "HW_can.hpp"
#include "can.h"
#include "dma.h"
#include "gpio.h"
#include "main.h"
#include "stdint.h"
#include "tim.h"
#include "usart.h"
#include "motor.hpp"
#include "pid.hpp"
#include <cmath> 

// 定义电机实例
Class_Motor_GM6020 motor1;
// 全局角速度变量，方便监控
float g_now_omega = 0.0f;
/**
 * @brief 主初始化函数
 */
void MainInit(void)
{
  // 初始化电机为速度控制模式，接收ID为0x201
  motor1.Init(&hcan1, 0x201, Control_Method_OMEGA);

  // 初始化CAN过滤器并启动CAN
  CanFilter_Init(&hcan1);
  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  // 启动用于调用MainTask的定时器 (TIM6)
  HAL_TIM_Base_Start_IT(&htim6);
}

/**
 * @brief 主任务，由定时器中断以1kHz频率调用
 */
void MainTask(void)
{
  // 获取时间，单位为秒
  float time_s = HAL_GetTick() / 1000.0f;

  // 定义正弦曲线参数
  const float amplitude = 15.0f; // 振幅
  const float frequency = 0.5f;  // 频率

  // 计算目标速度 (rad/s)
  float target_omega = amplitude * sin(2.0f * M_PI * frequency * time_s);

  // 设置目标速度
  motor1.Set_Target_Omega(target_omega);

  // 执行PID计算和CAN发送
  motor1.TIM_PID_PeriodElapsedCallback();
  g_now_omega = motor1.Get_Now_Omega();

}

/**
 * @brief 定时器中断回调函数
 * @param htim 定时器句柄
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim6)
  {
    MainTask();
  }
}
