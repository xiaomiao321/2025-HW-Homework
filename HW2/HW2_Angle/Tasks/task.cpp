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
#include "drv_uart.h"
#include "dvc_serialplot.h"

// 定义电机实例
Class_Motor_GM6020 motor1;
// 定义串口绘图实例
Class_Serialplot serial_plot;

// 串口绘图接收缓冲区
uint8_t Uart6_Rx_Buffer[256];

// 定义要通过串口修改的变量名列表
char Serialplot_Rx_Variable_List[][SERIALPLOT_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH] = {
    "pa",
    "ia",
    "da",
    "po",
    "io",
    "do" };

// 全局角度和角速度变量，方便监控
float g_now_angle = 0.0f;
float g_now_omega = 0.0f;
float g_target_angle = 0.0f;
// 定义π
const float PI = 3.14159265f;

// 串口接收回调函数
void Uart6_Rx_Callback(uint8_t *Buffer, uint16_t Length)
{
  serial_plot.UART_RxCpltCallback(Buffer);
}

/**
 * @brief 主初始化函数
 */
void MainInit(void)
{
  // 初始化电机为角度控制模式，接收ID为0x201
  motor1.Init(&hcan1, 0x201, Control_Method_ANGLE);

  // 初始化串口驱动，使用huart6，并注册回调函数
  Uart_Init(&huart6, Uart6_Rx_Buffer, 256, Uart6_Rx_Callback);
  // 初始化串口绘图，传入变量列表
  serial_plot.Init(&huart6, 6, (char **) Serialplot_Rx_Variable_List);

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
  uint32_t tick = HAL_GetTick();

  // 简单的状态机，用于切换目标角度
  // 每5秒切换一次目标
  int state = (tick / 5000) % 6;

  switch (state)
  {
  case 0:
    g_target_angle = -5.0f * PI / 6.0f;
    break;
  case 1:
    g_target_angle = 5.0f * PI / 6.0f;
    break;
  case 2:
    g_target_angle = PI / 3.0f;
    break;
  case 3:
    g_target_angle = 2.0f * PI / 3.0f;
    break;
  case 4:
    g_target_angle = PI / 4.0f;
    break;
  case 5:
    g_target_angle = -PI;
    break;
  }

  // 设置目标角度
  motor1.Set_Target_Angle(g_target_angle);

  // 执行PID计算和CAN发送
  motor1.TIM_PID_PeriodElapsedCallback();
  g_now_angle = motor1.Get_Now_Angle();
  g_now_omega = motor1.Get_Now_Omega();

  // 检查并更新从串口接收到的参数
  int8_t variable_index = serial_plot.Get_Variable_Index();
  if (variable_index != -1)
  {
    float value = serial_plot.Get_Variable_Value();
    switch (variable_index)
    {
    case 0:
      motor1.PID_Angle.Set_K_P(value);
      break;
    case 1:
      motor1.PID_Angle.Set_K_I(value);
      break;
    case 2:
      motor1.PID_Angle.Set_K_D(value);
      break;
    case 3:
      motor1.PID_Omega.Set_K_P(value);
      break;
    case 4:
      motor1.PID_Omega.Set_K_I(value);
      break;
    case 5:
      motor1.PID_Omega.Set_K_D(value);
      break;
    }
  }

  // 设置串口绘图数据
  serial_plot.Set_Data(3, &g_target_angle, &g_now_angle, &g_now_omega);
}

/**
 * @brief 定时器中断回调函数
 * @param htim 定时器句柄
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim6)
  {
    static int cnt = 0;
    MainTask();
    if (cnt++ >= 9) //100Hz
    {
      serial_plot.TIM_Add_PeriodElapsedCallback();
      cnt = 0;
    }
  }
}