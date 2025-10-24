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
#include "dvc_serialplot.h"
#include "drv_uart.h"

// 定义电机实例
Class_Motor_GM6020 motor1;
// 全局角速度变量，方便监控
float g_now_omega = 0.0f;

// 串口绘图对象
Class_Serialplot serial_plot;
// 串口接收缓冲
uint8_t uart_rx_buff[256];

// 串口接收指令字典
char serialplot_rx_variable_assignment_list[3][SERIALPLOT_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH] =
{
    "kp",
    "ki",
    "kd",
};
char *serialplot_rx_variable_assignment_list_ptr[3] =
{
    serialplot_rx_variable_assignment_list[0],
    serialplot_rx_variable_assignment_list[1],
    serialplot_rx_variable_assignment_list[2],
};

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

  // 初始化串口
  Uart_Init(&huart6, uart_rx_buff, 256, Uart_Callback);
  // 初始化串口绘图
  serial_plot.Init(&huart6, 3, serialplot_rx_variable_assignment_list_ptr);

  // 启动用于调用MainTask的定时器 (TIM6)
  HAL_TIM_Base_Start_IT(&htim6);
}

/**
 * @brief 主任务，由定时器中断以1kHz频率调用
 */
void MainTask(void)
{
  static int task_count = 0;
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

  // 每10ms发送一次数据
  if (task_count % 10 == 0)
  {
    // 设置发送数据
    serial_plot.Set_Data(2, &target_omega, &g_now_omega);
    // 准备发送数据
    serial_plot.TIM_Add_PeriodElapsedCallback();
    // 发送数据
    UART_Send_Data(&huart6, UART3_Tx_Data, 1 + 4 * 2);
  }
  task_count++;
}

/**
 * @brief 串口接收回调函数
 * @param Buffer 接收数据指针
 * @param Length 接收数据长度
 */
void Uart_Callback(uint8_t *Buffer, uint16_t Length)
{
  serial_plot.UART_RxCpltCallback(Buffer);
  int8_t index = serial_plot.Get_Variable_Index();
  double value = serial_plot.Get_Variable_Value();
  if (index == 0) //kp
  {
    motor1.PID_Omega.Set_K_P(value);
  }
  else if (index == 1) //ki
  {
    motor1.PID_Omega.Set_K_I(value);
  }
  else if (index == 2) //kd
  {
    motor1.PID_Omega.Set_K_D(value);
  }
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
