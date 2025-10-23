/**
 *******************************************************************************
 * @file      :HW_can.cpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2023 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
 /* Includes ------------------------------------------------------------------*/
#include "HW_can.hpp"
#include "stdint.h"
#include "task.hpp"
#include "motor.hpp"
#include "pid.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern Class_Motor_GM6020 motor1;
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/**
 * @brief
 * @param        *hcan:
 * @retval       None
 * @note        None
 */
void CanFilter_Init(CAN_HandleTypeDef *hcan)
{
  CAN_FilterTypeDef canfilter;

  canfilter.FilterActivation = ENABLE;
  canfilter.SlaveStartFilterBank = 14; // CAN2 start filter bank
  canfilter.FilterBank = 0;
  canfilter.FilterFIFOAssignment = CAN_RX_FIFO0;
  canfilter.FilterMode = CAN_FILTERMODE_IDLIST;
  canfilter.FilterScale = CAN_FILTERSCALE_16BIT;

  // ID=0x201
  canfilter.FilterIdHigh = (0x201) << 5;
  canfilter.FilterIdLow = 0;
  canfilter.FilterMaskIdHigh = 0;
  canfilter.FilterMaskIdLow = 0;

  if (HAL_CAN_ConfigFilter(hcan, &canfilter) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief   CAN中断的回调函数，全部数据解析都在该函数中
 * @param   hcan为CAN句柄
 * @retval  none
 * @note
 **/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

  if (rx_header.StdId == motor1.receive_can_id)
  {
    motor1.CAN_RxCpltCallback(rx_data);
  }

  HAL_CAN_ActivateNotification(
    hcan, CAN_IT_RX_FIFO0_MSG_PENDING); // 再次使能FIFO0接收中断
}

/**
 * @brief   向can总线发送数据
 * @param   hcan为CAN句柄
 * @param	msg为发送数组首地址
 * @param	id为发送报文id
 * @param	len为发送数据长度（字节数）
 * @retval  none
 **/
void CAN_Send_Msg(CAN_HandleTypeDef *hcan, uint8_t *msg, uint32_t id,
  uint8_t len)
{
  CAN_TxHeaderTypeDef TxMessageHeader = { 0 };
  uint32_t pTxMailbox;
  TxMessageHeader.StdId = id;
  TxMessageHeader.IDE = CAN_ID_STD;
  TxMessageHeader.RTR = CAN_RTR_DATA;
  TxMessageHeader.DLC = len;
  if (HAL_CAN_AddTxMessage(hcan, &TxMessageHeader, msg, &pTxMailbox) !=
    HAL_OK)
  {
    Error_Handler();
  }
}