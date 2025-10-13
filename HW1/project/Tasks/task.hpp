#ifndef __TASK_HPP__
#define __TASK_HPP__

#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>
#include <stdbool.h>
struct CANCommData{
  uint32_t tick;
  float value1;
  uint8_t value2;
  bool flag1;
  bool flag2;
  bool flag3;
  bool flag4;
} ;
extern struct CANCommData CAN_Data_RX;
#include "main.h"
void MainInit(void);
void MainTask(void);
void CAN_RxUnpack(uint8_t *rx_data, struct CANCommData *data);
void CAN_TxPack(struct CANCommData *data, uint8_t *tx_data);
#ifdef __cplusplus
}
#endif
#endif // !__TASK_HPP__