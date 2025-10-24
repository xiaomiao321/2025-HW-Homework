#ifndef __TASK_HPP__
#define __TASK_HPP__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void MainInit(void);
void MainTask(void);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
#include "dvc_serialplot.h"
extern Class_Serialplot serial_plot;
void Uart_Callback(uint8_t *Buffer, uint16_t Length);
#endif

#endif // !__TASK_HPP__
