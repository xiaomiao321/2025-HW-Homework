#ifndef __TASK_HPP__
#define __TASK_HPP__

#ifdef __cplusplus
#include "dvc_serialplot.h"
extern "C" {
#endif

#include "main.h"

void MainInit(void);
void MainTask(void);

#ifdef __cplusplus
}
extern Class_Serialplot serial_plot;
#endif
#endif // !__TASK_HPP__
