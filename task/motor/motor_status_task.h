#ifndef _MOTOR_STATUS_TASK_H_
#define _MOTOR_STATUS_TASK_H_

#include "STC15.h"
#include "compiler.h"
#include <intrins.h>
#include "config.h"

void motor_status_task_init(void);
void motor_status_task(void);

#endif