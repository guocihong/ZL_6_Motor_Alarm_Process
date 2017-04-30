#ifndef _ADC_TASK_H_
#define _ADC_TASK_H_

#include "task/uart/uart_task.h"
#include "config.h"

void adc_task_init(void);
void adc_task(void);
void check_still_stress(Byte index);
void motor_adjust(Byte index);
void update_led_status(void);
void update_alarm_status(void);
void save_alarm_detail_info(void);
#endif