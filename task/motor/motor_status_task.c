#include "task/motor/motor_status_task.h"

#define MT_CONFORM_TEMP1    (30 / SCHEDULER_TICK)  //电机确认延时1
#define MT_CONFORM_TEMP2    (200 / SCHEDULER_TICK) //电机确认延时2

/* 任务状态 */
#define MT_READ_START    0
#define MT_READ_IDLE     1
#define MT_READ_DELAY1   2
#define MT_READ_DELAY2   3

/* for system */
extern idata  Byte     system_status;                 //系统状态

/* 电机堵转检测 */
extern xdata  Byte     gl_motor_overcur_tick;         //电机堵转检测计时tick
extern bdata  bit      gl_motor_adjust_flag;          //电机是否处于工作状态：1-正处于工作状态; 0-停止工作状态
extern bdata  bit      gl_motor_overcur_flag;         //电机堵转标志：0-正常工作；1-电机堵转
static xdata  Byte     bt_read_state;                 //task state

void motor_status_task_init(void)
{
	gl_motor_overcur_tick = 0;
	gl_motor_overcur_flag = 0;                        //上电时，缺省为正常工作
	bt_read_state         = MT_READ_START;
}

void motor_status_task(void)
{
	switch (bt_read_state)
	{
	case MT_READ_START: //开始任务
		if (system_status == SYS_SELF_CHECK) {
			//系统进入自检阶段才开始检查
			bt_read_state = MT_READ_IDLE;
		}
		break;

	case MT_READ_IDLE:
		if (gl_motor_adjust_flag == 1) {//只有电机工作时，才检测电机是否堵转
			if (bMotorOverCur == 0) {//电机可能被堵转，进入延时确认阶段
				gl_motor_overcur_tick = 0;
				bt_read_state = MT_READ_DELAY1;
			}
		}
		break;

	case MT_READ_DELAY1://延时确认
		if (gl_motor_overcur_tick > MT_CONFORM_TEMP1) {
			// 延时时间到，前30ms低电平不予处理，因为存在毛刺干扰
			bt_read_state = MT_READ_DELAY2;
			gl_motor_overcur_tick = 0;
		}
		break;
		
	case MT_READ_DELAY2://延时确认
		if (gl_motor_overcur_tick > MT_CONFORM_TEMP2) {
			// 延时时间到, 判断是否是稳定的变化
			if (bMotorOverCur == 0) {//电机确实为堵转，将电机停止工作
				gl_motor_overcur_flag = 1;
			}
			
			//复位
			bt_read_state = MT_READ_IDLE;
		}
		break;
	}
}