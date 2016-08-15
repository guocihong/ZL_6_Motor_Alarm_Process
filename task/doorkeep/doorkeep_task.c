#include "task/doorkeep/doorkeep_task.h"

#define DK_CHECK_INTERVAL      (1000 / SCHEDULER_TICK)	        //门磁开关检查周期
#define DK_CONFORM_DELAY       ( 100 / SCHEDULER_TICK)           //门磁开关变化确认延时

#define DK_READ_START          0
#define DK_READ_IDLE           1
#define DK_READ_DELAY          2

/* Doorkeep(门磁) */
extern bdata  bit         gl_local_dk_status;          //门磁开关状态（每1s动态检测）: 0 - 闭合; 1 - 打开(需要报警)                    
extern xdata  Byte        gl_local_dk_tick;  	       //门磁检测计时tick
static xdata  Byte        dk_read_state;               //task state

/* for system */
extern  data  Byte        system_status;               //系统状态

void doorkeep_task_init(void)
{
	gl_local_dk_status = 0;                                  //上电时，缺省为闭合
	dk_read_state = DK_READ_START;
}

void doorkeep_task(void)
{	
	switch (dk_read_state)
	{
	case DK_READ_START: //开始任务
		if (system_status >= SYS_SELF_CHECK1) {//系统进入自检阶段才开始检查门磁
			gl_local_dk_tick = 0;
			dk_read_state = DK_READ_IDLE;
		}
		break;

	case DK_READ_IDLE://检测门磁有无变化
		if (gl_local_dk_tick > DK_CHECK_INTERVAL) {//每秒检测一次
			gl_local_dk_tick = 0;
			if (gl_local_dk_status == 0) {//原来为门磁闭合
				if (bDoorKeeper == 1) {//门磁可能被打开, 进入延时确认阶段
					dk_read_state = DK_READ_DELAY;
				}
			} else if (gl_local_dk_status == 1) {//原来为门磁打开
				if (bDoorKeeper == 0) {//门磁可能被闭合, 进入延时确认阶段
					dk_read_state = DK_READ_DELAY;
				} 
			}
		}
		break;

	case DK_READ_DELAY://延时确认
		if (gl_local_dk_tick > DK_CONFORM_DELAY) {// 延时时间到, 判断是否是稳定的变化
			if ((gl_local_dk_status == 0) && (bDoorKeeper == 1)) {// 门磁已经打开								
				gl_local_dk_status = 1;//更新变量
			} else if ((gl_local_dk_status == 1) && (bDoorKeeper == 0)) {// 门磁已经闭合
				gl_local_dk_status = 0;//更新变量
			}

			gl_local_dk_tick = 0;
			dk_read_state = DK_READ_IDLE;
		}
		break;
	}
}
