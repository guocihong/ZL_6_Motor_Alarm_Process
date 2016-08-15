#include "task/alarm/alarm_task.h"

/* for beep */
extern bdata  bit     beep_flag;        // 蜂鸣标志 : 0 - 禁鸣; 1 - 正在蜂鸣
extern  data  Uint16  beep_timer;       // 剩余蜂鸣时间, 单位tick
extern xdata  Uint16  beep_during_temp; // 蜂鸣最大持续时间

/* for alarm */
extern bdata  Byte    alarm_out_flag;   //报警输出标志：位值0 - 无报警（继电器上电吸合）;  位值1 - 报警(断电)
										//位址76543210  对应  X X X 报警2 报警1 联动输出 X X
										//ZZX: 报警输出位值与实际硬件控制脚电平相反; 	联动输出位值与实际硬件控制脚电平相同

extern bdata  bit     alarm1_flag;
extern bdata  bit     alarm2_flag;
extern  data  Uint16  alarm1_timer;     // 计时器，报警器1已报警时间,单位tick
extern  data  Uint16  alarm2_timer;     // 计时器，报警器2已报警时间,单位tick

/* variables for 联动 */
extern bdata  bit     adl_alarm_flag;   //左侧张力组合报警标志: 0 - 无报警; 1 - 超阀值，报警
extern bdata  bit     adr_alarm_flag;   //右侧张力组合报警标志: 0 - 无报警; 1 - 超阀值，报警
                                        //报警原因可能为：外力报警， 静态张力超范围报警

/* Doorkeep */
extern bdata  bit     gl_local_dk_status;          //门磁开关状态: 1 - 闭合; 0 - 打开(需要报警)
extern bdata  bit     gl_control_dk_status;        //控制杆门磁状态： 1 - 闭合; 0 - 打开(需要报警)   

/* for system */
extern  data  Byte    system_status;               //系统状态

void alarm_task_init(void)
{
	/* for beep */
	beep_flag = 0;       //无蜂鸣

	/* for alarm */
	alarm_out_flag = 0x00;  //报警口1/2均无报警输出

	adl_alarm_flag = 0;  //初始上电后，左侧无组合报警
	adr_alarm_flag = 0;  //初始上电后，右侧无组合报警
}

void alarm_task(void)
{
	Uint16 temp16;
	
	//门磁被打开，左右防区都报警,两路开关量都断开
	if ((gl_local_dk_status || !gl_control_dk_status) && (system_status >= SYS_SELF_CHECK1)) {	
		Disable_interrupt();
		alarm1_timer = 0;     //清报警器1已报警时间(至少报警3秒)
		Enable_interrupt();
		
		Disable_interrupt();
		alarm2_timer = 0;     //清报警器2已报警时间(至少报警3秒)
		Enable_interrupt();

		if (alarm1_flag == 0) {
			//新报警
			bRelay_Left_Ctrl = 0;	   //报警
			alarm1_flag = 1;   //报警器1报警
			
			//beep
			if (beep_during_temp > 0) {
				//允许beep
				Disable_interrupt();
				beep_timer = beep_during_temp;
				Enable_interrupt();
				if (!beep_flag) {
					//静音 -> 蜂鸣
					bBeep_Ctrl = 1;
					beep_flag = 1;
				}
			}	
		}
		
		if (alarm2_flag == 0) { 
			//新报警
			bRelay_Right_Ctrl = 0;	   //报警
			alarm2_flag = 1;   //报警器2报警
		}	
	}
  
	//报警口1（左侧）
	if (adl_alarm_flag && (system_status >= SYS_SELF_CHECK1)) {
		//左侧张力异常: 新报警或继续报警
		Disable_interrupt();
		alarm1_timer = 0;     //清报警器1已报警时间(至少报警3秒)
		Enable_interrupt();
		if (alarm1_flag == 0) { //新报警
			bRelay_Left_Ctrl = 0;	   //报警
			alarm1_flag = 1;   //报警器1报警
		}
        
        //beep
        if (beep_flag == 0) {//静音
			if (beep_during_temp > 0) {	//允许beep
				Disable_interrupt();
				beep_timer = beep_during_temp;
				Enable_interrupt();
				
				bBeep_Ctrl = 1;//蜂鸣
				beep_flag = 1;
			}
		}
	}

	//报警口2（右侧）
	if (adr_alarm_flag && (system_status >= SYS_SELF_CHECK1)) {	
		//右侧张力异常: 新报警或继续报警
		Disable_interrupt();
		alarm2_timer = 0;    //清报警器2已报警时间(至少报警3秒)
		Enable_interrupt();
		if (alarm2_flag == 0) { 
			//新报警
			bRelay_Right_Ctrl = 0;	   //报警
			alarm2_flag = 1;   //报警器2报警
		}
		
		//beep
        if (beep_flag == 0) {//静音
			if (beep_during_temp > 0) {	//允许beep
				Disable_interrupt();
				beep_timer = beep_during_temp;
				Enable_interrupt();
				
				bBeep_Ctrl = 1;//蜂鸣
				beep_flag = 1;
			}
		}
	}

	//检查报警口1报警时间是否已到
	if (alarm1_flag) { 
		//报警口1正在报警
		Disable_interrupt();
		temp16 = alarm1_timer;
		Enable_interrupt();
		if (temp16 > ALARM_TEMPO) { 
			//报警口1已经到最大报警时间, 停止报警
			alarm1_flag = 0;
			bRelay_Left_Ctrl = 1;
		}
	}

	//检查报警口2报警时间是否已到
	if (alarm2_flag) {	
		//报警口2正在报警
		Disable_interrupt();
		temp16 = alarm2_timer;
		Enable_interrupt();
		if (temp16 > ALARM_TEMPO) { 
			//报警口2已经到最大报警时间, 停止报警
			alarm2_flag = 0;
			bRelay_Right_Ctrl = 1;
		}
	}
}
