#include "driver/timer/timer_drv.h"
#include "driver/flash/flash_drv.h"

/* UART2 */
extern xdata  Byte        recv2_state;                 //receive state
extern xdata  Byte        recv2_timer;                 //receive time-out, 用于字节间超时判定

/* UART3 */
extern xdata  Byte        recv3_state;                 //receive state
extern xdata  Byte        recv3_timer;                 //receive time-out, 用于字节间超时判定

/* UART4 */
extern xdata  Byte        recv4_state;                 //receive state
extern xdata  Byte        recv4_timer;                 //receive time-out, 用于字节间超时判定

/* Doorkeep(门磁) */
extern xdata  Byte        gl_local_dk_tick;            //防水箱的门磁检测计时tick  

/* 系统计时 */
extern xdata  Uint16      gl_delay_tick;               //通用延时用tick
extern xdata  Uint16      gl_ack_tick;	               //应答延时计时 tick

/* for beep */
extern bdata  bit         beep_flag;                   //蜂鸣标志: 0 - 禁鸣; 1 - 正在蜂鸣
extern data   Uint16      beep_timer;                  //计时器，蜂鸣剩余时间, 单位:tick

/* for alarm output (继电器及LED) */
extern bdata  bit         alarm1_flag;                 //左防区报警
extern bdata  bit         alarm2_flag;	               //右防区报警
extern bdata  bit         alarm3_flag;	               //杆自身攀爬报警
extern bdata  bit         alarm4_flag;                 //右开关量攀爬报警	 
extern bdata  bit         alarm5_flag;	               //左开关量攀爬报警

extern data   Uint16      alarm1_timer;                //计时器，左防区已报警时间,单位tick 
extern data   Uint16      alarm2_timer;                //计时器，右防区已报警时间,单位tick
extern data   Uint16      alarm3_timer;                //计时器，杆自身攀爬报警已报警时间,单位:tick 
extern data   Uint16      alarm4_timer;                //计时器，右开关量攀爬报警已报警时间,单位:tick 
extern data   Uint16      alarm5_timer;                //计时器，左开关量攀爬报警已报警时间,单位:tick 
 
/* variables for alarm output */ 
extern xdata  Uint16      ad_alarm_tick[13];           //各通道报警计时tick

/* 电机堵转标志 */
extern xdata  Byte        gl_wait_delay_tick;          //发出控制电机命令包以后延时等待的时间，然后才同步更新基准值以及是否电机堵转和时间是否用完
extern bdata  bit         is_sample_clear;             //采样值是否清零:0-没有清零；1-已经清零
extern xdata  Uint16      check_sample_clear_tick;     //用来检测采样值是否清零成功计时tick

/* for AD */
extern xdata  Uint16      ad_chn_sample[13];           //最新一轮采样值（已均衡去噪声，每通道一个点，循环保存）

void timer0_init(void)   // 5ms@22.1184MHz
{    
    // 定时器0初始化	
	AUXR &= 0x7F;		//定时器时钟12T模式
	TMOD &= 0xF0;		//设置定时器0工作模式为方式0：16位自动重装定时器
	TL0 = 0x00;		    //设置定时初值
	TH0 = 0xDC;		    //设置定时初值
	TF0 = 0;		    //清除TF0标志
	ET0 = 1;            //使能T0中断允许位
	PT0 = 0;            //设置中断优先级为优先级1
	TR0 = 1;		    //定时器0开始计时
}

void timer0_isr(void) interrupt TIMER0_VECTOR
{
	Byte i;
	Uint16 sum;
    
	// increment task tick counters
	gl_local_dk_tick++;                           //防水箱的门磁检测计时tick
    
    if (gl_delay_tick > 0) {
    	gl_delay_tick--;                          //通用延时用tick
    }
    
	if (gl_ack_tick > 0) {
		gl_ack_tick--;                            //应答延时计时
	}
	
	if (gl_wait_delay_tick > 0) {
		gl_wait_delay_tick--;
	}
	
	for (i = 0; i < 13; i++) {
		if (ad_alarm_tick[i] > 0) {
			ad_alarm_tick[i]--;
		}
	}
	
    if (check_sample_clear_tick > 0) {
        check_sample_clear_tick--;
        
        if (check_sample_clear_tick == 0) {//时间到，检测采样值清零是否成功
            sum = 0;
            for (i = 0; i < 8; i++) {
                sum += ad_chn_sample[i];
            }
            
            if (sum < 100) {//采样值清零成功
                is_sample_clear = 1;
                
                flash_enable();
                flash_erase(EEPROM_SECTOR9);
                flash_write(1, EEPROM_SECTOR9 + 1);
                flash_write(0x5a, EEPROM_SECTOR9);
                flash_disable();
            }
        }
    }
    
	/* beep & alarm out */
	if (beep_flag) { 
		//正在beep
		if (beep_timer > 0) {
			beep_timer--;
		}
			
		if (beep_timer == 0) { 
			//蜂鸣时间到，静音
			bBeep_Ctrl = 0;
			beep_flag = 0;
		}
	}

	if (alarm1_flag) {
		alarm1_timer++;
	}
	
	if (alarm2_flag) {
		alarm2_timer++;
	}
 
    if (alarm3_flag) {
		alarm3_timer++;
	}
        
    if (alarm4_flag) {
		alarm4_timer++;
	}
        
    if (alarm5_flag) {
		alarm5_timer++;
	}
        
	// UART2字节之间接收超时
	if (recv2_state != FSA_INIT) { 
		//非初始状态，需要检测是否超时
		if (recv2_timer > 0) {
			recv2_timer--;
		}
		
		if (recv2_timer == 0) {
			recv2_state = FSA_INIT;   //接收超时, 恢复至初始状态			
		}
	}
	
	// UART3字节之间接收超时
	if (recv3_state != FSA_INIT) {
		//非初始状态，需要检测是否超时
		if (recv3_timer > 0) {
			recv3_timer--;
		}
		
		if (recv3_timer == 0){
			recv3_state = FSA_INIT;   //接收超时, 恢复至初始状态
		}
	}
	
	// UART4字节之间接收超时
	if (recv4_state != FSA_INIT) { 
		//非初始状态，需要检测是否超时
		if (recv4_timer > 0) {
			recv4_timer--;
		}
		
		if (recv4_timer == 0) {
			recv4_state = FSA_INIT;   //接收超时, 恢复至初始状态			
		}
	}
}