#include "config.h"

/* UART2 */
xdata  Byte        recv2_buf[MAX_RecvFrame];    // receiving buffer
xdata  Byte        recv2_state;                 // receive state
xdata  Byte        recv2_timer;                 // receive time-out, 用于字节间超时判定
xdata  Byte        recv2_chksum;                // computed checksum
xdata  Byte        recv2_ctr;                   // reveiving pointer

xdata  Byte        trans2_buf[MAX_TransFrame];  // uart transfer message buffer
xdata  Byte        trans2_ctr;                  // transfer pointer
xdata  Byte        trans2_size;                 // transfer bytes number
xdata  Byte        trans2_chksum;               // computed check-sum of already transfered message

xdata  Byte        uart2_q_index;               // 正在发送某队列项的序号：若为0xFF, 表示没有任何项进入发送流程
xdata  sUART_Q     uart2_send_queue[UART_QUEUE_NUM];     // 串口发送队列
xdata  sUART_Q     uart2_recv_queue[UART_QUEUE_NUM];     // 串口接收队列

/* UART3 */
xdata  Byte        recv3_buf[MAX_RecvFrame];    // receiving buffer
xdata  Byte        recv3_state;                 // receive state
xdata  Byte        recv3_timer;                 // receive time-out, 用于字节间超时判定
xdata  Byte        recv3_chksum;                // computed checksum
xdata  Byte        recv3_ctr;                   // reveiving pointer

xdata  Byte        trans3_buf[MAX_TransFrame];  // uart transfer message buffer
xdata  Byte        trans3_ctr;                  // transfer pointer
xdata  Byte        trans3_size;                 // transfer bytes number
xdata  Byte        trans3_chksum;               // computed check-sum of already transfered message

xdata  Byte        uart3_q_index;               // 正在发送某队列项的序号：若为0xFF, 表示没有任何项进入发送流程
xdata  sUART_Q     uart3_send_queue[UART_QUEUE_NUM];     // 串口发送队列
xdata  sUART_Q     uart3_recv_queue[UART_QUEUE_NUM];     // 串口接收队列

/* UART4 */
xdata  Byte        recv4_buf[MAX_RecvFrame];    // receiving buffer
xdata  Byte        recv4_state;                 // receive state
xdata  Byte        recv4_timer;                 // receive time-out, 用于字节间超时判定
xdata  Byte        recv4_chksum;                // computed checksum
xdata  Byte        recv4_ctr;                   // reveiving pointer

xdata  Byte        trans4_buf[MAX_TransFrame];  // uart transfer message buffer
xdata  Byte        trans4_ctr;                  // transfer pointer
xdata  Byte        trans4_size;                 // transfer bytes number
xdata  Byte        trans4_chksum;               // computed check-sum of already transfered message

xdata  Byte        uart4_q_index;               // 正在发送某队列项的序号：若为0xFF, 表示没有任何项进入发送流程
xdata  sUART_Q     uart4_send_queue[UART_QUEUE_NUM];     // 串口发送队列
xdata  sUART_Q     uart4_recv_queue[UART_QUEUE_NUM];     // 串口接收队列

/* AD sample */
xdata  Uint16      ad_sensor_mask_LR;           //按左右顺序重排序的sensor mask: 杆、左开关量、左6 ~ 1,杆、右开关量、右6 ~ 1
xdata  Uint16      ad_sensor_mask;              //15  14  13  12  11  10  9  8  7   6    5  4  3  2  1  0
												//				  右6			   右1  左6       	   左1	

xdata  Union16     ad_chn_sample[13];           //最新一轮采样值（已均衡去噪声，每通道一个点，循环保存）- 从左至右的顺序为：左1~6、右1~6、杆自身
xdata  Uint16      ad_samp_pnum;                //采样点数(计算静态基准值时总采样点数)
xdata  sAD_Sum     ad_samp_sum[13];             //阶段求和 - 从左至右的顺序为：左1~6、右1~6、杆自身
xdata  sAD_BASE    ad_chn_base[13];             //各通道运行时静态基准值/上下限阀值（单位：采样值）- 从左至右的顺序为：左1~6、右1~6、杆自身
xdata  Byte        ad_chn_over[13];             //各通道连续采样点(均衡后)的阀值判定： 0 - 范围内； 1 - 超阀值 - 从左至右的顺序为：左1~6、右1~6、杆自身
                                                //每通道一个字节： CH0~12 对应 ad_chn_over[0~12]
								                //某字节中的每个位对应顺序采样点的阀值判定

xdata  Uint16      ad_still_dn;                 //静态拉力值下限
xdata  Uint16      ad_still_up;                 //静态拉力值上限
xdata  Byte        ad_still_Dup[13];            //报警阀值上限 - 从左至右的顺序为：左1~6、右1~6、杆自身
 data  Uint16      ad_alarm_exts;               //外力报警标志（未经mask）：位值 0 - 无； 1 - 超阀值                    
 data  Uint16      ad_alarm_base;	            //静态张力报警标志（未经mask）：位值 0 - 允许范围内； 1 - 超允许范围                                   					
												//从左至右的顺序为：杆自身、左开关量、左6~左1,杆自身、右开关量、右6~右1

xdata  Uint16      ad_chnn_state[12];           //用来分析钢丝的松紧程度，从左至右的顺序为：左1~左6,右1~右6

 /* for system */
 data  Byte        system_status;               //系统状态
												// 0 - 初始上电
												// 1 - 基准值采样前延时(约5秒)
												// 2 - 基准值采样(10秒左右)
												// 3 - 开机自检
												// 4 - 实时监测

 data  Byte        gl_comm_addr;                //本模块485通信地址
xdata  Byte        matrix_index[12] = {0, 6, 1, 7, 2, 8, 3, 9, 4, 10, 5, 11};//左1 右1 左2 右2 左3 右3 左4 右4 左5 右5 左6 右6
										 
/* 系统计时 */
xdata  Byte        gl_ack_tick = 0;	            //上位机485口应答延时计时 tick
xdata  Uint16      gl_delay_tick;               //通用延时用tick
 
/* for alarm */
bdata  Byte        alarm_out_flag;              //位址76543210  对应  X X X 报警2 报警1 联动输出 X X
                                                //报警输出标志：位值0 - 无报警（继电器上电吸合）; 位值1 - 报警(断电)
                                                //联动输出标志：位值0 - 无输出（断电,使用常闭）;  位值1 - 有联动输出(继电器上电吸合，开路) 
								                //ZZX: 报警输出位值与实际硬件控制脚电平相反; 	联动输出位值与实际硬件控制脚电平相同							
sbit     alarm2_flag  = alarm_out_flag^4;
sbit     alarm1_flag  = alarm_out_flag^3;
	   
 data  Uint16      alarm1_timer;                //计时器，报警器1已报警时间,单位:tick 
 data  Uint16      alarm2_timer;                //计时器，报警器2已报警时间,单位:tick 
 
/* variables for beep */
bdata  bit         beep_flag;                   //蜂鸣标志: 0 - 禁鸣; 1 - 正在蜂鸣
 data  Uint16      beep_timer;                  //计时器，剩余蜂鸣时间, 单位:tick
xdata  Uint16      beep_during_temp;            //预设的一次蜂鸣持续时间, 单位:tick 

/* Doorkeep(门磁) */
bdata  bit         gl_control_dk_status;        //控制杆门磁状态： 1 - 闭合; 0 - 打开(需要报警)   
bdata  bit         gl_local_dk_status;          //防水箱的门磁状态（每1s动态检测）: 0 - 闭合; 1 - 打开(需要报警)                    
xdata  Byte        gl_local_dk_tick;  	        //防水箱的门磁检测计时tick

/* variables for alarm flag */
bdata  bit         adl_alarm_flag;              //主控板左侧张力组合报警标志: 0 - 无报警; 1 - 报警
bdata  bit         adr_alarm_flag;              //主控板右侧张力组合报警标志: 0 - 无报警; 1 - 报警
											    //  报警原因可能为：外力报警,张力静态基准值超范围报警
											    //  ZZX: 已经过 mask 处理，包含扩展模块，但不包含级联
xdata  Uint16      ad_alarm_tick[13];           //各通道报警计时tick
 
/* 电机堵转检测 */
bdata  bit         gl_motor_overcur_flag;       //电机是否处于堵转状态：0-正常工作;1-电机堵转
bdata  bit         gl_motor_adjust_flag;        //电机是否处于工作状态：0-停止工作状态;1-正处于工作状态
bdata  bit         is_timeout;                  //电机时间是否用完：0-没有;1-时间用完			
xdata  Byte        gl_chnn_index;               //当前正在调整的钢丝的索引
xdata  Byte        gl_motor_overcur_point[12];  //电机堵转次数
xdata  Byte        gl_motor_adjust_end[12];     //是否调整完成：0-没有调整完成;1-表示调整完成
xdata  Uint16      ad_chnn_wire_cut;            //0-表示钢丝没有被剪断;1-表示钢丝被剪断 -->X X 左6~左1、X X 右6~右1
xdata  Byte        gl_wait_delay_tick;          //发出控制电机命令包以后延时等待的时间，然后才同步更新基准值以及是否电机堵转和时间是否用完

xdata  Byte        test;