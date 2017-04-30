#include "task/uart/uart_task.h"

#define REPLY_DLY      (100/SCHEDULER_TICK)         //收到PC命令后的应答延时

/* UART2 */
extern xdata  Byte     recv2_buf[MAX_RecvFrame];    // receiving buffer
extern xdata  Byte     recv2_state;                 // receive state
extern xdata  Byte     recv2_timer;                 // receive time-out, 用于字节间超时判定
extern xdata  Byte     recv2_chksum;                // computed checksum
extern xdata  Byte     recv2_ctr;                   // reveiving pointer

extern xdata  Byte     trans2_buf[MAX_TransFrame];  // uart transfer message buffer
extern xdata  Byte     trans2_ctr;                  // transfer pointer
extern xdata  Byte     trans2_size;                 // transfer bytes number
extern xdata  Byte     trans2_chksum;               // computed check-sum of already transfered message

extern xdata  Byte     uart2_q_index;               // 正在发送某队列项的序号：若为0xFF, 表示没有任何项进入发送流程
extern xdata  sUART_Q  uart2_send_queue[UART_QUEUE_NUM];     // 串口发送队列
extern xdata  sUART_Q  uart2_recv_queue[UART_QUEUE_NUM];     // 串口接收队列

/* UART3 */
extern xdata  Byte     recv3_buf[MAX_RecvFrame];    // receiving buffer
extern xdata  Byte     recv3_state;                 // receive state
extern xdata  Byte     recv3_timer;                 // receive time-out, 用于字节间超时判定
extern xdata  Byte     recv3_chksum;                // computed checksum
extern xdata  Byte     recv3_ctr;                   // reveiving pointer

extern xdata  Byte     trans3_buf[MAX_TransFrame];  // uart transfer message buffer
extern xdata  Byte     trans3_ctr;                  // transfer pointer
extern xdata  Byte     trans3_size;                 // transfer bytes number
extern xdata  Byte     trans3_chksum;               // computed check-sum of already transfered message

extern xdata  Byte     uart3_q_index;               // 正在发送某队列项的序号：若为0xFF, 表示没有任何项进入发送流程
extern xdata  sUART_Q  uart3_send_queue[UART_QUEUE_NUM];     // 串口发送队列
extern xdata  sUART_Q  uart3_recv_queue[UART_QUEUE_NUM];     // 串口接收队列

/* UART4 */
extern xdata  Byte     recv4_buf[MAX_RecvFrame];    // receiving buffer
extern xdata  Byte     recv4_state;                 // receive state
extern xdata  Byte     recv4_timer;                 // receive time-out, 用于字节间超时判定
extern xdata  Byte     recv4_chksum;                // computed checksum
extern xdata  Byte     recv4_ctr;                   // reveiving pointer

extern xdata  Byte     trans4_buf[MAX_TransFrame];  // uart transfer message buffer
extern xdata  Byte     trans4_ctr;                  // transfer pointer
extern xdata  Byte     trans4_size;                 // transfer bytes number
extern xdata  Byte     trans4_chksum;               // computed check-sum of already transfered message

extern xdata  Byte     uart4_q_index;               // 正在发送某队列项的序号：若为0xFF, 表示没有任何项进入发送流程
extern xdata  sUART_Q  uart4_send_queue[UART_QUEUE_NUM];     // 串口发送队列
extern xdata  sUART_Q  uart4_recv_queue[UART_QUEUE_NUM];     // 串口接收队列

/* 系统计时 */
extern xdata  Uint16   gl_ack_tick;	                //应答延时计时 tick
extern xdata  Byte     gl_reply_tick;               //设备返回延时

/* for system */
extern  data  Byte     gl_comm_addr;                //本模块485通信地址
extern  data  Byte     system_status;               //系统状态
extern xdata  Byte     matrix_index[12];

/* for alarm */
extern bdata  Byte     alarm_out_flag;              //位址76543210  对应  左开关量攀爬报警 右开关量攀爬报警 杆自身攀爬报警 右防区报警 左防区报警 X X X
													//报警输出标志：位值0 - 无报警（继电器上电吸合）; 位值1 - 报警(断电)
													//联动输出标志：位值0 - 无输出（断电,使用常闭）;  位值1 - 有联动输出(继电器上电吸合，开路)
													//ZZX: 报警输出位值与实际硬件控制脚电平相反; 	联动输出位值与实际硬件控制脚电平相同

/* for beep */
extern xdata  volatile Uint16   beep_during_temp;            // 预设的一次蜂鸣持续时间, 单位:tick

/* Doorkeep(门磁) */
extern bdata  bit      gl_local_dk_status;          //门磁开关状态（每1s动态检测）: 1 - 闭合; 0 - 打开(需要报警)
extern bdata  bit      gl_control_dk_status;        //控制杆的门磁状态: 1 - 闭合; 0 - 打开(需要报警)                    

/* AD sample */
extern xdata  Uint16   ad_sensor_mask_LR;           //按左右顺序重排序的sensor mask: 杆、左开关量、左6 ~ 1,杆、右开关量、右6 ~ 1
extern xdata  Uint16   ad_chn_sample[13];           //最新一轮采样值（已均衡去噪声，每通道一个点，循环保存）
extern xdata  sAD_BASE ad_chn_base[13];             //各通道运行时静态基准值/上下限阀值（单位：采样值）
extern  data  Uint16   ad_alarm_exts;               //外力报警标志（无mask）： 位值 0 - 无； 1 - 超阀值
extern  data  Uint16   ad_alarm_base;	            //静态张力报警标志（无mask）： 位值 0 - 允许范围内； 1 - 超允许范围
extern xdata  Uint16   ad_still_dn;                 //静态拉力值下限
extern xdata  Uint16   ad_still_up;                 //静态拉力值上限
extern xdata  Byte     ad_still_Dup[13];            //报警阀值上限
extern xdata  Byte     alarm_point_num;             //报警点数-->连续多少个点报警才判定为报警

/* 电机堵转标志 */
extern bdata  volatile bit      gl_motor_adjust_flag;        //电机是否处于工作状态：0-停止工作状态;1-正处于工作状态
extern xdata  Uint16   ad_chnn_wire_cut;            //0-表示钢丝没有被剪断;1-表示钢丝被剪断 -->右6~右1、左6~左1
extern xdata  Byte     gl_chnn_index;               //当前正在调整的钢丝的索引
extern xdata  Byte     gl_motor_channel_number;     //用来区分是5道电机张力还是6道电机张力-->5道电机包括：4道电机、4道电机+1道级联、5道电机   6道电机包括：6道电机、4道电机+2道级联、5道电机+1道级联
extern bdata  bit      is_motor_add_link;           //电机张力是否添加级联:0-不级联;1-级联
extern bdata  bit      is_sample_clear;             //采样值是否清零:0-没有清零；1-已经清零
extern xdata  Uint16   check_sample_clear_tick;     //用来检测采样值是否清零成功计时tick

//2016-02-28新增
extern xdata sAlarmDetailInfo  AlarmDetailInfo;     //保存最后一次报警详细信息

/* 函数声明 */
extern void check_still_stress(Byte index);

//2016-02-28新增
//保存报警详细信息
extern void save_alarm_detail_info(void);
    
void uart_task_init(void)
{
	Byte i;

    check_sample_clear_tick = 0;
    
	//uart2相关变量初始化
	recv2_state = FSA_INIT;
	recv2_timer = 0;
	recv2_ctr = 0;
	recv2_chksum = 0;
	trans2_size = 0;
	trans2_ctr = 0;

	for (i = 0; i < UART_QUEUE_NUM; i++){
		uart2_send_queue[i].flag = 0; //均空闲
		uart2_recv_queue[i].flag = 0; //均空闲
	}
	uart2_q_index = 0xFF;    //无队列项进入发送流程

	//uart3相关变量初始化
	recv3_state = FSA_INIT;
	recv3_timer = 0;
	recv3_ctr = 0;
	recv3_chksum = 0;
	trans3_size = 0;
	trans3_ctr = 0;

	for (i = 0; i < UART_QUEUE_NUM; i++){
		uart3_send_queue[i].flag = 0; //均空闲
		uart3_recv_queue[i].flag = 0; //均空闲
	}
	uart3_q_index = 0xFF;    //无队列项进入发送流程

	//uart4相关变量初始化
	recv4_state = FSA_INIT;
	recv4_timer = 0;
	recv4_ctr = 0;
	recv4_chksum = 0;
	trans4_size = 0;
	trans4_ctr = 0;

	for (i = 0; i < UART_QUEUE_NUM; i++){
		uart4_send_queue[i].flag = 0; //均空闲
		uart4_recv_queue[i].flag = 0; //均空闲
	}
	uart4_q_index = 0xFF;    //无队列项进入发送流程

	//UART硬件初始化
	uart_init();             //之后，已经准备好串口收发，只是还未使能全局中断
}

void uart_task(void)
{
	Byte   i,j,k;
	Uint16 temp16;
	Byte   *ptr;
	
	//1.处理 UART3：来自上位机的命令包
	//找是否有等待处理的项
	for (k = 0; k < UART_QUEUE_NUM; k++)
	{
		if (uart3_recv_queue[k].flag == 1)//有等待处理的项
		{
			ptr = uart3_recv_queue[k].tdata;
			
			//是否需要转发本命令
			if ((ptr[0] == CMD_ADDR_BC) || (ptr[0] != gl_comm_addr)) {
				//广播地址或非本设备, 需要转发到 UART4
				//在UART4 发送队列中找空闲Buffer
				i = uart4_get_send_buffer();
				if (i < UART_QUEUE_NUM) {
					//找到了空闲buffer, 写入data
					uart4_send_queue[i].tdata[0] = FRAME_STX;
					memcpy(&uart4_send_queue[i].tdata[1], ptr, ptr[2] + 3);
					uart4_send_queue[i].len = ptr[2] + 5;
				} 
			}

			//是否需要执行本命令
			if ((ptr[0] == CMD_ADDR_BC) || (ptr[0] == gl_comm_addr)) {
				//广播地址或指定本设备, 需要执行
				switch (ptr[3])
				{
				case CMD_DADDR_qSTAT://询问防区状态 - 报告给上位机
					//在UART3发送队列中找空闲Buffer
					i = uart3_get_send_buffer();
					if (i < UART_QUEUE_NUM) {
						//找到了空闲buffer, 准备应答
                        uart3_send_queue[i].package_type = 1;              //设备自身的数据包
                        
						uart3_send_queue[i].tdata[0] = FRAME_STX;	       //帧头
						uart3_send_queue[i].tdata[1] = ptr[1];	           //目的地址
						uart3_send_queue[i].tdata[2] = gl_comm_addr;       //源地址
						if (gl_comm_addr == CMD_ADDR_UNSOLV) {             //本设备无有效地址
							//只回参数应答
							uart3_send_queue[i].tdata[3] = 1;
							uart3_send_queue[i].tdata[4] = CMD_DADDR_aPARA;
							uart3_send_queue[i].len = 6;
						} else { //有有效地址,回防区状态
							if ((alarm_out_flag & 0xF8) == 0x00) { //2个防区均无报警
								uart3_send_queue[i].tdata[3] = 1;
								uart3_send_queue[i].tdata[4] = CMD_ACK_OK;
								uart3_send_queue[i].len = 6;
							} else { //有报警
								uart3_send_queue[i].tdata[3] = 2;
								uart3_send_queue[i].tdata[4] = CMD_DADDR_aSTAT;
								uart3_send_queue[i].tdata[5] = (alarm_out_flag & 0xF8) >> 3;
								uart3_send_queue[i].len = 7;
                                
                                //保存报警详细信息
                                save_alarm_detail_info();
							}
						}
					}
					
					break;

				case CMD_DADDR_qPARA://询问地址- 报告给上位机
					//在UART3发送队列中找空闲Buffer
					i = uart3_get_send_buffer();
					if (i < UART_QUEUE_NUM) {
						//找到了空闲buffer, 准备应答
                        uart3_send_queue[i].package_type = 1;                      //设备自身的数据包
                        
						uart3_send_queue[i].tdata[0] = FRAME_STX;	               //帧头
						uart3_send_queue[i].tdata[1] = ptr[1];	                   //目的地址
						uart3_send_queue[i].tdata[2] = gl_comm_addr;	           //源地址
						uart3_send_queue[i].tdata[3] = 1;                          //命令长度
						uart3_send_queue[i].tdata[4] = CMD_DADDR_aPARA;            //命令ID
						uart3_send_queue[i].len = 6;
					}
					
					break;
                    
                case 0xE2://修改地址
                    gl_comm_addr = ptr[4];
                
                    flash_enable();
                    flash_erase(EEPROM_SECTOR6);
                    flash_write(ptr[4], EEPROM_SECTOR6 + 1);
                    flash_write(0x5a, EEPROM_SECTOR6);
                    flash_disable();
                
                	//在UART3发送队列中找空闲Buffer
					i = uart3_get_send_buffer();
					if (i < UART_QUEUE_NUM) {
						//找到了空闲buffer, 准备应答
                        uart3_send_queue[i].package_type = 1;                      //设备自身的数据包
                        
						uart3_send_queue[i].tdata[0] = FRAME_STX;	               //帧头
						uart3_send_queue[i].tdata[1] = ptr[1];	                   //目的地址
						uart3_send_queue[i].tdata[2] = gl_comm_addr;	           //源地址
						uart3_send_queue[i].tdata[3] = 1;                          //命令长度
						uart3_send_queue[i].tdata[4] = CMD_DADDR_aPARA;            //命令ID
						uart3_send_queue[i].len = 6;
					}
                    
                    break;
                
                case 0xE3://设置延时时间
                    //1. 写入flash
                    flash_enable();                              
                    flash_erase(EEPROM_SECTOR10);                                        
                    flash_write(ptr[4], EEPROM_SECTOR10 + 1);  
                    flash_write(0x5a, EEPROM_SECTOR10);                                                              
                    flash_disable();

                    //2. 更新变量
                    gl_reply_tick = ptr[4];

                    break;

                case 0xE4://读取延时时间
                    //在UART3队列中找空闲Buffer
                    i = uart3_get_send_buffer();
                    if (i < UART_QUEUE_NUM) { //找到了空闲buffer, 准备应答
                        uart3_send_queue[i].package_type = 1;                          //设备自身的数据包
                        
                        uart3_send_queue[i].tdata[0] = FRAME_STX;	                   //帧头
                        uart3_send_queue[i].tdata[1] = ptr[1];	                       //目的地址
                        uart3_send_queue[i].tdata[2] = gl_comm_addr;	               //源地址																 
                        uart3_send_queue[i].tdata[3] = 2;
                        uart3_send_queue[i].tdata[4] = 0xF4;
                        uart3_send_queue[i].tdata[5] = gl_reply_tick;
                        uart3_send_queue[i].len = 7;														
                    }
                    
                    break;	
                         
				case CMD_ZL_PRE://张力/脉冲专用命令标志
					switch (ptr[5])
					{
					case 0x10: //读配置参数
						//在UART3队列中找空闲Buffer
						i = uart3_get_send_buffer();
						if (i < UART_QUEUE_NUM) {//找到了空闲buffer, 写入data
                            uart3_send_queue[i].package_type = 1;                     //设备自身的数据包
                            
							uart3_send_queue[i].tdata[0]  = FRAME_STX;
							uart3_send_queue[i].tdata[1]  = ptr[1];	                  //目的地址
							uart3_send_queue[i].tdata[2]  = gl_comm_addr;	          //源地址
							uart3_send_queue[i].tdata[3]  = 0x1E;                     //命令长度
							uart3_send_queue[i].tdata[4]  = CMD_ZL_PRE;		          //命令ID
							uart3_send_queue[i].tdata[5]  = 0x1C;                     //参数1
							uart3_send_queue[i].tdata[6]  = 0x08;                     //参数2
							uart3_send_queue[i].tdata[7]  = HIGH(ad_sensor_mask_LR);  //张力掩码左
							uart3_send_queue[i].tdata[8]  = LOW(ad_sensor_mask_LR);   //张力掩码右
							uart3_send_queue[i].tdata[9]  = HIGH(ad_still_dn);        //静态张力允许下限高
							uart3_send_queue[i].tdata[10] = LOW(ad_still_dn);         //静态张力允许下限低
							uart3_send_queue[i].tdata[11] = HIGH(ad_still_up);        //静态张力允许上限高
							uart3_send_queue[i].tdata[12] = LOW(ad_still_up);         //静态张力允许上限低
							uart3_send_queue[i].tdata[13] = system_status;            //报警阀值下浮比例，固定为66
							for (j = 0; j < 6; j++) {                                 //报警阀值左1~6
								uart3_send_queue[i].tdata[14 + j] = ad_still_Dup[j];
							}

          					uart3_send_queue[i].tdata[20] = 0;                        //左开关量
							uart3_send_queue[i].tdata[21] = ad_still_Dup[12];         //杆自身
                            
							for (j = 0; j < 6; j++) {                                 //报警阀值右1~6
								uart3_send_queue[i].tdata[22 + j] = ad_still_Dup[6 + j];
							}
                            uart3_send_queue[i].tdata[28] = 0;                        //右开关量
							uart3_send_queue[i].tdata[29] = ad_still_Dup[12];         //杆自身
                            
							uart3_send_queue[i].tdata[30] = 0;                        //双/单防区
							uart3_send_queue[i].tdata[31] = gl_comm_addr;             //拨码地址
							uart3_send_queue[i].tdata[32] = (Byte)((beep_during_temp * SCHEDULER_TICK) / 1000);	//声光报警输出时间
							uart3_send_queue[i].tdata[33] = 0;
							uart3_send_queue[i].len = 35;
						}
						
						break;

					case 0x12: //读报警信息
						//在UART3队列中找空闲Buffer
						i = uart3_get_send_buffer();
						if (i < UART_QUEUE_NUM) {
							//找到了空闲buffer, 写入data
                            uart3_send_queue[i].package_type = 1;                    //设备自身的数据包
                            
							uart3_send_queue[i].tdata[0]  = FRAME_STX;
							uart3_send_queue[i].tdata[1]  = ptr[1];	                 //目的地址
							uart3_send_queue[i].tdata[2]  = gl_comm_addr;	         //源地址
							uart3_send_queue[i].tdata[3]  = 0x0A;                    //命令长度
							uart3_send_queue[i].tdata[4]  = CMD_ZL_PRE;              //命令ID
							uart3_send_queue[i].tdata[5]  = 0x08;                    //参数1
							uart3_send_queue[i].tdata[6]  = 0x1A;                    //参数1
							uart3_send_queue[i].tdata[7]  = HIGH(ad_sensor_mask_LR); //张力掩码左
							uart3_send_queue[i].tdata[8]  = LOW(ad_sensor_mask_LR);  //张力掩码右

							// 杆，左开关量，左6~1报警标志
							uart3_send_queue[i].tdata[9]  = HIGH(ad_alarm_exts);     //外力报警左

							// 杆，右开关量，右6~1报警标志
							uart3_send_queue[i].tdata[10] = LOW(ad_alarm_exts);      //外力报警右

							// 杆，左开关量，左6~1报警标志
							uart3_send_queue[i].tdata[11] = HIGH(ad_alarm_base);     //静态张力报警左

							// 杆，右开关量，右6~1报警标志
							uart3_send_queue[i].tdata[12] = LOW(ad_alarm_base);      //静态张力报警右
							
							//门磁
							uart3_send_queue[i].tdata[13] = (Byte)(gl_local_dk_status | !gl_control_dk_status);   
							uart3_send_queue[i].len = 15;
						}
						
						break;

					case 0x14: //读瞬间张力
						//在UART3队列中找空闲Buffer
						i = uart3_get_send_buffer();
						if (i < UART_QUEUE_NUM) {
							//找到了空闲buffer, 写入data
                            uart3_send_queue[i].package_type = 1;               //设备自身的数据包
                            
							uart3_send_queue[i].tdata[0] = FRAME_STX;
							uart3_send_queue[i].tdata[1] = ptr[1];	            //目的地址
							uart3_send_queue[i].tdata[2] = gl_comm_addr;	    //源地址
							uart3_send_queue[i].tdata[3] = 0x23;
							uart3_send_queue[i].tdata[4] = CMD_ZL_PRE;
							uart3_send_queue[i].tdata[5] = 0x21;
							uart3_send_queue[i].tdata[6] = 0x1C;
							for (j = 0; j < 6; j++) { //左1~6
								temp16 = ad_chn_sample[j];
								uart3_send_queue[i].tdata[7 + (j << 1)] = HIGH(temp16);
								uart3_send_queue[i].tdata[8 + (j << 1)] = LOW(temp16);
							}
                            //左开关量
                            uart3_send_queue[i].tdata[19] = 0;
							uart3_send_queue[i].tdata[20] = 0;
                            
                            //杆自身
                            temp16 = ad_chn_sample[12];
							uart3_send_queue[i].tdata[21] = HIGH(temp16);
							uart3_send_queue[i].tdata[22] = LOW(temp16);
                            
							for (j = 0; j < 6; j++) { //右1~6
								temp16 = ad_chn_sample[6 + j];
								uart3_send_queue[i].tdata[23 + (j << 1)] = HIGH(temp16);
								uart3_send_queue[i].tdata[24 + (j << 1)] = LOW(temp16);
							}
                            //右开关量
							uart3_send_queue[i].tdata[35] = 0;
							uart3_send_queue[i].tdata[36] = 0;
                            
                            //杆自身
                            temp16 = ad_chn_sample[12];
                            uart3_send_queue[i].tdata[37] = HIGH(temp16);
							uart3_send_queue[i].tdata[38] = LOW(temp16);
                            
							uart3_send_queue[i].len = 40;
						}
                        
						break;

					case 0x15: //读静态张力基准
						//在UART3队列中找空闲Buffer
						i = uart3_get_send_buffer();
						if (i < UART_QUEUE_NUM) {
							//找到了空闲buffer, 写入data
                            uart3_send_queue[i].package_type = 1;                //设备自身的数据包
                            
							uart3_send_queue[i].tdata[0] = FRAME_STX;
							uart3_send_queue[i].tdata[1] = ptr[1];	            //目的地址
							uart3_send_queue[i].tdata[2] = gl_comm_addr;	    //源地址
							uart3_send_queue[i].tdata[3] = 0x23;
							uart3_send_queue[i].tdata[4] = CMD_ZL_PRE;
							uart3_send_queue[i].tdata[5] = 0x21;
							uart3_send_queue[i].tdata[6] = 0x1D;
							for (j = 0; j < 6; j++) { //左1~6
								temp16 = ad_chn_base[j].base;
								uart3_send_queue[i].tdata[7 + (j << 1)] = HIGH(temp16);
								uart3_send_queue[i].tdata[8 + (j << 1)] = LOW(temp16);
							}
                            //左开关量
                            uart3_send_queue[i].tdata[19] = 0;
							uart3_send_queue[i].tdata[20] = 0;
                            
                            //杆自身
                            temp16 = ad_chn_base[12].base;
							uart3_send_queue[i].tdata[21] = HIGH(temp16);
							uart3_send_queue[i].tdata[22] = LOW(temp16);
                            
							for (j = 0; j < 6; j++) { //右1~6
								temp16 = ad_chn_base[6 + j].base;
								uart3_send_queue[i].tdata[23 + (j << 1)] = HIGH(temp16);
								uart3_send_queue[i].tdata[24 + (j << 1)] = LOW(temp16);
							}
                            //右开关量
                            uart3_send_queue[i].tdata[35] = 0;
							uart3_send_queue[i].tdata[36] = 0;
                            
                            //杆自身
                            temp16 = ad_chn_base[12].base;
							uart3_send_queue[i].tdata[37] = HIGH(temp16);
							uart3_send_queue[i].tdata[38] = LOW(temp16);
                            
							uart3_send_queue[i].len = 40;
						}
                        
						break;

					case 0x40: //设置静态张力值范围
						//1. 写入flash
						flash_enable();
						flash_erase(EEPROM_SECTOR3);
						flash_write(ptr[6], EEPROM_SECTOR3 + 1);
						flash_write(ptr[7], EEPROM_SECTOR3 + 2);
						flash_write(ptr[8], EEPROM_SECTOR3 + 3);
						flash_write(ptr[9], EEPROM_SECTOR3 + 4);
						flash_write(0x5a, EEPROM_SECTOR3);
						flash_disable();

						//2. 更新变量
						ad_still_dn = ((Uint16)ptr[6] << 8) + ptr[7];	 //下限
						ad_still_up = ((Uint16)ptr[8] << 8) + ptr[9];	 //上限

						//3. 检查当前静态张力值
						if (system_status >= SYS_SELF_CHECK1) {
							//已开始运行检测
							for (i = 0; i < 12; i++) {
								check_still_stress(i);
							}
						}
						break;

					case 0x50: //设置报警阀值 (仅上限)
						//1. 写入flash并更新变量
						flash_enable();
						flash_erase(EEPROM_SECTOR4);
						flash_write(ptr[6], EEPROM_SECTOR4 + 1); 	 //下限浮动值，比例，目前没有被读取使用

						for (j = 0; j < 6; j++) {
							//左1 ~6
							ad_still_Dup[j] = ptr[7 + j];
							flash_write(ptr[7 + j], EEPROM_SECTOR4 + 2 + j);
						}

						for (j = 0; j < 6; j++) {
							//右1 ~6
							ad_still_Dup[6 + j] = ptr[15 + j];
							flash_write(ptr[15 + j], EEPROM_SECTOR4 + 8 + j);
						}

                        //杆自身
                        ad_still_Dup[12] = ptr[22];
						flash_write(ptr[22], EEPROM_SECTOR4 + 15);
                        
						flash_write(0x5a, EEPROM_SECTOR4);
						flash_disable();

						//下限固定取基准值的 1/3
						//2. 更新换算后的张力报警上限(采样值)
						if (system_status >= SYS_SELF_CHECK1) {
							//已开始运行检测
							for (i = 0; i < 13; i++) {
								if ((1023 - ad_chn_base[i].base) > ad_still_Dup[i])
									ad_chn_base[i].base_up = ad_chn_base[i].base + ad_still_Dup[i];
								else
									ad_chn_base[i].base_up = 1023;
							}
						}
						break;

					case 0x60: //设置声光报警时间
						//1. 写入flash
						flash_enable();
						flash_erase(EEPROM_SECTOR5);
						flash_write(ptr[6], EEPROM_SECTOR5 + 1);
						flash_write(0x5a, EEPROM_SECTOR5);
						flash_disable();

						//2. 更新变量
						beep_during_temp = (Uint16)(((Uint32)ptr[6] * 1000) / SCHEDULER_TICK);
						break;

					case 0xF0: //张力杆电机控制(主机->设备)
						//在UART2队列中找空闲Buffer
						i = uart2_get_send_buffer();
						if (i < UART_QUEUE_NUM) {
							//找到了空闲buffer, 写入data
							uart2_send_queue[i].tdata[0] = FRAME_STX;
							uart2_send_queue[i].tdata[1] = 0xFF;        //目的地址
							uart2_send_queue[i].tdata[2] = 0x01;	    //源地址
							uart2_send_queue[i].tdata[3] = 0x06;
							uart2_send_queue[i].tdata[4] = CMD_ZL_PRE;
							uart2_send_queue[i].tdata[5] = 0x04;
							uart2_send_queue[i].tdata[6] = 0xF0;
							uart2_send_queue[i].tdata[7] = ptr[6];      //电机号
							uart2_send_queue[i].tdata[8] = ptr[7];      //正转、反转、停止
							uart2_send_queue[i].tdata[9] = ptr[8];      //运转时间
							
							uart2_send_queue[i].len = 11;
                            
                            gl_chnn_index = matrix_index[ptr[6]];
						}
						
						break;

					case 0xF1: //设置传感器采样偏差---->消除电路上的误差
						//在UART2队列中找空闲Buffer
						i = uart2_get_send_buffer();
						if (i < UART_QUEUE_NUM) {
							//找到了空闲buffer, 写入data
							uart2_send_queue[i].tdata[0] = FRAME_STX;
							uart2_send_queue[i].tdata[1] = 0xFF;        //目的地址
							uart2_send_queue[i].tdata[2] = 0x01;	    //源地址
							uart2_send_queue[i].tdata[3] = 0x23;
							uart2_send_queue[i].tdata[4] = CMD_ZL_PRE;
							uart2_send_queue[i].tdata[5] = 0x21;
							uart2_send_queue[i].tdata[6] = 0xF1;
							
							//左1~8、右1~8
							for (j = 0; j < 32; j++) {
								uart2_send_queue[i].tdata[7 + j] = ptr[6 + j];      
							} 

							uart2_send_queue[i].len = 40;
						}
                        
////////////////////////////////////////////////////////////////////////////////////    
                        //只有报警主机发出的采样值清零命令才会去检测采样值清零是否成功                                                    
                        if (ptr[0] == gl_comm_addr) {
                            temp16 = 0;
                            for (i = 0; i < 32; i++) {
                                temp16 += ptr[6 + i];
                            }
                        
                            if (temp16 == 0) {//采样值恢复的情况
                                is_sample_clear = 0;
                                
                                flash_enable();
                                flash_erase(EEPROM_SECTOR9);
                                flash_write(0, EEPROM_SECTOR9 + 1);
                                flash_write(0x5a, EEPROM_SECTOR9);
                                flash_disable();
                            } else {//采样值清零的情况
                                check_sample_clear_tick = 3000 / SCHEDULER_TICK;
                            }
                        } else if (ptr[0] == CMD_ADDR_BC) {
                            //生产车间发出的采样值清零命令不会去检测采样值清零是否成功
                            //什么都不做
                        }
						
						break;

////////////////////////////////////////////////////////////////////////////////////
                        
					case 0xF2: //设置采样点数---->连续采样多少个点以确定钢丝是否比较松
						break;

					case 0xF3: //读钢丝是否剪断以及是否处于调整钢丝模式
						//在UART3队列中找空闲Buffer
						i = uart3_get_send_buffer();
						if (i < UART_QUEUE_NUM) {
							//找到了空闲buffer, 写入data
                            uart3_send_queue[i].package_type = 1;               //设备自身的数据包
                            
							uart3_send_queue[i].tdata[0] = FRAME_STX;
							uart3_send_queue[i].tdata[1] = ptr[1];	            //目的地址
							uart3_send_queue[i].tdata[2] = gl_comm_addr;	    //源地址
							uart3_send_queue[i].tdata[3] = 0x06;
							uart3_send_queue[i].tdata[4] = CMD_ZL_PRE;
							uart3_send_queue[i].tdata[5] = 0x04;
							uart3_send_queue[i].tdata[6] = 0x1E;

                            uart3_send_queue[i].tdata[7] = HIGH(ad_chnn_wire_cut);
							uart3_send_queue[i].tdata[8] = LOW(ad_chnn_wire_cut);
							uart3_send_queue[i].tdata[9] = (Byte)gl_motor_adjust_flag;
							
							uart3_send_queue[i].len = 11;
						}
						break;
                     
                    case 0xF4: //设置电机张力通道数:4道电机、5道电机、6道电机
                        gl_motor_channel_number = ptr[6];
                    
						flash_enable();
						flash_erase(EEPROM_SECTOR7);
						flash_write(ptr[6], EEPROM_SECTOR7 + 1);
						flash_write(0x5a, EEPROM_SECTOR7);
						flash_disable();
                        break;
                    
                    case 0xF5: //读电机张力通道数
                        //在UART3队列中找空闲Buffer
						i = uart3_get_send_buffer();
						if (i < UART_QUEUE_NUM) {
							//找到了空闲buffer, 写入data
                            uart3_send_queue[i].package_type = 1;               //设备自身的数据包
                            
							uart3_send_queue[i].tdata[0] = FRAME_STX;
							uart3_send_queue[i].tdata[1] = ptr[1];	            //目的地址
							uart3_send_queue[i].tdata[2] = gl_comm_addr;	    //源地址
							uart3_send_queue[i].tdata[3] = 0x04;
							uart3_send_queue[i].tdata[4] = CMD_ZL_PRE;
							uart3_send_queue[i].tdata[5] = 0x02;
							uart3_send_queue[i].tdata[6] = 0x1F;
                            uart3_send_queue[i].tdata[7] = gl_motor_channel_number;
							
							uart3_send_queue[i].len = 9;
						}
                        break;
                        
                    case 0xF6: //设置电机张力是否添加级联
                        is_motor_add_link = ptr[6];
                        
                    	flash_enable();
						flash_erase(EEPROM_SECTOR8);
						flash_write(ptr[6], EEPROM_SECTOR8 + 1);
						flash_write(0x5a, EEPROM_SECTOR8);
						flash_disable();
                        break;
                    
                    case 0xF7: //读电机张力是否添加级联
                        //在UART3队列中找空闲Buffer
						i = uart3_get_send_buffer();
						if (i < UART_QUEUE_NUM) {
							//找到了空闲buffer, 写入data
                            uart3_send_queue[i].package_type = 1;               //设备自身的数据包
                            
							uart3_send_queue[i].tdata[0] = FRAME_STX;
							uart3_send_queue[i].tdata[1] = ptr[1];	            //目的地址
							uart3_send_queue[i].tdata[2] = gl_comm_addr;	    //源地址
							uart3_send_queue[i].tdata[3] = 0x04;
							uart3_send_queue[i].tdata[4] = CMD_ZL_PRE;
							uart3_send_queue[i].tdata[5] = 0x02;
							uart3_send_queue[i].tdata[6] = 0x20;
                            uart3_send_queue[i].tdata[7] = (Byte)is_motor_add_link;
							
							uart3_send_queue[i].len = 9;
						}
                        break;
                        
                    //2016-02-28新增
                    case 0xF8://读取报警详细信息                                
                        get_alarm_detail_info();
                        break;
                    
                    case 0xF9: //设置平均值点数-->采样多少个点求平均值                        
						//在UART2队列中找空闲Buffer
						i = uart2_get_send_buffer();
						if (i < UART_QUEUE_NUM) {
							//找到了空闲buffer, 写入data
							uart2_send_queue[i].tdata[0] = FRAME_STX;
							uart2_send_queue[i].tdata[1] = 0xFF;        //目的地址
							uart2_send_queue[i].tdata[2] = 0x01;	    //源地址
							uart2_send_queue[i].tdata[3] = 0x04;
							uart2_send_queue[i].tdata[4] = CMD_ZL_PRE;
							uart2_send_queue[i].tdata[5] = 0x02;
							uart2_send_queue[i].tdata[6] = 0xF9;	
                            if (ptr[6] < 4) {
                                uart2_send_queue[i].tdata[7] = 4;	
                            } else if (ptr[6] > 8){
                                uart2_send_queue[i].tdata[7] = 8;	
                            } else {
                                uart2_send_queue[i].tdata[7] = ptr[6];	
                            }
                            
							uart2_send_queue[i].len = 9;
						}
                        
                        //返回信息
                        //在UART3队列中找空闲Buffer
						i = uart3_get_send_buffer();
						if (i < UART_QUEUE_NUM) {
							//找到了空闲buffer, 写入data
                            uart3_send_queue[i].package_type = 1;               //设备自身的数据包
                            
							uart3_send_queue[i].tdata[0] = FRAME_STX;
							uart3_send_queue[i].tdata[1] = ptr[1];	            //目的地址
							uart3_send_queue[i].tdata[2] = gl_comm_addr;	    //源地址
							uart3_send_queue[i].tdata[3] = 0x04;
							uart3_send_queue[i].tdata[4] = CMD_ZL_PRE;
							uart3_send_queue[i].tdata[5] = 0x02;
							uart3_send_queue[i].tdata[6] = 0x21;
                            if (ptr[6] < 4) {
                                uart3_send_queue[i].tdata[7] = 4;	
                            } else if (ptr[6] > 8){
                                uart3_send_queue[i].tdata[7] = 8;	
                            } else {
                                uart3_send_queue[i].tdata[7] = ptr[6];	
                            }
							
							uart3_send_queue[i].len = 9;
						}
                        
                        break;
                        
                    case 0xFA: //设置报警点数-->连续多少个点报警才判定为报警
                        //更新变量
                        if (ptr[6] < 4) {
                            alarm_point_num = 4;

                        } else if (ptr[6] > 8){
                            alarm_point_num = 8;
                        } else {
                            alarm_point_num = ptr[6];
                        }
                        
                        //写入flash
                        flash_enable();
						flash_erase(EEPROM_SECTOR11);
						flash_write(alarm_point_num, EEPROM_SECTOR11 + 1);
						flash_write(0x5a, EEPROM_SECTOR11);
						flash_disable();
                        
                        //返回信息
                        //在UART3队列中找空闲Buffer
						i = uart3_get_send_buffer();
						if (i < UART_QUEUE_NUM) {
							//找到了空闲buffer, 写入data
                            uart3_send_queue[i].package_type = 1;               //设备自身的数据包
                            
							uart3_send_queue[i].tdata[0] = FRAME_STX;
							uart3_send_queue[i].tdata[1] = ptr[1];	            //目的地址
							uart3_send_queue[i].tdata[2] = gl_comm_addr;	    //源地址
							uart3_send_queue[i].tdata[3] = 0x04;
							uart3_send_queue[i].tdata[4] = CMD_ZL_PRE;
							uart3_send_queue[i].tdata[5] = 0x02;
							uart3_send_queue[i].tdata[6] = 0x22;
                            uart3_send_queue[i].tdata[7] = alarm_point_num;
							
							uart3_send_queue[i].len = 9;
						}
                        break;
					}
					break;
				}
                
                //设置应答延时
                Disable_interrupt();
                gl_ack_tick = REPLY_DLY + (gl_comm_addr - 16) * gl_reply_tick / SCHEDULER_TICK;
                Enable_interrupt();
			}
			
			//处理完成,释放该队列项
			uart3_recv_queue[k].flag = 0;
			
			break;
		}
	}

	//2.处理UART4：来自下位机的命令包
	//找是否有等待处理的项
	for (k = 0; k < UART_QUEUE_NUM; k++)
	{
		if (uart4_recv_queue[k].flag == 1)//有等待处理的项
		{
			ptr = uart4_recv_queue[k].tdata;
			
			//转发此命令,在UART3队列中找空闲Buffer
			i = uart3_get_send_buffer();
			if (i < UART_QUEUE_NUM) {
				//找到了空闲buffer, 写入data
                uart3_send_queue[i].package_type = 0;         //来自下位机的数据包
                
				uart3_send_queue[i].tdata[0] = FRAME_STX;
				memcpy(&uart3_send_queue[i].tdata[1], ptr, ptr[2] + 3);
				uart3_send_queue[i].len = ptr[2] + 5;
			}
			
			//处理完成,释放该队列项
			uart4_recv_queue[k].flag = 0;

			break;
		}
	}
	
	//3. UART3 队列发送
	if ((uart3_q_index == 0xFF) && (recv3_state == FSA_INIT)) {
		//UART3无进入发送流程的队列项, 找是否有等待发送的项
		for (i = 0; i < UART_QUEUE_NUM; i++) {
			if ((uart3_send_queue[i].flag == 1) && (uart3_send_queue[i].package_type == 0)){
                //来自下位机的数据包:有等待发送的项，安排此项发送
				uart3_send_queue[i].flag = 2;
				uart3_q_index = i;
				memcpy(trans3_buf, uart3_send_queue[i].tdata, uart3_send_queue[i].len - 1);
				trans3_size = uart3_send_queue[i].len;
				uart3_start_trans();
				break;
			}else if((uart3_send_queue[i].flag == 1) && (uart3_send_queue[i].package_type == 1) && (gl_ack_tick == 0)){
                //设备自身的数据包:有等待发送的项，安排此项发送
				uart3_send_queue[i].flag = 2;
				uart3_q_index = i;
				memcpy(trans3_buf, uart3_send_queue[i].tdata, uart3_send_queue[i].len - 1);
				trans3_size = uart3_send_queue[i].len;
				uart3_start_trans();
				break;
            }
		}
	}

	//4. UART4 队列发送
	if ((uart4_q_index == 0xFF) && (recv4_state == FSA_INIT)) {
		//UART4无进入发送流程的队列项, 找是否有等待发送的项
		for (i = 0; i < UART_QUEUE_NUM; i++) {
			if (uart4_send_queue[i].flag == 1) {
				//有等待发送的项，安排此项发送
				uart4_send_queue[i].flag = 2;
				uart4_q_index = i;
				memcpy(trans4_buf, uart4_send_queue[i].tdata, uart4_send_queue[i].len - 1);
				trans4_size = uart4_send_queue[i].len;
				uart4_start_trans();
				break;
			}
		}
	}
	
	//5. UART2 队列发送
	if (uart2_q_index == 0xFF) {
		//UART2无进入发送流程的队列项, 找是否有等待发送的项
		for (i = 0; i < UART_QUEUE_NUM; i++) {
			if (uart2_send_queue[i].flag == 1) {
				//有等待发送的项，安排此项发送
				uart2_send_queue[i].flag = 2;
				uart2_q_index = i;
				memcpy(trans2_buf, uart2_send_queue[i].tdata, uart2_send_queue[i].len - 1);
				trans2_size = uart2_send_queue[i].len;
				uart2_start_trans();
				break;
			}
		}
	}
}

/***************************************************************************
* NAME: uart2_get_send_buffer
*----------------------------------------------------------------------------
* PARAMS:
* return: Byte
*         若返回值 >= UART_QUEUE_NUM, 则表示没有申请到空闲buffer
*----------------------------------------------------------------------------
* PURPOSE: 在串口2队列中寻找空闲队列项，若找到，返回队列项序号(0 ~ (UART_QUEUE_NUM-1))
*****************************************************************************/

Byte uart2_get_send_buffer(void)
{
	Byte i, flag;

	for (i = 0; i < UART_QUEUE_NUM; i++)
	{
		Disable_interrupt();
		flag = uart2_send_queue[i].flag;
		Enable_interrupt();
		if (flag == 0)   //已找到空闲Buffer
		{
			uart2_send_queue[i].flag = 1;
			break;
		}
	}
	return i;
}


Byte uart2_get_recv_buffer(void)
{
	Byte i, flag;

	for (i = 0; i < UART_QUEUE_NUM; i++)
	{
		Disable_interrupt();
		flag = uart2_recv_queue[i].flag;
		Enable_interrupt();
		if (flag == 0)   //已找到空闲Buffer
		{
			uart2_recv_queue[i].flag = 1;
			break;
		}
	}
	return i;
}

/***************************************************************************
* NAME: uart3_get_send_buffer
*----------------------------------------------------------------------------
* PARAMS:
* return: Byte
*         若返回值 >= UART_QUEUE_NUM, 则表示没有申请到空闲buffer
*----------------------------------------------------------------------------
* PURPOSE: 在串口2队列中寻找空闲队列项，若找到，返回队列项序号(0 ~ (UART_QUEUE_NUM-1))
*****************************************************************************/
Byte uart3_get_send_buffer(void)
{
	Byte i, flag;

	for (i = 0; i < UART_QUEUE_NUM; i++)
	{
		Disable_interrupt();
		flag = uart3_send_queue[i].flag;
		Enable_interrupt();
		if (flag == 0)   //已找到空闲Buffer
		{
			uart3_send_queue[i].flag = 1;
			break;
		}
	}
	return i;
}

Byte uart3_get_recv_buffer(void)
{
	Byte i, flag;

	for (i = 0; i < UART_QUEUE_NUM; i++)
	{
		Disable_interrupt();
		flag = uart3_recv_queue[i].flag;
		Enable_interrupt();
		if (flag == 0)   //已找到空闲Buffer
		{
			uart3_recv_queue[i].flag = 1;
			break;
		}
	}
	return i;
}

/***************************************************************************
* NAME: uart4_get_send_buffer
*----------------------------------------------------------------------------
* PARAMS:
* return: Byte
*         若返回值 >= UART_QUEUE_NUM, 则表示没有申请到空闲buffer
*----------------------------------------------------------------------------
* PURPOSE: 在串口2队列中寻找空闲队列项，若找到，返回队列项序号(0 ~ (UART_QUEUE_NUM-1))
*****************************************************************************/
Byte uart4_get_send_buffer(void)
{
	Byte i, flag;

	for (i = 0; i < UART_QUEUE_NUM; i++)
	{
		Disable_interrupt();
		flag = uart4_send_queue[i].flag;
		Enable_interrupt();
		if (flag == 0)   //已找到空闲Buffer
		{
			uart4_send_queue[i].flag = 1;
			break;
		}
	}
	return i;
}

Byte uart4_get_recv_buffer(void)
{
	Byte i, flag;

	for (i = 0; i < UART_QUEUE_NUM; i++)
	{
		Disable_interrupt();
		flag = uart4_recv_queue[i].flag;
		Enable_interrupt();
		if (flag == 0)   //已找到空闲Buffer
		{
			uart4_recv_queue[i].flag = 1;
			break;
		}
	}
	return i;
}

//2016-02-28新增
void get_alarm_detail_info(void)
{
    Byte i,j;
    Uint16 temp16;
    
    //1、返回配置信息   
    //在UART3队列中找空闲Buffer
    i = uart3_get_send_buffer();
    if (i < UART_QUEUE_NUM) {//找到了空闲buffer, 写入data
        uart3_send_queue[i].package_type = 1;                     //设备自身的数据包
        
        uart3_send_queue[i].tdata[0]  = FRAME_STX;
        uart3_send_queue[i].tdata[1]  = 0x01;	                  //目的地址
        uart3_send_queue[i].tdata[2]  = gl_comm_addr;	          //源地址
        uart3_send_queue[i].tdata[3]  = 0x1E;                     //命令长度
        uart3_send_queue[i].tdata[4]  = CMD_ZL_PRE;		          //命令ID
        uart3_send_queue[i].tdata[5]  = 0x1C;                     //参数1
        uart3_send_queue[i].tdata[6]  = 0x08;                     //参数2
        uart3_send_queue[i].tdata[7]  = HIGH(ad_sensor_mask_LR);  //张力掩码左
        uart3_send_queue[i].tdata[8]  = LOW(ad_sensor_mask_LR);   //张力掩码右
        uart3_send_queue[i].tdata[9]  = HIGH(ad_still_dn);        //静态张力允许下限高
        uart3_send_queue[i].tdata[10] = LOW(ad_still_dn);         //静态张力允许下限低
        uart3_send_queue[i].tdata[11] = HIGH(ad_still_up);        //静态张力允许上限高
        uart3_send_queue[i].tdata[12] = LOW(ad_still_up);         //静态张力允许上限低
        uart3_send_queue[i].tdata[13] = system_status;            //报警阀值下浮比例
        for (j = 0; j < 6; j++) {                                 //报警阀值左1~6
            uart3_send_queue[i].tdata[14 + j] = ad_still_Dup[j];
        }

        uart3_send_queue[i].tdata[20] = 0;                        //左开关量
        uart3_send_queue[i].tdata[21] = ad_still_Dup[12];         //杆自身
        
        for (j = 0; j < 6; j++) {                                 //报警阀值右1~6
            uart3_send_queue[i].tdata[22 + j] = ad_still_Dup[6 + j];
        }
        uart3_send_queue[i].tdata[28] = 0;                        //右开关量
        uart3_send_queue[i].tdata[29] = ad_still_Dup[12];         //杆自身
        
        uart3_send_queue[i].tdata[30] = 0;                        //双/单防区
        uart3_send_queue[i].tdata[31] = gl_comm_addr;             //拨码地址
        uart3_send_queue[i].tdata[32] = (Byte)((beep_during_temp * SCHEDULER_TICK) / 1000);	//声光报警输出时间
        uart3_send_queue[i].tdata[33] = 0;
        uart3_send_queue[i].len = 35;
    }
          
    //2、读取报警信息
    //在UART3队列中找空闲Buffer
    i = uart3_get_send_buffer();
    if (i < UART_QUEUE_NUM) {
        //找到了空闲buffer, 写入data
        uart3_send_queue[i].package_type = 1;                    //设备自身的数据包
        
        uart3_send_queue[i].tdata[0]  = FRAME_STX;
        uart3_send_queue[i].tdata[1]  = 0x01;	                 //目的地址
        uart3_send_queue[i].tdata[2]  = gl_comm_addr;	         //源地址
        uart3_send_queue[i].tdata[3]  = 0x0A;                    //命令长度
        uart3_send_queue[i].tdata[4]  = CMD_ZL_PRE;              //命令ID
        uart3_send_queue[i].tdata[5]  = 0x08;                    //参数1
        uart3_send_queue[i].tdata[6]  = 0x1A;                    //参数1
        uart3_send_queue[i].tdata[7]  = HIGH(ad_sensor_mask_LR); //张力掩码左
        uart3_send_queue[i].tdata[8]  = LOW(ad_sensor_mask_LR);  //张力掩码右

        // 杆，左开关量，左6~1报警标志
        uart3_send_queue[i].tdata[9]  = HIGH(AlarmDetailInfo.ExternalAlarm);     //外力报警左

        // 杆，右开关量，右6~1报警标志
        uart3_send_queue[i].tdata[10] = LOW(AlarmDetailInfo.ExternalAlarm);      //外力报警右

        // 杆，左开关量，左6~1报警标志
        uart3_send_queue[i].tdata[11] = HIGH(AlarmDetailInfo.StaticAlarm);     //静态张力报警左

        // 杆，右开关量，右6~1报警标志
        uart3_send_queue[i].tdata[12] = LOW(AlarmDetailInfo.StaticAlarm);      //静态张力报警右
        
        //门磁
        uart3_send_queue[i].tdata[13] = (Byte)(gl_local_dk_status | !gl_control_dk_status);   
        uart3_send_queue[i].len = 15;
    }
    
    //3、读瞬态张力
    //在UART3队列中找空闲Buffer
    i = uart3_get_send_buffer();
    if (i < UART_QUEUE_NUM) {
        //找到了空闲buffer, 写入data
        uart3_send_queue[i].package_type = 1;               //设备自身的数据包
        
        uart3_send_queue[i].tdata[0] = FRAME_STX;
        uart3_send_queue[i].tdata[1] = 0x01;	            //目的地址
        uart3_send_queue[i].tdata[2] = gl_comm_addr;	    //源地址
        uart3_send_queue[i].tdata[3] = 0x23;
        uart3_send_queue[i].tdata[4] = CMD_ZL_PRE;
        uart3_send_queue[i].tdata[5] = 0x21;
        uart3_send_queue[i].tdata[6] = 0x1C;
        for (j = 0; j < 6; j++) { //左1~6
            temp16 = AlarmDetailInfo.InstantSampleValue[j];
            uart3_send_queue[i].tdata[7 + (j << 1)] = HIGH(temp16);
            uart3_send_queue[i].tdata[8 + (j << 1)] = LOW(temp16);
        }
        //左开关量
        uart3_send_queue[i].tdata[19] = 0;
        uart3_send_queue[i].tdata[20] = 0;
        
        //杆自身
        temp16 = AlarmDetailInfo.InstantSampleValue[12];
        uart3_send_queue[i].tdata[21] = HIGH(temp16);
        uart3_send_queue[i].tdata[22] = LOW(temp16);
        
        for (j = 0; j < 6; j++) { //右1~6
            temp16 = AlarmDetailInfo.InstantSampleValue[6+j];
            uart3_send_queue[i].tdata[23 + (j << 1)] = HIGH(temp16);
            uart3_send_queue[i].tdata[24 + (j << 1)] = LOW(temp16);
        }
        //右开关量
        uart3_send_queue[i].tdata[35] = 0;
        uart3_send_queue[i].tdata[36] = 0;
        
        //杆自身
        temp16 = AlarmDetailInfo.InstantSampleValue[12];
        uart3_send_queue[i].tdata[37] = HIGH(temp16);
        uart3_send_queue[i].tdata[38] = LOW(temp16);
        
        uart3_send_queue[i].len = 40;
    }
    
    //4、读静态张力基准
    //在UART3队列中找空闲Buffer
    i = uart3_get_send_buffer();
    if (i < UART_QUEUE_NUM) {
        //找到了空闲buffer, 写入data
        uart3_send_queue[i].package_type = 1;                //设备自身的数据包
        
        uart3_send_queue[i].tdata[0] = FRAME_STX;
        uart3_send_queue[i].tdata[1] = 0x01;	            //目的地址
        uart3_send_queue[i].tdata[2] = gl_comm_addr;	    //源地址
        uart3_send_queue[i].tdata[3] = 0x23;
        uart3_send_queue[i].tdata[4] = CMD_ZL_PRE;
        uart3_send_queue[i].tdata[5] = 0x21;
        uart3_send_queue[i].tdata[6] = 0x1D;
        for (j = 0; j < 6; j++) { //左1~6
            temp16 = AlarmDetailInfo.StaticBaseValue[j];
            uart3_send_queue[i].tdata[7 + (j << 1)] = HIGH(temp16);
            uart3_send_queue[i].tdata[8 + (j << 1)] = LOW(temp16);
        }
        //左开关量
        uart3_send_queue[i].tdata[19] = 0;
        uart3_send_queue[i].tdata[20] = 0;
        
        //杆自身
        temp16 = AlarmDetailInfo.StaticBaseValue[12];
        uart3_send_queue[i].tdata[21] = HIGH(temp16);
        uart3_send_queue[i].tdata[22] = LOW(temp16);
        
        for (j = 0; j < 6; j++) { //右1~6
            temp16 = AlarmDetailInfo.StaticBaseValue[6+j];
            uart3_send_queue[i].tdata[23 + (j << 1)] = HIGH(temp16);
            uart3_send_queue[i].tdata[24 + (j << 1)] = LOW(temp16);
        }
        //右开关量
        uart3_send_queue[i].tdata[35] = 0;
        uart3_send_queue[i].tdata[36] = 0;
        
        //杆自身
        temp16 = AlarmDetailInfo.StaticBaseValue[12];
        uart3_send_queue[i].tdata[37] = HIGH(temp16);
        uart3_send_queue[i].tdata[38] = LOW(temp16);
        
        uart3_send_queue[i].len = 40;
    }
}