#include "driver/uart/uart_drv.h"

#define FOSC   22118400UL                                     //晶振频率，SysClk对晶振频率不分频
#define BAUD   9600                                          //波特率
#define TM     (65536 - (FOSC/12/4/BAUD))

#define S2TI   0x02
#define S2RI   0x01
#define S3TI   0x02
#define S3RI   0x01
#define S4TI   0x02
#define S4RI   0x01

/* UART2 */
extern xdata  Byte     recv2_buf[MAX_RecvFrame];       // receiving buffer
extern xdata  Byte     recv2_state;                    // receive state
extern xdata  Byte     recv2_timer;                    // receive time-out, 用于字节间超时判定
extern xdata  Byte     recv2_chksum;                   // computed checksum
extern xdata  Byte     recv2_ctr;                      // reveiving pointer
extern xdata  Byte     trans2_buf[MAX_TransFrame];     // uart transfer message buffer
extern xdata  Byte     trans2_ctr;                     // transfer pointer
extern xdata  Byte     trans2_size;                    // transfer bytes number
extern xdata  Byte     trans2_chksum;                  // computed check-sum of already transfered message

extern xdata  Byte     uart2_q_index;                  // 正在发送某队列项的序号：若为0xFF, 表示没有任何项进入发送流程
extern xdata  sUART_Q  uart2_send_queue[UART_QUEUE_NUM];     // 串口发送队列
extern xdata  sUART_Q  uart2_recv_queue[UART_QUEUE_NUM];     // 串口接收队列

/* UART3 */
#define UART3_RECEIVE_ENABLE()    (S3CON |= 0x10)
#define UART3_RECEIVE_DISABLE()   (S3CON &= 0xEF)

extern xdata  Byte     recv3_buf[MAX_RecvFrame];       // receiving buffer
extern xdata  Byte     recv3_state;                    // receive state
extern xdata  Byte     recv3_timer;                    // receive time-out, 用于字节间超时判定
extern xdata  Byte     recv3_chksum;                   // computed checksum
extern xdata  Byte     recv3_ctr;                      // reveiving pointer

extern xdata  Byte     trans3_buf[MAX_TransFrame];     // uart transfer message buffer
extern xdata  Byte     trans3_ctr;                     // transfer pointer
extern xdata  Byte     trans3_size;                    // transfer bytes number
extern xdata  Byte     trans3_chksum;                  // computed check-sum of already transfered message
 
extern xdata  Byte     uart3_q_index;                  // 正在发送某队列项的序号：若为0xFF, 表示没有任何项进入发送流程
extern xdata  sUART_Q  uart3_send_queue[UART_QUEUE_NUM];     // 串口发送队列
extern xdata  sUART_Q  uart3_recv_queue[UART_QUEUE_NUM];     // 串口接收队列

/* UART4 */
#define UART4_RECEIVE_ENABLE()    (S4CON |= 0x10)
#define UART4_RECEIVE_DISABLE()   (S4CON &= 0xEF)

extern xdata  Byte     recv4_buf[MAX_RecvFrame];       // receiving buffer
extern xdata  Byte     recv4_state;                    // receive state
extern xdata  Byte     recv4_timer;                    // receive time-out, 用于字节间超时判定
extern xdata  Byte     recv4_chksum;                   // computed checksum
extern xdata  Byte     recv4_ctr;                      // reveiving pointer

extern xdata  Byte     trans4_buf[MAX_TransFrame];     // uart transfer message buffer
extern xdata  Byte     trans4_ctr;                     // transfer pointer
extern xdata  Byte     trans4_size;                    // transfer bytes number
extern xdata  Byte     trans4_chksum;                  // computed check-sum of already transfered message

extern xdata  Byte     uart4_q_index;                  // 正在发送某队列项的序号：若为0xFF, 表示没有任何项进入发送流程
extern xdata  sUART_Q  uart4_send_queue[UART_QUEUE_NUM];     // 串口发送队列
extern xdata  sUART_Q  uart4_recv_queue[UART_QUEUE_NUM];     // 串口接收队列

/* 函数声明 */
extern Byte uart2_get_recv_buffer(void);
extern Byte uart3_get_recv_buffer(void);
extern Byte uart4_get_recv_buffer(void);

void uart_init(void)    //9600bps@22.1184MHz
{
/*
    //uart1硬件初始化	
    SCON = 0x50;		//8位数据,可变波特率,接收使能
    PCON &= 0x3F;       //波特率不加倍
    AUXR &= 0xBF;		//定时器1时钟为Fosc/12,即12T
	AUXR &= 0xFE;		//串口1选择定时器1为波特率发生器
    TMOD &= 0x0F;       //定时器1为模式0:16位自动重装
    TL1 = LOW(TM);      //设定定时初值
	TH1 = HIGH(TM);	    //设定定时初值
    ET1 = 0;            //关闭定时器T1中断
    ES = 1;             //使能串口1中断
    PS = 1;             //设置中断优先级为优先级1
    P_SW1 &= 0x3F;      //设置串口1在P30，P31
    CLK_DIV &= 0xEF;    //设置串口1为正常工作模式
    TR1 = 1;            //启动定时器1
*/  
	//uart2硬件初始化	
	S2CON = 0x50;		//8位数据,可变波特率,接收使能
	AUXR &= 0xE3;		//定时器2模式为12T
	T2L = LOW(TM);      //设定定时初值
	T2H = HIGH(TM);	    //设定定时初值
	IE2 |= 0x01;        //使能串口2中断
    IE2 &= 0xFB;        //关闭定时器T2中断
	IP2 |= 0x01;        //设置中断优先级为优先级1
	P_SW2 |= 0x01;      //设置串口2在P46,P47
	AUXR |= 0x10;		//启动定时器2

	//uart3硬件初始化
	S3CON = 0x50;		//8位数据,可变波特率,接收使能,选择定时器3为波特率发生器
    T4T3M &= 0xF1;      //定时器3模式为12T
    T3L = LOW(TM);      //设定定时初值
	T3H = HIGH(TM);	    //设定定时初值
	IE2 |= 0x08;        //使能串口3中断
    IE2 &= 0xDF;        //关闭定时器T3中断
	P_SW2 |= 0x02;      //设置串口3在P50,P51
	T4T3M |= 0x08;      //启动定时器3

	//uart4硬件初始化
	S4CON = 0x50;		//8位数据,可变波特率,接收使能,,选择定时器4为波特率发生器
    T4T3M &= 0x9F;      //定时器4模式为12T
    T4L = LOW(TM);      //设定定时初值
	T4H = HIGH(TM);	    //设定定时初值
	IE2 |= 0x10;        //使能串口4中断
    IE2 &= 0xBF;        //关闭定时器T4中断
	P_SW2 |= 0x04;      //设置串口4在P52,P53
    T4T3M |= 0x80;      //启动定时器4
}

/*
//与控制杆通信
void uart1_isr(void) interrupt UART1_VECTOR
{
	Byte c,i;
    
    if (TI) { //发送中断
		trans2_ctr++;   //取下一个待传送index
		if (trans2_ctr < trans2_size) { //未传送完成
			if (trans2_ctr == (trans2_size - 1)) { //已经指向校验字节
				SBUF = trans2_chksum;    //发送校验字节
			} else { //非校验字节, 需要传送并计算checksum
				SBUF = trans2_buf[trans2_ctr];
				if (trans2_ctr > 0) { //计算check_sum
					trans2_chksum += trans2_buf[trans2_ctr];   //更新chksum
				}
			}
		} else { //已经全部传送完成(含校验字节)，可以置发送器空闲
			//目前设计：均不需等待应答, 可以释放该队列项
			if (uart2_q_index < UART_QUEUE_NUM) {
				uart2_send_queue[uart2_q_index].flag = 0;   //该队列项空闲
			}
				
			uart2_q_index = 0xFF;	//无队列项在发送
		}
		
		TI = 0;   //must clear by user software
	}

	if (RI) { //接收中断
		c = SBUF;
		switch (recv2_state)
		{
		case FSA_INIT://是否为帧头
			if (c == FRAME_STX) { //为帧头, 开始新的一帧
				recv2_ctr = 0;
				recv2_chksum = 0;
				recv2_timer = RECV_TIMEOUT;
				recv2_state = FSA_ADDR_D;
			}
			break;

		case FSA_ADDR_D://为目的地址, 开始保存并计算效验和
			recv2_buf[recv2_ctr++] = c;
			recv2_chksum += c;
			recv2_timer = RECV_TIMEOUT;
			recv2_state = FSA_ADDR_S;
			break;

		case FSA_ADDR_S://为源地址
			recv2_buf[recv2_ctr++] = c;
			recv2_chksum += c;
			recv2_timer = RECV_TIMEOUT;
			recv2_state = FSA_LENGTH;
			break;

		case FSA_LENGTH://为长度字节
			if ((c > 0) && (c < (MAX_RecvFrame - 3))) { //有效串
				recv2_buf[recv2_ctr++] = c;    //第三个字节保存长度
				recv2_chksum += c;
				recv2_timer = RECV_TIMEOUT;
				recv2_state = FSA_DATA;
			} else {	//非有效串
				recv2_state = FSA_INIT;
			}
			break;

		case FSA_DATA://读取命令串
			recv2_buf[recv2_ctr] = c;
			recv2_chksum += c;   //更新校验和
			if (recv2_ctr == (recv2_buf[2] + 2)){ //已经收到指定长度的命令数据
				recv2_state = FSA_CHKSUM;
			}else{//还未结束
				recv2_ctr ++;
			}
			recv2_timer = RECV_TIMEOUT;
			break;

		case FSA_CHKSUM://检查校验字节			
			if (recv2_chksum == c){//已经收到完整一帧
				i = uart2_get_recv_buffer();
				if (i < UART_QUEUE_NUM) {//找到了空闲buffer, 写入data
					memcpy(uart2_recv_queue[i].tdata, recv2_buf, recv2_buf[2] + 3);
				}
			}
			
		default:
			//复位
			recv2_state = FSA_INIT;
			break;
		}
		
		RI = 0;     //must clear by user software
	}
}
*/


//与控制杆通信
void uart2_isr(void) interrupt UART2_VECTOR
{
	Byte c,i;
    
	if (S2CON & S2TI) { //发送中断
		trans2_ctr++;   //取下一个待传送index
		if (trans2_ctr < trans2_size) { //未传送完成
			if (trans2_ctr == (trans2_size - 1)) { //已经指向校验字节
				S2BUF = trans2_chksum;    //发送校验字节
			} else { //非校验字节, 需要传送并计算checksum
				S2BUF = trans2_buf[trans2_ctr];
				if (trans2_ctr > 0) { //计算check_sum
					trans2_chksum += trans2_buf[trans2_ctr];   //更新chksum
				}
			}
		} else { //已经全部传送完成(含校验字节)，可以置发送器空闲
			//目前设计：均不需等待应答, 可以释放该队列项
			if (uart2_q_index < UART_QUEUE_NUM) {
				uart2_send_queue[uart2_q_index].flag = 0;   //该队列项空闲
			}
			
            uart2_q_index = 0xFF;	   //无队列项在发送
		}
		
		S2CON &= ~S2TI;   //must clear by user software
	}

	if (S2CON & S2RI) { //接收中断
		c = S2BUF;
		switch (recv2_state)
		{
		case FSA_INIT://是否为帧头
			if (c == FRAME_STX) { //为帧头, 开始新的一帧
				recv2_ctr = 0;
				recv2_chksum = 0;
				recv2_timer = RECV_TIMEOUT;
				recv2_state = FSA_ADDR_D;
			}
			break;

		case FSA_ADDR_D://为目的地址, 开始保存并计算效验和
			recv2_buf[recv2_ctr++] = c;
			recv2_chksum += c;
			recv2_timer = RECV_TIMEOUT;
			recv2_state = FSA_ADDR_S;
			break;

		case FSA_ADDR_S://为源地址
			recv2_buf[recv2_ctr++] = c;
			recv2_chksum += c;
			recv2_timer = RECV_TIMEOUT;
			recv2_state = FSA_LENGTH;
			break;

		case FSA_LENGTH://为长度字节
			if ((c > 0) && (c < (MAX_RecvFrame - 3))) { //有效串
				recv2_buf[recv2_ctr++] = c;    //第三个字节保存长度
				recv2_chksum += c;
				recv2_timer = RECV_TIMEOUT;
				recv2_state = FSA_DATA;
			} else {	//非有效串
				recv2_state = FSA_INIT;
			}
			break;

		case FSA_DATA://读取命令串
			recv2_buf[recv2_ctr] = c;
			recv2_chksum += c;   //更新校验和
			if (recv2_ctr == (recv2_buf[2] + 2)){ //已经收到指定长度的命令数据
				recv2_state = FSA_CHKSUM;
			}else{//还未结束
				recv2_ctr ++;
			}
			recv2_timer = RECV_TIMEOUT;
			break;

		case FSA_CHKSUM://检查校验字节
			if (recv2_chksum == c){//已经收到完整一帧
                i = uart2_get_recv_buffer();
				if (i < UART_QUEUE_NUM) {//找到了空闲buffer, 写入data
                    memcpy(uart2_recv_queue[i].tdata, recv2_buf, recv2_buf[2] + 3);
                }
			}
			
		default:
			//复位
			recv2_state = FSA_INIT;
			break;
		}
		
		S2CON &= ~S2RI;//must clear by user software
	}
}

//上位机
void uart3_isr(void) interrupt UART3_VECTOR
{
	volatile Byte nop_num;
	Byte c,i;
    
	if (S3CON & S3TI) { //发送中断
		trans3_ctr++;   //取下一个待传送index
		if (trans3_ctr < trans3_size) { //未传送完成
			if (trans3_ctr == (trans3_size - 1)) { //已经指向校验字节
				S3BUF = trans3_chksum;    //发送校验字节
			} else { //非校验字节, 需要传送并计算checksum
				S3BUF = trans3_buf[trans3_ctr];
				if (trans3_ctr > 0) { //计算check_sum
					trans3_chksum += trans3_buf[trans3_ctr];   //更新chksum
				}
			}
		} else { //已经全部传送完成(含校验字节)，可以置发送器空闲
			//目前设计：均不需等待应答, 可以释放该队列项
			if (uart3_q_index < UART_QUEUE_NUM) {
				uart3_send_queue[uart3_q_index].flag = 0;   //该队列项空闲
			}
				
			uart3_q_index = 0xFF;	//无队列项在发送
			nop_num = 100;
			while (nop_num--);
			bRS485_1_Ctrl = 0;	        //禁止发送, 转为接收
			UART3_RECEIVE_ENABLE();  //UART1允许接收
		}
		
		S3CON &= ~S3TI;   //must clear by user software
	}

	if (S3CON & S3RI) { //接收中断
		c = S3BUF;
		switch (recv3_state)
		{
		case FSA_INIT://是否为帧头
			if (c == FRAME_STX) { //为帧头, 开始新的一帧
				recv3_ctr = 0;
				recv3_chksum = 0;
				recv3_timer = RECV_TIMEOUT;
				recv3_state = FSA_ADDR_D;
			}
			break;

		case FSA_ADDR_D://为目的地址, 开始保存并计算效验和
			recv3_buf[recv3_ctr++] = c;
			recv3_chksum += c;
			recv3_timer = RECV_TIMEOUT;
			recv3_state = FSA_ADDR_S;
			break;

		case FSA_ADDR_S://为源地址
			recv3_buf[recv3_ctr++] = c;
			recv3_chksum += c;
			recv3_timer = RECV_TIMEOUT;
			recv3_state = FSA_LENGTH;
			break;

		case FSA_LENGTH://为长度字节
			if ((c > 0) && (c < (MAX_RecvFrame - 3))) { //有效串
				recv3_buf[recv3_ctr++] = c;    //第三个字节保存长度
				recv3_chksum += c;
				recv3_timer = RECV_TIMEOUT;
				recv3_state = FSA_DATA;
			} else {	//非有效串
				recv3_state = FSA_INIT;
			}
			break;

		case FSA_DATA://读取命令串
			recv3_buf[recv3_ctr] = c;
			recv3_chksum += c;   //更新校验和
			if (recv3_ctr == (recv3_buf[2] + 2)){ //已经收到指定长度的命令数据
				recv3_state = FSA_CHKSUM;
			}else{//还未结束
				recv3_ctr ++;
			}
			recv3_timer = RECV_TIMEOUT;
			break;

		case FSA_CHKSUM://检查校验字节			
			if (recv3_chksum == c){//已经收到完整一帧
				i = uart3_get_recv_buffer();
				if (i < UART_QUEUE_NUM) {//找到了空闲buffer, 写入data
					memcpy(uart3_recv_queue[i].tdata, recv3_buf, recv3_buf[2] + 3);
				}
			}
			
		default:
			//复位
			recv3_state = FSA_INIT;
			break;
		}
		
		S3CON &= ~S3RI;     //must clear by user software
	}
}

//下位机
void uart4_isr(void) interrupt UART4_VECTOR
{
	volatile Byte nop_num;
	Byte c,i;
    
	if (S4CON & S4TI) { //UART2发送中断
		trans4_ctr++;   //取下一个待传送index
		if (trans4_ctr < trans4_size) { //未传送完成
			if (trans4_ctr == (trans4_size - 1)) { //已经指向校验字节
				S4BUF = trans4_chksum;    //发送校验字节
			} else { //非校验字节, 需要传送并计算checksum
				S4BUF = trans4_buf[trans4_ctr];
				if (trans4_ctr > 0) { //计算check_sum
					trans4_chksum += trans4_buf[trans4_ctr];   //更新chksum
				}
			}
		} else { //已经全部传送完成(含校验字节)，可以置发送器空闲
			//目前设计：均不需等待应答, 可以释放该队列项
			if (uart4_q_index < UART_QUEUE_NUM) {
				uart4_send_queue[uart4_q_index].flag = 0;   //该队列项空闲
			}
				
			uart4_q_index = 0xFF;	   //无队列项在发送
			nop_num = 100;
			while (nop_num--);
			bRS485_2_Ctrl = 0;	         //禁止发送, 转为接收
			UART4_RECEIVE_ENABLE();  //UART2允许接收
		}
		
		S4CON &= ~S4TI;   //must clear by user software
	}

	if (S4CON & S4RI) { //接收中断
		c = S4BUF;
		switch (recv4_state)
		{
		case FSA_INIT://是否为帧头
			if (c == FRAME_STX) { //为帧头, 开始新的一帧
				recv4_ctr = 0;
				recv4_chksum = 0;
				recv4_timer = RECV_TIMEOUT;
				recv4_state = FSA_ADDR_D;
			}
			break;

		case FSA_ADDR_D://为目的地址, 开始保存并计算效验和
			recv4_buf[recv4_ctr++] = c;
			recv4_chksum += c;
			recv4_timer = RECV_TIMEOUT;
			recv4_state = FSA_ADDR_S;
			break;

		case FSA_ADDR_S://为源地址
			recv4_buf[recv4_ctr++] = c;
			recv4_chksum += c;
			recv4_timer = RECV_TIMEOUT;
			recv4_state = FSA_LENGTH;
			break;

		case FSA_LENGTH://为长度字节
			if ((c > 0) && (c < (MAX_RecvFrame - 3))) { //有效串
				recv4_buf[recv4_ctr++] = c;    //第三个字节保存长度
				recv4_chksum += c;
				recv4_timer = RECV_TIMEOUT;
				recv4_state = FSA_DATA;
			} else {	//非有效串
				recv4_state = FSA_INIT;
			}
			break;

		case FSA_DATA://读取命令串
			recv4_buf[recv4_ctr] = c;
			recv4_chksum += c;   //更新校验和
			if (recv4_ctr == (recv4_buf[2] + 2)) { //已经收到指定长度的命令数据
				recv4_state = FSA_CHKSUM;
			} else {	//还未结束
				recv4_ctr++;
			}
			recv4_timer = RECV_TIMEOUT;
			break;

		case FSA_CHKSUM://检查校验字节			
			if (recv4_chksum == c){//已经收到完整一帧
				i = uart4_get_recv_buffer();
				if (i < UART_QUEUE_NUM) {//找到了空闲buffer, 写入data
					memcpy(uart4_recv_queue[i].tdata, recv4_buf, recv4_buf[2] + 3);
				}
			}
			
		default://复位
			recv4_state = FSA_INIT;
			break;
		}
		
		S4CON &= ~S4RI;     //must clear by user software
	}
}

void uart2_start_trans(void)
{ 
	trans2_chksum = 0;
	trans2_ctr = 0;
	S2BUF = trans2_buf[trans2_ctr];
}

void uart3_start_trans(void)
{ 
	volatile Byte nop_num = 100;

	UART3_RECEIVE_DISABLE();
	_nop_();
	_nop_();
	bRS485_1_Ctrl = 1;    //允许发送
	while (nop_num--);
	trans3_chksum = 0;
	trans3_ctr = 0;
	S3BUF = trans3_buf[trans3_ctr];
}

void uart4_start_trans(void)
{ 
	volatile Byte nop_num = 100;

	UART4_RECEIVE_DISABLE();
	_nop_();
	_nop_();
	bRS485_2_Ctrl = 1;    //允许发送
	while (nop_num--);
	trans4_chksum = 0;
	trans4_ctr = 0;
	S4BUF = trans4_buf[trans4_ctr];
}