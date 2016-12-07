#include "driver/uart/uart_drv.h"

#define FOSC   22118400UL                                     //����Ƶ�ʣ�SysClk�Ծ���Ƶ�ʲ���Ƶ
#define BAUD   9600                                          //������
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
extern xdata  Byte     recv2_timer;                    // receive time-out, �����ֽڼ䳬ʱ�ж�
extern xdata  Byte     recv2_chksum;                   // computed checksum
extern xdata  Byte     recv2_ctr;                      // reveiving pointer
extern xdata  Byte     trans2_buf[MAX_TransFrame];     // uart transfer message buffer
extern xdata  Byte     trans2_ctr;                     // transfer pointer
extern xdata  Byte     trans2_size;                    // transfer bytes number
extern xdata  Byte     trans2_chksum;                  // computed check-sum of already transfered message

extern xdata  Byte     uart2_q_index;                  // ���ڷ���ĳ���������ţ���Ϊ0xFF, ��ʾû���κ�����뷢������
extern xdata  sUART_Q  uart2_send_queue[UART_QUEUE_NUM];     // ���ڷ��Ͷ���
extern xdata  sUART_Q  uart2_recv_queue[UART_QUEUE_NUM];     // ���ڽ��ն���

/* UART3 */
#define UART3_RECEIVE_ENABLE()    (S3CON |= 0x10)
#define UART3_RECEIVE_DISABLE()   (S3CON &= 0xEF)

extern xdata  Byte     recv3_buf[MAX_RecvFrame];       // receiving buffer
extern xdata  Byte     recv3_state;                    // receive state
extern xdata  Byte     recv3_timer;                    // receive time-out, �����ֽڼ䳬ʱ�ж�
extern xdata  Byte     recv3_chksum;                   // computed checksum
extern xdata  Byte     recv3_ctr;                      // reveiving pointer

extern xdata  Byte     trans3_buf[MAX_TransFrame];     // uart transfer message buffer
extern xdata  Byte     trans3_ctr;                     // transfer pointer
extern xdata  Byte     trans3_size;                    // transfer bytes number
extern xdata  Byte     trans3_chksum;                  // computed check-sum of already transfered message
 
extern xdata  Byte     uart3_q_index;                  // ���ڷ���ĳ���������ţ���Ϊ0xFF, ��ʾû���κ�����뷢������
extern xdata  sUART_Q  uart3_send_queue[UART_QUEUE_NUM];     // ���ڷ��Ͷ���
extern xdata  sUART_Q  uart3_recv_queue[UART_QUEUE_NUM];     // ���ڽ��ն���

/* UART4 */
#define UART4_RECEIVE_ENABLE()    (S4CON |= 0x10)
#define UART4_RECEIVE_DISABLE()   (S4CON &= 0xEF)

extern xdata  Byte     recv4_buf[MAX_RecvFrame];       // receiving buffer
extern xdata  Byte     recv4_state;                    // receive state
extern xdata  Byte     recv4_timer;                    // receive time-out, �����ֽڼ䳬ʱ�ж�
extern xdata  Byte     recv4_chksum;                   // computed checksum
extern xdata  Byte     recv4_ctr;                      // reveiving pointer

extern xdata  Byte     trans4_buf[MAX_TransFrame];     // uart transfer message buffer
extern xdata  Byte     trans4_ctr;                     // transfer pointer
extern xdata  Byte     trans4_size;                    // transfer bytes number
extern xdata  Byte     trans4_chksum;                  // computed check-sum of already transfered message

extern xdata  Byte     uart4_q_index;                  // ���ڷ���ĳ���������ţ���Ϊ0xFF, ��ʾû���κ�����뷢������
extern xdata  sUART_Q  uart4_send_queue[UART_QUEUE_NUM];     // ���ڷ��Ͷ���
extern xdata  sUART_Q  uart4_recv_queue[UART_QUEUE_NUM];     // ���ڽ��ն���

/* �������� */
extern Byte uart2_get_recv_buffer(void);
extern Byte uart3_get_recv_buffer(void);
extern Byte uart4_get_recv_buffer(void);

void uart_init(void)    //9600bps@22.1184MHz
{
/*
    //uart1Ӳ����ʼ��	
    SCON = 0x50;		//8λ����,�ɱ䲨����,����ʹ��
    PCON &= 0x3F;       //�����ʲ��ӱ�
    AUXR &= 0xBF;		//��ʱ��1ʱ��ΪFosc/12,��12T
	AUXR &= 0xFE;		//����1ѡ��ʱ��1Ϊ�����ʷ�����
    TMOD &= 0x0F;       //��ʱ��1Ϊģʽ0:16λ�Զ���װ
    TL1 = LOW(TM);      //�趨��ʱ��ֵ
	TH1 = HIGH(TM);	    //�趨��ʱ��ֵ
    ET1 = 0;            //�رն�ʱ��T1�ж�
    ES = 1;             //ʹ�ܴ���1�ж�
    PS = 1;             //�����ж����ȼ�Ϊ���ȼ�1
    P_SW1 &= 0x3F;      //���ô���1��P30��P31
    CLK_DIV &= 0xEF;    //���ô���1Ϊ��������ģʽ
    TR1 = 1;            //������ʱ��1
*/  
	//uart2Ӳ����ʼ��	
	S2CON = 0x50;		//8λ����,�ɱ䲨����,����ʹ��
	AUXR &= 0xE3;		//��ʱ��2ģʽΪ12T
	T2L = LOW(TM);      //�趨��ʱ��ֵ
	T2H = HIGH(TM);	    //�趨��ʱ��ֵ
	IE2 |= 0x01;        //ʹ�ܴ���2�ж�
    IE2 &= 0xFB;        //�رն�ʱ��T2�ж�
	IP2 |= 0x01;        //�����ж����ȼ�Ϊ���ȼ�1
	P_SW2 |= 0x01;      //���ô���2��P46,P47
	AUXR |= 0x10;		//������ʱ��2

	//uart3Ӳ����ʼ��
	S3CON = 0x50;		//8λ����,�ɱ䲨����,����ʹ��,ѡ��ʱ��3Ϊ�����ʷ�����
    T4T3M &= 0xF1;      //��ʱ��3ģʽΪ12T
    T3L = LOW(TM);      //�趨��ʱ��ֵ
	T3H = HIGH(TM);	    //�趨��ʱ��ֵ
	IE2 |= 0x08;        //ʹ�ܴ���3�ж�
    IE2 &= 0xDF;        //�رն�ʱ��T3�ж�
	P_SW2 |= 0x02;      //���ô���3��P50,P51
	T4T3M |= 0x08;      //������ʱ��3

	//uart4Ӳ����ʼ��
	S4CON = 0x50;		//8λ����,�ɱ䲨����,����ʹ��,,ѡ��ʱ��4Ϊ�����ʷ�����
    T4T3M &= 0x9F;      //��ʱ��4ģʽΪ12T
    T4L = LOW(TM);      //�趨��ʱ��ֵ
	T4H = HIGH(TM);	    //�趨��ʱ��ֵ
	IE2 |= 0x10;        //ʹ�ܴ���4�ж�
    IE2 &= 0xBF;        //�رն�ʱ��T4�ж�
	P_SW2 |= 0x04;      //���ô���4��P52,P53
    T4T3M |= 0x80;      //������ʱ��4
}

/*
//����Ƹ�ͨ��
void uart1_isr(void) interrupt UART1_VECTOR
{
	Byte c,i;
    
    if (TI) { //�����ж�
		trans2_ctr++;   //ȡ��һ��������index
		if (trans2_ctr < trans2_size) { //δ�������
			if (trans2_ctr == (trans2_size - 1)) { //�Ѿ�ָ��У���ֽ�
				SBUF = trans2_chksum;    //����У���ֽ�
			} else { //��У���ֽ�, ��Ҫ���Ͳ�����checksum
				SBUF = trans2_buf[trans2_ctr];
				if (trans2_ctr > 0) { //����check_sum
					trans2_chksum += trans2_buf[trans2_ctr];   //����chksum
				}
			}
		} else { //�Ѿ�ȫ���������(��У���ֽ�)�������÷���������
			//Ŀǰ��ƣ�������ȴ�Ӧ��, �����ͷŸö�����
			if (uart2_q_index < UART_QUEUE_NUM) {
				uart2_send_queue[uart2_q_index].flag = 0;   //�ö��������
			}
				
			uart2_q_index = 0xFF;	//�޶������ڷ���
		}
		
		TI = 0;   //must clear by user software
	}

	if (RI) { //�����ж�
		c = SBUF;
		switch (recv2_state)
		{
		case FSA_INIT://�Ƿ�Ϊ֡ͷ
			if (c == FRAME_STX) { //Ϊ֡ͷ, ��ʼ�µ�һ֡
				recv2_ctr = 0;
				recv2_chksum = 0;
				recv2_timer = RECV_TIMEOUT;
				recv2_state = FSA_ADDR_D;
			}
			break;

		case FSA_ADDR_D://ΪĿ�ĵ�ַ, ��ʼ���沢����Ч���
			recv2_buf[recv2_ctr++] = c;
			recv2_chksum += c;
			recv2_timer = RECV_TIMEOUT;
			recv2_state = FSA_ADDR_S;
			break;

		case FSA_ADDR_S://ΪԴ��ַ
			recv2_buf[recv2_ctr++] = c;
			recv2_chksum += c;
			recv2_timer = RECV_TIMEOUT;
			recv2_state = FSA_LENGTH;
			break;

		case FSA_LENGTH://Ϊ�����ֽ�
			if ((c > 0) && (c < (MAX_RecvFrame - 3))) { //��Ч��
				recv2_buf[recv2_ctr++] = c;    //�������ֽڱ��泤��
				recv2_chksum += c;
				recv2_timer = RECV_TIMEOUT;
				recv2_state = FSA_DATA;
			} else {	//����Ч��
				recv2_state = FSA_INIT;
			}
			break;

		case FSA_DATA://��ȡ���
			recv2_buf[recv2_ctr] = c;
			recv2_chksum += c;   //����У���
			if (recv2_ctr == (recv2_buf[2] + 2)){ //�Ѿ��յ�ָ�����ȵ���������
				recv2_state = FSA_CHKSUM;
			}else{//��δ����
				recv2_ctr ++;
			}
			recv2_timer = RECV_TIMEOUT;
			break;

		case FSA_CHKSUM://���У���ֽ�			
			if (recv2_chksum == c){//�Ѿ��յ�����һ֡
				i = uart2_get_recv_buffer();
				if (i < UART_QUEUE_NUM) {//�ҵ��˿���buffer, д��data
					memcpy(uart2_recv_queue[i].tdata, recv2_buf, recv2_buf[2] + 3);
				}
			}
			
		default:
			//��λ
			recv2_state = FSA_INIT;
			break;
		}
		
		RI = 0;     //must clear by user software
	}
}
*/


//����Ƹ�ͨ��
void uart2_isr(void) interrupt UART2_VECTOR
{
	Byte c,i;
    
	if (S2CON & S2TI) { //�����ж�
		trans2_ctr++;   //ȡ��һ��������index
		if (trans2_ctr < trans2_size) { //δ�������
			if (trans2_ctr == (trans2_size - 1)) { //�Ѿ�ָ��У���ֽ�
				S2BUF = trans2_chksum;    //����У���ֽ�
			} else { //��У���ֽ�, ��Ҫ���Ͳ�����checksum
				S2BUF = trans2_buf[trans2_ctr];
				if (trans2_ctr > 0) { //����check_sum
					trans2_chksum += trans2_buf[trans2_ctr];   //����chksum
				}
			}
		} else { //�Ѿ�ȫ���������(��У���ֽ�)�������÷���������
			//Ŀǰ��ƣ�������ȴ�Ӧ��, �����ͷŸö�����
			if (uart2_q_index < UART_QUEUE_NUM) {
				uart2_send_queue[uart2_q_index].flag = 0;   //�ö��������
			}
			
            uart2_q_index = 0xFF;	   //�޶������ڷ���
		}
		
		S2CON &= ~S2TI;   //must clear by user software
	}

	if (S2CON & S2RI) { //�����ж�
		c = S2BUF;
		switch (recv2_state)
		{
		case FSA_INIT://�Ƿ�Ϊ֡ͷ
			if (c == FRAME_STX) { //Ϊ֡ͷ, ��ʼ�µ�һ֡
				recv2_ctr = 0;
				recv2_chksum = 0;
				recv2_timer = RECV_TIMEOUT;
				recv2_state = FSA_ADDR_D;
			}
			break;

		case FSA_ADDR_D://ΪĿ�ĵ�ַ, ��ʼ���沢����Ч���
			recv2_buf[recv2_ctr++] = c;
			recv2_chksum += c;
			recv2_timer = RECV_TIMEOUT;
			recv2_state = FSA_ADDR_S;
			break;

		case FSA_ADDR_S://ΪԴ��ַ
			recv2_buf[recv2_ctr++] = c;
			recv2_chksum += c;
			recv2_timer = RECV_TIMEOUT;
			recv2_state = FSA_LENGTH;
			break;

		case FSA_LENGTH://Ϊ�����ֽ�
			if ((c > 0) && (c < (MAX_RecvFrame - 3))) { //��Ч��
				recv2_buf[recv2_ctr++] = c;    //�������ֽڱ��泤��
				recv2_chksum += c;
				recv2_timer = RECV_TIMEOUT;
				recv2_state = FSA_DATA;
			} else {	//����Ч��
				recv2_state = FSA_INIT;
			}
			break;

		case FSA_DATA://��ȡ���
			recv2_buf[recv2_ctr] = c;
			recv2_chksum += c;   //����У���
			if (recv2_ctr == (recv2_buf[2] + 2)){ //�Ѿ��յ�ָ�����ȵ���������
				recv2_state = FSA_CHKSUM;
			}else{//��δ����
				recv2_ctr ++;
			}
			recv2_timer = RECV_TIMEOUT;
			break;

		case FSA_CHKSUM://���У���ֽ�
			if (recv2_chksum == c){//�Ѿ��յ�����һ֡
                i = uart2_get_recv_buffer();
				if (i < UART_QUEUE_NUM) {//�ҵ��˿���buffer, д��data
                    memcpy(uart2_recv_queue[i].tdata, recv2_buf, recv2_buf[2] + 3);
                }
			}
			
		default:
			//��λ
			recv2_state = FSA_INIT;
			break;
		}
		
		S2CON &= ~S2RI;//must clear by user software
	}
}

//��λ��
void uart3_isr(void) interrupt UART3_VECTOR
{
	volatile Byte nop_num;
	Byte c,i;
    
	if (S3CON & S3TI) { //�����ж�
		trans3_ctr++;   //ȡ��һ��������index
		if (trans3_ctr < trans3_size) { //δ�������
			if (trans3_ctr == (trans3_size - 1)) { //�Ѿ�ָ��У���ֽ�
				S3BUF = trans3_chksum;    //����У���ֽ�
			} else { //��У���ֽ�, ��Ҫ���Ͳ�����checksum
				S3BUF = trans3_buf[trans3_ctr];
				if (trans3_ctr > 0) { //����check_sum
					trans3_chksum += trans3_buf[trans3_ctr];   //����chksum
				}
			}
		} else { //�Ѿ�ȫ���������(��У���ֽ�)�������÷���������
			//Ŀǰ��ƣ�������ȴ�Ӧ��, �����ͷŸö�����
			if (uart3_q_index < UART_QUEUE_NUM) {
				uart3_send_queue[uart3_q_index].flag = 0;   //�ö��������
			}
				
			uart3_q_index = 0xFF;	//�޶������ڷ���
			nop_num = 100;
			while (nop_num--);
			bRS485_1_Ctrl = 0;	        //��ֹ����, תΪ����
			UART3_RECEIVE_ENABLE();  //UART1�������
		}
		
		S3CON &= ~S3TI;   //must clear by user software
	}

	if (S3CON & S3RI) { //�����ж�
		c = S3BUF;
		switch (recv3_state)
		{
		case FSA_INIT://�Ƿ�Ϊ֡ͷ
			if (c == FRAME_STX) { //Ϊ֡ͷ, ��ʼ�µ�һ֡
				recv3_ctr = 0;
				recv3_chksum = 0;
				recv3_timer = RECV_TIMEOUT;
				recv3_state = FSA_ADDR_D;
			}
			break;

		case FSA_ADDR_D://ΪĿ�ĵ�ַ, ��ʼ���沢����Ч���
			recv3_buf[recv3_ctr++] = c;
			recv3_chksum += c;
			recv3_timer = RECV_TIMEOUT;
			recv3_state = FSA_ADDR_S;
			break;

		case FSA_ADDR_S://ΪԴ��ַ
			recv3_buf[recv3_ctr++] = c;
			recv3_chksum += c;
			recv3_timer = RECV_TIMEOUT;
			recv3_state = FSA_LENGTH;
			break;

		case FSA_LENGTH://Ϊ�����ֽ�
			if ((c > 0) && (c < (MAX_RecvFrame - 3))) { //��Ч��
				recv3_buf[recv3_ctr++] = c;    //�������ֽڱ��泤��
				recv3_chksum += c;
				recv3_timer = RECV_TIMEOUT;
				recv3_state = FSA_DATA;
			} else {	//����Ч��
				recv3_state = FSA_INIT;
			}
			break;

		case FSA_DATA://��ȡ���
			recv3_buf[recv3_ctr] = c;
			recv3_chksum += c;   //����У���
			if (recv3_ctr == (recv3_buf[2] + 2)){ //�Ѿ��յ�ָ�����ȵ���������
				recv3_state = FSA_CHKSUM;
			}else{//��δ����
				recv3_ctr ++;
			}
			recv3_timer = RECV_TIMEOUT;
			break;

		case FSA_CHKSUM://���У���ֽ�			
			if (recv3_chksum == c){//�Ѿ��յ�����һ֡
				i = uart3_get_recv_buffer();
				if (i < UART_QUEUE_NUM) {//�ҵ��˿���buffer, д��data
					memcpy(uart3_recv_queue[i].tdata, recv3_buf, recv3_buf[2] + 3);
				}
			}
			
		default:
			//��λ
			recv3_state = FSA_INIT;
			break;
		}
		
		S3CON &= ~S3RI;     //must clear by user software
	}
}

//��λ��
void uart4_isr(void) interrupt UART4_VECTOR
{
	volatile Byte nop_num;
	Byte c,i;
    
	if (S4CON & S4TI) { //UART2�����ж�
		trans4_ctr++;   //ȡ��һ��������index
		if (trans4_ctr < trans4_size) { //δ�������
			if (trans4_ctr == (trans4_size - 1)) { //�Ѿ�ָ��У���ֽ�
				S4BUF = trans4_chksum;    //����У���ֽ�
			} else { //��У���ֽ�, ��Ҫ���Ͳ�����checksum
				S4BUF = trans4_buf[trans4_ctr];
				if (trans4_ctr > 0) { //����check_sum
					trans4_chksum += trans4_buf[trans4_ctr];   //����chksum
				}
			}
		} else { //�Ѿ�ȫ���������(��У���ֽ�)�������÷���������
			//Ŀǰ��ƣ�������ȴ�Ӧ��, �����ͷŸö�����
			if (uart4_q_index < UART_QUEUE_NUM) {
				uart4_send_queue[uart4_q_index].flag = 0;   //�ö��������
			}
				
			uart4_q_index = 0xFF;	   //�޶������ڷ���
			nop_num = 100;
			while (nop_num--);
			bRS485_2_Ctrl = 0;	         //��ֹ����, תΪ����
			UART4_RECEIVE_ENABLE();  //UART2�������
		}
		
		S4CON &= ~S4TI;   //must clear by user software
	}

	if (S4CON & S4RI) { //�����ж�
		c = S4BUF;
		switch (recv4_state)
		{
		case FSA_INIT://�Ƿ�Ϊ֡ͷ
			if (c == FRAME_STX) { //Ϊ֡ͷ, ��ʼ�µ�һ֡
				recv4_ctr = 0;
				recv4_chksum = 0;
				recv4_timer = RECV_TIMEOUT;
				recv4_state = FSA_ADDR_D;
			}
			break;

		case FSA_ADDR_D://ΪĿ�ĵ�ַ, ��ʼ���沢����Ч���
			recv4_buf[recv4_ctr++] = c;
			recv4_chksum += c;
			recv4_timer = RECV_TIMEOUT;
			recv4_state = FSA_ADDR_S;
			break;

		case FSA_ADDR_S://ΪԴ��ַ
			recv4_buf[recv4_ctr++] = c;
			recv4_chksum += c;
			recv4_timer = RECV_TIMEOUT;
			recv4_state = FSA_LENGTH;
			break;

		case FSA_LENGTH://Ϊ�����ֽ�
			if ((c > 0) && (c < (MAX_RecvFrame - 3))) { //��Ч��
				recv4_buf[recv4_ctr++] = c;    //�������ֽڱ��泤��
				recv4_chksum += c;
				recv4_timer = RECV_TIMEOUT;
				recv4_state = FSA_DATA;
			} else {	//����Ч��
				recv4_state = FSA_INIT;
			}
			break;

		case FSA_DATA://��ȡ���
			recv4_buf[recv4_ctr] = c;
			recv4_chksum += c;   //����У���
			if (recv4_ctr == (recv4_buf[2] + 2)) { //�Ѿ��յ�ָ�����ȵ���������
				recv4_state = FSA_CHKSUM;
			} else {	//��δ����
				recv4_ctr++;
			}
			recv4_timer = RECV_TIMEOUT;
			break;

		case FSA_CHKSUM://���У���ֽ�			
			if (recv4_chksum == c){//�Ѿ��յ�����һ֡
				i = uart4_get_recv_buffer();
				if (i < UART_QUEUE_NUM) {//�ҵ��˿���buffer, д��data
					memcpy(uart4_recv_queue[i].tdata, recv4_buf, recv4_buf[2] + 3);
				}
			}
			
		default://��λ
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
	bRS485_1_Ctrl = 1;    //������
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
	bRS485_2_Ctrl = 1;    //������
	while (nop_num--);
	trans4_chksum = 0;
	trans4_ctr = 0;
	S4BUF = trans4_buf[trans4_ctr];
}