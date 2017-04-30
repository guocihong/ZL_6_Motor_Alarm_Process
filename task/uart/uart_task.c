#include "task/uart/uart_task.h"

#define REPLY_DLY      (100/SCHEDULER_TICK)         //�յ�PC������Ӧ����ʱ

/* UART2 */
extern xdata  Byte     recv2_buf[MAX_RecvFrame];    // receiving buffer
extern xdata  Byte     recv2_state;                 // receive state
extern xdata  Byte     recv2_timer;                 // receive time-out, �����ֽڼ䳬ʱ�ж�
extern xdata  Byte     recv2_chksum;                // computed checksum
extern xdata  Byte     recv2_ctr;                   // reveiving pointer

extern xdata  Byte     trans2_buf[MAX_TransFrame];  // uart transfer message buffer
extern xdata  Byte     trans2_ctr;                  // transfer pointer
extern xdata  Byte     trans2_size;                 // transfer bytes number
extern xdata  Byte     trans2_chksum;               // computed check-sum of already transfered message

extern xdata  Byte     uart2_q_index;               // ���ڷ���ĳ���������ţ���Ϊ0xFF, ��ʾû���κ�����뷢������
extern xdata  sUART_Q  uart2_send_queue[UART_QUEUE_NUM];     // ���ڷ��Ͷ���
extern xdata  sUART_Q  uart2_recv_queue[UART_QUEUE_NUM];     // ���ڽ��ն���

/* UART3 */
extern xdata  Byte     recv3_buf[MAX_RecvFrame];    // receiving buffer
extern xdata  Byte     recv3_state;                 // receive state
extern xdata  Byte     recv3_timer;                 // receive time-out, �����ֽڼ䳬ʱ�ж�
extern xdata  Byte     recv3_chksum;                // computed checksum
extern xdata  Byte     recv3_ctr;                   // reveiving pointer

extern xdata  Byte     trans3_buf[MAX_TransFrame];  // uart transfer message buffer
extern xdata  Byte     trans3_ctr;                  // transfer pointer
extern xdata  Byte     trans3_size;                 // transfer bytes number
extern xdata  Byte     trans3_chksum;               // computed check-sum of already transfered message

extern xdata  Byte     uart3_q_index;               // ���ڷ���ĳ���������ţ���Ϊ0xFF, ��ʾû���κ�����뷢������
extern xdata  sUART_Q  uart3_send_queue[UART_QUEUE_NUM];     // ���ڷ��Ͷ���
extern xdata  sUART_Q  uart3_recv_queue[UART_QUEUE_NUM];     // ���ڽ��ն���

/* UART4 */
extern xdata  Byte     recv4_buf[MAX_RecvFrame];    // receiving buffer
extern xdata  Byte     recv4_state;                 // receive state
extern xdata  Byte     recv4_timer;                 // receive time-out, �����ֽڼ䳬ʱ�ж�
extern xdata  Byte     recv4_chksum;                // computed checksum
extern xdata  Byte     recv4_ctr;                   // reveiving pointer

extern xdata  Byte     trans4_buf[MAX_TransFrame];  // uart transfer message buffer
extern xdata  Byte     trans4_ctr;                  // transfer pointer
extern xdata  Byte     trans4_size;                 // transfer bytes number
extern xdata  Byte     trans4_chksum;               // computed check-sum of already transfered message

extern xdata  Byte     uart4_q_index;               // ���ڷ���ĳ���������ţ���Ϊ0xFF, ��ʾû���κ�����뷢������
extern xdata  sUART_Q  uart4_send_queue[UART_QUEUE_NUM];     // ���ڷ��Ͷ���
extern xdata  sUART_Q  uart4_recv_queue[UART_QUEUE_NUM];     // ���ڽ��ն���

/* ϵͳ��ʱ */
extern xdata  Uint16   gl_ack_tick;	                //Ӧ����ʱ��ʱ tick
extern xdata  Byte     gl_reply_tick;               //�豸������ʱ

/* for system */
extern  data  Byte     gl_comm_addr;                //��ģ��485ͨ�ŵ�ַ
extern  data  Byte     system_status;               //ϵͳ״̬
extern xdata  Byte     matrix_index[12];

/* for alarm */
extern bdata  Byte     alarm_out_flag;              //λַ76543210  ��Ӧ  �󿪹����������� �ҿ������������� �������������� �ҷ������� ��������� X X X
													//���������־��λֵ0 - �ޱ������̵����ϵ����ϣ�; λֵ1 - ����(�ϵ�)
													//���������־��λֵ0 - ��������ϵ�,ʹ�ó��գ�;  λֵ1 - ���������(�̵����ϵ����ϣ���·)
													//ZZX: �������λֵ��ʵ��Ӳ�����ƽŵ�ƽ�෴; 	�������λֵ��ʵ��Ӳ�����ƽŵ�ƽ��ͬ

/* for beep */
extern xdata  volatile Uint16   beep_during_temp;            // Ԥ���һ�η�������ʱ��, ��λ:tick

/* Doorkeep(�Ŵ�) */
extern bdata  bit      gl_local_dk_status;          //�Ŵſ���״̬��ÿ1s��̬��⣩: 1 - �պ�; 0 - ��(��Ҫ����)
extern bdata  bit      gl_control_dk_status;        //���Ƹ˵��Ŵ�״̬: 1 - �պ�; 0 - ��(��Ҫ����)                    

/* AD sample */
extern xdata  Uint16   ad_sensor_mask_LR;           //������˳���������sensor mask: �ˡ��󿪹�������6 ~ 1,�ˡ��ҿ���������6 ~ 1
extern xdata  Uint16   ad_chn_sample[13];           //����һ�ֲ���ֵ���Ѿ���ȥ������ÿͨ��һ���㣬ѭ�����棩
extern xdata  sAD_BASE ad_chn_base[13];             //��ͨ������ʱ��̬��׼ֵ/�����޷�ֵ����λ������ֵ��
extern  data  Uint16   ad_alarm_exts;               //����������־����mask���� λֵ 0 - �ޣ� 1 - ����ֵ
extern  data  Uint16   ad_alarm_base;	            //��̬����������־����mask���� λֵ 0 - ����Χ�ڣ� 1 - ������Χ
extern xdata  Uint16   ad_still_dn;                 //��̬����ֵ����
extern xdata  Uint16   ad_still_up;                 //��̬����ֵ����
extern xdata  Byte     ad_still_Dup[13];            //������ֵ����
extern xdata  Byte     alarm_point_num;             //��������-->�������ٸ��㱨�����ж�Ϊ����

/* �����ת��־ */
extern bdata  volatile bit      gl_motor_adjust_flag;        //����Ƿ��ڹ���״̬��0-ֹͣ����״̬;1-�����ڹ���״̬
extern xdata  Uint16   ad_chnn_wire_cut;            //0-��ʾ��˿û�б�����;1-��ʾ��˿������ -->��6~��1����6~��1
extern xdata  Byte     gl_chnn_index;               //��ǰ���ڵ����ĸ�˿������
extern xdata  Byte     gl_motor_channel_number;     //����������5�������������6���������-->5�����������4�������4�����+1��������5�����   6�����������6�������4�����+2��������5�����+1������
extern bdata  bit      is_motor_add_link;           //��������Ƿ���Ӽ���:0-������;1-����
extern bdata  bit      is_sample_clear;             //����ֵ�Ƿ�����:0-û�����㣻1-�Ѿ�����
extern xdata  Uint16   check_sample_clear_tick;     //����������ֵ�Ƿ�����ɹ���ʱtick

//2016-02-28����
extern xdata sAlarmDetailInfo  AlarmDetailInfo;     //�������һ�α�����ϸ��Ϣ

/* �������� */
extern void check_still_stress(Byte index);

//2016-02-28����
//���汨����ϸ��Ϣ
extern void save_alarm_detail_info(void);
    
void uart_task_init(void)
{
	Byte i;

    check_sample_clear_tick = 0;
    
	//uart2��ر�����ʼ��
	recv2_state = FSA_INIT;
	recv2_timer = 0;
	recv2_ctr = 0;
	recv2_chksum = 0;
	trans2_size = 0;
	trans2_ctr = 0;

	for (i = 0; i < UART_QUEUE_NUM; i++){
		uart2_send_queue[i].flag = 0; //������
		uart2_recv_queue[i].flag = 0; //������
	}
	uart2_q_index = 0xFF;    //�޶�������뷢������

	//uart3��ر�����ʼ��
	recv3_state = FSA_INIT;
	recv3_timer = 0;
	recv3_ctr = 0;
	recv3_chksum = 0;
	trans3_size = 0;
	trans3_ctr = 0;

	for (i = 0; i < UART_QUEUE_NUM; i++){
		uart3_send_queue[i].flag = 0; //������
		uart3_recv_queue[i].flag = 0; //������
	}
	uart3_q_index = 0xFF;    //�޶�������뷢������

	//uart4��ر�����ʼ��
	recv4_state = FSA_INIT;
	recv4_timer = 0;
	recv4_ctr = 0;
	recv4_chksum = 0;
	trans4_size = 0;
	trans4_ctr = 0;

	for (i = 0; i < UART_QUEUE_NUM; i++){
		uart4_send_queue[i].flag = 0; //������
		uart4_recv_queue[i].flag = 0; //������
	}
	uart4_q_index = 0xFF;    //�޶�������뷢������

	//UARTӲ����ʼ��
	uart_init();             //֮���Ѿ�׼���ô����շ���ֻ�ǻ�δʹ��ȫ���ж�
}

void uart_task(void)
{
	Byte   i,j,k;
	Uint16 temp16;
	Byte   *ptr;
	
	//1.���� UART3��������λ���������
	//���Ƿ��еȴ��������
	for (k = 0; k < UART_QUEUE_NUM; k++)
	{
		if (uart3_recv_queue[k].flag == 1)//�еȴ��������
		{
			ptr = uart3_recv_queue[k].tdata;
			
			//�Ƿ���Ҫת��������
			if ((ptr[0] == CMD_ADDR_BC) || (ptr[0] != gl_comm_addr)) {
				//�㲥��ַ��Ǳ��豸, ��Ҫת���� UART4
				//��UART4 ���Ͷ������ҿ���Buffer
				i = uart4_get_send_buffer();
				if (i < UART_QUEUE_NUM) {
					//�ҵ��˿���buffer, д��data
					uart4_send_queue[i].tdata[0] = FRAME_STX;
					memcpy(&uart4_send_queue[i].tdata[1], ptr, ptr[2] + 3);
					uart4_send_queue[i].len = ptr[2] + 5;
				} 
			}

			//�Ƿ���Ҫִ�б�����
			if ((ptr[0] == CMD_ADDR_BC) || (ptr[0] == gl_comm_addr)) {
				//�㲥��ַ��ָ�����豸, ��Ҫִ��
				switch (ptr[3])
				{
				case CMD_DADDR_qSTAT://ѯ�ʷ���״̬ - �������λ��
					//��UART3���Ͷ������ҿ���Buffer
					i = uart3_get_send_buffer();
					if (i < UART_QUEUE_NUM) {
						//�ҵ��˿���buffer, ׼��Ӧ��
                        uart3_send_queue[i].package_type = 1;              //�豸��������ݰ�
                        
						uart3_send_queue[i].tdata[0] = FRAME_STX;	       //֡ͷ
						uart3_send_queue[i].tdata[1] = ptr[1];	           //Ŀ�ĵ�ַ
						uart3_send_queue[i].tdata[2] = gl_comm_addr;       //Դ��ַ
						if (gl_comm_addr == CMD_ADDR_UNSOLV) {             //���豸����Ч��ַ
							//ֻ�ز���Ӧ��
							uart3_send_queue[i].tdata[3] = 1;
							uart3_send_queue[i].tdata[4] = CMD_DADDR_aPARA;
							uart3_send_queue[i].len = 6;
						} else { //����Ч��ַ,�ط���״̬
							if ((alarm_out_flag & 0xF8) == 0x00) { //2���������ޱ���
								uart3_send_queue[i].tdata[3] = 1;
								uart3_send_queue[i].tdata[4] = CMD_ACK_OK;
								uart3_send_queue[i].len = 6;
							} else { //�б���
								uart3_send_queue[i].tdata[3] = 2;
								uart3_send_queue[i].tdata[4] = CMD_DADDR_aSTAT;
								uart3_send_queue[i].tdata[5] = (alarm_out_flag & 0xF8) >> 3;
								uart3_send_queue[i].len = 7;
                                
                                //���汨����ϸ��Ϣ
                                save_alarm_detail_info();
							}
						}
					}
					
					break;

				case CMD_DADDR_qPARA://ѯ�ʵ�ַ- �������λ��
					//��UART3���Ͷ������ҿ���Buffer
					i = uart3_get_send_buffer();
					if (i < UART_QUEUE_NUM) {
						//�ҵ��˿���buffer, ׼��Ӧ��
                        uart3_send_queue[i].package_type = 1;                      //�豸��������ݰ�
                        
						uart3_send_queue[i].tdata[0] = FRAME_STX;	               //֡ͷ
						uart3_send_queue[i].tdata[1] = ptr[1];	                   //Ŀ�ĵ�ַ
						uart3_send_queue[i].tdata[2] = gl_comm_addr;	           //Դ��ַ
						uart3_send_queue[i].tdata[3] = 1;                          //�����
						uart3_send_queue[i].tdata[4] = CMD_DADDR_aPARA;            //����ID
						uart3_send_queue[i].len = 6;
					}
					
					break;
                    
                case 0xE2://�޸ĵ�ַ
                    gl_comm_addr = ptr[4];
                
                    flash_enable();
                    flash_erase(EEPROM_SECTOR6);
                    flash_write(ptr[4], EEPROM_SECTOR6 + 1);
                    flash_write(0x5a, EEPROM_SECTOR6);
                    flash_disable();
                
                	//��UART3���Ͷ������ҿ���Buffer
					i = uart3_get_send_buffer();
					if (i < UART_QUEUE_NUM) {
						//�ҵ��˿���buffer, ׼��Ӧ��
                        uart3_send_queue[i].package_type = 1;                      //�豸��������ݰ�
                        
						uart3_send_queue[i].tdata[0] = FRAME_STX;	               //֡ͷ
						uart3_send_queue[i].tdata[1] = ptr[1];	                   //Ŀ�ĵ�ַ
						uart3_send_queue[i].tdata[2] = gl_comm_addr;	           //Դ��ַ
						uart3_send_queue[i].tdata[3] = 1;                          //�����
						uart3_send_queue[i].tdata[4] = CMD_DADDR_aPARA;            //����ID
						uart3_send_queue[i].len = 6;
					}
                    
                    break;
                
                case 0xE3://������ʱʱ��
                    //1. д��flash
                    flash_enable();                              
                    flash_erase(EEPROM_SECTOR10);                                        
                    flash_write(ptr[4], EEPROM_SECTOR10 + 1);  
                    flash_write(0x5a, EEPROM_SECTOR10);                                                              
                    flash_disable();

                    //2. ���±���
                    gl_reply_tick = ptr[4];

                    break;

                case 0xE4://��ȡ��ʱʱ��
                    //��UART3�������ҿ���Buffer
                    i = uart3_get_send_buffer();
                    if (i < UART_QUEUE_NUM) { //�ҵ��˿���buffer, ׼��Ӧ��
                        uart3_send_queue[i].package_type = 1;                          //�豸��������ݰ�
                        
                        uart3_send_queue[i].tdata[0] = FRAME_STX;	                   //֡ͷ
                        uart3_send_queue[i].tdata[1] = ptr[1];	                       //Ŀ�ĵ�ַ
                        uart3_send_queue[i].tdata[2] = gl_comm_addr;	               //Դ��ַ																 
                        uart3_send_queue[i].tdata[3] = 2;
                        uart3_send_queue[i].tdata[4] = 0xF4;
                        uart3_send_queue[i].tdata[5] = gl_reply_tick;
                        uart3_send_queue[i].len = 7;														
                    }
                    
                    break;	
                         
				case CMD_ZL_PRE://����/����ר�������־
					switch (ptr[5])
					{
					case 0x10: //�����ò���
						//��UART3�������ҿ���Buffer
						i = uart3_get_send_buffer();
						if (i < UART_QUEUE_NUM) {//�ҵ��˿���buffer, д��data
                            uart3_send_queue[i].package_type = 1;                     //�豸��������ݰ�
                            
							uart3_send_queue[i].tdata[0]  = FRAME_STX;
							uart3_send_queue[i].tdata[1]  = ptr[1];	                  //Ŀ�ĵ�ַ
							uart3_send_queue[i].tdata[2]  = gl_comm_addr;	          //Դ��ַ
							uart3_send_queue[i].tdata[3]  = 0x1E;                     //�����
							uart3_send_queue[i].tdata[4]  = CMD_ZL_PRE;		          //����ID
							uart3_send_queue[i].tdata[5]  = 0x1C;                     //����1
							uart3_send_queue[i].tdata[6]  = 0x08;                     //����2
							uart3_send_queue[i].tdata[7]  = HIGH(ad_sensor_mask_LR);  //����������
							uart3_send_queue[i].tdata[8]  = LOW(ad_sensor_mask_LR);   //����������
							uart3_send_queue[i].tdata[9]  = HIGH(ad_still_dn);        //��̬�����������޸�
							uart3_send_queue[i].tdata[10] = LOW(ad_still_dn);         //��̬�����������޵�
							uart3_send_queue[i].tdata[11] = HIGH(ad_still_up);        //��̬�����������޸�
							uart3_send_queue[i].tdata[12] = LOW(ad_still_up);         //��̬�����������޵�
							uart3_send_queue[i].tdata[13] = system_status;            //������ֵ�¸��������̶�Ϊ66
							for (j = 0; j < 6; j++) {                                 //������ֵ��1~6
								uart3_send_queue[i].tdata[14 + j] = ad_still_Dup[j];
							}

          					uart3_send_queue[i].tdata[20] = 0;                        //�󿪹���
							uart3_send_queue[i].tdata[21] = ad_still_Dup[12];         //������
                            
							for (j = 0; j < 6; j++) {                                 //������ֵ��1~6
								uart3_send_queue[i].tdata[22 + j] = ad_still_Dup[6 + j];
							}
                            uart3_send_queue[i].tdata[28] = 0;                        //�ҿ�����
							uart3_send_queue[i].tdata[29] = ad_still_Dup[12];         //������
                            
							uart3_send_queue[i].tdata[30] = 0;                        //˫/������
							uart3_send_queue[i].tdata[31] = gl_comm_addr;             //�����ַ
							uart3_send_queue[i].tdata[32] = (Byte)((beep_during_temp * SCHEDULER_TICK) / 1000);	//���ⱨ�����ʱ��
							uart3_send_queue[i].tdata[33] = 0;
							uart3_send_queue[i].len = 35;
						}
						
						break;

					case 0x12: //��������Ϣ
						//��UART3�������ҿ���Buffer
						i = uart3_get_send_buffer();
						if (i < UART_QUEUE_NUM) {
							//�ҵ��˿���buffer, д��data
                            uart3_send_queue[i].package_type = 1;                    //�豸��������ݰ�
                            
							uart3_send_queue[i].tdata[0]  = FRAME_STX;
							uart3_send_queue[i].tdata[1]  = ptr[1];	                 //Ŀ�ĵ�ַ
							uart3_send_queue[i].tdata[2]  = gl_comm_addr;	         //Դ��ַ
							uart3_send_queue[i].tdata[3]  = 0x0A;                    //�����
							uart3_send_queue[i].tdata[4]  = CMD_ZL_PRE;              //����ID
							uart3_send_queue[i].tdata[5]  = 0x08;                    //����1
							uart3_send_queue[i].tdata[6]  = 0x1A;                    //����1
							uart3_send_queue[i].tdata[7]  = HIGH(ad_sensor_mask_LR); //����������
							uart3_send_queue[i].tdata[8]  = LOW(ad_sensor_mask_LR);  //����������

							// �ˣ��󿪹�������6~1������־
							uart3_send_queue[i].tdata[9]  = HIGH(ad_alarm_exts);     //����������

							// �ˣ��ҿ���������6~1������־
							uart3_send_queue[i].tdata[10] = LOW(ad_alarm_exts);      //����������

							// �ˣ��󿪹�������6~1������־
							uart3_send_queue[i].tdata[11] = HIGH(ad_alarm_base);     //��̬����������

							// �ˣ��ҿ���������6~1������־
							uart3_send_queue[i].tdata[12] = LOW(ad_alarm_base);      //��̬����������
							
							//�Ŵ�
							uart3_send_queue[i].tdata[13] = (Byte)(gl_local_dk_status | !gl_control_dk_status);   
							uart3_send_queue[i].len = 15;
						}
						
						break;

					case 0x14: //��˲������
						//��UART3�������ҿ���Buffer
						i = uart3_get_send_buffer();
						if (i < UART_QUEUE_NUM) {
							//�ҵ��˿���buffer, д��data
                            uart3_send_queue[i].package_type = 1;               //�豸��������ݰ�
                            
							uart3_send_queue[i].tdata[0] = FRAME_STX;
							uart3_send_queue[i].tdata[1] = ptr[1];	            //Ŀ�ĵ�ַ
							uart3_send_queue[i].tdata[2] = gl_comm_addr;	    //Դ��ַ
							uart3_send_queue[i].tdata[3] = 0x23;
							uart3_send_queue[i].tdata[4] = CMD_ZL_PRE;
							uart3_send_queue[i].tdata[5] = 0x21;
							uart3_send_queue[i].tdata[6] = 0x1C;
							for (j = 0; j < 6; j++) { //��1~6
								temp16 = ad_chn_sample[j];
								uart3_send_queue[i].tdata[7 + (j << 1)] = HIGH(temp16);
								uart3_send_queue[i].tdata[8 + (j << 1)] = LOW(temp16);
							}
                            //�󿪹���
                            uart3_send_queue[i].tdata[19] = 0;
							uart3_send_queue[i].tdata[20] = 0;
                            
                            //������
                            temp16 = ad_chn_sample[12];
							uart3_send_queue[i].tdata[21] = HIGH(temp16);
							uart3_send_queue[i].tdata[22] = LOW(temp16);
                            
							for (j = 0; j < 6; j++) { //��1~6
								temp16 = ad_chn_sample[6 + j];
								uart3_send_queue[i].tdata[23 + (j << 1)] = HIGH(temp16);
								uart3_send_queue[i].tdata[24 + (j << 1)] = LOW(temp16);
							}
                            //�ҿ�����
							uart3_send_queue[i].tdata[35] = 0;
							uart3_send_queue[i].tdata[36] = 0;
                            
                            //������
                            temp16 = ad_chn_sample[12];
                            uart3_send_queue[i].tdata[37] = HIGH(temp16);
							uart3_send_queue[i].tdata[38] = LOW(temp16);
                            
							uart3_send_queue[i].len = 40;
						}
                        
						break;

					case 0x15: //����̬������׼
						//��UART3�������ҿ���Buffer
						i = uart3_get_send_buffer();
						if (i < UART_QUEUE_NUM) {
							//�ҵ��˿���buffer, д��data
                            uart3_send_queue[i].package_type = 1;                //�豸��������ݰ�
                            
							uart3_send_queue[i].tdata[0] = FRAME_STX;
							uart3_send_queue[i].tdata[1] = ptr[1];	            //Ŀ�ĵ�ַ
							uart3_send_queue[i].tdata[2] = gl_comm_addr;	    //Դ��ַ
							uart3_send_queue[i].tdata[3] = 0x23;
							uart3_send_queue[i].tdata[4] = CMD_ZL_PRE;
							uart3_send_queue[i].tdata[5] = 0x21;
							uart3_send_queue[i].tdata[6] = 0x1D;
							for (j = 0; j < 6; j++) { //��1~6
								temp16 = ad_chn_base[j].base;
								uart3_send_queue[i].tdata[7 + (j << 1)] = HIGH(temp16);
								uart3_send_queue[i].tdata[8 + (j << 1)] = LOW(temp16);
							}
                            //�󿪹���
                            uart3_send_queue[i].tdata[19] = 0;
							uart3_send_queue[i].tdata[20] = 0;
                            
                            //������
                            temp16 = ad_chn_base[12].base;
							uart3_send_queue[i].tdata[21] = HIGH(temp16);
							uart3_send_queue[i].tdata[22] = LOW(temp16);
                            
							for (j = 0; j < 6; j++) { //��1~6
								temp16 = ad_chn_base[6 + j].base;
								uart3_send_queue[i].tdata[23 + (j << 1)] = HIGH(temp16);
								uart3_send_queue[i].tdata[24 + (j << 1)] = LOW(temp16);
							}
                            //�ҿ�����
                            uart3_send_queue[i].tdata[35] = 0;
							uart3_send_queue[i].tdata[36] = 0;
                            
                            //������
                            temp16 = ad_chn_base[12].base;
							uart3_send_queue[i].tdata[37] = HIGH(temp16);
							uart3_send_queue[i].tdata[38] = LOW(temp16);
                            
							uart3_send_queue[i].len = 40;
						}
                        
						break;

					case 0x40: //���þ�̬����ֵ��Χ
						//1. д��flash
						flash_enable();
						flash_erase(EEPROM_SECTOR3);
						flash_write(ptr[6], EEPROM_SECTOR3 + 1);
						flash_write(ptr[7], EEPROM_SECTOR3 + 2);
						flash_write(ptr[8], EEPROM_SECTOR3 + 3);
						flash_write(ptr[9], EEPROM_SECTOR3 + 4);
						flash_write(0x5a, EEPROM_SECTOR3);
						flash_disable();

						//2. ���±���
						ad_still_dn = ((Uint16)ptr[6] << 8) + ptr[7];	 //����
						ad_still_up = ((Uint16)ptr[8] << 8) + ptr[9];	 //����

						//3. ��鵱ǰ��̬����ֵ
						if (system_status >= SYS_SELF_CHECK1) {
							//�ѿ�ʼ���м��
							for (i = 0; i < 12; i++) {
								check_still_stress(i);
							}
						}
						break;

					case 0x50: //���ñ�����ֵ (������)
						//1. д��flash�����±���
						flash_enable();
						flash_erase(EEPROM_SECTOR4);
						flash_write(ptr[6], EEPROM_SECTOR4 + 1); 	 //���޸���ֵ��������Ŀǰû�б���ȡʹ��

						for (j = 0; j < 6; j++) {
							//��1 ~6
							ad_still_Dup[j] = ptr[7 + j];
							flash_write(ptr[7 + j], EEPROM_SECTOR4 + 2 + j);
						}

						for (j = 0; j < 6; j++) {
							//��1 ~6
							ad_still_Dup[6 + j] = ptr[15 + j];
							flash_write(ptr[15 + j], EEPROM_SECTOR4 + 8 + j);
						}

                        //������
                        ad_still_Dup[12] = ptr[22];
						flash_write(ptr[22], EEPROM_SECTOR4 + 15);
                        
						flash_write(0x5a, EEPROM_SECTOR4);
						flash_disable();

						//���޹̶�ȡ��׼ֵ�� 1/3
						//2. ���»�����������������(����ֵ)
						if (system_status >= SYS_SELF_CHECK1) {
							//�ѿ�ʼ���м��
							for (i = 0; i < 13; i++) {
								if ((1023 - ad_chn_base[i].base) > ad_still_Dup[i])
									ad_chn_base[i].base_up = ad_chn_base[i].base + ad_still_Dup[i];
								else
									ad_chn_base[i].base_up = 1023;
							}
						}
						break;

					case 0x60: //�������ⱨ��ʱ��
						//1. д��flash
						flash_enable();
						flash_erase(EEPROM_SECTOR5);
						flash_write(ptr[6], EEPROM_SECTOR5 + 1);
						flash_write(0x5a, EEPROM_SECTOR5);
						flash_disable();

						//2. ���±���
						beep_during_temp = (Uint16)(((Uint32)ptr[6] * 1000) / SCHEDULER_TICK);
						break;

					case 0xF0: //�����˵������(����->�豸)
						//��UART2�������ҿ���Buffer
						i = uart2_get_send_buffer();
						if (i < UART_QUEUE_NUM) {
							//�ҵ��˿���buffer, д��data
							uart2_send_queue[i].tdata[0] = FRAME_STX;
							uart2_send_queue[i].tdata[1] = 0xFF;        //Ŀ�ĵ�ַ
							uart2_send_queue[i].tdata[2] = 0x01;	    //Դ��ַ
							uart2_send_queue[i].tdata[3] = 0x06;
							uart2_send_queue[i].tdata[4] = CMD_ZL_PRE;
							uart2_send_queue[i].tdata[5] = 0x04;
							uart2_send_queue[i].tdata[6] = 0xF0;
							uart2_send_queue[i].tdata[7] = ptr[6];      //�����
							uart2_send_queue[i].tdata[8] = ptr[7];      //��ת����ת��ֹͣ
							uart2_send_queue[i].tdata[9] = ptr[8];      //��תʱ��
							
							uart2_send_queue[i].len = 11;
                            
                            gl_chnn_index = matrix_index[ptr[6]];
						}
						
						break;

					case 0xF1: //���ô���������ƫ��---->������·�ϵ����
						//��UART2�������ҿ���Buffer
						i = uart2_get_send_buffer();
						if (i < UART_QUEUE_NUM) {
							//�ҵ��˿���buffer, д��data
							uart2_send_queue[i].tdata[0] = FRAME_STX;
							uart2_send_queue[i].tdata[1] = 0xFF;        //Ŀ�ĵ�ַ
							uart2_send_queue[i].tdata[2] = 0x01;	    //Դ��ַ
							uart2_send_queue[i].tdata[3] = 0x23;
							uart2_send_queue[i].tdata[4] = CMD_ZL_PRE;
							uart2_send_queue[i].tdata[5] = 0x21;
							uart2_send_queue[i].tdata[6] = 0xF1;
							
							//��1~8����1~8
							for (j = 0; j < 32; j++) {
								uart2_send_queue[i].tdata[7 + j] = ptr[6 + j];      
							} 

							uart2_send_queue[i].len = 40;
						}
                        
////////////////////////////////////////////////////////////////////////////////////    
                        //ֻ�б������������Ĳ���ֵ��������Ż�ȥ������ֵ�����Ƿ�ɹ�                                                    
                        if (ptr[0] == gl_comm_addr) {
                            temp16 = 0;
                            for (i = 0; i < 32; i++) {
                                temp16 += ptr[6 + i];
                            }
                        
                            if (temp16 == 0) {//����ֵ�ָ������
                                is_sample_clear = 0;
                                
                                flash_enable();
                                flash_erase(EEPROM_SECTOR9);
                                flash_write(0, EEPROM_SECTOR9 + 1);
                                flash_write(0x5a, EEPROM_SECTOR9);
                                flash_disable();
                            } else {//����ֵ��������
                                check_sample_clear_tick = 3000 / SCHEDULER_TICK;
                            }
                        } else if (ptr[0] == CMD_ADDR_BC) {
                            //�������䷢���Ĳ���ֵ���������ȥ������ֵ�����Ƿ�ɹ�
                            //ʲô������
                        }
						
						break;

////////////////////////////////////////////////////////////////////////////////////
                        
					case 0xF2: //���ò�������---->�����������ٸ�����ȷ����˿�Ƿ�Ƚ���
						break;

					case 0xF3: //����˿�Ƿ�����Լ��Ƿ��ڵ�����˿ģʽ
						//��UART3�������ҿ���Buffer
						i = uart3_get_send_buffer();
						if (i < UART_QUEUE_NUM) {
							//�ҵ��˿���buffer, д��data
                            uart3_send_queue[i].package_type = 1;               //�豸��������ݰ�
                            
							uart3_send_queue[i].tdata[0] = FRAME_STX;
							uart3_send_queue[i].tdata[1] = ptr[1];	            //Ŀ�ĵ�ַ
							uart3_send_queue[i].tdata[2] = gl_comm_addr;	    //Դ��ַ
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
                     
                    case 0xF4: //���õ������ͨ����:4�������5�������6�����
                        gl_motor_channel_number = ptr[6];
                    
						flash_enable();
						flash_erase(EEPROM_SECTOR7);
						flash_write(ptr[6], EEPROM_SECTOR7 + 1);
						flash_write(0x5a, EEPROM_SECTOR7);
						flash_disable();
                        break;
                    
                    case 0xF5: //���������ͨ����
                        //��UART3�������ҿ���Buffer
						i = uart3_get_send_buffer();
						if (i < UART_QUEUE_NUM) {
							//�ҵ��˿���buffer, д��data
                            uart3_send_queue[i].package_type = 1;               //�豸��������ݰ�
                            
							uart3_send_queue[i].tdata[0] = FRAME_STX;
							uart3_send_queue[i].tdata[1] = ptr[1];	            //Ŀ�ĵ�ַ
							uart3_send_queue[i].tdata[2] = gl_comm_addr;	    //Դ��ַ
							uart3_send_queue[i].tdata[3] = 0x04;
							uart3_send_queue[i].tdata[4] = CMD_ZL_PRE;
							uart3_send_queue[i].tdata[5] = 0x02;
							uart3_send_queue[i].tdata[6] = 0x1F;
                            uart3_send_queue[i].tdata[7] = gl_motor_channel_number;
							
							uart3_send_queue[i].len = 9;
						}
                        break;
                        
                    case 0xF6: //���õ�������Ƿ���Ӽ���
                        is_motor_add_link = ptr[6];
                        
                    	flash_enable();
						flash_erase(EEPROM_SECTOR8);
						flash_write(ptr[6], EEPROM_SECTOR8 + 1);
						flash_write(0x5a, EEPROM_SECTOR8);
						flash_disable();
                        break;
                    
                    case 0xF7: //����������Ƿ���Ӽ���
                        //��UART3�������ҿ���Buffer
						i = uart3_get_send_buffer();
						if (i < UART_QUEUE_NUM) {
							//�ҵ��˿���buffer, д��data
                            uart3_send_queue[i].package_type = 1;               //�豸��������ݰ�
                            
							uart3_send_queue[i].tdata[0] = FRAME_STX;
							uart3_send_queue[i].tdata[1] = ptr[1];	            //Ŀ�ĵ�ַ
							uart3_send_queue[i].tdata[2] = gl_comm_addr;	    //Դ��ַ
							uart3_send_queue[i].tdata[3] = 0x04;
							uart3_send_queue[i].tdata[4] = CMD_ZL_PRE;
							uart3_send_queue[i].tdata[5] = 0x02;
							uart3_send_queue[i].tdata[6] = 0x20;
                            uart3_send_queue[i].tdata[7] = (Byte)is_motor_add_link;
							
							uart3_send_queue[i].len = 9;
						}
                        break;
                        
                    //2016-02-28����
                    case 0xF8://��ȡ������ϸ��Ϣ                                
                        get_alarm_detail_info();
                        break;
                    
                    case 0xF9: //����ƽ��ֵ����-->�������ٸ�����ƽ��ֵ                        
						//��UART2�������ҿ���Buffer
						i = uart2_get_send_buffer();
						if (i < UART_QUEUE_NUM) {
							//�ҵ��˿���buffer, д��data
							uart2_send_queue[i].tdata[0] = FRAME_STX;
							uart2_send_queue[i].tdata[1] = 0xFF;        //Ŀ�ĵ�ַ
							uart2_send_queue[i].tdata[2] = 0x01;	    //Դ��ַ
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
                        
                        //������Ϣ
                        //��UART3�������ҿ���Buffer
						i = uart3_get_send_buffer();
						if (i < UART_QUEUE_NUM) {
							//�ҵ��˿���buffer, д��data
                            uart3_send_queue[i].package_type = 1;               //�豸��������ݰ�
                            
							uart3_send_queue[i].tdata[0] = FRAME_STX;
							uart3_send_queue[i].tdata[1] = ptr[1];	            //Ŀ�ĵ�ַ
							uart3_send_queue[i].tdata[2] = gl_comm_addr;	    //Դ��ַ
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
                        
                    case 0xFA: //���ñ�������-->�������ٸ��㱨�����ж�Ϊ����
                        //���±���
                        if (ptr[6] < 4) {
                            alarm_point_num = 4;

                        } else if (ptr[6] > 8){
                            alarm_point_num = 8;
                        } else {
                            alarm_point_num = ptr[6];
                        }
                        
                        //д��flash
                        flash_enable();
						flash_erase(EEPROM_SECTOR11);
						flash_write(alarm_point_num, EEPROM_SECTOR11 + 1);
						flash_write(0x5a, EEPROM_SECTOR11);
						flash_disable();
                        
                        //������Ϣ
                        //��UART3�������ҿ���Buffer
						i = uart3_get_send_buffer();
						if (i < UART_QUEUE_NUM) {
							//�ҵ��˿���buffer, д��data
                            uart3_send_queue[i].package_type = 1;               //�豸��������ݰ�
                            
							uart3_send_queue[i].tdata[0] = FRAME_STX;
							uart3_send_queue[i].tdata[1] = ptr[1];	            //Ŀ�ĵ�ַ
							uart3_send_queue[i].tdata[2] = gl_comm_addr;	    //Դ��ַ
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
                
                //����Ӧ����ʱ
                Disable_interrupt();
                gl_ack_tick = REPLY_DLY + (gl_comm_addr - 16) * gl_reply_tick / SCHEDULER_TICK;
                Enable_interrupt();
			}
			
			//�������,�ͷŸö�����
			uart3_recv_queue[k].flag = 0;
			
			break;
		}
	}

	//2.����UART4��������λ���������
	//���Ƿ��еȴ��������
	for (k = 0; k < UART_QUEUE_NUM; k++)
	{
		if (uart4_recv_queue[k].flag == 1)//�еȴ��������
		{
			ptr = uart4_recv_queue[k].tdata;
			
			//ת��������,��UART3�������ҿ���Buffer
			i = uart3_get_send_buffer();
			if (i < UART_QUEUE_NUM) {
				//�ҵ��˿���buffer, д��data
                uart3_send_queue[i].package_type = 0;         //������λ�������ݰ�
                
				uart3_send_queue[i].tdata[0] = FRAME_STX;
				memcpy(&uart3_send_queue[i].tdata[1], ptr, ptr[2] + 3);
				uart3_send_queue[i].len = ptr[2] + 5;
			}
			
			//�������,�ͷŸö�����
			uart4_recv_queue[k].flag = 0;

			break;
		}
	}
	
	//3. UART3 ���з���
	if ((uart3_q_index == 0xFF) && (recv3_state == FSA_INIT)) {
		//UART3�޽��뷢�����̵Ķ�����, ���Ƿ��еȴ����͵���
		for (i = 0; i < UART_QUEUE_NUM; i++) {
			if ((uart3_send_queue[i].flag == 1) && (uart3_send_queue[i].package_type == 0)){
                //������λ�������ݰ�:�еȴ����͵�����Ŵ����
				uart3_send_queue[i].flag = 2;
				uart3_q_index = i;
				memcpy(trans3_buf, uart3_send_queue[i].tdata, uart3_send_queue[i].len - 1);
				trans3_size = uart3_send_queue[i].len;
				uart3_start_trans();
				break;
			}else if((uart3_send_queue[i].flag == 1) && (uart3_send_queue[i].package_type == 1) && (gl_ack_tick == 0)){
                //�豸��������ݰ�:�еȴ����͵�����Ŵ����
				uart3_send_queue[i].flag = 2;
				uart3_q_index = i;
				memcpy(trans3_buf, uart3_send_queue[i].tdata, uart3_send_queue[i].len - 1);
				trans3_size = uart3_send_queue[i].len;
				uart3_start_trans();
				break;
            }
		}
	}

	//4. UART4 ���з���
	if ((uart4_q_index == 0xFF) && (recv4_state == FSA_INIT)) {
		//UART4�޽��뷢�����̵Ķ�����, ���Ƿ��еȴ����͵���
		for (i = 0; i < UART_QUEUE_NUM; i++) {
			if (uart4_send_queue[i].flag == 1) {
				//�еȴ����͵�����Ŵ����
				uart4_send_queue[i].flag = 2;
				uart4_q_index = i;
				memcpy(trans4_buf, uart4_send_queue[i].tdata, uart4_send_queue[i].len - 1);
				trans4_size = uart4_send_queue[i].len;
				uart4_start_trans();
				break;
			}
		}
	}
	
	//5. UART2 ���з���
	if (uart2_q_index == 0xFF) {
		//UART2�޽��뷢�����̵Ķ�����, ���Ƿ��еȴ����͵���
		for (i = 0; i < UART_QUEUE_NUM; i++) {
			if (uart2_send_queue[i].flag == 1) {
				//�еȴ����͵�����Ŵ����
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
*         ������ֵ >= UART_QUEUE_NUM, ���ʾû�����뵽����buffer
*----------------------------------------------------------------------------
* PURPOSE: �ڴ���2������Ѱ�ҿ��ж�������ҵ������ض��������(0 ~ (UART_QUEUE_NUM-1))
*****************************************************************************/

Byte uart2_get_send_buffer(void)
{
	Byte i, flag;

	for (i = 0; i < UART_QUEUE_NUM; i++)
	{
		Disable_interrupt();
		flag = uart2_send_queue[i].flag;
		Enable_interrupt();
		if (flag == 0)   //���ҵ�����Buffer
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
		if (flag == 0)   //���ҵ�����Buffer
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
*         ������ֵ >= UART_QUEUE_NUM, ���ʾû�����뵽����buffer
*----------------------------------------------------------------------------
* PURPOSE: �ڴ���2������Ѱ�ҿ��ж�������ҵ������ض��������(0 ~ (UART_QUEUE_NUM-1))
*****************************************************************************/
Byte uart3_get_send_buffer(void)
{
	Byte i, flag;

	for (i = 0; i < UART_QUEUE_NUM; i++)
	{
		Disable_interrupt();
		flag = uart3_send_queue[i].flag;
		Enable_interrupt();
		if (flag == 0)   //���ҵ�����Buffer
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
		if (flag == 0)   //���ҵ�����Buffer
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
*         ������ֵ >= UART_QUEUE_NUM, ���ʾû�����뵽����buffer
*----------------------------------------------------------------------------
* PURPOSE: �ڴ���2������Ѱ�ҿ��ж�������ҵ������ض��������(0 ~ (UART_QUEUE_NUM-1))
*****************************************************************************/
Byte uart4_get_send_buffer(void)
{
	Byte i, flag;

	for (i = 0; i < UART_QUEUE_NUM; i++)
	{
		Disable_interrupt();
		flag = uart4_send_queue[i].flag;
		Enable_interrupt();
		if (flag == 0)   //���ҵ�����Buffer
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
		if (flag == 0)   //���ҵ�����Buffer
		{
			uart4_recv_queue[i].flag = 1;
			break;
		}
	}
	return i;
}

//2016-02-28����
void get_alarm_detail_info(void)
{
    Byte i,j;
    Uint16 temp16;
    
    //1������������Ϣ   
    //��UART3�������ҿ���Buffer
    i = uart3_get_send_buffer();
    if (i < UART_QUEUE_NUM) {//�ҵ��˿���buffer, д��data
        uart3_send_queue[i].package_type = 1;                     //�豸��������ݰ�
        
        uart3_send_queue[i].tdata[0]  = FRAME_STX;
        uart3_send_queue[i].tdata[1]  = 0x01;	                  //Ŀ�ĵ�ַ
        uart3_send_queue[i].tdata[2]  = gl_comm_addr;	          //Դ��ַ
        uart3_send_queue[i].tdata[3]  = 0x1E;                     //�����
        uart3_send_queue[i].tdata[4]  = CMD_ZL_PRE;		          //����ID
        uart3_send_queue[i].tdata[5]  = 0x1C;                     //����1
        uart3_send_queue[i].tdata[6]  = 0x08;                     //����2
        uart3_send_queue[i].tdata[7]  = HIGH(ad_sensor_mask_LR);  //����������
        uart3_send_queue[i].tdata[8]  = LOW(ad_sensor_mask_LR);   //����������
        uart3_send_queue[i].tdata[9]  = HIGH(ad_still_dn);        //��̬�����������޸�
        uart3_send_queue[i].tdata[10] = LOW(ad_still_dn);         //��̬�����������޵�
        uart3_send_queue[i].tdata[11] = HIGH(ad_still_up);        //��̬�����������޸�
        uart3_send_queue[i].tdata[12] = LOW(ad_still_up);         //��̬�����������޵�
        uart3_send_queue[i].tdata[13] = system_status;            //������ֵ�¸�����
        for (j = 0; j < 6; j++) {                                 //������ֵ��1~6
            uart3_send_queue[i].tdata[14 + j] = ad_still_Dup[j];
        }

        uart3_send_queue[i].tdata[20] = 0;                        //�󿪹���
        uart3_send_queue[i].tdata[21] = ad_still_Dup[12];         //������
        
        for (j = 0; j < 6; j++) {                                 //������ֵ��1~6
            uart3_send_queue[i].tdata[22 + j] = ad_still_Dup[6 + j];
        }
        uart3_send_queue[i].tdata[28] = 0;                        //�ҿ�����
        uart3_send_queue[i].tdata[29] = ad_still_Dup[12];         //������
        
        uart3_send_queue[i].tdata[30] = 0;                        //˫/������
        uart3_send_queue[i].tdata[31] = gl_comm_addr;             //�����ַ
        uart3_send_queue[i].tdata[32] = (Byte)((beep_during_temp * SCHEDULER_TICK) / 1000);	//���ⱨ�����ʱ��
        uart3_send_queue[i].tdata[33] = 0;
        uart3_send_queue[i].len = 35;
    }
          
    //2����ȡ������Ϣ
    //��UART3�������ҿ���Buffer
    i = uart3_get_send_buffer();
    if (i < UART_QUEUE_NUM) {
        //�ҵ��˿���buffer, д��data
        uart3_send_queue[i].package_type = 1;                    //�豸��������ݰ�
        
        uart3_send_queue[i].tdata[0]  = FRAME_STX;
        uart3_send_queue[i].tdata[1]  = 0x01;	                 //Ŀ�ĵ�ַ
        uart3_send_queue[i].tdata[2]  = gl_comm_addr;	         //Դ��ַ
        uart3_send_queue[i].tdata[3]  = 0x0A;                    //�����
        uart3_send_queue[i].tdata[4]  = CMD_ZL_PRE;              //����ID
        uart3_send_queue[i].tdata[5]  = 0x08;                    //����1
        uart3_send_queue[i].tdata[6]  = 0x1A;                    //����1
        uart3_send_queue[i].tdata[7]  = HIGH(ad_sensor_mask_LR); //����������
        uart3_send_queue[i].tdata[8]  = LOW(ad_sensor_mask_LR);  //����������

        // �ˣ��󿪹�������6~1������־
        uart3_send_queue[i].tdata[9]  = HIGH(AlarmDetailInfo.ExternalAlarm);     //����������

        // �ˣ��ҿ���������6~1������־
        uart3_send_queue[i].tdata[10] = LOW(AlarmDetailInfo.ExternalAlarm);      //����������

        // �ˣ��󿪹�������6~1������־
        uart3_send_queue[i].tdata[11] = HIGH(AlarmDetailInfo.StaticAlarm);     //��̬����������

        // �ˣ��ҿ���������6~1������־
        uart3_send_queue[i].tdata[12] = LOW(AlarmDetailInfo.StaticAlarm);      //��̬����������
        
        //�Ŵ�
        uart3_send_queue[i].tdata[13] = (Byte)(gl_local_dk_status | !gl_control_dk_status);   
        uart3_send_queue[i].len = 15;
    }
    
    //3����˲̬����
    //��UART3�������ҿ���Buffer
    i = uart3_get_send_buffer();
    if (i < UART_QUEUE_NUM) {
        //�ҵ��˿���buffer, д��data
        uart3_send_queue[i].package_type = 1;               //�豸��������ݰ�
        
        uart3_send_queue[i].tdata[0] = FRAME_STX;
        uart3_send_queue[i].tdata[1] = 0x01;	            //Ŀ�ĵ�ַ
        uart3_send_queue[i].tdata[2] = gl_comm_addr;	    //Դ��ַ
        uart3_send_queue[i].tdata[3] = 0x23;
        uart3_send_queue[i].tdata[4] = CMD_ZL_PRE;
        uart3_send_queue[i].tdata[5] = 0x21;
        uart3_send_queue[i].tdata[6] = 0x1C;
        for (j = 0; j < 6; j++) { //��1~6
            temp16 = AlarmDetailInfo.InstantSampleValue[j];
            uart3_send_queue[i].tdata[7 + (j << 1)] = HIGH(temp16);
            uart3_send_queue[i].tdata[8 + (j << 1)] = LOW(temp16);
        }
        //�󿪹���
        uart3_send_queue[i].tdata[19] = 0;
        uart3_send_queue[i].tdata[20] = 0;
        
        //������
        temp16 = AlarmDetailInfo.InstantSampleValue[12];
        uart3_send_queue[i].tdata[21] = HIGH(temp16);
        uart3_send_queue[i].tdata[22] = LOW(temp16);
        
        for (j = 0; j < 6; j++) { //��1~6
            temp16 = AlarmDetailInfo.InstantSampleValue[6+j];
            uart3_send_queue[i].tdata[23 + (j << 1)] = HIGH(temp16);
            uart3_send_queue[i].tdata[24 + (j << 1)] = LOW(temp16);
        }
        //�ҿ�����
        uart3_send_queue[i].tdata[35] = 0;
        uart3_send_queue[i].tdata[36] = 0;
        
        //������
        temp16 = AlarmDetailInfo.InstantSampleValue[12];
        uart3_send_queue[i].tdata[37] = HIGH(temp16);
        uart3_send_queue[i].tdata[38] = LOW(temp16);
        
        uart3_send_queue[i].len = 40;
    }
    
    //4������̬������׼
    //��UART3�������ҿ���Buffer
    i = uart3_get_send_buffer();
    if (i < UART_QUEUE_NUM) {
        //�ҵ��˿���buffer, д��data
        uart3_send_queue[i].package_type = 1;                //�豸��������ݰ�
        
        uart3_send_queue[i].tdata[0] = FRAME_STX;
        uart3_send_queue[i].tdata[1] = 0x01;	            //Ŀ�ĵ�ַ
        uart3_send_queue[i].tdata[2] = gl_comm_addr;	    //Դ��ַ
        uart3_send_queue[i].tdata[3] = 0x23;
        uart3_send_queue[i].tdata[4] = CMD_ZL_PRE;
        uart3_send_queue[i].tdata[5] = 0x21;
        uart3_send_queue[i].tdata[6] = 0x1D;
        for (j = 0; j < 6; j++) { //��1~6
            temp16 = AlarmDetailInfo.StaticBaseValue[j];
            uart3_send_queue[i].tdata[7 + (j << 1)] = HIGH(temp16);
            uart3_send_queue[i].tdata[8 + (j << 1)] = LOW(temp16);
        }
        //�󿪹���
        uart3_send_queue[i].tdata[19] = 0;
        uart3_send_queue[i].tdata[20] = 0;
        
        //������
        temp16 = AlarmDetailInfo.StaticBaseValue[12];
        uart3_send_queue[i].tdata[21] = HIGH(temp16);
        uart3_send_queue[i].tdata[22] = LOW(temp16);
        
        for (j = 0; j < 6; j++) { //��1~6
            temp16 = AlarmDetailInfo.StaticBaseValue[6+j];
            uart3_send_queue[i].tdata[23 + (j << 1)] = HIGH(temp16);
            uart3_send_queue[i].tdata[24 + (j << 1)] = LOW(temp16);
        }
        //�ҿ�����
        uart3_send_queue[i].tdata[35] = 0;
        uart3_send_queue[i].tdata[36] = 0;
        
        //������
        temp16 = AlarmDetailInfo.StaticBaseValue[12];
        uart3_send_queue[i].tdata[37] = HIGH(temp16);
        uart3_send_queue[i].tdata[38] = LOW(temp16);
        
        uart3_send_queue[i].len = 40;
    }
}