#include "task/adc/adc_task.h"

/* UART2 */
extern xdata  sUART_Q  uart2_send_queue[UART_QUEUE_NUM];     // ���ڷ��Ͷ���
extern xdata  sUART_Q  uart2_recv_queue[UART_QUEUE_NUM];     // ���ڽ��ն���

/* for AD */
extern xdata  Uint16   ad_sensor_mask_LR;              //������˳���������sensor mask: �ˡ��󿪹�������6 ~ 1,�ˡ��ҿ���������6 ~ 1
extern xdata  Uint16   ad_sensor_mask;                 //15  14  13  12  11  10  9  8  7   6   5  4  3  2  1  0
											           //				��6			      ��1 ��6       	 ��1	

extern xdata  Union16  ad_chn_sample[13];              //����һ�ֲ���ֵ���Ѿ���ȥ������ÿͨ��һ���㣬ѭ�����棩
extern xdata  Uint16   ad_samp_pnum;                   //��������(���㾲̬��׼ֵʱ�ܲ�������)
extern xdata  sAD_Sum  ad_samp_sum[13];                //�׶����
extern xdata  sAD_BASE ad_chn_base[13];                //��ͨ������ʱ��̬��׼ֵ/�����޷�ֵ����λ������ֵ��
extern xdata  Byte     ad_chn_over[13];                //��ͨ������������(�����)�ķ�ֵ�ж��� 0 - ��Χ�ڣ� 1 - ����ֵ
                                                       //ÿͨ��һ���ֽڣ� CH0~12 ��Ӧ ad_chn_over[0~12]
extern xdata  Uint16   ad_still_dn;                    //��̬����ֵ����
extern xdata  Uint16   ad_still_up;                    //��̬����ֵ����
extern xdata  Byte     ad_still_Dup[13];               //������ֵ����
extern  data  Uint16   ad_alarm_exts;                  //����������־����mask���� λֵ 0 - �ޣ� 1 - ����ֵ
extern  data  Uint16   ad_alarm_base;                  //��̬����������־����mask���� λֵ 0 - ����Χ�ڣ� 1 - ������Χ
extern xdata  Uint16   ad_chnn_state[12];              //����������˿���ɽ��̶ȣ��������ҵ�˳��Ϊ����1~��6,��1~��6

/* for this task: ���ڻ�׼ֵ���� */
static xdata  Byte     md_point[13];                   //���ڻ�׼ֵ���ٵļ�������

/* for system */
extern data  Byte     system_status;                   //ϵͳ״̬
extern xdata Byte     matrix_index[12];

/* variables for alarm output */
extern bdata  bit      adl_alarm_flag;                 //���������ϱ�����־: 0 - �ޱ���; 1 - ����ֵ������
extern bdata  bit      adr_alarm_flag;                 //�Ҳ�������ϱ�����־: 0 - �ޱ���; 1 - ����ֵ������
                                                       //  ����ԭ�����Ϊ������������ ��̬��������Χ����
                                                       //  ZZX: �Ѿ��� mask ����, δ���Ŵ�״̬
extern xdata  Uint16   ad_alarm_tick[13];              //��ͨ��������ʱtick

/* �����ת��־ */
extern bdata  bit      gl_motor_overcur_flag;          //����Ƿ��ڶ�ת״̬��0-��������;1-�����ת
extern bdata  bit      gl_motor_adjust_flag;           //����Ƿ��ڹ���״̬��0-ֹͣ����״̬;1-�����ڹ���״̬
extern bdata  bit      is_timeout;                     //���ʱ���Ƿ����꣺0-û��;1-ʱ������			
extern xdata  Byte     gl_chnn_index;                  //��ǰ���ڵ����ĸ�˿������
extern xdata  Byte     gl_motor_overcur_point[12];     //�����ת����
extern xdata  Byte     gl_motor_adjust_end[12];        //�Ƿ������ɣ�0-û�е������;1-��ʾ�������
extern xdata  Uint16   ad_chnn_wire_cut;               //0-��ʾ��˿û�б�����;1-��ʾ��˿������ -->��6~��1����6~��1
extern xdata  Byte     gl_wait_delay_tick;             //�������Ƶ��������Ժ���ʱ�ȴ���ʱ�䣬Ȼ���ͬ�����»�׼ֵ�Լ��Ƿ�����ת��ʱ���Ƿ�����

/* Doorkeep(�Ŵ�) */
extern bdata  bit      gl_control_dk_status;           //���Ƹ˵��Ŵ�״̬: 1 - �պ�; 0 - ��(��Ҫ����)                    				

extern xdata  Byte        test;
/* ��������*/
extern Byte uart2_get_send_buffer(void);

void adc_task_init(void)
{
	Byte i;

	//��ر�����ʼ��
	ad_samp_pnum    = 0;                     //��������(���㾲̬��׼ֵʱ�ܲ�������)
	for (i = 0; i < 13; i++)
	{
		ad_chn_sample[i].w       = 0;        //����һ�ֲ���ֵ
		ad_samp_sum[i].sum       = 0;        //�׶����
		ad_samp_sum[i].point     = 0;
		ad_chn_base[i].base      = 0;        //��ͨ����̬��׼ֵ/�����޷�ֵ
		ad_chn_base[i].base_down = 0;
		ad_chn_base[i].base_up   = 0;
		ad_chn_over[i]           = 0;        //��ͨ������������(�����)�ķ�ֵ�ж������ڷ�Χ��
		md_point[i]              = 0;        //���ڻ�׼ֵ���ٵļ�������
		ad_alarm_tick[i]         = 0;
	}
	
	for (i = 0; i < 12; i++) {
		gl_motor_overcur_point[i] = 0;
		gl_motor_adjust_end[i]    = 0;
		ad_chnn_state[i]          = 0;
	}

	ad_alarm_exts    = 0;                    //����������־����mask��: ��
	ad_alarm_base    = 0;                    //��̬����������־����mask��������Χ��
	gl_chnn_index    = 0;                    //����1��ʼ
	ad_chnn_wire_cut = 0;
	test = 10;
}

void adc_task(void)
{
	Byte i,j;            //ѭ������
    Byte isValidData = 0;
	Uint16 val_temp;
    Byte val_sum;
	Byte temp;
	Uint16 val;
	Byte index;
	
	//1.����UART2�����Կ��Ƹ˵����ݰ�
	//���Ƿ��еȴ��������
	for (i = 0; i < UART_QUEUE_NUM; i++)
	{
		if (uart2_recv_queue[i].flag == 1)//�еȴ��������
		{           
            isValidData = 1;
						
			//��1~6,��1~6,�������˲������ֵ
			for (j = 0; j < 13; j++) {
				ad_chn_sample[j].w = ((Uint16)uart2_recv_queue[i].tdata[6 + (j << 1)] << 8) + uart2_recv_queue[i].tdata[7 + (j << 1)];
			}
			
			//���Ƹ˵��Ŵ�״̬
			gl_control_dk_status   = uart2_recv_queue[i].tdata[32];//0-����;1-����
			
			//���״̬
			gl_motor_adjust_flag   = uart2_recv_queue[i].tdata[33];//0-ֹͣ;1-��ת
			
			//�����ת״̬
			gl_motor_overcur_flag  = uart2_recv_queue[i].tdata[34];//0-����;1-��ת
				
			//ʱ���Ƿ�����
			is_timeout             = uart2_recv_queue[i].tdata[35];//0-û��;1-ʱ������
			
			//�������,�ͷŸö�����
			uart2_recv_queue[i].flag = 0;

			break;
		}
	}

    if (isValidData == 0) {//û�еȴ��������
        return;
    }
    
	switch (system_status)
	{
	case SYS_SAMP_BASE: //��ʼ�ϵ�ʱ�ľ�̬��׼ֵ����        
		for (i = 0; i < 13; i++) {
			ad_samp_sum[i].sum += ad_chn_sample[i].w;
		}
		ad_samp_pnum++;
		
		if (ad_samp_pnum == 32) //�Ѿ�����׼ֵ����������ÿͨ��32�㣬�����, ��ʱԼ10��)
		{			
			ad_samp_pnum = 0;
			
			//�����ֵ��������
			for (i = 0; i < 13; i++)
			{
				//��׼
				ad_chn_base[i].base = ad_samp_sum[i].sum >> 5;   //����32

				//���� = ��׼ * ��1 / 3��
				val_temp = ad_chn_base[i].base;
				ad_chn_base[i].base_down = val_temp / 3;

				//����
				if ((1023 - ad_chn_base[i].base) > ad_still_Dup[i])
				{
					ad_chn_base[i].base_up = ad_chn_base[i].base + ad_still_Dup[i];
				}
				else
				{
					ad_chn_base[i].base_up = 1023;
				}

				//��龲̬�����Ƿ�������Χ��
				//ֻ����6����˿����6����˿����Ҫ�ж��Ƿ�̬����,���������ھ�̬����,������ֻ����������
				if ((i >= 0) && (i <= 11))
				{
					check_still_stress(i);
				}

				//��λ�׶κͱ�����׼����������Ӧ��ֵ����
				ad_samp_sum[i].sum   = 0;
				ad_samp_sum[i].point = 0;
			}

			//����led״̬
			update_led_status();
		
			//״̬->�����Լ�
			system_status = SYS_SELF_CHECK1;
		}
        
		break;
		
	case SYS_SELF_CHECK1://�����Լ�׶�1-->��1 ��1 ��2 ��2 ... ��6 ��6
		//gl_chnn_indexȡֵ��ΧΪ0~11���ֱ����ζ�ӦΪ����1 ��1 ��2 ��2 ... ��6 ��6
		//ad_chn_sample/ad_chn_base��0~5��Ӧ��1~��6,6~11��Ӧ��1~��6��������Ҫת��һ������
		index = matrix_index[gl_chnn_index];
		         						
		//�жϲ��뿪���Ƿ��
		if ((ad_sensor_mask >> index) & 0x0001) {//�����
			if (ad_chn_sample[index].w < 100) {//��˿�Ƚ���
				system_status = SYS_SELF_CHECK2;
				gl_motor_adjust_flag  = 0;
				gl_motor_overcur_flag = 0;
				is_timeout            = 0;
				gl_wait_delay_tick    = 4;//��ʱ�ȴ�20ms
				motor_adjust(gl_chnn_index);
			} else {
				//����˿�������
				gl_motor_adjust_end[index] = 1;
				
				//������һ����˿
				gl_chnn_index++;
				if (gl_chnn_index == 12) {
					gl_chnn_index = 0;
				}
			}
		} else {//����ر�
			//����˿�������
			gl_motor_adjust_end[index] = 1;
				
			//������һ����˿
			gl_chnn_index++;
			if (gl_chnn_index == 12) {
				gl_chnn_index = 0;
			}
		}
		
		//�ж����и�˿�Ƿ�������
		val_sum = 0;
		for (i = 0; i < 12; i++) {
			val_sum += gl_motor_adjust_end[i];
		}

		if (val_sum == 12) {//���и�˿�������
			system_status = SYS_CHECK;
			gl_chnn_index = 0;
		}
		
		break;
		
	case SYS_SELF_CHECK2://�����Լ�׶�2
		if (gl_wait_delay_tick > 0) {//��ʱ�ȴ�20ms
			return;
		}
		
		index = matrix_index[gl_chnn_index];
		if (gl_motor_adjust_flag == 1) {//���������ת
			//����ͬ�����¾�̬��׼ֵ
			ad_chn_base[index].base = ad_chn_sample[index].w;
			val_temp = ad_chn_base[index].base;
			ad_chn_base[index].base_down = val_temp / 3;
			if ((1023 - ad_chn_base[index].base) > ad_still_Dup[index]) {
				ad_chn_base[index].base_up = ad_chn_base[index].base + ad_still_Dup[index];
			} else {
				ad_chn_base[index].base_up = 1023;
			}
		}
			
		//��龲̬�����Ƿ�������Χ��
		//ֻ����6����˿����6����˿����Ҫ�ж��Ƿ�̬����,���������ھ�̬����,������ֻ����������
		check_still_stress(index);
		
		//����led״̬
		update_led_status();
		
		if (gl_motor_overcur_flag == 1) {//�����ת
			gl_motor_overcur_point[index]++;
			if (gl_motor_overcur_point[index] >= 3) {
				gl_motor_adjust_end[index] = 1;//������˿��������
			}
			
			//������һ����˿
			gl_chnn_index++;
			if (gl_chnn_index == 12) {
				gl_chnn_index = 0;
			}
			system_status = SYS_SELF_CHECK1;
		}
		
		if (is_timeout == 1) {//ʱ�����꣬������һ����˿
			gl_chnn_index++;
			if (gl_chnn_index == 12) {
				gl_chnn_index = 0;
			}		
			system_status = SYS_SELF_CHECK1;
		}
						
		//�ж����и�˿�Ƿ�������
		val_sum = 0;
		for (i = 0; i < 12; i++) {
			val_sum += gl_motor_adjust_end[i];
		}
		
		if (val_sum == 12) {//���и�˿�������
			system_status = SYS_CHECK;
			gl_chnn_index = 0;
		}
		
		break;
		
	case SYS_CHECK:     //ʵʱ���   
		//������˿���ɽ��̶�
		for (i = 0; i < 12; i++) {
			ad_chnn_state[i] += ad_chn_sample[i].w;
		}
		ad_samp_pnum++;
		if (ad_samp_pnum == 45) {//��Լ��ʱ11.7s�����ת��ʱ��Ϊ10s
			ad_samp_pnum = 0;
			
			for (i = 0; i < 12; i++) {//��1 ��1 ��2 ��2 ��3 ��3 ��4 ��4 ��5 ��5 ��6 ��6
				//�жϲ��뿪���Ƿ��
				gl_chnn_index = matrix_index[i];
				if ((ad_sensor_mask >> gl_chnn_index) & 0x0001) {//�����
					if ((gl_chnn_index >= 0) && (gl_chnn_index <= 5)) {//��1~��6					
						if (((ad_chnn_wire_cut >> (gl_chnn_index + 8)) & 0x0001) == 0) {//��˿û�б�����
							val = ad_chnn_state[gl_chnn_index] / 45;
							if (val < 60) {//��˿�Ƚ��ɣ��ս���˿
								gl_motor_adjust_flag  = 0;
								gl_motor_overcur_flag = 0;
								is_timeout            = 0;
								motor_adjust(i);
								break;
							}
						}
					} else if ((gl_chnn_index >= 6) && (gl_chnn_index <= 11)) {//��1~��6					
						if (((ad_chnn_wire_cut >> (gl_chnn_index - 6)) & 0x0001) == 0) {//��˿û�б�����
							val = ad_chnn_state[gl_chnn_index] / 45;
							if (val < 60) {//��˿�Ƚ��ɣ��ս���˿
								gl_motor_adjust_flag  = 0;
								gl_motor_overcur_flag = 0;
								is_timeout            = 0;
								motor_adjust(i);
								break;
							}
						}
					}
				}
			}
			
			for (i = 0; i < 12; i++) {
				ad_chnn_state[i] = 0;
			}
		}
		
		if (gl_motor_adjust_flag == 1) {//���������ת
			//����ͬ�����¾�̬��׼ֵ
			ad_chn_base[gl_chnn_index].base = ad_chn_sample[gl_chnn_index].w;
			val_temp = ad_chn_base[gl_chnn_index].base;
			ad_chn_base[gl_chnn_index].base_down = val_temp / 3;
			if ((1023 - ad_chn_base[gl_chnn_index].base) > ad_still_Dup[gl_chnn_index]) {
				ad_chn_base[gl_chnn_index].base_up = ad_chn_base[gl_chnn_index].base + ad_still_Dup[gl_chnn_index];
			} else {
				ad_chn_base[gl_chnn_index].base_up = 1023;
			}
		}
			
		//������˿�Ƿ񱻼���
		if (is_timeout == 1) {
			val_temp = ad_chn_base[gl_chnn_index].base;
			if (val_temp < 10) {//��˿������
				//��λ
				if ((gl_chnn_index >= 0) && (gl_chnn_index <= 5)) {//��1~��6
					ad_chnn_wire_cut |= ((Uint16)0x01 << (gl_chnn_index + 8));
				} else if ((gl_chnn_index >= 6) && (gl_chnn_index <= 11)) {//��1~��6
					ad_chnn_wire_cut |= ((Uint16)0x01 << (gl_chnn_index - 6));
				}
			}
		}

		//����12����˿�Ƿ����������Լ���̬����-->��1~��6����1~��6��������
		//�������Ƹ��Ƿ��������������Ƹ˲����ھ�̬����			
		for (i = 0; i < 13; i++) {			
			ad_chn_over[i] = ad_chn_over[i] << 1;   //Bit0��0�����ȱʡ������Χ��
			val = ad_chn_sample[i].w;
			if (val <= ad_chn_base[i].base_up) {//��������/��������Χ��
				//a. ���־(ȱʡ)
				//b. ������ٻ�׼ֵ�����
				ad_samp_sum[i].sum += val;
				ad_samp_sum[i].point++;

				if (ad_samp_sum[i].point == 2) {
					//��2��(Լ��0.6��)
					//b.0 ������2���ֵ
					val_temp = ad_samp_sum[i].sum >> 1;   //����2, �õ���2��ľ�ֵ
					//b.1 ���»�׼ֵ
					if (ad_chn_base[i].base > (val_temp + 1)) {
						//����С2, �ڻ����ɳ�
						//ZZX: ��������, ���ٲ�ֵ�� 1/2
						val_temp = (ad_chn_base[i].base - val_temp) >> 1;
						if (ad_chn_base[i].base >= val_temp) {
							ad_chn_base[i].base -= val_temp;
							//ͬ������������
							val_temp = ad_chn_base[i].base;
							ad_chn_base[i].base_down = val_temp / 3;
							if ((1023 - ad_chn_base[i].base) > ad_still_Dup[i]) {
								ad_chn_base[i].base_up = ad_chn_base[i].base + ad_still_Dup[i];
							} else {
								ad_chn_base[i].base_up = 1023;
							}
						}

						//�建���Ž����ٱ���
						md_point[i] = 0;
					} else if (val_temp > (ad_chn_base[i].base + 1)) {
						// ���ٴ�2, �����Ž�
						md_point[i]++;
						if (md_point[i] >= DEF_ModiBASE_PT) {
							// ���������Ž�ʱ��������������, ����һ�θ���
							// 1. ���ٻ�׼ֵ
							if (ad_chn_base[i].base < 1023) {
								//���Ե���1
								ad_chn_base[i].base++;
								// ͬ������������
								val_temp = ad_chn_base[i].base;
								ad_chn_base[i].base_down = val_temp / 3;
								if (ad_chn_base[i].base_up < 1023) {
									ad_chn_base[i].base_up++;
								}
							}
							// 2. �建���Ž����ٱ���
							md_point[i] = 0;
						}
					}

					//b.2 ��λ�׶κͱ��� - ����4��4��ƽ������ͽṹ
					ad_samp_sum[i].sum   = 0;
					ad_samp_sum[i].point = 0;
				}
			} else {
				//��������, �ñ�־
				ad_chn_over[i] |= 0x01;
			}

			if ((ad_chn_over[i] & 0x0F) == 0x0F) {//����4�㳬��Χ����ͨ������������
				if ((i >= 0) && (i <= 5)) {//��1~��6
					//��������Χ���ñ�־
					ad_alarm_exts |= ((Uint16)0x01 << (i + 8));
				} else if ((i >= 6) && (i <= 11)) {//��1~��6
					//��������Χ���ñ�־
					ad_alarm_exts |= ((Uint16)0x01 << (i - 6));
				}

				if (i == 12) {//������
					ad_alarm_exts |= ((Uint16)0x01 << (i + 3));
					ad_alarm_exts |= ((Uint16)0x01 << (i - 5));
				}

				//������ʱtick����
				ad_alarm_tick[i] = 0;

				//�������¾�̬��׼ֵ
				ad_chn_base[i].base = val;
				val_temp = ad_chn_base[i].base;
				ad_chn_base[i].base_down = val_temp / 3;
				if ((1023 - ad_chn_base[i].base) > ad_still_Dup[i]) {
					ad_chn_base[i].base_up = ad_chn_base[i].base + ad_still_Dup[i];
				} else {
					ad_chn_base[i].base_up = 1023;
				}
			} else if ((ad_chn_over[i] & 0x0F) == 0x00) {//����������
				if (ad_alarm_tick[i] > ALARM_TEMPO) {//��鱨��ʱ���Ƿ��ѵ�
					//�����Ѿ�����󱨾�ʱ��, ֹͣ����
					if ((i >= 0) && (i <= 5)) {//��1~��6
						//������Χ��, ���־
						ad_alarm_exts &= (~((Uint16)0x01 << (i + 8)));
					} else if ((i >= 6) && (i <= 11)) {//��1~��6
						//������Χ��, ���־
						ad_alarm_exts &= (~((Uint16)0x01 << (i - 6)));
					}

					if (i == 12) {//������
						ad_alarm_exts &= (~((Uint16)0x01 << (i + 3)));
						ad_alarm_exts &= (~((Uint16)0x01 << (i - 5)));
					}
				}
			}

			//��龲̬�����Ƿ�������Χ��
			//ֻ����6����˿����6����˿����Ҫ�ж��Ƿ�̬����
			if ((i >= 0) && (i <= 11)) {
				check_still_stress(i);
			}		
		}
		
		//�����󿪹������ҿ������Ƿ����������������ھ�̬����
		//��ȡ�󿪹�����ֵ
		if (bLeftSwitch == 0) {//����
			ad_alarm_exts |= (1 << 14);
		} else {//����
			ad_alarm_exts &= (~(1 << 14));
		}
		
		//��ȡ�ҿ�������ֵ
		if (bRightSwitch == 0) {//����
			ad_alarm_exts |= (1 << 6);
		} else {//����
			ad_alarm_exts &= (~(1 << 6));
		}

		//����led״̬
		update_led_status();
		
		//������ϱ�����־
		//�ˣ��󿪹�������6~1������־
		temp = HIGH((ad_alarm_exts | ad_alarm_base)) & HIGH(ad_sensor_mask_LR);
		//�������ϱ���
		if (temp == 0) {
			adl_alarm_flag = 0;   //�ޱ���
		} else {
			adl_alarm_flag = 1;     //�б���
		}

		//�ˣ��ҿ���������6~1������־
		//���Ҳ���ϱ���
		temp = LOW((ad_alarm_exts | ad_alarm_base)) & LOW(ad_sensor_mask_LR);
		if (temp == 0) {
			adr_alarm_flag = 0;   //�ޱ���
		} else {
			adr_alarm_flag = 1;   //�б���
		}

		break;
	}
}

//���ָ��ͨ���ľ�̬�����Ƿ�������Χ��
void check_still_stress(Byte index)
{
	if ((ad_chn_base[index].base >= ad_still_dn) && 
	    (ad_chn_base[index].base <= ad_still_up)) {		
		if ((index >= 0) && (index <= 5)) {//��1~��6
			//������Χ��, ���־
			ad_alarm_base &= (~((Uint16)0x01 << (index + 8)));
			
			//����
			if (ad_alarm_base & 0x0080) {
				test = 1;
			}
		} else if ((index >= 6) && (index <= 11)) {//��1~��6
			//������Χ��, ���־
			ad_alarm_base &= (~((Uint16)0x01 << (index - 6)));
			
			//����
			if (ad_alarm_base & 0x0080) {
				test = 2;
			}
		}
	} else {
		if ((index >= 0) && (index <= 5)) {//��1~��6
			//��������Χ���ñ�־
			ad_alarm_base |= ((Uint16)0x01 << (index + 8));	

			//����
			if (ad_alarm_base & 0x0080) {
				test = 3;
			}
		} else if ((index >= 6) && (index <= 11)) {//��1~��6
			//��������Χ���ñ�־
			ad_alarm_base |= ((Uint16)0x01 << (index - 6));
			
			//����
			if (ad_alarm_base & 0x0080) {
				test = 4;
			}
		}
	}
}

//index��0-11�ֱ��ӦΪ��1 ��1 ��2 ��2 ... ��6 ��6
void motor_adjust(Byte index)
{
	Byte i;
	
	//��UART2�������ҿ���Buffer
	i = uart2_get_send_buffer();
	if (i < UART_QUEUE_NUM) {
		//�ҵ��˿���buffer, д��data
		uart2_send_queue[i].tdata[0] = FRAME_STX;
		uart2_send_queue[i].tdata[1] = 0xFF;       //Ŀ�ĵ�ַ
		uart2_send_queue[i].tdata[2] = 0x01;	   //Դ��ַ
		uart2_send_queue[i].tdata[3] = 0x06;
		uart2_send_queue[i].tdata[4] = CMD_ZL_PRE;
		uart2_send_queue[i].tdata[5] = 0x04;
		uart2_send_queue[i].tdata[6] = 0xF0;
		uart2_send_queue[i].tdata[7] = index;      //�����
		uart2_send_queue[i].tdata[8] = 1;          //��ת
		uart2_send_queue[i].tdata[9] = 10;         //��תʱ��10s

		uart2_send_queue[i].len = 11;
	}
}

void update_led_status(void)
{
	Uint16 temp16;
	
	//LEDָʾ
	temp16 = (ad_alarm_exts | ad_alarm_base);
	//��1~6������־
	P3 = (~(HIGH(temp16) << 2));

	//��1~6������־
	P0 = (~(LOW(temp16) & 0x3F));

	//����������־
	bSelf_Led_Ctrl = (~((ad_alarm_exts >> 7) & 0x0001) & 0x0001);	
}
