#include "task/alarm/alarm_task.h"

/* for beep */
extern bdata  bit     beep_flag;        // ������־ : 0 - ����; 1 - ���ڷ���
extern  data  Uint16  beep_timer;       // ʣ�����ʱ��, ��λtick
extern xdata  Uint16  beep_during_temp; // ����������ʱ��

/* for alarm */
extern bdata  Byte    alarm_out_flag;   //���������־��λֵ0 - �ޱ������̵����ϵ����ϣ�;  λֵ1 - ����(�ϵ�)
										//λַ76543210  ��Ӧ  X X X ����2 ����1 ������� X X
										//ZZX: �������λֵ��ʵ��Ӳ�����ƽŵ�ƽ�෴; 	�������λֵ��ʵ��Ӳ�����ƽŵ�ƽ��ͬ

extern bdata  bit     alarm1_flag;
extern bdata  bit     alarm2_flag;
extern  data  Uint16  alarm1_timer;     // ��ʱ����������1�ѱ���ʱ��,��λtick
extern  data  Uint16  alarm2_timer;     // ��ʱ����������2�ѱ���ʱ��,��λtick

/* variables for ���� */
extern bdata  bit     adl_alarm_flag;   //���������ϱ�����־: 0 - �ޱ���; 1 - ����ֵ������
extern bdata  bit     adr_alarm_flag;   //�Ҳ�������ϱ�����־: 0 - �ޱ���; 1 - ����ֵ������
                                        //����ԭ�����Ϊ������������ ��̬��������Χ����

/* Doorkeep */
extern bdata  bit     gl_local_dk_status;          //�Ŵſ���״̬: 1 - �պ�; 0 - ��(��Ҫ����)
extern bdata  bit     gl_control_dk_status;        //���Ƹ��Ŵ�״̬�� 1 - �պ�; 0 - ��(��Ҫ����)   

/* for system */
extern  data  Byte    system_status;               //ϵͳ״̬

void alarm_task_init(void)
{
	/* for beep */
	beep_flag = 0;       //�޷���

	/* for alarm */
	alarm_out_flag = 0x00;  //������1/2���ޱ������

	adl_alarm_flag = 0;  //��ʼ�ϵ���������ϱ���
	adr_alarm_flag = 0;  //��ʼ�ϵ���Ҳ�����ϱ���
}

void alarm_task(void)
{
	Uint16 temp16;
	
	//�Ŵű��򿪣����ҷ���������,��·���������Ͽ�
	if ((gl_local_dk_status || !gl_control_dk_status) && (system_status >= SYS_SELF_CHECK1)) {	
		Disable_interrupt();
		alarm1_timer = 0;     //�屨����1�ѱ���ʱ��(���ٱ���3��)
		Enable_interrupt();
		
		Disable_interrupt();
		alarm2_timer = 0;     //�屨����2�ѱ���ʱ��(���ٱ���3��)
		Enable_interrupt();

		if (alarm1_flag == 0) {
			//�±���
			bRelay_Left_Ctrl = 0;	   //����
			alarm1_flag = 1;   //������1����
			
			//beep
			if (beep_during_temp > 0) {
				//����beep
				Disable_interrupt();
				beep_timer = beep_during_temp;
				Enable_interrupt();
				if (!beep_flag) {
					//���� -> ����
					bBeep_Ctrl = 1;
					beep_flag = 1;
				}
			}	
		}
		
		if (alarm2_flag == 0) { 
			//�±���
			bRelay_Right_Ctrl = 0;	   //����
			alarm2_flag = 1;   //������2����
		}	
	}
  
	//������1����ࣩ
	if (adl_alarm_flag && (system_status >= SYS_SELF_CHECK1)) {
		//��������쳣: �±������������
		Disable_interrupt();
		alarm1_timer = 0;     //�屨����1�ѱ���ʱ��(���ٱ���3��)
		Enable_interrupt();
		if (alarm1_flag == 0) { //�±���
			bRelay_Left_Ctrl = 0;	   //����
			alarm1_flag = 1;   //������1����
		}
        
        //beep
        if (beep_flag == 0) {//����
			if (beep_during_temp > 0) {	//����beep
				Disable_interrupt();
				beep_timer = beep_during_temp;
				Enable_interrupt();
				
				bBeep_Ctrl = 1;//����
				beep_flag = 1;
			}
		}
	}

	//������2���Ҳࣩ
	if (adr_alarm_flag && (system_status >= SYS_SELF_CHECK1)) {	
		//�Ҳ������쳣: �±������������
		Disable_interrupt();
		alarm2_timer = 0;    //�屨����2�ѱ���ʱ��(���ٱ���3��)
		Enable_interrupt();
		if (alarm2_flag == 0) { 
			//�±���
			bRelay_Right_Ctrl = 0;	   //����
			alarm2_flag = 1;   //������2����
		}
		
		//beep
        if (beep_flag == 0) {//����
			if (beep_during_temp > 0) {	//����beep
				Disable_interrupt();
				beep_timer = beep_during_temp;
				Enable_interrupt();
				
				bBeep_Ctrl = 1;//����
				beep_flag = 1;
			}
		}
	}

	//��鱨����1����ʱ���Ƿ��ѵ�
	if (alarm1_flag) { 
		//������1���ڱ���
		Disable_interrupt();
		temp16 = alarm1_timer;
		Enable_interrupt();
		if (temp16 > ALARM_TEMPO) { 
			//������1�Ѿ�����󱨾�ʱ��, ֹͣ����
			alarm1_flag = 0;
			bRelay_Left_Ctrl = 1;
		}
	}

	//��鱨����2����ʱ���Ƿ��ѵ�
	if (alarm2_flag) {	
		//������2���ڱ���
		Disable_interrupt();
		temp16 = alarm2_timer;
		Enable_interrupt();
		if (temp16 > ALARM_TEMPO) { 
			//������2�Ѿ�����󱨾�ʱ��, ֹͣ����
			alarm2_flag = 0;
			bRelay_Right_Ctrl = 1;
		}
	}
}