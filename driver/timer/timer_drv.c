#include "driver/timer/timer_drv.h"
#include "driver/flash/flash_drv.h"

/* UART2 */
extern xdata  Byte        recv2_state;                 //receive state
extern xdata  Byte        recv2_timer;                 //receive time-out, �����ֽڼ䳬ʱ�ж�

/* UART3 */
extern xdata  Byte        recv3_state;                 //receive state
extern xdata  Byte        recv3_timer;                 //receive time-out, �����ֽڼ䳬ʱ�ж�

/* UART4 */
extern xdata  Byte        recv4_state;                 //receive state
extern xdata  Byte        recv4_timer;                 //receive time-out, �����ֽڼ䳬ʱ�ж�

/* Doorkeep(�Ŵ�) */
extern xdata  Byte        gl_local_dk_tick;            //��ˮ����Ŵż���ʱtick  

/* ϵͳ��ʱ */
extern xdata  Uint16      gl_delay_tick;               //ͨ����ʱ��tick
extern xdata  Uint16      gl_ack_tick;	               //Ӧ����ʱ��ʱ tick

/* for beep */
extern bdata  bit         beep_flag;                   //������־: 0 - ����; 1 - ���ڷ���
extern data   Uint16      beep_timer;                  //��ʱ��������ʣ��ʱ��, ��λ:tick

/* for alarm output (�̵�����LED) */
extern bdata  bit         alarm1_flag;                 //���������
extern bdata  bit         alarm2_flag;	               //�ҷ�������
extern bdata  bit         alarm3_flag;	               //��������������
extern bdata  bit         alarm4_flag;                 //�ҿ�������������	 
extern bdata  bit         alarm5_flag;	               //�󿪹�����������

extern data   Uint16      alarm1_timer;                //��ʱ����������ѱ���ʱ��,��λtick 
extern data   Uint16      alarm2_timer;                //��ʱ�����ҷ����ѱ���ʱ��,��λtick
extern data   Uint16      alarm3_timer;                //��ʱ�������������������ѱ���ʱ��,��λ:tick 
extern data   Uint16      alarm4_timer;                //��ʱ�����ҿ��������������ѱ���ʱ��,��λ:tick 
extern data   Uint16      alarm5_timer;                //��ʱ�����󿪹������������ѱ���ʱ��,��λ:tick 
 
/* variables for alarm output */ 
extern xdata  Uint16      ad_alarm_tick[13];           //��ͨ��������ʱtick

/* �����ת��־ */
extern xdata  Byte        gl_wait_delay_tick;          //�������Ƶ��������Ժ���ʱ�ȴ���ʱ�䣬Ȼ���ͬ�����»�׼ֵ�Լ��Ƿ�����ת��ʱ���Ƿ�����
extern bdata  bit         is_sample_clear;             //����ֵ�Ƿ�����:0-û�����㣻1-�Ѿ�����
extern xdata  Uint16      check_sample_clear_tick;     //����������ֵ�Ƿ�����ɹ���ʱtick

/* for AD */
extern xdata  Uint16      ad_chn_sample[13];           //����һ�ֲ���ֵ���Ѿ���ȥ������ÿͨ��һ���㣬ѭ�����棩

void timer0_init(void)   // 5ms@22.1184MHz
{    
    // ��ʱ��0��ʼ��	
	AUXR &= 0x7F;		//��ʱ��ʱ��12Tģʽ
	TMOD &= 0xF0;		//���ö�ʱ��0����ģʽΪ��ʽ0��16λ�Զ���װ��ʱ��
	TL0 = 0x00;		    //���ö�ʱ��ֵ
	TH0 = 0xDC;		    //���ö�ʱ��ֵ
	TF0 = 0;		    //���TF0��־
	ET0 = 1;            //ʹ��T0�ж�����λ
	PT0 = 0;            //�����ж����ȼ�Ϊ���ȼ�1
	TR0 = 1;		    //��ʱ��0��ʼ��ʱ
}

void timer0_isr(void) interrupt TIMER0_VECTOR
{
	Byte i;
	Uint16 sum;
    
	// increment task tick counters
	gl_local_dk_tick++;                           //��ˮ����Ŵż���ʱtick
    
    if (gl_delay_tick > 0) {
    	gl_delay_tick--;                          //ͨ����ʱ��tick
    }
    
	if (gl_ack_tick > 0) {
		gl_ack_tick--;                            //Ӧ����ʱ��ʱ
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
        
        if (check_sample_clear_tick == 0) {//ʱ�䵽��������ֵ�����Ƿ�ɹ�
            sum = 0;
            for (i = 0; i < 8; i++) {
                sum += ad_chn_sample[i];
            }
            
            if (sum < 100) {//����ֵ����ɹ�
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
		//����beep
		if (beep_timer > 0) {
			beep_timer--;
		}
			
		if (beep_timer == 0) { 
			//����ʱ�䵽������
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
        
	// UART2�ֽ�֮����ճ�ʱ
	if (recv2_state != FSA_INIT) { 
		//�ǳ�ʼ״̬����Ҫ����Ƿ�ʱ
		if (recv2_timer > 0) {
			recv2_timer--;
		}
		
		if (recv2_timer == 0) {
			recv2_state = FSA_INIT;   //���ճ�ʱ, �ָ�����ʼ״̬			
		}
	}
	
	// UART3�ֽ�֮����ճ�ʱ
	if (recv3_state != FSA_INIT) {
		//�ǳ�ʼ״̬����Ҫ����Ƿ�ʱ
		if (recv3_timer > 0) {
			recv3_timer--;
		}
		
		if (recv3_timer == 0){
			recv3_state = FSA_INIT;   //���ճ�ʱ, �ָ�����ʼ״̬
		}
	}
	
	// UART4�ֽ�֮����ճ�ʱ
	if (recv4_state != FSA_INIT) { 
		//�ǳ�ʼ״̬����Ҫ����Ƿ�ʱ
		if (recv4_timer > 0) {
			recv4_timer--;
		}
		
		if (recv4_timer == 0) {
			recv4_state = FSA_INIT;   //���ճ�ʱ, �ָ�����ʼ״̬			
		}
	}
}