#include "task/motor/motor_status_task.h"

#define MT_CONFORM_TEMP1    (30 / SCHEDULER_TICK)  //���ȷ����ʱ1
#define MT_CONFORM_TEMP2    (200 / SCHEDULER_TICK) //���ȷ����ʱ2

/* ����״̬ */
#define MT_READ_START    0
#define MT_READ_IDLE     1
#define MT_READ_DELAY1   2
#define MT_READ_DELAY2   3

/* for system */
extern idata  Byte     system_status;                 //ϵͳ״̬

/* �����ת��� */
extern xdata  Byte     gl_motor_overcur_tick;         //�����ת����ʱtick
extern bdata  bit      gl_motor_adjust_flag;          //����Ƿ��ڹ���״̬��1-�����ڹ���״̬; 0-ֹͣ����״̬
extern bdata  bit      gl_motor_overcur_flag;         //�����ת��־��0-����������1-�����ת
static xdata  Byte     bt_read_state;                 //task state

void motor_status_task_init(void)
{
	gl_motor_overcur_tick = 0;
	gl_motor_overcur_flag = 0;                        //�ϵ�ʱ��ȱʡΪ��������
	bt_read_state         = MT_READ_START;
}

void motor_status_task(void)
{
	switch (bt_read_state)
	{
	case MT_READ_START: //��ʼ����
		if (system_status == SYS_SELF_CHECK) {
			//ϵͳ�����Լ�׶βſ�ʼ���
			bt_read_state = MT_READ_IDLE;
		}
		break;

	case MT_READ_IDLE:
		if (gl_motor_adjust_flag == 1) {//ֻ�е������ʱ���ż�����Ƿ��ת
			if (bMotorOverCur == 0) {//������ܱ���ת��������ʱȷ�Ͻ׶�
				gl_motor_overcur_tick = 0;
				bt_read_state = MT_READ_DELAY1;
			}
		}
		break;

	case MT_READ_DELAY1://��ʱȷ��
		if (gl_motor_overcur_tick > MT_CONFORM_TEMP1) {
			// ��ʱʱ�䵽��ǰ30ms�͵�ƽ���账����Ϊ����ë�̸���
			bt_read_state = MT_READ_DELAY2;
			gl_motor_overcur_tick = 0;
		}
		break;
		
	case MT_READ_DELAY2://��ʱȷ��
		if (gl_motor_overcur_tick > MT_CONFORM_TEMP2) {
			// ��ʱʱ�䵽, �ж��Ƿ����ȶ��ı仯
			if (bMotorOverCur == 0) {//���ȷʵΪ��ת�������ֹͣ����
				gl_motor_overcur_flag = 1;
			}
			
			//��λ
			bt_read_state = MT_READ_IDLE;
		}
		break;
	}
}