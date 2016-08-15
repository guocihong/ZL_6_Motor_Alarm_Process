#include "system_init.h"

/*
ע�����
1����Ҫ���ó���ֵ������Ҫ���òɼ�ֵ����
2�����Թ�������Ҫ����Ƹ˵�·�����
3���������
	a������led�Ƿ�����
	b�����Բ��뿪���Ƿ�����
	c�����Լ̵����Ƿ�����
	d������uart2�Ƿ�����
	e��������·RS485�Ƿ�����
	f�� �����󿪹������ҿ������Ƿ�����
	g���������ó���ֵ�Ƿ�����
*/

void main(void)
{
    //ϵͳ��ʼ��
    system_init();

    //�����ж�
    Enable_interrupt();

    //ʹ�ÿ��Ź�
    Wdt_enable();// 2.276s 

    while(1) {       
        //ϵͳ״̬����
        status_task();

        //����uart���յ�����
        uart_task();
        
        //ι��
        Wdt_refresh();
		
		//adc_task
		adc_task();
				
        //ι��
        Wdt_refresh();
		
        //�ŴŴ���
        doorkeep_task();

        //��������
        alarm_task();

        //ι��
        Wdt_refresh();
    }
}