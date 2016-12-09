#ifndef _CONFIG_H_
#define _CONFIG_H_

#include "STC15.h"
#include "compiler.h"
#include "stress.h"
#include <intrins.h>
#include <string.h>

/* Scheduler Tick */
#define SCHEDULER_TICK     5                        // unit is ms

#define ALARM_TEMPO        (3000/SCHEDULER_TICK)    //�����źų���ʱ��

/* System status */
#define SYS_PowerON        0                        // 0 - ��ʼ�ϵ�
#define SYS_B5S            1                        // 1 - ��׼ֵ����ǰ��ʱ(Լ5��)
#define SYS_SAMP_BASE      2                        // 2 - ��׼ֵ����(10������)
#define SYS_SELF_CHECK1    3                        // 3 - �����Լ�׶�1
#define SYS_SELF_CHECK2    4                        // 4 - �����Լ�׶�2
#define SYS_CHECK          5                        // 5 - ʵʱ���

/* AD */
typedef struct strAD_Sum
{ //����ֵ�ۼӺ�
  Uint16   sum;                             //�ۼƺ� (����64��,�������)
  Uint8    point;                           //�Ѳ�������
}sAD_Sum;

typedef struct strAD_BASE
{ //ϵͳ����ʱ��̬��׼ֵ��Ӧ�Ĳ���ֵ
  Uint16   base;                            //��̬��׼ֵ
  Uint16   base_down;                       //��׼ֵ����(��)
  Uint16   base_up;                         //��׼ֵ����(��)
}sAD_BASE;

//for Uart
#define	FRAME_STX         0x16                     // Frame header
#define	MAX_RecvFrame     50                       // ���ջ�������С
#define	MAX_TransFrame    50                       // ���ͻ�������С
#define RECV_TIMEOUT       4                       // �ֽڼ�����ʱ����, ��λΪtick
                                            // ��Сֵ����Ϊ1,���Ϊ0���ʾ�����г�ʱ�ж�                                    
/* state constant(�����ڽ���) */                    
#define FSA_INIT            0                       //�ȴ�֡ͷ
#define FSA_ADDR_D          1                       //�ȴ�Ŀ�ĵ�ַ
#define FSA_ADDR_S          2                       //�ȴ�Դ��ַ
#define FSA_LENGTH          3                       //�ȴ������ֽ�
#define FSA_DATA            4                       //�ȴ����(���� ����ID �� ����)
#define FSA_CHKSUM          5                       //�ȴ�У���

/* Uart Queue */
typedef struct strUART_Q
{
  Byte  flag;                               //״̬�� 0 - ���У� 1 - �ȴ����ͣ� 2 - ���ڷ���; 3 - �ѷ��ͣ��ȴ�Ӧ��
  Byte  tdata[MAX_TransFrame];              //���ݰ�(���һ��У���ֽڿ��Բ���ǰ���㣬���ڷ���ʱ�߷��ͱ߼���)
  Byte  len;					            //���ݰ���Ч����(��У���ֽ�)
  Byte  package_type;                       //0-������λ�������ݰ�;1-�豸��������ݰ�
}sUART_Q;

#define UART_QUEUE_NUM      7                       //UART ������
           
#define bLeftSwitch         P10                     //׼˫��ڣ��󿪹�����0-������1-����
#define bRightSwitch        P11                     //׼˫��ڣ��ҿ�������0-������1-����                   
#define bSelf_Led_Ctrl      P15                     //׼˫��ڣ�����������1-��������0-����
#define bRS485_1_Ctrl       P40                     //���������RS485����ʹ��: 1-������; 0 - ��ֹ����(����)
#define bRS485_2_Ctrl       P41                     //���������RS485����ʹ��: 1-������; 0 - ��ֹ����(����)
#define bBeep_Ctrl          P42                     //���������Beep����:  1-����; 0-����
#define bDoorKeeper         P43                     //�������룬�Ŵż��: 0-�ŴŹر�; 1-�ŴŴ򿪣�Ӧ������
#define bRelay_Left_Ctrl    P44		              //���������(���)�������1��1-�̵����ӵ�����(�ϵ�ȱʡ)�� 0-���ӵ�
#define bRelay_Right_Ctrl   P45		              //���������(�Ҳ�)�������2��1-�̵����ӵ�����(�ϵ�ȱʡ)�� 0-���ӵ�
#define bAdjustButton       P55                     //�������룬1-���ڿ��ضϿ�; 0-���ڿ��رպ�

/* interrupt enable */
#define Enable_interrupt()  (EA = 1)
#define Disable_interrupt() (EA = 0)

#endif