#include "system_init.h"

static void gpio_init(void);
static void get_defence_info(void);
static void get_config_info(void);

/* System */
extern  data  Byte        gl_comm_addr;                //��ģ��485ͨ�ŵ�ַ

/* for beep */
extern xdata  Uint16      beep_during_temp;            // Ԥ���һ�η�������ʱ��, ��λ:tick 

/* AD sample */
extern xdata  Uint16      ad_sensor_mask_LR;           //������˳���������sensor mask: �ˡ��󿪹�������6 ~ 1,�ˡ��ҿ���������6 ~ 1
extern xdata  Uint16      ad_sensor_mask;              //15  14  13  12  11  10  9  8  7   6   5  4  3  2  1  0
											           //				��6			      ��1 ��6       	 ��1	

extern xdata  Uint16      ad_still_dn;                 //��̬����ֵ����
extern xdata  Uint16      ad_still_up;                 //��̬����ֵ����
extern xdata  Byte        ad_still_Dup[13];            //������ֵ����

void system_init(void)
{   
	gpio_init();
        
	get_defence_info();
	get_config_info();

	status_task_init();
	uart_task_init();
	adc_task_init();
	doorkeep_task_init();
	alarm_task_init();
	
	timer0_init();
}


static void gpio_init(void)
{    
    Uint32 i;
    
    CLK_DIV = 0x00;       //ϵͳʱ�Ӷ���ʱ�Ӳ���Ƶ����ʱ�������ⲿ����
    IAP_CONTR &= 0xBF;    //���Ź���λ��ϵͳ���û�������0000H����ʼִ���û�����
		
	//����P0��Ϊ׼˫���
	P0M1 = 0x00;
	P0M0 = 0x00;

	//����P10~P15Ϊ׼˫���
	P1M1 = 0x00;
	P1M0 = 0x00;

	//����P2��Ϊ׼˫���-->��������Ϊ�������룬��������Ϊ׼˫��ڣ���Ȼ���뿪�ص�ֵ����������
	P2M1 = 0x00;
	P2M0 = 0x00;

	//����P3��Ϊ׼˫���
	P3M1 = 0x00;
	P3M0 = 0x00;

	//����P40,P41,P42,P44,P45Ϊ�������,P43Ϊ��������,P46,P47Ϊ׼˫���
	P4M1 = 0x08;
	P4M0 = 0x37;
		
	//����P50,P51,P52,P53Ϊ׼˫���,P55Ϊ��������
	P5M1 = 0x20;
	P5M0 = 0x00;
    
	//��ʼ������
	bRS485_1_Ctrl     = 0;//1#RS485���ͽ�ֹ�����ڽ���״̬��
	bRS485_2_Ctrl     = 0;//2#RS485���ͽ�ֹ�����ڽ���״̬��
	bBeep_Ctrl        = 0;//���Ž�ֹ
	bRelay_Left_Ctrl  = 1;//1#�������ϵ�պ�
	bRelay_Right_Ctrl = 1;//2#�������ϵ�պ�
	bSelf_Led_Ctrl    = 1;//����������
	
	//��һ��19��LED -- ȫ�����������ڼ������LED�Ƿ����
	P3  = 0x00;   //����19��LED
	P0  = 0xC0;
	P15 = 0;
	P50 = P51 = P52 = P53 = P46 = P47 = 0;
    
	i = 150000;
	while (i>0)  i--;
	
	P3  = 0xFF;   //Ϩ��19��LED
	P0  = 0xFF;
	P15 = 1;
	P50 = P51 = P52 = P53 = P46 = P47 = 1;

	//��ʱ
	i = 150000;
	while (i>0)  i--;
}

/*1.��ȡRS485ͨ�ŵ�ַ  2.������������mask */
static void get_defence_info(void)
{
	Uint16 i;
	Byte   j;
	Byte   temp;
	Byte   left_sensor_mask;
	Byte   right_sensor_mask;
	Byte   other_sensor_mask;

	//��ȡRS485ͨ�ŵ�ַ
	i = 0;
	temp = P2;
	do {
		//��ʱ
		j = 255;
		while (j>0)  j--;
		if (temp == P2) { //������ͬ��ֵ
			i++;
		} else { //������ͬ��ֵ
			i = 0;
			temp = P2;
		}
	} while (i < 8);
	gl_comm_addr = ~temp;    //��ģ��RS485ͨ�ŵ�ַ
	//ʹ��8λ���뿪�أ���ַ��ΧΪ 0x00 ~ 0xFF,�����豸��ַҪ���0x10��ʼ
	if ((gl_comm_addr < 0x10) || (gl_comm_addr == 0xFF) || (gl_comm_addr == 0xFE)) {
		//�豸��ַ����Ϊ 0x10 ~ 0xFD ֮��(��),0xFFΪ�㲥��ַ,0xFE��ʾδ�����ַ
		gl_comm_addr = CMD_ADDR_UNSOLV;
	}

	//������������mask
	//��������,�ҿ�����,�󿪹���mask
	i = 0;
	temp = (P1 >> 2) & 0x07;
	do {
		//��ʱ
		j = 255;
		while (j > 0)  j--;
		if (temp == ((P1 >> 2) & 0x07)) {	//������ͬ��ֵ
			i++;
		} else {	//������ͬ��ֵ
			i = 0;
			temp = (P1 >> 2) & 0x07;
		}
	} while (i < 8);
	other_sensor_mask = ~temp & 0x07;
	
	//��12����˿mask
	//����6����˿����mask
	P06 = 0; //��ʱmaskλ������������Ӧ��LED
	i = 0;
	temp = (P3 >> 2) & 0x3F;
	do {
		//��ʱ
		j = 255;
		while (j > 0)  j--;
		if (temp == ((P3 >> 2) & 0x3F)) {	//������ͬ��ֵ
			i++;
		} else {	//������ͬ��ֵ
			i = 0;
			temp = (P3 >> 2) & 0x3F;
		}
	} while (i < 6500);
	left_sensor_mask = ~temp & 0x3F;
	
	//����6����˿����mask
	P07 = 0;  //��ʱmaskλ������������Ӧ��LED
	i = 0;
	temp = P0 & 0x3F;
	do {
		//��ʱ
		j = 255;
		while (j > 0)  j--;
		if (temp == (P0 & 0x3F)) {	//������ͬ��ֵ
			i++;
		} else {	//������ͬ��ֵ
			i = 0;
			temp = P0 & 0x3F;
		}
	} while (i < 6500);
	right_sensor_mask = ~temp & 0x3F;

	//��ֹ���뿪�����룬������LED
	P06 = 1; 
	P07 = 1;

	//���� ad_sensor_mask_LR
	ad_sensor_mask_LR = (((other_sensor_mask >> 2) & 0x01) << 15) | //��
						(((other_sensor_mask >> 0) & 0x01) << 14) | //�󿪹���
						(((left_sensor_mask  >> 0) & 0x3F) << 8)  | //��6~��1
						(((other_sensor_mask >> 2) & 0x01) << 7)  | //��
						(((other_sensor_mask >> 1) & 0x01) << 6)  | //�ҿ�����
						(((right_sensor_mask >> 0) & 0x3F) << 0);   //��6~��1
	
	//����ad_sensor_mask
	ad_sensor_mask = (((right_sensor_mask >> 0) & 0x3F) << 6) |     //��6~��1 
		             (((left_sensor_mask  >> 0) & 0x3F) << 0);      //��6~��1
}

//��ȡϵͳԤ������
static void get_config_info(void)
{
	Byte temp;
	Byte j;
	
	//ʹ��Flash����
	flash_enable();
	
	//����̬����ֵ��Χ
	temp = flash_read(EEPROM_SECTOR3);
	if (temp == 0x5A) { //����Ч����
		//����
		temp = flash_read(EEPROM_SECTOR3 + 1);
		ad_still_dn = (Uint16)temp << 8;
		temp = flash_read(EEPROM_SECTOR3 + 2);
		ad_still_dn += temp;
		//����
		temp = flash_read(EEPROM_SECTOR3 + 3);
		ad_still_up = (Uint16)temp << 8;
		temp = flash_read(EEPROM_SECTOR3 + 4);
		ad_still_up += temp;
		//��Ч��?
		if ((ad_still_dn < STD_STILL_DN) ||
		        (ad_still_up > STD_STILL_UP) ||
		        (ad_still_dn >= ad_still_up)) { //�޺Ϸ����ݣ�ȡȱʡֵ
			ad_still_dn = STD_STILL_DN;
			ad_still_up = STD_STILL_UP;
		}
	} else {	//����Ч���ã�ȡȱʡֵ
		ad_still_dn = STD_STILL_DN;
		ad_still_up = STD_STILL_UP;
	}

	//��������ֵ����
	temp = flash_read(EEPROM_SECTOR4);
	if (temp == 0x5A) { //����Ч����
		//��1~6
		for (j = 0; j < 6; j++) {
			ad_still_Dup[j] = flash_read(EEPROM_SECTOR4 + 2 + j);
		}
		
		//��1~6
		for (j = 0; j < 6; j++) {
			ad_still_Dup[6 + j] = flash_read(EEPROM_SECTOR4 + 8 + j);
		}
		
        //������
        ad_still_Dup[12] = flash_read(EEPROM_SECTOR4 + 15);
        
		//�Ƿ���Ч��
		for (j = 0; j < 13; j++) {
			if ((ad_still_Dup[j] < STD_ALARM_MIN) || (ad_still_Dup[j] > STD_ALARM_MAX))
				ad_still_Dup[j] = STD_ALARM_DEF;    //�޺Ϸ����ݣ�ȡȱʡֵ
		}
	} else {	//����Ч���ã�ȡȱʡֵ
		for (j = 0; j < 13; j++) {
			ad_still_Dup[j] = STD_ALARM_DEF;
		}
	}

	//�����ⱨ��ʱ������
	temp = flash_read(EEPROM_SECTOR5);
	if (temp == 0x5A) { //����Ч����
		temp = flash_read(EEPROM_SECTOR5 + 1);
		beep_during_temp = (Uint16)(((Uint32)temp * 1000) / SCHEDULER_TICK);
	} else {	//ȡȱʡֵ
		beep_during_temp = 0;   //��λ�� tick
	}

	//��ֹFlash����
	flash_disable();
}
