#include "system_init.h"

static void gpio_init(void);
static void get_defence_info(void);
static void get_config_info(void);

/* System */
extern  data  Byte        gl_comm_addr;                //本模块485通信地址

/* for beep */
extern xdata  Uint16      beep_during_temp;            // 预设的一次蜂鸣持续时间, 单位:tick 

/* AD sample */
extern xdata  Uint16      ad_sensor_mask_LR;           //按左右顺序重排序的sensor mask: 杆、左开关量、左6 ~ 1,杆、右开关量、右6 ~ 1
extern xdata  Uint16      ad_sensor_mask;              //15  14  13  12  11  10  9  8  7   6   5  4  3  2  1  0
											           //				右6			      右1 左6       	 左1	

extern xdata  Uint16      ad_still_dn;                 //静态拉力值下限
extern xdata  Uint16      ad_still_up;                 //静态拉力值上限
extern xdata  Byte        ad_still_Dup[13];            //报警阀值上限

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
    
    CLK_DIV = 0x00;       //系统时钟对主时钟不分频，主时钟来自外部晶振
    IAP_CONTR &= 0xBF;    //看门狗复位后，系统从用户程序区0000H处开始执行用户程序
		
	//设置P0口为准双向口
	P0M1 = 0x00;
	P0M0 = 0x00;

	//设置P10~P15为准双向口
	P1M1 = 0x00;
	P1M0 = 0x00;

	//设置P2口为准双向口-->不能设置为高阻输入，必须设置为准双向口，不然拨码开关的值不能正常读
	P2M1 = 0x00;
	P2M0 = 0x00;

	//设置P3口为准双向口
	P3M1 = 0x00;
	P3M0 = 0x00;

	//设置P40,P41,P42,P44,P45为推挽输出,P43为高阻输入,P46,P47为准双向口
	P4M1 = 0x08;
	P4M0 = 0x37;
		
	//设置P50,P51,P52,P53为准双向口,P55为高阻输入
	P5M1 = 0x20;
	P5M0 = 0x00;
    
	//初始化设置
	bRS485_1_Ctrl     = 0;//1#RS485发送禁止（处于接收状态）
	bRS485_2_Ctrl     = 0;//2#RS485发送禁止（处于接收状态）
	bBeep_Ctrl        = 0;//警号禁止
	bRelay_Left_Ctrl  = 1;//1#开关量上电闭合
	bRelay_Right_Ctrl = 1;//2#开关量上电闭合
	bSelf_Led_Ctrl    = 1;//杆自身不报警
	
	//闪一下19个LED -- 全部点亮，用于检测所有LED是否完好
	P3  = 0x00;   //点亮19个LED
	P0  = 0xC0;
	P15 = 0;
	P50 = P51 = P52 = P53 = P46 = P47 = 0;
    
	i = 150000;
	while (i>0)  i--;
	
	P3  = 0xFF;   //熄灭19个LED
	P0  = 0xFF;
	P15 = 1;
	P50 = P51 = P52 = P53 = P46 = P47 = 1;

	//延时
	i = 150000;
	while (i>0)  i--;
}

/*1.读取RS485通信地址  2.读张力传感器mask */
static void get_defence_info(void)
{
	Uint16 i;
	Byte   j;
	Byte   temp;
	Byte   left_sensor_mask;
	Byte   right_sensor_mask;
	Byte   other_sensor_mask;

	//读取RS485通信地址
	i = 0;
	temp = P2;
	do {
		//延时
		j = 255;
		while (j>0)  j--;
		if (temp == P2) { //读到相同的值
			i++;
		} else { //读到不同的值
			i = 0;
			temp = P2;
		}
	} while (i < 8);
	gl_comm_addr = ~temp;    //本模块RS485通信地址
	//使用8位拨码开关，地址范围为 0x00 ~ 0xFF,总线设备地址要求从0x10开始
	if ((gl_comm_addr < 0x10) || (gl_comm_addr == 0xFF) || (gl_comm_addr == 0xFE)) {
		//设备地址必须为 0x10 ~ 0xFD 之间(含),0xFF为广播地址,0xFE表示未定义地址
		gl_comm_addr = CMD_ADDR_UNSOLV;
	}

	//读张力传感器mask
	//读杆自身,右开关量,左开关量mask
	i = 0;
	temp = (P1 >> 2) & 0x07;
	do {
		//延时
		j = 255;
		while (j > 0)  j--;
		if (temp == ((P1 >> 2) & 0x07)) {	//读到相同的值
			i++;
		} else {	//读到不同的值
			i = 0;
			temp = (P1 >> 2) & 0x07;
		}
	} while (i < 8);
	other_sensor_mask = ~temp & 0x07;
	
	//读12道钢丝mask
	//读左6道钢丝拨码mask
	P06 = 0; //此时mask位会驱动点亮对应的LED
	i = 0;
	temp = (P3 >> 2) & 0x3F;
	do {
		//延时
		j = 255;
		while (j > 0)  j--;
		if (temp == ((P3 >> 2) & 0x3F)) {	//读到相同的值
			i++;
		} else {	//读到不同的值
			i = 0;
			temp = (P3 >> 2) & 0x3F;
		}
	} while (i < 6500);
	left_sensor_mask = ~temp & 0x3F;
	
	//读右6道钢丝拨码mask
	P07 = 0;  //此时mask位会驱动点亮对应的LED
	i = 0;
	temp = P0 & 0x3F;
	do {
		//延时
		j = 255;
		while (j > 0)  j--;
		if (temp == (P0 & 0x3F)) {	//读到相同的值
			i++;
		} else {	//读到不同的值
			i = 0;
			temp = P0 & 0x3F;
		}
	} while (i < 6500);
	right_sensor_mask = ~temp & 0x3F;

	//禁止拨码开关输入，允许报警LED
	P06 = 1; 
	P07 = 1;

	//计算 ad_sensor_mask_LR
	ad_sensor_mask_LR = (((other_sensor_mask >> 2) & 0x01) << 15) | //杆
						(((other_sensor_mask >> 0) & 0x01) << 14) | //左开关量
						(((left_sensor_mask  >> 0) & 0x3F) << 8)  | //左6~左1
						(((other_sensor_mask >> 2) & 0x01) << 7)  | //杆
						(((other_sensor_mask >> 1) & 0x01) << 6)  | //右开关量
						(((right_sensor_mask >> 0) & 0x3F) << 0);   //右6~右1
	
	//计算ad_sensor_mask
	ad_sensor_mask = (((right_sensor_mask >> 0) & 0x3F) << 6) |     //右6~右1 
		             (((left_sensor_mask  >> 0) & 0x3F) << 0);      //左6~左1
}

//读取系统预设数据
static void get_config_info(void)
{
	Byte temp;
	Byte j;
	
	//使能Flash访问
	flash_enable();
	
	//读静态张力值范围
	temp = flash_read(EEPROM_SECTOR3);
	if (temp == 0x5A) { //有有效设置
		//下限
		temp = flash_read(EEPROM_SECTOR3 + 1);
		ad_still_dn = (Uint16)temp << 8;
		temp = flash_read(EEPROM_SECTOR3 + 2);
		ad_still_dn += temp;
		//上限
		temp = flash_read(EEPROM_SECTOR3 + 3);
		ad_still_up = (Uint16)temp << 8;
		temp = flash_read(EEPROM_SECTOR3 + 4);
		ad_still_up += temp;
		//有效否?
		if ((ad_still_dn < STD_STILL_DN) ||
		        (ad_still_up > STD_STILL_UP) ||
		        (ad_still_dn >= ad_still_up)) { //无合法数据，取缺省值
			ad_still_dn = STD_STILL_DN;
			ad_still_up = STD_STILL_UP;
		}
	} else {	//无有效设置，取缺省值
		ad_still_dn = STD_STILL_DN;
		ad_still_up = STD_STILL_UP;
	}

	//读报警阀值参数
	temp = flash_read(EEPROM_SECTOR4);
	if (temp == 0x5A) { //有有效设置
		//左1~6
		for (j = 0; j < 6; j++) {
			ad_still_Dup[j] = flash_read(EEPROM_SECTOR4 + 2 + j);
		}
		
		//右1~6
		for (j = 0; j < 6; j++) {
			ad_still_Dup[6 + j] = flash_read(EEPROM_SECTOR4 + 8 + j);
		}
		
        //杆自身
        ad_still_Dup[12] = flash_read(EEPROM_SECTOR4 + 15);
        
		//是否有效？
		for (j = 0; j < 13; j++) {
			if ((ad_still_Dup[j] < STD_ALARM_MIN) || (ad_still_Dup[j] > STD_ALARM_MAX))
				ad_still_Dup[j] = STD_ALARM_DEF;    //无合法数据，取缺省值
		}
	} else {	//无有效设置，取缺省值
		for (j = 0; j < 13; j++) {
			ad_still_Dup[j] = STD_ALARM_DEF;
		}
	}

	//读声光报警时间设置
	temp = flash_read(EEPROM_SECTOR5);
	if (temp == 0x5A) { //有有效设置
		temp = flash_read(EEPROM_SECTOR5 + 1);
		beep_during_temp = (Uint16)(((Uint32)temp * 1000) / SCHEDULER_TICK);
	} else {	//取缺省值
		beep_during_temp = 0;   //单位： tick
	}

	//禁止Flash访问
	flash_disable();
}
