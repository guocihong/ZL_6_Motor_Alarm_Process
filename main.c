#include "system_init.h"

/*
注意事项：
1、需要设置出厂值，不需要设置采集值清零
2、测试过程中需要与控制杆电路板配合
3、测试事项：
	a、测试led是否正常
	b、测试拨码开关是否正常
	c、测试继电器是否正常
	d、测试uart2是否正常
	e、测试两路RS485是否正常
	f、 测试左开关量、右开关量是否正常
	g、测试设置出厂值是否正常
*/

void main(void)
{
    //系统初始化
    system_init();

    //打开总中断
    Enable_interrupt();

    //使用看门狗
    Wdt_enable();// 2.276s 

    while(1) {       
        //系统状态处理
        status_task();

        //解析uart接收的命令
        uart_task();
        
        //喂狗
        Wdt_refresh();
		
		//adc_task
		adc_task();
				
        //喂狗
        Wdt_refresh();
		
        //门磁处理
        doorkeep_task();

        //报警处理
        alarm_task();

        //喂狗
        Wdt_refresh();
    }
}