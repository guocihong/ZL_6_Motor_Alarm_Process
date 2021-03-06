#include "task/adc/adc_task.h"

/* UART2 */
extern xdata  sUART_Q  uart2_send_queue[UART_QUEUE_NUM];     // 串口发送队列
extern xdata  sUART_Q  uart2_recv_queue[UART_QUEUE_NUM];     // 串口接收队列

/* for AD */
extern xdata  Uint16   ad_sensor_mask_LR;              //按左右顺序重排序的sensor mask: 杆、左开关量、左6 ~ 1,杆、右开关量、右6 ~ 1
extern xdata  Uint16   ad_sensor_mask;                 //15  14  13  12  11  10  9  8  7   6   5  4  3  2  1  0
											           //				右6			      右1 左6       	 左1	

extern xdata  Uint16   ad_chn_sample[13];              //最新一轮采样值（已均衡去噪声，每通道一个点，循环保存）
extern xdata  Uint16   ad_samp_pnum;                   //采样点数(计算静态基准值时总采样点数)
extern xdata  sAD_Sum  ad_samp_sum[13];                //阶段求和
extern xdata  sAD_BASE ad_chn_base[13];                //各通道运行时静态基准值/上下限阀值（单位：采样值）
extern xdata  Byte     ad_chn_over[13];                //各通道连续采样点(均衡后)的阀值判定： 0 - 范围内； 1 - 超阀值
                                                       //每通道一个字节： CH0~12 对应 ad_chn_over[0~12]
extern xdata  Uint16   ad_still_dn;                    //静态拉力值下限
extern xdata  Uint16   ad_still_up;                    //静态拉力值上限
extern xdata  Byte     ad_still_Dup[13];               //报警阀值上限
extern  data  Uint16   ad_alarm_exts;                  //外力报警标志（无mask）： 位值 0 - 无； 1 - 超阀值
extern  data  Uint16   ad_alarm_base;                  //静态张力报警标志（无mask）： 位值 0 - 允许范围内； 1 - 超允许范围
extern xdata  Uint16   ad_chnn_state[12];              //用来分析钢丝的松紧程度，从左至右的顺序为：左1~左6,右1~右6
extern xdata  Byte     alarm_point_num;                //报警点数-->连续多少个点报警才判定为报警

static xdata  Byte     alarm_matrix[5] = {0x0F,0x1F,0x3F,0x7F,0xFF};

/* for this task: 用于基准值跟踪 */
static xdata  Byte     md_point[13];                   //用于基准值跟踪的计量点数

/* for system */
extern  data Byte      system_status;                   //系统状态
extern xdata Byte      matrix_index[12];

/* variables for alarm output */
extern bdata  bit      zs_climb_alarm_flag;            //杆自身攀爬报警标志：0-无报警；1-报警
extern bdata  bit      right_climb_alarm_flag;         //右开关量攀爬报警标志：0-无报警；1-报警
extern bdata  bit      left_climb_alarm_flag;          //左开关量攀爬报警标志：0-无报警；1-报警
extern bdata  bit      adl_alarm_flag;                 //左侧张力组合报警标志: 0 - 无报警; 1 - 超阀值，报警
extern bdata  bit      adr_alarm_flag;                 //右侧张力组合报警标志: 0 - 无报警; 1 - 超阀值，报警
extern xdata  Uint16   ad_alarm_tick[13];              //各通道报警计时tick

/* 电机堵转标志 */
extern bdata  bit      gl_motor_overcur_flag;          //电机是否处于堵转状态：0-正常工作;1-电机堵转
extern bdata  volatile bit      gl_motor_adjust_flag;           //电机是否处于工作状态：0-停止工作状态;1-正处于工作状态
extern bdata  bit      is_timeout;                     //电机时间是否用完：0-没有;1-时间用完			
extern xdata  Byte     gl_chnn_index;                  //当前正在调整的钢丝的索引
extern xdata  Byte     gl_motor_overcur_point[12];     //电机堵转次数
extern xdata  Byte     gl_motor_adjust_end[12];        //是否调整完成：0-没有调整完成;1-表示调整完成
extern xdata  Uint16   ad_chnn_wire_cut;               //0-表示钢丝没有被剪断;1-表示钢丝被剪断 -->右6~右1、左6~左1
extern xdata  Byte     gl_wait_delay_tick;             //发出控制电机命令包以后延时等待的时间，然后才同步更新基准值以及是否电机堵转和时间是否用完
extern xdata  Byte     gl_5_motor_control_code[12];    //左1、右1、左2、右2、左3、右3、左4、右4、左5、右5
extern xdata  Byte     gl_6_motor_control_code[12];    //左1、右1、左2、右2、左3、右3、左4、右4、左5、右5、左6、右6
extern xdata  Byte     gl_motor_channel_number;        //用来区分是5道电机张力还是6道电机张力-->5道电机包括：4道电机、4道电机+1道级联、5道电机   6道电机包括：6道电机、4道电机+2道级联、5道电机+1道级联
extern bdata  bit      is_motor_add_link;              //电机张力是否添加级联:0-不级联;1-级联
extern bdata  bit      is_sample_clear;                //采样值是否清零:0-没有清零；1-已经清零

/* Doorkeep(门磁) */
extern bdata  bit      gl_local_dk_status;            //门磁开关状态（每1s动态检测）: 1 - 闭合; 0 - 打开(需要报警)
extern bdata  bit      gl_control_dk_status;          //控制杆的门磁状态: 1 - 闭合; 0 - 打开(需要报警)                    

//2016-02-28新增
extern xdata sAlarmDetailInfo  AlarmDetailInfo;        //保存最后一次报警详细信息

/* 函数声明*/
extern Byte uart2_get_send_buffer(void);

void adc_task_init(void)
{
	Byte i;

	//相关变量初始化
	ad_samp_pnum    = 0;                     //采样点数(计算静态基准值时总采样点数)
	for (i = 0; i < 13; i++)
	{
		ad_chn_sample[i]       = 0;        //最新一轮采样值
		ad_samp_sum[i].sum       = 0;        //阶段求和
		ad_samp_sum[i].point     = 0;
		ad_chn_base[i].base      = 0;        //各通道静态基准值/上下限阀值
		ad_chn_base[i].base_down = 0;
		ad_chn_base[i].base_up   = 0;
		ad_chn_over[i]           = 0;        //各通道连续采样点(均衡后)的阀值判定：均在范围内
		md_point[i]              = 0;        //用于基准值跟踪的计量点数
		ad_alarm_tick[i]         = 0;
	}
	
	for (i = 0; i < 12; i++) {
		gl_motor_overcur_point[i] = 0;
		gl_motor_adjust_end[i]    = 0;
		ad_chnn_state[i]          = 0;
	}

    zs_climb_alarm_flag    = 0;
    right_climb_alarm_flag = 0;
    left_climb_alarm_flag  = 0;
    
	ad_alarm_exts    = 0;                    //外力报警标志（无mask）: 无
	ad_alarm_base    = 0;                    //静态张力报警标志（无mask）：允许范围内
	gl_chnn_index    = 0;                    //从左1开始
	ad_chnn_wire_cut = 0;   
}

void adc_task(void)
{
	Byte i,j;            //循环变量
    Byte isValidData = 0;
	Uint16 val_temp;
    Byte val_sum;
	Uint16 val;
	Byte index;
    Byte temp_index;
    Byte k,y;
    
	//1.处理UART2：来自控制杆的数据包
	//找是否有等待处理的项
	for (i = 0; i < UART_QUEUE_NUM; i++)
	{
		if (uart2_recv_queue[i].flag == 1)//有等待处理的项
		{           
            isValidData = 1;
						
			//左1~6,右1~6,杆自身的瞬间张力值
			for (j = 0; j < 13; j++) {
				ad_chn_sample[j] = ((Uint16)(uart2_recv_queue[i].tdata[6 + (j << 1)]) << 8) + uart2_recv_queue[i].tdata[7 + (j << 1)];
			}
			
			//控制杆的门磁状态
			gl_control_dk_status   = uart2_recv_queue[i].tdata[32];//0-报警;1-正常
			
			//电机状态
			gl_motor_adjust_flag   = uart2_recv_queue[i].tdata[33];//0-停止;1-运转
			
			//电机堵转状态
			gl_motor_overcur_flag  = uart2_recv_queue[i].tdata[34];//0-正常;1-堵转
				
			//时间是否用完
			is_timeout             = uart2_recv_queue[i].tdata[35];//0-没有;1-时间用完
            
			//处理完成,释放该队列项
			uart2_recv_queue[i].flag = 0;

			break;
		}
	}


    if (isValidData == 0) {//没有等待处理的项
        return;
    }
       
	switch (system_status)
	{
	case SYS_SAMP_BASE: //初始上电时的静态基准值采样        
		for (i = 0; i < 13; i++) {
			ad_samp_sum[i].sum += ad_chn_sample[i];
		}
		ad_samp_pnum++;
		
		if (ad_samp_pnum == 32) //已经满基准值采样点数（每通道32点，均衡后, 耗时约10秒)
		{			
			ad_samp_pnum = 0;
			
			//计算均值和上下限
			for (i = 0; i < 13; i++)
			{
				//基准
				ad_chn_base[i].base = ad_samp_sum[i].sum >> 5;   //除于32

				//下限 = 基准 * （1 / 3）
				val_temp = ad_chn_base[i].base;
				ad_chn_base[i].base_down = (val_temp >> 1) - (val_temp >> 3) - (val_temp >> 4);
                //ad_chn_base[i].base_down = val_temp - ad_still_Dup[i] * 2;

				//上限
				if ((1023 - ad_chn_base[i].base) > ad_still_Dup[i])
				{
					ad_chn_base[i].base_up = ad_chn_base[i].base + ad_still_Dup[i];
				}
				else
				{
					ad_chn_base[i].base_up = 1023;
				}

				//检查静态张力是否在允许范围内
				//只有左6道钢丝和右6道钢丝才需要判断是否静态报警,杆自身不存在静态报警,杆自身只有外力报警
				if ((i >= 0) && (i <= 11))
				{
					check_still_stress(i);
				}

				//复位阶段和变量，准备用于自适应阀值跟踪
				ad_samp_sum[i].sum   = 0;
				ad_samp_sum[i].point = 0;
			}

			//更新led状态
			update_led_status();

			//更新组合报警标志
			update_alarm_status();
		
			//状态->开机自检
			system_status = SYS_SELF_CHECK1;
		}
        
		break;

	case SYS_SELF_CHECK1://开机自检阶段1-->左1 右1 左2 右2 ... 左6 右6
        if((is_motor_add_link == 1) && (is_sample_clear == 0)){
            //加级联的情况下，必须要等到采样值清零之后才会自动收紧钢丝
            //不加级联的情况下，主机一上电，就会自动收紧钢丝
            
            //立即同步更新静态基准值
            for (k = 0; k < 12; k++) {
            	ad_chn_base[k].base = ad_chn_sample[k];
                val_temp = ad_chn_base[k].base;
                ad_chn_base[k].base_down = (val_temp >> 1) - (val_temp >> 3) - (val_temp >> 4);
                if ((1023 - ad_chn_base[k].base) > ad_still_Dup[k]) {
                    ad_chn_base[k].base_up = ad_chn_base[k].base + ad_still_Dup[k];
                } else {
                    ad_chn_base[k].base_up = 1023;
                }
                
                //检查静态张力是否在允许范围内
                //只有左6道钢丝和右6道钢丝才需要判断是否静态报警,杆自身不存在静态报警,杆自身只有外力报警
                check_still_stress(k);
            }
            
            gl_chnn_index = 0;
            
            //更新led状态
            update_led_status();
            
            //更新组合报警标志
            update_alarm_status();
            
            return;
        }
        
		//gl_chnn_index取值范围为0~11，分别依次对应为：左1 右1 左2 右2 ... 左6 右6
		//ad_chn_sample/ad_chn_base的0~5对应左1~左6,6~11对应右1~右6，所以需要转换一下索引
		index = matrix_index[gl_chnn_index];
		         						
		//判断拨码开关是否打开
		if ((ad_sensor_mask >> index) & 0x0001) {//拨码打开
			if (ad_chn_sample[index] < 100) {//钢丝比较松
				system_status = SYS_SELF_CHECK2;
				gl_motor_adjust_flag  = 0;
				gl_motor_overcur_flag = 0;
				is_timeout            = 0;
				gl_wait_delay_tick    = 4;//延时等待20ms
				motor_adjust(gl_chnn_index);
			} else {
				//本钢丝调整完成
				gl_motor_adjust_end[index] = 1;
				
				//操作下一根钢丝
				gl_chnn_index++;
				if (gl_chnn_index == 12) {
					gl_chnn_index = 0;
				}
			}
		} else {//拨码关闭
			//本钢丝调整完成
			gl_motor_adjust_end[index] = 1;
				
			//操作下一根钢丝
			gl_chnn_index++;
			if (gl_chnn_index == 12) {
				gl_chnn_index = 0;
			}
		}
		
		//判断所有钢丝是否调整完成
		val_sum = 0;
		for (i = 0; i < 12; i++) {
			val_sum += gl_motor_adjust_end[i];
		}

		if (val_sum == 12) {//所有钢丝调整完成
			system_status = SYS_CHECK;
			gl_chnn_index = 0;
		}
		
		break;
		
	case SYS_SELF_CHECK2://开机自检阶段2
		if (gl_wait_delay_tick > 0) {//延时等待20ms
			return;
		}
		
		index = matrix_index[gl_chnn_index];
		if (gl_motor_adjust_flag == 1) {//电机正在运转
			//立即同步更新静态基准值
			ad_chn_base[index].base = ad_chn_sample[index];
			val_temp = ad_chn_base[index].base;
			ad_chn_base[index].base_down = (val_temp >> 1) - (val_temp >> 3) - (val_temp >> 4);
            //ad_chn_base[index].base_down = val_temp - ad_still_Dup[index] * 2;
			if ((1023 - ad_chn_base[index].base) > ad_still_Dup[index]) {
				ad_chn_base[index].base_up = ad_chn_base[index].base + ad_still_Dup[index];
			} else {
				ad_chn_base[index].base_up = 1023;
			}
		}
			
		//检查静态张力是否在允许范围内
		//只有左6道钢丝和右6道钢丝才需要判断是否静态报警,杆自身不存在静态报警,杆自身只有外力报警
		check_still_stress(index);
		
		//更新led状态
		update_led_status();
		
		//更新组合报警标志
		update_alarm_status();
		
		if (gl_motor_overcur_flag == 1) {//电机堵转
			gl_motor_overcur_point[index]++;
			if (gl_motor_overcur_point[index] >= 3) {
				gl_motor_adjust_end[index] = 1;//本道钢丝调整结束
			}
			
			//操作下一根钢丝
			gl_chnn_index++;
			if (gl_chnn_index == 12) {
				gl_chnn_index = 0;
			}
			system_status = SYS_SELF_CHECK1;
		}
		
		if (is_timeout == 1) {//时间用完，操作下一根钢丝
			gl_chnn_index++;
			if (gl_chnn_index == 12) {
				gl_chnn_index = 0;
			}		
			system_status = SYS_SELF_CHECK1;
		}
						
		//判断所有钢丝是否调整完成
		val_sum = 0;
		for (i = 0; i < 12; i++) {
			val_sum += gl_motor_adjust_end[i];
		}
		
		if (val_sum == 12) {//所有钢丝调整完成
			system_status = SYS_CHECK;
			gl_chnn_index = 0;
		}
		
		break;
		
	case SYS_CHECK:     //实时检测  
        //分析钢丝的松紧程度
		for (i = 0; i < 12; i++) {
			ad_chnn_state[i] += ad_chn_sample[i];
		}
		ad_samp_pnum++;
		if (ad_samp_pnum == 45) {//大约耗时11.7s，电机转动时间为10s
			ad_samp_pnum = 0;
			
			for (i = 0; i < 12; i++) {//左1 右1 左2 右2 左3 右3 左4 右4 左5 右5 左6 右6
				//判断拨码开关是否打开
				temp_index = matrix_index[i];
				if ((ad_sensor_mask >> temp_index) & 0x0001) {//拨码打开
					if ((temp_index >= 0) && (temp_index <= 5)) {//左1~左6					
						if (((ad_chnn_wire_cut >> (temp_index + 8)) & 0x0001) == 0) {//钢丝没有被剪断
							val = ad_chnn_state[temp_index] / 45;
							if (val < 60) {//钢丝比较松，收紧钢丝
								gl_motor_adjust_flag  = 0;
								gl_motor_overcur_flag = 0;
								is_timeout            = 0;
								motor_adjust(i);
                                gl_chnn_index = temp_index;
								break;
							}
						}
					} else if ((temp_index >= 6) && (temp_index <= 11)) {//右1~右6					
						if (((ad_chnn_wire_cut >> (temp_index - 6)) & 0x0001) == 0) {//钢丝没有被剪断
							val = ad_chnn_state[temp_index] / 45;
							if (val < 60) {//钢丝比较松，收紧钢丝
								gl_motor_adjust_flag  = 0;
								gl_motor_overcur_flag = 0;
								is_timeout            = 0;
								motor_adjust(i);
                                gl_chnn_index = temp_index;
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
		
		if (gl_motor_adjust_flag == 1) {//电机正在运转
			//立即同步更新静态基准值
			ad_chn_base[gl_chnn_index].base = ad_chn_sample[gl_chnn_index];
			val_temp = ad_chn_base[gl_chnn_index].base;
			ad_chn_base[gl_chnn_index].base_down = (val_temp >> 1) - (val_temp >> 3) - (val_temp >> 4);
			if ((1023 - ad_chn_base[gl_chnn_index].base) > ad_still_Dup[gl_chnn_index]) {
				ad_chn_base[gl_chnn_index].base_up = ad_chn_base[gl_chnn_index].base + ad_still_Dup[gl_chnn_index];
			} else {
				ad_chn_base[gl_chnn_index].base_up = 1023;
			}
		}
			
		//分析钢丝是否被剪断
		if (is_timeout == 1) {
			val_temp = ad_chn_base[gl_chnn_index].base;
			if (val_temp < 10) {//钢丝被剪断
				//置位
				if ((gl_chnn_index >= 0) && (gl_chnn_index <= 5)) {//左1~左6
					ad_chnn_wire_cut |= ((Uint16)0x01 << (gl_chnn_index + 8));
				} else if ((gl_chnn_index >= 6) && (gl_chnn_index <= 11)) {//右1~右6
					ad_chnn_wire_cut |= ((Uint16)0x01 << (gl_chnn_index - 6));
				}
			}
		}
        
		//解析12道钢丝是否外力报警以及静态报警-->左1~左6、右1~右6、杆自身
		//解析控制杆是否外力报警，控制杆不存在静态报警			
		for (i = 0; i < 13; i++) {			
			ad_chn_over[i] = ad_chn_over[i] << 1;   //Bit0填0，因此缺省在允许范围内
			val = ad_chn_sample[i];
			//if ((val >= ad_chn_base[i].base_down) && (val <= ad_chn_base[i].base_up)) {//在张力上/下限允许范围内		
            if (val <= ad_chn_base[i].base_up) {//在张力上/下限允许范围内
                //a. 清标志(缺省)
				//b. 计入跟踪基准值求和中
				ad_samp_sum[i].sum += val;
				ad_samp_sum[i].point++;

				if (ad_samp_sum[i].point == 2) {
					//满2点(约需0.6秒)
					//b.0 计算这2点均值
					val_temp = ad_samp_sum[i].sum >> 1;   //除于2, 得到这2点的均值
					//b.1 更新基准值
					if (ad_chn_base[i].base > (val_temp + 1)) {
						//至少小2, 在缓慢松弛
						//ZZX: 立即跟踪, 跟踪差值的 1/2
						val_temp = (ad_chn_base[i].base - val_temp) >> 1;
						if (ad_chn_base[i].base >= val_temp) {
                            
							ad_chn_base[i].base -= val_temp;
							//同步更新上下限
							val_temp = ad_chn_base[i].base;
							ad_chn_base[i].base_down = (val_temp >> 1) - (val_temp >> 3) - (val_temp >> 4);
							//ad_chn_base[i].base_down = val_temp - ad_still_Dup[i] * 2;
                            if ((1023 - ad_chn_base[i].base) > ad_still_Dup[i]) {
								ad_chn_base[i].base_up = ad_chn_base[i].base + ad_still_Dup[i];
							} else {
								ad_chn_base[i].base_up = 1023;
							}
                            
						}

						//清缓慢张紧跟踪变量
						md_point[i] = 0;
					} else if (val_temp > (ad_chn_base[i].base + 1)) {
						// 至少大2, 缓慢张紧
						md_point[i]++;
						if (md_point[i] >= DEF_ModiBASE_PT) {
							// 已满缓慢张紧时的连续计量点数, 进行一次跟踪
							// 1. 跟踪基准值
							if (ad_chn_base[i].base < 1023) {
								//可以递增1
								ad_chn_base[i].base++;
								// 同步更新上下限
								val_temp = ad_chn_base[i].base;
								ad_chn_base[i].base_down = (val_temp >> 1) - (val_temp >> 3) - (val_temp >> 4);
								//ad_chn_base[i].base_down = val_temp - ad_still_Dup[i] * 2;
                                if (ad_chn_base[i].base_up < 1023) {
									ad_chn_base[i].base_up++;
								}
							}
							// 2. 清缓慢张紧跟踪变量
							md_point[i] = 0;
						}
					}

					//b.2 复位阶段和变量 - 用于4点4点平均的求和结构
					ad_samp_sum[i].sum   = 0;
					ad_samp_sum[i].point = 0;
				}              
			} else {
				//外力报警, 置标志
				ad_chn_over[i] |= 0x01;
			}

            //连续6点超范围，此通道有外力报警
			if ((ad_chn_over[i] & alarm_matrix[alarm_point_num - 4]) == alarm_matrix[alarm_point_num - 4]) {
				//先保存报警详细信息，然后再更新静态基准值，这样就可以很好的分析出为什么报警
                for (y = 0; y < 13; y++) {
                    AlarmDetailInfo.StaticBaseValue[y] = ad_chn_base[y].base;
                    AlarmDetailInfo.InstantSampleValue[y] = ad_chn_sample[y];
                }
                
                if ((i >= 0) && (i <= 5)) {//左1~左6
					//超出允许范围，置标志
					ad_alarm_exts |= ((Uint16)0x01 << (i + 8));
				} else if ((i >= 6) && (i <= 11)) {//右1~右6
					//超出允许范围，置标志
					ad_alarm_exts |= ((Uint16)0x01 << (i - 6));
				}

				if (i == 12) {//杆自身
					ad_alarm_exts |= ((Uint16)0x01 << (i + 3));
					ad_alarm_exts |= ((Uint16)0x01 << (i - 5));
                    
                    if ((ad_sensor_mask_LR >> 7) & 0x0001) {//杆自身拨码打开
                        zs_climb_alarm_flag = 1;
                    }
				}

				//报警计时tick
				ad_alarm_tick[i] = ALARM_TEMPO;

				//立即更新静态基准值
				ad_chn_base[i].base = val;
				val_temp = ad_chn_base[i].base;
				ad_chn_base[i].base_down = (val_temp >> 1) - (val_temp >> 3) - (val_temp >> 4);
				//ad_chn_base[i].base_down = val_temp - ad_still_Dup[i] * 2;
                if ((1023 - ad_chn_base[i].base) > ad_still_Dup[i]) {
					ad_chn_base[i].base_up = ad_chn_base[i].base + ad_still_Dup[i];
				} else {
					ad_chn_base[i].base_up = 1023;
				}
                
                //复位阶段和变量 - 用于4点4点平均的求和结构
                ad_samp_sum[i].sum = 0;
                ad_samp_sum[i].point = 0;               
                //清缓慢张紧跟踪变量
                md_point[i] = 0;   //用于基准值跟踪的计量点数                
			} else if ((ad_chn_over[i] & alarm_matrix[alarm_point_num - 4]) == 0x00) {//无外力报警
				if (ad_alarm_tick[i] == 0) {//检查报警时间是否已到
					//报警已经到最大报警时间, 停止报警
					if ((i >= 0) && (i <= 5)) {//左1~左6
						//在允许范围内, 清标志
						ad_alarm_exts &= (~((Uint16)0x01 << (i + 8)));
					} else if ((i >= 6) && (i <= 11)) {//右1~右6
						//在允许范围内, 清标志
						ad_alarm_exts &= (~((Uint16)0x01 << (i - 6)));
					}

					if (i == 12) {//杆自身
						ad_alarm_exts &= (~((Uint16)0x01 << (i + 3)));
						ad_alarm_exts &= (~((Uint16)0x01 << (i - 5)));
                        zs_climb_alarm_flag = 0;
					}
				}
			}

			//检查静态张力是否在允许范围内
			//只有左6道钢丝和右6道钢丝才需要判断是否静态报警
			if ((i >= 0) && (i <= 11)) {
				check_still_stress(i);
			}		
		}
		
		//解析左开关量，右开关量是否外力报警，不存在静态报警
		//读取左开关量的值
		if (bLeftSwitch == 0) {//报警
			ad_alarm_exts |= (1 << 14);
            
            if ((ad_sensor_mask_LR >> 14) & 0x0001) {//左开关量拨码打开
                left_climb_alarm_flag = 1;
            }            
		} else {//正常
			ad_alarm_exts &= (~(1 << 14));
            left_climb_alarm_flag = 0;
		}
		
		//读取右开关量的值
		if (bRightSwitch == 0) {//报警
			ad_alarm_exts |= (1 << 6);
            
            if ((ad_sensor_mask_LR >> 6) & 0x0001) {//右开关量拨码打开
                right_climb_alarm_flag = 1;
            }   
		} else {//正常
			ad_alarm_exts &= (~(1 << 6));
            right_climb_alarm_flag = 0;
		}

		//更新led状态
		update_led_status();
		
		//更新组合报警标志
		update_alarm_status();
		
		break;
	}
}

//检查指定通道的静态张力是否在允许范围内
void check_still_stress(Byte index)
{
	if ((ad_chn_base[index].base >= ad_still_dn) && 
	    (ad_chn_base[index].base <= ad_still_up)) {		
		if ((index >= 0) && (index <= 5)) {//左1~左6
			//在允许范围内, 清标志
			ad_alarm_base &= (~((Uint16)0x01 << (index + 8)));
		} else if ((index >= 6) && (index <= 11)) {//右1~右6
			//在允许范围内, 清标志
			ad_alarm_base &= (~((Uint16)0x01 << (index - 6)));
		}
	} else {
		if ((index >= 0) && (index <= 5)) {//左1~左6
			//超出允许范围，置标志
			ad_alarm_base |= ((Uint16)0x01 << (index + 8));	
		} else if ((index >= 6) && (index <= 11)) {//右1~右6
			//超出允许范围，置标志
			ad_alarm_base |= ((Uint16)0x01 << (index - 6));
		}
	}
	
	//杆自身、左开关量、右开关量不存在静态报警
	ad_alarm_base &= 0x3F3F;
}

//index：0-11分别对应为左1 右1 左2 右2 ... 左6 右6
void motor_adjust(Byte index)
{
	Byte i;
	
	//在UART2队列中找空闲Buffer
	i = uart2_get_send_buffer();
	if (i < UART_QUEUE_NUM) {
		//找到了空闲buffer, 写入data
		uart2_send_queue[i].tdata[0] = FRAME_STX;
		uart2_send_queue[i].tdata[1] = 0xFF;       //目的地址
		uart2_send_queue[i].tdata[2] = 0x01;	   //源地址
		uart2_send_queue[i].tdata[3] = 0x06;
		uart2_send_queue[i].tdata[4] = CMD_ZL_PRE;
		uart2_send_queue[i].tdata[5] = 0x04;
		uart2_send_queue[i].tdata[6] = 0xF0;
		uart2_send_queue[i].tdata[7] = index;      //电机号
        if ((gl_motor_channel_number == 4) || (gl_motor_channel_number == 5)) {
        	uart2_send_queue[i].tdata[8] = gl_5_motor_control_code[index];          //正转
        } else if (gl_motor_channel_number == 6) {
        	uart2_send_queue[i].tdata[8] = gl_6_motor_control_code[index];          //正转
        }
		uart2_send_queue[i].tdata[9] = 10;         //运转时间10s

		uart2_send_queue[i].len = 11;
	}
}

void update_led_status(void)
{
	Uint16 temp16;
	
	//LED指示
	temp16 = (ad_alarm_exts | ad_alarm_base);
	//左1~6报警标志
	P3 = (~(HIGH(temp16) << 2));

	//右1~6报警标志
	P0 = (~(LOW(temp16) & 0x3F));

	//杆自身报警标志
	bSelf_Led_Ctrl = (~((ad_alarm_exts >> 7) & 0x0001) & 0x0001);	
}

void update_alarm_status(void)
{
	Byte temp;
	
	//左6~1报警标志
	temp = HIGH((ad_alarm_exts | ad_alarm_base)) & HIGH(ad_sensor_mask_LR) & 0x3F;//过滤杆自身攀爬报警和左开关量攀爬报警，杆自身攀爬报警和左开关量攀爬报警不算在左防区报警内，单独开辟一个攀爬报警
	//判左侧组合报警
	if (temp == 0) {
		adl_alarm_flag = 0;   //无报警
	} else {
		adl_alarm_flag = 1;     //有报警
	}

	//右6~1报警标志
	temp = LOW((ad_alarm_exts | ad_alarm_base)) & LOW(ad_sensor_mask_LR) & 0x3F;//过滤杆自身攀爬报警和右开关量攀爬报警，杆自身攀爬报警和右开关量攀爬报警不算在右防区报警内，单独开辟一个攀爬报警
    //判右侧组合报警
    if (temp == 0) {
		adr_alarm_flag = 0;   //无报警
	} else {
		adr_alarm_flag = 1;   //有报警
	}
}

//2016-02-28新增
//保存报警详细信息
void save_alarm_detail_info(void)
{    
    Byte y;
    
    if ((left_climb_alarm_flag == 1) || (right_climb_alarm_flag == 1) || 
        ((gl_local_dk_status | !gl_control_dk_status) == 1)) {
        for (y = 0; y < 13; y++) {
            AlarmDetailInfo.StaticBaseValue[y] = ad_chn_base[y].base;
            AlarmDetailInfo.InstantSampleValue[y] = ad_chn_sample[y];
        }
    }
        
    AlarmDetailInfo.DoorKeepAlarm = (Byte)(gl_local_dk_status | !gl_control_dk_status);
    AlarmDetailInfo.StaticAlarm   = ad_alarm_base;
    AlarmDetailInfo.ExternalAlarm = ad_alarm_exts;
}