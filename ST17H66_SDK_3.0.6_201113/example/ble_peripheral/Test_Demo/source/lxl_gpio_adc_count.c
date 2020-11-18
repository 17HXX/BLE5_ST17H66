#include "lxl_gpio_all_init.h"
#define LXL_MRT_ABS(a,b) ((a)>(b)?(a-b):(b-a))

#if 0
osal_snv_read    //写数据到flash中
osal_snv_write   //读数据到mcu中

所以需要你定义一个宏 #define 2000_UNIX_TIME 946656000
osal_setClock（1584598114-946656000）
要注意一下， osal_setClock(0) ,这个参数0的意思是 2000年1月1日0时0分0秒
osal_ConvertUTCTime   转为为 年月日
osal_getClock   就获取 UTCTime  单位为秒
#endif

lxl_struct_adc_all lxl_adc_all_data = {
    .diff_adc_run_flag=0,            //当前工作状态://0--idle  1--busy
    .adc_echo_flag =0 ,              //当前事件状态://0--busy  1--idle
    .adc_heat_value=0,               //当前的温度
    .adc_enable = 0,                 //adc是否正常工作
	.adc_ntc_value= 0, 		         //当前ntc的温度

	.adc_diff_value = 0,             //当前adc值
} ;

adc_Cfg_t normal_adc_cfg = {
      //.channel = ADC_BIT(ADC_CH1N_P11)|ADC_BIT(ADC_CH1P_P12)|ADC_BIT(ADC_CH2N_P13)|ADC_BIT(ADC_CH2P_P14)|ADC_BIT(ADC_CH3N_P15),
      .channel = ADC_BIT(ADC_CH3P_P20)|ADC_BIT(ADC_CH3N_P15),
      .is_continue_mode = FALSE,
      .is_differential_mode = 0x00,
      .is_high_resolution = 0xff,
};

void normal_adc_handle_evt(adc_Evt_t* pev)
{
	bool is_high_resolution = FALSE;
	bool is_differential_mode = FALSE;
	float adc_result=0;
	static uint8 check_flag=0x00;

	if(pev->type == HAL_ADC_EVT_DATA){
		if(pev->ch==ADC_CH3N_P15)
		{
			//normal_adc_cfg.is_high_resolution = 0xff ;  //修改基准电压
			is_high_resolution = (normal_adc_cfg.is_high_resolution & ADC_BIT(ADC_CH3N_P15))?TRUE:FALSE;
			is_differential_mode = (normal_adc_cfg.is_differential_mode & ADC_BIT(ADC_CH3N_P15))?TRUE:FALSE;
			adc_result = hal_adc_value_cal((adc_CH_t)(ADC_CH3N_P15),pev->data, pev->size, is_high_resolution,is_differential_mode);

			int ntc_volt =(int)(adc_result*1000);
			uint32 Ressis=(uint32)(((adc_result*1000)*470*1000/(3300-ntc_volt)-1300)/10);
			check_flag|=0x01;

			float adc_ntc_data = 0 ;	//计算的温度
			adc_ntc_data  = lxl_get_near_ntc_temp_index(Ressis) ;
			lxl_adc_all_data.adc_ntc_value= (u16)(adc_ntc_data*10) ;
			lxl_all_func_data.dev_rtc_t = lxl_adc_all_data.adc_ntc_value ;

			LOG("NTC_Temp=%d \n",lxl_adc_all_data.adc_ntc_value);
		}else if(pev->ch==ADC_CH3P_P20){
			//normal_adc_cfg.is_high_resolution = 0 ;  //修改基准电压
			is_high_resolution = (normal_adc_cfg.is_high_resolution & ADC_BIT(ADC_CH3P_P20))?TRUE:FALSE;
			is_differential_mode = (normal_adc_cfg.is_differential_mode & ADC_BIT(ADC_CH3P_P20))?TRUE:FALSE;
			adc_result = hal_adc_value_cal((adc_CH_t)(ADC_CH3P_P20),pev->data, pev->size, is_high_resolution,is_differential_mode);
			lxl_adc_all_data.adc_diff_value = (int)(adc_result*1000) ;
			lxl_adc_all_data.adc_diff_value = (lxl_adc_all_data.adc_diff_value*72)/10 ;
			//lxl_adc_all_data.adc_diff_value = lxl_adc_all_data.adc_diff_value+100 ;

			if(lxl_adc_all_data.adc_diff_value>=3950){
				lxl_device_all_data.dev_batt_value= 100 ;
			}else if(lxl_adc_all_data.adc_diff_value>=3700){
				lxl_device_all_data.dev_batt_value= 75 ;
			}else if(lxl_adc_all_data.adc_diff_value>=3550){
				lxl_device_all_data.dev_batt_value= 50 ;
			}else if(lxl_adc_all_data.adc_diff_value>=3400){
				lxl_device_all_data.dev_batt_value= 25 ;
			}else{
				lxl_device_all_data.dev_batt_value= 0 ;
			}
			check_flag|=0x02;
			LOG("bat Volt=%d  ",lxl_adc_all_data.adc_diff_value);
			//osal_start_timerEx(uint8 task_id, uint16 event_id, uint32 timeout_value)
		}

		if(check_flag==0x03){	//计算adc数值
			check_flag=0;
			osal_start_timerEx(lxl_adc_count_TaskID, MY_ADC_22_EVENT_TICK, 10);
		}
	}
}


int normal_adc_init(void)
{
	int ret;  
	ret = hal_adc_config_channel(normal_adc_cfg, normal_adc_handle_evt);
	if(ret)
	return ret;
	return PPlus_SUCCESS;
}

void lxl_gpio_charge_count_func()   //充电判断
{
	static u32 gpio_charge_adc_value_numbale = 0 ;
	if(hal_gpio_read(MY_GPIO_ADC_5V_TEST)){
		if(lxl_device_all_data.dev_charge_tick == 0){
			lxl_device_all_data.dev_charge_tick = 1 ;
			lxl_device_all_data.dev_boot_up_tick = 1 ;

			lxl_all_func_data.dev_onoff_tick = 0 ;
			lxl_all_func_data.dev_mode_tick = 0 ;
			lxl_all_func_data.dev_warm_tick = 0 ;

			lxl_led_enter_mode_1(4) ;
		}
		if(lxl_device_all_data.dev_batt_value == 100){
			LOG("BATT_MAX: %d \n",gpio_charge_adc_value_numbale);
			gpio_charge_adc_value_numbale++ ;
			if(gpio_charge_adc_value_numbale>5000){
				gpio_charge_adc_value_numbale = 5000 ;
				lxl_device_all_data.dev_charge_tick = 2 ;
			}
		}else {
			gpio_charge_adc_value_numbale = 0 ;
		}
	}else {
		if(lxl_device_all_data.dev_charge_tick){      //拔掉充电线执行关机功能
			lxl_device_all_data.dev_charge_tick = 0 ;
			write_reg(0x4000f030,1) ;
			lxl_device_all_data.dev_job_tick = 0 ;    //关机
		}
		lxl_device_all_data.dev_charge_tick = 0 ;
	}
}








/*******************************************************************
** 函数名:     get_near_index
** 函数描述:   获取指定数值在数组中最临近的值的索引
** 参数:      value:需要查找
** 返回:      最临近值的索引
********************************************************************/
float lxl_get_near_ntc_temp_index(uint32 value)
{
	int8 Mrt117_NTC_Temp[121]=
	{
        -20,  -19, -18, -17, -16, -15, -14, -13, -12, -11,
        -10,   -9,  -8,  -7,  -6,  -5,  -4,  -3,  -2,  -1,
        0,    1,   2,   3,   4,   5,   6,   7,   8,   9,
        10,   11,  12,  13,  14,  15,  16,  17,  18,  19,
        20,   21,  22,  23,  24,  25,  26,  27,  28,  29,
        30,   31,  32,  33,  34,  35,  36,  37,  38,  39,
        40,   41,  42,  43,  44,  45,  46,  47,  48,  49,
        50,   51,  52,  53,  54,  55,  56,  57,  58,  59,
        60,   61,  62,  63,  64,  65,  66,  67,  68,  69,
        70,   71,  72,  73,  74,  75,  76,  77,  78,  79,
        80,   81,  82,  83,  84,  85,  86,  87,  88,  89,
        90,   91,  92,  93,  94,  95,  96,  97,  98,  99,
        100
	};

	#if 0
	uint32 Mrt117_NTC_Resis[121]=
	{
	    93050,88014,83284,78837,74654,70718,67011,63518,60226,57120,//-20
	    54190,51423,48810,46341,44007,41799,39710,37734,35862,34089,//-10
	    32410,30768,29234,27799,26453,25189,24000,22878,21820,20819,//  0
	    19871,18972,18119,17307,16535,15798,15096,14426,13785,13172,// 10
	    12586,12024,11486,10970,10475,10000, 9564, 9150, 8757, 8384,// 20
	    8029, 7692, 7371, 7066, 6775, 6499, 6235, 5984, 5744, 5516, // 30
	    5298, 5090, 4891, 4701, 4520, 4347, 4181, 4023, 3871, 3726, // 40
	    3588, 3455, 3328, 3206, 3089, 2977, 2870, 2767, 2668, 2574, // 50
	    2483, 2395, 2312, 2231, 2154, 2080, 2008, 1910, 1874, 1811, // 60
	    1750, 1691, 1635, 1580, 1508, 1478, 1429, 1382, 1337, 1294, // 70
	    1252, 1212, 1173, 1136, 1100, 1065, 1031,  999,  968,  937, // 80
	    908,  880,  853,  827,  801,  777,  753,  731,  708,  687,  // 90
	    667,                                                        //100
	};
	#endif
	uint32 Mrt117_NTC_Resis[121]=
	{
		97580,92059,86886,82036,77487,73218,69212,65449,61915,58593,// -20
		55470,52532,49768,47166,44715,42407,40232,38182,36248,34423,// -10
		32701,31076,29541,28090,26720,25424,24198,23039,21942,20903,// -0
		19920,18988,18105,17268,16475,15722,15008,14331,13688,13077,// 10
		12497,11946,11422,10924,10450,10000,9571 ,9163 ,8774 ,8405, // 20
		8052 ,7717 ,7397 ,7092 ,6801 ,6524 ,6259 ,6007 ,5766 ,5536, // 30
		5316 ,5106 ,4906 ,4714 ,4531 ,4356 ,4188 ,4028 ,3875 ,3728, // 40
		3588 ,3454 ,3325 ,3202 ,3084 ,2970 ,2862 ,2758 ,2658 ,2563, // 50
		2471 ,2383 ,2299 ,2218 ,2140 ,2065 ,1994 ,1925 ,1859 ,1795, // 60
		1734 ,1675 ,1619 ,1564 ,1512 ,1462 ,1414 ,1367 ,1322 ,1279, // 70
		1238 ,1198 ,1159 ,1122 ,1086 ,1052 ,1019 ,987  ,956  ,926,  // 80
		898  ,870  ,843  ,817  ,793  ,769  ,745  ,723  ,702  ,681,  // 90
		661                                                         //  100                                      
	};

	float ret_index=0;                        /* 结果索引 */
    uint8 mid_index=0;                        /* 中位游标 */
    uint8 left_index=0;                       /* 左位游标 */
    uint8 right_index=120;                      /* 右位游标 */
    uint32 left_abs=0;                       /* 左位的值与目标值之间的差的绝对值 */
    uint32 right_abs=0;                      /* 右位的值与目标值之间的差的绝对值 */

    while(left_index != right_index)
    {
        mid_index = (right_index + left_index) / 2;
        if (value >= Mrt117_NTC_Resis[mid_index])
        {
            right_index = mid_index;
        }else{
            left_index = mid_index;
        }
        if (right_index - left_index < 2)
        {
            break;
        }
    }

    left_abs = MRT_ABS(Mrt117_NTC_Resis[left_index],value);
    right_abs = MRT_ABS(Mrt117_NTC_Resis[right_index],value);
    //mid_index = right_abs <= left_abs ? right_index : left_index;
    mid_index= left_index ;
	ret_index = (float)(Mrt117_NTC_Temp[mid_index])+(1.0*(Mrt117_NTC_Resis[left_index]-value)/(Mrt117_NTC_Resis[left_index]-Mrt117_NTC_Resis[right_index])) ;
    return ret_index;
}




