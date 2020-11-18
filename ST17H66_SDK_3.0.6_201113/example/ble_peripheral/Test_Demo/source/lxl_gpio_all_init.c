#include "lxl_gpio_all_init.h"
/***********************************************************************************
lxl_gpio_all_init.o(+RO)   
lxl_gpio_adc_count.o(+RO)   
lxl_gpio_lcd_fun.o(+RO)	 
lxl_gpio_pwm_count.o(+RO)	 
lxl_gpio_sensor_func.o(+RO)   
lxl_log_uart_func.o(+RO) 		
lxl_simple_profile_func.o(+RO)			
lxl_ui_count_func.o(+RO)	 


************************************************************************************/

uint8 lxl_ui_count_TaskID ;       //个人ui事件的id
uint8 lxl_sensor_count_TaskID ;   //外挂器件的通讯处理
uint8 lxl_uart_count_TaskID ;     //uart的数据处理
uint8 lxl_key_count_TaskID ;      //按键处理事件的id
uint8 lxl_adc_count_TaskID ;      //adc检测事件的id

lxl_struct_device_all lxl_device_all_data = {
	.dev_time_power_tick= 0xffffffff ,  //当前设备定时关机 秒为单位

	.dev_power_off_tick=0,    //上电是否直接开关机
	.dev_job_tick=0,          //当前设备工作标志位: 1->工作, 0->关机
	.dev_sleep_tick=0,        //当前设备是否低功耗状态
	.dev_batt_value=0,        //当前设备电量
	.dev_charged_state_tick=0,//设备的充电状态
	.dev_ble_connect_en=0,    //当前ble的连接状态
	.dev_test_mode_en=0,      //当前为是否进入测试模式

	.dev_boot_up_tick=0,      //开机标志位
	.dev_charge_tick=0,       //充电标志位
	.dev_buzzer_onoff=0,      //蜂鸣器开关


	.dev_app_set_data_0=0,    //app设置断开连接报警的要求
	.dev_app_set_data_1=3,    //app设置连接后报警的要求

	.dev_app_push_data={0},   //app下发数据缓存
};   //当前设备的全部状态标志位




void my_flash_tick_count_func()     //用于判断flash中的数据
{
#if 0
	static u32 power_data_tick_11 = 0 ;  //判断上电还是开机数据一
	static u32 power_data_tick_22 = 0 ;  //判断上电还是开机数据二

	power_data_tick_22 = 0xffffffff ;
	power_data_tick_11 = 0xffffffff ;
	osal_snv_read(LXL_FLASH_HEAT_36_ADDR,4,&power_data_tick_11) ;   //把校准数据读出
	osal_snv_read(LXL_FLASH_HEAT_42_ADDR,4,&power_data_tick_22) ;   //把校准数据读出
#endif



}
void my_gpio_all_init_func()
{
#if 1
    LOG("3:	") ;

    hal_gpio_pin_init(MY_GPIO_KEY_1,IE) ;                  //设置为输入
    hal_gpio_pull_set(MY_GPIO_KEY_1,STRONG_PULL_UP) ;      //设置当前io的上下拉
    hal_gpioin_enable(MY_GPIO_KEY_1) ;
	hal_gpioin_register(MY_GPIO_KEY_1, lxl_pin_event_handler_key, lxl_pin_event_handler_key);

    hal_gpio_pin_init(MY_GPIO_ADC_5V_TEST,IE) ;                  //设置为输入
    hal_gpio_pull_set(MY_GPIO_ADC_5V_TEST,PULL_DOWN) ;      //设置当前io的上下拉
    hal_gpioin_enable(MY_GPIO_ADC_5V_TEST) ;
	hal_gpioin_register(MY_GPIO_ADC_5V_TEST, lxl_pin_event_handler_key, lxl_pin_event_handler_key);
	if(hal_gpio_read(MY_GPIO_ADC_5V_TEST)){
		lxl_device_all_data.dev_power_off_tick =1 ;
	}else {
		lxl_device_all_data.dev_power_off_tick =0 ;
	}
	
	hal_gpio_fmux(MY_GPIO_BACK_LED_1,Bit_ENABLE) ;            
	hal_gpio_pin_init(MY_GPIO_BACK_LED_1,OEN) ;               
	hal_gpio_write(MY_GPIO_BACK_LED_1,0) ;
	
    hal_gpio_fmux(MY_GPIO_PWM_M0_1,Bit_ENABLE) ;              
    hal_gpio_pin_init(MY_GPIO_PWM_M0_1,OEN) ;                  
    hal_gpio_write(MY_GPIO_PWM_M0_1,0) ;  
    lxl_pwm_timeout_handle() ;  
    
	hal_gpio_cfg_analog_io(MY_GPIO_ADC_BATT_1, Bit_ENABLE);
	hal_gpio_cfg_analog_io(MY_GPIO_ADC_MOTO_1, Bit_ENABLE);

	hal_pwrmgr_register(MOD_USR8, NULL, NULL);                //进入低功耗函数
	hal_pwrmgr_lock(MOD_USR8) ;                               //退出低功耗

	lxl_proc_power_onoff(0,50) ;							  //短按开机
	lxl_led_enter_mode_1(1) ;

	lxl_device_all_data.dev_time_power_tick= LXL_POWER_OFF_TIME_NUM ;
	lxl_adc_all_data.adc_enable = 1 ;
	lxl_device_all_data.dev_batt_value= 100 ;
	lxl_all_func_data.dev_onoff_tick =1 ;
	lxl_all_func_data.dev_mode_tick = 17 ;
	//my_flash_tick_count_func() ;   //获取flash中的数据

	LOG("1:  ") ;
#endif 
}

void lxl_ui_init_task_func( uint8 task_id )
{
    lxl_ui_count_TaskID = task_id;
    my_gpio_all_init_func() ;
	if(lxl_device_all_data.dev_job_tick){
   		osal_start_timerEx(lxl_ui_count_TaskID, MY_UI_10MS_EVENT_TICK, 20);   //跳入UI处理第一次
		LOG("MY_INIT_GPIO_111111:\n") ;
	}
}
void lxl_sensor_init_task_func( uint8 task_id )
{
    //lxl_sensor_count_TaskID = task_id;
    //osal_start_timerEx(lxl_sensor_count_TaskID, MY_SENSOR_1_EVENT_TICK, 50);   //跳入lcd屏幕处理第一次
    //LOG("MY_LCD_GPIO_111111:\n") ;
}
void lxl_uart_init_task_func( uint8 task_id )
{
//    lxl_uart_count_TaskID = task_id;
//    osal_start_timerEx(lxl_uart_count_TaskID, MY_UART_1_EVENT_TICK, 50);   //跳入lcd屏幕处理第一次
    //LOG("MY_LCD_GPIO_111111:\n") ;
}
void lxl_key_init_task_func( uint8 task_id )
{
    lxl_key_count_TaskID = task_id;
	if(lxl_key_all_data.key_batt_power_tick == 0&&lxl_device_all_data.dev_job_tick){
   		osal_start_timerEx(lxl_key_count_TaskID, MY_KEY_1_EVENT_TICK, 100);   // 跳入按键处理第一次
	}
    //LOG("MY_KEY_GPIO_111111:\n") ;
}
void lxl_adc_init_task_func( uint8 task_id )
{
    lxl_adc_count_TaskID = task_id;
	if(lxl_device_all_data.dev_job_tick ){
	    osal_start_timerEx(lxl_adc_count_TaskID, MY_ADC_11_EVENT_TICK, 100);   // 跳入按键处理第一次
	    lxl_adc_all_data.adc_enable = 1 ;     //开始检测adc数据
	}
}

u32 lxl_clock_time_exceed_func(u32 ref, u32 span_ms)
{
#if 0
	u32 deltTick ,T0 ;
	T0 = hal_read_current_time();
	deltTick =TIME_DELTA(T0,ref);
	if(deltTick>span_ms){
		return 1 ;
	}else {
		return 0 ;
	}
#else 
	u32 deltTick  = 0 ;
	deltTick = hal_ms_intv(ref) ;
	if(deltTick>span_ms){
		return 1 ;
	}else {
		return 0 ;
	}	
#endif
}

lxl_ui_type_t led_ui_buffer_MS_1[] = {
		//off	  		on      cnt     next mode
		{{0,			0},		0,		0},	   
		{{0,		    2000},  0x01,	3},	
		{{0,		    100},   0x01,	0},
		{{900,		    100},   0xff,	3},	
		{{0,		    1000},  0xff,	4}
};
lxl_ui_type_t led_ui_buffer_MS_2[] = {
		//off	  		on      cnt     next mode
		{{0,			0},		0,		0},	   
		{{160,		    80},    0x02,	0},	
		{{0,		    100},   0x01,	0},		
};
lxl_ui_type_t led_ui_buffer_MS_3[] = {
		//off	  		on      cnt     next mode
		{{0,			0},		0,		0},	   
		{{160,		    80},    0x02,	0},	
		{{0,		    100},   0x01,	0},		
};
lxl_ui_type_t buzzer_ui_buffer_MS[] = {
		//off	  		on      cnt     next mode
		{{0,			0},		0,		0},	   
		{{160,		    80},    0x02,	3},	
		{{0,		    100},   0x01,	0},		
		{{400,		    100},   0xff,	4},	
};

lxl_ui_param_t lxl_led_param_1 ={led_ui_buffer_MS_1};        //当前led显示控制
lxl_ui_param_t lxl_led_param_2 ={led_ui_buffer_MS_2};        //当前led显示控制
lxl_ui_param_t lxl_led_param_3 ={led_ui_buffer_MS_3};        //当前led显示控制
lxl_ui_param_t lxl_buzzer_param ={buzzer_ui_buffer_MS};      //当前蜂鸣器控制

static u8 lxl_BACK_LED_1_value = 0 ;
void lxl_led_beep_1(u8 onOff){
	if(onOff){
		if(lxl_device_all_data.dev_charge_tick == 0){
			lxl_BACK_LED_1_value = 250 ;
			lxl_pwm_ctrl(1,lxl_BACK_LED_1_value) ;
		}else {
			if(lxl_device_all_data.dev_charge_tick == 2){
				lxl_BACK_LED_1_value =  250 ;
			}else {
				static u8 gpio_tick = 0;
				if(gpio_tick){
					lxl_BACK_LED_1_value = lxl_BACK_LED_1_value+2;
					if(lxl_BACK_LED_1_value>252){
						lxl_BACK_LED_1_value = 252 ;
						gpio_tick = 0 ;
					}
				}else {
					lxl_BACK_LED_1_value = lxl_BACK_LED_1_value-2;
					if(lxl_BACK_LED_1_value<3){
						gpio_tick = 1 ;
						lxl_BACK_LED_1_value = 2 ;
					}
				}
			}
			//LOG("value: %d\n",lxl_BACK_LED_1_value) ;
			lxl_pwm_ctrl(1,lxl_BACK_LED_1_value) ;
		}
	}
	else {
		lxl_pwm_off(1) ;
		hal_gpio_write(MY_GPIO_BACK_LED_1, 0);
	}

}
void lxl_led_beep_2(u8 onOff){
	if(onOff){
		//hal_gpio_write(MY_GPIO_BACK_LED_2,0) ;
	}else {
		//hal_gpio_write(MY_GPIO_BACK_LED_2,1) ;
	}
}
void lxl_led_beep_3(u8 onOff){
	if(onOff){
	//	hal_gpio_write(MY_GPIO_BACK_LED_3,0) ;
	}else {
	//	hal_gpio_write(MY_GPIO_BACK_LED_3,1) ;
	}
}
void lxl_buzzer_beep(u8 onOff){
	if(onOff){
		//lxl_pwm_on(0) ;
		//lxl_pwm_ctrl(0,MY_PWM_CMP_VALUE) ;
	}else {
		//lxl_pwm_off(0) ;
	}
}

u32 lxl_calc_led_ui(lxl_ui_param_t *ui_bl)
{
	if(ui_bl->cur_mode == 0) return 0;

	if(ui_bl->next_wakeup_tick - ((hal_systick()|1) + 2) < BIT(30)){
		return ui_bl->next_wakeup_tick;
	}
	if(ui_bl->cur_state && ui_bl->cur_cnt && ui_bl->cur_cnt != 0xff){//!=0 !=0xff ==on
		ui_bl->cur_cnt --;
	}
	if(ui_bl->cur_cnt == 0 ){
		ui_bl->cur_mode = ui_bl->ui_type[ui_bl->cur_mode].next_mode;
		ui_bl->cur_cnt = ui_bl->ui_type[ui_bl->cur_mode].offOn_cnt;
	}
	if(ui_bl->cur_cnt && ui_bl->ui_type[ui_bl->cur_mode].offOn_Ms[0] == 0){
		ui_bl->cur_state = 1;
	}else {
		ui_bl->cur_state = ui_bl->cur_state ? 0: 1;
	}
	//ui_bl->cur_state = ui_bl->cur_state ? 0: 1;
	ui_bl->next_wakeup_tick = hal_systick() + ui_bl->ui_type[ui_bl->cur_mode].offOn_Ms[ui_bl->cur_state] ;
	return ui_bl->next_wakeup_tick;
}

u32 lxl_buzzer_led_ui ()
{
	static u32 led_next_wakeup_timeout ;
	u32 next_led_wakeup_tick_1 = lxl_calc_led_ui(&lxl_led_param_1);
	//u32 next_led_wakeup_tick_2 = lxl_calc_led_ui(&lxl_led_param_2);
	//lxl_led_beep_2(lxl_led_param_2.cur_state);
	//u32 next_led_wakeup_tick_3 = lxl_calc_led_ui(&lxl_led_param_3);
	//lxl_led_beep_3(lxl_led_param_3.cur_state);
	//u32 next_buzzer_wakeup_tick = lxl_calc_led_ui(&lxl_buzzer_param);
	//lxl_buzzer_param.cur_mode = 0 ;
	//if(lxl_buzzer_param.cur_mode){
	//	lxl_device_all_data.dev_sleep_tick = 1 ;
	//}else 
	if(lxl_led_param_1.cur_mode){
		lxl_device_all_data.dev_sleep_tick = 1 ;
		///if(!lxl_led_param_1.cur_state){
		//	lxl_device_all_data.dev_sleep_tick = 0 ;
		//}
		led_next_wakeup_timeout = hal_systick()|1;
	}else {
		if(led_next_wakeup_timeout&& lxl_clock_time_exceed_func(led_next_wakeup_timeout,1000)){
			led_next_wakeup_timeout = 0;
			lxl_device_all_data.dev_sleep_tick = 0 ;
		}
	}

	//if(lxl_device_all_data.dev_sleep_tick==0){
	//	hal_pwrmgr_unlock(MOD_USR8) ;	 //进入低功耗
	//}else{
		hal_pwrmgr_lock(MOD_USR8) ;      //退出低功耗
	//} 
	
	//lxl_buzzer_beep(lxl_buzzer_param.cur_state);
	lxl_led_beep_1(lxl_led_param_1.cur_state);
	return next_led_wakeup_tick_1;
}

void lxl_ui_enter_mode(lxl_ui_param_t *ui_param,u8 mode){
	ui_param->cur_cnt = ui_param->ui_type[mode].offOn_cnt;
	ui_param->cur_mode = mode;
	ui_param->cur_state = 0;
	ui_param->next_wakeup_tick = hal_systick()|1;
}
void lxl_led_enter_mode_1(u8 mode){    //红色
	lxl_ui_enter_mode(&lxl_led_param_1, mode);
}
void lxl_led_enter_mode_2(u8 mode){    //绿色
	//lxl_ui_enter_mode(&lxl_led_param_2, mode);
}
void lxl_led_enter_mode_3(u8 mode){    //黄色
	//lxl_ui_enter_mode(&lxl_led_param_3, mode);
}

void lxl_buzzer_enter_mode(u8 mode){
	//lxl_ui_enter_mode(&lxl_buzzer_param, mode);
}

void lxl_proc_power_onoff(u8 cur_state, u8 power_start_tick)   //开机调用函数
{
	if(lxl_device_all_data.dev_power_off_tick){
		lxl_device_all_data.dev_job_tick = 1 ;
		return ;
	}
	u8 poweron_start_num = power_start_tick ;
	static u32 poweron_start_time_100ms  ;
	if(!cur_state){
		while(poweron_start_num){
			WaitUs(1000);
			if(lxl_clock_time_exceed_func(poweron_start_time_100ms,25)){
				poweron_start_time_100ms = hal_systick()|1;
				if(!hal_gpio_read(MY_GPIO_KEY_1)){
					poweron_start_num-- ;
				}else {
					poweron_start_num = power_start_tick;
					lxl_device_all_data.dev_job_tick = 0 ;
					lxl_proc_powerOff_handle(0) ;
					return ;
				}
			}
		}
		poweron_start_time_100ms =  hal_systick()|1;
		lxl_led_beep_1(1) ;
		while(!hal_gpio_read(MY_GPIO_KEY_1)){
			if(lxl_clock_time_exceed_func(poweron_start_time_100ms,500)){
				lxl_device_all_data.dev_job_tick = 1 ;
				poweron_start_time_100ms = hal_systick()|1;
				return ;
			}
		}
		lxl_device_all_data.dev_job_tick = 1 ;
	}
}

void lxl_power_time_count_time_func()   //定时关机
{
	if(lxl_device_all_data.dev_time_power_tick!=0xffffffff){
		if(lxl_device_all_data.dev_time_power_tick == 0){
			lxl_device_all_data.dev_job_tick = 0 ;  //关机
			lxl_led_enter_mode_1(0) ;
		}else {
			//lxl_device_all_data.dev_test_mode_en = 1 ;
			if(lxl_device_all_data.dev_test_mode_en||lxl_all_func_data.dev_mode_tick){   //测试模式，正式版本中关闭
				lxl_device_all_data.dev_time_power_tick= LXL_POWER_OFF_TIME_NUM ;
			}else {
				if(lxl_device_all_data.dev_ble_connect_en == 0){
					static u32 lxl_dev_power_time_1s ;
					if(lxl_clock_time_exceed_func(lxl_dev_power_time_1s,4000)){
						lxl_dev_power_time_1s = hal_systick()|1;
						lxl_device_all_data.dev_time_power_tick-- ;
					}
				}else {
					lxl_device_all_data.dev_time_power_tick= LXL_POWER_OFF_TIME_NUM ;
				}
			}
			
		}
	}
}

void lxl_proc_powerOff_handle(u8 data_tick)   //关机调用函数
{
	LOG("lxl_proc_powerOff_handle: %d\n",data_tick) ;

	if(data_tick){
		for(u8 i=0; i<6; i++){
		   lxl_led_beep_1(i&0x01);
		   WaitUs(150*1000);
		}
	}
    hal_gpio_pin_init(MY_GPIO_KEY_1,IE) ;                  //设置为输入
    //hal_gpio_cfg_analog_io(MY_GPIO_KEY_1,Bit_DISABLE) ;
    hal_gpio_pull_set(MY_GPIO_KEY_1,STRONG_PULL_UP) ;      //设置当前io的上下拉
    hal_gpioin_enable(MY_GPIO_KEY_1) ;
	hal_gpioin_register(MY_GPIO_KEY_1, lxl_pin_event_handler_key, lxl_pin_event_handler_key);

	hal_gpio_fmux(MY_GPIO_BACK_LED_1,Bit_ENABLE) ;            
	hal_gpio_pin_init(MY_GPIO_BACK_LED_1,OEN) ;               
	hal_gpio_write(MY_GPIO_BACK_LED_1,0) ;     
	
    hal_gpio_fmux(MY_GPIO_PWM_M0_1,Bit_ENABLE) ;              
    hal_gpio_pin_init(MY_GPIO_PWM_M0_1,OEN) ;                  
    hal_gpio_write(MY_GPIO_PWM_M0_1,0) ; 
    lxl_pwm_timeout_handle() ;

	hal_gpio_cfg_analog_io(MY_GPIO_ADC_BATT_1, Bit_ENABLE);
	hal_gpio_cfg_analog_io(MY_GPIO_ADC_MOTO_1, Bit_ENABLE);
    hal_gpio_pin_init(MY_GPIO_ADC_5V_TEST,IE) ;                  //设置为输入
    hal_gpio_pull_set(MY_GPIO_ADC_5V_TEST,PULL_DOWN) ;      //设置当前io的上下拉
    hal_gpioin_enable(MY_GPIO_ADC_5V_TEST) ;
	hal_gpioin_register(MY_GPIO_ADC_5V_TEST, lxl_pin_event_handler_key, lxl_pin_event_handler_key);

    hal_pwrmgr_unlock(MOD_USR8) ;	 //进入低功耗
	if(data_tick){
		while(!hal_gpio_read(MY_GPIO_KEY_1))
				WaitUs(10*1000);
	}
	uint8 initial_advertising_enable = FALSE;
	GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
    u32 data_taskid_tick = 1 ;
    for(u8 i=0; i<15; i++){
    	//LOG("TaskID_STOP_DATA: %d\n",data_taskid_tick) ;
		osal_stop_timerEx(lxl_ui_count_TaskID,data_taskid_tick) ;
		osal_stop_timerEx(lxl_sensor_count_TaskID,data_taskid_tick) ;
		osal_stop_timerEx(lxl_uart_count_TaskID,data_taskid_tick) ;
		osal_stop_timerEx(lxl_key_count_TaskID,data_taskid_tick) ;
		osal_stop_timerEx(lxl_adc_count_TaskID,data_taskid_tick) ;
		data_taskid_tick = data_taskid_tick<<1 ;
	}
}

void lxl_SimpleProfile_SetParameter(uint8 param, uint8 len, void *value )
{
	//SimpleProfile_SetParameter(param,len,value) ;
	simpleProfile_Notify(param,len,value) ;
	LOG("APP_DATA: %d  ",len) ;
	lxl_uart_printf_hex(value,len) ;
}


