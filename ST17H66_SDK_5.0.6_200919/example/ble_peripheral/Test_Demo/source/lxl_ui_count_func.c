#include "lxl_gpio_all_init.h"

uint16 lxl_ui_all_func(uint8 task_id, uint16 events)
{
	VOID task_id; // OSAL required parameter that isn't used in this function
	if ( events & SYS_EVENT_MSG )
	{
		uint8 *pMsg;
		if ( (pMsg = osal_msg_receive( task_id )) != NULL )
		{
			lxl_key_count_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );
			VOID osal_msg_deallocate( pMsg );
		}
		return (events ^ SYS_EVENT_MSG);
	}
	if(events&MY_UI_POWER_EVENT_TICK){
		if(lxl_device_all_data.dev_job_tick == 0){	 //关机状态下,重新初始化io，以及时间状态
			NVIC_SystemReset() ;
		}
		return (events ^ MY_UI_POWER_EVENT_TICK);
	}

	if(events&MY_UI_10MS_EVENT_TICK){
		if(lxl_device_all_data.dev_job_tick){
			lxl_buzzer_led_ui () ;               //运行led和蜂鸣器
			lxl_pwm1_count_value_func() ; 
			lxl_gpio_charge_count_func() ;       //充电判断
			lxl_power_time_count_time_func() ;   //定时关机
			osal_start_timerEx(lxl_ui_count_TaskID, MY_UI_10MS_EVENT_TICK, 20);
		}else {
			lxl_proc_powerOff_handle(1) ;
		}
		return (events ^ MY_UI_10MS_EVENT_TICK);
	}

	if(events&MY_UI_100MS_EVENT_TICK){
		u8 pValue[20] = {0} ;
		uint8 push_data_app[20] = {0x00} ;	  //上传数据到app中
		for(u8 i=0; i<20; i++){
			pValue[i] = lxl_device_all_data.dev_app_push_data[i] ;
		}
		LOG("APP_DATA: %d -",lxl_device_all_data.dev_app_push_data[39]) ;
		lxl_uart_printf_hex(pValue,lxl_device_all_data.dev_app_push_data[39]) ;

		if(lxl_device_all_data.dev_charge_tick){   //上传充电标志位、以及电量
			pValue[0] = 0x55 ;  pValue[1] = 0xf2 ;  pValue[2] = 0x01 ;
			pValue[3] = lxl_device_all_data.dev_batt_value;
			pValue[4] = 0xaa ;
			lxl_SimpleProfile_SetParameter(0,5,pValue) ;
			return (events ^ MY_UI_100MS_EVENT_TICK);
		}

		if(pValue[0]==0xa1&&pValue[1]==0x01&&pValue[2]){   //上传按键值
			
		}
		else{  //错误消息
			pValue[0] = 0x55 ;  pValue[1] = 0xff ;  pValue[2] = 0x01 ;
			pValue[3] = 0xff;
			pValue[4] = 0xaa ;
			lxl_SimpleProfile_SetParameter(0,5,pValue) ;
		}   

		return (events ^ MY_UI_100MS_EVENT_TICK);
	}


    osal_start_timerEx(lxl_ui_count_TaskID, MY_UI_10MS_EVENT_TICK, 1000);
    return 0 ;
}


uint16 lxl_key_all_func(uint8 task_id, uint16 events)
{
	VOID task_id; // OSAL required parameter that isn't used in this function
	if ( events & SYS_EVENT_MSG )
	{
		uint8 *pMsg;
		if ( (pMsg = osal_msg_receive( task_id )) != NULL )
		{
			lxl_key_count_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );
			VOID osal_msg_deallocate( pMsg );
		}
		return (events ^ SYS_EVENT_MSG);
	}
    
    if(MY_KEY_1_EVENT_TICK&events){// 按键处理
		static u8 lxl_last_button_pressed = 0 ;      //当前按下只能进入一次
		static u8 lxl_last_button_numbale = 0 ;      //当前按键的值
		static u8 lxl_long_key_enable = 0 ;          //当前按下只能进入一次
		
		static u32 lxl_last_button_press_time = 0 ;  //所有按键抬起的时间
		static u32 lxl_last_button_release_time = 0 ;//所有按键按下的时间
		static u32 lxl_key_time_temp = 0 ;           //当前time以20ms加一作为时间单位

		if(lxl_device_all_data.dev_charge_tick){   //上传充电标志位、以及电量
			u8 pValue[20] ;
			pValue[0] = 0x55 ;  pValue[1] = 0xf2 ;  pValue[2] = 0x01 ;
			pValue[3] = lxl_device_all_data.dev_batt_value;
			pValue[4] = 0xaa ;
			lxl_SimpleProfile_SetParameter(0,5,pValue) ;
			return (events ^ MY_KEY_1_EVENT_TICK);
		}
		
		//LOG("key: %d ",lxl_last_button_numbale) ;
		lxl_key_time_temp = hal_systick()|1;
		if(lxl_key_all_data.key_dowm_tick_0){
			if(lxl_last_button_numbale&&lxl_long_key_enable && lxl_clock_time_exceed_func(lxl_last_button_press_time,2100)){
				lxl_long_key_enable = 0 ;
				lxl_key_all_data.key_repeatedly_tick = 0 ;
				lxl_last_button_numbale = 0 ;
				LOG("KEY_UP_2222: %d\n",lxl_last_button_numbale) ;
				lxl_device_all_data.dev_job_tick = 0 ;
				write_reg(0x4000f030,1) ;
			}
		}else { 
			lxl_long_key_enable = 1 ;
		}

		if(lxl_key_all_data.key_dowm_tick_0){
			if(!lxl_last_button_pressed&&lxl_clock_time_exceed_func(lxl_last_button_release_time,20)){
				lxl_last_button_pressed = 1 ;
				lxl_last_button_numbale = lxl_key_all_data.key_dowm_tick_0 ;
				lxl_device_all_data.dev_sleep_tick = 1;

				//LOG("KEY_DOWM_0000: %d\n",lxl_last_button_numbale) ;
				lxl_last_button_press_time = lxl_key_time_temp ;
				lxl_device_all_data.dev_time_power_tick= LXL_POWER_OFF_TIME_NUM ;   //刷新每次工作时间
			}
		}else {
			if(lxl_last_button_pressed&&lxl_clock_time_exceed_func(lxl_last_button_press_time,20) ){
				lxl_last_button_release_time = lxl_key_time_temp;
				lxl_last_button_pressed = 0 ;
			}
		}

	    //if(lxl_key_all_data.key_repeatedly_tick&&lxl_key_all_data.key_dowm_time_0&&lxl_clock_time_exceed_func(lxl_key_all_data.key_dowm_time_0,400)){
		//	LOG("KEY_UP_3333: %d\n",lxl_key_all_data.key_repeatedly_tick) ;
		//	lxl_key_all_data.key_dowm_time_0= 0 ;
		//	lxl_key_all_data.key_repeatedly_tick = 0;
		//	lxl_last_button_numbale = 0 ;
	    //}

	    if(lxl_last_button_numbale&&!lxl_key_all_data.key_dowm_tick_0&& lxl_clock_time_exceed_func(lxl_last_button_press_time,20)){
			lxl_led_enter_mode_1(2) ;

			lxl_key_all_data.key_repeatedly_tick = 0;
			lxl_key_all_data.key_dowm_time_0 = lxl_key_time_temp ;    //计算下一次按键时间
			LOG("KEY_UP_1111: %d\n",lxl_last_button_numbale) ;
			lxl_last_button_numbale = 0 ;
	    }

        if(lxl_key_all_data.key_dowm_tick_0){
			osal_start_timerEx(lxl_key_count_TaskID, MY_KEY_1_EVENT_TICK, 40);
        }
        return (events ^ MY_KEY_1_EVENT_TICK);
    }
    //osal_start_timerEx(lxl_key_count_TaskID, MY_KEY_1_EVENT_TICK, 2000);
    return 0 ;
}


uint16 lxl_adc_all_func(uint8 task_id, uint16 events)
{
	VOID task_id; // OSAL required parameter that isn't used in this function

	u8 push_data_app[20] = {0x00};
	if ( events & SYS_EVENT_MSG )
	{
		uint8 *pMsg;
		if ( (pMsg = osal_msg_receive( task_id )) != NULL )
		{
			lxl_key_count_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );
			VOID osal_msg_deallocate( pMsg );
		}
		return (events ^ SYS_EVENT_MSG);
	}

	if(MY_ADC_11_EVENT_TICK&events){        //开始
		hal_adc_init() ;
		normal_adc_init() ;
		hal_adc_start() ;
		//LOG("8") ;

		osal_start_timerEx(lxl_adc_count_TaskID, MY_ADC_11_EVENT_TICK, 3*1000);
		return (events ^ MY_ADC_11_EVENT_TICK);
	}
	if(MY_ADC_22_EVENT_TICK&events){        //停止
		hal_adc_stop() ;
	
		return (events ^ MY_ADC_22_EVENT_TICK);
	}

	//osal_start_timerEx(lxl_adc_count_TaskID, MY_ADC_11_EVENT_TICK, 10);
	return 0;
}

lxl_struct_key_all lxl_key_all_data = {
    .key_dowm_time_0 =0,            
    .key_dowm_tick_0 =0,    
    .key_batt_power_tick =0,   //记忆当前开机状态
} ;    

void lxl_pin_event_handler_key(GPIO_Pin_e pin,IO_Wakeup_Pol_e type)
{
    static u8 key_dowm_tick_temp = 0 ;
    switch(pin)
    { 
        case MY_GPIO_KEY_1:   //按键检测
            if(type == NEGEDGE){
                lxl_key_all_data.key_dowm_tick_0 |= BIT(0);
                if(lxl_device_all_data.dev_job_tick){
					lxl_device_all_data.dev_sleep_tick = 1 ;
					osal_start_timerEx(lxl_ui_count_TaskID, MY_UI_10MS_EVENT_TICK, 10);
                }else {
					osal_start_timerEx(lxl_ui_count_TaskID, MY_UI_POWER_EVENT_TICK, 10);
                }
                LOG("4:\n") ;
            }else {
                lxl_key_all_data.key_dowm_tick_0 &= (!BIT(0)) ;
                LOG("5:\n") ;
            }
            osal_start_timerEx(lxl_key_count_TaskID, MY_KEY_1_EVENT_TICK, 10);
        break;
        case MY_GPIO_ADC_5V_TEST:   //充电检测
            if(type == NEGEDGE){
				osal_start_timerEx(lxl_ui_count_TaskID, MY_UI_POWER_EVENT_TICK, 10);
            }else {
				//进入充电状态，其他关闭只闪烁灯，以及充满判断
            }
        break;
        default:
            LOG("MY_key_dowm_0000:\n") ;
        break;
    }
}



void lxl_key_count_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
    switch ( pMsg->event )
    {  
	    default:
        // do nothing
        break;
    }
}


