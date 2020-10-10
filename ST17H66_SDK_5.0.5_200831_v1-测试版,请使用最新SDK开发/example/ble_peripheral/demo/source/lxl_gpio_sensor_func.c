#include "lxl_gpio_all_init.h"

lxl_dev_func_data lxl_all_func_data = {
    .dev_onoff_tick=0,    //��ǰ�豸�Ŀ���
	.dev_mode_tick=0,     //��ǰ�豸��ģʽ
	.dev_mode_class=0,    //ģʽ��ǿ�ȵȼ�
	.dev_timeout_tick=0,  //��ǰ�豸�Ĺ���ʱ������λ��
	.dev_pwm_value=0,     //��ǰ�豸��pwmֵ
	.dev_warm_tick=0,	  //��ǰ���ȿ���
	.dev_rtc_t=0,		  //��ǰ�豸���¶ȣ���
};   //��ǰ�豸�Ĺ��ܱ�־λ

void lxl_pwm1_value_count_func(u8 pwm_duty)  //����_pwm1��ռ�ձ�
{
	if(pwm_duty>100){
		pwm_duty = 100 ;
	}else if(pwm_duty == 0){
		lxl_pwm_off(PWM_CH0) ;
		return ;
	}
	u16 pwm_value_tick = (MY_PWM_CNTTOP_VALUE*pwm_duty)/100 ;
	pwm_value_tick = MY_PWM_CNTTOP_VALUE - pwm_value_tick ;
	lxl_pwm_ctrl(PWM_CH0,pwm_value_tick) ;
	lxl_all_func_data.dev_pwm_value = pwm_duty ;
	lxl_all_func_data.dev_mode_class = 0 ;
}

void lxl_pwm1_count_onoff_timeout_func(u32 on_time,u32 off_time,u8 pwm_duty)
{
	//on_time:��λΪ1ms      pwm_duty:ռ�ձ�-(0-100)
	static u32 lxl_pwm1_timeout_tick_on ;
	static u32 lxl_pwm1_timeout_tick_off ;

	if(on_time == 0xffffffff){     //����
		lxl_pwm1_value_count_func(pwm_duty) ;
	}else if(on_time == 0){  //����
		lxl_pwm1_value_count_func(0) ;
	}else {                  //���ݿ���ʱ��������
		if(lxl_clock_time_exceed_func(lxl_pwm1_timeout_tick_on,on_time)){   //����
			lxl_pwm1_timeout_tick_off = hal_systick()|1;
			lxl_pwm1_value_count_func(pwm_duty) ;
		}
		if(lxl_clock_time_exceed_func(lxl_pwm1_timeout_tick_off,off_time)){ //�ر�
			lxl_pwm1_timeout_tick_on = hal_systick()|1;
			lxl_pwm1_value_count_func(0) ;
		}
	}
}

void lxl_pwm1_count_value_func()   //�������ģʽ����"1"
{
	static u32 lxl_pwm1_mode_timeout_tick ;      //��ǰ��ģʽ�µ�Сģʽ��ʱʱ��
	static u8 lxl_pwm1_mode_numable_temp = 0 ;   //��ǰ��ģʽ�µ�Сģʽ��־λ
	static u8 lxl_pwm1_mode_duty_temp = 0 ;      //��ǰ��ģʽ�µ�Сģʽ��ռ�ձ�
}





#if 0
uint16 lxl_snesor_all_func(uint8 task_id, uint16 events)
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
	u8 wt588_data_tick[20] = {0} ;
    if(MY_SENSOR_1_EVENT_TICK&events){

		//osal_start_timerEx(lxl_sensor_count_TaskID, MY_SENSOR_1_EVENT_TICK, 5000);
		return (events ^ MY_SENSOR_1_EVENT_TICK);    
    }
    //osal_start_timerEx(lxl_lcd_count_TaskID, MY_LCD_1_EVENT_TICK, 2000);
    return 0 ;
}
#endif


