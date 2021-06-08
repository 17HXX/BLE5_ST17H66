/**************************************************************************************************
*******
**************************************************************************************************/

/******************************************************************************


 *****************************************************************************/


/*********************************************************************
 * INCLUDES
 */

#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "gatt.h"
#include "hci.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "gatt_profile_uuid.h"
#include "linkdb.h"
#include "peripheral.h"
#include "gapbondmgr.h"
#include "peripheral.h"


#include "global_config.h"

#include "ll.h"
#include "ll_common.h"
#include "ll_def.h"
#include "log.h"

#include "clock.h"

#include "pwrmgr.h"
#include "led.h"
#include "ui.h"

static uint8 led_cur_status;

uint8 ui_led_mode;

extern uint8 get_cur_gap_status(void);


enum{
	UI_LED_POWEROFF = 0,
	UI_LED_POWERON,
	UI_LED_ADV,	
	UI_LED_ADV_KEY,
	UI_LED_CON_KEY,	
	UI_LED_ADV_PAIRING,	
	UI_LED_CON_PAIRING,	
};

typedef struct{
	uint16 	offOn_Ms[2];
	uint8 		offOn_cnt;	// if offCnt == 0xff, indicate that this state would continues until change to other state.
	uint8 		next_mode;
}ui_type_t;

typedef struct{
	ui_type_t *ui_type;
	uint8 	cur_mode; //current mode
	uint8 	cur_cnt;// current cnt left
	uint8 	cur_state;//current state  e.g. led on/off  buzzer on/off
	uint32	next_wakeup_tick;
}ui_param_t;

ui_type_t led_ui_buffer_MS[] = {
		//off	  		on      cnt		next mode
		{{0,				0},			0,			0},			//power off  and connect state
		{{10,				2000},  0x01,		2},		//power on:0ms /2S: 1time
		{{3000,			60},		0xff,			0},		//adv : 3S  /50ms  :2950 + 50
		{{0,				60},		1,			2},		//button adv
		{{0,				60},		1,			0},		//button con
		{{500,			60},    0xff,		2},		// pairing mode adv
		{{180,			60},    0xff,		0},		// alert mode con
};

ui_param_t led_param ={led_ui_buffer_MS};

void led_on(void)
{
	hal_gpio_write(UI_IO_LED_BT,1) ;	
	led_cur_status = 1;
}

void led_off(void)
{
	hal_gpio_write(UI_IO_LED_BT,0) ;	
	led_cur_status = 0;
}

void led_onOff(uint8 flag)
{
//	LOG_LED("\n                                                        led %d",flag);
	led_cur_status = flag;
	hal_gpio_write(UI_IO_LED_BT,led_cur_status);	
}

void ui_enter_mode(ui_param_t *ui_param,uint8 mode) {
	if(ui_param->cur_mode == 1 || ui_param->cur_mode == mode){
//		if(ui_param->cur_mode == mode){
//			LOG("111");
			return;
		}
	
		LOG_LED("\n led_enter_mode------------------%d",mode);	

		ui_param->cur_cnt = ui_param->ui_type[mode].offOn_cnt;
		ui_param->cur_mode = mode;
		ui_param->cur_state = 0;	
		if(ui_param->ui_type[mode].offOn_Ms[0] == 0){
				ui_param->cur_state = 1;	
				osal_start_timerEx(ui_task_id,UI_LED_EVT,ui_param->ui_type[mode].offOn_Ms[1]);	 
				if(ui_param->ui_type[mode].offOn_Ms[1]){
					led_onOff(1);
				}
				else {
						LOG_LED("\n 333333osal_stop_timerEx------------------UI_LED_EVT");	
					osal_stop_timerEx(ui_task_id, UI_LED_EVT);	
				}
		}
		else {
				osal_start_timerEx(ui_task_id,UI_LED_EVT,ui_param->ui_type[mode].offOn_Ms[0]);	 
		}	
}

void ui_led_set_mode(uint8 led_mode)
{
//		ui_enter_mode(&led_param,led_mode);		
}

uint32 calc_led_ui(ui_param_t *ui_bl)
{
	if(ui_bl->cur_mode == 0) return 0;

	if(ui_bl->cur_state && ui_bl->cur_cnt && ui_bl->cur_cnt != 0xff){//!=0 !=0xff ==on
		ui_bl->cur_cnt --;
	}
	if(ui_bl->cur_cnt == 0 ) {
		ui_bl->cur_mode = ui_bl->ui_type[ui_bl->cur_mode].next_mode;
		ui_bl->cur_cnt = ui_bl->ui_type[ui_bl->cur_mode].offOn_cnt;
	}
	ui_bl->cur_state = ui_bl->cur_state ? 0: 1;
	
	if(ui_bl->cur_mode == 0) {
		LOG_LED("\n osal_stop_timerEx------------------UI_LED_EVT");	
		osal_stop_timerEx(ui_task_id, UI_LED_EVT);		
	}
	else {
		osal_start_timerEx(ui_task_id,UI_LED_EVT,ui_bl->ui_type[ui_bl->cur_mode].offOn_Ms[ui_bl->cur_state]);	/// 3s timeout		
	}	
	return ui_bl->cur_state;
}

void ui_led_key(void)
{
//	LOG_LED("\n ui_led_key------------------");	
	if(!get_cur_gap_status())
	{
		ui_led_set_mode(UI_LED_ADV_KEY);
	}
	else {
		ui_led_set_mode(UI_LED_CON_KEY);
	}	
}

void ui_led_pairing(void)
{
//	LOG_LED("\n ui_led_pairing------------------");	
	ui_led_set_mode(UI_LED_ADV_PAIRING);	
}


void ui_led_adv(void)
{
//	LOG_LED("\n ui_led_adv------------------");	
	ui_led_set_mode(UI_LED_ADV);	
}

void ui_led_con(void)
{
//	LOG_LED("\n ui_led_con------------------");		
	ui_led_set_mode(UI_LED_CON_KEY);
}

void ui_led_evt(void)
{
//	LOG_LED("\n led evt- ui_led_mode = %d",led_param.cur_mode);
	led_onOff(calc_led_ui(&led_param));	
}

void ui_led_init(void)
{
	hal_gpio_pin_init(UI_LED0_IO,OEN) ;            //output
	hal_gpioretention_register(UI_LED0_IO);
	hal_gpio_write(UI_LED0_IO,0) ;			
	ui_led_set_mode(UI_LED_POWERON);
}
