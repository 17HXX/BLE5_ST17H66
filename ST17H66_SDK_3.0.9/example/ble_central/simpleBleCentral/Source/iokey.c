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
#include "led.h"

#include "clock.h"
#include "iokey.h"
#include "pwrmgr.h"
#include "ui.h"




uint16 key_msg_val;
uint8 curr_key_number = NO_KEY;
uint8 last_key_number;
void ui_key_gpio_wkup_init(void);
void ui_pin_event_handler_key(GPIO_Pin_e pin,IO_Wakeup_Pol_e type);



uint8 ui_get_cur_key( void)
{
		static uint8 key_val = NO_KEY;
		key_val = 0x00;
		if(hal_gpio_read(UI_KEY0_IO)) {
				key_val |= BIT(0);
		}
	

    return key_val;
}

void ui_key_scan(void)
{
    static uint8 last_curr_key_number = 0;
    static uint8 key_long_cnt;
		static uint8 key_more_click;
		static uint8 key_last_effect_val;
		static uint16 key_scan_time_release;
		
    curr_key_number = ui_get_cur_key();
    if(curr_key_number != last_curr_key_number) {
        if(curr_key_number) {
            ui_led_key();
            key_long_cnt =0;
						key_msg_val = 0;
						if(key_last_effect_val != curr_key_number){	
							key_last_effect_val = curr_key_number;
							key_more_click = 0;	/// 不同按键快速按下，清0操作
							LOG("\n----- clear 0");
						}
        }
				else {
					if(key_long_cnt < KEY_SHORT_CNT) {						
							key_more_click++;///short key  max 0x0f
//							LOG("\n short key = %d",key_more_click);
					}				
				}
        last_curr_key_number = curr_key_number;
    }

    if(curr_key_number) {
        if(key_long_cnt < KEY_LONG_CNT) {
            key_long_cnt++;
            if(key_long_cnt == KEY_LONG_CNT) {
                key_msg_val = key_last_effect_val|((key_more_click&0x0f)<<8)|KEY_LONG;
								osal_start_timerEx(ui_task_id, UI_KEY_MSG_EVT, 1);	/// msg out
//								LOG("\n                   KEY_LONG = %x",key_msg_val);
            }
        }
				key_scan_time_release = KEY_MORE_KEY_TIMEOUT_CNT;	///more key timeout
		}
    else {
				if(key_msg_val & KEY_LONG) {
						key_more_click = 0;
						key_msg_val =	key_last_effect_val|KEY_LONG_UP;
						osal_start_timerEx(ui_task_id, UI_KEY_MSG_EVT, 1);				/// msg out
//						LOG("\n                   KEY_LONG_UP = %x",key_msg_val);
				}
    }
		if(key_scan_time_release) {
				key_scan_time_release--;
				if(key_scan_time_release == 0) {
					key_msg_val = key_last_effect_val|((key_more_click&0x0f)<<8)|KEY_SHORT_CLICK;					
					osal_start_timerEx(ui_task_id, UI_KEY_MSG_EVT, 1);	/// msg out
//					LOG("\n short key = %x",key_msg_val);					
					key_more_click = 0;
					osal_stop_timerEx(ui_task_id,UI_KEY_SCAN_EVT);
					hal_gpioin_register(UI_KEY0_IO, ui_pin_event_handler_key, ui_pin_event_handler_key);				
				}
				else {
					osal_start_timerEx(ui_task_id, UI_KEY_SCAN_EVT, KEY_SCAN_INTERVAL);
				}
				
				extern uint32 key_timeout_release;
				key_timeout_release = KEY_MORE_KEY_TIMEOUT_CNT;
				osal_start_timerEx(ui_task_id, UI_POWER_OFF_EVT, 200);				
		}
		else {
			key_scan_time_release = KEY_MORE_KEY_TIMEOUT_CNT;				/// if key time <30ms  ;also need reset key intrupt
			osal_start_timerEx(ui_task_id, UI_KEY_SCAN_EVT, KEY_SCAN_INTERVAL);
			LOG("\n--------------------------------------------------------\n");
		}
}

void ui_pin_event_handler_key(GPIO_Pin_e pin,IO_Wakeup_Pol_e type)
{
    hal_gpioin_register(UI_KEY0_IO, NULL, NULL);
    hal_gpioin_register(UI_KEY1_IO, NULL, NULL);
    hal_gpioin_register(UI_KEY2_IO, NULL, NULL);
    osal_start_timerEx(ui_task_id, UI_KEY_SCAN_EVT, KEY_SCAN_INTERVAL);
}

void ui_power_off_fun(void)
{
//		WEAK_PULL_UP
//		PULL_DOWN
//		STRONG_PULL_UP
		
		LOG("\nui_power_off_fun");
		pwroff_cfg_t hidMouse_pcfg[3] = {
				UI_KEY0_IO,	POL_FALLING, 1,
				UI_KEY1_IO,	POL_FALLING, 1,
				UI_KEY2_IO,	POL_FALLING, 1,
		};
		hal_pwrmgr_enter_standby(hidMouse_pcfg,3);		
}

void ui_key_gpio_wkup_init(void)
{
    LOG("ui_key_gpio_init");
}

void ui_io_key_init()
{
		hal_gpio_fmux(UI_KEY0_IO, Bit_DISABLE);
		hal_gpio_pin_init(UI_KEY0_IO,IE) ;
		hal_gpio_pull_set(UI_KEY0_IO,PULL_DOWN) ;	
}

void ui_key_gpio_init(void)
{
		ui_io_key_init();
		hal_gpioin_register(UI_KEY0_IO, ui_pin_event_handler_key, NULL);	
		osal_start_timerEx(ui_task_id, UI_KEY_SCAN_EVT, 1);/// 唤醒开机也要去读取按键		
}



