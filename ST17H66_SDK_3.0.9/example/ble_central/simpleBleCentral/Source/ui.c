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

#include "pwm.h"

#include "ll.h"
#include "ll_common.h"
#include "ll_def.h"
#include "log.h"

#include "clock.h"
#include "ui.h"

#include "led.h"
#include "simpleBLECentral.h"



uint8 ui_task_id;
static uint32 ui_interval = 400;		///2:4mA   10:1.5ms     3000
static uint32 n2ms_cnt;

uint32 key_timeout_release = 12;

//static bool flag;
uint32 ui_pairing_mode_cnt = 0;

static uint32 del_repeat_key;
static uint8 last_sw_status;
static uint8 last_led_status;
uint32 led_flash_cnt = 0;
extern uint8 is_pair_mode;

#define UI_LED_MAX_PAIRING_CNT		(60*1000/400)
#define UI_LED_INIT_WHITE_LIST		16

void ui_set_sw_onOff(uint8 onOff)
{
	hal_gpio_write(UI_SW_IO,onOff) ;
	last_sw_status = onOff;
}

void ui_set_led_onOff(uint8 onOff)
{
	hal_gpio_write(UI_LED0_IO,onOff) ;
	last_led_status = onOff;
}

void ui_time_fun()
{
		if(led_flash_cnt){
			led_flash_cnt--;
			ui_set_led_onOff(led_flash_cnt&0x01);
			if(led_flash_cnt == 0){
				is_pair_mode = 0;
				ui_set_led_onOff(0);
			}
		}
}

void ui_n2ms_stop()
{
    osal_stop_timerEx(ui_task_id, UI_TIME_EVT);
}

void ui_n2ms_restart()
{
    osal_start_timerEx(ui_task_id,UI_TIME_EVT,ui_interval);
}
static void UI_Task_ProcessOSALMsg( osal_event_hdr_t *pMsg );
uint16 UI_ProcessEvent( uint8 task_id, uint16 events )
{
    if ( events & SYS_EVENT_MSG )
    {
        uint8 *pMsg;
        if ( (pMsg = osal_msg_receive( ui_task_id )) != NULL )
        {
            UI_Task_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );
            // Release the OSAL message
            VOID osal_msg_deallocate( pMsg );
        }
        // return unprocessed events
        return (events ^ SYS_EVENT_MSG);
    }
		
    if(events & UI_TIME_EVT) {
        ui_time_fun();
        osal_start_timerEx(ui_task_id,UI_TIME_EVT,ui_interval);
        return ( events ^ UI_TIME_EVT );
    }
		
    if(events & UI_RECEIVE_MSG_EVT) {	//// PS:收到正确广播数据在这里处理，控制开关  开关io 详见ui.h
				extern uint8 rec_key_val;

				AT_LOG("\n------------ rec_key_val = %x:",rec_key_val);			
		
				if(rec_key_val >0x0C){
					ui_set_sw_onOff(!last_sw_status);							
				}
				else 
					ui_set_sw_onOff(rec_key_val&0x01);
					ui_set_led_onOff(rec_key_val&0x01);
//				if(del_repeat_key>3){
//					uart_send_buf(&rec_key_val,1);
//				}
//				del_repeat_key = 0;
			
        return ( events ^ UI_RECEIVE_MSG_EVT );
    }		
		if(events & UI_PAIRING_SUCCESS_EVT){
			
//			uint8 temp_val = 0x5A;
//			uart_send_buf(&temp_val,1);
			return (events ^ UI_PAIRING_SUCCESS_EVT);
		}
		
		if(events & UI_START_STOP_SCAN_EVT ) {
				extern void simpleBleCentral_onOff_scan();
				simpleBleCentral_onOff_scan();
				return ( events ^ UI_START_STOP_SCAN_EVT);
		}
	if(events & UI_KEY_SCAN_EVT){
			extern void ui_key_scan(void);
			ui_key_scan();
			return (UI_KEY_SCAN_EVT ^ events);
	}
	if(UI_KEY_MSG_EVT & events){
			extern uint16 key_msg_val;
			LOG("\n short key = %x",key_msg_val);			
			if(key_msg_val &0x0100) {
				LOG(" signed klick");
				if(is_pair_mode){
					led_flash_cnt = 0;
					is_pair_mode = 0;
				}				

				ui_set_sw_onOff(!last_sw_status);
				ui_set_led_onOff(last_sw_status);
			}
			else if(key_msg_val &0x0200) {
					LOG(" double klick");
				
				if(is_pair_mode){
					led_flash_cnt = 1;
					is_pair_mode = 0;
				}
				else {
					led_flash_cnt = UI_LED_MAX_PAIRING_CNT;
					is_pair_mode = 1;
				}
			}			
			else if(key_msg_val &0x0800) {
				LOG(" eight klick");
				led_flash_cnt = UI_LED_INIT_WHITE_LIST;
				extern void ui_clear_white_list();
				ui_clear_white_list();
			}						
			return (UI_KEY_MSG_EVT ^ events);
	}
//	if(events & UI_KEY_LONG_TIME_EVT) {
//			LOG("===long key\n");
//
//		if(last_key_number == 0x01)
//			ui_power_off_fun();
//		return ( events ^ UI_KEY_LONG_TIME_EVT);
//	}
//	if(events & UI_KEY_MORE_CLICK_TIME_EVT) {
////			ui_led_key();// led key
////			extern uint8 get_simpleBLE_taskID(void);
////		osal_start_timerEx(get_simpleBLE_taskID(), SBP_KEY_EVT, 20);
//
//			return ( events ^ UI_KEY_MORE_CLICK_TIME_EVT );
//	}
    if(events & UI_LED_EVT) {
        ui_led_evt();
        return ( events ^ UI_LED_EVT );
    }
    return 0;
}

//static bool is_charge_in;
//bool get_charge_status()
//{
//    return hal_gpio_read(UI_CHARGE_IN_IO);
//}

//void ui_pin_event_handler_key(GPIO_Pin_e pin,IO_Wakeup_Pol_e type)
//{
//    if(hal_gpio_read(UI_CHARGE_IN_IO)) {
//        is_charge_in = 1;
//        extern uint8 get_simpleBLE_taskID(void);
//        osal_start_timerEx(get_simpleBLE_taskID(), SBC_TERMINATED_CONN, 100);
//    }
//    else {
//        is_charge_in = 0;
//    }
//}

//void ui_gpio_init()
//{
////		ui_key_gpio_init();
//    hal_gpio_pin_init(UI_CHARGE_IN_IO,IE) ;
//    hal_gpio_pull_set(UI_CHARGE_IN_IO,PULL_DOWN) ;
//    hal_gpioin_register(UI_CHARGE_IN_IO, ui_pin_event_handler_key, ui_pin_event_handler_key);

////    UI_PWM_ALL_CH_INIT();
////    hal_pwm_start();
////    WaitMs(200);
////    UI_PWM_SET(PWM_CH0,PWM_CLK_DIV_4,2750,0);
////    UI_PWM_SET(PWM_CH1,PWM_CLK_DIV_4,255,0);
////    UI_PWM_SET(PWM_CH2,PWM_CLK_DIV_4,255,0);
////    UI_PWM_SET(PWM_CH3,PWM_CLK_DIV_4,255,0);
////    UI_PWM_SET(PWM_CH4,PWM_CLK_DIV_4,255,0);
////		hal_pwm_stop();
//}

void UI_Init( uint8 task_id )
{
    ui_task_id = task_id;
		
		extern void ui_key_gpio_init();
    ui_key_gpio_init();
//    ui_led_init();
		hal_gpio_pin_init(UI_LED0_IO,OEN) ;            //output
		hal_gpioretention_register(UI_LED0_IO);
		hal_gpio_write(UI_LED0_IO,0) ;		

		hal_gpio_pin_init(UI_SW_IO,OEN) ;            //output
		hal_gpioretention_register(UI_SW_IO);
		hal_gpio_write(UI_SW_IO,0) ;		

    ui_n2ms_restart();
}


static void UI_Task_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
    switch ( pMsg->event )
    {

				default:
						break;
    }
}