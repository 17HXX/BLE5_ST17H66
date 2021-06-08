/**************************************************************************************************
*******
**************************************************************************************************/

/******************************************************************************

 *****************************************************************************/
#include "ll.h"
#include "ll_common.h"
#include "ll_def.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define UI_KEY_MORE_CLICK_TIME_EVT										0x0010
#define UI_KEY_LONG_TIME_EVT													0x0020	

//#define UI_KEY_MSG_EVT																0x0080

#define UI_POWER_OFF_EVT																0x0100

#define NO_KEY						0xFF



#define KEY_LONG					BIT(12)
#define KEY_LONG_UP				BIT(13)
#define KEY_LONG_HOLD			BIT(14)
#define KEY_SHORT_CLICK		BIT(15)

#define KEY_SCAN_INTERVAL	40	//uint ms
#define KEY_LONG_CNT			(1200/KEY_SCAN_INTERVAL)
#define KEY_SHORT_CNT			(800/KEY_SCAN_INTERVAL)



#define KEY_MORE_KEY_TIMEOUT	500//350	//uint ms
#define KEY_MORE_KEY_TIMEOUT_CNT	(KEY_MORE_KEY_TIMEOUT/KEY_SCAN_INTERVAL)

#define UI_LONG_KEY_COUNT			3000
#define UI_MORE_KEY_TIMEOUT		30	///300


void ui_key_gpio_init(void);
extern uint8 ui_task_id;
extern void ui_power_off_fun(void);
extern uint8 last_key_number;
extern void ui_key_scan(void);


extern void ui_power_off_fun(void);

