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


#define UI_LED_EVT													0x1000


extern void ui_led_evt(void);
extern void ui_led_init(void);
extern void ui_led_key(void);
extern void ui_led_con(void);
extern void ui_led_adv(void);
extern void ui_led_pairing(void);


//#define LOG_LED(...)
#define LOG_LED(...)  dbg_printf(__VA_ARGS__)

