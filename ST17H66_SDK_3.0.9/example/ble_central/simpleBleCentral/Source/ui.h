/**************************************************************************************************
*******
**************************************************************************************************/

/******************************************************************************

 *****************************************************************************/
#include "ll.h"
#include "ll_common.h"
#include "ll_def.h"
#include "pwm.h"

#ifdef __cplusplus
extern "C"
{
#endif

extern uint16 UI_ProcessEvent( uint8 task_id, uint16 events );
extern void UI_Init( uint8 task_id );

extern uint8 ui_task_id;



#define UI_CHARGE_IN_IO	P20

#define UI_IO_FMQ			P34
#define UI_IO_LED_R		P2
#define UI_IO_LED_G		P3
#define UI_IO_LED_B		P7
#define UI_IO_LED_W 	P11



#define UI_IO_LED_BT	P2
#define UI_IO_UART1_RX	P3
#define UI_IO_UART1_TX	P14


#define UI_KEY0_IO P2
#define UI_KEY1_IO GPIO_DUMMY
#define UI_KEY2_IO GPIO_DUMMY

#define UI_LED0_IO	P34
#define UI_LED1_IO	GPIO_DUMMY
#define UI_LED2_IO	GPIO_DUMMY

#define UI_SW_IO	P11



//#define IO_LED1				P15
//#define IO_LED2				P20
//#define IO_LED3				P14


#define UI_TIME_EVT                                  	0x0001
#define UI_RECEIVE_MSG_EVT														0x0002

#define UI_START_STOP_SCAN_EVT															0x0010

#define UI_PAIRING_SUCCESS_EVT															0x0100

#define UI_KEY_SCAN_EVT																0x0040
#define UI_KEY_MSG_EVT																0x0080

/******* pwm setting *****/
#define UI_PWM_CLK_DIV		PWM_CLK_DIV_4

#define UI_IO_OUT_INIT(x)	do{hal_gpio_fmux(x, Bit_DISABLE);hal_gpio_pin_init(x, OEN);hal_gpio_write(x, 0);}while(0);
#define UI_INIT_PWM(CHx,PWM_POLARITY,UI_PWM_CLK_DIV,PWM_CNT_MODE,GPIO)	do{hal_pwm_init(CHx, UI_PWM_CLK_DIV, PWM_CNT_MODE, PWM_POLARITY);hal_pwm_open_channel(CHx, GPIO);}while(0);
#define UI_PWM_SET(CHx,UI_PWM_CLK_DIV,FREQ,DUTY)	hal_pwm_set_count_val(CHx, (((16000000/(1<<UI_PWM_CLK_DIV))/FREQ)*DUTY/100), ((16000000/(1<<UI_PWM_CLK_DIV))/FREQ));

#define UI_PWM_FMQ_INIT()  UI_IO_OUT_INIT(UI_IO_FMQ);\
													UI_INIT_PWM(PWM_CH0,PWM_POLARITY_FALLING,PWM_CLK_DIV_4,PWM_CNT_UP,UI_IO_FMQ);\
													UI_PWM_SET(PWM_CH0,PWM_CLK_DIV_4,2750,20)

#define UI_PWM_LED_R_INIT()  UI_IO_OUT_INIT(UI_IO_LED_R);\
													UI_INIT_PWM(PWM_CH1,PWM_POLARITY_FALLING,PWM_CLK_DIV_4,PWM_CNT_UP,UI_IO_LED_R);\
													UI_PWM_SET(PWM_CH1,PWM_CLK_DIV_4,255,20)

#define UI_PWM_LED_G_INIT()  UI_IO_OUT_INIT(UI_IO_LED_G);\
													UI_INIT_PWM(PWM_CH2,PWM_POLARITY_FALLING,PWM_CLK_DIV_4,PWM_CNT_UP,UI_IO_LED_G);\
													UI_PWM_SET(PWM_CH2,PWM_CLK_DIV_4,255,20)

#define UI_PWM_LED_B_INIT()  UI_IO_OUT_INIT(UI_IO_LED_B);\
													UI_INIT_PWM(PWM_CH3,PWM_POLARITY_FALLING,PWM_CLK_DIV_4,PWM_CNT_UP,UI_IO_LED_B);\
													UI_PWM_SET(PWM_CH3,PWM_CLK_DIV_4,255,20)

#define UI_PWM_LED_W_INIT()  UI_IO_OUT_INIT(UI_IO_LED_W);\
													UI_INIT_PWM(PWM_CH4,PWM_POLARITY_FALLING,PWM_CLK_DIV_4,PWM_CNT_UP,UI_IO_LED_W);\
													UI_PWM_SET(PWM_CH4,PWM_CLK_DIV_4,255,20)

#define UI_PWM_ALL_CH_INIT() 	UI_PWM_FMQ_INIT();\
															UI_PWM_LED_R_INIT();\
															UI_PWM_LED_G_INIT();\
															UI_PWM_LED_B_INIT();\
															UI_PWM_LED_W_INIT()



