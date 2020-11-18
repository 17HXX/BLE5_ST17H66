#ifndef __LXL_GPIO_ALL_INIT_H__
#define __LXL_GPIO_ALL_INIT_H__

#include <string.h>
#include <stdlib.h>
#include "types.h"
#include "OSAL_Tasks.h"
#include "rom_sym_def.h"
#include "types.h"
#include "gpio.h"
#include "clock.h"
#include "log.h"
#include "error.h"
#include "OSAL.h"
#include "pwrmgr.h"
#include "bcomdef.h"
#include "OSAL.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gatt_profile_uuid.h"
#include "linkdb.h"
#include "gattservapp.h"
#include "peripheral.h"
#include "bcomdef.h"
#include "linkdb.h"
#include "osal.h"
#include "gatt.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "peripheral.h"
#include "gapbondmgr.h"
#include "gatt_profile_uuid.h"
#include "log.h"
#include "adc.h"
#include "pwm.h"
#include "uart.h"





#ifdef __cplusplus
extern "C"
{
#endif
/****************************************************************************************
*  ������������
****************************************************************************************/
#define MRT_ABS(a,b) ((a)>(b)?(a-b):(b-a))
#define LXL_FLASH_HEAT_DATA_ADDR     0x81       //�洢�¶����� 40��- u16�ֽ�20��


typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef signed   char    int8_t;     //!< Signed 8 bit integer
typedef unsigned char    uint8_t;    //!< Unsigned 8 bit integer
typedef signed   short   int16_t;    //!< Signed 16 bit integer
typedef unsigned short   uint16_t;   //!< Unsigned 16 bit integer
typedef signed   int     int32_t;     //!< Signed 32 bit integer
typedef unsigned int     uint32_t;    //!< Unsigned 32 bit integer
typedef signed   char    int8;       //!< Signed 8 bit integer
typedef unsigned char    uint8;      //!< Unsigned 8 bit integer
typedef signed   short   int16;      //!< Signed 16 bit integer
typedef unsigned short   uint16;     //!< Unsigned 16 bit integer
typedef signed   long    int32;      //!< Signed 32 bit integer
typedef unsigned long    uint32;     //!< Unsigned 32 bit integer

#define MY_GPIO_KEY_1           P15           //�����������
#define MY_GPIO_BACK_LED_1      P7    //P9          //led������  - ��ͨgpio��

#define MY_PWM_CMP_VALUE        370       //ռ�ձ�      740-370
#define MY_PWM_CNTTOP_VALUE     740       //Ƶ�ʼ���   (16M/8)/740
#define MY_GPIO_PWM_M0_1        P34        //���������� - PWM��
#define MY_GPIO_PWM_M0_2        P2        //���������� - PWM��
#define MY_GPIO_PWM_M0_3        P3        //���������� - PWM��
#define MY_GPIO_WARM_PWM        0xff        //���������� - PWM��

#define MY_GPIO_UART_TX           P9      //���Զ˿�TX - uart���,��¼����
#define MY_GPIO_UART_RX           P10     //���Զ˿�RX - uart����,��¼����

#define MY_GPIO_ADC_BATT_1        P20     //adc���ntc,��⻷���¶�
#define MY_GPIO_ADC_MOTO_1        P14     //adc���ntc,��⻷���¶�
#define MY_GPIO_ADC_5V_TEST       P18     //adc���ntc,��⻷���¶�

#define MY_ADC_11_EVENT_TICK      0x0001   //ADC�¼���һ����Ӧ
#define MY_ADC_22_EVENT_TICK      0x0002   //ADC�¼��Ķ�����Ӧ
#define MY_ADC_33_EVENT_TICK      0x0004   //ADC�¼���������Ӧ
#define MY_ADC_44_EVENT_TICK      0x0008   //ADC�¼����ļ���Ӧ
#define MY_ADC_55_EVENT_TICK      0x0010   //ADC�¼����弶��Ӧ

#define MY_UART_1_EVENT_TICK      0x0100     //��ʾ���¼���һ����Ӧ
#define MY_SENSOR_1_EVENT_TICK    0x0100     //sensor�¼���һ����Ӧ
#define MY_KEY_1_EVENT_TICK       0x1000     //�����¼���һ����Ӧ

#define MY_UI_10MS_EVENT_TICK     0x1000     //ui�¼���һ����Ӧ
#define MY_UI_100MS_EVENT_TICK    0x2000     //ui�¼��Ķ�����Ӧ
#define MY_UI_POWER_EVENT_TICK    0x4000     //�ػ�����


#define LXL_POWER_OFF_TIME_NUM    (10*15)
typedef struct {
	u32 dev_time_power_tick ;  //��ǰ�豸��ʱ�ػ� ��Ϊ��λ

	u8 dev_power_off_tick ;    //�ϵ��Ƿ�ֱ�ӿ��ػ�
	u8 dev_job_tick ;          //��ǰ�豸������־λ: 1->����, 0->�ػ�
	u8 dev_sleep_tick ;        //��ǰ�豸�Ƿ�͹���״̬
	u8 dev_batt_value ;        //��ǰ�豸�ĵ���
	u8 dev_charged_state_tick ;//�豸�ĳ��״̬
	u8 dev_ble_connect_en ;    //��ǰble������״̬
	u8 dev_test_mode_en ;      //��ǰΪ�Ƿ�������ģʽ

	u8 dev_boot_up_tick ;      //������־λ
	u8 dev_charge_tick ;       //����־λ
	u8 dev_buzzer_onoff ;      //����������

	u8 dev_app_set_data_0 ;    //app���öϿ����ӱ�����Ҫ��
	u8 dev_app_set_data_1 ;    //app�������Ӻ󱨾���Ҫ��

	u8 dev_app_push_data[40] ; //app�·����ݻ���
}lxl_struct_device_all ;
extern lxl_struct_device_all lxl_device_all_data ;   //��ǰ�豸��ȫ��״̬��־λ

typedef struct {
    u8 dev_onoff_tick ;    //��ǰ�豸�Ŀ���
	u8 dev_mode_tick ;     //��ǰ�豸��ģʽ
	u8 dev_mode_class ;    //ģʽ��ǿ�ȵȼ�
	u16 dev_timeout_tick ; //��ǰ�豸�Ĺ���ʱ������λ��
	u8 dev_pwm_value ;     //��ǰ�豸��pwmֵ
	u8 dev_warm_tick ;     //��ǰ���ȿ���
	u16 dev_rtc_t ;         //��ǰ�豸���¶�
}lxl_dev_func_data ;
extern lxl_dev_func_data lxl_all_func_data ;   //��ǰ�豸�Ĺ��ܱ�־λ


extern uint8 lxl_ui_count_TaskID ;           //��ǰ�¼���id
extern uint8 lxl_sensor_count_TaskID ;       //��ǰ�¼���id
extern uint8 lxl_uart_count_TaskID ;         //��ǰ�¼���id
extern uint8 lxl_key_count_TaskID ;          //��ǰ�¼���id
extern uint8 lxl_adc_count_TaskID ;          //��ǰ�¼���id

extern void my_gpio_all_init_func() ;   //���ų�ʼ��
extern void lxl_ui_init_task_func(uint8 task_id) ;      //ע������
extern void lxl_snesor_init_task_func( uint8 task_id ) ;//ע������
extern void lxl_uart_init_task_func( uint8 task_id ) ;  //ע������
extern void lxl_key_init_task_func( uint8 task_id ) ;   //ע������
extern void lxl_adc_init_task_func( uint8 task_id ) ;   //ע������
extern void lxl_key_count_ProcessOSALMsg( osal_event_hdr_t *pMsg ) ;

extern u32 lxl_clock_time_exceed_func(u32 ref, u32 span_ms) ;  //�Ա�ʱ�亯��


extern void lxl_proc_power_onoff(u8 cur_state, u8 power_start_tick);   //�������ú���
extern void lxl_power_time_count_time_func() ;  //��ʱ�ػ�����
extern void lxl_proc_powerOff_handle(u8 data_tick);         //�ػ����ú���


extern uint8 SimpleProfile_SetParameter( uint8 param, uint8 len, void *value ) ;    //�������ݺ���
/****************************************************************************************
*  ϵͳui����
****************************************************************************************/
typedef struct{            //���ƹ���ʾ
	u16 	offOn_Ms[2];   //����ʱ��
	u8 		offOn_cnt;	   //���д���
	u8 		next_mode;     //��һ��ģʽ
}lxl_ui_type_t;

typedef struct{             //���ƹ���ʾ
	lxl_ui_type_t *ui_type;
	u8 	cur_mode;           //����ģʽ
	u8 	cur_cnt;            //��ǰ�����д���
	u8 	cur_state;          //��ǰ״̬
	u32 next_wakeup_tick;   //��һ������ʱ��
}lxl_ui_param_t;

extern lxl_ui_type_t led_ui_buffer_MS_1[] ;
extern lxl_ui_type_t led_ui_buffer_MS_2[] ;
extern lxl_ui_type_t led_ui_buffer_MS_3[] ;
extern lxl_ui_type_t buzzer_ui_buffer_MS[] ;
extern lxl_ui_param_t lxl_led_param_1 ;        //��ǰled��ʾ����
extern lxl_ui_param_t lxl_led_param_2 ;        //��ǰled��ʾ����
extern lxl_ui_param_t lxl_led_param_3 ;        //��ǰled��ʾ����
extern lxl_ui_param_t lxl_buzzer_param ;       //��ǰ����������

extern void lxl_led_beep_1(u8 onOff) ;         //����led��
extern void lxl_led_beep_2(u8 onOff) ;         //����led��
extern void lxl_led_beep_3(u8 onOff) ;         //����led��
extern void lxl_buzzer_beep(u8 onOff) ;      //���ط�����
extern u32 lxl_calc_led_ui(lxl_ui_param_t *ui_bl) ;  //������һ�ο���ʱ��
extern u32 lxl_buzzer_led_ui () ;   //�������ݣ�ѭ������
extern void lxl_ui_enter_mode(lxl_ui_param_t *ui_param,u8 mode) ;  //����ṹ��
extern void lxl_led_enter_mode_1(u8 mode) ;     //��������led
extern void lxl_led_enter_mode_2(u8 mode) ;     //��������led
extern void lxl_led_enter_mode_3(u8 mode) ;     //��������led
extern void lxl_buzzer_enter_mode(u8 mode) ;  //�������ݷ�����

extern uint16 lxl_ui_all_func(uint8 task_id, uint16 events) ;  //ѭ��������

extern void lxl_SimpleProfile_SetParameter(uint8 param, uint8 len, void *value ) ;
extern bStatus_t SimpleProfile_SetParameter( uint8 param, uint8 len, void *value ) ;


/**************************************************************************************
* lcd��ʾ��Ļ����
**************************************************************************************/
extern void lxl_set_seg_start_timer(void) ;   //��ʱ1.8ms
extern void lxl_set_seg_start_delay(void) ;   //��ʱ0.2ms
extern void lxl_set_seg_stop_timer(void) ;    //ֹͣ��ʱ

/**************************************************************************************
* ������������
**************************************************************************************/
typedef struct {
    u32 key_dowm_time_0 ;      //���������ǵ�ϵͳtime
    u8 key_dowm_tick_0 ;       //�������»���̧��ı�־
	u8 key_batt_power_tick ;   //���䵱ǰ����״̬
    u8 key_repeatedly_tick ;   //���䰴�����´���
}lxl_struct_key_all ;
extern lxl_struct_key_all lxl_key_all_data ;   //key��״̬��־λ

extern uint16 lxl_key_all_func(uint8 task_id, uint16 events) ;
extern void lxl_pin_event_handler_key(GPIO_Pin_e pin,IO_Wakeup_Pol_e type) ;

/**************************************************************************************
* adc�������
**************************************************************************************/

typedef struct {
    u8 diff_adc_run_flag ;        //��ǰ����״̬://0--idle  1--busy
    u8 adc_echo_flag ;            //��ǰ�¼�״̬://0--busy  1--idle

    u8 adc_enable ;               //adc�Ƿ���������
    u16 adc_heat_value;           //��ǰ���¶�
    u16 adc_ntc_value ;           //��ǰntc���¶�

    int32 adc_diff_value  ;       //��¼��ǰ��β������
}lxl_struct_adc_all ;
extern lxl_struct_adc_all lxl_adc_all_data ;  //adc��״̬��־λ

extern uint16 lxl_adc_all_func(uint8 task_id, uint16 events) ; //ѭ��������
extern void normal_adc_handle_evt(adc_Evt_t* pev);             //�ɼ�adc����
extern int normal_adc_init(void) ;                             //adc��ʼ��

extern void lxl_gpio_charge_count_func() ;   //����ж�
extern float lxl_get_near_ntc_temp_index(uint32 value) ;

/**************************************************************************************
* PWM���ܺ���
**************************************************************************************/
void lxl_pwm_timeout_handle(void) ;           //�ر�pwm����
int lxl_pwm_ctrl(uint8_t ch, uint16_t value); //����pwm��ռ�ձ�
int lxl_pwm_on(uint8_t ch);                   //�����ĸ����Ź���pwm,�Լ�����Ƶ��
int lxl_pwm_off(uint8_t ch);                  //�رյ�ǰ���ŵĹ���
int lxl_pwm_init(void);                       //��ʼ����ǰռ�ձ�

/**************************************************************************************
* UART���ܺ���
**************************************************************************************/
#define LXL_IIC_ID_ADDV_TICK          (0x24)       //iic�ĵ�ַ
#define UART_TX_BUFFER_SIZE   255
#define UART_RX_BUFFER_SIZE   255
typedef struct {
	u8 adv_mode_tick ;	  //��ǰ���� 1:nfc 2:����ͷ 3:��

	uint16_t	  push_data_numable ;		   //�ϴ����ݵĴ�С
	uint16_t	  push_tick ;				   //��ʼ�ϴ����ݱ�־λ
	uint16_t	  tx_cnt;					   //tx�������ݸ���
	uint8_t 	  tx_buf[255]; //tx��������
	uint16_t	  rx_cnt;					   //rx�������ݸ���
	uint8_t 	  rx_buf[255]; //rx��������
}lxl_struct_sensor_all ;
extern lxl_struct_sensor_all lxl_sensor_all_data ;	  //��ǰsensor������


extern void lxl_uart_printf_hex (const uint8 *data, uint16 len) ;  //��ӡ����
extern void lxl_uart_init_task_func( uint8 task_id ) ;
extern uint16 lxl_uart_all_func(uint8 task_id, uint16 events) ;

extern void lxl_uart_sensor_gpio_init(u8 mode) ;   //��ʼ��uart������
extern void lxl_uart_console_rx_handler(uart_Evt_t* pev) ;  //��ǰuart��ȡ���ݺ���
extern int hal_uart_set_tx_buf(UART_INDEX_e uart_index,uint8_t* buf, uint16_t size) ;//��ǰuart�����ݺ���




/**************************************************************************************
* SENSOR���ܺ���
**************************************************************************************/
extern void lxl_pwm1_count_value_func() ;   //�������ģʽ����"1"
void lxl_warm_count_onoff_func();    //���ȿ��ƺ���






/**************************************************************************************
* �����б����ݶ���
**************************************************************************************/
typedef void (*battServiceCB_t)(uint8 event);
typedef void (*battServiceSetupCB_t)(void);
typedef uint8 (*battServiceCalcCB_t)(uint16 adcVal);
typedef void (*battServiceTeardownCB_t)(void);

// Callback events
#define BATT_LEVEL_NOTI_ENABLED         1
#define BATT_LEVEL_NOTI_DISABLED        2

#define BATT_LEVEL_VALUE_IDX        2 // Position of battery level in attribute array
#define BATT_LEVEL_VALUE_CCCD_IDX   3 // Position of battery level CCCD in attribute array


extern bStatus_t SimpleProfile_AddService_batt( uint32 services ) ;
extern bStatus_t SimpleProfile_AddService_alert( uint32 services ) ;


/**************************************************************************************
* �͹��ĺ���
**************************************************************************************/
extern int hal_pwrmgr_lock(MODULE_e mod) ;     //�͹��ĺ���
extern int hal_pwrmgr_unlock(MODULE_e mod) ;   //�ǵ͹��ĺ���
extern int hal_pwrmgr_register(MODULE_e mod, pwrmgr_Hdl_t sleepHandle, pwrmgr_Hdl_t wakeupHandle) ;





#ifdef __cplusplus
}
#endif

#endif

