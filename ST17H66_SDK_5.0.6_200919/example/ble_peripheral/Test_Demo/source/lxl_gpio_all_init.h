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
*  公用数据声明
****************************************************************************************/
#define MRT_ABS(a,b) ((a)>(b)?(a-b):(b-a))
#define LXL_FLASH_HEAT_DATA_ADDR     0x81       //存储温度数据 40个- u16字节20个


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

#define MY_GPIO_KEY_1           P15           //按键检测引脚
#define MY_GPIO_BACK_LED_1      P7    //P9          //led灯引脚  - 普通gpio口

#define MY_PWM_CMP_VALUE        370       //占空比      740-370
#define MY_PWM_CNTTOP_VALUE     740       //频率计算   (16M/8)/740
#define MY_GPIO_PWM_M0_1        P34        //蜂鸣器驱动 - PWM口
#define MY_GPIO_PWM_M0_2        P2        //蜂鸣器驱动 - PWM口
#define MY_GPIO_PWM_M0_3        P3        //蜂鸣器驱动 - PWM口
#define MY_GPIO_WARM_PWM        0xff        //蜂鸣器驱动 - PWM口

#define MY_GPIO_UART_TX           P9      //测试端口TX - uart输出,烧录引脚
#define MY_GPIO_UART_RX           P10     //测试端口RX - uart输入,烧录引脚

#define MY_GPIO_ADC_BATT_1        P20     //adc检测ntc,检测环境温度
#define MY_GPIO_ADC_MOTO_1        P14     //adc检测ntc,检测环境温度
#define MY_GPIO_ADC_5V_TEST       P18     //adc检测ntc,检测环境温度

#define MY_ADC_11_EVENT_TICK      0x0001   //ADC事件的一级响应
#define MY_ADC_22_EVENT_TICK      0x0002   //ADC事件的二级响应
#define MY_ADC_33_EVENT_TICK      0x0004   //ADC事件的三级响应
#define MY_ADC_44_EVENT_TICK      0x0008   //ADC事件的四级响应
#define MY_ADC_55_EVENT_TICK      0x0010   //ADC事件的五级响应

#define MY_UART_1_EVENT_TICK      0x0100     //显示屏事件的一级响应
#define MY_SENSOR_1_EVENT_TICK    0x0100     //sensor事件的一级响应
#define MY_KEY_1_EVENT_TICK       0x1000     //按键事件的一级响应

#define MY_UI_10MS_EVENT_TICK     0x1000     //ui事件的一级响应
#define MY_UI_100MS_EVENT_TICK    0x2000     //ui事件的二级响应
#define MY_UI_POWER_EVENT_TICK    0x4000     //关机重启


#define LXL_POWER_OFF_TIME_NUM    (10*15)
typedef struct {
	u32 dev_time_power_tick ;  //当前设备定时关机 秒为单位

	u8 dev_power_off_tick ;    //上电是否直接开关机
	u8 dev_job_tick ;          //当前设备工作标志位: 1->工作, 0->关机
	u8 dev_sleep_tick ;        //当前设备是否低功耗状态
	u8 dev_batt_value ;        //当前设备的电量
	u8 dev_charged_state_tick ;//设备的充电状态
	u8 dev_ble_connect_en ;    //当前ble的连接状态
	u8 dev_test_mode_en ;      //当前为是否进入测试模式

	u8 dev_boot_up_tick ;      //开机标志位
	u8 dev_charge_tick ;       //充电标志位
	u8 dev_buzzer_onoff ;      //蜂鸣器开关

	u8 dev_app_set_data_0 ;    //app设置断开连接报警的要求
	u8 dev_app_set_data_1 ;    //app设置连接后报警的要求

	u8 dev_app_push_data[40] ; //app下发数据缓存
}lxl_struct_device_all ;
extern lxl_struct_device_all lxl_device_all_data ;   //当前设备的全部状态标志位

typedef struct {
    u8 dev_onoff_tick ;    //当前设备的开关
	u8 dev_mode_tick ;     //当前设备的模式
	u8 dev_mode_class ;    //模式的强度等级
	u16 dev_timeout_tick ; //当前设备的工作时长，单位秒
	u8 dev_pwm_value ;     //当前设备的pwm值
	u8 dev_warm_tick ;     //当前加热开关
	u16 dev_rtc_t ;         //当前设备的温度
}lxl_dev_func_data ;
extern lxl_dev_func_data lxl_all_func_data ;   //当前设备的功能标志位


extern uint8 lxl_ui_count_TaskID ;           //当前事件的id
extern uint8 lxl_sensor_count_TaskID ;       //当前事件的id
extern uint8 lxl_uart_count_TaskID ;         //当前事件的id
extern uint8 lxl_key_count_TaskID ;          //当前事件的id
extern uint8 lxl_adc_count_TaskID ;          //当前事件的id

extern void my_gpio_all_init_func() ;   //引脚初始化
extern void lxl_ui_init_task_func(uint8 task_id) ;      //注册任务
extern void lxl_snesor_init_task_func( uint8 task_id ) ;//注册任务
extern void lxl_uart_init_task_func( uint8 task_id ) ;  //注册任务
extern void lxl_key_init_task_func( uint8 task_id ) ;   //注册任务
extern void lxl_adc_init_task_func( uint8 task_id ) ;   //注册任务
extern void lxl_key_count_ProcessOSALMsg( osal_event_hdr_t *pMsg ) ;

extern u32 lxl_clock_time_exceed_func(u32 ref, u32 span_ms) ;  //对比时间函数


extern void lxl_proc_power_onoff(u8 cur_state, u8 power_start_tick);   //开机调用函数
extern void lxl_power_time_count_time_func() ;  //定时关机函数
extern void lxl_proc_powerOff_handle(u8 data_tick);         //关机调用函数


extern uint8 SimpleProfile_SetParameter( uint8 param, uint8 len, void *value ) ;    //发送数据函数
/****************************************************************************************
*  系统ui声明
****************************************************************************************/
typedef struct{            //做灯光显示
	u16 	offOn_Ms[2];   //开关时间
	u8 		offOn_cnt;	   //运行次数
	u8 		next_mode;     //下一个模式
}lxl_ui_type_t;

typedef struct{             //做灯光显示
	lxl_ui_type_t *ui_type;
	u8 	cur_mode;           //运行模式
	u8 	cur_cnt;            //当前的运行次数
	u8 	cur_state;          //当前状态
	u32 next_wakeup_tick;   //下一个开关时间
}lxl_ui_param_t;

extern lxl_ui_type_t led_ui_buffer_MS_1[] ;
extern lxl_ui_type_t led_ui_buffer_MS_2[] ;
extern lxl_ui_type_t led_ui_buffer_MS_3[] ;
extern lxl_ui_type_t buzzer_ui_buffer_MS[] ;
extern lxl_ui_param_t lxl_led_param_1 ;        //当前led显示控制
extern lxl_ui_param_t lxl_led_param_2 ;        //当前led显示控制
extern lxl_ui_param_t lxl_led_param_3 ;        //当前led显示控制
extern lxl_ui_param_t lxl_buzzer_param ;       //当前蜂鸣器控制

extern void lxl_led_beep_1(u8 onOff) ;         //开关led灯
extern void lxl_led_beep_2(u8 onOff) ;         //开关led灯
extern void lxl_led_beep_3(u8 onOff) ;         //开关led灯
extern void lxl_buzzer_beep(u8 onOff) ;      //开关蜂鸣器
extern u32 lxl_calc_led_ui(lxl_ui_param_t *ui_bl) ;  //计算下一次开关时间
extern u32 lxl_buzzer_led_ui () ;   //传入数据，循环函数
extern void lxl_ui_enter_mode(lxl_ui_param_t *ui_param,u8 mode) ;  //代入结构体
extern void lxl_led_enter_mode_1(u8 mode) ;     //设置数据led
extern void lxl_led_enter_mode_2(u8 mode) ;     //设置数据led
extern void lxl_led_enter_mode_3(u8 mode) ;     //设置数据led
extern void lxl_buzzer_enter_mode(u8 mode) ;  //设置数据蜂鸣器

extern uint16 lxl_ui_all_func(uint8 task_id, uint16 events) ;  //循环处理函数

extern void lxl_SimpleProfile_SetParameter(uint8 param, uint8 len, void *value ) ;
extern bStatus_t SimpleProfile_SetParameter( uint8 param, uint8 len, void *value ) ;


/**************************************************************************************
* lcd显示屏幕声明
**************************************************************************************/
extern void lxl_set_seg_start_timer(void) ;   //定时1.8ms
extern void lxl_set_seg_start_delay(void) ;   //定时0.2ms
extern void lxl_set_seg_stop_timer(void) ;    //停止定时

/**************************************************************************************
* 按键计算声明
**************************************************************************************/
typedef struct {
    u32 key_dowm_time_0 ;      //按键按下是的系统time
    u8 key_dowm_tick_0 ;       //按键按下或者抬起的标志
	u8 key_batt_power_tick ;   //记忆当前开机状态
    u8 key_repeatedly_tick ;   //记忆按键按下次数
}lxl_struct_key_all ;
extern lxl_struct_key_all lxl_key_all_data ;   //key的状态标志位

extern uint16 lxl_key_all_func(uint8 task_id, uint16 events) ;
extern void lxl_pin_event_handler_key(GPIO_Pin_e pin,IO_Wakeup_Pol_e type) ;

/**************************************************************************************
* adc检测声明
**************************************************************************************/

typedef struct {
    u8 diff_adc_run_flag ;        //当前工作状态://0--idle  1--busy
    u8 adc_echo_flag ;            //当前事件状态://0--busy  1--idle

    u8 adc_enable ;               //adc是否正常工作
    u16 adc_heat_value;           //当前的温度
    u16 adc_ntc_value ;           //当前ntc的温度

    int32 adc_diff_value  ;       //记录当前五次差分数据
}lxl_struct_adc_all ;
extern lxl_struct_adc_all lxl_adc_all_data ;  //adc的状态标志位

extern uint16 lxl_adc_all_func(uint8 task_id, uint16 events) ; //循环处理函数
extern void normal_adc_handle_evt(adc_Evt_t* pev);             //采集adc数据
extern int normal_adc_init(void) ;                             //adc初始化

extern void lxl_gpio_charge_count_func() ;   //充电判断
extern float lxl_get_near_ntc_temp_index(uint32 value) ;

/**************************************************************************************
* PWM功能函数
**************************************************************************************/
void lxl_pwm_timeout_handle(void) ;           //关闭pwm功能
int lxl_pwm_ctrl(uint8_t ch, uint16_t value); //设置pwm的占空比
int lxl_pwm_on(uint8_t ch);                   //设置哪个引脚工作pwm,以及工作频率
int lxl_pwm_off(uint8_t ch);                  //关闭当前引脚的工作
int lxl_pwm_init(void);                       //初始化当前占空比

/**************************************************************************************
* UART功能函数
**************************************************************************************/
#define LXL_IIC_ID_ADDV_TICK          (0x24)       //iic的地址
#define UART_TX_BUFFER_SIZE   255
#define UART_RX_BUFFER_SIZE   255
typedef struct {
	u8 adv_mode_tick ;	  //当前处于 1:nfc 2:摄像头 3:无

	uint16_t	  push_data_numable ;		   //上传数据的大小
	uint16_t	  push_tick ;				   //开始上传数据标志位
	uint16_t	  tx_cnt;					   //tx发送数据个数
	uint8_t 	  tx_buf[255]; //tx发送数据
	uint16_t	  rx_cnt;					   //rx接收数据个数
	uint8_t 	  rx_buf[255]; //rx接收数据
}lxl_struct_sensor_all ;
extern lxl_struct_sensor_all lxl_sensor_all_data ;	  //当前sensor的数据


extern void lxl_uart_printf_hex (const uint8 *data, uint16 len) ;  //打印函数
extern void lxl_uart_init_task_func( uint8 task_id ) ;
extern uint16 lxl_uart_all_func(uint8 task_id, uint16 events) ;

extern void lxl_uart_sensor_gpio_init(u8 mode) ;   //初始化uart的引脚
extern void lxl_uart_console_rx_handler(uart_Evt_t* pev) ;  //当前uart获取数据函数
extern int hal_uart_set_tx_buf(UART_INDEX_e uart_index,uint8_t* buf, uint16_t size) ;//当前uart发数据函数




/**************************************************************************************
* SENSOR功能函数
**************************************************************************************/
extern void lxl_pwm1_count_value_func() ;   //电机功能模式控制"1"
void lxl_warm_count_onoff_func();    //加热控制函数






/**************************************************************************************
* 服务列表数据定义
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
* 低功耗函数
**************************************************************************************/
extern int hal_pwrmgr_lock(MODULE_e mod) ;     //低功耗函数
extern int hal_pwrmgr_unlock(MODULE_e mod) ;   //非低功耗函数
extern int hal_pwrmgr_register(MODULE_e mod, pwrmgr_Hdl_t sleepHandle, pwrmgr_Hdl_t wakeupHandle) ;





#ifdef __cplusplus
}
#endif

#endif

