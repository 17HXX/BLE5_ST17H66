/**************************************************************************************************
*******
**************************************************************************************************/

/**************************************************************************************************
    Filename:       gpio_demo.c
    Revised:        $Date $
    Revision:       $Revision $


**************************************************************************************************/

/*********************************************************************
    INCLUDES
*/

#include "OSAL.h"
#include "gpio_demo.h"
#include "log.h"

#include "gpio.h"
#include "clock.h"

#include "pwrmgr.h"
#include "error.h"
#include "key.h"


/*********************************************************************
    pulseMeasure_Task
    Task pulseMeasure sample code,we can use p04~p07 and p11~p15 easily.
*/
static uint8 pulseMeasure_TaskID;

typedef struct
{
    bool          enable;
    bool          pinstate;
    uint32_t      edge_tick;
} gpioin_Trig_t;

typedef struct
{
    GPIO_Pin_e    pin;
    bool          type;
    uint32_t      ticks;
} gpioin_pulse_Width_measure_t;

gpioin_pulse_Width_measure_t measureResult =
{
    .pin = GPIO_P14,
};

static gpioin_Trig_t gpioTrig =
{
    .enable = FALSE,
    .edge_tick = 0,
};

void plus_edge_callback(void)
{
    LOG("pulse:%d %d\n",measureResult.type,measureResult.ticks);
}

void pulse_measure_callback(GPIO_Pin_e pin,IO_Wakeup_Pol_e type)
{
    if(gpioTrig.enable == FALSE)
    {
        gpioTrig.enable = TRUE;
        gpioTrig.edge_tick = hal_systick();
        return;
    }

    measureResult.type = type;
    measureResult.ticks = hal_ms_intv(gpioTrig.edge_tick);
    plus_edge_callback();
    gpioTrig.edge_tick = hal_systick();
}


void Pulse_Measure_Init( uint8 task_id )
{
    pulseMeasure_TaskID = task_id;
    hal_gpio_init();
    hal_gpioin_register(measureResult.pin,pulse_measure_callback,pulse_measure_callback);
    gpioTrig.pinstate = hal_gpio_read(measureResult.pin);
}

uint16 Pulse_Measure_ProcessEvent( uint8 task_id, uint16 events )
{
    if(task_id != pulseMeasure_TaskID)
    {
        return 0;
    }

    // Discard unknown events
    return 0;
}

/*********************************************************************
    gpio_wakeup_Task
    Task gpio wakeup sample code
    The followinng code shows P14 wakeup the system when there is a posedge or negedge.
*/
static uint8 gpio_wakeup_TaskID;
void posedge_callback_wakeup(GPIO_Pin_e pin,IO_Wakeup_Pol_e type)
{
    if(type == POSEDGE)
    {
        LOG("wakeup(pos):gpio:%d type:%d\n",pin,type);
    }
    else
    {
        LOG("error\n");
    }
}

void negedge_callback_wakeup(GPIO_Pin_e pin,IO_Wakeup_Pol_e type)
{
    if(type == NEGEDGE)
    {
        LOG("wakeup(neg):gpio:%d type:%d\n",pin,type);
    }
    else
    {
        LOG("wakeup(pos):gpio:%d type:%d\n",pin,type);
    }
}

/*
     P00~P03:default jtag,we can use it as wakeup pin when no debug.
     P04~P07,P11~P15,P18~P30:default gpio,use it easily.
     P08:mode select pin,cannot used as other usage.
     P09~P10,it is uart in burn mode which cannot config.it is configable when in debug mode.
     P16~P17:xtal pin,when use this pins,please use rc as system frequency.config hal_rtc_clock_config(CLK_32K_RCOSC) in hal_init first.
     P31~P34:default spif,we can use it as wakeup pin directly,we driver have completed its multiplex config.
*/
typedef struct gpioin_wakeup_t
{
    GPIO_Pin_e pin;
    gpioin_Hdl_t posedgeHdl;
    gpioin_Hdl_t negedgeHdl;
} gpioin_wakeup;

gpioin_wakeup gpiodemo[GPIO_WAKEUP_PIN_NUM] =
{
    GPIO_P14,posedge_callback_wakeup,negedge_callback_wakeup,
    GPIO_P23,posedge_callback_wakeup,negedge_callback_wakeup,
    GPIO_P31,posedge_callback_wakeup,negedge_callback_wakeup,
};

void GPIO_Wakeup_Init(uint8 task_id )
{
    uint8_t i = 0;
    static bool gpioin_state[GPIO_WAKEUP_PIN_NUM];
    hal_gpio_init();
    gpio_wakeup_TaskID = task_id;
    LOG("gpio wakeup demo start...\n");

    //hal_gpio_pull_set(P14,WEAK_PULL_UP);

    for(i = 0; i<GPIO_WAKEUP_PIN_NUM; i++)
    {
        hal_gpioin_register(gpiodemo[i].pin,gpiodemo[i].posedgeHdl,gpiodemo[i].negedgeHdl);
        gpioin_state[i] = hal_gpio_read(gpiodemo[i].pin);
        LOG("gpioin_state:%d %d\n",i,gpioin_state[i]);
    }
}

uint16 GPIO_Wakeup_ProcessEvent( uint8 task_id, uint16 events )
{
    if(task_id != gpio_wakeup_TaskID)
    {
        return 0;
    }

    return 0;
}

/*********************************************************************
    key_Task:gpio config as key

*/
static uint8 key_TaskID;

#define KEY_DEMO_ONCE_TIMER      0x0001
#define KEY_DEMO_CYCLE_TIMER     0x0002
//#define HAL_KEY_EVENT          0x0100//assign short key event in your app event process

#ifdef HAL_KEY_SUPPORT_LONG_PRESS
    //    #define KEY_DEMO_LONG_PRESS_EVT   0x0200 //if use long key,assign long key event in your app process
#endif

static void key_press_evt(uint8_t i,key_evt_t key_evt)
{
    LOG("\nkey index:%d gpio:%d ",i,key_state.key[i].pin);

    switch(key_evt)
    {
    case HAL_KEY_EVT_PRESS:
        LOG("key(press down)\n");
        break;

    case HAL_KEY_EVT_RELEASE:
        LOG("key(press release)\n");
        break;
        #ifdef HAL_KEY_SUPPORT_LONG_PRESS

    case HAL_KEY_EVT_LONG_RELEASE:
        hal_pwrmgr_unlock(MOD_USR1);
        LOG("key(long press release)\n");
        break;
        #endif

    default:
        LOG("unexpect\n");
        break;
    }
}

static void P16_wakeup_handler(void)
{
    hal_gpio_cfg_analog_io(P16,Bit_DISABLE);
}

static void P17_wakeup_handler(void)
{
    hal_gpio_cfg_analog_io(P17,Bit_DISABLE);
}

typedef struct _uart_Context
{
    bool        enable;
    uint8_t     tx_state;
    uart_Tx_Buf_t tx_buf;
    uart_Cfg_t  cfg;
} uart_Ctx_t;


//extern uart_Ctx_t m_uartCtx;
void uart_port_reconfig(void)
{
    uart_Cfg_t cfg_user =
    {
        .tx_pin = P14,
        .rx_pin = P15,

        .rts_pin = GPIO_DUMMY,
        .cts_pin = GPIO_DUMMY,
        .baudrate = 115200,
        .use_fifo = TRUE,
        .hw_fwctrl = FALSE,
        .use_tx_buf = FALSE,
        .parity     = FALSE,
        .evt_handler = NULL,
    };
    hal_gpio_fmux(P9,Bit_DISABLE);
    hal_gpio_fmux(P10,Bit_DISABLE);
    hal_gpio_pin_init(P14,OEN);
    hal_gpio_pin_init(P15,IE);
    //m_uartCtx.enable = FALSE;
    hal_uart_init(cfg_user,UART0);//uart init
    LOG("uart new port...\n");
}

void Key_Demo_Init(uint8 task_id)
{
    uint8_t i = 0;
    key_TaskID = task_id;
    LOG("gpio key demo start...\n");
    hal_gpio_init();
    hal_gpioretention_register(P20);
    hal_gpio_write(P20,1);
//    hal_gpio_pin2pin3_control(P2,1);
    /*
        when use key,please set the following parameters:
        1.key number,config KEY_NUM in key.h
        2.gpio used,config key_state.pin
             P00~P03:default jtag,we can use it as key when no debug.
             P04~P07,P11~P15:default gpio,use it easily.
             P08:mode select pin,cannot used as other usage.
             P09~P10,it is uart in burn mode which cannot config.it is configable when in debug mode.
             P16~P17:xtal pin,when use this pins,please use rc as system frequency.config hal_rtc_clock_config(CLK_32K_RCOSC) in hal_init first.
             P18~P34:wakeup is supported,but interrupt is not supported,so config it as key is not suggested.
        3.idle level,config key_state.idle_level
        4.key type,if only use press and release,ignore the long press and release code
        5.taskID and callback function
    */
//  key_state.key[0].pin = GPIO_P14;//default gpio
//  key_state.key[1].pin = GPIO_P15;
//  key_state.key[2].pin = GPIO_P00;//default jtag
//  key_state.key[3].pin = GPIO_P01;
//  key_state.key[4].pin = GPIO_P02;
//  key_state.key[5].pin = GPIO_P03;
//  key_state.key[6].pin = GPIO_P16;//default xtal
//  key_state.key[7].pin = GPIO_P17;//default xtal
//
//  key_state.key[0].pin = GPIO_P09;
//  key_state.key[1].pin = GPIO_P10;
    key_state.key[0].pin = GPIO_P03;
//  key_state.key[1].pin = GPIO_P15;

    for(i = 0; i < HAL_KEY_NUM; ++i)
    {
        key_state.key[i].state = HAL_STATE_KEY_IDLE;
        key_state.key[i].idle_level = HAL_LOW_IDLE;

        if(key_state.key[i].pin == GPIO_P16)
        {
            hal_pwrmgr_register(MOD_USR2,NULL,P16_wakeup_handler);
            hal_gpio_cfg_analog_io(key_state.key[i].pin,Bit_DISABLE);
            LOG("P16 is used\n");
        }
        else if(key_state.key[i].pin == GPIO_P17)
        {
            hal_pwrmgr_register(MOD_USR3,NULL,P17_wakeup_handler);
            hal_gpio_cfg_analog_io(key_state.key[i].pin,Bit_DISABLE);
            LOG("P17 is used\n");
        }
        else if((key_state.key[i].pin == GPIO_P09) || (key_state.key[i].pin == GPIO_P10))
        {
            uart_port_reconfig();
        }
    }

//  key_state.key[0].idle_level = HAL_LOW_IDLE;
//  key_state.key[1].idle_level = HAL_HIGH_IDLE;
    key_state.task_id = key_TaskID;
    key_state.key_callbank = key_press_evt;
    key_init();
    osal_start_timerEx(key_TaskID, KEY_DEMO_ONCE_TIMER, 5000);
    osal_start_reload_timer(key_TaskID, KEY_DEMO_CYCLE_TIMER, 5000);
}

uint16 Key_ProcessEvent( uint8 task_id, uint16 events )
{
    if(task_id != key_TaskID)
    {
        return 0;
    }

    if( events & KEY_DEMO_ONCE_TIMER)
    {
        //LOG("once timer\n");
        osal_start_timerEx( key_TaskID, KEY_DEMO_ONCE_TIMER, 5000);
        return (events ^ KEY_DEMO_ONCE_TIMER);
    }

    if( events & KEY_DEMO_CYCLE_TIMER)
    {
        //LOG("recycle timer\n");
        return (events ^ KEY_DEMO_CYCLE_TIMER);
    }

    if( events & HAL_KEY_EVENT)                                                     //do not modify,key will use it
    {
        for (uint8 i = 0; i < HAL_KEY_NUM; ++i)
        {
            if ((key_state.temp[i].in_enable == TRUE)||
                    (key_state.key[i].state == HAL_STATE_KEY_RELEASE_DEBOUNCE))
            {
                gpio_key_timer_handler(i);
            }
        }

        return (events ^ HAL_KEY_EVENT);
    }

    #ifdef HAL_KEY_SUPPORT_LONG_PRESS

    if( events & KEY_DEMO_LONG_PRESS_EVT)
    {
        for (int i = 0; i < HAL_KEY_NUM; ++i)
        {
            if(key_state.key[i].state == HAL_KEY_EVT_PRESS)
            {
                LOG("key:%d gpio:%d	",i,key_state.key[i].pin);
                LOG("key(long press down)");
                //user app code long press down process
            }
        }

        return (events ^ KEY_DEMO_LONG_PRESS_EVT);
    }

    #endif
    return 0;
}

/*********************************************************************
*********************************************************************/
