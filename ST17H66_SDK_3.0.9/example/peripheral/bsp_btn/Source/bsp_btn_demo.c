/**************************************************************************************************
*******
**************************************************************************************************/

/**************************************************************************************************
    Filename:       bsp_btn_demo.c
    Revised:        $Date $
    Revision:       $Revision $


**************************************************************************************************/

/*********************************************************************
    INCLUDES
*/
#include <string.h>
#include "gpio.h"
#include "bsp_gpio.h"
#include "kscan.h"
#include "log.h"
#include "OSAL.h"
#include "OSAL_Timers.h"
#include "bsp_button_task.h"
#include "bsp_btn_demo.h"

static uint8 Demo_TaskID;

#if (BSP_BTN_HARDWARE_CONFIG == BSP_BTN_JUST_GPIO)

BTN_T usr_sum_btn_array[BSP_TOTAL_BTN_NUM];

#if (BSP_COMBINE_BTN_NUM > 0)
uint32_t usr_combine_btn_array[BSP_COMBINE_BTN_NUM] =
{
    (BIT(0)|BIT(1)),
    (BIT(0)|BIT(2)),
};
#endif

void hal_bsp_btn_callback(uint8_t evt)
{
    LOG("gpio evt:0x%x  ",evt);

    switch(BSP_BTN_TYPE(evt))
    {
    case BSP_BTN_PD_TYPE:
        LOG("press down ");
        break;

    case BSP_BTN_UP_TYPE:
        LOG("press up ");
        break;

    case BSP_BTN_LPS_TYPE:
        LOG("long press start ");
        break;

    case BSP_BTN_LPK_TYPE:
        LOG("long press keep ");
        break;

    default:
        LOG("unexpected ");
        break;
    }

    LOG("value:%d\n",BSP_BTN_INDEX(evt));
}

Gpio_Btn_Info gpio_btn_info =
{
    {P14,P15,P26},
    hal_bsp_btn_callback,
};

#endif

#if (BSP_BTN_HARDWARE_CONFIG == BSP_BTN_JUST_KSCAN)

KSCAN_ROWS_e rows[NUM_KEY_ROWS] = {KEY_ROW_P00,KEY_ROW_P02,KEY_ROW_P25,KEY_ROW_P18};
KSCAN_COLS_e cols[NUM_KEY_COLS] = {KEY_COL_P01,KEY_COL_P03,KEY_COL_P24,KEY_COL_P20};

BTN_T usr_sum_btn_array[BSP_TOTAL_BTN_NUM];

#if (BSP_COMBINE_BTN_NUM > 0)
uint32_t usr_combine_btn_array[BSP_COMBINE_BTN_NUM] =
{
    (BIT(0)|BIT(1)),
    (BIT(2)|BIT(3)),
};
#endif

void hal_bsp_btn_callback(uint8_t evt)
{
    LOG("kscan evt:0x%x  ",evt);

    switch(BSP_BTN_TYPE(evt))
    {
    case BSP_BTN_PD_TYPE:
        LOG("press down ");
        break;

    case BSP_BTN_UP_TYPE:
        LOG("press up ");
        break;

    case BSP_BTN_LPS_TYPE:
        LOG("long press start ");
        break;

    case BSP_BTN_LPK_TYPE:
        LOG("long press keep ");
        break;

    default:
        LOG("unexpected ");
        break;
    }

    LOG("value:%d\n",BSP_BTN_INDEX(evt));
}

void hal_kscan_btn_check(bsp_btn_callback_t cb)
{
    if((NUM_KEY_ROWS != sizeof(rows)/sizeof(rows[0])) || (NUM_KEY_COLS != sizeof(cols)/sizeof(cols[0])))
    {
        return;
    }

    #if (BSP_COMBINE_BTN_NUM > 0)

    if(BSP_COMBINE_BTN_NUM != sizeof(usr_combine_btn_array)/sizeof(usr_combine_btn_array[0]))
    {
        return;
    }

    #endif

    if(cb == NULL)
    {
        return;
    }

    #if (BSP_COMBINE_BTN_NUM > 0)

    for(int i = 0; i < BSP_COMBINE_BTN_NUM; i++)
    {
        if(usr_combine_btn_array[i] == 0x00)
        {
            return;
        }
    }

    #endif
    bsp_btn_cb = cb;
    bsp_btn_kscan_flag = TRUE;
}
#endif


#if (BSP_BTN_HARDWARE_CONFIG == BSP_BTN_GPIO_AND_KSCAN)

KSCAN_ROWS_e rows[NUM_KEY_ROWS] = {KEY_ROW_P00,KEY_ROW_P02,KEY_ROW_P25,KEY_ROW_P18};
KSCAN_COLS_e cols[NUM_KEY_COLS] = {KEY_COL_P01,KEY_COL_P03,KEY_COL_P24,KEY_COL_P20};

BTN_T usr_sum_btn_array[BSP_TOTAL_BTN_NUM];

#if (BSP_COMBINE_BTN_NUM > 0)
uint32_t usr_combine_btn_array[BSP_COMBINE_BTN_NUM] =
{
    (BIT(2)|BIT(3)),
    (BIT(16)|BIT(17)),
};
#endif

void hal_bsp_btn_callback(uint8_t evt)
{
    LOG("kscan evt:0x%x  ",evt);

    switch(BSP_BTN_TYPE(evt))
    {
    case BSP_BTN_PD_TYPE:
        LOG("press down ");
        break;

    case BSP_BTN_UP_TYPE:
        LOG("press up ");
        break;

    case BSP_BTN_LPS_TYPE:
        LOG("long press start ");
        break;

    case BSP_BTN_LPK_TYPE:
        LOG("long press keep ");
        break;

    default:
        LOG("unexpected ");
        break;
    }

    LOG("value:%d\n",BSP_BTN_INDEX(evt));
}

void hal_kscan_btn_check(bsp_btn_callback_t cb)
{
    if((NUM_KEY_ROWS != sizeof(rows)/sizeof(rows[0])) || (NUM_KEY_COLS != sizeof(cols)/sizeof(cols[0])))
    {
        return;
    }

    #if (BSP_COMBINE_BTN_NUM > 0)

    if(BSP_COMBINE_BTN_NUM != sizeof(usr_combine_btn_array)/sizeof(usr_combine_btn_array[0]))
    {
        return;
    }

    #endif

    if(cb == NULL)
    {
        return;
    }

    #if (BSP_COMBINE_BTN_NUM > 0)

    for(int i = 0; i < BSP_COMBINE_BTN_NUM; i++)
    {
        if(usr_combine_btn_array[i] == 0x00)
        {
            return;
        }
    }

    #endif
    bsp_btn_cb = cb;
    bsp_btn_kscan_flag = TRUE;
}

Gpio_Btn_Info gpio_btn_info =
{
    {P14,P15,P26},
    hal_bsp_btn_callback,
};
#endif

void Demo_Init( uint8 task_id )
{
    Demo_TaskID = task_id;
    #if (BSP_BTN_HARDWARE_CONFIG == BSP_BTN_JUST_GPIO)

    if(PPlus_SUCCESS == hal_gpio_btn_init(&gpio_btn_info))
    {
        bsp_btn_gpio_flag = TRUE;
    }
    else
    {
        LOG("hal_gpio_btn_init error:%d\n",__LINE__);
    }

    #endif
    #if (BSP_BTN_HARDWARE_CONFIG == BSP_BTN_JUST_KSCAN)
    hal_kscan_btn_check(hal_bsp_btn_callback);

    if(bsp_btn_kscan_flag != TRUE)
    {
        LOG("hal_kscan_btn_check error:%d\n",__LINE__);
    }

    #endif
    #if (BSP_BTN_HARDWARE_CONFIG == BSP_BTN_GPIO_AND_KSCAN)
    hal_kscan_btn_check(hal_bsp_btn_callback);

    if(bsp_btn_kscan_flag != TRUE)
    {
        LOG("hal_kscan_btn_check error:%d\n",__LINE__);
    }

    if(PPlus_SUCCESS == hal_gpio_btn_init(&gpio_btn_info))
    {
        bsp_btn_gpio_flag = TRUE;
    }
    else
    {
        LOG("hal_gpio_btn_init error:%d\n",__LINE__);
    }

    #endif
}

uint16 Demo_ProcessEvent( uint8 task_id, uint16 events )
{
    if(Demo_TaskID != task_id)
    {
        return 0;
    }

    return 0;
}

