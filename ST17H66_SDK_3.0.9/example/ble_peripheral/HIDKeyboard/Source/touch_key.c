/**************************************************************************************************
*******
**************************************************************************************************/



#include "error.h"
#include "gpio.h"
#include "touch_key.h"
#include "log.h"

static touch_evt_hdl_t s_touch_hdl = NULL;

static void pin_event_handler(GPIO_Pin_e pin, IO_Wakeup_Pol_e type)
{
    if(type == NEGEDGE)
    {
        //em70xx_start();
        s_touch_hdl(TOUCH_EVT_PRESS);
    }
}

int touch_init(touch_evt_hdl_t hdl)
{
    //uint32_t ret;
    if(hdl == NULL)
        return PPlus_ERR_INVALID_PARAM;

    s_touch_hdl = hdl;
    hal_gpio_pull_set(P14, PULL_DOWN);
    hal_gpioin_register(P14, NULL, pin_event_handler);//pin_event_handler);
    //LOG("touch pin input enable %d\n", ret);
    return PPlus_SUCCESS;
}


