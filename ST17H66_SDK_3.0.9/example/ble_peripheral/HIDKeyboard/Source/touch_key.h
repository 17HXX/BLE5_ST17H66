/**************************************************************************************************
*******
**************************************************************************************************/


/**************************************************************
    Module Name: touch key
    File name:   touch_key.h
    Brief description:
      key driver module
    Author:  Eagle.Lao
    Data:    2017-07-01
    Revision:V0.01
****************************************************************/

#ifndef _TOUCH_KEY_H_FILE
#define _TOUCH_KEY_H_FILE

typedef enum
{
    TOUCH_EVT_TOUCH,
    TOUCH_EVT_PRESS,
    TOUCH_EVT_LONG_PRESS,
    TOUCH_EVT_LL_PRESS  //very long press
} touch_evt_t;

#define     IO_TOUCH_1_PIN          25u
#define     TOUCH_INT_GPIOTE_USER_ID   (1<<(IO_TOUCH_1_PIN))

typedef void (* touch_evt_hdl_t)(touch_evt_t key_evt);

int touch_init(touch_evt_hdl_t hdl);

#endif

