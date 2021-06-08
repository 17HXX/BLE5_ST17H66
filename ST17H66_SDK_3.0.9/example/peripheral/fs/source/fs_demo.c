/**************************************************************************************************
*******
**************************************************************************************************/

/**************************************************************************************************
    Filename:       fs_demo.c
    Revised:        $Date $
    Revision:       $Revision $


**************************************************************************************************/

/*********************************************************************
    INCLUDES
*/
#include "OSAL.h"
#include "gpio.h"
#include "clock.h"
#include "timer.h"
#include "fs_test.h"
#include "fs_demo.h"
#include "log.h"

static uint8 fs_TaskID;


/*********************************************************************
    @fn      AP_TIMER_Demo_Init

    @brief

    @param

    @return
*/

void fs_Init( uint8 task_id )
{
    fs_TaskID = task_id;
    #if (FS_TEST_TYPE == FS_MODULE_TEST)
    osal_start_timerEx(fs_TaskID, FS_TEST_EVT,1000);
    #elif (FS_TEST_TYPE == FS_EXAMPLE)
    osal_start_timerEx(fs_TaskID, FS_EXAMPLE_EVT,1000);
    #elif (FS_TEST_TYPE == FS_TIMING_TEST)
    osal_start_timerEx(fs_TaskID, FS_TIMING_EVT,1000);
    #elif (FS_TEST_TYPE == FS_XIP_TEST)
    osal_start_timerEx(fs_TaskID, FS_XIP_EVT,1000);
    #else
#error please check your config parameter
    #endif
}
uint16 fs_ProcessEvent( uint8 task_id, uint16 events )
{
    if (events & FS_TEST_EVT)
    {
        #if (FS_TEST_TYPE == FS_MODULE_TEST)
        LOG("ftcase_write_del_test\n");
        ftcase_write_del_test();
        osal_start_timerEx(fs_TaskID, FS_EXAMPLE_EVT,10000);
        #endif
        return (events ^ FS_TEST_EVT);
    }

    if (events & FS_EXAMPLE_EVT)
    {
        #if (FS_TEST_TYPE == FS_EXAMPLE)
        LOG("fs_example\n");
        fs_example();
        osal_start_timerEx(fs_TaskID, FS_EXAMPLE_EVT,500);
        #endif
        return (events ^ FS_EXAMPLE_EVT);
    }

    if (events & FS_XIP_EVT)
    {
        #if (FS_TEST_TYPE == FS_XIP_TEST)
        LOG("fs_xip_test\n");
        fs_xip_test();
        osal_start_timerEx(fs_TaskID, FS_XIP_EVT,500);
        #endif
        return (events ^ FS_EXAMPLE_EVT);
    }

    if (events & FS_TIMING_EVT)
    {
        #if (FS_TEST_TYPE == FS_TIMING_TEST)
        LOG("fs_timing_test\n");
        fs_timing_test();
        #endif
        return (events ^ FS_TIMING_EVT);
    }

    return 0;
}

