/**************************************************************************************************
*******
**************************************************************************************************/

/**************************************************************************************************
  Filename:       SimpleBLEPeripheral_Main.c
  Revised:        
  Revision:        

  Description:    This file contains the main and callback functions for
                  the Simple BLE Peripheral sample application.

 **************************************************************************************************/

/**************************************************************************************************
 *                                           Includes
 **************************************************************************************************/
/* Hal Drivers */

/* OSAL */
#include "OSAL.h"
#include "OSAL_Tasks.h"
#include "OSAL_PwrMgr.h"
#include "osal_snv.h"

#include "ll_sleep.h"


#include "uart.h"
/**************************************************************************************************
 * FUNCTIONS
 **************************************************************************************************/
#include "comdef.h"
//#include "OnBoard.h"
//#include "hal_mcu.h"
#include "OSAL.h"
#include "OSAL_Tasks.h"
#include "OSAL_Timers.h"
#include "OSAL_PwrMgr.h"

#include "timer.h"
#include "ll_sleep.h"
#include "jump_function.h"
#include "global_config.h"
extern pwrmgr_attribute_t pwrmgr_attribute;
extern uint32 ll_remain_time;

/*********************************************************************
 * EXTERNAL VARIABLES
 */


/**************************************************************************************************
 * @fn          main
 *
 * @brief       Start of application.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
int app_main(void)
{

    /* Initialize the operating system */
    osal_init_system();

    osal_pwrmgr_device( PWRMGR_BATTERY );

    /* Start OSAL */
    osal_start_system(); // No Return from here
    return 0;
}

/**************************************************************************************************
                                           CALL-BACKS
**************************************************************************************************/


/*************************************************************************************************
**************************************************************************************************/
