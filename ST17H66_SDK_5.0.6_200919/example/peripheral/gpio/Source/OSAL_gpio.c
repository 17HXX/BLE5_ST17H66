/**************************************************************************************************
*******
**************************************************************************************************/

/**************************************************************************************************
  Filename:       OSAL_heartrate.c
  Revised:        $Date: 2011-03-30 20:15:59 -0700 (Wed, 30 Mar 2011) $
  Revision:       $Revision: 16 $


**************************************************************************************************/

/**************************************************************************************************
 *                                            INCLUDES
 **************************************************************************************************/
#include "OSAL.h"
#include "OSAL_Tasks.h"
#include "ll.h"

/* Application */
#include "gpio_demo.h"
/*********************************************************************
 * GLOBAL VARIABLES
 */

// The order in this table must be identical to the task initialization calls below in osalInitTask.
const pTaskEventHandlerFn tasksArr[] =
{
    LL_ProcessEvent,
//    Pulse_Measure_ProcessEvent,
//    GPIO_Wakeup_ProcessEvent,
	Key_ProcessEvent,
};

const uint8 tasksCnt = sizeof( tasksArr ) / sizeof( tasksArr[0] );
uint16 *tasksEvents;

/*********************************************************************
 * FUNCTIONS
 *********************************************************************/

/*********************************************************************
 * @fn      osalInitTasks
 *
 * @brief   This function invokes the initialization function for each task.
 *
 * @param   void
 *
 * @return  none
 */
void osalInitTasks( void )
{
    uint8 taskID = 0;

    tasksEvents = (uint16 *)osal_mem_alloc( sizeof( uint16 ) * tasksCnt);
    osal_memset( tasksEvents, 0, (sizeof( uint16 ) * tasksCnt));

    LL_Init( taskID++);
  /*
	Application
	*/
//    Pulse_Measure_Init(taskID);
//	GPIO_Wakeup_Init(taskID);
	Key_Demo_Init(taskID);
}

/*********************************************************************
*********************************************************************/
