/**************************************************************************************************
*******
**************************************************************************************************/

/**************************************************************************************************
    Filename:       OSAL_heartrate.c
    Revised:        $Date: 2011-03-30 20:15:59 -0700 (Wed, 30 Mar 2011) $
    Revision:       $Revision: 16 $


**************************************************************************************************/

/**************************************************************************************************
                                              INCLUDES
 **************************************************************************************************/
#include "OSAL.h"
#include "OSAL_Tasks.h"

/* HCI */
#include "hci_tl.h"

/* LL */
#include "ll.h"

/* L2CAP */
#include "l2cap.h"

/* gap */
#include "gap.h"
#include "gapgattserver.h"
#include "gapbondmgr.h"

/* GATT */
#include "gatt.h"

#include "gattservapp.h"

/* Profiles */
#include "peripheral.h"

/* Application */
#include "ota.h"

/*********************************************************************
    GLOBAL VARIABLES
*/

// The order in this table must be identical to the task initialization calls below in osalInitTask.
const pTaskEventHandlerFn tasksArr[] =
{
    LL_ProcessEvent,
    HCI_ProcessEvent,
    L2CAP_ProcessEvent,
    GAP_ProcessEvent,
    GATT_ProcessEvent,
    SM_ProcessEvent,
    GAPRole_ProcessEvent,
    GATTServApp_ProcessEvent,
    otaApp_ProcessEvent
};

const uint8 tasksCnt = sizeof(tasksArr) / sizeof(tasksArr[0]);
uint16* tasksEvents;

/*********************************************************************
    FUNCTIONS
 *********************************************************************/

/*********************************************************************
    @fn      osalInitTasks

    @brief   This function invokes the initialization function for each task.

    @param   void

    @return  none
*/
void osalInitTasks(void)
{
    uint8 taskID = 0;
    tasksEvents = (uint16*)osal_mem_alloc(sizeof(uint16) * tasksCnt);
    osal_memset(tasksEvents, 0, (sizeof(uint16) * tasksCnt));
    /* LL Task */
    LL_Init(taskID++);
    /* HCI Task */
    HCI_Init(taskID++);
    /* L2CAP Task */
    L2CAP_Init(taskID++);
    /* GAP Task */
    GAP_Init(taskID++);
    /* GATT Task */
    GATT_Init(taskID++);
    /* SM Task */
    SM_Init(taskID++);
    /* Profiles */
    GAPRole_Init(taskID++);
    GATTServApp_Init(taskID++);
    /* Application */
    otaApp_Init(taskID);
}

/*********************************************************************
*********************************************************************/
