/**************************************************************************************************
*******
**************************************************************************************************/

/**************************************************************************************************
    Filename:       heartrate.h
    Revised:        $Date $
    Revision:       $Revision $


**************************************************************************************************/

#ifndef HEARTRATE_H
#define HEARTRATE_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
    INCLUDES
*/

/*********************************************************************
    CONSTANTS
*/

#define START_DEVICE_EVT 0x0001
#define OTA_TIMER_EVT 0x0002

/*********************************************************************
    MACROS
*/

/*********************************************************************
    FUNCTIONS
*/
extern uint8 ota_TaskID; // Task ID for internal task/event processing

/*
    Task Initialization for the BLE Application
*/
extern void otaApp_Init(uint8 task_id);

/*
    Task Event Processor for the BLE Application
*/
extern uint16 otaApp_ProcessEvent(uint8 task_id, uint16 events);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* HEARTRATE_H */
