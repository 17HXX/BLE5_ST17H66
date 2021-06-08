/**************************************************************************************************
*******
**************************************************************************************************/

/******************************************************************************

 *****************************************************************************/

#ifndef HIDEMUKBD_H
#define HIDEMUKBD_H

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

// Task Events
#define START_DEVICE_EVT                              0x0001
#define HID_TEST_EVT                                  0x0100

/*********************************************************************
    MACROS
*/

/*********************************************************************
    FUNCTIONS
*/

/*********************************************************************
    GLOBAL VARIABLES
*/

extern uint8 hidKbdTaskId;
extern uint8 g_instant_cnt;

/*
    Task Initialization for the BLE Application
*/
extern void HidKbd_Init( uint8 task_id );

/*
    Task Event Processor for the BLE Application
*/
extern uint16 HidKbd_ProcessEvent( uint8 task_id, uint16 events );
extern uint8 hidKbdSendVoiceCMDtReport( uint8 keycode );
extern void hidCCSendReportKey( uint8 cmd, bool keyPressed);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /*HIDEMUKBD_H */
