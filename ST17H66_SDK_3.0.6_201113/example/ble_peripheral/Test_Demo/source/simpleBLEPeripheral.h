/**************************************************************************************************
THIS IS EMPTY HEADER
**************************************************************************************************/

/**************************************************************************************************
  Filename:       simpleBLEperipheral.h
  Revised:         
  Revision:        

  Description:    This file contains the Simple BLE Peripheral sample application
                  definitions and prototypes.

 
**************************************************************************************************/

#ifndef SIMPLEBLEPERIPHERAL_H
#define SIMPLEBLEPERIPHERAL_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */


// Simple BLE Peripheral Task Events
#define SBP_START_DEVICE_EVT                           0x0001
#define SBP_ADD_RL_EVT                                 0x0002
#define SBP_PERIODIC_EVT                               0x0004
#define SBP_RESET_ADV_EVT                              0x0008
#define SBP_DISABLE_LATENCY_TEST_EVT                   0x0010
#define SBP_ENABLE_LATENCY_EVT                         0x0020
//#define SBP_DLE_CHANGE_EVT                             0x0020
//#define SBP_PHY_UPDATE_EVT                             0x0040
//#define SBP_CONN_NOTIFY_EVT                            0x0080
#define SBP_RTC_TEST_EVT                               0x0100

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task Initialization for the BLE Application
 */
extern void SimpleBLEPeripheral_Init( uint8 task_id );

/*
 * Task Event Processor for the BLE Application
 */
extern uint16 SimpleBLEPeripheral_ProcessEvent( uint8 task_id, uint16 events );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SIMPLEBLEPERIPHERAL_H */
