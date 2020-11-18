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
#include "types.h"
/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */

#define CHANGE_ADV_TYPE_TIME   30000  //30S如果没有连接就变成不可连接广播
// Simple BLE Peripheral Task Events
#define SBP_START_DEVICE_EVT                           0x0001
#define SBP_ADD_RL_EVT                                 0x0002
#define SBP_NOTIFY_EVT                               0x0004
#define SBP_RESET_ADV_EVT                              0x0008
#define SBP_CONNECTED_EVT                              0x0010

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
extern uint16 gapConnHandle;
extern uint16 SimpleBLEPeripheral_ProcessEvent( uint8 task_id, uint16 events );
extern void simpleProfile_Set_Notify_Event(void); //notify cmd set rsp data
extern void SimpleBLEPeripheral_SetDevName(uint8*data,uint8 len);
extern void SimpleBLEPeripheral_SetBeaconUUID(uint8* data,uint8 len);
extern void SimpleBLEPeripheral_SetMajor(uint8* data,uint8 len);
extern void SimpleBLEPeripheral_SetMinor(uint8* data,uint8 len);
extern void SimpleBLEPeripheral_SetRSSI(uint8 data);
extern void SimpleBLEPeripheral_SetAdvIntvlTime(uint16 data);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SIMPLEBLEPERIPHERAL_H */
