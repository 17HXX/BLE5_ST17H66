/**************************************************************************************************
*******
**************************************************************************************************/

#ifndef MULTI_ROLE_H
#define MULTI_ROLE_H

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


// Simple BLE Central Task Events
#define START_DEVICE_EVT                              0x0001
#define BUP_OTA_PERIOD_EVT                            0x0020
#define MULTIROLE_HID_SEND_REPORT_EVT                 0x1000
#define MULTIROLE_HID_IDLE_EVT                        0x2000
#define MULTIROLE_PERIOD_EVT                          0x4000


/*********************************************************************
    MACROS
*/

// LCD macros
#if HAL_LCD == TRUE
#define LCD_WRITE_STRING(str, option)                       HalLcdWriteString( (str), (option))
#define LCD_WRITE_SCREEN(line1, line2)                      HalLcdWriteScreen( (line1), (line2) )
#define LCD_WRITE_STRING_VALUE(title, value, format, line)  HalLcdWriteStringValue( (title), (value), (format), (line) )
#else
#define LCD_WRITE_STRING(str, option)
#define LCD_WRITE_SCREEN(line1, line2)
#define LCD_WRITE_STRING_VALUE(title, value, format, line)
#endif

/*********************************************************************
    Typedef
*/
//typedef struct
//{
//  GAPMultiRole_states_t state;
//  GAPMultiRole_State_t role;
//
//  // SMP
//  uint8 ConnSecure;
//  uint8 PairingStarted;
//  // slave
//  uint8 notify;
//  uint8 dle;
//  uint8 MTUExchange;
//
//  // master
//  uint8  SDPDone;
//
//  uint16 cccdHandle;
//  uint16 enableCCCD;
//  uint16 Char5ValueHandle;
//  uint16 Char5isWrite;
//
//}MultiRoleApp_Link_t;

/*********************************************************************
    FUNCTIONS
*/

/*
    Task Initialization for the BLE Application
*/
extern void multiRoleApp_Init( uint8 task_id );

/*
    Task Event Processor for the BLE Application
*/
extern uint16 multiRoleApp_ProcessEvent( uint8 task_id, uint16 events );


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SIMPLEBLECENTRAL_H */
