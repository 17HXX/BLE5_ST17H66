/**************************************************************************************************
*******
**************************************************************************************************/

/**************************************************************************************************
    Filename:       simpleBLECentral.h
    Revised:        $Date: 2011-03-03 15:46:41 -0800 (Thu, 03 Mar 2011) $
    Revision:       $Revision: 12 $

    Description:    This file contains the Simple BLE Central sample application
                  definitions and prototypes.

**************************************************************************************************/

#ifndef SIMPLEBLECENTRAL_H
#define SIMPLEBLECENTRAL_H

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
#define START_DEVICE_EVT                                0x0001
#define CENTRAL_INIT_DONE_EVT                           0x0002
#define CENTRAL_DISCOVER_DEVDONE_EVT                    0x0004
#define START_DISCOVERY_SERVICE_EVT                     0x0008
#define UPD_CHAN_MAP_EVENT                              0x0010
#define UPD_CONN_PARAM                                  0x0020
#define SBC_CANCEL_CONN                                 0x0040
#define SBC_TERMINATED_CONN                             0x0080
#define UPD_DATA_LENGTH_EVT                             0x0100
#define UPD_PHY_MODE_EVT                                0x0200
#define SBC_READ_WRITE_TEST_EVT                         0x0400
#define SLA_DATA_LENGTH_EVT                             0x0800
#define SLA_PHY_MODE_EVT                                0x1000

//#define SBC_PERIODIC_EVT                                0x2000
#define START_CHAR_DATA_TEST                            0x4000

#define START_N100MS_EVT			                            0x2000

//#define SBC_START_DISC                                0x0040

//#define SBC_SEND_RAW_DATA                             0x0010
//#define PER_STATS_EVT                                 0x0100

// add by zhufei.zhang


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
    FUNCTIONS
*/

/*
    Task Initialization for the BLE Application
*/
extern void SimpleBLECentral_Init( uint8 task_id );

/*
    Task Event Processor for the BLE Application
*/
extern uint16 SimpleBLECentral_ProcessEvent( uint8 task_id, uint16 events );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SIMPLEBLECENTRAL_H */
