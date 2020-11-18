

/**************************************************************************************************
  Filename:       bleuart_service.h
  Revised:         
  Revision:        

  Description:    This file contains the Simple GATT profile definitions and
                  prototypes.

 **************************************************************************************************/

#ifndef _BLE_UART_SERVICE_H
#define _BLE_UART_SERVICE_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "att.h"

/*********************************************************************
 * CONSTANTS
 */

// Profile Parameters
#define PROFILE_RAWPASS_CHAR_RX                   0  // RW uint8 - Profile Characteristic 1 value 
#define PROFILE_RAWPASS_CHAR_TX                   1  // RW uint8 - Profile Characteristic 2 value
  
// Simple Keys Profile Services bit fields
#define PROFILE_RAWPASS_SERVICE               0x00000001


#define RAWPASS_RX_BUFF_SIZE                  1


enum{
  bleuart_EVT_TX_NOTI_DISABLED = 1,
  bleuart_EVT_TX_NOTI_ENABLED,
  bleuart_EVT_BLE_DATA_RECIEVED,
};
  

typedef struct{
  uint8_t   ev;
  uint16_t  param;
  void*     data;
}bleuart_Evt_t;

typedef void (*bleuart_ProfileChangeCB_t)(bleuart_Evt_t* pev);

    

/*********************************************************************
 * API FUNCTIONS 
 */


/*
 * bleuart_AddService- Initializes the raw pass GATT Profile service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 */

extern bStatus_t bleuart_AddService( bleuart_ProfileChangeCB_t cb);


  

extern uint8 bleuart_NotifyIsReady(void);

extern bStatus_t bleuart_Notify( uint16 connHandle, attHandleValueNoti_t *pNoti, uint8 taskId );


extern CONST uint8 bleuart_ServiceUUID[ATT_UUID_SIZE];
extern CONST uint8 bleuart_RxCharUUID[ATT_UUID_SIZE];
extern CONST uint8 bleuart_TxCharUUID[ATT_UUID_SIZE];
/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* _BLE_UART_SERVICE_H */

