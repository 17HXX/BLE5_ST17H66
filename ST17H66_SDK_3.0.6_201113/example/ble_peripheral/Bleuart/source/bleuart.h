
/**************************************************************************************************
  Filename:       bleuart.h
  Revised:         
  Revision:        

  Description:    This file contains the Simple BLE Peripheral sample application
                  definitions and prototypes.

 
**************************************************************************************************/

#ifndef SIMPLEBLEPERIPHERAL_H
#define SIMPLEBLEPERIPHERAL_H

#include "types.h" 
#include "rf_phy_driver.h"
#include "bleuart_protocol.h"
/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */ 
 
 

#define INVALID_CONNHANDLE                    0xFFFF

// Simple BLE Peripheral Task Events
// Simple BLE Peripheral Task Events
#define BUP_OSAL_EVT_START_DEVICE                         0x0001
#define BUP_OSAL_EVT_BLE_TIMER                            0x0002
#define BUP_OSAL_EVT_ENTER_NOCONN                         0x0004
#define BUP_OSAL_EVT_RESET_ADV                            0x0008

#define BUP_OSAL_EVT_CCCD_UPDATE                          0x0010
#define BUP_OSAL_EVT_UART_DATA_RX                         0x0020
#define BUP_OSAL_EVT_NOTIFY_DATA                          0x0040
#define BUP_OSAL_EVT_UARTRX_TIMER                         0x0080
#define BUP_OSAL_EVT_UART_TX_COMPLETE                     0x0100
#define BUP_OSAL_EVT_UART_TO_TIMER                        0x0200
#define BUP_OSAL_EVT_RF433_KEY                            0x0400// chendy add just for
#define BUP_OSAL_EVT_AT                            		  0x0800// JFM add for AT


//#define FLOW_CTRL_IO_UART_TX          P18 //mobile --> ble --> uart --> host
//#define FLOW_CTRL_IO_BLE_TX           P23 //host-->uart-->ble-->mobile
#define FLOW_CTRL_IO_HOST_WAKEUP      P14 //host mcu wakeup befor host-->uart-->620x
#define UART_INDICATE_LED             P15
//#define FLOW_CTRL_IO_BLE_CONNECTION   P20 //indicate host 620x BLE connection status: 1: connected; 0: advertising


#define io_lock(io) {hal_gpio_write(io, 1);hal_gpio_pull_set(io, STRONG_PULL_UP);}
#define io_unlock(io) {hal_gpio_write(io, 0);hal_gpio_pull_set(io, PULL_DOWN);}

#define FLOW_CTRL_UART_TX_LOCK()   // io_lock(FLOW_CTRL_IO_UART_TX)
#define FLOW_CTRL_UART_TX_UNLOCK() // io_unlock(FLOW_CTRL_IO_UART_TX)

#define FLOW_CTRL_BLE_TX_LOCK()    // io_lock(FLOW_CTRL_IO_BLE_TX)
#define FLOW_CTRL_BLE_TX_UNLOCK()   //io_unlock(FLOW_CTRL_IO_BLE_TX)

#define FLOW_CTRL_BLE_CONN()     //io_lock(FLOW_CTRL_IO_BLE_CONNECTION)
#define FLOW_CTRL_BLE_DISCONN() // io_unlock(FLOW_CTRL_IO_BLE_CONNECTION)

extern uint8 bleuart_TaskID;   // Task ID for internal task/event processing
extern uint16 gapConnHandle;


void bleuart_Init( uint8 task_id );
uint16_t bleuart_conn_interval(void);
uint16 bleuart_ProcessEvent( uint8 task_id, uint16 events );

extern void on_BUP_Evt(BUP_Evt_t* pev);
extern uint8 Modify_BLEDevice_Data;
extern uint8  advint;
extern uint8 AT_bleuart_auto;
extern uint8 AT_bleuart_sleep;
extern uint8 AT_bleuart_txpower;

extern uint8 AT_Tx_Power[8];
extern uint8 at_tx_power;

extern uint8*scanR;
extern uint8*advertdata;
extern uint8 Modify_Mac_Data;



/*********************************************************************
*********************************************************************/

#endif /* SIMPLEBLEPERIPHERAL_H */

