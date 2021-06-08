
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
#define SBP_NOTIFY_EVT									  0x1000


#if (AT_UART==1)

#define MY_MODIFY_ID					0x80

#define MY_DEVICE_NAME_ID				0x81
#define MY_MAC_ADDRESS_ID				0x82
#define MY_UART_BAUDRATE_ID				0x83
#define MY_ADV_INTERVAL_ID				0x84
#define MY_BLE_UART_AUTO_ID				0x85
#define MY_TX_POWER_ID					0x86
#define MY_RESERVED_DATA_ID				0x87

#define MY_RESTORE_DEVICE_NAME_ID		0x91
#define MY_RESTORE_MAC_ADDRESS_ID		0x92
#define MY_RESTORE_UART_BAUDRATE_ID		0x93
#define MY_RESTORE_ADV_INTERVAL_ID		0x94
#define MY_RESTORE_BLE_UART_AUTO_ID		0x95
#define MY_RESTORE_TX_POWER_ID			0x96
#define MY_RESTORE_RESERVED_DATA_ID		0x97

#define MY_MODIFY_ID_LEN				1
#define MY_DEVICE_NAME_ID_LEN			20
#define MY_MAC_ADDRESS_ID_LEN			6
#define MY_UART_BAUDRATE_ID_LEN			4
#define MY_ADV_INTERVAL_ID_LEN			1
#define MY_BLE_UART_AUTO_ID_LEN			1
#define MY_TX_POWER_ID_LEN				1
#define MY_RESERVED_DATA_ID_LEN			8

#endif



#define UART_TX_PIN					  P34	//串口TX引脚
#define UART_RX_PIN					  P2	//串口RX引脚
#define FLOW_CTRL_IO_HOST_WAKEUP      P15 	//串口低功耗模式唤醒脚
#define UART_INDICATE_LED             P14	//蓝牙连接指示灯
#define FLOW_CTRL_IO_BLE_CONNECTION   P18	//IO控制断开蓝牙连接


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
extern uint8 device_connect_state;

extern uint8 AT_Tx_Power[8];
extern uint8 at_tx_power;

extern uint8*scanR;
extern uint8*advertdata;
extern uint8 Modify_Mac_Data;
extern uint8 AT_cnt_advdata;




/*********************************************************************
*********************************************************************/

#endif /* SIMPLEBLEPERIPHERAL_H */

