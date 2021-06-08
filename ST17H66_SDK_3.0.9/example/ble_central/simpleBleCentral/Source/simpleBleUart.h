/**************************************************************************************************
THIS IS EMPTY HEADER
**************************************************************************************************/

/*********************************************************************
    INCLUDES
*/

#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "OSAL_bufmgr.h"
#include "gatt.h"
#include "ll.h"
#include "ll_common.h"
#include "hci.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "central.h"
#include "gapbondmgr.h"
#include "simpleGATTprofile_ota.h"
#include "simpleBLECentral.h"
#include "timer.h"
#include "log.h"
#include "ll_def.h"
#include "global_config.h"
#include "flash.h"
#include "rflib.h"
#include "clock.h"



#define UART_RX_BUF_SIZE  1024 //512
#define UART_TX_BUF_SIZE  512


	

typedef struct {
  uint8_t ev;

}BUP_Evt_t;

typedef struct{
	uint8 len;
	uint8 data[100];
}AT_BLEUART_RX_t;


enum{
  BUP_RX_ST_IDLE = 0,
  BUP_RX_ST_DELAY_SLOT,
  BUP_RX_ST_SENDING
};

enum{
  BUP_TX_ST_IDLE = 0,
  BUP_TX_ST_DELAY_SLOT,
  BUP_TX_ST_SENDING
};

typedef struct{
  bool    conn_state;
  //uart_rx
  uint8_t rx_state;
  uint8_t rx_size;
  uint8_t rx_offset;
  uint8_t rx_buf[UART_RX_BUF_SIZE];

  //uart tx
  uint8_t tx_state;
  uint8_t tx_size;
  uint8_t tx_buf[UART_TX_BUF_SIZE];


  uint8_t hal_uart_rx_size;
  uint8_t hal_uart_rx_buf[UART_RX_BUF_SIZE];
  uint8_t hal_uart_tx_buf[UART_TX_BUF_SIZE];
  
}BUP_ctx_t;
typedef void (*BUP_CB_t)(BUP_Evt_t* pev);

extern int BUP_init(BUP_CB_t cb);
extern void on_BUP_Evt(BUP_Evt_t* pev);

extern void uart_send_buf(uint8 *data, uint32 size);

