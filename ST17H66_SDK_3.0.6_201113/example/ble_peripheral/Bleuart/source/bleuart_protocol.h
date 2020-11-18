

#ifndef _BLE_UART_PROTOCOL_H
#define _BLE_UART_PROTOCOL_H

#include "types.h"
//#include "bleuart.h"
#include "osal_snv.h"
#include "flash.h"

typedef struct {
  uint8_t ev;

}BUP_Evt_t;

typedef struct{
	uint8 len;
	uint8 data[100];
}AT_BLEUART_RX_t;


typedef void (*BUP_CB_t)(BUP_Evt_t* pev);

int BUP_disconnect_handler(void);
int BUP_connect_handler(void);
int BUP_data_BLE_to_uart_completed(void);
int BUP_data_BLE_to_uart_send(void);
int BUP_data_BLE_to_uart(uint8_t* pdata, uint8_t size);
int BUP_data_uart_to_BLE_send(void);
int BUP_data_uart_to_BLE(void);
int BUP_init(BUP_CB_t cb);

extern uint32 UART_Baudrate;
extern uint8 AT_BLEUART_EVT;
extern uint8 Bleuart_C_D;
extern AT_BLEUART_RX_t at_bleuart_rx;

extern uint8 AT_query(AT_BLEUART_RX_t* pev);
extern uint8 AT_setdata(AT_BLEUART_RX_t* pev);

//void AT_Response(UART_INDEX_e uart_index,AT_BLEUART_RX_t* pev,uint8_t *buff,uint8 len);

#endif /*_BLE_UART_PROTOCOL_H*/

