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

#include "simpleBLEUart.h"
#include "ui.h"


uint8 AT_BLEUART_EVT=0;

uint32 UART_Baudrate = 115200;

//bleuart传输指令——>0	传输数据——>1
uint8 Bleuart_C_D=0;

//固件版本FW Version  v1.1.1
uint8 AT_FW_version[]={0x76,0x31,0x2E,0x31,0x2E,0x31};

BUP_ctx_t mBUP_Ctx;

AT_BLEUART_RX_t at_bleuart_rx;

void uart_send_buf(uint8 *data, uint32 size)
{
		hal_uart_send_buff(UART1, (uint8_t*)data, size);
}

void uart_evt_hdl(uart_Evt_t* pev)
{
//  BUP_ctx_t* pctx = & mBUP_Ctx;
//  AT_BLEUART_RX_t* at_rx = & at_bleuart_rx;
////  uint16_t conn_hdl;
////  GAPRole_GetParameter( GAPROLE_CONNHANDLE, &conn_hdl);
////  if(conn_hdl == INVALID_CONNHANDLE){
////    BUP_disconnect_handler();
////    return;
////  }
//  switch(pev->type){
//    case  UART_EVT_TYPE_RX_DATA:
//			if((pctx->hal_uart_rx_size + pev->len)>=UART_RX_BUF_SIZE)
//				break;

///***************************************************************************************************************************/
////串口接收AT指令
//	if(Bleuart_C_D==0)
//	{	
//		osal_stop_timerEx(bleuart_TaskID, BUP_OSAL_EVT_AT);
//		osal_start_timerEx(bleuart_TaskID, BUP_OSAL_EVT_AT, 10);
//		
//		memcpy(at_rx->data + at_rx->len, pev->data, pev->len);
//		at_rx->len += pev->len;
////		LOG("LEN:%d\n",pev->len);
////		LOG("RXLEN:%d\n",at_rx->len);
//		return ;
//	}		
///***************************************************************************************************************************/
//	else
//	{
//			
//	  uartrx_timeout_timer_stop();
//      uartrx_timeout_timer_start();
//      memcpy(pctx->hal_uart_rx_buf + pctx->hal_uart_rx_size, pev->data, pev->len);
//      pctx->hal_uart_rx_size += pev->len;

////			LOG("LEN1:%d\n",pev->len);
////			LOG("RXLEN1:%d\n",pctx->hal_uart_rx_size);
//	}
////			uartrx_timeout_timer_stop();
////      uartrx_timeout_timer_start();
////      memcpy(pctx->hal_uart_rx_buf + pctx->hal_uart_rx_size, pev->data, pev->len);
////      pctx->hal_uart_rx_size += pev->len;
//      break;
//    case  UART_EVT_TYPE_RX_DATA_TO:
//			if((pctx->hal_uart_rx_size + pev->len)>=UART_RX_BUF_SIZE)
//				break;
//			
///***************************************************************************************************************************/
////串口接收AT指令
//	if(Bleuart_C_D==0)
//	{	
//		osal_stop_timerEx(bleuart_TaskID, BUP_OSAL_EVT_AT);
//		osal_start_timerEx(bleuart_TaskID, BUP_OSAL_EVT_AT, 10);
//		
//		memcpy(at_rx->data + at_rx->len, pev->data, pev->len);
//		at_rx->len += pev->len;
////		LOG("LEN:%d\n",pev->len);
////		LOG("RXLEN:%d\n",at_rx->len);
//		return ;
//	}		
///***************************************************************************************************************************/
//	else
//	{
//			
//			uartrx_timeout_timer_stop();
//      uartrx_timeout_timer_start();
//      memcpy(pctx->hal_uart_rx_buf + pctx->hal_uart_rx_size, pev->data, pev->len);
//      pctx->hal_uart_rx_size += pev->len;
//      //BUP_data_uart_to_BLE();
//    } 
//      //pctx->hal_uart_rx_size = 0;
//      //LOG("uart_evt_hdl: %d\n", pev->type);
//      break;
//  case  UART_EVT_TYPE_TX_COMPLETED:
//    osal_set_event(bleuart_TaskID, BUP_OSAL_EVT_UART_TX_COMPLETE);
//    break;
//  default:
//    break;
//  }
}
void on_BUP_Evt(BUP_Evt_t* pev)
{
  switch(pev->ev){
    
  }
}
int BUP_init(BUP_CB_t cb)
{
	
	hal_gpio_pin_init(UI_IO_UART1_TX,OEN) ;         
	hal_gpioretention_register(UI_IO_UART1_TX);	
	
  BUP_ctx_t* pctx = &mBUP_Ctx;
  uart_Cfg_t cfg = {
  .tx_pin = UI_IO_UART1_TX,
  .rx_pin = UI_IO_UART1_RX,
  .rts_pin = GPIO_DUMMY,
  .cts_pin = GPIO_DUMMY,
  .baudrate = UART_Baudrate,
  .use_fifo = TRUE,
  .hw_fwctrl = FALSE,
  .use_tx_buf = TRUE,
  .parity     = FALSE,
  .evt_handler = uart_evt_hdl,
  };
  hal_uart_init(cfg,UART1);
  
  hal_uart_set_tx_buf(UART1,pctx->hal_uart_tx_buf, UART_TX_BUF_SIZE);

//#if(CFG_SLEEP_MODE == PWR_MODE_SLEEP)
//	hal_gpio_pin_init(FLOW_CTRL_IO_HOST_WAKEUP,IE);						/*UART唤醒控制*/
//	hal_gpio_pull_set(FLOW_CTRL_IO_HOST_WAKEUP,STRONG_PULL_UP);
//    hal_gpioin_register(FLOW_CTRL_IO_HOST_WAKEUP, gpio_sleep_handle, gpio_wakeup_handle);
//#endif  
//	hal_gpio_pin_init(UART_INDICATE_LED,OEN);
//	hal_gpio_write(UART_INDICATE_LED,0);
//	hal_gpio_retention_enable(UART_INDICATE_LED,1);
  //config gpio wakeup
//	hal_pwrmgr_register(MOD_USR1, NULL, NULL);

//  hal_gpio_write(FLOW_CTRL_IO_UART_TX, 0);
//  hal_gpio_write(FLOW_CTRL_IO_BLE_TX, 0);
//  hal_gpio_write(FLOW_CTRL_IO_BLE_CONNECTION, 0);
  
//  memset(&mBUP_Ctx, 0, sizeof(mBUP_Ctx));
  LOG("BUP_init\n");   
//	uint8 power_on[]={'p','o','w','e','r',' ','o','n',' '};
//	uart_send_buf(power_on,9);

  return PPlus_SUCCESS;
}

