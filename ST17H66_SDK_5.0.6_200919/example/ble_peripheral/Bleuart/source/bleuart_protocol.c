
#include <string.h>
#include "types.h"
#include "att.h"
#include "peripheral.h"
#include "uart.h"
#include "OSAL.h"
#include "bcomdef.h"
#include "pwrmgr.h"
#include "bleuart.h"
#include "error.h"
#include "log.h"
#include "bleuart_protocol.h"
#include "bleuart.h"
#include "bleuart_service.h"

#define UART_RX_BUF_SIZE  1024 //512
#define UART_TX_BUF_SIZE  512


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

BUP_ctx_t mBUP_Ctx;

int BUP_disconnect_handler(void);


static void uartrx_timeout_timer_start(void)
{
  LOG("RX TO S\n");
  osal_start_timerEx(bleuart_TaskID, BUP_OSAL_EVT_UART_TO_TIMER, 2);

}

static void uartrx_timeout_timer_stop(void)
{
  osal_stop_timerEx(bleuart_TaskID, BUP_OSAL_EVT_UART_TO_TIMER);

}


//timer for ble send delay slot
static void tx_start_timer(uint16_t timeout)
{
  LOG("BLE start Timer\n");
  osal_start_timerEx(bleuart_TaskID, BUP_OSAL_EVT_BLE_TIMER, timeout);
}

//static void tx_stop_timer(void)
//{
//  osal_stop_timerEx(bleuart_TaskID, BUP_OSAL_EVT_BLE_TIMER);

//}

//timer for uart send delay slot
static void rx_start_timer(uint16_t timeout)
{
  LOG("uart start Timer\n");
  osal_start_timerEx(bleuart_TaskID, BUP_OSAL_EVT_UARTRX_TIMER, timeout);
}

//static void rx_stop_timer(void)
//{
//  osal_stop_timerEx(bleuart_TaskID, BUP_OSAL_EVT_UARTRX_TIMER);

//}
//int s_cnt = 0;

void uart_evt_hdl(uart_Evt_t* pev)
{
  BUP_ctx_t* pctx = & mBUP_Ctx;
  uint16_t conn_hdl;
  GAPRole_GetParameter( GAPROLE_CONNHANDLE, &conn_hdl);
  if(conn_hdl == INVALID_CONNHANDLE){
    BUP_disconnect_handler();
    return;
  }
  switch(pev->type){
    case  UART_EVT_TYPE_RX_DATA:
			if((pctx->hal_uart_rx_size + pev->len)>=UART_RX_BUF_SIZE)
				break;
			uartrx_timeout_timer_stop();
      uartrx_timeout_timer_start();
      memcpy(pctx->hal_uart_rx_buf + pctx->hal_uart_rx_size, pev->data, pev->len);
      pctx->hal_uart_rx_size += pev->len;
      break;
    case  UART_EVT_TYPE_RX_DATA_TO:
			if((pctx->hal_uart_rx_size + pev->len)>=UART_RX_BUF_SIZE)
				break;
			uartrx_timeout_timer_stop();
      uartrx_timeout_timer_start();
      memcpy(pctx->hal_uart_rx_buf + pctx->hal_uart_rx_size, pev->data, pev->len);
      pctx->hal_uart_rx_size += pev->len;
    
      //BUP_data_uart_to_BLE();
      
      //pctx->hal_uart_rx_size = 0;
      //LOG("uart_evt_hdl: %d\n", pev->type);
      break;
  case  UART_EVT_TYPE_TX_COMPLETED:
    osal_set_event(bleuart_TaskID, BUP_OSAL_EVT_UART_TX_COMPLETE);
    break;
  default:
    break;


  }
}




void gpio_wakeup_handle(GPIO_Pin_e pin,IO_Wakeup_Pol_e type)
{
  uint16_t conn_hdl;
  BUP_ctx_t* pctx = &mBUP_Ctx;
  LOG("GPIO Lock\n");
//  GAPRole_GetParameter( GAPROLE_CONNHANDLE, &conn_hdl);
//  if(conn_hdl == INVALID_CONNHANDLE || pctx->conn_state == FALSE){
//    hal_pwrmgr_unlock(MOD_USR1);
//    return;
//  }
  if(hal_pwrmgr_is_lock(MOD_USR1) == FALSE){
    hal_pwrmgr_lock(MOD_USR1);
  }
}

void gpio_sleep_handle(GPIO_Pin_e pin,IO_Wakeup_Pol_e type)
{
  uint16_t conn_hdl;
  BUP_ctx_t* pctx = &mBUP_Ctx;
  LOG("GPIO unlock\n");
//  GAPRole_GetParameter( GAPROLE_CONNHANDLE, &conn_hdl);
//  if(conn_hdl == INVALID_CONNHANDLE || pctx->conn_state == FALSE){
//    hal_pwrmgr_unlock(MOD_USR1);
//    return;
//  }
  if(hal_pwrmgr_is_lock(MOD_USR1) == TRUE){
    hal_pwrmgr_unlock(MOD_USR1);
  }
}

int BUP_data_BLE_to_uart_completed(void)
{
  BUP_ctx_t* pctx = &mBUP_Ctx;
  if(pctx->tx_size){
    BUP_data_BLE_to_uart_send();
    return PPlus_SUCCESS;
  }
  //case no data in buffer
 // FLOW_CTRL_UART_TX_UNLOCK();
  pctx->tx_state = BUP_TX_ST_IDLE;
  return PPlus_SUCCESS;
}

//uint16_t s_buf[32];

int BUP_data_BLE_to_uart_send(void)
{
  BUP_ctx_t* pctx = &mBUP_Ctx;

  if(pctx->tx_state != BUP_TX_ST_IDLE && pctx->tx_size)
  {
    //s_buf[s_cnt%32] = pctx->tx_size;
    //s_cnt++;
    hal_uart_send_buff(UART1,pctx->tx_buf, pctx->tx_size);
    pctx->tx_size = 0;
    pctx->tx_state = BUP_TX_ST_SENDING;
    return PPlus_SUCCESS;
  }

  LOG("BUP_data_BLE_to_uart_send: incorrect state\n");
  return PPlus_ERR_INVALID_STATE;
}

int BUP_data_BLE_to_uart(uint8_t* pdata, uint8_t size)
{
  BUP_ctx_t* pctx = &mBUP_Ctx;
  switch(pctx->tx_state){
  case BUP_TX_ST_IDLE:
    memcpy(pctx->tx_buf + pctx->tx_size, pdata, size);
    pctx->tx_size += size;
    FLOW_CTRL_UART_TX_LOCK();
    pctx->tx_state = BUP_TX_ST_DELAY_SLOT;
    tx_start_timer(1);  //1ms delay slot
    break;
  case BUP_TX_ST_DELAY_SLOT:
  case BUP_TX_ST_SENDING:
  {
    memcpy(pctx->tx_buf + pctx->tx_size, pdata, size);
    pctx->tx_size += size;
    break;
  }
  default:
    mBUP_Ctx.tx_state = BUP_TX_ST_IDLE;
    break;
  }
  return PPlus_SUCCESS;
}


int BUP_data_uart_to_BLE_send(void)
{
  BUP_ctx_t* pctx = &mBUP_Ctx;
  bool start_flg = FALSE;
  uint16 mtu_size=0;
	
	 bStatus_t ret = 0;
	
	mtu_size=gAttMtuSize[gapConnHandle];
  LOG("mtu=%d\n",gAttMtuSize[gapConnHandle]);
//	if(mtu_size>200)
//	{
//		mtu_size=200;
//	}	
//	else
	{
		mtu_size -=3;
	}	
	
//  hal_pwrmgr_unlock(MOD_USR1);
  if(pctx->rx_state != BUP_RX_ST_IDLE && pctx->rx_size)
  {
    //LOG1("cnt:%d",s_cnt);
    if(bleuart_NotifyIsReady() == FALSE)
      return PPlus_ERR_BLE_NOT_READY;

    if(pctx->rx_state == BUP_RX_ST_DELAY_SLOT){
      start_flg = TRUE;
      FLOW_CTRL_BLE_TX_LOCK();
      pctx->rx_state = BUP_RX_ST_SENDING;
    }
    
    while(1){
      uint8_t size =0;
     
      attHandleValueNoti_t notify_data={0};
      size = ((pctx->rx_size - pctx->rx_offset) > mtu_size) ? mtu_size : pctx->rx_size - pctx->rx_offset;

      memcpy(notify_data.value,pctx->rx_buf + pctx->rx_offset, size);
      notify_data.len = size;
      
      ret = bleuart_Notify(gapConnHandle, &notify_data, bleuart_TaskID);
			LOG("bleuart_Notify: %d, %d, %d\n", ret,pctx->rx_offset, pctx->rx_size);
      if(ret == SUCCESS){
        pctx->rx_offset += size;
      }
      else
      {
				LOG("TX R=%x\n",ret);
        if(ret == MSG_BUFFER_NOT_AVAIL){
          if(start_flg){
            rx_start_timer(1);
          }
          else
          {
            rx_start_timer(bleuart_conn_interval()-1);
          }
          return PPlus_SUCCESS;
        }
        else
        {
          return PPlus_ERR_BUSY;
        }
      }
      
      if(pctx->rx_offset == pctx->rx_size){
        LOG("Success\n");
        pctx->rx_state = BUP_RX_ST_IDLE;
        pctx->rx_offset = 0;
        pctx->rx_size = 0;
        FLOW_CTRL_BLE_TX_UNLOCK();
        return PPlus_SUCCESS;
      }

    }
  
  }
	else
	{
		LOG("U2B s=:%x,%x",pctx->rx_state,pctx->rx_size);
		  pctx->rx_state = BUP_RX_ST_IDLE;
        pctx->rx_offset = 0;
        pctx->rx_size = 0;
		 FLOW_CTRL_BLE_TX_UNLOCK();
	}	

  LOG("U2B ret : %x\n",ret);
  return PPlus_ERR_INVALID_STATE;
}

int BUP_data_uart_to_BLE(void)
{
  BUP_ctx_t* pctx = &mBUP_Ctx;
//  LOG("BUP_data_uart_to_BLE\n");
  if(pctx->conn_state == FALSE){
		LOG("no cnt\n");
    pctx->rx_size = 0;
    pctx->rx_offset = 0;
    pctx->hal_uart_rx_size = 0;
    return PPlus_ERR_INVALID_STATE;
  }
  memcpy(pctx->rx_buf + pctx->rx_size, pctx->hal_uart_rx_buf, pctx->hal_uart_rx_size);
  if(pctx->rx_offset != 0){
    return PPlus_ERR_BUSY;

  }
	//s_cnt++;
  //if(pctx->hal_uart_rx_size != 80){
  //  s_cnt++;
  //}
  pctx->rx_size += pctx->hal_uart_rx_size;
  
	pctx->hal_uart_rx_size = 0;
  switch(pctx->rx_state){
  case BUP_RX_ST_IDLE:
    pctx->rx_state = BUP_RX_ST_DELAY_SLOT;
    rx_start_timer(1);  //1ms delay slot
    break;
  case BUP_RX_ST_DELAY_SLOT:
  case BUP_RX_ST_SENDING:
  default:
    //drop data
	LOG("U2B err:%d\n",pctx->rx_state);
    return PPlus_ERR_INVALID_STATE;
  }
  return PPlus_SUCCESS;

}

int BUP_disconnect_handler(void)
{
  memset(&mBUP_Ctx, 0, sizeof(mBUP_Ctx));
//  hal_gpio_write(FLOW_CTRL_IO_UART_TX, 0);
//  hal_gpio_write(FLOW_CTRL_IO_BLE_TX, 0);
//  hal_gpio_write(FLOW_CTRL_IO_BLE_CONNECTION, 0);
 // hal_pwrmgr_unlock(MOD_USR1);
	return PPlus_SUCCESS;
}

int BUP_connect_handler(void)
{
  if(mBUP_Ctx.conn_state == FALSE){
    memset(&mBUP_Ctx, 0, sizeof(mBUP_Ctx));
    mBUP_Ctx.conn_state = TRUE;
//    hal_gpio_write(FLOW_CTRL_IO_UART_TX, 0);
//    hal_gpio_write(FLOW_CTRL_IO_BLE_TX, 0);
//    hal_gpio_write(FLOW_CTRL_IO_BLE_CONNECTION, 0);
//    hal_pwrmgr_unlock(MOD_USR1);
  }
	return PPlus_SUCCESS;
}

int BUP_init(BUP_CB_t cb)
{
  BUP_ctx_t* pctx = &mBUP_Ctx;
  uart_Cfg_t cfg = {
  .tx_pin = P2,
  .rx_pin = P3,
  .rts_pin = GPIO_DUMMY,
  .cts_pin = GPIO_DUMMY,
  .baudrate = 115200,
  .use_fifo = TRUE,
  .hw_fwctrl = FALSE,
  .use_tx_buf = TRUE,
  .parity     = FALSE,
  .evt_handler = uart_evt_hdl,
  };
  hal_uart_init(cfg,UART1);
  
  hal_uart_set_tx_buf(UART1,pctx->hal_uart_tx_buf, UART_TX_BUF_SIZE);

#if(CFG_SLEEP_MODE == PWR_MODE_SLEEP)
	hal_gpio_pin_init(FLOW_CTRL_IO_HOST_WAKEUP,IE);						/*UART»½ÐÑ¿ØÖÆ*/
	hal_gpio_pull_set(FLOW_CTRL_IO_HOST_WAKEUP,STRONG_PULL_UP);
    hal_gpioin_register(FLOW_CTRL_IO_HOST_WAKEUP, gpio_sleep_handle, gpio_wakeup_handle);
#endif  
//	hal_gpio_pin_init(UART_INDICATE_LED,OEN);
//	hal_gpio_write(UART_INDICATE_LED,0);
//	hal_gpio_retention_enable(UART_INDICATE_LED,1);
  //config gpio wakeup
	hal_pwrmgr_register(MOD_USR1, NULL, NULL);
  


//  hal_gpio_write(FLOW_CTRL_IO_UART_TX, 0);
//  hal_gpio_write(FLOW_CTRL_IO_BLE_TX, 0);
//  hal_gpio_write(FLOW_CTRL_IO_BLE_CONNECTION, 0);
  
  memset(&mBUP_Ctx, 0, sizeof(mBUP_Ctx));
  LOG("BUP_init\n");
    
  return PPlus_SUCCESS;
}


