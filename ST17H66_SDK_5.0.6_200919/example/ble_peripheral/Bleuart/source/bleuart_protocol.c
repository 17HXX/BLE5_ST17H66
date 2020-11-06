
#include <string.h>
#include "types.h"
#include "att.h"
#include "peripheral.h"
#include "OSAL.h"
#include "bcomdef.h"
#include "pwrmgr.h"
#include "bleuart.h"
#include "error.h"
#include "log.h"
#include "bleuart_protocol.h"
//#include "bleuart.h"
#include "bleuart_service.h"

#define UART_RX_BUF_SIZE  1024 //512
#define UART_TX_BUF_SIZE  512

uint8 AT_BLEUART_EVT=0;

uint32 UART_Baudrate = 115200;

//bleuart传输指令——>0	传输数据——>1
uint8 Bleuart_C_D=0;

//固件版本FW Version  v1.1.1
uint8 AT_FW_version[]={0x76,0x31,0x2E,0x31,0x2E,0x31};



AT_BLEUART_RX_t at_bleuart_rx;

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
  osal_start_timerEx(bleuart_TaskID, BUP_OSAL_EVT_UART_TO_TIMER, 10);

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
  AT_BLEUART_RX_t* at_rx = & at_bleuart_rx;
//  uint16_t conn_hdl;
//  GAPRole_GetParameter( GAPROLE_CONNHANDLE, &conn_hdl);
//  if(conn_hdl == INVALID_CONNHANDLE){
//    BUP_disconnect_handler();
//    return;
//  }
  switch(pev->type){
    case  UART_EVT_TYPE_RX_DATA:
			if((pctx->hal_uart_rx_size + pev->len)>=UART_RX_BUF_SIZE)
				break;

/***************************************************************************************************************************/
//串口接收AT指令
	if(Bleuart_C_D==0)
	{	
		osal_stop_timerEx(bleuart_TaskID, BUP_OSAL_EVT_AT);
		osal_start_timerEx(bleuart_TaskID, BUP_OSAL_EVT_AT, 10);
		
		memcpy(at_rx->data + at_rx->len, pev->data, pev->len);
		at_rx->len += pev->len;
//		LOG("LEN:%d\n",pev->len);
//		LOG("RXLEN:%d\n",at_rx->len);
		return ;
	}		
/***************************************************************************************************************************/
	else
	{
			
	  uartrx_timeout_timer_stop();
      uartrx_timeout_timer_start();
      memcpy(pctx->hal_uart_rx_buf + pctx->hal_uart_rx_size, pev->data, pev->len);
      pctx->hal_uart_rx_size += pev->len;

//			LOG("LEN1:%d\n",pev->len);
//			LOG("RXLEN1:%d\n",pctx->hal_uart_rx_size);
	}
//			uartrx_timeout_timer_stop();
//      uartrx_timeout_timer_start();
//      memcpy(pctx->hal_uart_rx_buf + pctx->hal_uart_rx_size, pev->data, pev->len);
//      pctx->hal_uart_rx_size += pev->len;
      break;
    case  UART_EVT_TYPE_RX_DATA_TO:
			if((pctx->hal_uart_rx_size + pev->len)>=UART_RX_BUF_SIZE)
				break;
			
/***************************************************************************************************************************/
//串口接收AT指令
	if(Bleuart_C_D==0)
	{	
		osal_stop_timerEx(bleuart_TaskID, BUP_OSAL_EVT_AT);
		osal_start_timerEx(bleuart_TaskID, BUP_OSAL_EVT_AT, 10);
		
		memcpy(at_rx->data + at_rx->len, pev->data, pev->len);
		at_rx->len += pev->len;
//		LOG("LEN:%d\n",pev->len);
//		LOG("RXLEN:%d\n",at_rx->len);
		return ;
	}		
/***************************************************************************************************************************/
	else
	{
			
			uartrx_timeout_timer_stop();
      uartrx_timeout_timer_start();
      memcpy(pctx->hal_uart_rx_buf + pctx->hal_uart_rx_size, pev->data, pev->len);
      pctx->hal_uart_rx_size += pev->len;
      //BUP_data_uart_to_BLE();
    } 
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
//  uint16_t conn_hdl;
//  BUP_ctx_t* pctx = &mBUP_Ctx;
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
//  uint16_t conn_hdl;
//  BUP_ctx_t* pctx = &mBUP_Ctx;
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
  LOG("mtu=%d,%d,%d\n",gAttMtuSize[gapConnHandle],pctx->rx_state,pctx->rx_size);
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
  .baudrate = UART_Baudrate,
  .use_fifo = TRUE,
  .hw_fwctrl = FALSE,
  .use_tx_buf = TRUE,
  .parity     = FALSE,
  .evt_handler = uart_evt_hdl,
  };
  hal_uart_init(cfg,UART1);
  
  hal_uart_set_tx_buf(UART1,pctx->hal_uart_tx_buf, UART_TX_BUF_SIZE);

#if(CFG_SLEEP_MODE == PWR_MODE_SLEEP)
	hal_gpio_pin_init(FLOW_CTRL_IO_HOST_WAKEUP,IE);						/*UART唤醒控制*/
	hal_gpio_pull_set(FLOW_CTRL_IO_HOST_WAKEUP,STRONG_PULL_UP);
    hal_gpioin_register(FLOW_CTRL_IO_HOST_WAKEUP, gpio_sleep_handle, gpio_wakeup_handle);
#endif  
	hal_gpio_pin_init(UART_INDICATE_LED,OEN);
	hal_gpio_write(UART_INDICATE_LED,0);
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


void AT_Response(UART_INDEX_e uart_index,AT_BLEUART_RX_t* pev,uint8_t *buff,uint8 len)
{
	uint8 at_response[30]={0};
	uint8 i;
	
	memcpy(at_response, pev->data, pev->len-3);
	at_response[pev->len-3]=0x3a;
	memcpy(&(at_response[pev->len-2]), buff, len);
	i=(pev->len-2)+len;
	at_response[i]=0x0d;
	at_response[i+1]=0x0a;
	at_response[i+2]=0x4f;
	at_response[i+3]=0x4b;
	at_response[i+4]=0x0d;
	at_response[i+5]=0x0a;
	
	hal_uart_send_buff(UART1,at_response, i+6);
}




uint8 AT_query(AT_BLEUART_RX_t* pev)
{
	if(((pev->data[0])==0x41)&&((pev->data[1])==0x54)&&((pev->data[2])==0x2B)&&((pev->data[(pev->len)-2])==0x0D)&&((pev->data[(pev->len)-1])==0x0A))
	{
		//恢复出厂设置52 45 53 45 54
		if(((pev->len)==10)&&((pev->data[3])==0x52)&&((pev->data[4])==0x45)&&((pev->data[5])==0x53)&&((pev->data[6])==0x45)&&((pev->data[7])==0x54))
		{
			uint8 response[pev->len+4];
			
			Modify_BLEDevice_Data = 3;
			osal_snv_write(0x80,1,&Modify_BLEDevice_Data);
			
			memcpy(response, pev->data, pev->len);
			response[pev->len]=0x4f;
			response[pev->len+1]=0x4b;
			response[pev->len+2]=0x0d;
			response[pev->len+3]=0x0a;
			hal_uart_send_buff(UART1,response, pev->len+4);
			NVIC_SystemReset();
			return 0;
		}
		//断开当前连接44 49 53 43 4F 4E 4E
		if(((pev->len)==12)&&((pev->data[3])==0x44)&&((pev->data[4])==0x49)&&((pev->data[5])==0x53)&&((pev->data[6])==0x43)&&((pev->data[7])==0x4F)&&((pev->data[8])==0x4E)&&((pev->data[9])==0x4E))
		{
			uint8 response[pev->len+4];
			GAPRole_TerminateConnection();
			memcpy(response, pev->data, pev->len);
			response[pev->len]=0x4f;
			response[pev->len+1]=0x4b;
			response[pev->len+2]=0x0d;
			response[pev->len+3]=0x0a;
			hal_uart_send_buff(UART1,response, pev->len+4);
			return 0;
		}
		//重启5A
		if(((pev->len)==6)&&((pev->data[3])==0x5A))
		{
			uint8 response[pev->len+4];
			memcpy(response, pev->data, pev->len);
			response[pev->len]=0x4f;
			response[pev->len+1]=0x4b;
			response[pev->len+2]=0x0d;
			response[pev->len+3]=0x0a;
			hal_uart_send_buff(UART1,response, pev->len+4);
			NVIC_SystemReset();
			return 0;
		}
		//设置进入透传模式2B 2B 2B
		if(((pev->len)==8)&&((pev->data[3])==0x2B)&&((pev->data[4])==0x2B)&&((pev->data[5])==0x2B))
		{
			uint8 response[pev->len+4];
			Bleuart_C_D=1;
			memcpy(response, pev->data, pev->len);
			response[pev->len]=0x4f;
			response[pev->len+1]=0x4b;
			response[pev->len+2]=0x0d;
			response[pev->len+3]=0x0a;
			hal_uart_send_buff(UART1,response, pev->len+4);
			return 0;
		}
		//设置FLASH记忆参数46 4C 41 53 48
		if(((pev->len)==10)&&((pev->data[3])==0x46)&&((pev->data[4])==0x4C)&&((pev->data[5])==0x41)&&((pev->data[6])==0x53)&&((pev->data[7])==0x48))
		{
			uint8 response[pev->len+4];
			uint8 at_mac_addr[6];
			
			uint8 uart_baudrate[4]={BREAK_UINT32(UART_Baudrate,3),BREAK_UINT32(UART_Baudrate,2),BREAK_UINT32(UART_Baudrate,1),BREAK_UINT32(UART_Baudrate,0)};
			osal_snv_read(0x89,6,at_mac_addr);
			
			osal_snv_write(0x81,20,&(scanR[2]));				// 20 Bytes Device Name
			osal_snv_write(0x82,6,at_mac_addr);					// 6 Bytes MAC address
			osal_snv_write(0x83,4,uart_baudrate);				// 4 Bytes uart_baudrate
			osal_snv_write(0x84,1,&advint);						// 1 Byte Advert interval
			osal_snv_write(0x85,1,&AT_bleuart_auto);			// 1 Byte BLE_UART AUTO
			osal_snv_write(0x86,1,&AT_bleuart_sleep);			// 1 Byte SLEEP MODE
			osal_snv_write(0x87,1,&AT_bleuart_txpower);			// 1 Byte TX power
			osal_snv_write(0x88,8,&(advertdata[23]));			// 8 Bytes reserved data
			
			memcpy(response, pev->data, pev->len);
			response[pev->len]=0x4f;
			response[pev->len+1]=0x4b;
			response[pev->len+2]=0x0d;
			response[pev->len+3]=0x0a;
			hal_uart_send_buff(UART1,response, pev->len+4);
			return 1;
		}
		
		//查询蓝牙设备名4E 41 4D 45 3F
		if(((pev->len)==10)&&((pev->data[3])==0x4E)&&((pev->data[4])==0x41)&&((pev->data[5])==0x4D)&&((pev->data[6])==0x45)&&((pev->data[7])==0x3F))
		{
			uint8 device_name[20];
			memcpy(device_name, &(scanR[2]), 20);
			AT_Response(UART1,pev,device_name,20);
			return 0;
		}
//		//查询设备模式4D 4F 44 45 3F
//		if(((pev->len)==10)&&((pev->data[3])==0x4D)&&((pev->data[4])==0x4F)&&((pev->data[5])==0x44)&&((pev->data[6])==0x45)&&((pev->data[7])==0x3F))
//		{
//			
//			return 0;
//		}
		//查询MAC地址4D 41 43 3F
		if(((pev->len)==9)&&((pev->data[3])==0x4D)&&((pev->data[4])==0x41)&&((pev->data[5])==0x43)&&((pev->data[6])==0x3F))
		{
			uint8 at_mac_address[6];
			uint8 mac_addr[12];
			uint8 i,j;
			hal_flash_read(0x4004,at_mac_address,2);   //读取mac地址
			hal_flash_read(0x4000,at_mac_address+2,4);
			
			for(i=0,j=0;i<6;i++,j++)
			{
				if((((at_mac_address[i])>>4)<=9)&&(((at_mac_address[i])>>4)>=0))
				{
					mac_addr[j]=((at_mac_address[i])>>4)+0x30;
				}
				else
				{
					mac_addr[j]=((at_mac_address[i])>>4)+0x37;
				}
				
				j=j+1;
				
				if(((at_mac_address[i]&0x0f)<=9)&&((at_mac_address[i]&0x0f)>=0))
				{
					mac_addr[j]=(at_mac_address[i]&0x0f)+0x30;
				}
				else
				{
					mac_addr[j]=(at_mac_address[i]&0x0f)+0x37;
				}
			}
			
			AT_Response(UART1,pev,mac_addr,12);
			return 0;
		}
		//查询软件版本号43 49 56 45 52 3F
		if(((pev->len)==11)&&((pev->data[3])==0x43)&&((pev->data[4])==0x49)&&((pev->data[5])==0x56)&&((pev->data[6])==0x45)&&((pev->data[7])==0x52)&&((pev->data[8])==0x3F))
		{
			AT_Response(UART1,pev,AT_FW_version,sizeof (AT_FW_version));
			return 0;
		}
		//查询串口参数55 41 52 54 3F
		if(((pev->len)==10)&&((pev->data[3])==0x55)&&((pev->data[4])==0x41)&&((pev->data[5])==0x52)&&((pev->data[6])==0x54)&&((pev->data[7])==0x3F))
		{
			uint8 uart_baudrate[4]={BREAK_UINT32(UART_Baudrate,3),BREAK_UINT32(UART_Baudrate,2),BREAK_UINT32(UART_Baudrate,1),BREAK_UINT32(UART_Baudrate,0)};
			AT_Response(UART1,pev,uart_baudrate,4);
			return 0;
		}
		//查询连接上后是否自动进入透传模式41 55 54 4F 2B 2B 2B 3F
		if(((pev->len)==13)&&((pev->data[3])==0x41)&&((pev->data[4])==0x55)&&((pev->data[5])==0x54)&&((pev->data[6])==0x4F)&&((pev->data[7])==0x2B)&&((pev->data[8])==0x2B)&&((pev->data[9])==0x2B)&&((pev->data[10])==0x3F))
		{
			uint8 at_bleuart_auto;
			osal_snv_read(0x85,1,&at_bleuart_auto);
			AT_Response(UART1,pev,&at_bleuart_auto,1);
			return 0;
		}
		//查询连接状态4C 49 4E 4B 3F
		if(((pev->len)==10)&&((pev->data[3])==0x4C)&&((pev->data[4])==0x49)&&((pev->data[5])==0x4E)&&((pev->data[6])==0x4B)&&((pev->data[7])==0x3F))
		{
			uint16_t conn_hdl;
			uint8 conn_online[6]={0x4f,0x6e,0x4c,0x69,0x6e,0x65};
			uint8 conn_offline[7]={0x4f,0x66,0x66,0x4c,0x69,0x6e,0x65};
			GAPRole_GetParameter( GAPROLE_CONNHANDLE, &conn_hdl);
			if(conn_hdl == INVALID_CONNHANDLE)
			{
				AT_Response(UART1,pev,conn_offline,7);
			}
			else
			{
				AT_Response(UART1,pev,conn_online, 6);
			}
			return 0;
		}
		//查询广播间隔41 44 56 49 4E 54 3F
		if(((pev->len)==12)&&((pev->data[3])==0x41)&&((pev->data[4])==0x44)&&((pev->data[5])==0x56)&&((pev->data[6])==0x49)&&((pev->data[7])==0x4E)&&((pev->data[8])==0x54)&&((pev->data[9])==0x3F))
		{
			AT_Response(UART1,pev,&advint, 1);
			return 0;
		}
		//查询发射功率50 4F 57 45 52 3F
		if(((pev->len)==11)&&((pev->data[3])==0x50)&&((pev->data[4])==0x4F)&&((pev->data[5])==0x57)&&((pev->data[6])==0x45)&&((pev->data[7])==0x52)&&((pev->data[8])==0x3F))
		{
			uint8 at_power;
			if(g_rfPhyTxPower==RF_PHY_TX_POWER_MAX)
			{
				at_power=0x00;
			}
			else if(g_rfPhyTxPower==RF_PHY_TX_POWER_5DBM)
			{
				at_power=0x01;
			}
			else if(g_rfPhyTxPower==RF_PHY_TX_POWER_4DBM)
			{
				at_power=0x02;
			}
			else if(g_rfPhyTxPower==RF_PHY_TX_POWER_3DBM)
			{
				at_power=0x03;
			}
			else if(g_rfPhyTxPower==RF_PHY_TX_POWER_0DBM)
			{
				at_power=0x04;
			}
			else if(g_rfPhyTxPower==RF_PHY_TX_POWER_N2DBM)
			{
				at_power=0x05;
			}
			else if(g_rfPhyTxPower==RF_PHY_TX_POWER_N5DBM)
			{
				at_power=0x06;
			}
			else if(g_rfPhyTxPower==RF_PHY_TX_POWER_N10DBM)
			{
				at_power=0x07;
			}
			AT_Response(UART1,pev,&at_power, 1);
			return 0;
		}
		//查询UUID 55 55 49 44 3F
		if(((pev->len)==10)&&((pev->data[3])==0x55)&&((pev->data[4])==0x55)&&((pev->data[5])==0x49)&&((pev->data[6])==0x44)&&((pev->data[7])==0x3F))
		{
			uint8 service_uuid[41]={0x41,0x54,0x2B,0x62,0x6c,0x65,0x55,0x61,0x72,0x74,0x5f,0x53,0x65,0x72,0x76,0x65,0x72,0x5f,0x55,0x75,0x69,0x64,0x3a};
			uint8 service_tx_uuid[44]={0x41,0x54,0x2B,0x62,0x6c,0x65,0x55,0x61,0x72,0x74,0x5f,0x53,0x65,0x72,0x76,0x65,0x72,0x5f,0x54,0x78,0x5f,0x55,0x75,0x69,0x64,0x3a};
			uint8 service_rx_uuid[44]={0x41,0x54,0x2B,0x62,0x6c,0x65,0x55,0x61,0x72,0x74,0x5f,0x53,0x65,0x72,0x76,0x65,0x72,0x5f,0x52,0x78,0x5f,0x55,0x75,0x69,0x64,0x3a};
				
			memcpy(&service_uuid[23], bleuart_ServiceUUID, 16);
			service_uuid[39]=0x0d;
			service_uuid[40]=0x0a;
			memcpy(&service_tx_uuid[26], bleuart_TxCharUUID, 16);
			service_tx_uuid[42]=0x0d;
			service_tx_uuid[43]=0x0a;
			memcpy(&service_rx_uuid[26], bleuart_RxCharUUID, 16);
			service_rx_uuid[42]=0x0d;
			service_rx_uuid[43]=0x0a;

			hal_uart_send_buff(UART1,service_uuid, 41);
			WaitMs(50);
			hal_uart_send_buff(UART1,service_tx_uuid, 44);
			WaitMs(50);
			hal_uart_send_buff(UART1,service_rx_uuid, 44);
			return 0;
		}
		//查询自定义广播数据52 45 53 45 3F
		if(((pev->len)==10)&&((pev->data[3])==0x52)&&((pev->data[4])==0x45)&&((pev->data[5])==0x53)&&((pev->data[6])==0x45)&&((pev->data[7])==0x3F))
		{
			uint8 reserved_data[8];
			memcpy(reserved_data, &(advertdata[23]), 8);
			AT_Response(UART1,pev,reserved_data,8);
			return 0;
		}
	}
	return 2;
}




//void AT_Response(UART_INDEX_e uart_index,AT_BLEUART_RX_t* pev,uint8_t *buff,uint8 len)
//{
//	uint8 at_response[30]={0};
//	uint8 i;
//	
//	memcpy(at_response, pev->data, pev->len-3);
//	at_response[pev->len-3]=0x3a;
//	memcpy(&(at_response[pev->len-2]), buff, len);
//	i=(pev->len-2)+len;
//	at_response[i]=0x0d;
//	at_response[i+1]=0x0a;
//	at_response[i+2]=0x4f;
//	at_response[i+3]=0x4b;
//	at_response[i+4]=0x0d;
//	at_response[i+5]=0x0a;
//	
//	hal_uart_send_buff(UART1,at_response, i+6);
//}



void AT_Response2(UART_INDEX_e uart_index,AT_BLEUART_RX_t* pev,uint8_t *buff,uint8 len1,uint8 len2)
{
	uint8 at_response[35]={0};
	uint8 i;
	
	memcpy(at_response, pev->data, len2);
	at_response[len2]=0x3a;
	memcpy(&at_response[len2+1], buff, len1);
	i=(len2+1)+len1;
	at_response[i]=0x0d;
	at_response[i+1]=0x0a;
	at_response[i+2]=0x4f;
	at_response[i+3]=0x4b;
	at_response[i+4]=0x0d;
	at_response[i+5]=0x0a;
	
	hal_uart_send_buff(UART1,at_response, i+6);
}




uint8 AT_setdata(AT_BLEUART_RX_t* pev)
{
	if(((pev->data[0])==0x41)&&((pev->data[1])==0x54)&&((pev->data[2])==0x2B)&&((pev->data[(pev->len)-2])==0x0D)&&((pev->data[(pev->len)-1])==0x0A))
	{
		//设置蓝牙设备名4E 41 4D 45 3D
		if(((pev->len)<=30)&&((pev->data[3])==0x4E)&&((pev->data[4])==0x41)&&((pev->data[5])==0x4D)&&((pev->data[6])==0x45)&&((pev->data[7])==0x3D))
		{
			uint8 i;
			for(i=0;i<(pev->len-10);i++)
			{
				scanR[i+2]=pev->data[i+8] ;
			}
			for(;i<20;i++)
			{
				scanR[i+2]=0x20;
			}
			AT_Response2(UART1,pev,&scanR[2], 20, 7);
			return 3;
		}
//		//设置设备模式4D 4F 44 45 3D
//		if(((pev->data[3])==0x4D)&&((pev->data[4])==0x4F)&&((pev->data[5])==0x44)&&((pev->data[6])==0x45)&&((pev->data[7])==0x3D))
//		{
//			return 0;
//		}
		//设置MAC地址4D 41 43 3D
		if(((pev->len)==15)&&((pev->data[3])==0x4D)&&((pev->data[4])==0x41)&&((pev->data[5])==0x43)&&((pev->data[6])==0x3D))
		{
			uint8 at_mac_address[6];
			memcpy(at_mac_address, &pev->data[7], 6);
			osal_snv_write(0x89,6,at_mac_address);
			AT_Response2(UART1,pev,at_mac_address, 6, 6);
			return 0;
		}
		//设置串口参数55 41 52 54 3D
		if(((pev->len)==14)&&((pev->data[3])==0x55)&&((pev->data[4])==0x41)&&((pev->data[5])==0x52)&&((pev->data[6])==0x54)&&((pev->data[7])==0x3D))
		{
			uint8 uart_baudrate[4];
			memcpy(uart_baudrate, &pev->data[8], 4);
			UART_Baudrate=BUILD_UINT32(uart_baudrate[3],uart_baudrate[2],uart_baudrate[1],uart_baudrate[0]);
			AT_Response2(UART1,pev,uart_baudrate, 4, 7);
			WaitMs(70);
			hal_uart_deinit(UART1);
			BUP_init(on_BUP_Evt);
			return 0;
		}
//		//设置开启/停止睡眠模式53 4C 45 45 50 3D
//		if(((pev->len)==12)&&((pev->data[3])==0x53)&&((pev->data[4])==0x4C)&&((pev->data[5])==0x45)&&((pev->data[6])==0x45)&&((pev->data[7])==0x50)&&((pev->data[8])==0x3D))
//		{
//			return 0;
//		}
		//设置发射功率50 4F 57 45 52 3D
		if(((pev->len)==12)&&((pev->data[3])==0x50)&&((pev->data[4])==0x4F)&&((pev->data[5])==0x57)&&((pev->data[6])==0x45)&&((pev->data[7])==0x52)&&((pev->data[8])==0x3D))
		{
			AT_bleuart_txpower=pev->data[9];
			AT_Response2(UART1,pev,&AT_bleuart_txpower, 1, 8);
			return 3;
		}
		//设置广播间隔41 44 56 49 4E 54 3D
		if(((pev->len)==13)&&((pev->data[3])==0x41)&&((pev->data[4])==0x44)&&((pev->data[5])==0x56)&&((pev->data[6])==0x49)&&((pev->data[7])==0x4E)&&((pev->data[8])==0x54)&&((pev->data[9])==0x3D))
		{
			advint=pev->data[10];
			AT_Response2(UART1,pev,&advint, 1, 9);
			return 3;
		}
		//设置连接上后自动进入透传模式41 55 54 4F 2B 2B 2B 3D
		if(((pev->len)==14)&&((pev->data[3])==0x41)&&((pev->data[4])==0x55)&&((pev->data[5])==0x54)&&((pev->data[6])==0x4F)&&((pev->data[7])==0x2B)&&((pev->data[8])==0x2B)&&((pev->data[9])==0x2B)&&((pev->data[10])==0x3D))
		{
			AT_bleuart_auto=pev->data[11];
			AT_Response2(UART1,pev,&AT_bleuart_auto, 1, 10);
			return 0;
		}
		//设置自定义广播数据52 45 53 45 3D
		if(((pev->len)==18)&&((pev->data[3])==0x52)&&((pev->data[4])==0x45)&&((pev->data[5])==0x53)&&((pev->data[6])==0x45)&&((pev->data[7])==0x3D))
		{
			memcpy(&(advertdata[23]), &(pev->data[8]), 8);
			AT_Response2(UART1,pev,&(advertdata[23]), 8, 7);
			return 3;
		}
	}
	return 4;
}












