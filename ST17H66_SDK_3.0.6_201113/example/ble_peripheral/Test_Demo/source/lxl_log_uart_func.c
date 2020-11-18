#include "lxl_gpio_all_init.h"

void lxl_uart_printf_hex (const uint8 *data, uint16 len)
{
    uint16 i;

    for (i = 0; i < len - 1; i++)
    {
        LOG("%x,",data[i]);
        LOG(" ");
    }
    LOG("%x\n",data[i]);
}

#if 0
#define BAUD        9600
#define DELAY_UNIT (1000000/BAUD)//()us/bit
#define PRINT_IO    P1

extern void WaitUs(uint32_t wtTime);
#define Delay_us WaitUs
void sendByte(uint8_t val)
{
	BM_CLR(reg_gpio_swporta_dr, (1<<PRINT_IO)); //startbit
	Delay_us(DELAY_UNIT);
	
    for (int i = 0; i < 8; i++)
    {
		if (val & (1<<i))
		{
			BM_SET(reg_gpio_swporta_dr, (1<<PRINT_IO));  
		} else {
			BM_CLR(reg_gpio_swporta_dr, (1<<PRINT_IO));  
		}
		Delay_us(DELAY_UNIT);
    }
	BM_SET(reg_gpio_swporta_dr, (1<<PRINT_IO));  //stopbit  
	Delay_us(DELAY_UNIT);
}

void sendBytes(uint8_t* buf,uint8_t len)
{
	uint8_t i = 0,val;
	
	while(i < len)
	{
		BM_CLR(reg_gpio_swporta_dr, (1<<PRINT_IO)); //startbit
		Delay_us(DELAY_UNIT);
		val = *(buf+i);
	    for (int i = 0; i < 8; i++)
	    {
	        if (val & 0x01){
				BM_SET(reg_gpio_swporta_dr, (1<<PRINT_IO));  
	        }else {
				BM_CLR(reg_gpio_swporta_dr, (1<<PRINT_IO));  
	        }
	        Delay_us(DELAY_UNIT);
	        val >>= 1;
	    }
		BM_SET(reg_gpio_swporta_dr, (1<<PRINT_IO));  //stopbit
		Delay_us(DELAY_UNIT);
		i++;
	}
}

void sendStr(char *str)
{
	char val;
	int i = 0;
	
	while(*(str+i)!='\0')
	{
		val=*(str+i);

		BM_CLR(reg_gpio_swporta_dr, (1<<PRINT_IO)); //startbit
		Delay_us(DELAY_UNIT);

		for (int i = 0; i < 8; i++)
		{
			if (val & 0x01)
			{
				BM_SET(reg_gpio_swporta_dr, (1<<PRINT_IO));  
			} else {
				BM_CLR(reg_gpio_swporta_dr, (1<<PRINT_IO));  
			}
			Delay_us(DELAY_UNIT);
			val >>= 1;
		}
		BM_SET(reg_gpio_swporta_dr, (1<<PRINT_IO));  //stopbit
		Delay_us(DELAY_UNIT);
		i++;
	}	
}

void gpio_as_uart_init(void)
{
	BM_SET(reg_gpio_swporta_dr, (1<<PRINT_IO));
	hal_gpio_pin_init(PRINT_IO, OEN);
	hal_gpio_pull_set(PRINT_IO,STRONG_PULL_UP);//WEAK_PULL_UP);
}

#if 0
int gpio_printf(const char *format, ...)
{
#ifdef OPEN_LOG
	va_list args;
	memset(put_char, 0, PRINTF_MAX_SIZE);
	va_start( args, format );
#ifdef LOG_OUT_BUFFER
    print( 0, format, args );
		sendStr((char *)put_char);
	
    put_char_size =0;
    return 0;
#else
	return print( 0, format, args );
#endif
#else
    return 0;
#endif
}
#endif

#endif
/*********************************************************************
*********************************************************************/


lxl_struct_sensor_all lxl_sensor_all_data = {
	.adv_mode_tick=0,              //当前处于 1:nfc 2:摄像头 3:无

	.push_data_numable=0,          //上传数据的大小
	.push_tick=0,                  //开始上传数据标志位
	.tx_cnt=0,                     //tx发送数据个数
	.tx_buf={0},                   //tx发送数据
	.rx_cnt=0,                     //rx接收数据个数
	.rx_buf={0},                   //rx接收数据
};    //当前sensor的数据

void lxl_uart_sensor_gpio_init(u8 mode)
{
	if(mode == 2){    //当前为摄像头uart通讯
		//hal_gpio_fmux(MY_GPIO_UART_TX_NFC,Bit_DISABLE); 
		//hal_gpio_fmux(MY_GPIO_UART_RX_NFC,Bit_DISABLE);
		uart_Cfg_t cfg1 = {
			.tx_pin = MY_GPIO_UART_TX,
			.rx_pin = MY_GPIO_UART_RX,
			.rts_pin = GPIO_DUMMY,
			.cts_pin = GPIO_DUMMY,
			.baudrate = 9600,
			.use_fifo = TRUE,
			.hw_fwctrl = FALSE,
			.use_tx_buf = FALSE,
			.parity     = FALSE,
			.evt_handler = lxl_uart_console_rx_handler,
		};
		lxl_sensor_all_data.adv_mode_tick = 2 ;
		//hal_uart_init(cfg1);//uart init
		//int hal_uart_init(uart_Cfg_t cfg,UART_INDEX_e uart_index)
		//LOG("uart_11111111111");
	}else if (mode == 1){      //当前为身份证uart通讯
		//hal_gpio_fmux(MY_GPIO_UART_TX,Bit_DISABLE); 
		//hal_gpio_fmux(MY_GPIO_UART_RX,Bit_DISABLE);
		uart_Cfg_t cfg2 = {
			//.tx_pin = MY_GPIO_UART_TX_NFC,
			//.rx_pin = MY_GPIO_UART_RX_NFC,
			.rts_pin = GPIO_DUMMY,
			.cts_pin = GPIO_DUMMY,
			.baudrate = 115200,
			.use_fifo = TRUE,
			.hw_fwctrl = FALSE,
			.use_tx_buf = FALSE,
			.parity     = FALSE,
			.evt_handler = lxl_uart_console_rx_handler,
		};
		lxl_sensor_all_data.adv_mode_tick = 1 ;
		//hal_uart_init(cfg2);//uart init
		//LOG("uart_2222222222");
	}else {
		lxl_sensor_all_data.adv_mode_tick = 0 ; 
	}
}



void lxl_uart_console_rx_handler(uart_Evt_t* pev)
{
	lxl_struct_sensor_all* pctx = &lxl_sensor_all_data;
	switch(pev->type){
		case UART_EVT_TYPE_RX_DATA:      //rx获取数据中
		case UART_EVT_TYPE_RX_DATA_TO:   //rx获取数据超时
			if((pctx->rx_cnt+pev->len)>=UART_RX_BUFFER_SIZE){
				break ;
			}
			if(lxl_sensor_all_data.push_tick == 1)
				break ;
			osal_memcpy(pctx->rx_buf+pctx->rx_cnt,pev->data,pev->len) ;
			pctx->rx_cnt += pev->len ;
	
			//hal_uart_send_buff(pctx->rx_buf,pctx->rx_cnt) ;
			osal_stop_timerEx(lxl_uart_count_TaskID, MY_UART_1_EVENT_TICK);
			osal_start_timerEx(lxl_uart_count_TaskID, MY_UART_1_EVENT_TICK,10);
		break;
		default:
			break;
	}
}


uint16 lxl_uart_all_func(uint8 task_id, uint16 events)
{
	VOID task_id; // OSAL required parameter that isn't used in this function
	if ( events & SYS_EVENT_MSG )
	{
		uint8 *pMsg;
		if ( (pMsg = osal_msg_receive( task_id )) != NULL )
		{
			lxl_key_count_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );
			VOID osal_msg_deallocate( pMsg );
		}
		return (events ^ SYS_EVENT_MSG);
	}
    if(MY_UART_1_EVENT_TICK&events){
		if(lxl_sensor_all_data.adv_mode_tick == 1){       //当前为身份证uart通讯
			//hal_uart_send_buff(lxl_sensor_all_data.rx_buf,lxl_sensor_all_data.rx_cnt) ;
			
			lxl_sensor_all_data.push_data_numable = lxl_sensor_all_data.rx_cnt&0xff;
			lxl_sensor_all_data.push_tick = 1 ;
			LOG("UART_11111_NFC: %d \n",lxl_sensor_all_data.rx_buf[0]) ;
		}else if(lxl_sensor_all_data.adv_mode_tick == 2 ){//当前为摄像头uart通讯 
			//hal_uart_send_buff(lxl_sensor_all_data.rx_buf,lxl_sensor_all_data.rx_cnt) ;



			LOG("UART_22222_SXT: %d \n",lxl_sensor_all_data.rx_buf[0]) ;
			lxl_uart_printf_hex(lxl_sensor_all_data.rx_buf,lxl_sensor_all_data.rx_cnt) ;
			//lxl_sensor_all_data.adv_mode_tick = 1 ;
			//lxl_uart_sensor_gpio_init(lxl_sensor_all_data.adv_mode_tick) ;  //使能一组uart
		}else{   //无通讯

		}
		lxl_sensor_all_data.rx_cnt = 0 ;
		return (events ^ MY_UART_1_EVENT_TICK);    
    }

    //osal_start_timerEx(lxl_lcd_count_TaskID, MY_LCD_1_EVENT_TICK, 2000);
    return 0 ;
}











