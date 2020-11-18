/**************************************************************************************************
*******
**************************************************************************************************/

/**************************************************************************************************
  Filename:       dmac_demo.c
  Revised:        $Date $
  Revision:       $Revision $


**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */
 #include "string.h"
#include "OSAL.h"
#include "dmac_demo.h"
#include "log.h"

#include "gpio.h"
#include "clock.h"

#include "pwrmgr.h"
#include "error.h"
#include "dma.h"
#include "flash.h"

//#define MEM2MEM             0
//#define MEM2FLASH           1
//#define FLASH2MEM           2
//#define MEM2UART1           3
//#define UART12MEM           4
//#define UART12UART2         5

enum{
	MEM2MEM,		
	MEM2FLASH,
	FLASH2MEM,
	MEM2UART1,
	TRAN_COUNT
} ;



uint8_t g_dma_src_buffer_u8[0x400*2];
uint8_t g_dma_dst_buffer_u8[0x400*2];


static uint8 dma_TaskID,dma_demolst;

static DMA_CH_CFG_t cfg;

void dma_cb(DMA_CH_t ch)
{
    LOG("\ndma done!!!\n");
}


void DMA_Demo_Init(uint8 task_id ){    
    hal_gpio_init();
    hal_dma_init();
	
	dma_TaskID = task_id;

    HAL_DMA_t ch_cfg;
    ch_cfg.dma_channel = DMA_CH_0;
    ch_cfg.evt_handler = dma_cb;
    hal_dma_init_channel(ch_cfg);
	LOG("dma demo start...\n");

    osal_start_reload_timer(dma_TaskID, DMA_DEMO_CYCLE_TIMER , 2000);
}

void demo_dma_test(uint8 list)
{
    uint8_t mode;
    uint32_t i;

    mode = list % TRAN_COUNT;

    hal_dma_stop_channel(DMA_CH_0); //for dma demo test
    
    switch(mode)
    {
        case MEM2MEM:              
            cfg.transf_size = 0x100;				
            cfg.sinc = DMA_INC_INC;
            cfg.src_tr_width = DMA_WIDTH_WORD;
            cfg.src_msize = DMA_BSIZE_256;
            cfg.src_addr = (uint32_t)g_dma_src_buffer_u8;
				
            cfg.dinc = DMA_INC_INC;
            cfg.dst_tr_width = DMA_WIDTH_WORD;
            cfg.dst_msize = DMA_BSIZE_256;
            cfg.dst_addr = (uint32_t)g_dma_dst_buffer_u8;
            
            cfg.enable_int = false;
            hal_dma_config_channel(DMA_CH_0,&cfg);
            hal_dma_start_channel(DMA_CH_0);
            if(cfg.enable_int == false)
            {
                hal_dma_wait_channel_complete(DMA_CH_0);
                LOG("dma success\n");
            }             
            break;
        case MEM2FLASH:
            for(i = 0; i< 0x100; i++)
                g_dma_src_buffer_u8[i] = (uint8_t)(0xff-i);

            spif_erase_sector(0x11004000);
            
            cfg.transf_size = 0x100;
				
			cfg.sinc = DMA_INC_INC;
			cfg.src_tr_width = DMA_WIDTH_WORD;
			cfg.src_msize = DMA_BSIZE_1;
			cfg.src_addr = (uint32_t)g_dma_src_buffer_u8;
			
			cfg.dinc = DMA_INC_INC;
			cfg.dst_tr_width = DMA_WIDTH_WORD;
			cfg.dst_msize = DMA_BSIZE_1;
			cfg.dst_addr = (uint32_t)0x11004000;
			
			cfg.enable_int = true;

            hal_dma_config_channel(DMA_CH_0,&cfg);
            hal_dma_start_channel(DMA_CH_0);
            if(cfg.enable_int == false)
            {
                hal_dma_wait_channel_complete(DMA_CH_0);
                LOG("dma success\n");
            }               
            break;
        case FLASH2MEM:
            memset(g_dma_dst_buffer_u8,0,0x400*2);
            cfg.transf_size = 0x100;
				
			cfg.sinc = DMA_INC_INC;
			cfg.src_tr_width = DMA_WIDTH_WORD;
			cfg.src_msize = DMA_BSIZE_1;
			cfg.src_addr = (uint32_t)0x11004000;
			
			cfg.dinc = DMA_INC_INC;
			cfg.dst_tr_width = DMA_WIDTH_WORD;
			cfg.dst_msize = DMA_BSIZE_1;
			cfg.dst_addr = (uint32_t)g_dma_dst_buffer_u8;
			
			cfg.enable_int = true;

            hal_dma_config_channel(DMA_CH_0,&cfg);
            hal_dma_start_channel(DMA_CH_0);
            if(cfg.enable_int == false)
            {
                hal_dma_wait_channel_complete(DMA_CH_0);
                LOG("dma success\n");
            }  
            break;
        case MEM2UART1:
            for(i = 0; i< 100; i++)
                g_dma_src_buffer_u8[i] = (uint8_t)((0x7e)-i);
            cfg.transf_size = 0x20;
				
			cfg.sinc = DMA_INC_INC;
			cfg.src_tr_width = DMA_WIDTH_BYTE;
			cfg.src_msize = DMA_BSIZE_1;
			cfg.src_addr = (uint32_t)g_dma_src_buffer_u8;
			
			cfg.dinc = DMA_INC_NCHG;
			cfg.dst_tr_width = DMA_WIDTH_BYTE;
			cfg.dst_msize = DMA_BSIZE_1;
			cfg.dst_addr = (uint32_t)0x40004000;
			
			cfg.enable_int = true;
			
			hal_dma_config_channel(DMA_CH_0,&cfg);
            hal_dma_start_channel(DMA_CH_0);
            if(cfg.enable_int == false)
            {
                hal_dma_wait_channel_complete(DMA_CH_0);
                LOG("dma success\n");
            }
            break;
        default:
            break;
    }
}


uint16 DMA_ProcessEvent( uint8 task_id, uint16 events ){
	if(task_id != dma_TaskID){		
		return 0;
	}
    if( events & DMA_DEMO_CYCLE_TIMER){
        demo_dma_test(dma_demolst++);
//		LOG("once timer\n");
		return (events ^ DMA_DEMO_CYCLE_TIMER);
	}
	return 0;
}

/*********************************************************************
*********************************************************************/
