/**************************************************************************************************
THIS IS EMPTY HEADER
**************************************************************************************************/

/*******************************************************************************
* @file		flash.c
* @brief	Contains all functions support for flash driver
* @version	0.0
* @date		27. Nov. 2017
* @author	qing.han
* 
* Copyright(C) 2016,  Semiconductor
* All rights reserved.
*
*******************************************************************************/
#include <string.h>
#include "types.h"
#include "flash.h"
#include "log.h"
#include "pwrmgr.h"
#include "error.h"

#define SPIF_STATUS_WAIT_IDLE(n) do{\
                                {\
                                    volatile int delay_cycle = n;\
                                    while(delay_cycle --){\
                                    ;}\
                                }\
                                while ((AP_SPIF->config & 0x80000000)==0);\
                            }while(0);
#define spif_wait_nobusy(flg, tout_ns, return_val)   {if(_spif_wait_nobusy(flg, tout_ns)){if(return_val){ HAL_EXIT_CRITICAL_SECTION();return return_val;}}}



static xflash_Ctx_t s_xflashCtx ={.spif_ref_clk=SYS_CLK_DLL_64M,.rd_instr=XFRD_FCMD_READ_DUAL};

static void hal_cache_init(void)
{
    volatile int dly=100;
    //clock gate
    hal_clk_gate_enable(MOD_HCLK_CACHE);
    hal_clk_gate_enable(MOD_PCLK_CACHE);

    //cache rst
    AP_PCR->CACHE_RST=0x00;
    while(dly--){};
    AP_PCR->CACHE_RST=0x03;
    //cache flush tag
    AP_CACHE->CTRL0 = 0x01;
    //cache enable
    AP_PCR->CACHE_BYPASS = 0;
}

static void hw_spif_cache_config(void)
{
    clk_spif_ref_clk(s_xflashCtx.spif_ref_clk);
    AP_SPIF->read_instr = s_xflashCtx.rd_instr;
    hal_cache_init();
}
int hal_spif_cache_init(xflash_Ctx_t cfg)
{
    memset(&(s_xflashCtx), 0, sizeof(s_xflashCtx));    
    memcpy(&(s_xflashCtx), &cfg, sizeof(s_xflashCtx));

    hw_spif_cache_config();

    hal_pwrmgr_register(MOD_SPIF, NULL,  hw_spif_cache_config);

    return PPlus_SUCCESS; 
}

int hal_flash_read(uint32_t addr, uint8_t *data, uint32_t size)
{
    volatile uint8_t *u8_spif_addr = (volatile uint8_t *)((addr & 0x7ffff) | FLASH_BASE_ADDR);
#if(SPIF_FLASH_SZIE==FLASH_SIZE_1MB)    
    uint32_t remap = addr & 0xf80000;
    HAL_ENTER_CRITICAL_SECTION(); 
    if (remap)
    {
        AP_SPIF->remap = remap;
        AP_SPIF->config |= 0x10000;
    }
#endif
    //read flash addr direct access
    for(int i=0;i<size;i++)
        data[i]=u8_spif_addr[i];

#if(SPIF_FLASH_SIZE==FLASH_SIZE_1MB)    
    if (remap)
    {
        AP_SPIF->remap = 0;
        AP_SPIF->config &= ~0x10000ul;
    }
    HAL_EXIT_CRITICAL_SECTION();
#endif

    return PPlus_SUCCESS;
}
int hal_flash_write(uint32_t addr, uint8_t *data, uint32_t size)
{
    volatile uint32_t i=0,u32_data;
    volatile uint32_t *u32_spif_addr = (volatile uint32_t *)((addr & 0x7ffff) | FLASH_BASE_ADDR);
    
    if(addr&0x03>0)
        return PPlus_ERR_INVALID_DATA;

    HAL_ENTER_CRITICAL_SECTION();
#if(SPIF_FLASH_SIZE==FLASH_SIZE_1MB)    
    //remap
    uint32_t remap = addr & 0xf80000;
    if (remap)
    {
        AP_SPIF->remap = remap;
        AP_SPIF->config |= 0x10000;
    }
#endif
    AP_SPIF->wr_protection = 0;
    //ZQ:20200828
    //word align write keep cache tag data correct
    while(i<(((size+3)>>2)<<2))
    {
        switch (size - i)
        {
        case 1:
            u32_data = BUILD_UINT32(data[i], 0x00, 0x00, 0x00);
            break;
        case 2:
            u32_data = BUILD_UINT32(data[i], data[i+1], 0x00, 0x00);
            break;
        case 3:
            u32_data = BUILD_UINT32(data[i], data[i+1], data[i+2],0x00);
            break;
        default:
            u32_data = BUILD_UINT32(data[i], data[i+1], data[i+2], data[i+3]);
            break;
        }
        *(u32_spif_addr++) = u32_data;
        i+=4;

    }
    SPIF_STATUS_WAIT_IDLE(8);
#if(SPIF_FLASH_SIZE==FLASH_SIZE_1MB)    
    if (remap)
    {
        AP_SPIF->remap = 0;
        AP_SPIF->config &= ~0x10000ul;
    }
#endif
    AP_SPIF->wr_protection = 2;
    spif_wait_nobusy(SFLG_WIP, SPIF_TIMEOUT, PPlus_ERR_BUSY);
    
    HAL_EXIT_CRITICAL_SECTION();

    return PPlus_SUCCESS;
}
int hal_flash_write_bypass_cache(uint32_t addr, uint8_t* data, uint32_t size)
{
    uint8_t retval;
    uint32_t cb = AP_PCR->CACHE_BYPASS;
    
    HAL_ENTER_CRITICAL_SECTION();
    retval = spif_write(addr,data,size);
    SPIF_STATUS_WAIT_IDLE(8);
    spif_wait_nobusy(SFLG_WIP, SPIF_TIMEOUT, PPlus_ERR_BUSY);
    if(cb == 0)
    {
        AP_CACHE->CTRL0 = 0x01;
    }
    HAL_EXIT_CRITICAL_SECTION();
    
    return retval;
}

int hal_flash_erase_sector(unsigned int addr)
{
    uint8_t retval;
    uint32_t cb = AP_PCR->CACHE_BYPASS;
    
    HAL_ENTER_CRITICAL_SECTION();
    retval = spif_erase_sector(addr);
    if(cb == 0)
    {
        AP_CACHE->CTRL0 = 0x01;
    }
    HAL_EXIT_CRITICAL_SECTION();
    return retval;
}

int hal_flash_erase_block64(unsigned int addr)
{
    uint8_t retval;
    uint32_t cb = AP_PCR->CACHE_BYPASS;
    
    HAL_ENTER_CRITICAL_SECTION();
    retval = spif_erase_block64(addr);
    if(cb == 0)
    {
        AP_CACHE->CTRL0 = 0x01;
    }
    HAL_EXIT_CRITICAL_SECTION();
    return retval;
}

int hal_flash_erase_all(void)
{
    uint8_t retval;
    uint32_t cb = AP_PCR->CACHE_BYPASS;
    
    HAL_ENTER_CRITICAL_SECTION();
    retval = spif_erase_all();
    if(cb == 0)
    {
        AP_CACHE->CTRL0 = 0x01;
    }
    HAL_EXIT_CRITICAL_SECTION();
    return retval;
}

int flash_write_word(unsigned int offset, uint32_t  value)
{
	uint32_t temp = value;
    offset &= 0x00ffffff;
	    
    return (hal_flash_write (offset, (uint8_t *) &temp, 4));
	
 	
}


