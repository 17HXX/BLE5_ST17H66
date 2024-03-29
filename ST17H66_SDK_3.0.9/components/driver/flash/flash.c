﻿/**************************************************************************************************
*******
**************************************************************************************************/

/*******************************************************************************
    @file     flash.c
    @brief    Contains all functions support for flash driver
    @version  0.0
    @date     27. Nov. 2017
    @author   qing.han



*******************************************************************************/
#include "rom_sym_def.h"
#include <string.h>
#include "types.h"
#include "flash.h"
#include "log.h"
#include "pwrmgr.h"
#include "error.h"

#define SPIF_WAIT_IDLE_CYC                          (32)

#define SPIF_STATUS_WAIT_IDLE(n)                    \
    do                                              \
    {                                               \
        while((AP_SPIF->fcmd &0x02)==0x02);         \
        {                                           \
            volatile int delay_cycle = n;           \
            while (delay_cycle--){;}                \
        }                                           \
        while ((AP_SPIF->config & 0x80000000) == 0);\
    } while (0);


#define HAL_CACHE_ENTER_BYPASS_SECTION()  do{ \
        HAL_ENTER_CRITICAL_SECTION();\
        AP_CACHE->CTRL0 = 0x02; \
        AP_PCR->CACHE_RST = 0x02;\
        AP_PCR->CACHE_BYPASS = 1;    \
        HAL_EXIT_CRITICAL_SECTION();\
    }while(0);


#define HAL_CACHE_EXIT_BYPASS_SECTION()  do{ \
        HAL_ENTER_CRITICAL_SECTION();\
        AP_CACHE->CTRL0 = 0x00;\
        AP_PCR->CACHE_RST = 0x03;\
        AP_PCR->CACHE_BYPASS = 0;\
        HAL_EXIT_CRITICAL_SECTION();\
    }while(0);

#define spif_wait_nobusy(flg, tout_ns, return_val)   {if(_spif_wait_nobusy_x(flg, tout_ns)){if(return_val){ return return_val;}}}

static xflash_Ctx_t s_xflashCtx = {.spif_ref_clk=SYS_CLK_DLL_64M,.rd_instr=XFRD_FCMD_READ_DUAL};

chipMAddr_t  g_chipMAddr;

static void hal_cache_tag_flush(void)
{
    HAL_ENTER_CRITICAL_SECTION();
    uint32_t cb = AP_PCR->CACHE_BYPASS;
    volatile int dly = 8;

    if(cb==0)
    {
        AP_PCR->CACHE_BYPASS = 1;
    }

    AP_CACHE->CTRL0 = 0x02;

    while (dly--) {;};

    AP_CACHE->CTRL0 = 0x03;

    dly = 8;

    while (dly--) {;};

    AP_CACHE->CTRL0 = 0x00;

    if(cb==0)
    {
        AP_PCR->CACHE_BYPASS = 0;
    }

    HAL_EXIT_CRITICAL_SECTION();
}


static uint8_t _spif_read_status_reg_x(void)
{
    uint8_t status;
    spif_cmd(FCMD_RDST, 0, 2, 0, 0, 0);
    SPIF_STATUS_WAIT_IDLE(SPIF_WAIT_IDLE_CYC);
    spif_rddata(&status, 1);
    return status;
}

static int _spif_wait_nobusy_x(uint8_t flg, uint32_t tout_ns)
{
    uint8_t status;
    volatile int tout = (int )(tout_ns);

    for(; tout ; tout --)
    {
        status = _spif_read_status_reg_x();

        if((status & flg) == 0)
            return PPlus_SUCCESS;

        //insert polling interval
        //5*32us
        WaitRTCCount(5);
    }

    return PPlus_ERR_BUSY;
}

static void hal_cache_init(void)
{
    volatile int dly=100;
    //clock gate
    hal_clk_gate_enable(MOD_HCLK_CACHE);
    hal_clk_gate_enable(MOD_PCLK_CACHE);
    //cache rst ahp
    AP_PCR->CACHE_RST=0x02;

    while(dly--) {};

    AP_PCR->CACHE_RST=0x03;

    hal_cache_tag_flush();

    //cache enable
    AP_PCR->CACHE_BYPASS = 0;
}

static void hw_spif_cache_config(void)
{
    spif_config(s_xflashCtx.spif_ref_clk,/*div*/1,s_xflashCtx.rd_instr,0,0);
    AP_SPIF->wr_completion_ctrl=0xff010005;//set longest polling interval
    NVIC_DisableIRQ(SPIF_IRQn);
    NVIC_SetPriority((IRQn_Type)SPIF_IRQn, IRQ_PRIO_HAL);
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

int hal_flash_read(uint32_t addr, uint8_t* data, uint32_t size)
{
    volatile uint8_t* u8_spif_addr = (volatile uint8_t*)((addr & 0x7ffff) | FLASH_BASE_ADDR);
    uint32_t cb = AP_PCR->CACHE_BYPASS;
    #if(SPIF_FLASH_SIZE==FLASH_SIZE_1MB)
    uint32_t remap = addr & 0xf80000;

    if (remap)
    {
        AP_SPIF->remap = remap;
        AP_SPIF->config |= 0x10000;
    }

    #endif

    //read flash addr direct access
    //bypass cache
    if(cb == 0)
    {
        HAL_CACHE_ENTER_BYPASS_SECTION();
    }

    for(int i=0; i<size; i++)
        data[i]=u8_spif_addr[i];

    //bypass cache
    if(cb == 0)
    {
        HAL_CACHE_EXIT_BYPASS_SECTION();
    }

    #if(SPIF_FLASH_SIZE==FLASH_SIZE_1MB)

    if (remap)
    {
        AP_SPIF->remap = 0;
        AP_SPIF->config &= ~0x10000ul;
    }

    #endif
    return PPlus_SUCCESS;
}

int hal_flash_write(uint32_t addr, uint8_t* data, uint32_t size)
{
    uint8_t retval;
    HAL_CACHE_ENTER_BYPASS_SECTION();
    SPIF_STATUS_WAIT_IDLE(SPIF_WAIT_IDLE_CYC);
    spif_wait_nobusy(SFLG_WIP, SPIF_TIMEOUT, PPlus_ERR_BUSY);
    retval = spif_write(addr,data,size);
    SPIF_STATUS_WAIT_IDLE(SPIF_WAIT_IDLE_CYC);
    spif_wait_nobusy(SFLG_WIP, SPIF_TIMEOUT, PPlus_ERR_BUSY);
    HAL_CACHE_EXIT_BYPASS_SECTION();
    return retval;
}
int hal_flash_write_by_dma(uint32_t addr, uint8_t* data, uint32_t size)
{
    uint8_t retval;
    HAL_CACHE_ENTER_BYPASS_SECTION();
    SPIF_STATUS_WAIT_IDLE(SPIF_WAIT_IDLE_CYC);
    spif_wait_nobusy(SFLG_WIP, SPIF_TIMEOUT, PPlus_ERR_BUSY);
    retval = spif_write_dma(addr,data,size);
    SPIF_STATUS_WAIT_IDLE(SPIF_WAIT_IDLE_CYC);
    spif_wait_nobusy(SFLG_WIP, SPIF_TIMEOUT, PPlus_ERR_BUSY);
    HAL_CACHE_EXIT_BYPASS_SECTION();
    return retval;
}

int hal_flash_erase_sector(unsigned int addr)
{
    uint8_t retval;
    uint32_t cb = AP_PCR->CACHE_BYPASS;
    HAL_CACHE_ENTER_BYPASS_SECTION();
    SPIF_STATUS_WAIT_IDLE(SPIF_WAIT_IDLE_CYC);
    spif_wait_nobusy(SFLG_WIP, SPIF_TIMEOUT, PPlus_ERR_BUSY);
    retval = spif_erase_sector(addr);
    SPIF_STATUS_WAIT_IDLE(SPIF_WAIT_IDLE_CYC);
    spif_wait_nobusy(SFLG_WELWIP, SPIF_TIMEOUT, PPlus_ERR_BUSY);
    HAL_CACHE_EXIT_BYPASS_SECTION();

    if(cb == 0)
    {
        hal_cache_tag_flush();
    }

    return retval;
}

int hal_flash_erase_block64(unsigned int addr)
{
    uint8_t retval;
    uint32_t cb = AP_PCR->CACHE_BYPASS;
    HAL_CACHE_ENTER_BYPASS_SECTION();
    SPIF_STATUS_WAIT_IDLE(SPIF_WAIT_IDLE_CYC);
    spif_wait_nobusy(SFLG_WIP, SPIF_TIMEOUT, PPlus_ERR_BUSY);
    retval = spif_erase_block64(addr);
    SPIF_STATUS_WAIT_IDLE(SPIF_WAIT_IDLE_CYC);
    spif_wait_nobusy(SFLG_WELWIP, SPIF_TIMEOUT, PPlus_ERR_BUSY);
    HAL_CACHE_EXIT_BYPASS_SECTION();

    if(cb == 0)
    {
        hal_cache_tag_flush();
    }

    return retval;
}

int hal_flash_erase_all(void)
{
    uint8_t retval;
    uint32_t cb = AP_PCR->CACHE_BYPASS;
    HAL_CACHE_ENTER_BYPASS_SECTION();
    SPIF_STATUS_WAIT_IDLE(SPIF_WAIT_IDLE_CYC);
    spif_wait_nobusy(SFLG_WIP, SPIF_TIMEOUT, PPlus_ERR_BUSY);
    retval = spif_erase_all();
    SPIF_STATUS_WAIT_IDLE(SPIF_WAIT_IDLE_CYC);
    spif_wait_nobusy(SFLG_WELWIP, SPIF_TIMEOUT, PPlus_ERR_BUSY);
    HAL_CACHE_EXIT_BYPASS_SECTION();

    if(cb == 0)
    {
        hal_cache_tag_flush();
    }

    return retval;
}

int flash_write_word(unsigned int offset, uint32_t  value)
{
    uint32_t temp = value;
    offset &= 0x00ffffff;
    return (hal_flash_write (offset, (uint8_t*) &temp, 4));
}


CHIP_ID_STATUS_e read_chip_mAddr(void)
{
    CHIP_ID_STATUS_e ret = CHIP_ID_UNCHECK;
    uint8_t b;
    for(int i=0;i<CHIP_MADDR_LEN;i++)
    {
        ret = chip_id_one_bit_hot_convter(&b, read_reg(CHIP_MADDR_FLASH_ADDRESS+(i<<2)));

        if(ret==CHIP_ID_VALID)
        {
            g_chipMAddr.mAddr[CHIP_MADDR_LEN-1-i]=b;
        }
        else
        {
            if(i>0 && ret==CHIP_ID_EMPTY)
            {
                ret =CHIP_ID_INVALID;
            }

            return ret;
        }
    }

    return ret;
}

void check_chip_mAddr(void)
{
    //chip id check
    for(int i=0;i<CHIP_MADDR_LEN;i++)
    {
       g_chipMAddr.mAddr[i]=0xff;
    }
    g_chipMAddr.chipMAddrStatus=read_chip_mAddr();
}

void LOG_CHIP_MADDR(void)
{
    
    LOG("\n");
    if(g_chipMAddr.chipMAddrStatus==CHIP_ID_EMPTY)
    {
        LOG("[CHIP_MADDR EMPTY]\n");
    }
    else if(g_chipMAddr.chipMAddrStatus==CHIP_ID_INVALID)
    {
        LOG("[CHIP_MADDR INVALID]\n");
    }
    else if(g_chipMAddr.chipMAddrStatus==CHIP_ID_VALID)
    {

        LOG("[CHIP_MADDR VALID]\n");
        for(int i=0;i<CHIP_MADDR_LEN;i++)
        {
            LOG("%02x",g_chipMAddr.mAddr[i]);
        }
        LOG("\n");

    }
    else
    {
        LOG("[CHIP_MADDR UNCHECKED]\n");
    }
}


