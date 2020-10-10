/**************************************************************************************************
THIS IS EMPTY HEADER
**************************************************************************************************/

/*******************************************************************************
* @file		flash.h
* @brief	Contains all functions support for flash driver
* @version	0.0
* @date		27. Nov. 2017
* @author	qing.han
* 
* Copyright(C) 2016,  Semiconductor
* All rights reserved.
*
*******************************************************************************/
#ifndef _FLASH_H_
#define _FLASH_H_

#include "rom_sym_def.h"
#include "clock.h"
#include "types.h"
#include "gpio.h"
#include "version.h"

#define FLASH_SIZE_256KB        (0)
#define FLASH_SIZE_512KB        (1)
#define FLASH_SIZE_1MB          (2)

#define SPIF_FLASH_SIZE         FLASH_SIZE_512KB

#define SPIF_TIMEOUT       1000000

#define SFLG_WIP    1
#define SFLG_WEL    2
#define SFLG_WELWIP 3

//define flash ucds
#define FLASH_BASE_ADDR         (0x11000000)
#define FLASH_UCDS_ADDR_BASE    0x11005000

#define CHIP_ID_LENGTH          64
#define CHIP_ID_PID_LEN         16
#define CHIP_ID_LID_LEN         10
#define CHIP_ID_MID_LEN         16
#define CHIP_ID_TID_LEN         14
#define CHIP_ID_SID_LEN         8

#define CHIP_MADDR_LEN          6

//xip flash read instrcution
#define XFRD_FCMD_READ          0x801000B
#define XFRD_FCMD_READ_DUAL     0x801003B
#define XFRD_FCMD_READ_QUAD     0x801006B

typedef struct {
    sysclk_t      spif_ref_clk;         //
    uint32_t      rd_instr;      
}xflash_Ctx_t;

extern int _spif_wait_nobusy(uint8_t flg, uint32_t tout_ns);
extern int  spif_write(uint32_t addr, uint8_t* data, uint32_t size);
extern int  spif_read(uint32_t addr, uint8_t* data, uint32_t size);
extern int spif_erase_sector(unsigned int addr);
extern int spif_erase_block64(unsigned int addr);
extern int spif_erase_all(void);
extern uint8_t spif_flash_status_reg_0(void);


int hal_spif_cache_init(xflash_Ctx_t cfg);

int hal_flash_write(uint32_t addr, uint8_t* data, uint32_t size);
int hal_flash_read(uint32_t addr, uint8_t* data, uint32_t size);
int hal_flash_erase_sector(unsigned int addr);
int hal_flash_erase_block64(unsigned int addr);

int flash_write_word(unsigned int offset, uint32_t  value);

#endif








