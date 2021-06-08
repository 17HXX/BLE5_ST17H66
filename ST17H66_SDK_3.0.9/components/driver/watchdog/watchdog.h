/**************************************************************************************************
*******
**************************************************************************************************/
#ifndef __WATCHDOG_H__
#define __WATCHDOG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "types.h"
#include "bus_dev.h"

typedef enum
{
    WDG_2S =   0,
    WDG_4S =   1,
    WDG_8S =   2,
    WDG_16S =  3,
    WDG_32S =  4,
    WDG_64S =  5,
    WDG_128S = 6,
    WDG_256S = 7
} WDG_CYCLE_Type_e;

#define WDG_USE_POLLING_MODE 0//this mode is recommended
#define WDG_USE_INT_MODE     1

#define HAL_WDG_CFG_CYCLE       WDG_2S
#define HAL_WDG_CFG_MODE        WDG_USE_POLLING_MODE

void Watchdog_Init(uint8 task_id);
uint16 Watchdog_ProcessEvent(uint8 task_id, uint16 events);

#ifdef __cplusplus
}
#endif

#endif
