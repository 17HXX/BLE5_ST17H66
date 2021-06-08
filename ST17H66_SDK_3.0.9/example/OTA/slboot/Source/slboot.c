/**************************************************************************************************
*******
**************************************************************************************************/


/*********************************************************************
    INCLUDES
*/

#include "bcomdef.h"
#include "OSAL.h"
#include "slboot.h"
#include "OSAL_Tasks.h"
#include "slb.h"
#include "ota_flash.h"
#include "error.h"
const uint8 tasksCnt = 0;
uint16* tasksEvents;
extern void bx_to_application(uint32_t run_addr);

const pTaskEventHandlerFn tasksArr[2] =
{
    NULL,
    NULL
};

void osalInitTasks( void )
{
}

#define __APP_RUN_ADDR__ (0x1FFF1838)

__asm void __attribute__((section("ota_app_loader_area"))) jump2app(void)
{
    LDR R0, = __APP_RUN_ADDR__
              LDR R1, [R0, #4]
              BX R1
              ALIGN
}

int __attribute__((section("ota_app_loader_area"))) run_application(void)
{
    int ret;
    HAL_ENTER_CRITICAL_SECTION();
    ret = ota_flash_load_app();

    if(ret == PPlus_SUCCESS)
    {
        jump2app();
    }

    HAL_EXIT_CRITICAL_SECTION();
    return PPlus_SUCCESS;
}

void slboot_main(void)
{
    //check firmware update (exchange area)
    slb_boot_load_exch_zone();
    //boot firmware
    run_application();

    while(1)
    {
        ;
    }
}





/*********************************************************************
*********************************************************************/
