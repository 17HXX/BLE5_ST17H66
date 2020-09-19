/**************************************************************************************************
*******
**************************************************************************************************/

/**************************************************************************************************
  Filename:       watchdog_demo.c
  Revised:        $Date $
  Revision:       $Revision $


**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include "OSAL.h"
#include "watchdog_demo.h"
#include "log.h"
#include "gpio.h"
#include "clock.h"

static uint8_t watchdog_demo_TaskID;
#define DEMO_1000MS_EVENT    0x0001 
#define DEMO_1000MS_CYCLE    1000 

void Watchdog_Demo_Init( uint8 task_id )
{
	watchdog_demo_TaskID = task_id;

	LOG("\n-watchdog demo start-\n");
	LOG("when P11 and P14 are alwayt 0,feed watchdog,systye will not reset\n");
	LOG("when P11 is 1,watchdog timeout(the time you set through HAL_WDG_CFG_CYCLE),reset system\n");
	LOG("when P14 is 1,exception,reset system\n");
	
	hal_gpio_pin_init(P11,IE);
	hal_gpio_pin_init(P14,IE);
	
	osal_start_reload_timer(watchdog_demo_TaskID, DEMO_1000MS_EVENT, DEMO_1000MS_CYCLE);
}

uint16 Watchdog_Demo_ProcessEvent( uint8 task_id, uint16 events )
{
	static uint32_t counter = 0;
	
	if(events & DEMO_1000MS_EVENT)
	{
		LOG("%d\n",counter++);

		if(hal_gpio_read(P11) == 1) 
		{
			LOG("system will reset(watchdog reset)\n");
			while(1);
		}
		
		if(hal_gpio_read(P14) == 1) 
		{
			LOG("system will reset(hard fault,pc point an unknow zone)\n");
			((void(*)(void))0x10000)();
		}
		
		return (events ^ DEMO_1000MS_EVENT);
	}
	
  return 0;
}
