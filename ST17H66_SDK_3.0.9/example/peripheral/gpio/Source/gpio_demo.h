/**************************************************************************************************
*******
**************************************************************************************************/

/**************************************************************************************************
    Filename:       gpio_demo.h
    Revised:        $Date $
    Revision:       $Revision $


**************************************************************************************************/

#ifndef __GPIO_DEMO_H__
#define __GPIO_DEMO_H__

#include "types.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define GPIO_WAKEUP_PIN_NUM     3

/*********************************************************************
    FUNCTIONS
*/
void Pulse_Measure_Init( uint8 task_id);
uint16 Pulse_Measure_ProcessEvent( uint8 task_id, uint16 events);


void GPIO_Wakeup_Init( uint8 task_id );
uint16 GPIO_Wakeup_ProcessEvent( uint8 task_id, uint16 events);


void Key_Demo_Init( uint8 task_id );
uint16 Key_ProcessEvent( uint8 task_id, uint16 events);
/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* HEARTRATE_H */
