/**************************************************************************************************
*******
**************************************************************************************************/

/**************************************************************************************************
    Filename:       heartrate.h
    Revised:        $Date $
    Revision:       $Revision $


**************************************************************************************************/

#ifndef HEARTRATE_H
#define HEARTRATE_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
    INCLUDES
*/

/*********************************************************************
    CONSTANTS
*/


// Heart Rate Task Events
#define adcMeasureTask_EVT                            0x0080

#define TIMER_1S_ONCE                               0x0001
#define TIMER_2S_CYCLE                              0x0002
#define TIMER_1MIN_CYCLE                                                        0x0004
#define TIMER_HALF_SECOND                           0x0008
#define TIMER_10MS_CYCLE                            0x0010
/*********************************************************************
    MACROS
*/

/*********************************************************************
    FUNCTIONS
*/

/*
    Task Initialization for the BLE Application
*/
extern void pulse_measure_Init( uint8 task_id );

/*
    Task Event Processor for the BLE Application
*/
extern uint16 pulse_measure_ProcessEvent( uint8 task_id, uint16 events );


/*
    Task Initialization for the BLE Application
*/
extern void spi_demo_Init( uint8 task_id );

/*
    Task Event Processor for the BLE Application
*/
extern uint16 spi_demo_ProcessEvent( uint8 task_id, uint16 events );


extern void gpio_wakeup_Init( uint8 task_id );
extern uint16 gpio_wakeup_ProcessEvent( uint8 task_id, uint16 events );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* HEARTRATE_H */
