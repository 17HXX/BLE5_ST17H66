
/**************************************************************************************************
  Filename:       pwmdemo.h
  Revised:        $Date $
  Revision:       $Revision $

**************************************************************************************************/

#ifndef PWMDEMO_H
#define PWMDEMO_H
#include "types.h"

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */

extern uint8 PWM_TaskID;

// Heart Rate Task Events
#define START_DEVICE_EVT                        0x0001
#define PWM_LIGHT_SET_LIGHTDATA_EVT             0x0002 //…Ë÷√µ∆data
#define PWM_LIGHT_INQUIRE_LIGHTDATA_EVT         0x0004 //≤È—Ø∂® ± ˝æ›
#define PWM_LIGHT_OFF_TIMING_EVT             0x0008
#define PWM_LIGTH_PERIOD_TIME_EVT            0x0010
#define PMW_LIGHT_PERIOD_FADE_EVT            0x0020
//#define PWM_LIGHT_SWITCHON_OFF_DATA_EVT               0x0008 //ø™πÿµ∆ø™πÿ
//#define PWM_LIGHT_CLOROPLATE_MUSIC_EVT                0x0010 //…´≈Ã&“Ù¿÷¬…∂Ø
//#define PWM_LIGHT_MODE_SET_EVT                        0x0020 //ƒ£ Ω…Ë÷√
//#define PWM_LIGHT_DATE_SYNC_EVT                       0x0040 // ±º‰Õ¨≤Ω°
//#define PWM_LIGHT_ALARM_SETTING_EVT                   0x0080 //ƒ÷÷”…Ë÷√
//#define PWM_LIGHT_EXTEND_FUNC_EVT                     0x0100 //¿©’ππ¶ƒ‹


/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task Initialization for the BLE Application
 */
extern void PWM_Init( uint8 task_id );

/*
 * Task Event Processor for the BLE Application
 */
 
extern uint16 PWM_ProcessEvent( uint8 task_id, uint16 events );



/*********************************************************************
*********************************************************************/


#endif /* PWMDEMO_H */
