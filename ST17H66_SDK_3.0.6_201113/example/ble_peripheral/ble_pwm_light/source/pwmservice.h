

/**************************************************************************************************
  Filename:       pwmservice.h
  Revised:        
  Revision:       

  Description:    This file contains the pwm private service definitions and
                  prototypes.

**************************************************************************************************/

#ifndef PWMSERVICE_H
#define PWMSERVICE_H

#include "types.h"
#include "bcomdef.h"
/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */



#define PWM_SERV_UUID 0x7777
#define PWM_CTRL_UUID 0x8877
#define PWM_NOTI_UUID 0xff72


//#define PWM_LIGHT_CTRL_ONOFF 0x52 //
//#define PWM_LIGHT_CTRL_PLATE_MUSICRHYTHM 0x53 //
//#define PWM_LIGHT_CTRL_MODE_SET 0x54 //
//#define PWM_LIGHT_CTRL_DATE_SYNC 0x55 //
//#define PWM_LIGHT_CTRL_ALARM_SET 0x56 //



/*********************************************************************
 * TYPEDEFS
 */

// Heart Rate Service callback function
typedef void (*PWMServiceCB_t)(uint8 ctrl);
bStatus_t PWM_Light_Notify(
													uint8*data,uint8 data_len
													);
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * Profile Callbacks
 */


/*********************************************************************
 * API FUNCTIONS 
 */


extern bStatus_t PWMS_AddService(PWMServiceCB_t pfnServiceCB );


#endif /* PWMSERVICE_H */
