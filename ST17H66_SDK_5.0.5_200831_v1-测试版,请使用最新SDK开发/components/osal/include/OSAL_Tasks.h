/**************************************************************************************************
  Filename:       OSAL_Tasks.h
  Revised:         
  Revision:        

  Description:    This file contains the OSAL Task definition and manipulation functions.

 
**************************************************************************************************/

#ifndef OSAL_TASKS_H
#define OSAL_TASKS_H

#ifdef __cplusplus
extern "C"
{
#endif


typedef signed   char    int8_t;     //!< Signed 8 bit integer
typedef unsigned char    uint8_t;    //!< Unsigned 8 bit integer
typedef signed   short   int16_t;    //!< Signed 16 bit integer
typedef unsigned short   uint16_t;   //!< Unsigned 16 bit integer
typedef signed   int     int32_t;     //!< Signed 32 bit integer
typedef unsigned int     uint32_t;    //!< Unsigned 32 bit integer
typedef signed   char    int8;       //!< Signed 8 bit integer
typedef unsigned char    uint8;      //!< Unsigned 8 bit integer
typedef signed   short   int16;      //!< Signed 16 bit integer
typedef unsigned short   uint16;     //!< Unsigned 16 bit integer
typedef signed   long    int32;      //!< Signed 32 bit integer
typedef unsigned long    uint32;     //!< Unsigned 32 bit integer
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
#define TASK_NO_TASK      0xFF

/*********************************************************************
 * TYPEDEFS
 */

/*
 * Event handler function prototype
 */
typedef unsigned short (*pTaskEventHandlerFn)( unsigned char task_id, unsigned short event );

/*********************************************************************
 * GLOBAL VARIABLES
 */

extern  const pTaskEventHandlerFn tasksArr[];
extern  const signed char tasksCnt;
extern  u16 *tasksEvents;

/*********************************************************************
 * FUNCTIONS
 */

/*
 * Call each of the tasks initailization functions.
 */
extern void osalInitTasks( void );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* OSAL_TASKS_H */
