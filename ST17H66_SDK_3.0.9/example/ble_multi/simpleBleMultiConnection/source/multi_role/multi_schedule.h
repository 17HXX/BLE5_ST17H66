/*******************************************************************************************************
    -----------------------------------------------------------------------------------------------------
    @file      :    FileName multi role scheduler .h file
    @author    :
    @date      :    2020-12-4
    @brief     :    used to schedule the multi role task list
    @attention :
    -----------------------------------------------------------------------------------------------------
    Modification History
    DATE        NAME             DESCRIPTION
    -----------------------------------------------------------------------------------------------------

    -----------------------------------------------------------------------------------------------------
*******************************************************************************************************/


#ifndef _MULTI_SCHEDULE_H
#define _MULTI_SCHEDULE_H

/*******************************************************************************************************
    @ Description    :  MACRO DEFINE
    @author          :
 *******************************************************************************************************/
// for scheduler mode
#define MULTI_ADV_PARAM_MODE                    0x01        // advertising parameter
#define MULTI_SCH_SCAN_MODE                     0x02
#define MULTI_SCH_INITIATOR_MODE                0x04
#define MULTI_SCH_ADV_MODE                      0x08        // scheduler advertising list
#define MULTI_SCH_MODE                          MULTI_SCH_SCAN_MODE | MULTI_SCH_INITIATOR_MODE | MULTI_SCH_ADV_MODE
#define MULTI_LINK_MODE                         0x10

// scheduler error code
#define SCH_SUCCESS                     0x00
#define SCH_LIST_NULL                   0x01
#define SCH_INVALID_ERROR_CODE          0xFF

/*******************************************************************************************************
    @ Description    :  typedef -- multi role advertising parameter structure
    Modification History
    DATE        DESCRIPTION
    2020-12-7   first add for multi role advertising parameter support
    ------------------------------------------------------------------------------
    @author          :
 *******************************************************************************************************/
// advertising parameter structure
typedef struct
{
    uint8   AdvDataLen;
    uint8*  pAdvData;
    uint8   ScanRspDataLen;
    uint8*  pScanRspData;
} multiAdvParam_t;

// schedule advertising structure
typedef struct pAdv
{
    // idx : indicate which advertising parameter
    uint8   idx;

    // advertising parameter
    multiAdvParam_t advParam;
    struct pAdv*     next;
} multisch_adv_t;


/*******************************************************************************************************
    @ Description    :  API FUNCTIONS ---- multi role for multi.c
    @author          :
 *******************************************************************************************************/

/*******************************************************************************************************
 *******************************************************************************************************
    @ Description    :  multi role scheduler init
    @ Parameters     :
                   :  [IN]
                   :  [OUT]
    @ Return         :  Success: Failure:
    @ Other          :
    Modification History
    DATE        DESCRIPTION
    ------------------------------------------------------------------------------
    @author          :
 *******************************************************************************************************
 *******************************************************************************************************/
void multiSchedule_init(uint8 taskid);

/*******************************************************************************************************
 *******************************************************************************************************
    @ Description    :  multi role schedule advertising parameter init
    @ Parameters     :
                   :  [IN]  idx:the index of the advertising parameter , which want to be insert
                            param:advertising data or scan response data
                            len:the length of the data
                            pValue: the pointer of the value
                   :  [OUT] None
    @ Return         :  None
    @ Other          :
    Modification History
    DATE        DESCRIPTION
    ------------------------------------------------------------------------------
    @author          :
 *******************************************************************************************************
 *******************************************************************************************************/
void multiSchedule_advParam_init(uint8 idx,uint16 param, uint8 len, void* pValue);

/*******************************************************************************************************
 *******************************************************************************************************
    @ Description    :  multi role schedule advertising param index delete
    @ Parameters     :
                   :  [IN]  idx : the advertising parameter index,which want to be deleted
                   :  [OUT] None
    @ Return         :  Success: Failure:
    @ Other          :
    Modification History
    DATE        DESCRIPTION
    ------------------------------------------------------------------------------
    @author          :
 *******************************************************************************************************
 *******************************************************************************************************/
void multiSchedule_advParam_del( uint8 idx);

/*******************************************************************************************************
    @ Description    :  API FUNCTIONS ---- multi role for multi_role.c
    @author          :
 *******************************************************************************************************/

/*******************************************************************************************************
 *******************************************************************************************************
    @ Description    :  multi role schedule config
    @ Parameters     :
                   :  [IN]  mode : adv mode , scan mode or initiator mode
                            en_flag  :  enable or disable mode.for advertising en_flag bit0:enable/disable,
                                        bit4-bit7:advertising index
                   :  [OUT]
    @ Return         :  None
    @ Other          :  when enable schedule, will create the list,otherwise delete the list node
    Modification History
    DATE        DESCRIPTION
    ------------------------------------------------------------------------------
    @author          :
 *******************************************************************************************************
 *******************************************************************************************************/
uint8 muliSchedule_config(uint8 mode, uint8 en_flag);

/*******************************************************************************************************
 *******************************************************************************************************
    @ Description    :  multi role schedule process: advertising , scanning , initiating
    @ Parameters     :  None
                   :  [IN]  None
                   :  [OUT] None
    @ Return         :  None
    @ Other          :
    Modification History
    DATE        DESCRIPTION
    ------------------------------------------------------------------------------
    @author          :
  ********************************************************************************************************
  ********************************************************************************************************/
void multiScheduleProcess(void);

/*******************************************************************************************************
 *******************************************************************************************************
    @ Description    :  multiLinkStatusGetSlaveConnHandle
    @ Parameters     :
                   :  [IN]  idx : advertising parameter index
                   :  [OUT]
    @ Return         :  16bit value  connhandle
    @ Other          :
    Modification History
    DATE        DESCRIPTION
    ------------------------------------------------------------------------------
    @author          :
 *******************************************************************************************************
 *******************************************************************************************************/
uint16 multiLinkStatusGetSlaveConnHandle( uint8 idx);

uint8 multiLinkConnParamUpdate( gapLinkUpdateEvent_t* pPkt );
extern uint8 multiLinkGetMasterConnNum(void);

#endif


