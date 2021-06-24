/*******************************************************************************************************
    -----------------------------------------------------------------------------------------------------
    @file      :    FileName
    @author    :
    @date      :    2020-12-18
    @brief     :
    @attention :
    -----------------------------------------------------------------------------------------------------
    Modification History
    DATE        NAME             DESCRIPTION
    -----------------------------------------------------------------------------------------------------

    -----------------------------------------------------------------------------------------------------
*******************************************************************************************************/
/*******************************************************************************************************
    -----------------------------------------------------------------------------------------------------
    @file      :    FileName multi role scheduler C file
    @author    :
    @date      :    2020-12-7
    @brief     :    multi role scheduler code
    @attention :    None
    -----------------------------------------------------------------------------------------------------
    Modification History
    DATE        NAME             DESCRIPTION
    -----------------------------------------------------------------------------------------------------

    -----------------------------------------------------------------------------------------------------
*******************************************************************************************************/

/*******************************************************************************************************
    @ Description    :  include
 *******************************************************************************************************/
#include "types.h"
#include "ll.h"
#include "multi.h"
#include "log.h"
#include "multi_schedule.h"

/*******************************************************************************************************
    @ Description    :  macro define
 *******************************************************************************************************/
#define MULTI_SCH_DELAY         500     // unit ms
#define MULTI_SCH_ADV_DURATION_TIME     4 //ms


/*******************************************************************************************************
    @ Description    :  static variable
 *******************************************************************************************************/
static uint8 sch_tsk_id = 0xFF;

/*******************************************************************************************************
    @ Description    :  extern variable
 *******************************************************************************************************/
extern uint8 gapMultiRole_TaskID;

/*******************************************************************************************************
    @ Description    :  multi role local typedef -- role type
 *******************************************************************************************************/
typedef enum
{
    advertiser = 1,
    scanner,
    initiator
} GAPMultiRole_type;

/*******************************************************************************************************
    @ Description    :  multi role local typedef -- schedule list
 *******************************************************************************************************/
typedef struct multiList
{
    GAPMultiRole_type   role;
    uint8   busy;
    union
    {
        struct
        {
            uint8   perIdx;     // g_MultiPeriInfo index
            uint8   DatConfUpd; // advertising data and scan response data
            // configure and update status @ref macro def
            // GAPMULTI_UPDATEADV_FLAG ...
            GAPMultiRole_states_t state;
        } adv;
        struct
        {
            uint8 scanning;     // is scanning now?
        } scan;
        struct
        {
            uint8 initiating;
        } initiate;
    } roleScd;                  // role scheduler parameter
    uint32 nextScdTime;         // next multi schedule time unit milliseconds
    struct multiList* next;
} multiScehdule_t;


/*******************************************************************************************************
    @ Description    :  multi role local variable
 *******************************************************************************************************/
// advertising parameter list
multisch_adv_t* msAdv_param = NULL;

// scheduler list
multiScehdule_t* mSche = NULL ;

// multi-role link state
GAPMultiRoleLinkCtrl_t* g_multiLinkInfo = NULL;

/*******************************************************************************************************
    @ Description    :  multi role scheduler static API
 *******************************************************************************************************/
uint8 multiRole_create_list(uint8 mode);
uint8 multiRole_cacLen_list(uint8 mode );
void multiRole_insert_node(uint8 mode,uint16 param, uint16 idx_len, void* pValue,void** node);
void multiRole_delete_node(uint8 mode,uint8 idx,void** node);
void* multiRole_search_phead(uint8 mode);
void* multiRole_search_preHead(uint8 mode,void** node);
void* multiRole_insertTail_node( uint8 mode );
multiScehdule_t* multiRole_findBusyNode(  void);


/*******************************************************************************************************
    @ Description    :  multi role advertising ,scanning , initiating static API
 *******************************************************************************************************/
#if( MAX_CONNECTION_SLAVE_NUM > 0 )
    static void multiScheduleAdv(multiScehdule_t* node);
#endif

#if( MAX_CONNECTION_MASTER_NUM > 0 )
    static void MultiScheduleScan(multiScehdule_t* node);
    static void MultiScheduleInitating(multiScehdule_t* node);
#endif


/*******************************************************************************************************
    @ Description    :  multi role configure scheduler advertising API
 *******************************************************************************************************/
#if( MAX_CONNECTION_SLAVE_NUM > 0 )
    void multiConfigSchAdv_param(uint8 opcode,uint8 status,uint8 advType );
#endif

#if( MAX_CONNECTION_MASTER_NUM > 0 )
    /*******************************************************************************************************
    @ Description    :  multi role configure scheduler scan API
    *******************************************************************************************************/
    void multiConfigSchScan_param(void);

    /*******************************************************************************************************
    @ Description    :  multi role configure link status API
    *******************************************************************************************************/
    GAPMultiLinkInfo_t multiConfigLink_status(uint8 opcode,void* pkt);
#endif

/*******************************************************************************************************
 *******************************************************************************************************
    @ Description    :  configure multi role scheduler mode
    @ Parameters     :
                   :  [IN]  mode : adv,scan, or initiator mode
                            en_flag  : enable or disable the status of the mode
                   :  [OUT] uint8: return uint8 indicate the resault value of the configuration
    @ Return         :  Success: Failure:
    @ Other          :  Public function defined in multi_schedule.h
    Modification History
    DATE        DESCRIPTION
    ------------------------------------------------------------------------------
    @author          :
 *******************************************************************************************************
 *******************************************************************************************************/
uint8 muliSchedule_config(uint8 mode, uint8 en_flag)
{
    uint8 ret = SCH_INVALID_ERROR_CODE;
    uint8 pSchLen = multiRole_cacLen_list( MULTI_SCH_MODE );
    multiScehdule_t* pSchnode = NULL;

    switch( mode )
    {
    case MULTI_SCH_ADV_MODE:
    {
        // advertising mode search pHead success, indicate that there's configed advertising
        multisch_adv_t* pAdvParamNode = multiRole_search_phead( MULTI_ADV_PARAM_MODE );
        uint8 pAdvParamLen = multiRole_cacLen_list( MULTI_ADV_PARAM_MODE );

        // paramete list
        if( en_flag & 0x01 )
        {
            // enable advertising scheduler
            // 1.check if there's already advertising node in schedule list
            // flag enable insert the node into schedule list
            uint8 insertFlag = TRUE;

            for(uint8 i=0; i< pAdvParamLen ; i++)
            {
                insertFlag = TRUE;
                pSchnode = multiRole_search_phead( MULTI_SCH_ADV_MODE );

                for(uint8 j=0; j<pSchLen; j++)
                {
                    if( ( pSchnode->role == advertiser) && \
                            ( pAdvParamNode->idx == pSchnode->roleScd.adv.perIdx ) )
                    {
                        // the advertising idx already in schedule list, so shall not insert
                        insertFlag = FALSE;
                        break;
                    }

                    pSchnode = pSchnode->next;
                }

                if( insertFlag && ( ( 1 << pAdvParamNode->idx ) & ( en_flag >> 4 )) )
                {
                    if( multiRole_search_phead( MULTI_SCH_ADV_MODE ) )
                    {
                        pSchnode = osal_mem_alloc( sizeof( multiScehdule_t ) );
                        osal_memset(pSchnode, 0, sizeof(multiScehdule_t));
                        multiRole_insert_node( mode,i, NULL, NULL,(void**)&pSchnode );
                    }
                    else
                    {
                        mSche = osal_mem_alloc( sizeof( multiScehdule_t ) );
                        osal_memset(mSche, 0, sizeof(multiScehdule_t));
                        multiRole_insert_node( mode,i, NULL, NULL,(void**)&mSche );
                    }
                }

                // next advertising paramter node
                pAdvParamNode = pAdvParamNode->next;
            }
        }
        else
        {
            // disable advertising scheduler
            // flag enable delete the node from the schedule list
            uint8 delFlag = FALSE;

            for(uint8 i=0; i< pAdvParamLen ; i++)
            {
                delFlag = FALSE;
                pSchnode = multiRole_search_phead( MULTI_SCH_ADV_MODE );

                for(uint8 j=0; j<pSchLen; j++)
                {
                    if( ( pSchnode->role == advertiser) && \
                            ( pAdvParamNode->idx == pSchnode->roleScd.adv.perIdx ) )
                    {
                        // the advertising idx already in schedule list, so shall not insert
                        delFlag = TRUE;
                        break;
                    }

                    pSchnode = pSchnode->next;
                }

                // if the advertising parameter is active in schedule list
                if( delFlag && ( 1 << pAdvParamNode->idx ) & ( en_flag >> 4 ) )
                {
                    if( pSchnode->roleScd.adv.state == GAPMULTIROLE_ADVERTISING)
                    {
                        uint8 ret = GAP_EndDiscoverable(sch_tsk_id);
                        LOG("delete GAP end discoverable ret %d\n",ret);
                    }

                    multiRole_delete_node(mode,i,(void**)&pSchnode );
                }

                // next advertising paramter node
                pAdvParamNode = pAdvParamNode->next;
            }
        }

        ret = SCH_SUCCESS;
    }
    break;

    case MULTI_SCH_SCAN_MODE:
    {
        pSchnode = multiRole_search_phead( MULTI_SCH_MODE );

        if( en_flag & 0x01 )
        {
            if( 0 == pSchLen )
            {
                mSche = osal_mem_alloc( sizeof( multiScehdule_t ) );
                osal_memset(mSche, 0, sizeof(multiScehdule_t));
                multiRole_insert_node( mode,NULL, NULL, NULL,(void**)&mSche );
            }
            else
            {
                pSchnode = osal_mem_alloc( sizeof( multiScehdule_t ) );
                osal_memset(pSchnode, 0, sizeof(multiScehdule_t));
                multiRole_insert_node( mode,NULL, NULL, NULL,(void**)&pSchnode );
            }
        }
        else
        {
            if( mSche )
            {
                while( pSchnode )
                {
                    if( pSchnode->role == scanner )
                    {
                        multiRole_delete_node(mode,NULL,(void**)&pSchnode );
                        break;
                    }

                    pSchnode = pSchnode->next;
                }
            }
        }

        ret = SCH_SUCCESS;
    }
    break;

    case MULTI_SCH_INITIATOR_MODE:
    {
        pSchnode = multiRole_search_phead( MULTI_SCH_MODE );

        if( en_flag & 0x01 )
        {
            if( 0 == pSchLen )
            {
                mSche = osal_mem_alloc( sizeof( multiScehdule_t ) );
                osal_memset(mSche, 0, sizeof(multiScehdule_t));
                multiRole_insert_node( mode,NULL, NULL, NULL,(void**)&mSche );
            }
            else
            {
                pSchnode = osal_mem_alloc( sizeof( multiScehdule_t ) );
                osal_memset(pSchnode, 0, sizeof(multiScehdule_t));
                multiRole_insert_node( mode,NULL, NULL, NULL,(void**)&pSchnode );
            }
        }
        else
        {
            if( mSche )
            {
                while( pSchnode )
                {
                    if( pSchnode->role == initiator )
                    {
                        multiRole_delete_node(mode,NULL,(void**)&pSchnode );
                        break;
                    }

                    pSchnode = pSchnode->next;
                }
            }
        }

        ret = SCH_SUCCESS;
    }
    break;

    default:
        break;
    }

    // the legth of schedule list is 0 , then start scheduler process
//  if( (SCH_SUCCESS == ret ) && ( pSchLen == 0 ) )
    pSchLen = multiRole_cacLen_list( MULTI_SCH_MODE );

    if( (SCH_SUCCESS == ret) && ( pSchLen > 0 )  )
    {
        //  if endable schedule after insert node
        if( osal_get_timeoutEx( sch_tsk_id,MULTI_SCHEDULE_EVT ) == 0  )
        {
            osal_start_timerEx( sch_tsk_id, MULTI_SCHEDULE_EVT, MULTI_SCH_DELAY );
        }
    }

    return ret;
}

/*******************************************************************************************************
    @ Description    :  multi role scheduler process
 *******************************************************************************************************/
void multiScheduleProcess(void)
{
//  LOG("%s,0x%X\n",__func__,__return_address());
    multiScehdule_t* bNode = multiRole_findBusyNode();

    if( mSche != NULL )
    {
        if( bNode == NULL )
        {
            mSche->busy = TRUE;
            bNode = mSche;
//          LOG("bNode %p,bNode->role %d\n",bNode,bNode->role);
        }

        if( bNode->role == advertiser )
        {
            #if( MAX_CONNECTION_SLAVE_NUM > 0 )
            multiScheduleAdv( bNode );
            #endif
        }
        else if( bNode->role == scanner )
        {
            #if( MAX_CONNECTION_MASTER_NUM > 0 )
            MultiScheduleScan(bNode);
            #endif
        }
        else if( bNode->role == initiator )
        {
            #if( MAX_CONNECTION_MASTER_NUM > 0 )
            MultiScheduleInitating(bNode);
            #endif
        }
    }
}


/*******************************************************************************************************
    @ Description    :  multi role create schedule list
 *******************************************************************************************************/
uint8 multiRole_create_list(uint8 mode)
{
    uint8 ret=SCH_INVALID_ERROR_CODE;

    switch( mode )
    {
    case MULTI_ADV_PARAM_MODE:
    {
        msAdv_param = (multisch_adv_t*)osal_mem_alloc(sizeof(multisch_adv_t));

        if( msAdv_param )
        {
            ret = SCH_SUCCESS;
            osal_memset(msAdv_param, 0, sizeof(multisch_adv_t));
        }
        else
            ret = SCH_LIST_NULL;
    }
    break;

    case MULTI_SCH_SCAN_MODE:
        break;

    case MULTI_SCH_INITIATOR_MODE:
        break;

    default:
        break;
    }

    return ret;
}

/*******************************************************************************************************
 *******************************************************************************************************
    @ Description    :  calculate the length of the list
    @ Parameters     :
                   :  [IN]  mode :
                   :  [OUT]
    @ Return         :  length
    @ Other          :
    Modification History
    DATE        DESCRIPTION
    ------------------------------------------------------------------------------
    @author          :
 *******************************************************************************************************
 *******************************************************************************************************/
uint8 multiRole_cacLen_list(uint8 mode )
{
//  LOG("%s,mode %d\n",__func__,mode);
    uint8 len = 0;

    switch( mode )
    {
    case MULTI_ADV_PARAM_MODE:
    {
        multisch_adv_t* node = msAdv_param;

        while( node != NULL )
        {
            len++;
            node = node->next;
        }
    }
    break;

    case MULTI_SCH_MODE:
    case MULTI_SCH_ADV_MODE:
    case MULTI_SCH_SCAN_MODE:
    case MULTI_SCH_INITIATOR_MODE:
    {
        multiScehdule_t* node = mSche;

        while( node != NULL )
        {
            len++;
            node = node->next;
        }
    }
    break;

    case MULTI_LINK_MODE:
    {
        GAPMultiRoleLinkCtrl_t* node = g_multiLinkInfo;

        while( node != NULL )
        {
            len++;
            node = node->next;
        }
    }
    break;

    default:
        break;
    }

    return len;
}

/*******************************************************************************************************
 *******************************************************************************************************
    @ Description    :
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
void* multiRole_insertTail_node( uint8 mode )
{
    switch( mode )
    {
    case MULTI_ADV_PARAM_MODE:
    {
        multisch_adv_t* node = msAdv_param;

        while( node )
        {
            if( node->next == NULL )
            {
                // insert node at the end of the list
                node->next = (multisch_adv_t*)osal_mem_alloc(sizeof(multisch_adv_t));
                node->next->next = NULL;
                // jump out of the while loop
                return (void*)node->next;
            }

            node = node->next;
        }
    }
    break;

    case MULTI_SCH_SCAN_MODE:
        break;

    case MULTI_SCH_INITIATOR_MODE:
        break;

    default:
        break;
    }

    return NULL;
}

multiScehdule_t* multiRole_findBusyNode( void )
{
    multiScehdule_t* node = multiRole_search_phead( MULTI_SCH_MODE );

    while( node )
    {
        if( node->busy == TRUE )
            return node;
        else
            node = node->next;
    }

    return NULL;
}

/*******************************************************************************************************
 *******************************************************************************************************
    @ Description    :
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
void* multiRole_search_phead(uint8 mode)
{
    void* node;

    switch( mode )
    {
    case MULTI_ADV_PARAM_MODE:
    {
//          multisch_adv_t *node = (multisch_adv_t *)msAdv_param;
        node = (multisch_adv_t*)msAdv_param;
//          return node;
    }
    break;

    case MULTI_SCH_MODE:
    case MULTI_SCH_ADV_MODE:
    case MULTI_SCH_SCAN_MODE:
    case MULTI_SCH_INITIATOR_MODE:
    {
//          multiScehdule_t *
        node =  (multiScehdule_t*)mSche;
//          return node;
    }
    break;

    case MULTI_LINK_MODE:
    {
//          GAPMultiRoleLinkCtrl_t *
        node = (GAPMultiRoleLinkCtrl_t*)g_multiLinkInfo;
//          return node;
    }
    break;

    default:
        node = (uint8*)NULL;
        break;
    }

    return node;
}

/*******************************************************************************************************
 *******************************************************************************************************
    @ Description    :
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
void* multiRole_search_preHead(uint8 mode,void** node)
{
    void* Rnode = NULL;

    switch( mode )
    {
    case MULTI_SCH_MODE:
    case MULTI_SCH_ADV_MODE:
    case MULTI_SCH_SCAN_MODE:
    case MULTI_SCH_INITIATOR_MODE:
    {
        multiScehdule_t* tnode = (multiScehdule_t*)(*node);
        multiScehdule_t* pNode = mSche;

        while( pNode )
        {
            if( pNode->next == tnode )
                break;

            pNode = pNode->next;
        }

        Rnode = (multiScehdule_t*)pNode;
    }
    break;

    case MULTI_LINK_MODE:
    {
        GAPMultiRoleLinkCtrl_t* tnode = (GAPMultiRoleLinkCtrl_t*)(*node);
        GAPMultiRoleLinkCtrl_t* pNode = g_multiLinkInfo;

        while( pNode )
        {
            if( pNode->next == tnode )
                break;

            pNode = pNode->next;
        }

        Rnode = (GAPMultiRoleLinkCtrl_t*)pNode;
    }
    break;

    default:
        Rnode = (uint8*)NULL;
        break;
    }

    return Rnode ;
}


/*******************************************************************************************************
 *******************************************************************************************************
    @ Description    :
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
void multiRole_insert_node(uint8 mode,uint16 param, uint16 idx_len, void* pValue,void** pnode)
{
//  LOG("%s\n",__func__);
    switch( mode )
    {
    case MULTI_ADV_PARAM_MODE:
    {
        multisch_adv_t* pAdv = (multisch_adv_t*)(*pnode);
        uint8 len = idx_len & 0x00FF;
        uint8 idx = idx_len >> 8 ;
        pAdv->idx = idx;

        switch( param )
        {
        case GAPMULTIROLE_ADVERT_DATA:
            pAdv->advParam.AdvDataLen = len;
            pAdv->advParam.pAdvData = osal_mem_alloc( len );
            osal_memcpy( pAdv->advParam.pAdvData,(uint8*)pValue,len );
            break;

        case GAPMULTIROLE_SCAN_RSP_DATA:
            pAdv->advParam.ScanRspDataLen = len;
            pAdv->advParam.pScanRspData = osal_mem_alloc( len );
            osal_memcpy( pAdv->advParam.pScanRspData,(uint8*)pValue,len );
            break;

        default:
            break;
        }
    }
    break;

    case MULTI_SCH_SCAN_MODE:
    {
        multiScehdule_t* insNode = (multiScehdule_t*)(*pnode);
        // 1.   find first schedule list node,
        multiScehdule_t* node =  NULL;
        node = multiRole_search_phead( MULTI_SCH_SCAN_MODE );

        if( insNode != mSche )
        {
            while( node != NULL)
            {
                LOG("node %p\n",node);

                if( node->next == NULL )
                {
                    node->next = insNode;
                    LOG("node->next->next %p\n",node->next->next);
                    break;
                }

                node = node->next;
            }
        }
        else
            mSche = insNode;

        insNode->busy = FALSE;
        insNode->role = scanner;
        node = multiRole_search_phead( MULTI_SCH_SCAN_MODE );
        uint8 advCnt = 0;

        while( node != NULL)
        {
            if( node->role == advertiser )
                advCnt++;

            node = node->next;
        }

        uint16 advIntv = GAP_GetParamValue( TGAP_GEN_DISC_ADV_INT_MIN );

        if( advCnt > 0 )
            insNode->nextScdTime = ( (uint32)advIntv * 0.625 ) - ( MULTI_SCH_ADV_DURATION_TIME * advCnt );
        else
            insNode->nextScdTime = GAP_GetParamValue(TGAP_GEN_DISC_SCAN);
    }
    break;

    case MULTI_SCH_INITIATOR_MODE:
    {
        multiScehdule_t* insNode = (multiScehdule_t*)(*pnode);
        // 1.   find first schedule list node,
        multiScehdule_t* node =  NULL;
        node = multiRole_search_phead( MULTI_SCH_SCAN_MODE );

        if( insNode != mSche )
        {
            while( node != NULL)
            {
                LOG("node %p\n",node);

                if( node->next == NULL )
                {
                    node->next = insNode;
                    LOG("node->next->next %p\n",node->next->next);
                    break;
                }

                node = node->next;
            }
        }
        else
            mSche = insNode;

        insNode->busy = FALSE;
        insNode->role = initiator;
        node = multiRole_search_phead( MULTI_SCH_SCAN_MODE );
        uint8 advCnt = 0;

        while( node != NULL)
        {
            if( node->role == advertiser )
                advCnt++;

            node = node->next;
        }

        uint16 advIntv = GAP_GetParamValue( TGAP_GEN_DISC_ADV_INT_MIN );

        // nextScdTime seems to be create connection timeout ,units :ms
        if( advCnt > 0 )
            insNode->nextScdTime = ( (uint32)advIntv * 0.625 ) - ( MULTI_SCH_ADV_DURATION_TIME * advCnt );
        else
            insNode->nextScdTime = 10000;
    }
    break;

    case MULTI_SCH_ADV_MODE:
    {
        multiScehdule_t* insNode = (multiScehdule_t*)(*pnode);
        // 1.   find first schedule list node,
        multiScehdule_t* node =  NULL;
        node = multiRole_search_phead( MULTI_SCH_ADV_MODE );
        // 2.1. there's already has advertising schedule node, insert the advertising node after the first advertising node
        multiScehdule_t* temp = NULL;

        if( ( node!= NULL)&& (node->role == advertiser ))
        {
            // temporary node->next
            temp = node->next;
            // set the next node
            insNode->next = temp;
            node->next = insNode;
        }
        // 2-2. there's no advertising schedule node,then insert the advertising node in the first
        else
        {
            // 2.2.1 bugfix for first insert
            if( insNode == mSche )
            {
                mSche = insNode;
            }
            // 2.2.2 there's scanning or initiator node in schedule list
            else
            {
                temp = mSche;
                mSche = insNode;
                insNode->next = temp;
            }
        }

        // insert node parameter config
        insNode->role = advertiser;
        insNode->busy = FALSE;
        insNode->roleScd.adv.perIdx = param;
        uint16 advIntv = GAP_GetParamValue( TGAP_GEN_DISC_ADV_INT_MIN );
        insNode->nextScdTime = advIntv * 0.625;
        uint8 advEvtType;
        GAPMultiRole_GetParameter(GAPMULTIROLE_ADV_EVENT_TYPE, &advEvtType);

        if( advEvtType == LL_ADV_CONNECTABLE_UNDIRECTED_EVT )
            insNode->roleScd.adv.DatConfUpd = GAPMULTI_UPDATEADV_FLAG | GAPMULTI_UPDATESRD_FLAG;
    }
    break;

    case MULTI_LINK_MODE:
    {
        GAPMultiRoleLinkCtrl_t* pNode = (GAPMultiRoleLinkCtrl_t*)(*pnode);

        // 1. if g_multiLinkInfo is NULL
        if( g_multiLinkInfo == NULL )
        {
            g_multiLinkInfo = pNode;
        }
        else
        {
            // 2. find the tail node
            GAPMultiRoleLinkCtrl_t* tNode = g_multiLinkInfo;

            while( tNode )
            {
                if(tNode->next == NULL)
                    break;

                tNode = tNode->next;
            }

            tNode->next = pNode;
        }
    }
    break;

    default:
        break;
    }
}

/*******************************************************************************************************
 *******************************************************************************************************
    @ Description    :
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
void multiRole_delete_node(uint8 mode,uint8 idx,void** node)
{
    switch( mode )
    {
    case MULTI_SCH_MODE:
    case MULTI_SCH_ADV_MODE:
    case MULTI_SCH_SCAN_MODE:
    case MULTI_SCH_INITIATOR_MODE:
    {
        multiScehdule_t* delNode = (multiScehdule_t*)(*node);

        // 1. if the delete node is the head node
        if( delNode == mSche )
        {
            // case 1. the length of mSche is 1
            if( 1 == multiRole_cacLen_list( MULTI_SCH_MODE ) )
            {
                osal_mem_free( mSche );
                mSche = NULL;
            }
            // case 2. the length of mSche is greater than 1
            else
            {
                mSche = delNode->next;
                osal_mem_free( delNode );
            }
        }
        // 2. if the delete node is the mid/tail node
        else
        {
            // find previous node
            multiScehdule_t* tNode = multiRole_search_preHead(mode, node );
            tNode->next = delNode->next;
            osal_mem_free( delNode );
        }
    }
    break;

    case MULTI_LINK_MODE:
    {
        uint8 len = multiRole_cacLen_list( MULTI_LINK_MODE );
        GAPMultiRoleLinkCtrl_t* delNode = ( GAPMultiRoleLinkCtrl_t*)(*node);

        if( delNode == g_multiLinkInfo )
        {
            if( 1 == len )
            {
                osal_mem_free( g_multiLinkInfo );
                g_multiLinkInfo = NULL;
            }
            else
            {
                g_multiLinkInfo = delNode->next;
                osal_mem_free(delNode);
            }
        }
        else
        {
            GAPMultiRoleLinkCtrl_t* tNode = multiRole_search_preHead(mode, node );
            tNode->next = delNode->next;
            osal_mem_free( delNode );
        }
    }
    break;

    default:
        break;
    }
}

#if( MAX_CONNECTION_SLAVE_NUM > 0 )
/*******************************************************************************************************
 *******************************************************************************************************
    @ Description    :
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
static void multiScheduleAdv(multiScehdule_t* node)
{
    bStatus_t ret = SUCCESS;
//  LOG("%s\n",__func__);
//  LOG("node->roleScd.adv.perIdx %d\n",node->roleScd.adv.perIdx);
    multisch_adv_t* pAdvParamNode = multiRole_search_phead( MULTI_ADV_PARAM_MODE );

    for(uint8 i=0; i<multiRole_cacLen_list(MULTI_ADV_PARAM_MODE); i++)
    {
//      LOG("pAdvParamNode->idx %d\n",pAdvParamNode->idx);
        if( pAdvParamNode->idx == node->roleScd.adv.perIdx )
        {
            break;
        }

        pAdvParamNode = pAdvParamNode->next;
    }

//  LOG("pAdvParamNode %p\n",pAdvParamNode);
//  LOG("node->roleScd.adv.DatConfUpd 0x%02X\n",node->roleScd.adv.DatConfUpd);
//  LOG("node->roleScd.adv.state 0x%02X\n",node->roleScd.adv.state);

    if( (node->roleScd.adv.DatConfUpd & GAPMULTI_UPDATEADV_FLAG ) &&  \
            ((node->roleScd.adv.DatConfUpd & GAPMULTI_ADV_UPDATED ) != GAPMULTI_ADV_UPDATED ))
    {
        // shall update advertising data , and never updated
        ret = GAP_UpdateAdvertisingData(    sch_tsk_id,\
                                            TRUE, \
                                            pAdvParamNode->advParam.AdvDataLen, \
                                            pAdvParamNode->advParam.pAdvData);
        node->nextScdTime = 0;

        if( ret == SUCCESS  )
        {
            node->busy = TRUE;
//              LOG("Advertising data update success\n");
        }
        else
            LOG("Advertising data update error %d\n",ret);
    }
    else if( (node->roleScd.adv.DatConfUpd & GAPMULTI_UPDATESRD_FLAG ) &&  \
             ((node->roleScd.adv.DatConfUpd & GAPMULTI_SRD_UPDATED ) != GAPMULTI_SRD_UPDATED ))
    {
        // shall update scan response data , and never updated
        ret = GAP_UpdateAdvertisingData(    sch_tsk_id,\
                                            FALSE, \
                                            pAdvParamNode->advParam.ScanRspDataLen, \
                                            pAdvParamNode->advParam.pScanRspData);
        node->nextScdTime = 0;

        if( ret == SUCCESS  )
        {
//          LOG("scan response data update success\n");
        }
    }
    else if( node->roleScd.adv.state == GAPMULTIROLE_INIT )
    {
        uint8 advChnMap;
        GAPMultiRole_GetParameter(GAPMULTIROLE_ADV_CHANNEL_MAP,&advChnMap);
        uint8 advType;
        GAPMultiRole_GetParameter( GAPMULTIROLE_ADV_EVENT_TYPE, &advType);
        uint8 advFilterPolicy ;
        GAPMultiRole_GetParameter(GAPMULTIROLE_ADV_FILTER_POLICY,&advFilterPolicy);
        gapAdvertisingParams_t params;
        params.channelMap = advChnMap;
        params.eventType  = advType;
        params.filterPolicy = advFilterPolicy;
        // Start advertising
        uint8 ret = GAP_MakeDiscoverable(sch_tsk_id, &params);

        if( ret == SUCCESS )
        {
            node->nextScdTime = 0;
//            LOG("gap make discoverable done success\n");
        }
        else
            LOG("gap make discoverable done error %d\n",ret);
    }
    else
    {
        ret = GAP_EndDiscoverable(sch_tsk_id);
        node->nextScdTime = 0;

        if( ret == SUCCESS )
        {
            node->roleScd.adv.DatConfUpd &= ~( GAPMULTI_ADV_UPDATED | GAPMULTI_SRD_UPDATED);
//            LOG("gap end  discoverable done success\n");
        }
        else
        {
            LOG("gap end  discoverable done error %d\n",ret);
        }
    }
}


/*******************************************************************************************************
 *******************************************************************************************************
    @ Description    :  multiConfigSchAdv_param
    @ Parameters     :
                   :  [IN]  advType
                            TRUE:advertising data update,FALSE:scan response data
                   :  [OUT] None
    @ Return         :  None
    @ Other          :
    Modification History
    DATE        DESCRIPTION
    ------------------------------------------------------------------------------
    @author          :
 *******************************************************************************************************
 *******************************************************************************************************/
void multiConfigSchAdv_param(uint8 opcode,uint8 status,uint8 advType )
{
    multiScehdule_t* node = multiRole_findBusyNode();
//  LOG("%s,node %p\n",__func__,node);
    uint32 nextScdTime = 0;

    if( node )
    {
        switch( opcode )
        {
        case GAP_ADV_DATA_UPDATE_DONE_EVENT:
        {
            if (status == SUCCESS)
            {
                if( advType )
                    node->roleScd.adv.DatConfUpd |= GAPMULTI_ADV_UPDATED;
                else
                    node->roleScd.adv.DatConfUpd |= GAPMULTI_SRD_UPDATED;
            }
            else
            {
                if( advType )
                    node->roleScd.adv.DatConfUpd &= ~GAPMULTI_ADV_UPDATED;
                else
                    node->roleScd.adv.DatConfUpd &= ~GAPMULTI_SRD_UPDATED;
            }
        }
        break;

        case GAP_MAKE_DISCOVERABLE_DONE_EVENT:
        {
            if( status == SUCCESS)
            {
                nextScdTime = MULTI_SCH_ADV_DURATION_TIME;
//                  LOG("GAP_MakeDiscoverable Done \n");
            }
            else
            {
                LOG("GAP_MakeDiscoverable err 0x%02X \n",status);
            }

            node->roleScd.adv.state = GAPMULTIROLE_ADVERTISING;
        }
        break;

        case GAP_END_DISCOVERABLE_DONE_EVENT :
        {
            if( status == SUCCESS)
            {
                node->roleScd.adv.state = GAPMULTIROLE_INIT;
                // clear the current busy node
                node->busy = FALSE;

                // once advertising event done,check the next idle node
                // assume that only one node active
                if( node->next != NULL )
                {
                    node->next->busy = TRUE;
                }
                else
                {
                    // 2 advertising case
                    if( multiRole_search_phead( MULTI_SCH_MODE ) != NULL )
                        mSche->busy = TRUE;
                }

//                  LOG("GAP_EndDiscoverable Done \n");
            }
            else
            {
                LOG("GAP_EndDiscoverable err 0x%02X \n",status);
            }
        }
        break;
        }
    }

    if( multiRole_search_phead(MULTI_SCH_MODE) == NULL )
        return;

    if( ( 1 ==  multiRole_cacLen_list( MULTI_SCH_MODE ) ) &&
            ( node->role == advertiser) && (node->roleScd.adv.state == GAPMULTIROLE_ADVERTISING))
    {
        // adv
        // if the length of schedule list is 1 and is in advertising state
        // so not schedule,
    }
    else if( 2 ==  multiRole_cacLen_list( MULTI_SCH_MODE ) )
    {
        // if the length of schedule list is 2 and is in advertising state
        if( ( node->next->role != advertiser) && (node->roleScd.adv.state == GAPMULTIROLE_ADVERTISING) )
        {
            // node->next->role != advertiser -->
            // adv + adv
            uint16 advIntv = GAP_GetParamValue( TGAP_GEN_DISC_ADV_INT_MIN );
            nextScdTime = ( (uint32)advIntv * 0.625 ) - ( MULTI_SCH_ADV_DURATION_TIME * 2 );
        }
        else
        {
            // adv + scan
            // advertising done,start scan immediately
            nextScdTime = 0;
        }

        // adv + init

        if( nextScdTime > 0 )
            osal_start_timerEx(sch_tsk_id,MULTI_SCHEDULE_EVT,nextScdTime );
        else
//          osal_set_event(sch_tsk_id,MULTI_SCHEDULE_EVT);
            osal_start_timerEx(sch_tsk_id,MULTI_SCHEDULE_EVT,MULTI_SCH_ADV_DURATION_TIME );
    }
    else if( 3 ==  multiRole_cacLen_list( MULTI_SCH_MODE ) )
    {
        // 1. adv + adv + scan
        if( nextScdTime > 0 )
            osal_start_timerEx(sch_tsk_id,MULTI_SCHEDULE_EVT,nextScdTime );
        else
//          osal_set_event(sch_tsk_id,MULTI_SCHEDULE_EVT);
            osal_start_timerEx(sch_tsk_id,MULTI_SCHEDULE_EVT,MULTI_SCH_ADV_DURATION_TIME );

        // 2. adv + adv + init
    }
    else
    {
        // scheduler length : 1
        // scan
        // init
        if( nextScdTime > 0 )
            osal_start_timerEx(sch_tsk_id,MULTI_SCHEDULE_EVT,nextScdTime );
        else
//          osal_set_event(sch_tsk_id,MULTI_SCHEDULE_EVT);
            osal_start_timerEx(sch_tsk_id,MULTI_SCHEDULE_EVT,MULTI_SCH_ADV_DURATION_TIME );
    }
}

uint8 multiLinkConnParamUpdate( gapLinkUpdateEvent_t* pPkt )
{
    GAPMultiRoleLinkCtrl_t* node = g_multiLinkInfo;
    uint8 perIdx = 0xFF;

    while( node )
    {
        if( ( node->connectionHandle & 0x000F ) == pPkt->connectionHandle )
            break;

        node = node->next;
    }

    node->connInterval =  pPkt->connInterval;
    node->connLatency = pPkt->connLatency;
    node->connTimeout = pPkt->connTimeout;

    if( node->RoleState == Slave_Role )
    {
        perIdx = (node->connectionHandle & 0xF000 ) >> 12 ;
    }

    return perIdx;
}

/*******************************************************************************************************
 *******************************************************************************************************
    @ Description    :  multiLinkStatusGetSlaveConnHandle
    @ Parameters     :
                   :  [IN]  indexx
                   :  [OUT] None
    @ Return         :  16bit value   connhandle
    @ Other          :  bit value , the same as multiConfigLink_status()
    Modification History
    DATE        DESCRIPTION
    ------------------------------------------------------------------------------
    @author          :
 *******************************************************************************************************
 *******************************************************************************************************/
uint16 multiLinkStatusGetSlaveConnHandle( uint8 idx)
{
    GAPMultiRoleLinkCtrl_t* node = g_multiLinkInfo;
    uint16 connHandle = 0xFFFF;

    while( node )
    {
        if( node->RoleState == Slave_Role )
        {
            if( idx == ( ( node->connectionHandle & 0xF000 ) >> 12  ) )
            {
                connHandle = node->connectionHandle & 0x000F;
                break;
            }
        }

        node = node->next;
    }

    return connHandle;
}

/*******************************************************************************************************
    @ Description    :  multi role advertising parameter initilization , define in multi_schedule.h
 *******************************************************************************************************/
void multiSchedule_advParam_init(uint8 idx,uint16 param, uint8 len, void* pValue)
{
    multisch_adv_t* node =  NULL;
    node = multiRole_search_phead(MULTI_ADV_PARAM_MODE);
    uint16 idx_len = ( ( (uint16)idx) << 8 ) | len ;

//  LOG("%s node %p\n",__func__,node);
    if( node )
    {
        // find the index of the list , which have been created
        // TODO: shall verify the crossover creates scenarios for broadcast data and sacn response data
        for(uint8 i=0; i < multiRole_cacLen_list( MULTI_ADV_PARAM_MODE ); i++ )
        {
            if( idx == node->idx )
            {
                multiRole_insert_node(MULTI_ADV_PARAM_MODE,param,idx_len, pValue,(void**)&node);
                // case-1 : if idx matched , indicate  already set adv data , so that insert scan response , then return
                return;
            }

            node = node->next;
        }

        // case-2 : there's no idx matched in the list, so create the node
        node = multiRole_insertTail_node( MULTI_ADV_PARAM_MODE );
        multiRole_insert_node( MULTI_ADV_PARAM_MODE,param,idx_len, pValue, (void**)&node);
    }
    else
    {
        // return variable advparam is NULL indicate that there's no advertising parameter be configured
        if( SCH_SUCCESS == multiRole_create_list( MULTI_ADV_PARAM_MODE ) )
        {
            // first set , probably advertising data
            multiRole_insert_node( MULTI_ADV_PARAM_MODE,param,idx_len, pValue, (void**)&msAdv_param);
        }
    }
}

/*******************************************************************************************************
    @ Description    :  multi role advertising parameter delete , define in multi_schedule.h
 *******************************************************************************************************/
void multiSchedule_advParam_del( uint8 idx)
{
    multisch_adv_t* node =  NULL;
    node = multiRole_search_phead(MULTI_ADV_PARAM_MODE);

    for(uint8 i=0; i < multiRole_cacLen_list( MULTI_ADV_PARAM_MODE ); i++ )
    {
        if( idx == node->idx )
        {
            // matche the index that want to be delete
            if( node == multiRole_search_phead(MULTI_ADV_PARAM_MODE) )
            {
                // reset msAdv_param
                msAdv_param = node->next;
            }

            // memory free
            if( node->advParam.pAdvData )
                osal_mem_free( node->advParam.pAdvData );

            if( node->advParam.pScanRspData )
                osal_mem_free( node->advParam.pScanRspData );

            osal_mem_free( node );
            msAdv_param->next = NULL;
            break;
        }

        node = node->next;
    }
}

#endif

/*******************************************************************************************************
 *******************************************************************************************************
    @ Description    :  multi-role schedule scan
    @ Parameters     :
                   :  [IN]  None
                   :  [OUT]
    @ Return         :  None
    @ Other          :
    Modification History
    DATE        DESCRIPTION
    ------------------------------------------------------------------------------
    @author          :
 *******************************************************************************************************
 *******************************************************************************************************/
#if( MAX_CONNECTION_MASTER_NUM > 0 )
void multiConfigSchScan_param(void)
{
    multiScehdule_t* node = multiRole_findBusyNode();

    if( node->role == scanner )
    {
        node->busy = FALSE;
        node->roleScd.scan.scanning = FALSE;
    }
}


/*******************************************************************************************************
 *******************************************************************************************************
    @ Description    :  multi-role schedule advertising
    @ Parameters     :
                   :  [IN]  list: the adv list
                   :  [OUT]
    @ Return         :  None
    @ Other          :
    Modification History
    DATE        DESCRIPTION
    ------------------------------------------------------------------------------
    @author          :
 *******************************************************************************************************
 *******************************************************************************************************/
static void MultiScheduleScan(multiScehdule_t* node)
{
    uint8 ret = FALSE;
    uint8 param[3];
    multiGetScanStrategy(param,3);

    if( node->roleScd.scan.scanning == FALSE )
    {
        ret= GAPMultiRole_StartDiscovery( param[0],param[1],param[2] );
        LOG("start scan ret %d\n",ret);

        if( SUCCESS == ret )
        {
            node->busy = TRUE;
            node->roleScd.scan.scanning = TRUE;
        }
    }
    else
    {
        ret = GAPMultiRole_CancelDiscovery();

        if( SUCCESS == ret )
        {
            node->roleScd.scan.scanning = FALSE;
        }

        LOG("stop scan ret %d\n",ret);
    }
}

static void MultiScheduleInitating(multiScehdule_t* node)
{
    uint8 ret = FALSE;
    GAPMultiRoleCentralDev_t* lnode = multiGetSlaveConnList();
    uint8 param[2];
    multiGetLinkStrategy(param,2);

    if( lnode )
    {
        ret=  GAPMultiRole_EstablishLink(   param[0],
                                            param[1],
                                            lnode->addrType,
                                            lnode->addr );

        if( SUCCESS == ret )
        {
            node->busy = TRUE;
            node->roleScd.initiate.initiating = TRUE;
//          LOG( bdAddr2Str( lnode->addr ) );
            osal_start_timerEx(gapMultiRole_TaskID, CONN_TIMEOUT_EVT,node->nextScdTime);
            LOG("start establish success \n");
        }
        else
        {
            LOG("start establish failure ret %d\n",ret);
        }
    }
}

uint8 multiLinkGetMasterConnNum(void)
{
    uint8 len;
    GAPMultiRoleLinkCtrl_t* node = g_multiLinkInfo;

    while( node != NULL )
    {
        if( node->RoleState == Master_Role )
        {
            len++;
        }

        node = node->next;
    }

    return len;
}

#endif



/*******************************************************************************************************
 *******************************************************************************************************
    @ Description    :  multiConfigLink_status
    @ Parameters     :
                   :  [IN]
                   :  [OUT]
    @ Return         :  16bit value   bit0-bit3 connHandle
                                    bit4-bit7 role
                                    bit8-bit11 perIdx as slave
                                    bit12-bit15 RFU
    @ Other          :
    Modification History
    DATE        DESCRIPTION
    ------------------------------------------------------------------------------
    @author          :
 *******************************************************************************************************
 *******************************************************************************************************/
GAPMultiLinkInfo_t multiConfigLink_status(uint8 opcode,void* pkt)
{
    GAPMultiLinkInfo_t ret;
    ret.info = 0xFFFF;
    LOG("%s\n",__func__);

    switch (opcode)
    {
    case GAP_LINK_ESTABLISHED_EVENT:
    {
        // 1.create link list node
        gapEstLinkReqEvent_t* pPkt = (gapEstLinkReqEvent_t*)pkt;
        GAPMultiRoleLinkCtrl_t* node = NULL;
        node = osal_mem_alloc(sizeof(GAPMultiRoleLinkCtrl_t));
        osal_memset(node,0,sizeof(GAPMultiRoleLinkCtrl_t));
        node->connectionHandle = pPkt->connectionHandle;
        node->connInterval = pPkt->connInterval;
        node->connLatency = pPkt->connLatency;
        node->connTimeout = pPkt->connTimeout;
        node->peerDevAddrType = pPkt->devAddrType;
        osal_memcpy(node->peerDevAddr, pPkt->devAddr,B_ADDR_LEN);
        multiScehdule_t* bNode = multiRole_findBusyNode();
        ret.value.connHandle = pPkt->connectionHandle;

        if( bNode->role == advertiser )
        {
            uint8 idx = bNode->roleScd.adv.perIdx;
            node->connectionHandle |= (((uint16)idx )<< 12);
            LOG("bNode->roleScd.adv.perIdx %d,node->connectionHandle 0x%x\n",bNode->roleScd.adv.perIdx,node->connectionHandle);
            node->RoleState = Slave_Role;
//              ret.value.perIdx = ( ((uint16)bNode->roleScd.adv.perIdx) << 8 );
            ret.value.perIdx = bNode->roleScd.adv.perIdx;
        }
        else
            node->RoleState = Master_Role;

        ret.value.role = node->RoleState;
        multiRole_insert_node( MULTI_LINK_MODE,NULL,NULL, NULL, (void**)&node);
        // 2. delete the busy node
        multiRole_delete_node( MULTI_SCH_MODE,NULL,(void**)&bNode  );
    }
    break;

    case GAP_LINK_TERMINATED_EVENT:
    {
        gapTerminateLinkEvent_t* pPkt = (gapTerminateLinkEvent_t*)pkt;
        GAPMultiRoleLinkCtrl_t* tNode = multiRole_search_phead( MULTI_LINK_MODE );

        for(uint8 i=0; i<multiRole_cacLen_list(MULTI_LINK_MODE); i++)
        {
            // priority & > ==
            if( (tNode->connectionHandle & 0x0EFF) == pPkt->connectionHandle )
            {
                ret.value.connHandle = pPkt->connectionHandle;
                ret.value.role = tNode->RoleState;
                // perIdx
                ret.value.perIdx = (uint16)( (tNode->connectionHandle & 0xF000 ) >> 12 );
                break;
            }

            tNode = tNode->next;
        }

        #if( MAX_CONNECTION_MASTER_NUM > 0 )

        if( tNode->RoleState == Master_Role )
        {
            multiAddSlaveConnList( tNode->peerDevAddrType,tNode->peerDevAddr );
        }

        #endif
        multiRole_delete_node( MULTI_LINK_MODE,NULL,(void**)&tNode  );
    }
    break;
    }

//  LOG("%s,opcode %d,ret 0x%x\n",__func__,opcode,ret.value);
    return ret;
}




void* lsearch(void* key,void* base,int n,int elemSize,int(*cmpfn)(void*,void*,int))
{
    for(int i = 0; i < n; ++i)
    {
        void* elemAddr = (char*)base+i*elemSize;

        if(cmpfn(key,elemAddr,elemSize) == 0)
            return elemAddr;
    }

    return NULL;
}


/*******************************************************************************************************
    @ Description    :  multi role scheduler initilization , define in multi_schedule.h
 *******************************************************************************************************/
void multiSchedule_init(uint8 taskid)
{
    sch_tsk_id = taskid;
}




