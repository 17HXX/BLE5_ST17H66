/**************************************************************************************************
*******
**************************************************************************************************/

/*********************************************************************
    INCLUDES
*/

#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "OSAL_bufmgr.h"
#include "gatt.h"
#include "ll.h"
#include "ll_common.h"
#include "hci.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "central.h"
#include "gapbondmgr.h"
#include "simpleGATTprofile_ota.h"
#include "simpleBLECentral.h"
#include "timer.h"
#include "log.h"
#include "ll_def.h"
#include "global_config.h"
#include "flash.h"
#include "rflib.h"
#include "clock.h"

#define Central_Test_ToDoList               6
#define RWN_TEST_VAL_LEN            20

/**************************************************************************
    TYPEDEFS
*/
typedef struct
{
    uint32 toDoTick;
    uint16 toDoEvt;
} ctvToDoList_t;

typedef struct
{
    uint8 mtu;
    uint8 pduLen;
    uint8 pduLenSla;
    uint8 phyMode;
    uint8 phyModeSla;
    uint8 connIntv;
    uint8 latency;
    uint8 notfIntv;
    uint8 notfPktNum;
    uint32 testCnt;
    ctvToDoList_t ctvToDoList[Central_Test_ToDoList];

} centralTestVector_t;


typedef struct
{
    uint32 cnt;
    uint32 miss;
    uint32 err;
    uint32 isDone;
} ntfTest_t;


/**************************************************************************
    VARIABLES
*/
enum
{
    NULL_TO_DO = 0x00,
    CHAN_MAP   = 0x01,
    SLA_1M,
    SLA_2M,
    SLA_ANY,
    MST_1M,
    MST_2M,
    MST_ANY,
    SLA_HCLK48_32KRC,
    SLA_HCLK48_32KXTAL,
    SLA_HCLK16_32KRC,
    SLA_HCLK16_32KXTAL,
    SLA_HCLK64_32KRC,
    SLA_HCLK64_32KXTAL,
    SLA_CHECK_PER
};

enum
{
    BLE_STATE_IDLE,
    BLE_STATE_CONNECTING,
    BLE_STATE_CONNECTED,
    BLE_STATE_DISCONNECTING
};

static centralTestVector_t centralTestVectorTbl[]=
{
//mtu,  dleM,  dleS, phyM,  phyS, connIntv,latency,  notfIntv,  notfPkt,testTime,   toDoList0,      toDoList1,      toDoList2,      toDoList3,      toDoList4,
//{ 23,   23,    23,    1,      1,      10,     0,    0x80,     5,      1000,       100,CHAN_MAP,   200,SLA_2M, 300,SLA_1M, 500,SLA_ANY,    980,SLA_CHECK_PER},
//{ 23,   23,    23,    2,      2,      10,    10,    0x70,     3,      1000,       100,CHAN_MAP,   200,SLA_2M, 300,SLA_1M, 500,SLA_ANY,    980,SLA_CHECK_PER},
//{ 247,  23,    23,    1,      1,      10,     0,    0x70,     1,      1000,       100,CHAN_MAP,   200,SLA_2M, 300,SLA_1M, 500,SLA_ANY,    980,SLA_CHECK_PER},
//{ 247,  23,    23,    2,      2,      10,     0,    0x70,     1,      1000,       100,CHAN_MAP,   200,SLA_2M, 300,SLA_1M, 500,SLA_ANY,    980,SLA_CHECK_PER},
    { 247,  247,   247,     1,      1,      10,     0,    0x80,     3,      1000,       100,CHAN_MAP,   200,SLA_2M, 300,SLA_1M, 500,SLA_ANY,    980,SLA_CHECK_PER},
    { 247,  247,   247,     2,      2,      10,     0,    0x80,     3,      1000,       100,CHAN_MAP,   200,SLA_2M, 300,SLA_1M, 500,SLA_ANY,    980,SLA_CHECK_PER},
};

static uint16 cTVIdx=0;
static uint16 cTVErrCnt = 0;
static uint16 mstWtCnt=0;
uint8 rwnTestCnt =0;
ntfTest_t ntfTest;

/*********************************************************************
    EXTERNAL VARIABLES
*/
extern uint8 rwnTestVal[RWN_TEST_VAL_LEN];
extern l2capSARDbugCnt_t g_sarDbgCnt;
/*********************************************************************
    EXTERNAL FUNCTIONS
*/
extern uint32 osal_memory_statics(void);

/*********************************************************************
    LOCAL FUNCTIONS
*/
static void simpleBLECentral_ReadWriteNotifyTest(uint8 TaskId,uint16 ConnHandle)
{
    attWriteReq_t* pReq;
    attReadReq_t* pReqread;

    if(rwnTestCnt==0)
    {
        pReqread = osal_mem_alloc(sizeof(attReadReq_t));
        pReqread->handle = 0x20;
        bStatus_t status = GATT_ReadCharValue( ConnHandle, pReqread, TaskId );
        osal_mem_free(pReqread);
    }
    else if(rwnTestCnt==1)
    {
        pReq = osal_mem_alloc(sizeof(attWriteReq_t));
        pReq->sig = 0;
        pReq->cmd = TRUE;
        pReq->handle = 0x20;
        pReq->len = 5;
        rwnTestVal[0]=0x01;
        rwnTestVal[1]=centralTestVectorTbl[cTVIdx].connIntv;
        rwnTestVal[2]=centralTestVectorTbl[cTVIdx].connIntv;
        rwnTestVal[3]=centralTestVectorTbl[cTVIdx].latency;
        rwnTestVal[4]=0x05;
        osal_memcpy(pReq->value, rwnTestVal, pReq->len );
        bStatus_t status = GATT_WriteNoRsp(ConnHandle, pReq);
        osal_mem_free(pReq);
    }
    else if(rwnTestCnt==2)
    {
        pReqread = osal_mem_alloc(sizeof(attReadReq_t));
        pReqread->handle = 0x20;
        bStatus_t status = GATT_ReadCharValue( ConnHandle, pReqread, TaskId);
        osal_mem_free(pReqread);
    }
    else if(rwnTestCnt==3)
    {
        pReq = osal_mem_alloc(sizeof(attWriteReq_t));
        pReq->sig = 0;
        pReq->cmd = TRUE;
        pReq->handle = 0x20;
        pReq->len = 5;
        rwnTestVal[0]=0x00;
        rwnTestVal[1]=centralTestVectorTbl[cTVIdx].notfIntv;
        rwnTestVal[2]=centralTestVectorTbl[cTVIdx].notfPktNum;
        rwnTestVal[3]=0x00;
        rwnTestVal[4]=0x00;
        osal_memcpy(pReq->value, rwnTestVal, pReq->len );
        bStatus_t status = GATT_WriteNoRsp(ConnHandle, pReq);
        osal_mem_free(pReq);
    }
    else if(rwnTestCnt==4)
    {
        pReq = osal_mem_alloc(sizeof(attWriteReq_t));
        pReq->sig = 0;
        pReq->cmd = TRUE;
        pReq->handle = 0x24;
        pReq->len = 2;
        pReq->value[0] = (unsigned char)(GATT_CLIENT_CFG_NOTIFY);
        pReq->value[1] = (unsigned char)(GATT_CLIENT_CFG_NOTIFY >> 8);
        bStatus_t status = GATT_WriteNoRsp(ConnHandle, pReq);
        osal_mem_free(pReq);
    }

    if(rwnTestCnt<4)
    {
        rwnTestCnt++;
        osal_start_timerEx( TaskId, SBC_READ_WRITE_TEST_EVT, 2000 );
    }
}


void centralTestVectorProcess(uint8 TaskId,uint8 phyModeSlave,uint8 phyMode,uint8 slaCmd,uint8 BLEState,uint16 ConnHandle)
{
    uint32 uOS_size=0;

    for(int i=0; i<Central_Test_ToDoList; i++)
    {
        if(     centralTestVectorTbl[cTVIdx].ctvToDoList[i].toDoTick>0
                &&   centralTestVectorTbl[cTVIdx].ctvToDoList[i].toDoTick==mstWtCnt
                &&   centralTestVectorTbl[cTVIdx].ctvToDoList[i].toDoEvt!=NULL_TO_DO)
        {
            slaCmd=centralTestVectorTbl[cTVIdx].ctvToDoList[i].toDoEvt;

            switch ( centralTestVectorTbl[cTVIdx].ctvToDoList[i].toDoEvt )
            {
            case CHAN_MAP:
                osal_set_event(TaskId, UPD_CHAN_MAP_EVENT);
                break;

            case SLA_1M:
                phyModeSlave=0x01;
                osal_set_event(TaskId, SLA_PHY_MODE_EVT);
                break;

            case SLA_2M:
                phyModeSlave=0x02;
                osal_set_event(TaskId, SLA_PHY_MODE_EVT);
                break;

            case SLA_ANY:
                phyModeSlave=0x03;
                osal_set_event(TaskId, SLA_PHY_MODE_EVT);
                break;

            case SLA_HCLK16_32KRC:
            case SLA_HCLK16_32KXTAL:
            case SLA_HCLK48_32KRC:
            case SLA_HCLK48_32KXTAL:
            case SLA_HCLK64_32KRC:
            case SLA_HCLK64_32KXTAL:
            case SLA_CHECK_PER:
                osal_set_event(TaskId, SLA_PHY_MODE_EVT);
                break;

            case MST_1M:
                phyMode=0x01;
                osal_set_event(TaskId, UPD_PHY_MODE_EVT);
                break;

            case MST_2M:
                phyMode=0x02;
                osal_set_event(TaskId, UPD_PHY_MODE_EVT);
                break;

            case MST_ANY:
                phyMode=0x03;
                osal_set_event(TaskId, UPD_PHY_MODE_EVT);
                break;
            }

            uOS_size=osal_memory_statics();
            AT_LOG("[TVEC] %d toDoList %d %4x  [uOS %08d] \n",i,centralTestVectorTbl[cTVIdx].ctvToDoList[i].toDoTick,
                   centralTestVectorTbl[cTVIdx].ctvToDoList[i].toDoEvt,
                   uOS_size);
        }
    }

    if(mstWtCnt >=centralTestVectorTbl[cTVIdx].testCnt &&
            ntfTest.isDone==0)
    {
        uOS_size=osal_memory_statics();
        HCI_PPLUS_ConnEventDoneNoticeCmd(TaskId, NULL);
        AT_LOG("[TVEC] %08x case %3d/%d[cnt %6d miss %2d err %2d] [uOS %08d]\n",getMcuPrecisionCount()
               ,cTVIdx
               ,sizeof(centralTestVectorTbl)/sizeof(centralTestVector_t)
               ,ntfTest.cnt,ntfTest.miss,ntfTest.err
               ,uOS_size);

        if(ntfTest.miss>0 || ntfTest.err>0)
            cTVErrCnt++;

        AT_LOG ("[TVEC] mAloc[r%4x s%4x] SAR[r%4x %4x %4x %4x s%4x %4x %4x %4x ]\n"
                ,g_sarDbgCnt.resssambleMemAlocErr
                ,g_sarDbgCnt.segmentMemAlocErr
                ,g_sarDbgCnt.reassembleInCnt
                ,g_sarDbgCnt.reassembleOutCnt
                ,g_sarDbgCnt.reassembleErrInComp
                ,g_sarDbgCnt.reassembleErrIdx
                ,g_sarDbgCnt.segmentInCnt
                ,g_sarDbgCnt.segmentOutCnt
                ,g_sarDbgCnt.segmentErrCnt
                ,g_sarDbgCnt.segmentSentToLinkLayerErr);
        ntfTest.cnt=0;
        ntfTest.miss=0;
        ntfTest.err=0;
        ntfTest.isDone=1;
        osal_memset(&g_sarDbgCnt,0,sizeof(g_sarDbgCnt));
        cTVIdx++;

        if(cTVIdx==(sizeof(centralTestVectorTbl)/sizeof(centralTestVector_t)))
        {
            AT_LOG("[TVEC] %08x >>>>>>>>> ALL CASE FINISHED Total %d Err %d <<<<<<<<<\n",getMcuPrecisionCount(),cTVIdx,cTVErrCnt);
            cTVIdx=0;
//            while(1){};
        }

        if (BLEState == BLE_STATE_CONNECTED)   // not connected
        {
            GAPCentralRole_TerminateLink( ConnHandle);
        }

        AT_LOG("Terminated Link .s%d d%d\r\n",BLEState,ConnHandle);
    }

    return;
}
