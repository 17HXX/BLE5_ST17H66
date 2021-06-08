/**************************************************************************************************
*******
**************************************************************************************************/

/**************************************************************************************************
    Filename:       ota_if.c
    Revised:        $Date $
    Revision:       $Revision $

    OTA for internal flash

**************************************************************************************************/

/*********************************************************************
    INCLUDES
*/

#include "bcomdef.h"
#include "OSAL.h"
#include "ll_common.h"
#include "linkdb.h"
#include "gatt.h"
#include "gap.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "gatt_profile_uuid.h"
#include "devinfoservice.h"
#include "peripheral.h"
#include "ota.h"
#include "ota_protocol.h"
#include "log.h"
#include "ll_hw_drv.h"


// #define DEF_ADD_DEVINFO                       TRUE

// Fast advertising interval in 625us units
#define DEFAULT_FAST_ADV_INTERVAL             32

// Duration of fast advertising duration in ms
#define DEFAULT_FAST_ADV_DURATION             0

// Slow advertising interval in 625us units
#define DEFAULT_SLOW_ADV_INTERVAL             32

// Duration of slow advertising duration in ms (set to 0 for continuous advertising)
#define DEFAULT_SLOW_ADV_DURATION             0


// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         TRUE

// Minimum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     6*2//20//100//6//20

// Maximum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     6*2//30//200//6//30

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          500


#define DEFAULT_RXC_FLOW_CTRL_NUM              10
/*********************************************************************
    TYPEDEFS
*/

/*********************************************************************
    GLOBAL VARIABLES
*/

/*********************************************************************
    EXTERNAL VARIABLES
*/
extern uint8       g_largeHeap[];
/*********************************************************************
    EXTERNAL FUNCTIONS
*/

bStatus_t GAPBondMgr_LinkEst(uint8 addrType, uint8* pDevAddr, uint16 connHandle, uint8 role)
{
    return 0;
}

/*********************************************************************
    LOCAL VARIABLES
*/
uint8 ota_TaskID;   // Task ID for internal task/event processing

static gaprole_States_t gapProfileState = GAPROLE_INIT;

static uint8 advertData[] =
{
    // flags
    0x02,
    GAP_ADTYPE_FLAGS,
    GAP_ADTYPE_FLAGS_GENERAL | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

    9, //name
    GAP_ADTYPE_LOCAL_NAME_COMPLETE,
    'P',
    'P',
    'l',
    'u',
    's',
    'O',
    'T',
    'A',

    11, //Manufacture data
    GAP_ADTYPE_MANUFACTURER_SPECIFIC,
    04,
    05,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
};

// Device name attribute value
static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "PPlusOTA";

// GAP connection handle
static uint16 gapConnHandle;

/*********************************************************************
    LOCAL FUNCTIONS
*/
static void ota_ProcessOSALMsg(osal_event_hdr_t* pMsg);
static void ota_GapStateCB(gaprole_States_t newState);

/*********************************************************************
    PROFILE CALLBACKS
*/

// GAP Role Callbacks
static gapRolesCBs_t ota_PeripheralCB =
{
    ota_GapStateCB, // Profile State Change Callbacks
    NULL            // When a valid RSSI is read from controller
};

uint16 otaApp_ProcessEvent(uint8 task_id, uint16 events)
{
    VOID task_id; // OSAL required parameter that isn't used in this function
    //LOG("otaApp_ProcessEvent: 0x%x\n",events);

    if (events & SYS_EVENT_MSG)
    {
        uint8* pMsg;

        if ((pMsg = osal_msg_receive(ota_TaskID)) != NULL)
        {
            ota_ProcessOSALMsg((osal_event_hdr_t*)pMsg);
            // Release the OSAL message
            VOID osal_msg_deallocate(pMsg);
        }

        // return unprocessed events
        return (events ^ SYS_EVENT_MSG);
    }

    if (events & START_DEVICE_EVT)
    {
        // Start the Device
        VOID GAPRole_StartDevice(&ota_PeripheralCB);
        // Register with bond manager after starting device
        //GAPBondMgr_Register( (gapBondCBs_t *) &ota_BondCB );
        return (events ^ START_DEVICE_EVT);
    }

    if (events & OTA_TIMER_EVT)
    {
        otaProtocol_TimerEvt();
        return (events ^ OTA_TIMER_EVT);
    }

    // Discard unknown events
    return 0;
}
static void ota_flow_ctrl(uint16 mtu)
{
    ll_pdu_length_ctrl_t pdu;
    extern void LL_PLUS_GetCurrentPduDle(uint8_t connId, ll_pdu_length_ctrl_t* ppdu);
    LL_PLUS_GetCurrentPduDle(/*connId*/0,&pdu);
    LOG("LE CHECK tx %02x rx %02x\n",pdu.MaxTxOctets,pdu.MaxRxOctets);
    //RX_FLOW_CTRL

    if(pdu.MaxRxOctets<mtu)
    {
        GATT_SetHostToAppFlowCtrl(8*1024,1);
        extern uint16 numHostBufs;
        extern uint8 ctrlToHostEnable;
        LOG("HCI HOST Buffsize Set en %d buf %d\n",ctrlToHostEnable,numHostBufs);
    }
}
static void on_gatt_mtu_req(attExchangeMTUReq_t* preq)
{
    uint16_t mtu = preq->clientRxMTU;

    if (mtu > g_ATT_MTU_SIZE_MAX)
        mtu = g_ATT_MTU_SIZE_MAX;

    otaProtocol_mtu(mtu);
    LOG("MTU size req: %d | set: %d\n", preq->clientRxMTU, mtu);
    ota_flow_ctrl(mtu);
}

static void ota_ProcessGATTMsg(gattMsgEvent_t* pMsg)
{
    //LOG("[GATT] %x\n",pMsg->method);
    if (pMsg->hdr.status == bleTimeout)
    {
        LOG("[GATT TO] %x\n", pMsg->method);
        return;
    }

    switch (pMsg->method)
    {
    case ATT_EXCHANGE_MTU_REQ:
        on_gatt_mtu_req(&(pMsg->msg.exchangeMTUReq));
        break;

    default:
        break;
    }
}

static void ota_ProcessOSALMsg(osal_event_hdr_t* pMsg)
{
}

static void ota_GapStateCB(gaprole_States_t newState)
{
    LOG("ota_GapStateCB: %d\n", newState);

    // if connected
    if (newState == GAPROLE_CONNECTED)
    {
        // get connection handle
        GAPRole_GetParameter(GAPROLE_CONNHANDLE, &gapConnHandle);
    }
    // if disconnected
    else if (gapProfileState == GAPROLE_CONNECTED &&
             newState != GAPROLE_CONNECTED)
    {
        uint8 advState = TRUE;
        // link loss timeout-- use fast advertising
        GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, DEFAULT_FAST_ADV_INTERVAL);
        GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, DEFAULT_FAST_ADV_INTERVAL);
        GAP_SetParamValue(TGAP_GEN_DISC_ADV_MIN, DEFAULT_FAST_ADV_DURATION);
        //}
        //else
        //{
        // Else use slow advertising
        //  GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, DEFAULT_SLOW_ADV_INTERVAL );
        //  GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, DEFAULT_SLOW_ADV_INTERVAL );
        //  GAP_SetParamValue( TGAP_GEN_DISC_ADV_MIN, DEFAULT_SLOW_ADV_DURATION );
        //}
        // Enable advertising
        GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8), &advState);
        //Trun on flow ctrl
        GATT_SetHostToAppFlowCtrl( 0,0 );
    }
    // if advertising stopped
    else if (gapProfileState == GAPROLE_ADVERTISING &&
             newState == GAPROLE_WAITING)
    {
        // if fast advertising switch to slow
        if (GAP_GetParamValue(TGAP_GEN_DISC_ADV_INT_MIN) == DEFAULT_FAST_ADV_INTERVAL)
        {
            uint8 advState = TRUE;
            GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, DEFAULT_SLOW_ADV_INTERVAL);
            GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, DEFAULT_SLOW_ADV_INTERVAL);
            GAP_SetParamValue(TGAP_GEN_DISC_ADV_MIN, DEFAULT_SLOW_ADV_DURATION);
            GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8), &advState);
        }
    }

    // if started
    #if(DEF_ADD_DEVINFO)
    else if (newState == GAPROLE_STARTED)
    {
        // Set the system ID from the bd addr
        uint8 systemId[DEVINFO_SYSTEM_ID_LEN];
        GAPRole_GetParameter(GAPROLE_BD_ADDR, systemId);
        // shift three bytes up
        systemId[7] = systemId[5];
        systemId[6] = systemId[4];
        systemId[5] = systemId[3];
        // set middle bytes to zero
        systemId[4] = 0;
        systemId[3] = 0;
        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);
    }

    #endif
    gapProfileState = newState;
}

void otaApp_Init(uint8 task_id)
{
    ota_TaskID = task_id;
    volatile uint8_t* pubaddr = (volatile uint8_t*)0x1fff0965;

    if (CFG_MTU_23)
    {
        ATT_SetMTUSizeMax(23);
    }

    // Setup the GAP Peripheral Role Profile
    {
        uint8 initial_advertising_enable = TRUE;
        uint8 advChnMap = GAP_ADVCHAN_37 | GAP_ADVCHAN_38 | GAP_ADVCHAN_39;
        // By setting this to zero, the device will go into the waiting state after
        // being discoverable for 30.72 second, and will not being advertising again
        // until the enabler is set back to TRUE
        uint16 gapRole_AdvertOffTime = 0;
        uint8 enable_update_request = DEFAULT_ENABLE_UPDATE_REQUEST;
        uint16 desired_min_interval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
        uint16 desired_max_interval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
        uint16 desired_slave_latency = DEFAULT_DESIRED_SLAVE_LATENCY;
        uint16 desired_conn_timeout = DEFAULT_DESIRED_CONN_TIMEOUT;
        // set adv channel map
        GAPRole_SetParameter(GAPROLE_ADV_CHANNEL_MAP, sizeof(uint8), &advChnMap);
        // Set the GAP Role Parameters
        GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8), &initial_advertising_enable);
        GAPRole_SetParameter(GAPROLE_ADVERT_OFF_TIME, sizeof(uint16), &gapRole_AdvertOffTime);

        //update public address
        if (otaProtocol_address_plus())
        {
            pubaddr[0]++;
        }

        //set macaddress to advertising data
        {
            uint8_t* pmac_field = advertData + sizeof(advertData) - 8;
            osal_memcpy(pmac_field, (const void*)pubaddr, B_ADDR_LEN);
        }
        //GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanData ), scanData );
        GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData);
        GAPRole_SetParameter(GAPROLE_PARAM_UPDATE_ENABLE, sizeof(uint8), &enable_update_request);
        GAPRole_SetParameter(GAPROLE_MIN_CONN_INTERVAL, sizeof(uint16), &desired_min_interval);
        GAPRole_SetParameter(GAPROLE_MAX_CONN_INTERVAL, sizeof(uint16), &desired_max_interval);
        GAPRole_SetParameter(GAPROLE_SLAVE_LATENCY, sizeof(uint16), &desired_slave_latency);
        GAPRole_SetParameter(GAPROLE_TIMEOUT_MULTIPLIER, sizeof(uint16), &desired_conn_timeout);
    }
    // Set the GAP Characteristics
    GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);
    GAP_SetParamValue(TGAP_CONN_PAUSE_PERIPHERAL, 1);
    // Set advertising interval
    {
        uint16 advInt = 400; // actual time = advInt * 625us
        GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, advInt);
        GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, advInt);
        GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, advInt);
        GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, advInt);
    }
    // Initialize GATT attributes
    GGS_AddService(GATT_ALL_SERVICES);         // GAP
    GATTServApp_AddService(GATT_ALL_SERVICES); // GATT attributes
    #if(DEF_ADD_DEVINFO)
    DevInfo_AddService();
    #endif
    otaProtocol_init(ota_TaskID, OTA_TIMER_EVT);
    gattServApp_RegisterCB(ota_ProcessGATTMsg);
    //upload mtu config
    ATT_SetMTUSizeMax(240);
    ll_hw_set_tfifo_space(LL_HW_FIFO_TX_1K_RX_3K);
    llInitFeatureSet2MPHY(TRUE);
    llInitFeatureSetDLE(TRUE);
    // Setup a delayed profile startup
    osal_set_event(ota_TaskID, START_DEVICE_EVT);
}

/*********************************************************************
*********************************************************************/
