/**************************************************************************************************
THIS IS EMPTY HEADER
**************************************************************************************************/

/**************************************************************************************************
  Filename:       simpleBLEPeripheral.c
  Revised:        
  Revision:       

  Description:    This file contains the Simple BLE Peripheral sample application
                  

**************************************************************************************************/
/*********************************************************************
 * INCLUDES
 */
#include "bcomdef.h"
#include "rf_phy_driver.h"
#include "global_config.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "gatt.h"
#include "hci.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"
#include "sbpProfile_ota.h"
#include "ota_app_service.h"
#include "peripheral.h"
#include "gapbondmgr.h"

#include "simpleBLEPeripheral.h"
#include "ll.h"
#include "ll_hw_drv.h"
#include "ll_def.h"
#include "hci_tl.h"
#include "fs.h"
#include "osal_snv.h"
/*********************************************************************
 * MACROS
 */
//#define LOG(...)  
/*********************************************************************
 * CONSTANTS
 */

// How often to perform periodic event
#define SBP_PERIODIC_EVT_PERIOD					5000

#define DEVINFO_SYSTEM_ID_LEN						8
#define DEVINFO_SYSTEM_ID							0
 

#define DEFAULT_DISCOVERABLE_MODE					GAP_ADTYPE_FLAGS_GENERAL

// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL		24//32//80

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL		800//48//800

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY				0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT				500//1000

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST				TRUE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL				6

#define INVALID_CONNHANDLE							0xFFFF

// Default passcode
#define DEFAULT_PASSCODE							0//19655

// Length of bd addr as a string
#define B_ADDR_STR_LEN								15




/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
perStatsByChan_t g_perStatsByChanTest;

/*********************************************************************
 * EXTERNAL VARIABLES
 */
volatile uint8_t g_current_advType = LL_ADV_CONNECTABLE_UNDIRECTED_EVT;

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */


/*********************************************************************
 * LOCAL VARIABLES
 */
static uint8 simpleBLEPeripheral_TaskID;   // Task ID for internal task/event processing
static gaprole_States_t gapProfileState	=	GAPROLE_INIT;
//	GAP - SCAN RSP data (max size = 31 bytes)
static uint8 scanRspData[] =
{
    // complete name
    0x09,   // length of this data
    GAP_ADTYPE_LOCAL_NAME_COMPLETE,
    'U', 'I', 'N', 'L', 'A', 'N', ' ', ' ',
};
//	index start from 0
#define BEACON_ADV_UUID_INDEX		9
#define BEACON_ADV_MAJOR_INDEX	25
#define BEACON_ADV_MINOR_INDEX	27
#define BEACON_ADV_RSSI_INDEX		29
//	advert data for iBeacon
static uint8 advertData[] =
{	
    0x02,   // length of this data
    GAP_ADTYPE_FLAGS,
    DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,
    0x1A, // length of this data including the data type byte
    GAP_ADTYPE_MANUFACTURER_SPECIFIC, // manufacturer specific adv data type
    0x4c, // Company ID - Fixed
    0x00, // Company ID - Fixed
    0x02, // Data Type - Fixed
    0x15, // Data Length - Fixed
    0x7d, 0xb8, 0x60, 0xed, 0xb6, 0x4d, 0x4b, 0xb1, //uuid
    0x98, 0x75, 0x8f, 0x16, 0x35, 0x5a, 0x97, 0xd2,  //uuid

    0x00,0x07, // Major
    0x02,0x55,//0xb0, // Minor
    0xc5 // Power - The 2's complement of the calibrated Tx Power
};

ibeacon_store_data_t BeacondefualData={
	{'U', 'I', 'N', 'L', 'A', 'N', ' ', ' '},			//	dev name

	{0x7d, 0xb8, 0x60, 0xed, 0xb6, 0x4d, 0x4b, 0xb1, 	//	uuid
		0x98, 0x75, 0x8f, 0x16, 0x35, 0x5a, 0x97, 0xd2},
	{0x00,0x07},	//major
	{0x02,0x55},	//minor
	500,  			//adv default interval 800*0.625=500ms
	0xC5,			//RSSI
	0xff			//
};


#define BEACON_STOREDATA_FS_ID		0x88
#define BEACON_STORE_FS_FLAG_ID		0x89
#define BEACON_STORE_FS_FLAG_DATA		0x66 
uint16 gapConnHandle;


// GAP GATT Attributes
static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "UINLAN  ";
/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void simpleBLEPeripheral_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void peripheralStateNotificationCB( gaprole_States_t newState );
static void simpleProfileChangeCB( uint8 paramID );
static void updateAdvData(void);
static void peripheralStateReadRssiCB( int8 rssi  );
char *bdAddr2Str( uint8 *pAddr );

// GAP Role Callbacks
static gapRolesCBs_t simpleBLEPeripheral_PeripheralCBs =
{
    peripheralStateNotificationCB,  // Profile State Change Callbacks
    peripheralStateReadRssiCB       // When a valid RSSI is read from controller (not used by application)
};

// Simple GATT Profile Callbacks
static simpleProfileCBs_t simpleBLEPeripheral_SimpleProfileCBs =
{
    simpleProfileChangeCB    // Charactersitic value change callback
};

void SimpleBLEPeripheral_SetDevName(uint8*data,uint8 len)
{
	uint8 dev_name[IBEACON_DEV_NAME_MAX_LEN]={0};	
	osal_memcpy(dev_name,data,len);
	osal_memcpy(Ibeacon_store_data.beacon_dev_name,dev_name,IBEACON_DEV_NAME_MAX_LEN);
	osal_memcpy(&scanRspData[2],dev_name,IBEACON_DEV_NAME_MAX_LEN);	
	osal_snv_write(BEACON_STOREDATA_FS_ID,sizeof(ibeacon_store_data_t)/sizeof(uint8),&Ibeacon_store_data);	
}	

void SimpleBLEPeripheral_SetBeaconUUID(uint8* data,uint8 len)
{
	osal_memcpy(Ibeacon_store_data.uuid,data,len);
	osal_memcpy(&advertData[BEACON_ADV_UUID_INDEX],data,len);
//	GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanRspData ), scanRspData );
	GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );
	osal_snv_write(BEACON_STOREDATA_FS_ID,sizeof(ibeacon_store_data_t)/sizeof(uint8),&Ibeacon_store_data);
}	

void SimpleBLEPeripheral_SetMajor(uint8* data,uint8 len)
{
	osal_memcpy(&Ibeacon_store_data.major,data,len);
	osal_memcpy(&advertData[BEACON_ADV_MAJOR_INDEX],data,len);
	GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );
	osal_snv_write(BEACON_STOREDATA_FS_ID,sizeof(ibeacon_store_data_t)/sizeof(uint8),&Ibeacon_store_data);
}
void SimpleBLEPeripheral_SetMinor(uint8* data,uint8 len)
{
	osal_memcpy(&Ibeacon_store_data.minor,data,len);
	osal_memcpy(&advertData[BEACON_ADV_MINOR_INDEX],data,len);
	GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );
	osal_snv_write(BEACON_STOREDATA_FS_ID,sizeof(ibeacon_store_data_t)/sizeof(uint8),&Ibeacon_store_data);
}

void SimpleBLEPeripheral_SetRSSI(uint8 data)
{
	uint8 temp;
	Ibeacon_store_data.RSSI=data;
	advertData[BEACON_ADV_RSSI_INDEX]=data;
	GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );
	osal_snv_write(BEACON_STOREDATA_FS_ID,sizeof(ibeacon_store_data_t)/sizeof(uint8),&Ibeacon_store_data);
}

void SimpleBLEPeripheral_SetAdvIntvlTime(uint16 data)
{
	uint32 tmp=0;
	uint16 advInt = 800;//2400;//1600;//1600;//800;//1600;   // actual time = advInt * 625us
	tmp= (data*1000)/625;
	advInt=(uint16)tmp;
	Ibeacon_store_data.advIntvl=data;
	GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt );
	GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt );
	GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
	GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );

	osal_snv_write(BEACON_STOREDATA_FS_ID,sizeof(ibeacon_store_data_t)/sizeof(uint8),&Ibeacon_store_data);	
}

void simpleProfile_Set_Notify_Event(void)
{
	osal_set_event(simpleBLEPeripheral_TaskID,SBP_NOTIFY_EVT);
}	

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SimpleBLEPeripheral_Init
 *
 * @brief   Initialization function for the Simple BLE Peripheral App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void SimpleBLEPeripheral_Init( uint8 task_id )
{
	uint16 fs_read_len=0;
	int snv_ret=0;
	uint8 fs_flag;
	uint16 adv_intvl=0;
	simpleBLEPeripheral_TaskID = task_id;	

	//read configure parameters	
	{	
		osal_snv_read(BEACON_STORE_FS_FLAG_ID,1,&fs_flag);
		
		if(fs_flag != BEACON_STORE_FS_FLAG_DATA)
		{
			fs_flag=BEACON_STORE_FS_FLAG_DATA;
			osal_snv_write(BEACON_STORE_FS_FLAG_ID,1,&fs_flag);		
			osal_snv_write(BEACON_STOREDATA_FS_ID,sizeof(ibeacon_store_data_t)/sizeof(uint8),&BeacondefualData);	    		
		}	
		osal_snv_read(BEACON_STOREDATA_FS_ID,sizeof(ibeacon_store_data_t)/sizeof(uint8),&Ibeacon_store_data);
		osal_memcpy(attDeviceName,Ibeacon_store_data.beacon_dev_name,IBEACON_DEV_NAME_MAX_LEN);
		osal_memcpy(&scanRspData[2],Ibeacon_store_data.beacon_dev_name,IBEACON_DEV_NAME_MAX_LEN);
		osal_memcpy(&advertData[BEACON_ADV_UUID_INDEX],Ibeacon_store_data.uuid,IBEACON_UUID_LEN);
		osal_memcpy(&advertData[BEACON_ADV_MAJOR_INDEX],Ibeacon_store_data.major,IBEACON_VERSION_LEN);
		osal_memcpy(&advertData[BEACON_ADV_MINOR_INDEX],Ibeacon_store_data.minor,IBEACON_VERSION_LEN);
		osal_memcpy(&advertData[BEACON_ADV_RSSI_INDEX],&Ibeacon_store_data.RSSI,1);
		LOG("Ibeacon_store_data.advIntvl=%d\n",Ibeacon_store_data.advIntvl);
		adv_intvl=	Ibeacon_store_data.advIntvl*1000/625;
		LOG("intvl=%d\n",adv_intvl);
	} 		
 	
    // Setup the GAP
    VOID GAP_SetParamValue( TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL );
  
    // Setup the GAP Peripheral Role Profile
    {
        // device starts advertising upon initialization
        uint8 initial_advertising_enable =TRUE;

        uint8 enable_update_request = DEFAULT_ENABLE_UPDATE_REQUEST;
        uint8 advChnMap = GAP_ADVCHAN_37 | GAP_ADVCHAN_38 | GAP_ADVCHAN_39; 
        
        // By setting this to zero, the device will go into the waiting state after
        // being discoverable for 30.72 second, and will not being advertising again
        // until the enabler is set back to TRUE
        uint16 gapRole_AdvertOffTime	=	0;
    
        uint16 desired_min_interval	=	DEFAULT_DESIRED_MIN_CONN_INTERVAL;
        uint16 desired_max_interval	=	DEFAULT_DESIRED_MAX_CONN_INTERVAL;
        uint16 desired_slave_latency	=	DEFAULT_DESIRED_SLAVE_LATENCY;
        uint16 desired_conn_timeout	=	DEFAULT_DESIRED_CONN_TIMEOUT;
		
        uint8 peerPublicAddr[] = {
			0x01,
			0x02,
			0x03,
			0x04,
			0x05,
			0x06
		};

		uint8 advType =g_current_advType;// LL_ADV_NONCONNECTABLE_UNDIRECTED_EVT;//LL_ADV_SCANNABLE_UNDIRECTED_EVT;//LL_ADV_CONNECTABLE_LDC_DIRECTED_EVT;//;    // it seems a  bug to set GAP_ADTYPE_ADV_NONCONN_IND = 0x03
		GAPRole_SetParameter( GAPROLE_ADV_EVENT_TYPE, sizeof( uint8 ), &advType );

		GAPRole_SetParameter( GAPROLE_ADV_DIRECT_ADDR, sizeof(peerPublicAddr), peerPublicAddr);
		// set adv channel map
		GAPRole_SetParameter( GAPROLE_ADV_CHANNEL_MAP, sizeof(uint8), &advChnMap);        

		// Set the GAP Role Parameters
		GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
		GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );

		// osal_memcpy(&scanRspData[2],attDeviceName,0x12);
		GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanRspData ), scanRspData );
		GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );

		GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
		GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
		GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
		GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
		GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );
    }

    // Set the GAP Characteristics
    GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName );

    // Set advertising interval
    {
        uint16 advInt = 800;//2400;//1600;//1600;//800;//1600;   // actual time = advInt * 625us
        advInt=adv_intvl;
        GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt );
        GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt );
        GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
        GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );
    }
    // Initialize GATT attributes
    GGS_AddService( GATT_ALL_SERVICES );            // GAP
    GATTServApp_AddService( GATT_ALL_SERVICES );    // GATT attributes
    DevInfo_AddService();                           // Device Information Service
//	ota_app_AddService();
    SimpleProfile_AddService( GATT_ALL_SERVICES );  // Simple GATT Profile

    // Setup a delayed profile startup
    osal_set_event( simpleBLEPeripheral_TaskID, SBP_START_DEVICE_EVT );
    // for receive HCI complete message
    GAP_RegisterForHCIMsgs(simpleBLEPeripheral_TaskID);

    LL_PLUS_PerStats_Init(&g_perStatsByChanTest);
    osal_start_timerEx(simpleBLEPeripheral_TaskID, SBP_RESET_ADV_EVT, CHANGE_ADV_TYPE_TIME);  
		
    LOG("======================SimpleBLEPeripheral_Init Done====================\n");
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_ProcessEvent
 *
 * @brief   Simple BLE Peripheral Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 SimpleBLEPeripheral_ProcessEvent( uint8 task_id, uint16 events )
{
    VOID task_id; // OSAL required parameter that isn't used in this function

    if ( events & SYS_EVENT_MSG )
    {
        uint8 *pMsg;
		if ( (pMsg = osal_msg_receive( simpleBLEPeripheral_TaskID )) != NULL )
		{
			simpleBLEPeripheral_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );
			// Release the OSAL message
			VOID osal_msg_deallocate( pMsg );
		}
		// return unprocessed events
		return (events ^ SYS_EVENT_MSG);
    }

    if ( events & SBP_START_DEVICE_EVT )
    {
        // Start the Device
        VOID GAPRole_StartDevice( &simpleBLEPeripheral_PeripheralCBs );        
        HCI_LE_ReadResolvingListSizeCmd();
        LOG("dev start\n");
        return ( events ^ SBP_START_DEVICE_EVT );
    }

    // change to no conn adv
    if ( events & SBP_ADD_RL_EVT )
    {
		uint8 initial_advertising_enable=TRUE;
		initial_advertising_enable = TRUE;	 
		GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );	
		LOG("enable[%d] advtype=%x\n",initial_advertising_enable,g_current_advType);	
		return ( events ^ SBP_ADD_RL_EVT );
    }

    // enable adv
    if ( events & SBP_RESET_ADV_EVT )
    {
		LOG("rst adv\n");
		updateAdvData();
		return ( events ^ SBP_RESET_ADV_EVT );
    }  

	if ( events & SBP_CONNECTED_EVT )
	{
		uint8 advType;//g_current_advType;// LL_ADV_NONCONNECTABLE_UNDIRECTED_EVT;//LL_ADV_SCANNABLE_UNDIRECTED_EVT;//LL_ADV_CONNECTABLE_LDC_DIRECTED_EVT;//;    // it seems a  bug to set GAP_ADTYPE_ADV_NONCONN_IND = 0x03
		osal_stop_timerEx(simpleBLEPeripheral_TaskID, SBP_RESET_ADV_EVT);
		g_current_advType=LL_ADV_NONCONNECTABLE_UNDIRECTED_EVT;
		advType=g_current_advType;
		GAPRole_SetParameter( GAPROLE_ADV_EVENT_TYPE, sizeof( uint8 ), &advType );
		LOG("connected,change adv type=%x\n",g_current_advType);
		return ( events ^ SBP_CONNECTED_EVT );
	} 
	// notifity
    if ( events & SBP_NOTIFY_EVT )
	{
		simpleProfile_Notify(gapConnHandle ,(uint8*)&Beacon_cmd_rsp_data,sizeof(Beacon_cmd_rsp_data));
		return ( events ^ SBP_NOTIFY_EVT );
	} 
    return 0;
}

/*********************************************************************
 * @fn      simpleBLEPeripheral_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void simpleBLEPeripheral_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
	hciEvt_CmdComplete_t *pHciMsg;
	switch ( pMsg->event )
	{  
		case HCI_GAP_EVENT_EVENT:
		{
			switch( pMsg->status )
			{
				case HCI_COMMAND_COMPLETE_EVENT_CODE:
					pHciMsg = (hciEvt_CmdComplete_t *)pMsg;

					LOG("==> HCI_COMMAND_COMPLETE_EVENT_CODE: %x\n", pHciMsg->cmdOpcode);
//					safeToDealloc = gapProcessHCICmdCompleteEvt( (hciEvt_CmdComplete_t *)pMsg );
				break;
				default:
					//safeToDealloc = FALSE;  // Send to app
				break;
			}
		}
	}
}
/*********************************************************************
 * @fn      peripheralStateReadRssiCB
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void peripheralStateReadRssiCB( int8  rssi )
{
//    notifyBuf[15]++;
//    notifyBuf[16]=rssi;
//    notifyBuf[17]=HI_UINT16(g_conn_param_foff);
//    notifyBuf[18]=LO_UINT16(g_conn_param_foff);;
//    notifyBuf[19]=g_conn_param_carrSens;
}

/*********************************************************************
 * @fn      peripheralStateNotificationCB
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void peripheralStateNotificationCB( gaprole_States_t newState )
{
    switch ( newState )
    {
        case GAPROLE_STARTED:
        {
			uint8 ownAddress[B_ADDR_LEN];
			uint8 str_addr[14]={0}; 
			uint8 initial_advertising_enable =TRUE;//true
			GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);
//            #if(0) 
//            uint8 systemId[DEVINFO_SYSTEM_ID_LEN];
//            // use 6 bytes of device address for 8 bytes of system ID value
//            systemId[0] = ownAddress[0];
//            systemId[1] = ownAddress[1];
//            systemId[2] = ownAddress[2];
//        
//            // set middle bytes to zero
//            systemId[4] = 0x00;
//            systemId[3] = 0x00;
//        
//            // shift three bytes up
//            systemId[7] = ownAddress[5];
//            systemId[6] = ownAddress[4];
//            systemId[5] = ownAddress[3];
//        
//            DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);
//            #endif    

//            osal_memcpy(&str_addr[0],bdAddr2Str(ownAddress),14);
//            osal_memcpy(&scanRspData[11],&str_addr[6],8);
//            osal_memcpy(&attDeviceName[9],&str_addr[6],8);
            //GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanRspData ), scanRspData );
            // Set the GAP Characteristics
            GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName );
            GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
        }
            break;
        
        case GAPROLE_ADVERTISING:  
            break;
        
        case GAPROLE_CONNECTED:
			HCI_PPLUS_ConnEventDoneNoticeCmd(simpleBLEPeripheral_TaskID, NULL);
			GAPRole_GetParameter( GAPROLE_CONNHANDLE, &gapConnHandle );
			osal_set_event(simpleBLEPeripheral_TaskID,SBP_CONNECTED_EVT);
            break;
        
        case GAPROLE_CONNECTED_ADV:
            break;      
        case GAPROLE_WAITING:
            break;
        
        case GAPROLE_WAITING_AFTER_TIMEOUT:
            break;
        
        case GAPROLE_ERROR:
            break;
        
        default:
            break;        
    }  
    gapProfileState = newState;

    LOG("[GAP ROLE %d]\n",newState);
     
    VOID gapProfileState;     
}


/*********************************************************************
 * @fn      simpleProfileChangeCB
 *
 * @brief   Callback from SimpleBLEProfile indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void simpleProfileChangeCB( uint8 paramID )
{
	uint8 newValue[IBEACON_ATT_LONG_PKT];

	switch( paramID )
	{
		default:
		// not process other attribute change
		break;
	}
}


/*********************************************************************
 * @fn      updateAdvData
 *
 * @brief   update adv data and change the adv type
 *
 * @param   none
 *
 * @return  none
 */
static void updateAdvData(void)
{
	uint8  new_uuid[IBEACON_UUID_LEN];
	uint16  major;
	uint16  minor;
	uint8   power;
	uint8 initial_advertising_enable = FALSE;	
	initial_advertising_enable = FALSE;	

	GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );		
 	LOG("disable ADV\n");   
	// 5. update adv data
	// change adv type
	g_current_advType=LL_ADV_NONCONNECTABLE_UNDIRECTED_EVT;    
	uint8 advType = g_current_advType;
	GAPRole_SetParameter( GAPROLE_ADV_EVENT_TYPE, sizeof( uint8 ), &advType );   
	// 6. set reset advertisement event, note that GAP/LL will process close adv event in advance
	osal_start_timerEx(simpleBLEPeripheral_TaskID, SBP_ADD_RL_EVT, 500);
}

/*********************************************************************
* @fn      bdAddr2Str
*
* @brief   Convert Bluetooth address to string. Only needed when
*          LCD display is used.
*
* @return  none
*/
char *bdAddr2Str( uint8 *pAddr )
{
  uint8       i;
  char        hex[] = "0123456789ABCDEF";
  static char str[B_ADDR_STR_LEN];
  char        *pStr = str;
  
  *pStr++ = '0';
  *pStr++ = 'x';
  
  // Start from end of addr
  pAddr += B_ADDR_LEN;
  
  for ( i = B_ADDR_LEN; i > 0; i-- )
  {
    *pStr++ = hex[*--pAddr >> 4];
    *pStr++ = hex[*pAddr & 0x0F];
  }
  
  *pStr = 0;
  
  return str;
}

/*********************************************************************
*********************************************************************/
