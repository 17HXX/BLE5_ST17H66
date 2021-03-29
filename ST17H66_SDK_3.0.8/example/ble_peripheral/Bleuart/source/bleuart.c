
/**************************************************************************************************
  Filename:       bleuart.c
  Revised:        
  Revision:       

  Description:    This file contains the ble uart rawpass application
                  

**************************************************************************************************/


#include "types.h"
#include "bcomdef.h"
//#include "simpleGATTprofile_ota.h"
#include "bleuart_service.h"
#include "rf_phy_driver.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "gatt.h"
#include "hci.h"

#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"
#include "bleuart_service.h"

#include "peripheral.h"
#include "gapbondmgr.h"

#include "bleuart.h"
#include "bleuart_service.h"
#include "bleuart_protocol.h"
#include "log.h"
#include "osal_snv.h"
#include "flash.h"
#include "ota_app_service.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// How often to perform periodic event
#define BUP_PERIODIC_EVT_PERIOD                   5000

#define DEVINFO_SYSTEM_ID_LEN             8
#define DEVINFO_SYSTEM_ID                 0
 

#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     8

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     10

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          600

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         TRUE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         6


// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

//add bonding chendy 20200605

// Default passcode
#define DEFAULT_PASSCODE                      0

// Default GAP pairing mode
//#define DEFAULT_PAIRING_MODE                  GAPBOND_PAIRING_MODE_INITIATE
#define DEFAULT_PAIRING_MODE                  GAPBOND_PAIRING_MODE_WAIT_FOR_REQ

// Default MITM mode (TRUE to require passcode or OOB when pairing)
#define DEFAULT_MITM_MODE                     TRUE

// Default bonding mode, TRUE to bond
#define DEFAULT_BONDING_MODE                  TRUE

// Default GAP bonding I/O capabilities
#define DEFAULT_IO_CAPABILITIES               GAPBOND_IO_CAP_NO_INPUT_NO_OUTPUT//GAPBOND_IO_CAP_DISPLAY_ONLY // GAPBOND_IO_CAP_NO_INPUT_NO_OUTPUT


//AT指令数据标志位
uint8 Modify_BLEDevice_Data = 0;


//广播间隔
uint16 advInt[6] ={80,160,320,800,1600,3200};   // actual time = advInt * 625us				广播间隔
uint8  advint = 2;								//默认广播间隔200ms

//是否连接上自动进入透传
uint8 AT_bleuart_auto=0x59;			//0x59:Y  默认蓝牙连接后自动进入透传	0x4E：N   蓝牙连接后不自动进入透传
uint8 AT_bleuart_sleep=0;			//UNUSED


//发射功率
//0：10DB
//1：5DB
//2：4DB
//3：3DB
//4：0DB
//5：-2DB
//6：-5DB
//7：-10DB
uint8 AT_Tx_Power[8]={0x1f,0x1d,0x17,0x15,0x0d,0x0a,0x06,0x03};		
uint8 AT_bleuart_txpower=4;											//默认发射功率0DB


//广播包自定义数据起始字节数
uint8 AT_cnt_advdata=9;


uint8 bleuart_TaskID;   // Task ID for internal task/event processing

uint16 gapConnHandle;

static gaprole_States_t gapProfileState = GAPROLE_INIT;

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8 scanRspData[] =
{
    // complete name
    21,   // length of this data
    GAP_ADTYPE_LOCAL_NAME_COMPLETE,
    'U','a','r','t','-','T','e','s','t',' ',
	' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',

// connection interval range
//    0x05,   // length of this data
//    GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
//    LO_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),   // 100ms
//    HI_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),
//    LO_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),   // 1s
//    HI_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),

    // Tx power level
//    0x02,   // length of this data
//    GAP_ADTYPE_POWER_LEVEL,
//    5       // 0dBm
};



// advert data for bleuart
static uint8 advertData[] =
{	
    0x02,   // length of this data
    GAP_ADTYPE_FLAGS,
    DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,
	
    0x03,
    0x03,											//Complete list of 16-bit UUIDs
    0xf0, 0xff,
	
	0x09,
	0xff,
	0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,		//reserved data
};


// GAP GATT Attributes
static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "Uart-Test           ";

uint8*scanR=scanRspData;
uint8*advertdata=advertData;

//OTA升级密钥
uint8 ota_keylen=10;
uint8 ota_key[10]={'0','0','0','0','0','0','0','0','0','2'};
/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void bleuart_StateNotificationCB( gaprole_States_t newState );

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t bleuart_PeripheralCBs =
{
    bleuart_StateNotificationCB,  // Profile State Change Callbacks
    NULL                            // When a valid RSSI is read from controller (not used by application)
};


static void BleUart_ProcessPasscodeCB(uint8 *deviceAddr, uint16 connectionHandle, uint8 uiInputs, uint8 uiOutputs);
static void BleUart_ProcessPairStateCB(uint16 connHandle, uint8 state, uint8 status);

static gapBondCBs_t BleUart_BondMgrCBs =
{
  (pfnPasscodeCB_t) BleUart_ProcessPasscodeCB, // Passcode callback
  BleUart_ProcessPairStateCB                   // Pairing / Bonding state Callback
};

static void BleUart_ProcessPasscodeCB(uint8 *deviceAddr, uint16 connectionHandle, uint8 uiInputs, uint8 uiOutputs)
{
	
	uint32 passcode; 
// uint8 str[7]; 
	
	LOG("Entry passcode process\n");
 // create pass code
 #if 0 
  LL_Rand(((uint8 *) &passcode), sizeof( uint32 )); 
passcode %= 1000000; 
#else 
  passcode = 456890; 
  #endif 
LOG("passcode is=%d\n",passcode);	
  //display passcode or log passcode
 if(uiOutputs != 0) 
  { 
   LOG("passcode is=%d\n",passcode); 
  } 
  //pass code rsp
GAPBondMgr_PasscodeRsp(connectionHandle, SUCCESS, passcode); 
	
 } 



static void BleUart_ProcessPairStateCB(uint16 connHandle, uint8 state, uint8 status)
{
	

  //paring state 
  if(state == GAPBOND_PAIRING_STATE_STARTED) 
  { 
  LOG("Pairing started"); 
 
  } 
   //
  else if(state == GAPBOND_PAIRING_STATE_COMPLETE) 
  { 
  //
  if(status == SUCCESS) 
  { 
  LOG("Pairing success"); 
  
  } 
  //
  else if(status == SMP_PAIRING_FAILED_UNSPECIFIED) 
  { 
  LOG("Paired device"); 

  } 
  //
  else 
  { 
  LOG("Pairing fail"); 
 
  } 
 
  } 
  else if (state == GAPBOND_PAIRING_STATE_BONDED) 
  { 
   if (status == SUCCESS) 
   { 
      LOG("Bonding success"); 
   } 
  } 

}	
/*********************************************************************
 * PUBLIC FUNCTIONS
 */

void on_bleuartServiceEvt(bleuart_Evt_t* pev)
{
  LOG("on_bleuartServiceEvt:%d\n", pev->ev);
  switch(pev->ev){
  case bleuart_EVT_TX_NOTI_DISABLED:
    BUP_disconnect_handler();
    break;
  case bleuart_EVT_TX_NOTI_ENABLED :
    BUP_connect_handler();
    osal_set_event(bleuart_TaskID,BUP_OSAL_EVT_NOTIFY_DATA);
    break;
  case bleuart_EVT_BLE_DATA_RECIEVED:
    BUP_data_BLE_to_uart( (uint8_t*)pev->data, (uint8_t)pev->param);
	  LOG("%s,%d\n",pev->data,pev->param);
    break;
  default:
    break;
  }
}

void on_BUP_Evt(BUP_Evt_t* pev)
{
  switch(pev->ev){
    

  }
}


void bleuart_Init( uint8 task_id )
{
  bleuart_TaskID = task_id;

  // Setup the GAP
  VOID GAP_SetParamValue( TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL );
      
  // Setup the GAP Peripheral Role Profile
  {
    // device starts advertising upon initialization
    uint8 initial_advertising_enable = TRUE;
    
    uint8 enable_update_request = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint8 advChnMap = GAP_ADVCHAN_37 | GAP_ADVCHAN_38 | GAP_ADVCHAN_39; 
            
    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16 gapRole_AdvertOffTime = 0;
        
    uint16 desired_min_interval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16 desired_max_interval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16 desired_slave_latency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16 desired_conn_timeout = DEFAULT_DESIRED_CONN_TIMEOUT;
        
    uint8 peerPublicAddr[] = {
      0x01,
      0x02,
      0x03,
      0x04,
      0x05,
      0x06
    };
	
/***************************************************************************************************************/
//获取修改设备数据
	uint8 AT_mac_address[6];
	hal_flash_read(0x4004,AT_mac_address,2);   //读取mac地址
	hal_flash_read(0x4000,AT_mac_address+2,4);  
//	LOG("MAC: %x %x %x %x %x %x",AT_mac_address[0],AT_mac_address[1],AT_mac_address[2],AT_mac_address[3],AT_mac_address[4],AT_mac_address[5]) ;

	uint8 uart_baudrate[4]={BREAK_UINT32(UART_Baudrate,3),BREAK_UINT32(UART_Baudrate,2),BREAK_UINT32(UART_Baudrate,1),BREAK_UINT32(UART_Baudrate,0)};
	
#if (AT_UART==1)
	osal_snv_read(0x80,1,&Modify_BLEDevice_Data);
	if(Modify_BLEDevice_Data==1)
	{
		osal_snv_read(0x81,20,&(scanRspData[2]));			// 20 Bytes Device Name
		osal_snv_read(0x82,6,AT_mac_address);				// 6 Bytes MAC address
		osal_snv_read(0x83,4,uart_baudrate);				// 4 Bytes uart_baudrate
		osal_snv_read(0x84,1,&advint);						// 1 Byte Advert interval
		osal_snv_read(0x85,1,&AT_bleuart_auto);				// 1 Byte BLE_UART AUTO
		osal_snv_read(0x86,1,&AT_bleuart_sleep);			// 1 Byte SLEEP MODE		UNUSED
		osal_snv_read(0x87,1,&AT_bleuart_txpower);			// 1 Byte TX power
		osal_snv_read(0x88,8,&(advertdata[AT_cnt_advdata]));			// 8 Bytes reserved data
		
		UART_Baudrate=BUILD_UINT32(uart_baudrate[3],uart_baudrate[2],uart_baudrate[1],uart_baudrate[0]);
		g_rfPhyTxPower = AT_Tx_Power[AT_bleuart_txpower];
		
	}
	else if(Modify_BLEDevice_Data==2)
	{

	}
	else if(Modify_BLEDevice_Data==3)						//恢复出厂设置
	{
		osal_snv_read(0x92,6,AT_mac_address);				// 6 Bytes MAC address
		
		osal_snv_write(0x81,20,&(scanRspData[2]));			// 20 Bytes Device Name
		osal_snv_write(0x82,6,AT_mac_address);				// 6 Bytes MAC address
		osal_snv_write(0x83,4,uart_baudrate);				// 4 Bytes uart_baudrate
		osal_snv_write(0x84,1,&advint);						// 1 Byte Advert interval
		osal_snv_write(0x85,1,&AT_bleuart_auto);			// 1 Byte BLE_UART AUTO
		osal_snv_write(0x86,1,&AT_bleuart_sleep);			// 1 Byte SLEEP MODE		UNUSED
		osal_snv_write(0x87,1,&AT_bleuart_txpower);			// 1 Byte TX power
		osal_snv_write(0x88,8,&(advertdata[AT_cnt_advdata]));			// 8 Bytes reserved data
		
		osal_snv_write(0x89,6,AT_mac_address);				// 6 Bytes MAC address
		
		Modify_BLEDevice_Data = 2;
		osal_snv_write(0x80,1,&Modify_BLEDevice_Data);
	}
	else													//初始化设置
	{
		osal_snv_write(0x81,20,&(scanRspData[2]));			// 20 Bytes Device Name
		osal_snv_write(0x82,6,AT_mac_address);				// 6 Bytes MAC address
		osal_snv_write(0x83,4,uart_baudrate);				// 4 Bytes uart_baudrate
		osal_snv_write(0x84,1,&advint);						// 1 Byte Advert interval
		osal_snv_write(0x85,1,&AT_bleuart_auto);			// 1 Byte BLE_UART AUTO
		osal_snv_write(0x86,1,&AT_bleuart_sleep);			// 1 Byte SLEEP MODE		UNUSED
		osal_snv_write(0x87,1,&AT_bleuart_txpower);			// 1 Byte TX power
		osal_snv_write(0x88,8,&(advertdata[AT_cnt_advdata]));			// 8 Bytes reserved data
		
		osal_snv_write(0x89,6,AT_mac_address);				// 6 Bytes MAC address
		
		osal_snv_write(0x91,20,&(scanRspData[2]));			// 20 Bytes Device Name
		osal_snv_write(0x92,6,AT_mac_address);				// 6 Bytes MAC address
		osal_snv_write(0x93,4,uart_baudrate);				// 4 Bytes uart_baudrate
		osal_snv_write(0x94,1,&advint);						// 1 Byte Advert interval
		osal_snv_write(0x95,1,&AT_bleuart_auto);			// 1 Byte BLE_UART AUTO
		osal_snv_write(0x96,1,&AT_bleuart_sleep);			// 1 Byte SLEEP MODE		UNUSED
		osal_snv_write(0x97,1,&AT_bleuart_txpower);			// 1 Byte TX power
		osal_snv_write(0x98,8,&(advertdata[AT_cnt_advdata]));			// 8 Bytes reserved data
		
		Modify_BLEDevice_Data = 2;
		osal_snv_write(0x80,1,&Modify_BLEDevice_Data);
		
		osal_snv_read(0x81,20,&(scanRspData[2]));			// 20 Bytes Device Name
		osal_snv_read(0x82,6,AT_mac_address);				// 6 Bytes MAC address
		osal_snv_read(0x83,4,uart_baudrate);				// 4 Bytes uart_baudrate
		osal_snv_read(0x84,1,&advint);						// 1 Byte Advert interval
		osal_snv_read(0x85,1,&AT_bleuart_auto);				// 1 Byte BLE_UART AUTO
		osal_snv_read(0x86,1,&AT_bleuart_sleep);			// 1 Byte SLEEP MODE		UNUSED
		osal_snv_read(0x87,1,&AT_bleuart_txpower);			// 1 Byte TX power
		osal_snv_read(0x88,8,&(advertdata[AT_cnt_advdata]));			// 8 Bytes reserved data
	}
#endif
/***************************************************************************************************************/		
	
	
	
	
    //    uint8 advType = LL_ADV_NONCONNECTABLE_UNDIRECTED_EVT;//LL_ADV_SCANNABLE_UNDIRECTED_EVT;//LL_ADV_CONNECTABLE_LDC_DIRECTED_EVT;
    //    GAPRole_SetParameter( GAPROLE_ADV_EVENT_TYPE, sizeof( uint8 ), &advType );
    
    GAPRole_SetParameter(GAPROLE_ADV_DIRECT_ADDR, sizeof(peerPublicAddr), peerPublicAddr);
    // set adv channel map
    GAPRole_SetParameter(GAPROLE_ADV_CHANNEL_MAP, sizeof(uint8), &advChnMap);        
    
    // Set the GAP Role Parameters
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
    GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );
    
    GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanRspData ), scanRspData );
    GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );
    
    GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
    GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
    GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
    GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
    GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );
  }
    
  // Set the GAP Characteristics
  osal_memcpy(attDeviceName,&scanRspData[2],20);
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName );
    
//	 // Setup the GAP Bond Manager chendy 20200605
//  {
//    uint32 passkey = DEFAULT_PASSCODE;
//    uint8 pairMode = DEFAULT_PAIRING_MODE;
//    uint8 mitm = DEFAULT_MITM_MODE;
//    uint8 ioCap = DEFAULT_IO_CAPABILITIES;
//    uint8 bonding = DEFAULT_BONDING_MODE;
//    GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof( uint32 ), &passkey );
//    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof( uint8 ), &pairMode );
//    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof( uint8 ), &mitm );
//    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof( uint8 ), &ioCap );
//    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof( uint8 ), &bonding );
//  }
//	
	
	
	
	
	
	
  // Set advertising interval
  {
//    uint16 advInt = 400;   // actual time = advInt * 625us
    
    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt[advint] );
    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt[advint] );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt[advint] );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt[advint] );
  }
      
  // Initialize GATT attributes
//  GGS_AddService( GATT_ALL_SERVICES );            // GAP
//  GATTServApp_AddService( GATT_ALL_SERVICES );    // GATT attributes
//  DevInfo_AddService();                           // Device Information Service
  
//  ota_app_AddService(ota_keylen,ota_key);			//开启OTA服务
  
  bleuart_AddService(on_bleuartServiceEvt);		//串口透传服务
  BUP_init(on_BUP_Evt);							//串口初始化
	
  // Setup a delayed profile startup
  osal_set_event( bleuart_TaskID, BUP_OSAL_EVT_START_DEVICE );
//  //osal_start_timerEx( bleuart_TaskID, BUP_OSAL_EVT_BLE_TIMER, 1 );
// {
//	 extern void RF433_analysis_init(uint8 taskid,uint16 event);
//	 RF433_analysis_init(bleuart_TaskID,BUP_OSAL_EVT_RF433_KEY);
//	 
// }
}


static void BUP_ProcessOSALMsg( osal_event_hdr_t* pMsg )
{
    switch ( pMsg->event )
    {
		default:
        // do nothing
        break;
    }
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
uint16 bleuart_ProcessEvent( uint8 task_id, uint16 events )
{
  VOID task_id; // OSAL required parameter that isn't used in this function
	
    if (events & SYS_EVENT_MSG)
	{
		uint8* pMsg;

		if ((pMsg = osal_msg_receive(bleuart_TaskID)) != NULL)
		{
			BUP_ProcessOSALMsg((osal_event_hdr_t*)pMsg);
			// Release the OSAL message
			VOID osal_msg_deallocate(pMsg);
		}

		// return unprocessed events
		return (events ^ SYS_EVENT_MSG);
	}

  if ( events & BUP_OSAL_EVT_START_DEVICE )
  {
    // Start the Device
    VOID GAPRole_StartDevice( &bleuart_PeripheralCBs );

//		GAPBondMgr_Register(&BleUart_BondMgrCBs); //chendy add 20200605
    return ( events ^ BUP_OSAL_EVT_START_DEVICE );
  }

  // enable adv
  if ( events & BUP_OSAL_EVT_RESET_ADV )
  {
    uint8 initial_advertising_enable = TRUE;
		
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );	
	  
    return ( events ^ BUP_OSAL_EVT_RESET_ADV );
  }

  if(events & BUP_OSAL_EVT_NOTIFY_DATA)
  {
    LOG("BUP_OSAL_EVT_NOTIFY_DATA\n");
    return ( events ^ BUP_OSAL_EVT_NOTIFY_DATA );
  }

  if( events & BUP_OSAL_EVT_BLE_TIMER)
  {
    LOG("BUP_OSAL_EVT_BLE_TIMER\n");
    BUP_data_BLE_to_uart_send();
    return ( events ^ BUP_OSAL_EVT_BLE_TIMER );
  }
  if( events & BUP_OSAL_EVT_UARTRX_TIMER)
  {
    LOG("BUP_OSAL_EVT_UARTRX_TIMER\n");
    BUP_data_uart_to_BLE_send();
    return ( events ^ BUP_OSAL_EVT_UARTRX_TIMER );
  }

  if( events & BUP_OSAL_EVT_UART_TX_COMPLETE)
  {
    LOG("BUP_OSAL_EVT_UART_TX_COMPLETE\n");
    BUP_data_BLE_to_uart_completed();
    return ( events ^ BUP_OSAL_EVT_UART_TX_COMPLETE);
  }
  if( events & BUP_OSAL_EVT_UART_TO_TIMER)
  {
    LOG("RX TO\n");
    BUP_data_uart_to_BLE();
    return ( events ^ BUP_OSAL_EVT_UART_TO_TIMER);
  }
	
	if( events & BUP_OSAL_EVT_RF433_KEY)
  {
//   extern uint32 RF433_Get_data(void); 
//   
//		LOG("RF433 key:%x\n",RF433_Get_data());
    return ( events ^ BUP_OSAL_EVT_RF433_KEY);
  }
  
   if( events & BUP_OSAL_EVT_AT)
  {
#if (AT_UART==1)
	  
	uint8 ret=2;
	AT_BLEUART_RX_t* at_rx = & at_bleuart_rx;
	  
	//基础指令和查询指令
	ret=AT_query(at_rx);
	//设置指令
	if(ret==2){
	ret=AT_setdata(at_rx);
	}
	
	if(ret==0){
		at_rx->len=0;
	}
	else if(ret==3){
		GAPRole_TerminateConnection();
//		WaitMs(100);
		uint8 initial_advertisin_enable = FALSE;		
		GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertisin_enable );
		osal_memcpy(attDeviceName,&scanRspData[2],20);
		GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanRspData ), scanRspData );				//更新设备名
		GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName );
		GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );					//更新自定义广播数据
		GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt[advint] );									//更新广播间隔
		GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt[advint] );
		GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt[advint] );
		GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt[advint] );
		g_rfPhyTxPower = AT_Tx_Power[AT_bleuart_txpower];												//更新发射功率
		at_rx->len=0;
		osal_start_timerEx(bleuart_TaskID, BUP_OSAL_EVT_RESET_ADV,50);									//修改广播数据后必须延时30ms以上再开启广播
	}
	else if(ret==1){
	Modify_BLEDevice_Data = 1;
	osal_snv_write(0x80,1,&Modify_BLEDevice_Data);
	at_rx->len=0;
	}
	else{
		uint8 at_error[8]={0x41,0x54,0x2B,0x45,0x52,0x52,0x0d,0x0a};
		hal_uart_send_buff(UART1,at_error, 8);
		at_rx->len=0;
	}
	
#endif
    return ( events ^ BUP_OSAL_EVT_AT);
  }
	
  // Discard unknown events
  return 0;
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
extern uint8 ownPublicAddr[LL_DEVICE_ADDR_LEN];

static void bleuart_StateNotificationCB( gaprole_States_t newState )
{
    switch ( newState )
    {
        case GAPROLE_STARTED:
        {
            uint8 ownAddress[B_ADDR_LEN];
            uint8 systemId[DEVINFO_SYSTEM_ID_LEN];
        
            GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);
        
            // use 6 bytes of device address for 8 bytes of system ID value
            systemId[0] = ownAddress[0];
            systemId[1] = ownAddress[1];
            systemId[2] = ownAddress[2];
        
            // set middle bytes to zero
            systemId[4] = 0x00;
            systemId[3] = 0x00;
        
            // shift three bytes up
            systemId[7] = ownAddress[5];
            systemId[6] = ownAddress[4];
            systemId[5] = ownAddress[3];
        
            DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);
			
			#if (AT_UART==1)
			uint8 i;
			uint8 mac_addr[6];
			osal_snv_read(0x80,1,&Modify_BLEDevice_Data);
			if((Modify_BLEDevice_Data==2)||	(Modify_BLEDevice_Data==1))								//更新MAC地址
			{
				osal_snv_read(0x89,6,mac_addr);
				for(i=0;i<LL_DEVICE_ADDR_LEN;i++)
				{
					ownPublicAddr[i]=mac_addr[LL_DEVICE_ADDR_LEN-1-i];
				}
			}
			#endif
        }
            break;
        
        case GAPROLE_ADVERTISING:
         // FLOW_CTRL_BLE_DISCONN();
          BUP_disconnect_handler();
				  hal_gpio_write(UART_INDICATE_LED,0);
         // FLOW_CTRL_UART_TX_UNLOCK();
         // FLOW_CTRL_BLE_TX_UNLOCK();
		#if (AT_UART==1)
			if(AT_bleuart_auto==0x59){
				Bleuart_C_D=0;
			}
			else{Bleuart_C_D=0;}
		#endif
          LOG("advertising!\n");
          break;
        
        case GAPROLE_CONNECTED:
          GAPRole_GetParameter( GAPROLE_CONNHANDLE, &gapConnHandle );
				  hal_gpio_write(UART_INDICATE_LED,1);
         // FLOW_CTRL_BLE_CONN();
		#if (AT_UART==1)
			if(AT_bleuart_auto==0x59){
				Bleuart_C_D=1;
			}
			else{Bleuart_C_D=0;}
		#endif
          LOG("connected handle[%d]!\n", gapConnHandle);
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
     
    VOID gapProfileState;     
}


uint16_t bleuart_conn_interval(void)
{
    uint16_t interval, latency;
    GAPRole_GetParameter(GAPROLE_CONNECTION_INTERVAL, &interval);
    GAPRole_GetParameter(GAPROLE_CONNECTION_LATENCY, &latency);
    return ((1+latency)*interval*5/4);
}


/*********************************************************************
*********************************************************************/
