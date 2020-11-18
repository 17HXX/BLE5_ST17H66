
/**************************************************************************************************
  Filename:       pwmdemo.c
  Revised:        $Date $
  Revision:       $Revision $



*********************************************************************/
/* INCLUDES
 */

#include "bcomdef.h"
#include "OSAL.h"
#include "linkdb.h"
#include "gatt.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "gatt_profile_uuid.h"
#include "pwmservice.h"
#include "devinfoservice.h"
#include "peripheral.h"
#include "gapbondmgr.h"
#include "pwmdemo.h"
#include "pwm.h"
#include "color_mode.h"
#include "log.h"

/*********************************************************************
 * MACROS
 */


/*********************************************************************
 * CONSTANTS
 */
uint8 PWM_TaskID;
// Fast advertising interval in 625us units
#define DEFAULT_FAST_ADV_INTERVAL             32

// Duration of fast advertising duration in ms
#define DEFAULT_FAST_ADV_DURATION             0

// Slow advertising interval in 625us units
#define DEFAULT_SLOW_ADV_INTERVAL             160

// Duration of slow advertising duration in ms (set to 0 for continuous advertising)
#define DEFAULT_SLOW_ADV_DURATION             0


// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         FALSE

// Minimum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     200

// Maximum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     1600

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         1

// Supervision timeout value (units of 10ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          1000






/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
//uint8 PWM_TaskID;   // Task ID for internal task/event processing

static gaprole_States_t gapProfileState = GAPROLE_INIT;

// GAP Profile - Name attribute for SCAN RSP data
static uint8 scanData[] =
{
  0x0c,   // length of this data
  0x09,// local device name GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  'B','S','-','R','G','B','-','5','0','0','8'
};

#define ADVDATA_MAC_ADDR_OFFSET 11
static uint8 advertData[] = 
{ 
//	0x02,0x01,0x06,
//	0x03,0x02,0x77,0x77,
//	0x0f,0xff,
//	0x77,                   //当前灯带
//	0x66,                   //17H66
//	0xff,0xff,0xff,0xff,0xff,0xff,//当前设备MAC地址
//	0x66,//该设备类型高地址
//	0x01,//设备类型低地址：00 测试版 01 RGB 三色 02 RGBW四色
//	0x00,//版本号高地址
//	0x01,//版本号低地址
//	0xff,//CRC-16/XMODEM校验高位
//	0xff,//CRC-16/XMODEM校验地位
	0x02, 0x01, 0x06,
	0x03, 0x02, 0x77, 0x77,
	0x0f, 0xff, 0x05, 0x01, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
};

// Device name attribute value
static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "BS-RGB-5008";

// GAP connection handle
static uint16 gapConnHandle;

// Advertising user-cancelled state
static bool PWMAdvCancelled = FALSE;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void PWMGapStateCB( gaprole_States_t newState );

/*********************************************************************
 * PROFILE CALLBACKS
 */


// GAP Role Callbacks
static gapRolesCBs_t PWMPeripheralCB =
{
  PWMGapStateCB,  // Profile State Change Callbacks
  NULL                            // When a valid RSSI is read from controller
};

// Bond Manager Callbacks
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


static const gapBondCBs_t PWMBondCB =
{
  NULL,                   // Passcode callback
  NULL                    // Pairing state callback
};




/*********************************************************************
 * PUBLIC FUNCTIONS
 */

void PWM_Init( uint8 task_id )
{
  PWM_TaskID = task_id;

  // Setup the GAP Peripheral Role Profile
  {
    uint8 initial_advertising_enable =TRUE;// CHENDY ADD 20200915 TRUE;
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
    uint8 peerPublicAddr[] = {
			0x01,
			0x02,
			0x03,
			0x04,
			0x05,
			0x06
		};

    GAPRole_SetParameter(GAPROLE_ADV_DIRECT_ADDR, sizeof(peerPublicAddr), peerPublicAddr);
    // set adv channel map
    GAPRole_SetParameter(GAPROLE_ADV_CHANNEL_MAP, sizeof(uint8), &advChnMap);        
    
    // Set the GAP Role Parameters
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
    GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );
    
    GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanData ), scanData );
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
      uint16 advInt = 160;   // actual time = advInt * 625us
  
      GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt );
      GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt );
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );
  }
  
	
	
	// Setup the GAP Bond Manager chendy 20200917
  {
    uint32 passkey = DEFAULT_PASSCODE;
    uint8 pairMode = DEFAULT_PAIRING_MODE;
    uint8 mitm = DEFAULT_MITM_MODE;
    uint8 ioCap = DEFAULT_IO_CAPABILITIES;
    uint8 bonding = DEFAULT_BONDING_MODE;
    GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof( uint32 ), &passkey );
    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof( uint8 ), &pairMode );
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof( uint8 ), &mitm );
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof( uint8 ), &ioCap );
    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof( uint8 ), &bonding );
  }
	
	
	
	
  // Initialize GATT attributes
  GGS_AddService( GATT_ALL_SERVICES );         // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES ); // GATT attributes
  DevInfo_AddService( );
  PWMS_AddService(NULL);//PWMS_AddService(PWMCtrlCB);
   // Setup a delayed profile startup
  osal_set_event( PWM_TaskID, START_DEVICE_EVT );
  hal_pwm_module_init();//init pwm module
	pwm_light_init();
}




uint16 PWM_ProcessEvent( uint8 task_id, uint16 events )
{
  
  VOID task_id; // OSAL required parameter that isn't used in this function
  //phy_printf("PWM_ProcessEvent: 0x%x\n",events);
  static uint8 test_flag=0;

  if ( events & START_DEVICE_EVT )
  {
		
    // Start the Device
    VOID GAPRole_StartDevice( &PWMPeripheralCB );	
		
    // Register with bond manager after starting device
    GAPBondMgr_Register( (gapBondCBs_t *) &PWMBondCB );
    
    return ( events ^ START_DEVICE_EVT );
  }

	if ( events & PWM_LIGHT_SET_LIGHTDATA_EVT )
  {
		uint8 cmd;
		LOG("light ctrl data \n");
		cmd=PWM_light_set_data[PWM_LIGHT_CMD2_INDEX];
		switch(cmd)
		{	
			case PWM_LIGHT_MODE_SETTING:
				PWM_LIGHT_Set_work_mode(PWM_light_set_data[PWM_LIGHT_CMD3_INDEX],
			                          PWM_light_set_data[PWM_LIGHT_SPEED_INDEX],
			                          4,
			                          PWM_light_set_data[PWM_LIGHT_RGBW_RED1_INDEX],
			                          PWM_light_set_data[PWM_LIGHT_RGBW_GREEN1_INDEX],
			                          PWM_light_set_data[PWM_LIGHT_RGBW_BLUE1_INDEX],
			                          PWM_light_set_data[PWM_LIGHT_RGBW_WHITE1_INDEX]);
			  test_flag=1;
				break;
    
		  case PWM_LIGHT_TIMING_OFF_ENABLE:
			    {
				   uint32 offtime=0;						
					 offtime=PWM_light_set_data[PWM_LIGHT_TIMING_DATA_INDEX]+
						            ((uint32)(PWM_light_set_data[PWM_LIGHT_TIMING_DATA_INDEX+1])<<8)+
						            ((uint32)(PWM_light_set_data[PWM_LIGHT_TIMING_DATA_INDEX+2])<<16)+
						            ((uint32)(PWM_light_set_data[PWM_LIGHT_TIMING_DATA_INDEX+3])<<24);  
						LOG("offdata:%x,%x,%x,%x\n",PWM_light_set_data[PWM_LIGHT_TIMING_DATA_INDEX],
						                            PWM_light_set_data[PWM_LIGHT_TIMING_DATA_INDEX+1],
						                            PWM_light_set_data[PWM_LIGHT_TIMING_DATA_INDEX+2],
						                            PWM_light_set_data[PWM_LIGHT_TIMING_DATA_INDEX+3]);
						offtime	=	offtime*1000;
						LOG("offtime=%d\n",offtime);
				   osal_start_timerEx(PWM_TaskID,PWM_LIGHT_OFF_TIMING_EVT,offtime);
					}	
		   break;
			
			case PWM_LIGHT_TIMING_OFF_DISABLE:
				   osal_stop_timerEx(PWM_TaskID,PWM_LIGHT_OFF_TIMING_EVT);
			     LOG("disable poweroff timing\n"); 
				   break;
			default:
				LOG("pwm light set error cmd=%x\n",PWM_light_set_data[PWM_LIGHT_CMD2_INDEX]);
				break;			 
		}	
    return ( events ^ PWM_LIGHT_SET_LIGHTDATA_EVT );
  }
	
	if ( events & PWM_LIGHT_INQUIRE_LIGHTDATA_EVT )
  {
		LOG("PWM_LIGHT_INQUIRE_LIGHTDATA_EVT\n");
		
		return ( events ^ PWM_LIGHT_INQUIRE_LIGHTDATA_EVT );
	}
  
		
	if ( events & PWM_LIGTH_PERIOD_TIME_EVT )
  {
		LOG("PT\n");
		if(test_flag)
			test_flag=0;
		else
		{	
		 switch(color_data.work_mode)
	   {
			 case PWM_LIGHT_WORKMODE_SMOOTH_MODE:
				 PWM_light_smooth_mode();
				 break;
			 case PWM_LIGHT_WORKMODE_SEVEN_FLASH_MODE:
				 PWM_light_flash_mode();
				 break;
			 case PWM_LIGHT_WORKMODE_STROBE_MODE:
				 PWM_light_strobe_mode();
				 break;
			 case PWM_LIGHT_WORKMODE_FADE_MODE:
				 PWM_light_fade_mode();
				 break;
			 case PWM_LIGHT_WORKMODE_MORE_FADE_MODE:
				 PWM_light_more_fademode();
				 break;
			 defualt:
			   LOG("error work mode\n");
			   break;
		 }
	 }
		return ( events ^ PWM_LIGTH_PERIOD_TIME_EVT );
	}
	
	if ( events & PWM_LIGHT_OFF_TIMING_EVT )
  {
  		
		PWM_light_all_off();
		LOG("PWM_LIGHT_OFF_TIMING_EVT turn off led\n");
		
		return ( events ^ PWM_LIGHT_OFF_TIMING_EVT );
	}
  // Discard unknown events
  return 0;
}



static void PWMGapStateCB( gaprole_States_t newState )
{
//  phy_printf("PWMGapStateCB: %d", newState);
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

    if ( newState == GAPROLE_WAITING_AFTER_TIMEOUT )
    {
      // link loss timeout-- use fast advertising
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, DEFAULT_FAST_ADV_INTERVAL );
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, DEFAULT_FAST_ADV_INTERVAL );
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_MIN, DEFAULT_FAST_ADV_DURATION );
    }
    else
    {
      // Else use slow advertising
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, DEFAULT_SLOW_ADV_INTERVAL );
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, DEFAULT_SLOW_ADV_INTERVAL );
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_MIN, DEFAULT_SLOW_ADV_DURATION );
    }

    // Enable advertising
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &advState );    
  }    
  // if advertising stopped
  else if ( gapProfileState == GAPROLE_ADVERTISING && 
            newState == GAPROLE_WAITING )
  {
    // if advertising stopped by user
    if ( PWMAdvCancelled )
    {
      PWMAdvCancelled = FALSE;
    }
    // if fast advertising switch to slow
    else if ( GAP_GetParamValue( TGAP_GEN_DISC_ADV_INT_MIN ) == DEFAULT_FAST_ADV_INTERVAL )
    {
      uint8 advState = TRUE;
      
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, DEFAULT_SLOW_ADV_INTERVAL );
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, DEFAULT_SLOW_ADV_INTERVAL );
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_MIN, DEFAULT_SLOW_ADV_DURATION );
      GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &advState );   
    }  
  }
  // if started
  else if (newState == GAPROLE_STARTED)
  {
    // Set the system ID from the bd addr
    uint8 systemId[DEVINFO_SYSTEM_ID_LEN];
    GAPRole_GetParameter(GAPROLE_BD_ADDR, systemId);

		uint8 ownAddress[B_ADDR_LEN];
		uint8 initial_advertising_enable=TRUE;//chendy 20200915
		GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);

LOG("MAC ADDR:\n");
		for(uint8 i=0;i<B_ADDR_LEN;i++)
		{
			 LOG("%x ",ownAddress[B_ADDR_LEN-1-i]);
			advertData[ADVDATA_MAC_ADDR_OFFSET+i]=ownAddress[B_ADDR_LEN-1-i];
		}
		LOG("\n");

		GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );
		GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
    
    DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);
  }
  
  gapProfileState = newState;
}





/*********************************************************************
*********************************************************************/
