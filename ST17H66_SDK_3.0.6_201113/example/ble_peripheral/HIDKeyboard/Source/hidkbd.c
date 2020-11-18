/**************************************************************************************************
*******
**************************************************************************************************/

/******************************************************************************


 *****************************************************************************/


/*********************************************************************
 * INCLUDES
 */

#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "gatt.h"
#include "hci.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "gatt_profile_uuid.h"
#include "linkdb.h"
#include "peripheral.h"
#include "gapbondmgr.h"
#include "peripheral.h"
#include "hidkbdservice.h"
//#include "devinfoservice.h"
#include "hiddev.h"
#include "ota_app_service.h"
#include "global_config.h"
#include "hidkbd.h"

#include "ll.h"
#include "ll_common.h"
#include "ll_def.h"
#include "log.h"

#include "OSAL_Memory.h"
//#include "hal_mcu.h"
/*********************************************************************
 * MACROS
 */

// Selected HID keycodes
#define KEY_RIGHT_ARROW             0x4F
#define KEY_LEFT_ARROW              0x50
#define KEY_NONE                    0x00

// Selected HID LED bitmaps
#define LED_NUM_LOCK                0x01
#define LED_CAPS_LOCK               0x02

// Selected HID mouse button values
#define MOUSE_BUTTON_1              0x01
#define MOUSE_BUTTON_NONE           0x00

// HID keyboard input report length
#define HID_KEYBOARD_IN_RPT_LEN     8

// HID LED output report length
#define HID_LED_OUT_RPT_LEN         1

// HID mouse input report length
#define HID_MOUSE_IN_RPT_LEN        5

#if EN_CONSUMER_MODE
// HID consumer control input report length
#define HID_CC_IN_RPT_LEN                     2//2//1

#endif
/*********************************************************************
 * CONSTANTS
 */

// HID idle timeout in msec; set to zero to disable timeout
#define DEFAULT_HID_IDLE_TIMEOUT              0

// Minimum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     10//10

// Maximum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     20//10

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          500

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         TRUE//TRUE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         6

// Default passcode
#define DEFAULT_PASSCODE                      0

// Default GAP pairing mode
//#define DEFAULT_PAIRING_MODE                  GAPBOND_PAIRING_MODE_INITIATE
#define DEFAULT_PAIRING_MODE                  GAPBOND_PAIRING_MODE_WAIT_FOR_REQ

// Default MITM mode (TRUE to require passcode or OOB when pairing)
#define DEFAULT_MITM_MODE                     FALSE

// Default bonding mode, TRUE to bond
#define DEFAULT_BONDING_MODE                  TRUE

// Default GAP bonding I/O capabilities
#define DEFAULT_IO_CAPABILITIES               GAPBOND_IO_CAP_NO_INPUT_NO_OUTPUT

// Battery level is critical when it is less than this %
#define DEFAULT_BATT_CRITICAL_LEVEL           6

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
extern gaprole_States_t hidDevGapState;

// Task ID
uint8 hidKbdTaskId;
uint8 g_instant_cnt;

/*********************************************************************
 * EXTERNAL VARIABLES
 */


/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */


// GAP Profile - Name attribute for SCAN RSP data
static uint8 scanData[] =
{
    // appearance
    0x03,   // length of this data
    GAP_ADTYPE_APPEARANCE,
    LO_UINT16(GAP_APPEARE_HID_GAMEPAD),
    HI_UINT16(GAP_APPEARE_HID_GAMEPAD),

    // service UUIDs
    0x03,   // length of this data
    GAP_ADTYPE_16BIT_COMPLETE,
    LO_UINT16(HID_SERV_UUID),
    HI_UINT16(HID_SERV_UUID),
    0x0D,                             // length of this data
    GAP_ADTYPE_LOCAL_NAME_COMPLETE,   // AD Type = Complete local name
    'H',
    'I',
    'D',
    ' ',
    'K',
    'e',
    'y',
    'b',
    'o',
    'a',
    'r',
    'd'
};

// Advertising data
static uint8 advData[] =
{
    // flags
    0x02,   // length of this data
    GAP_ADTYPE_FLAGS,
    GAP_ADTYPE_FLAGS_LIMITED | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

    // appearance
    0x03,   // length of this data
    GAP_ADTYPE_APPEARANCE,
    LO_UINT16(GAP_APPEARE_HID_GAMEPAD),
    HI_UINT16(GAP_APPEARE_HID_GAMEPAD),

    // service UUIDs
    0x03,   // length of this data
    GAP_ADTYPE_16BIT_COMPLETE,
    LO_UINT16(HID_SERV_UUID),
    HI_UINT16(HID_SERV_UUID),
	
	  0x04,
	  GAP_ADTYPE_OOB_CLASS_OF_DEVICE,
	  0x04,
	  0x05,
		0x00,

};


// Device name attribute value
static CONST uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "HID Keyboard";

// HID Dev configuration
static hidDevCfg_t hidKbdCfg =
{
  DEFAULT_HID_IDLE_TIMEOUT,   // Idle timeout
  HID_KBD_FLAGS               // HID feature flags
};

// TRUE if boot mouse enabled
uint8 hidBootMouseEnabled = FALSE;



/*********************************************************************
 * LOCAL FUNCTIONS
 */



static void hidKbd_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void hidKbd_ProcessGattMsg( gattMsgEvent_t *pMsg );
void hidKbdSendReport( uint8 keycode );
void hidKbdSendMouseReport( uint8 buttons,uint8 x,uint8 y );
static uint8 hidKbdRcvReport( uint8 len, uint8 *pData );
static uint8 hidKbdRptCB( uint8 id, uint8 type, uint16 uuid,
                          uint8 oper, uint8 *pLen, uint8 *pData );
static void hidKbdEvtCB( uint8 evt );
#if EN_CONSUMER_MODE
static void hidCCSendReport( uint8 cmd, bool keyPressed, uint8 keyRepeated );
void hidCCSendReportKey( uint8 cmd, bool keyPressed);

#endif




/*********************************************************************
 * PROFILE CALLBACKS
 */

static hidDevCB_t hidKbdHidCBs =
{
    hidKbdRptCB,
    hidKbdEvtCB,
    NULL
};



/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      HidEmuKbd_Init
 *
 * @brief   Initialization function for the HidEmuKbd App Task.
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
void HidKbd_Init( uint8 task_id )
{
  hidKbdTaskId = task_id;
  LOG("%s\n",__FUNCTION__);
  // Setup the GAP
  VOID GAP_SetParamValue( TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL );

    // calibration time for 2 connection event, will advance the next conn event receive window
    // SLAVE_CONN_DELAY for sync catch, SLAVE_CONN_DELAY_BEFORE_SYNC for sync not catch
    // pGlobal_config[SLAVE_CONN_DELAY] = 500;//0;//1500;//0;//3000;//0;          ---> update 11-20
    // pGlobal_config[SLAVE_CONN_DELAY_BEFORE_SYNC] = 1100;  //800-1100


  // Setup the GAP Peripheral Role Profile
  {
    uint8 initial_advertising_enable = TRUE;

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16 gapRole_AdvertOffTime = 0;

    uint8 enable_update_request = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16 desired_min_interval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16 desired_max_interval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16 desired_slave_latency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16 desired_conn_timeout = DEFAULT_DESIRED_CONN_TIMEOUT;

    // Set the GAP Role Parameters
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
    GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );

    GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advData ), advData );
    GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanData ), scanData );

    GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
    GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
    GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
    GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
    GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );
  }
    uint8 appearence_data[2];
    appearence_data[0]=LO_UINT16(GAP_APPEARE_HID_GAMEPAD);//GAP_APPEARE_HID_KEYBOARD
    appearence_data[1]=HI_UINT16(GAP_APPEARE_HID_GAMEPAD);//GAP_APPEARE_HID_KEYBOARD
    GGS_SetParameter( GGS_APPEARANCE_ATT, 2, (void *) appearence_data );

  // Set the GAP Characteristics
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, (void *) attDeviceName );

  // Setup the GAP Bond Manager
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


  {
    // Use the same interval for general and limited advertising.
    // Note that only general advertising will occur based on the above configuration
    uint16_t advInt = 80;//160

    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, advInt);
  }

  // Set up HID keyboard service
  HidKbd_AddService();
  // Register for HID Dev callback
  HidDev_Register(&hidKbdCfg, &hidKbdHidCBs);

  ATT_SetMTUSizeMax(23);
  llInitFeatureSet2MPHY(FALSE);
  llInitFeatureSetDLE(FALSE);

  ota_app_AddService();
  // HCI_PPLUS_ExtendTRXCmd(1);

	 
	// hal_gpio_pin_init(P14,IE);
	// hal_gpio_pull_set(P14,STRONG_PULL_UP);
	
	
	// hal_gpioin_register(P14,P14_POS_Callback,P14_NEG_Callback);
	
	
	
	
	
  
  // Setup a delayed profile startup
  osal_set_event( hidKbdTaskId, START_DEVICE_EVT );
}

/*********************************************************************
 * @fn      HidEmuKbd_ProcessEvent
 *
 * @brief   HidEmuKbd Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 HidKbd_ProcessEvent( uint8 task_id, uint16 events )
{

  VOID task_id; // OSAL required parameter that isn't used in this function
  LOG("%s\n",__FUNCTION__);

  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( hidKbdTaskId )) != NULL )
    {
      hidKbd_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

    if ( events & START_DEVICE_EVT )
    {

        //  osal_start_timerEx(hidKbdTaskId, HID_KEY_TEST_EVT, 5000);
			
		//	 osal_start_timerEx(hidKbdTaskId,SBP_GPIO_POLL_EVT,2000);

        return ( events ^ START_DEVICE_EVT );
    }


    if ( events & HID_TEST_EVT )
    {


       // hidKbdSendReport(HID_KEYBOARD_2);
       // hidKbdSendReport(0);

				hidCCSendReport(0,1,0);
				hidCCSendReport(0,0,0);


        if(hidDevGapState==GAPROLE_CONNECTED)
        {
            osal_start_timerEx(hidKbdTaskId, HID_TEST_EVT, 5000);
		
        }
    return ( events ^ HID_TEST_EVT );
  }

   return 0;
}

/*********************************************************************
 * @fn      hidKbd_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void hidKbd_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
    unsigned char *msg;
    uint16 Instant=0;
  switch ( pMsg->event )
  {

    case GATT_MSG_EVENT:
      hidKbd_ProcessGattMsg( (gattMsgEvent_t *)pMsg );
      break;
    case 0XB2:

        g_instant_cnt++;
        msg=(unsigned char *)pMsg;
        Instant=(msg[2]<<8)|msg[3];
		    LOG("********Instant=%d--g_instant_cnt=%d\n\r",Instant,g_instant_cnt);

        if(Instant>200&&g_instant_cnt==1)
        {
            GAPRole_TerminateConnection();

        }

        if(g_instant_cnt>200)
            g_instant_cnt=200;
    default:
      break;
  }
}


/*********************************************************************
 * @fn      hidKbd_ProcessGattMsg
 *
 * @brief   Process GATT messages
 *
 * @return  none
 */
static void hidKbd_ProcessGattMsg( gattMsgEvent_t *pMsg )
{
// ble_ancs_handle_gatt_event(pMsg);
    //GATT_bm_free( &pMsg->msg, pMsg->method );
}

/*********************************************************************
 * @fn      hidKbdSendReport
 *
 * @brief   Build and send a HID keyboard report.
 *
 * @param   keycode - HID keycode.
 *
 * @return  none
 */
void hidKbdSendReport( uint8 keycode )
{
  uint8 buf[HID_KEYBOARD_IN_RPT_LEN];

  buf[0] = 0;         // Modifier keys
  buf[1] = 0;         // Reserved
  buf[2] = keycode;   // Keycode 1
  buf[3] = 0;         // Keycode 2
  buf[4] = 0;         // Keycode 3
  buf[5] = 0;         // Keycode 4
  buf[6] = 0;         // Keycode 5
  buf[7] = 0;         // Keycode 6

  HidDev_Report( HID_RPT_ID_KEY_IN, HID_REPORT_TYPE_INPUT,
                HID_KEYBOARD_IN_RPT_LEN, buf );
}

/*********************************************************************
 * @fn      hidKbdSendMouseReport
 *
 * @brief   Build and send a HID mouse report.
 *
 * @param   buttons - Mouse button code
 *
 * @return  none
 */
void hidKbdSendMouseReport( uint8 buttons,uint8 x,uint8 y )
{
  uint8 buf[HID_MOUSE_IN_RPT_LEN];

    buf[0] = buttons;   // Buttons
    buf[1] = x;         // X
    buf[2] = y;         // Y
    buf[3] = 0;         // Wheel
    buf[4] = 0;         // AC Pan

  HidDev_Report( HID_RPT_ID_MOUSE_IN, HID_REPORT_TYPE_INPUT,
                 HID_MOUSE_IN_RPT_LEN, buf );
}

uint8 hidKbdSendVoiceCMDtReport( uint8 keycode )
{
    uint8 status;
    //  uint8 buf[1];

//   buf[0] = keycode;         // Modifier keys
//   status=AudioProfile_SetParameter(AUDIOPROFILE_CHAR1, 1,buf);
// AudioProfile_SetParameter(AUDIOPROFILE_CHAR2, 1,buf);

    return status;
}

#if EN_CONSUMER_MODE
/*********************************************************************
 * @fn      hidCCSendReport
 *
 * @brief   Build and send a HID Consumer Control report.
 *
 * @param   cmd - bitmap of left/middle/right mouse button states
 * @param   keyPressed - amount of mouse movement along X-axis in units of Mickeys (1/200 of an inch)
 * @param   keyRepeated - amount of mouse movement along Y-axis
 *
 * @return  none
 */
static void hidCCSendReport( uint8 cmd, bool keyPressed, uint8 keyRepeated )
{

    // Only send the report if something meaningful to report
    if ( keyRepeated == 0 )
    {
        uint8 buf[HID_CC_IN_RPT_LEN] = {0};

        // No need to include Report Id
        if ( keyPressed )
        {
            buf[0]=0X40;
            buf[1]=0X00;
        }
        else
        {
            buf[0]=0X00;
            buf[1]=0X00;
        }



        HidDev_Report( HID_RPT_ID_CC_IN, HID_REPORT_TYPE_INPUT,
                       HID_CC_IN_RPT_LEN, buf );
    }
}
#endif
void hidCCSendReportKey( uint8 cmd, bool keyPressed)
{
    uint8 buf[1] = {0};

    if(keyPressed==1)
    {
        buf[0]=cmd;
    }
    else
        buf[0]=0;

    HidDev_Report( HID_RPT_ID_CC_IN, HID_REPORT_TYPE_INPUT,
                   1, buf );

}


/*********************************************************************
 * @fn      hidKbdRcvReport
 *
 * @brief   Process an incoming HID keyboard report.
 *
 * @param   len - Length of report.
 * @param   pData - Report data.
 *
 * @return  status
 */
static uint8 hidKbdRcvReport( uint8 len, uint8 *pData )
{
  // verify data length
  if ( len == HID_LED_OUT_RPT_LEN )
  {
    // set keyfob LEDs
    //HalLedSet( HAL_LED_1, ((*pData & LED_CAPS_LOCK) == LED_CAPS_LOCK) );
    //HalLedSet( HAL_LED_2, ((*pData & LED_NUM_LOCK) == LED_NUM_LOCK) );

    return SUCCESS;
  }
  else
  {
    return ATT_ERR_INVALID_VALUE_SIZE;
  }
}

/*********************************************************************
 * @fn      hidKbdRptCB
 *
 * @brief   HID Dev report callback.
 *
 * @param   id - HID report ID.
 * @param   type - HID report type.
 * @param   uuid - attribute uuid.
 * @param   oper - operation:  read, write, etc.
 * @param   len - Length of report.
 * @param   pData - Report data.
 *
 * @return  GATT status code.
 */
static uint8 hidKbdRptCB( uint8 id, uint8 type, uint16 uuid,
                             uint8 oper, uint8 *pLen, uint8 *pData )
{
  uint8 status = SUCCESS;

  // write
  if ( oper == HID_DEV_OPER_WRITE )
  {
    if ( uuid == REPORT_UUID )
    {
      // process write to LED output report; ignore others
      if ( type == HID_REPORT_TYPE_OUTPUT )
      {
        status = hidKbdRcvReport( *pLen, pData );
      }
    }

    if ( status == SUCCESS )
    {
      status = HidKbd_SetParameter( id, type, uuid, *pLen, pData );
    }
  }
  // read
  else if ( oper == HID_DEV_OPER_READ )
  {
    status = HidKbd_GetParameter( id, type, uuid, pLen, pData );
  }
  // notifications enabled
  else if ( oper == HID_DEV_OPER_ENABLE )
  {
    if ( id == HID_RPT_ID_MOUSE_IN && type == HID_REPORT_TYPE_INPUT )
    {
      hidBootMouseEnabled = TRUE;
    }
  }
  // notifications disabled
  else if ( oper == HID_DEV_OPER_DISABLE )
  {
    if ( id == HID_RPT_ID_MOUSE_IN && type == HID_REPORT_TYPE_INPUT )
    {
      hidBootMouseEnabled = FALSE;
    }
  }

  return status;
}

/*********************************************************************
 * @fn      hidKbdEvtCB
 *
 * @brief   HID Dev event callback.
 *
 * @param   evt - event ID.
 *
 * @return  HID response code.
 */
static void hidKbdEvtCB( uint8 evt )
{
  // process enter/exit suspend or enter/exit boot mode

  return;
}


/*********************************************************************
*********************************************************************/
