
/**************************************************************************************************
  Filename:       bleuart_service.c
  Revised:         
  Revision:       

  Description:    This file contains the Simple GATT profile sample GATT service 
                  profile for use with the BLE sample application.

 
**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "bcomdef.h"
#include "OSAL.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "gapbondmgr.h"

#include "uart.h"
#include "log.h"

#include "bleuart.h"
#include "bleuart_service.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

#define SERVAPP_NUM_ATTR_SUPPORTED        7

#define RAWPASS_TX_VALUE_HANDLE     4
#define RAWPASS_RX_VALUE_HANDLE     2
/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
// Rawpass GATT Profile Service UUID
CONST uint8 bleuart_ServiceUUID[ATT_UUID_SIZE] =
{0x55, 0xe4,0x05,0xd2,0xaf,0x9f,0xa9,0x8f,0xe5,0x4a,0x7d,0xfe,0x43,0x53,0x53,0x49};

// Characteristic rx uuid
CONST uint8 bleuart_RxCharUUID[ATT_UUID_SIZE] =
{0xb3,0x9b,0x72,0x34,0xbe,0xec, 0xd4,0xa8,0xf4,0x43,0x41,0x88,0x43,0x53,0x53,0x49};

// Characteristic tx uuid
CONST uint8 bleuart_TxCharUUID[ATT_UUID_SIZE] =
{0x16,0x96,0x24,0x47,0xc6,0x23, 0x61,0xba,0xd9,0x4b,0x4d,0x1e,0x43,0x53,0x53,0x49};

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

static bleuart_ProfileChangeCB_t bleuart_AppCBs = NULL;

/*********************************************************************
 * Profile Attributes - variables
 */

// Profile Service attribute
static CONST gattAttrType_t bleuart_Service = { ATT_UUID_SIZE, bleuart_ServiceUUID };


// Profile Characteristic 1 Properties
static uint8 bleuart_RxCharProps = GATT_PROP_WRITE_NO_RSP| GATT_PROP_WRITE;

// Characteristic 1 Value
static uint8 bleuart_RxCharValue[RAWPASS_RX_BUFF_SIZE];// = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0,};


// Profile Characteristic 2 Properties
static uint8 bleuart_TxCharProps = GATT_PROP_NOTIFY| GATT_PROP_INDICATE;

// Characteristic 2 Value
static uint8 bleuart_TxCharValue = 0;

// Simple Profile Characteristic 2 User Description
static gattCharCfg_t bleuart_TxCCCD;

/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t bleuart_ProfileAttrTbl[] = 
{
  // Simple Profile Service
  { 
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8 *)&bleuart_Service            /* pValue */
  },

    // Characteristic 1 Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &bleuart_RxCharProps 
    },

      // Characteristic Value 1
      { 
        { ATT_UUID_SIZE, bleuart_RxCharUUID },
        GATT_PERMIT_WRITE, 
        0, 
        &bleuart_RxCharValue[0] 
      },

    // Characteristic 2 Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &bleuart_TxCharProps 
    },

      // Characteristic Value 2
      { 
        { ATT_UUID_SIZE, bleuart_TxCharUUID },
        0, 
        0, 
        (uint8 *)&bleuart_TxCharValue 
      },

      // Characteristic 2 User Description
      { 
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ|GATT_PERMIT_WRITE, 
        0, 
        (uint8*)&bleuart_TxCCCD 
      },          

};


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static uint8 bleuart_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                            uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen );
static bStatus_t bleuart_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                 uint8 *pValue, uint8 len, uint16 offset );

static void bleuart_HandleConnStatusCB ( uint16 connHandle, uint8 changeType );


/*********************************************************************
 * PROFILE CALLBACKS
 */
// Simple Profile Service Callbacks
CONST gattServiceCBs_t bleuart_ProfileCBs =
{
  bleuart_ReadAttrCB,  // Read callback function pointer
  bleuart_WriteAttrCB, // Write callback function pointer
  NULL                       // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      bleuart_AddService
 *
 * @brief   Initializes the Simple Profile service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 *
 * @return  Success or Failure
 */
bStatus_t bleuart_AddService( bleuart_ProfileChangeCB_t cb)
{
  uint8 status = SUCCESS;

  // Initialize Client Characteristic Configuration attributes
  //GATTServApp_InitCharCfg( INVALID_CONNHANDLE, simpleProfileChar4Config );

  // Register with Link DB to receive link status change callback
  VOID linkDB_Register( bleuart_HandleConnStatusCB  );  
  
    bleuart_TxCCCD.connHandle = INVALID_CONNHANDLE;
    bleuart_TxCCCD.value = 0;
    // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService( bleuart_ProfileAttrTbl, 
                                          GATT_NUM_ATTRS( bleuart_ProfileAttrTbl ),
                                          &bleuart_ProfileCBs );
    if(status!=SUCCESS)
        LOG("Add rawpass service failed!\n");
    bleuart_AppCBs = cb;

  return ( status );
}



/*********************************************************************
 * @fn          bleuart_ReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 *
 * @return      Success or Failure
 */
static uint8 bleuart_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                            uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen )
{
    bStatus_t status = SUCCESS;
    LOG("ReadAttrCB\n");
    // If attribute permissions require authorization to read, return error
    if ( gattPermitAuthorRead( pAttr->permissions ) )
    {
        // Insufficient authorization
        return ( ATT_ERR_INSUFFICIENT_AUTHOR );
    }

    // Make sure it's not a blob operation (no attributes in the profile are long) 
    if ( pAttr->type.len == ATT_BT_UUID_SIZE )
    {
        // 16-bit UUID
        uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
        if(uuid == GATT_CLIENT_CHAR_CFG_UUID)
        {
            *pLen = 2;
            osal_memcpy(pValue, pAttr->pValue, 2);
        }
    }
    else
    {
        if(!osal_memcmp(pAttr->type.uuid, bleuart_TxCharUUID, 16))
        {
            *pLen = 1;
            pValue[0] = '1';
        }
        else if(!osal_memcmp(pAttr->type.uuid, bleuart_RxCharUUID, 16))
        {
            LOG("read tx char\n");
        }
    }

    return ( status );
}

/*********************************************************************
 * @fn      simpleProfile_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 *
 * @return  Success or Failure
 */

static bStatus_t bleuart_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                 uint8 *pValue, uint8 len, uint16 offset )
{
    bStatus_t status = SUCCESS;
    //uint8 notifyApp = 0xFF;
    // If attribute permissions require authorization to write, return error
    if ( gattPermitAuthorWrite( pAttr->permissions ) )
    {
        // Insufficient authorization
        return ( ATT_ERR_INSUFFICIENT_AUTHOR );
    }
  
    if ( pAttr->type.len == ATT_BT_UUID_SIZE )
    {
        // 16-bit UUID
        uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
        if(uuid == GATT_CLIENT_CHAR_CFG_UUID)
        {
          status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                                   offset, GATT_CLIENT_CFG_NOTIFY );
          if ( status == SUCCESS && bleuart_AppCBs)
          {
            uint16 charCfg = BUILD_UINT16( pValue[0], pValue[1] );
            bleuart_Evt_t evt;
          
            LOG("CCCD set: [%d]\n", charCfg);
            evt.ev = (charCfg == GATT_CFG_NO_OPERATION)?bleuart_EVT_TX_NOTI_DISABLED:bleuart_EVT_TX_NOTI_ENABLED;
            bleuart_AppCBs(&evt);
          }
        }

    }
    else
    {
        // 128-bit UUID
        if(pAttr->handle == bleuart_ProfileAttrTbl[RAWPASS_RX_VALUE_HANDLE].handle)
        {
          if(bleuart_AppCBs){
            bleuart_Evt_t evt;
            evt.ev = bleuart_EVT_BLE_DATA_RECIEVED;
            evt.param = (uint16_t)len;
            evt.data = pValue;
            bleuart_AppCBs(&evt);
          }
        }
    }
    return ( status );
}

/*********************************************************************
 * @fn          simpleProfile_HandleConnStatusCB
 *
 * @brief       Simple Profile link status change handler function.
 *
 * @param       connHandle - connection handle
 * @param       changeType - type of change
 *
 * @return      none
 */
static void bleuart_HandleConnStatusCB ( uint16 connHandle, uint8 changeType )
{ 
    // Make sure this is not loopback connection
    if ( connHandle != LOOPBACK_CONNHANDLE )
    {
        // Reset Client Char Config if connection has dropped
        if ( ( changeType == LINKDB_STATUS_UPDATE_REMOVED )      ||
                ( ( changeType == LINKDB_STATUS_UPDATE_STATEFLAGS ) && 
                ( !linkDB_Up( connHandle ) ) ) )
        { 
            bleuart_TxCCCD.value = 0;
            LOG("clear client configuration\n");
        }
    }
}

uint8 bleuart_NotifyIsReady(void)
{
    return (bleuart_TxCCCD.value == GATT_CLIENT_CFG_NOTIFY);
}
/*********************************************************************
 * @fn          BloodPressure_IMeasNotify
 *
 * @brief       Send a notification containing a bloodPressure
 *              measurement.
 *
 * @param       connHandle - connection handle
 * @param       pNoti - pointer to notification structure
 *
 * @return      Success or Failure
 */
bStatus_t bleuart_Notify( uint16 connHandle, attHandleValueNoti_t *pNoti, uint8 taskId )
{
  uint16 value = bleuart_TxCCCD.value;

  // If notifications enabled
  if ( value & GATT_CLIENT_CFG_NOTIFY )
  {
    // Set the handle
    pNoti->handle = bleuart_ProfileAttrTbl[RAWPASS_TX_VALUE_HANDLE].handle;
  
    // Send the Indication
    return GATT_Notification( connHandle, pNoti, FALSE);
   
  }
  return bleIncorrectMode;
  
}

/*********************************************************************
*********************************************************************/

