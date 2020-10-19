/**************************************************************************************************
THIS IS EMPTY HEADER
**************************************************************************************************/

/**************************************************************************************************
  Filename:       sbpProfile_ota.c
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
#include "simpleBlePeripheral.h"
//#include "log.h"
#include "sbpProfile_ota.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

//#define SERVAPP_NUM_ATTR_SUPPORTED        24

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
// Simple GATT Profile Service UUID: 0xFFF0
CONST uint8 simpleProfileServUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(SIMPLEPROFILE_SERV_UUID), HI_UINT16(SIMPLEPROFILE_SERV_UUID)
};

// Characteristic 1 UUID: 0xFFF1
CONST uint8 simpleProfilechar1UUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(SIMPLEPROFILE_CHAR1_UUID), HI_UINT16(SIMPLEPROFILE_CHAR1_UUID)
};

// Characteristic 2 UUID: 0xFFF2
CONST uint8 simpleProfilechar2UUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(SIMPLEPROFILE_CHAR2_UUID), HI_UINT16(SIMPLEPROFILE_CHAR2_UUID)
};

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

enum{
	IBEACON_SET_DEV_NAME_CMD			=	0x11,
	IBEACON_SET_UUID_CMD				=	0x12,
	IBEACON_SET_MAJOR_CMD				=	0x13,
	IBEACON_SET_MINOR_CMD				=	0x14,
	IBEACON_SET_RSSI_CMD				=	0x15,
	IBEACON_SET_ADV_INTVL_CMD			=	0x16,
	IBEACON_SET_MAX_CMD				=	0x17
}Beacon_CMD_DATA;


enum{
	IBEACON_SET_FAIL					=	0x00,
	IBEACON_SET_SUCCESS				=	0x01,
	IBEACON_SET_DATA_FORMAT_ERROR		=	0x02,
	IBEACON_SET_CHECKSUM_ERROR		=	0x03,
	IBEACON_SET_CONNECT_FAIL			=	0x04
}Beacon_CMD_RSP_DATA;


static simpleProfileCBs_t *simpleProfile_AppCBs = NULL;

/*********************************************************************
 * Profile Attributes - variables
 */

// Simple Profile Service attribute
static CONST gattAttrType_t simpleProfileService	=	{ ATT_BT_UUID_SIZE, simpleProfileServUUID };

// Simple Profile Characteristic 1 Properties
static uint8 simpleProfileChar1Props 			=	GATT_PROP_WRITE;
// Characteristic 1 Value
static uint8 simpleProfileChar1[IBEACON_SET_CMD_DATA_MAX_LEN];//max is 20bytes.
// Simple Profile Characteristic 1 User Description
static uint8 simpleProfileChar1UserDesp[]		=	"Set param\0";


// Simple Profile Characteristic 2 Properties
static uint8 simpleProfileChar2Props			=	GATT_PROP_READ | GATT_PROP_NOTIFY;
// Characteristic 2 Value
static uint8 simpleProfileChar2				=	0;
// Simple Profile Characteristic 2 User Description
static uint8 simpleProfileChar2UserDesp[]		=	"notify\0";
static gattCharCfg_t simpleProfileChar2Config;//[GATT_MAX_NUM_CONN];

/*********************************************************************
 * Profile Attributes - Table
 */
 
ibeacon_store_data_t Ibeacon_store_data;
Beacon_rsp_data_t Beacon_cmd_rsp_data;

//rsp cmd set state
extern void simpleProfile_Set_Notify_Event(void);
void Beacon_set_rsp_data(uint8 cmd,uint8 data)
{
	Beacon_cmd_rsp_data.sop=IBEACON_SET_CMD_RSP_SOP;
	Beacon_cmd_rsp_data.cmd=cmd;
	Beacon_cmd_rsp_data.data=data;
	simpleProfile_Set_Notify_Event();
}	
static uint8 Beacon_check_data_sum(uint8*data,uint8 len)
{
	uint8 temp=0;
	for(uint8 i=0;i<len;i++)
	{
		temp +=data[i];
	}
	return temp;
}


#define BEACON_NOTIFY_VALUE_HANDLE_INDEX     4
static gattAttribute_t simpleProfileAttrTbl[]/*[SERVAPP_NUM_ATTR_SUPPORTED]*/ = 
{
	/* type */								/* permissions */			/* handle */	/* pValue */
	// Simple Profile Service
	{{ ATT_BT_UUID_SIZE, primaryServiceUUID },			GATT_PERMIT_READ,		0,				(uint8 *)&simpleProfileService},

	// Characteristic 1 Declaration
	{{ ATT_BT_UUID_SIZE, characterUUID },				GATT_PERMIT_READ,		0,				&simpleProfileChar1Props},
	// Characteristic Value 1
	{{ ATT_BT_UUID_SIZE, simpleProfilechar1UUID },	GATT_PERMIT_READ | GATT_PERMIT_WRITE,	0,	&simpleProfileChar1[0]},

	// Characteristic 2 Declaration
	{{ ATT_BT_UUID_SIZE, characterUUID },				GATT_PERMIT_READ,		0,				&simpleProfileChar2Props},
	// Characteristic Value 2
	{{ ATT_BT_UUID_SIZE, simpleProfilechar2UUID },	GATT_PERMIT_READ | GATT_PERMIT_WRITE,	0,	(uint8 *)&simpleProfileChar2},
	// Characteristic 2 User Description
//	{{ ATT_BT_UUID_SIZE, charUserDescUUID },			GATT_PERMIT_READ,		0,				simpleProfileChar2UserDesp},           
	// Characteristic 2 configuration
	{{ ATT_BT_UUID_SIZE, clientCharCfgUUID },			GATT_PERMIT_READ | GATT_PERMIT_WRITE,	0,	(uint8 *)&simpleProfileChar2Config}, 

};


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static uint8 simpleProfile_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                            uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen );
static bStatus_t simpleProfile_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                 uint8 *pValue, uint8 len, uint16 offset );

static void simpleProfile_HandleConnStatusCB( uint16 connHandle, uint8 changeType );


/*********************************************************************
 * PROFILE CALLBACKS
 */
// Simple Profile Service Callbacks
CONST gattServiceCBs_t simpleProfileCBs =
{
  simpleProfile_ReadAttrCB,  	// Read callback function pointer
  simpleProfile_WriteAttrCB, 	// Write callback function pointer
  NULL                       	// Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SimpleProfile_AddService
 *
 * @brief   Initializes the Simple Profile service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 *
 * @return  Success or Failure
 */
bStatus_t SimpleProfile_AddService( uint32 services )
{
  uint8 status = SUCCESS;

  // Initialize Client Characteristic Configuration attributes
  //GATTServApp_InitCharCfg( INVALID_CONNHANDLE, &simpleProfileChar2Config );

  // Register with Link DB to receive link status change callback
  VOID linkDB_Register( simpleProfile_HandleConnStatusCB );  
  
  if ( services & SIMPLEPROFILE_SERVICE )
  {
    // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService( simpleProfileAttrTbl, 
                                          GATT_NUM_ATTRS( simpleProfileAttrTbl ),
                                          &simpleProfileCBs );
  }
	LOG("addservic ret=%x\n",status);

  return ( status );
}


/*********************************************************************
 * @fn      SimpleProfile_RegisterAppCBs
 *
 * @brief   Registers the application callback function. Only call 
 *          this function once.
 *
 * @param   callbacks - pointer to application callbacks.
 *
 * @return  SUCCESS or bleAlreadyInRequestedMode
 */
bStatus_t SimpleProfile_RegisterAppCBs( simpleProfileCBs_t *appCallbacks )
{
	if ( appCallbacks )
	{
		simpleProfile_AppCBs = appCallbacks;

		return ( SUCCESS );
	}
	else
	{
		return ( bleAlreadyInRequestedMode );
	}
}
  

/*********************************************************************
 * @fn      SimpleProfile_SetParameter
 *
 * @brief   Set a Simple Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to right
 * @param   value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t SimpleProfile_SetParameter( uint8 param, uint8 len, void *value )
{
	bStatus_t ret = SUCCESS;
	switch ( param )
	{
		case SIMPLEPROFILE_CHAR1:
			if ( len <= IBEACON_UUID_LEN )   
			{
				osal_memcpy(simpleProfileChar1, value, len);				
			}
			else
			{
				ret = bleInvalidRange;
			}
		break;

		case SIMPLEPROFILE_CHAR2:
			if ( len == sizeof ( uint16 ) ) 
			{
				simpleProfileChar2 = (*(uint8 *)value) << 8 | *((uint8 *)value + 1);
			}
			else
			{
				ret = bleInvalidRange;
			}
		break;

		default:
			ret = INVALIDPARAMETER;
		break;
	}
	return ( ret );
}

/*********************************************************************
 * @fn      SimpleProfile_GetParameter
 *
 * @brief   Get a Simple Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to put.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t SimpleProfile_GetParameter( uint8 param, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case SIMPLEPROFILE_CHAR1:
      VOID osal_memcpy( value, simpleProfileChar1, IBEACON_UUID_LEN );    
      break;

    case SIMPLEPROFILE_CHAR2:
      *((uint16*)value) = simpleProfileChar2;
      break;      

    default:
      ret = INVALIDPARAMETER;
      break;
  }
  
  return ( ret );
}

/*********************************************************************
 * @fn          simpleProfile_ReadAttrCB
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
static uint8 simpleProfile_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                            uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen )
{
  bStatus_t status = SUCCESS;

  // If attribute permissions require authorization to read, return error
  if ( gattPermitAuthorRead( pAttr->permissions ) )
  {
    // Insufficient authorization
    return ( ATT_ERR_INSUFFICIENT_AUTHOR );
  }
  
  // Make sure it's not a blob operation (no attributes in the profile are long)
  if ( offset > 0 )
  {
    return ( ATT_ERR_ATTR_NOT_LONG );
  }
 
  if ( pAttr->type.len == ATT_BT_UUID_SIZE )
  {
    // 16-bit UUID
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch ( uuid )
    {
      // No need for "GATT_SERVICE_UUID" or "GATT_CLIENT_CHAR_CFG_UUID" cases;
      // gattserverapp handles those reads

      case SIMPLEPROFILE_CHAR1_UUID:
//	      *pLen = IBEACON_UUID_LEN;
//	      VOID osal_memcpy( pValue, pAttr->pValue, IBEACON_UUID_LEN );
	      break;
      case SIMPLEPROFILE_CHAR2_UUID:
     
	      *pLen = 2;
	      VOID osal_memcpy( pValue, pAttr->pValue, *pLen );
	      break;       
        
      default:
	      // Should never get here! (characteristics 3 and 4 do not have read permissions)
	      *pLen = 0;
	      status = ATT_ERR_ATTR_NOT_FOUND;
	      break;
    }
  }
  else
  {
    // 128-bit UUID
    *pLen = 0;
    status = ATT_ERR_INVALID_HANDLE;
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
 // TODO: test this function
static bStatus_t simpleProfile_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                 uint8 *pValue, uint8 len, uint16 offset )
{
	bStatus_t status = SUCCESS;
	uint8 notifyApp = 0xFF;

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
		switch ( uuid )
		{
			case SIMPLEPROFILE_CHAR1_UUID:
				if ( offset > 0 )
				{
					status = ATT_ERR_ATTR_NOT_LONG;
				}
				else
				{	
					uint8 sum		=	0;	
					uint8 sum_index	=	0;					
					uint8 cmd		=	0;
					uint8 cmd_sop	=	0;
					uint8 cmd_len	=	0;
					uint8 status	=	IBEACON_SET_SUCCESS;
					cmd				=	pValue[IBEACON_SET_CMD_DATA_INDEX];
					cmd_len			=	pValue[IBEACON_SET_CMD_DATA_LEN_INDEX];
					cmd_sop			=	pValue[0];
					if((cmd<IBEACON_SET_DEV_NAME_CMD)||(cmd>=IBEACON_SET_MAX_CMD)||(cmd_sop !=IBEACON_SET_CMD_SOP))
					{
						uint8 rsp;
						rsp=IBEACON_SET_DATA_FORMAT_ERROR;						
						Beacon_set_rsp_data(pValue[2],rsp);
						LOG("the cmd cann't support\n");
						break;
					}
					sum=Beacon_check_data_sum(pValue,len-1);
					if(sum!=pValue[len-1])//check sum
					{
						uint8 rsp=0;
						rsp=IBEACON_SET_CHECKSUM_ERROR;
						LOG("Check sumerror sum=%x  errorsum=%x\n",sum,pValue[len-1]);
						Beacon_set_rsp_data(pValue[2],rsp);
						break;
					}					

					switch(cmd)//(pValue[IBEACON_SET_CMD_DATA_INDEX])
					{
						case IBEACON_SET_DEV_NAME_CMD:
							if(cmd_len>IBEACON_DEV_NAME_MAX_LEN)
							{
								status=IBEACON_SET_DATA_FORMAT_ERROR;
							}
							else
							{									 
								SimpleBLEPeripheral_SetDevName(&pValue[IBEACON_SET_DATA_INDEX],cmd_len);									 
							}									 
							LOG("set dev name\n");
						break;
						case IBEACON_SET_UUID_CMD:
							if(cmd_len !=IBEACON_UUID_LEN)
							{
								status=IBEACON_SET_DATA_FORMAT_ERROR;
							}
							else
							{
								extern void SimpleBLEPeripheral_SetDevName(uint8*data,uint8 len);
								SimpleBLEPeripheral_SetBeaconUUID(&pValue[IBEACON_SET_DATA_INDEX],cmd_len);
								//Beacon_set_rsp_data(cmd,IBEACON_SET_SUCCESS);
							}				
							LOG("set uuid\n");
						break;			
						case IBEACON_SET_MAJOR_CMD:
							if(cmd_len !=IBEACON_VERSION_LEN)
							{
								status=IBEACON_SET_DATA_FORMAT_ERROR;
							}
							else
							{	 
								SimpleBLEPeripheral_SetMajor(&pValue[IBEACON_SET_DATA_INDEX],cmd_len);;
							}
							LOG("set major\n");
						break;	
						case IBEACON_SET_MINOR_CMD:
							if(cmd_len !=IBEACON_VERSION_LEN)
							{
								status=IBEACON_SET_DATA_FORMAT_ERROR;
							}
							else
							{	 
								SimpleBLEPeripheral_SetMinor(&pValue[IBEACON_SET_DATA_INDEX],cmd_len);;
							}
							LOG("set minor\n");
						break;				
						case IBEACON_SET_RSSI_CMD:
							if(cmd_len !=IBEACON_RSSI_LEN)
							{
								status=IBEACON_SET_DATA_FORMAT_ERROR;
							}
							else
							{	 
								SimpleBLEPeripheral_SetRSSI(0x0100 - pValue[IBEACON_SET_DATA_INDEX]);
							}
							LOG("set RSSI=%d\n",pValue[IBEACON_SET_DATA_INDEX]);
						break;	

						case IBEACON_SET_ADV_INTVL_CMD:

							if(cmd_len !=IBEACON_ADV_INTVL_LEN)
							{
								status=IBEACON_SET_DATA_FORMAT_ERROR;
							}
							else
							{	 
								uint16 adv_intvl=0;
								adv_intvl=pValue[IBEACON_SET_DATA_INDEX]+(pValue[IBEACON_SET_DATA_INDEX+1]<<8);
								SimpleBLEPeripheral_SetAdvIntvlTime(adv_intvl);
								LOG("adv interval=%x,%d\n",adv_intvl,adv_intvl);
							}				
						break;

						default:
							LOG("cmd set erro\n");
						break;
					}	
					Beacon_set_rsp_data(cmd,status);
					LOG("write ctrl datalen=%d\n",len);
				}	
			break;			
			
			case SIMPLEPROFILE_CHAR2_UUID:	  			  	
			break;	  	

			case GATT_CLIENT_CHAR_CFG_UUID:
				status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
				                             offset, GATT_CLIENT_CFG_NOTIFY );
			break;

			default:
				status = ATT_ERR_ATTR_NOT_FOUND;
			break;
		}
	}
	else
	{
		// 128-bit UUID
		status = ATT_ERR_INVALID_HANDLE;
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
static void simpleProfile_HandleConnStatusCB( uint16 connHandle, uint8 changeType )
{ 
	// Make sure this is not loopback connection
	if ( connHandle != LOOPBACK_CONNHANDLE )
	{
		// Reset Client Char Config if connection has dropped
		if ( ( changeType == LINKDB_STATUS_UPDATE_REMOVED )      ||
		     ( ( changeType == LINKDB_STATUS_UPDATE_STATEFLAGS ) && 
		       ( !linkDB_Up( connHandle ) ) ) )
		{ 
			//   GATTServApp_InitCharCfg( connHandle, simpleProfileChar4Config );
		}
	}
}


bStatus_t simpleProfile_Notify( uint16 connHandle,uint8*data,uint8 len)
{
	bStatus_t ret = SUCCESS;
	uint16 notfEnable;
	attHandleValueNoti_t notify_data={0};

	uint16 ccd_value = simpleProfileChar2Config.value;

	// If notifications enabled
	if ( ccd_value & GATT_CLIENT_CFG_NOTIFY )
	{
		// Set the handle
		osal_memcpy(notify_data.value,data, len);
		notify_data.len = len;
		notify_data.handle = simpleProfileAttrTbl[BEACON_NOTIFY_VALUE_HANDLE_INDEX].handle;

		// Send notify
		ret= GATT_Notification( connHandle, &notify_data, FALSE);
	}
	LOG("NOTIFccd=%x,ret=%x",ccd_value,ret);		
	return ( ret );
}


/*********************************************************************
*********************************************************************/
