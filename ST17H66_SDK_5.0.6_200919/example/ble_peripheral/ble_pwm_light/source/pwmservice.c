

/**************************************************************************************************
  Filename:       heartrateservice.c
  Revised:        
  Revision:       

  Description:    This file contains the Heart Rate sample service 
                  for use with the Heart Rate sample application.

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
#include "gatt_profile_uuid.h"
#include "gattservapp.h"
#include "peripheral.h"
#include "pwmservice.h"
#include "pwmdemo.h"
#include "color_mode.h"
#include "log.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */


/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
// PWM light service
CONST uint8 PWMServUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(PWM_SERV_UUID), HI_UINT16(PWM_SERV_UUID)
};

// PWM light characteristic
CONST uint8 PWMCtrlUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(PWM_CTRL_UUID), HI_UINT16(PWM_CTRL_UUID)
};
//PWM NOTIFY
CONST uint8 PWMNotiUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(PWM_NOTI_UUID), HI_UINT16(PWM_NOTI_UUID)
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




static PWMServiceCB_t PWMServiceCB;

/*********************************************************************
 * Profile Attributes - variables
 */

// pwm Service attribute
static CONST gattAttrType_t PWMService = { ATT_BT_UUID_SIZE, PWMServUUID };


static uint8 PWMCtrlProps = GATT_PROP_READ|GATT_PROP_WRITE |GATT_PROP_WRITE_NO_RSP |GATT_PROP_AUTHEN | GATT_PROP_NOTIFY;// |GATT_PROP_WRITE_NO_RSP;
static uint8 PWMCtrlVal[PWM_CTRL_DATA_LEN] = {0};
static gattCharCfg_t PWMCtrl_CharCCCD;




/*********************************************************************
 * Profile Attributes - Table
 */
#define PWM_LIGHT_NOTI_HANDLE 2
static gattAttribute_t PWMAttrTbl[] = 
{
		// PWM light ctrl Service
		{ 
			{ ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
			GATT_PERMIT_READ,                         /* permissions */
			0,                                        /* handle */
			(uint8 *)&PWMService                /* pValue */
		},

    // PWM ctrl Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &PWMCtrlProps 
    },

      // PWM Ctrl Value
    { 
        { ATT_BT_UUID_SIZE, PWMCtrlUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,  
        0, 
        &PWMCtrlVal[0]
    },			
	  // 
	  { 
			{ ATT_BT_UUID_SIZE, clientCharCfgUUID },
			GATT_PERMIT_READ|GATT_PERMIT_WRITE, 
			0, 
			(uint8*)&PWMCtrl_CharCCCD 
	  },		  			
};


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static uint8 PWMS_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                            uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen );
static bStatus_t PWMS_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                 uint8 *pValue, uint8 len, uint16 offset );

/*********************************************************************
 * PROFILE CALLBACKS
 */
// Heart Rate Service Callbacks
CONST gattServiceCBs_t PWMSCBs =
{
  PWMS_ReadAttrCB,  // Read callback function pointer
  PWMS_WriteAttrCB, // Write callback function pointer
  NULL                   // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

bStatus_t PWMS_AddService(PWMServiceCB_t pfnServiceCB )
{
  uint8 status = SUCCESS;

    PWMCtrl_CharCCCD.connHandle = INVALID_CONNHANDLE;
    PWMCtrl_CharCCCD.value = 0;
    
  // Register GATT attribute list and CBs with GATT Server App
  status = GATTServApp_RegisterService( PWMAttrTbl, 
                                          GATT_NUM_ATTRS( PWMAttrTbl ),
                                          &PWMSCBs );
  PWMServiceCB = pfnServiceCB;

  return ( status );
}

static uint8 PWMS_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                            uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen )
{
  bStatus_t status = SUCCESS;

	LOG("PWMS_ReadAttrCB len=%d,maxlen=%d\n",*pLen,maxLen);
  // Make sure it's not a blob operation (no attributes in the profile are long)
  if ( offset > 0 )
  {
    return ( ATT_ERR_ATTR_NOT_LONG );
  }
 
  uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);

	 //uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
	switch(uuid)
	{	
		case GATT_CLIENT_CHAR_CFG_UUID:		
			 *pLen = 2;
			 osal_memcpy(pValue, pAttr->pValue, 2);
		   break;
		case PWM_CTRL_UUID:
//			*pLen = 1;
//			pValue[0] = *pAttr->pValue;
		     if(maxLen>PWM_CTRL_DATA_LEN)  
				 {	 
					 *pLen=PWM_CTRL_DATA_LEN;	        
				 }	 
				 else
				 {	
						*pLen=maxLen;					  
				 } 
				 osal_memcpy( pValue, pAttr->pValue, *pLen );
		    LOG("read PWM_CTRL_UUID\n");
		 break;
		defualt:
			status = ATT_ERR_ATTR_NOT_FOUND;
		}
		

  return ( status );
}

static bStatus_t PWMS_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                 uint8 *pValue, uint8 len, uint16 offset )
{
  bStatus_t status = SUCCESS;
 
  uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
	LOG("write callback=%x,len=%x\n",uuid,len);
  switch ( uuid )
  {
    case PWM_CTRL_UUID:
      if ( offset > 0 )
      {
        status = ATT_ERR_ATTR_NOT_LONG;
      }
      else
			{	
				if((len !=PWM_CTRL_DATA_LEN) ||(pValue[PWM_LIGHT_HEADER_INDEX] !=PWM_LIGHT_TO_BLE_HEADER)||(pValue[PWM_CTRL_DATA_LEN-1] !=PWM_LIGHT_END_DATA))//判断数据包是否是有效的数据包
				{
					LOG("It is not valiable data\n");
				}	
				else 
				{	
					uint8 *pCurValue;   						 
					pCurValue= (uint8 *)pAttr->pValue;        
					VOID osal_memcpy( pCurValue, pValue, PWM_CTRL_DATA_LEN );	
					switch(pValue[PWM_LIGHT_CMD1_INDEX])
					{
						case PWM_LIGHT_CTRL_SET_LIGHT_DATA:
						
              osal_memset(PWM_light_set_data,0,PWM_CTRL_DATA_LEN);		
							osal_memcpy( PWM_light_set_data, pValue, PWM_CTRL_DATA_LEN );
							osal_set_event(PWM_TaskID,PWM_LIGHT_SET_LIGHTDATA_EVT);
						 
							LOG("set light\n");
						
							break;
						case PWM_LIGHT_CTRL_INQUIRE_LIGHT_DATA:
							
						  osal_memset(PWM_light_inquire_data,0,PWM_CTRL_DATA_LEN);		
							osal_memcpy( PWM_light_inquire_data, pValue, PWM_CTRL_DATA_LEN );
							osal_set_event(PWM_TaskID,PWM_LIGHT_INQUIRE_LIGHTDATA_EVT);
							LOG("inquire light data\n");
							break;				

						default:
							LOG("not support function\n");
						break;
						
					}	
				//	PWM_Light_Notify(pValue,len);//notify data back just for test
					LOG("write ctrl datalen=%d\n",len);
				}	
		  }
      break;

		case GATT_CLIENT_CHAR_CFG_UUID: //enable or disable notify
		  status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
												   offset, GATT_CLIENT_CFG_NOTIFY );
		  LOG("NOTIFY handle=%x,value=%x\n",connHandle,pValue[0]);
			break;
		
    default:
      status = ATT_ERR_ATTR_NOT_FOUND;
      break;
  }

  return ( status );
}


bStatus_t PWM_Light_Notify(
													uint8*data,uint8 data_len
													)
{
	attHandleValueNoti_t pNoti={0}; 
	 uint16 conn_handle;
	unsigned short int value = PWMCtrl_CharCCCD.value;
 
	GAPRole_GetParameter( GAPROLE_CONNHANDLE, &conn_handle);
	
	osal_memcpy(pNoti.value,data,data_len);
	pNoti.len=data_len;
	// If notifications enabled
	if ( value & GATT_CLIENT_CFG_NOTIFY )
	{
		// Set the handle
		pNoti.handle = PWMAttrTbl[PWM_LIGHT_NOTI_HANDLE].handle;
    LOG("cnnn hdl=%x,notify handle=%d\n",conn_handle,pNoti.handle);
		// Send the Indication
		return GATT_Notification( conn_handle, &pNoti, FALSE);

	}
	return bleIncorrectMode;

}


/*********************************************************************
*********************************************************************/
