/**************************************************************************************************
*******
**************************************************************************************************/


#ifndef _OTA_APP_SERVICE_H
#define _OTA_APP_SERVICE_H
#include "bcomdef.h"
#include "version.h"

enum{
  OTA_MODE_OTA_APPLICATION= 0,
  OTA_MODE_OTA_FCT,//          1
  OTA_MODE_OTA,//              2
  OTA_MODE_RESOURCE,//         3
  OTA_MODE_OTA_NADDR = 6//    6 ota no address plus
};

#define OTA_MODE_SELECT_REG 0x4000f034


#define OTA_APP_SERVICE_VERSION "V2.0.1"

enum{
  OTAAPP_CMD_START_OTA = 1,
  OTAAPP_CMD_INFO,
  OTAAPP_CMD_FORMAT,
  OTAAPP_CMD_VER,
};

typedef struct{
  uint8_t cmd;
  union{
    struct{
      uint8_t reserv[20-1];
    } dummy;
    struct{
      uint8_t mode;
    } start;
  } p;  //parameter
}ota_app_cmd_t;


bStatus_t ota_app_AddService(void);

int ota_vendor_module_StartOTA(uint8_t mode);
int ota_vendor_module_Version(  uint8_t* major, uint8_t* minor, uint8_t* revision, uint8_t *test_build);


#endif

