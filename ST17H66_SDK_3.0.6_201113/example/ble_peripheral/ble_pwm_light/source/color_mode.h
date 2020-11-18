
#ifndef COLOR_MODE_H_
#define COLOR_MODE_H_
#include"types.h"
//**************************************************************************************************
//                                       数据定义
//**************************************************************************************************

#define PWM_LIGHT_RED_PIN P34
#define PWM_LIGHT_BLUE_PIN P2
#define PWM_LIGHT_GREEN_PIN P3
#define PWM_LIGHT_WHITE_PIN P7


#define		EEPROM_FIRSTUSE_ADD				0x00
#define		EEPROM_TEST_MODE_ADD			0x01
#define		EEPROM_LIGHTMODE_ADD			0x08
#define		EEPROM_STATICCOLOR				0x0e

#define		EEPROM_MYSELF_JUMP_COLOR		0x0f
#define		EEPROM_JUMP_NUM_COLOR			0x21

#define		EEPROM_JUMP_SPEED_COLOR			0x22
#define		EEPROM_MYSELF_BREATH_COLOR		0x35
#define		EEPROM_BREATH_SPEED_COLOR		0x48
#define  	EEPROM_Fade_Color_ADD 			0x4A
#define 	EEPROM_BTEATH_NUM				0x4C

//define base time for different mode 
#define PWM_LIGHT_STROBE_MODE_BASE_TIME  2000// base time ms
#define PWM_LIGHT_STROBE_MODE_MULTI_BASE 4 
#define PWM_LIGHT_SEVEN_FLASH_MODE_BASE_TIME 100// ms
#define PWM_LIGHT_S_MODE_BASE_TIME  20 //ms
#define PWM_LIGHT_FADE_MODE_BASE_TIME    5 //ms
#define PWM_LIGHT_MORE_FADE_MODE_BASE_TIME 10 //ms

#define PWM_LIGHT_CNTTOPVALUE     255 // PWM default duty
#define PWM_LIGHT_RGB_WHITE_VALUE 153 //RGB×éºÏÉú³ÉµÄ°×
#define PWM_LIGHT_DEFUALT_DUTY   PWM_LIGHT_CNTTOPVALUE //
#define PWM_LIGHT_MAX_DUTY  255
#define PWM_LIGHT_COLOR_DEFAULT_NUM 7

#define PWM_CTRL_DATA_LEN 20
#define PWM_NOTI_DATA_LEN  20
#define PWM_LIGHT_TO_BLE_HEADER 0xcd  //ÏÂÐÐÊý¾ÝÍ·Îª0xcd
#define PWM_LIGHT_FORM_BLE_HEADER PWM_LIGHT_TO_BLE_HEADER //ÉÏÐÐÊý¾ÝÍ·Îª0xcd
#define PWM_LIGHT_END_DATA 0x12        //Êý¾Ý½áÎ²Îª0x12
//APP TO BLE CMD
#define PWM_LIGHT_CTRL_SET_LIGHT_DATA 0x50 //ÉèÖÃµÆµÄÊý¾Ý
#define PWM_LIGHT_CTRL_INQUIRE_LIGHT_DATA 0x51 //²éÑ¯µÆµÄÊý¾Ý

#define PWM_LIGHT_HEADER_INDEX  0  //package header offset
#define PWM_LIGHT_CMD1_INDEX 1     //cmd1 offset
#define PWM_LIGHT_CMD2_INDEX 2     //cmd2  offset
#define PWM_LIGHT_CMD3_INDEX 3     //cmd3 offset 
#define PWM_LIGHT_DATA_LENGTH_INDEX 4
#define PWM_LIGHT_RGBW_GREEN1_INDEX 5
#define PWM_LIGHT_RGBW_BLUE1_INDEX 6
#define PWM_LIGHT_RGBW_RED1_INDEX 7
#define PWM_LIGHT_RGBW_WHITE1_INDEX 8
#define PWM_LIGHT_SPEED_INDEX  9

#define PWM_LIGHT_RGBW_GREEN2_INDEX 10
#define PWM_LIGHT_RGBW_BLUE2_INDEX 11
#define PWM_LIGHT_RGBW_RED2_INDEX 12
#define PWM_LIGHT_RGBW_WHITE2_INDEX 13
#define PWM_LIGHT_CHECK_INDEX     18
#define PWM_LIGHT_DATAPAC_END_INDEX 19

#define PWM_LIGHT_TIMING_DATA_INDEX 8 //ÉèÖÃ¶¨Ê±µÄÊý¾ÝµÄÆðÊ¼Î»
#define PWM_LIGTH_TIMING_DATA_LEN   4 //¶¨Ê±Êý¾Ý³¤¶È

//CMD2
#define PWM_LIGHT_MODE_SETTING 0x80   //ÉèÖÃÄ£Ê½
#define PWM_LIGHT_TIMING_OFF_ENABLE 0x81 //ÉèÖÃ¶¨Ê±¹ØµÆ
#define PWM_LIGHT_TIMING_OFF_DISABLE 0x82 //È¡Ïû¶¨Ê±¹ØµÆ

//cmd3 when cmd=0x80 will effective
#define PWM_LIGHT_WORKMODE_SMOOTH_MODE 0x60 // APPÏÂ·¢RGBÖµ
#define PWM_LIGHT_WORKMODE_SEVEN_FLASH_MODE 0x61 // ÉÁÒ»´Î on/off 1times
#define PWM_LIGHT_WORKMODE_STROBE_MODE  0x62 //Á÷Ë®µÆ
#define PWM_LIGHT_WORKMODE_FADE_MODE    0x63  //ºôÎüµÆ
#define PWM_LIGHT_WORKMODE_MORE_FADE_MODE 0x64 //»ìÉ«
#define PWM_LIGHT_WORKMODE_DEFUALT_MODE  PWM_LIGHT_WORKMODE_STROBE_MODE



typedef struct
{
 uint8   light_red;//0--255
 uint8   light_green;
 uint8   light_blue;
 uint8   light_white;
 uint8   light_speed;	
 uint8   work_mode;
 uint8   color_num;
}APP_To_Ble_Color_data_t;
extern APP_To_Ble_Color_data_t color_data;
extern uint8 PWM_light_set_data[PWM_CTRL_DATA_LEN]; //set data store buff
extern uint8 PWM_light_inquire_data[PWM_CTRL_DATA_LEN]; //inquire data buff

extern void PWM_LIGHT_Set_work_mode(uint8 work_mode,uint8 speed,uint8 color_num,uint8 red,uint8 green,uint8 blue,uint8 white);
// APPÏÂ·¢RGBÖµ
extern void PWM_light_smooth_mode(void);
// ÉÁÒ»´Î on/off 1times
extern void PWM_light_flash_mode(void);
//7 colorsÂÖÑ¯
extern void PWM_light_strobe_mode(void);
//ºôÎüµÆ
extern void PWM_light_fade_mode(void);
//»ìÉ«
extern void PWM_light_more_fademode(void);

extern void pwm_light_init(void);
extern void PWM_light_all_off(void);
extern unsigned short CRC16_XMODEM(unsigned char *puchMsg, unsigned int usDataLen); 

#endif /* COLOR_MODE_H_ */
