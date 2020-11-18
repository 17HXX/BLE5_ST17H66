/*
 * color_mode.c
 *
 */

#include "color_mode.h"
#include "pwmdemo.h"
#include "OSAL_Timers.h"
#include "pwm.h"
#include "log.h"
uint8 PWM_light_set_data[PWM_CTRL_DATA_LEN]={0}; //set data store buff
uint8 PWM_light_inquire_data[PWM_CTRL_DATA_LEN]={0}; //inquire data buff
static uint8 PWM_LIGHT_work_cnt=0;
static uint8 PWM_LIGHT_strobe_work_cnt=0;
//define ctrl data struct 
typedef struct
{  
	uint8 curr_mode;
	uint8 red_data;
	uint8 green_data;
  uint8 blue_data;
	uint8 white_data;
	uint32 timing_data;
	uint8  speed;
} pwm_light_ctrl_data;




APP_To_Ble_Color_data_t color_data;
pwm_ch_t pwm_light_red_ch;
pwm_ch_t pwm_light_green_ch;
pwm_ch_t pwm_light_blue_ch;
pwm_ch_t pwm_light_white_ch;


unsigned short CRC16_XMODEM(unsigned char *puchMsg, unsigned int usDataLen)  
{  
    unsigned short wCRCin = 0x0000;  
    unsigned short wCPoly = 0x1021;  
    unsigned char wChar = 0;  
    
    while (usDataLen--)     
    {  
        wChar = *(puchMsg++);  
        wCRCin ^= (wChar << 8);
        
        for(int i = 0; i < 8; i++)  
        {  
            if(wCRCin & 0x8000)  
            {
                wCRCin = (wCRCin << 1) ^ wCPoly;  
            }
            else
            {              
                wCRCin = wCRCin << 1;
            }
        }  
    }  
    return (wCRCin) ;  
}  

void pwm_light_init(void)
{
	
		color_data.light_red=0xff;
	  color_data.light_green=0xff;
	  color_data.light_blue=0xff;
	  color_data.light_white=0x00;
	  color_data.work_mode=PWM_LIGHT_WORKMODE_DEFUALT_MODE;
	  color_data.light_speed=0;
	  color_data.color_num=7;// RGBW
	  pwm_light_red_ch.pwmN=PWM_CH0;
	  pwm_light_red_ch.pwmPin=PWM_LIGHT_RED_PIN;
	  pwm_light_red_ch.pwmDiv=PWM_CLK_DIV_4;
	  pwm_light_red_ch.pwmPolarity=PWM_POLARITY_FALLING;
	  pwm_light_red_ch.cntTopVal = PWM_LIGHT_CNTTOPVALUE;	
	  pwm_light_red_ch.cmpVal = color_data.light_red;
	  hal_pwm_ch_start_config(pwm_light_red_ch);
	  
	  pwm_light_green_ch.pwmN=PWM_CH1;
	  pwm_light_green_ch.pwmPin=PWM_LIGHT_GREEN_PIN;
	  pwm_light_green_ch.pwmDiv=PWM_CLK_DIV_4;
	  pwm_light_green_ch.pwmPolarity=PWM_POLARITY_FALLING;
	  pwm_light_green_ch.cntTopVal = PWM_LIGHT_CNTTOPVALUE;	
	  pwm_light_green_ch.cmpVal = color_data.light_green;
	  hal_pwm_ch_start_config(pwm_light_green_ch);
	
	  pwm_light_blue_ch.pwmN=PWM_CH2;
	  pwm_light_blue_ch.pwmPin=PWM_LIGHT_BLUE_PIN;
	  pwm_light_blue_ch.pwmDiv=PWM_CLK_DIV_4;
	  pwm_light_blue_ch.pwmPolarity=PWM_POLARITY_FALLING;
	  pwm_light_blue_ch.cntTopVal = PWM_LIGHT_CNTTOPVALUE;	
	  pwm_light_blue_ch.cmpVal = color_data.light_blue;
	  hal_pwm_ch_start_config(pwm_light_blue_ch);
		
	  pwm_light_white_ch.pwmN=PWM_CH3;
	  pwm_light_white_ch.pwmPin=PWM_LIGHT_WHITE_PIN;
	  pwm_light_white_ch.pwmDiv=PWM_CLK_DIV_4;
	  pwm_light_white_ch.pwmPolarity=PWM_POLARITY_FALLING;
	  pwm_light_white_ch.cntTopVal = PWM_LIGHT_CNTTOPVALUE;	
	  pwm_light_white_ch.cmpVal = color_data.light_white;
	  hal_pwm_ch_start_config(pwm_light_white_ch);		  
		hal_pwm_start();	
		
   PWM_LIGHT_Set_work_mode(color_data.work_mode,color_data.light_speed,color_data.color_num,color_data.light_red,color_data.light_green,color_data.light_blue,color_data.light_white);		
	 LOG("PWM init workmode=%x\n",color_data.work_mode);
	//	osal_start_reload_timer(PWM_TaskID,PWM_LIGTH_PERIOD_TIME_EVT,(PWM_LIGHT_STROBE_MODE_BASE_TIME-color_data.light_speed*PWM_LIGHT_STROBE_MODE_MULTI_BASE));				
}	
//fade mode  variable
static uint8 PWM_light_fademode_cnt=0;
static uint8 PWM_light_fademode_reverse_flag=0;

//more fade mode variable

static uint8 PWM_light_flash_mode_cnt=0;
void PWM_LIGHT_Set_work_mode(uint8 work_mode,uint8 speed,uint8 color_num,uint8 red,uint8 green,uint8 blue,uint8 white)
{
	  uint32 time_cnt=0;
		color_data.light_red=red;
	  color_data.light_green=green;
	  color_data.light_blue=blue;
	  color_data.light_white=white;
	  color_data.work_mode=work_mode;
	  color_data.light_speed=speed;
	   color_data.color_num=color_num;
	  PWM_LIGHT_work_cnt=0;
	
	  switch(color_data.work_mode)
		{
			case PWM_LIGHT_WORKMODE_STROBE_MODE:
			     	color_data.light_red=PWM_LIGHT_DEFUALT_DUTY;
						color_data.light_green=PWM_LIGHT_DEFUALT_DUTY;
						color_data.light_blue=PWM_LIGHT_DEFUALT_DUTY;
						color_data.light_white=0;
						//color_data.work_mode=PWM_LIGHT_DEFUALT_DUTY;
			      color_data.color_num=PWM_LIGHT_COLOR_DEFAULT_NUM;
			      PWM_LIGHT_work_cnt=0;
			      PWM_LIGHT_strobe_work_cnt=0;
			      LOG("strobe startcnt=%d\n",PWM_LIGHT_work_cnt);
			      PWM_light_strobe_mode();			
			      time_cnt=PWM_LIGHT_STROBE_MODE_BASE_TIME-color_data.light_speed*PWM_LIGHT_STROBE_MODE_MULTI_BASE;
			      
  		     break;
			
			case PWM_LIGHT_WORKMODE_SEVEN_FLASH_MODE:
					 time_cnt=color_data.light_speed+PWM_LIGHT_SEVEN_FLASH_MODE_BASE_TIME;	
           PWM_light_flash_mode_cnt=0;			
			     PWM_light_flash_mode();
					 break;
			case PWM_LIGHT_WORKMODE_SMOOTH_MODE:
						PWM_light_smooth_mode();	 
						break;
			
			case PWM_LIGHT_WORKMODE_FADE_MODE:
						PWM_light_fademode_cnt=	0;			
			      time_cnt=PWM_LIGHT_FADE_MODE_BASE_TIME;
			      PWM_light_fademode_reverse_flag=0;
			      PWM_light_fade_mode();
						break;
			
			
			case PWM_LIGHT_WORKMODE_MORE_FADE_MODE:
				    PWM_light_more_fademode();
			      time_cnt=PWM_LIGHT_MORE_FADE_MODE_BASE_TIME;
						break;
						 
						default:
							 LOG("unknow work mode\n");
						break;
						  
		}
		
		
		LOG("period time=%d\n",time_cnt);
		if(color_data.work_mode==PWM_LIGHT_WORKMODE_SMOOTH_MODE)//just use app data and not need timer
		{	
			osal_stop_timerEx(PWM_TaskID,PWM_LIGTH_PERIOD_TIME_EVT);
		}
		else
    {			
			osal_stop_timerEx(PWM_TaskID,PWM_LIGTH_PERIOD_TIME_EVT);
			osal_start_reload_timer(PWM_TaskID,PWM_LIGTH_PERIOD_TIME_EVT,time_cnt);	
		}	
}	



	
uint8 PWMLIGHT_RGBLight_ColorMode[][24] = {
		{255,0,0,																			},//ºìÉ«
		{0,255,0,																			},//ÂÌÉ«
		{0,0,255,																			},//À¶É«
		{255,255,0,																			},//»ÆÉ«
		{0,255,255,																			},//ÇàÉ«
		{255,0,255,																			},//×ÏÉ«
		{153,153,153,																		},//°×É«
		{255,0,0,	0,255,0,																},//ºìÂÌ½¥±ä
		{255,0,0,	0,0,255,																},//ºìÀ¶½¥±ä
		{0,255,0,	0,0,255,																},//ÂÌÀ¶½¥±ä
		{255,0,0,	0,255,0,	0,0,255,													},//ºìÂÌÀ¶
		{255,0,0,	0,255,0,	0,0,255,	255,255,0,	0,255,255,	255,0,255,	153,153,153,},//ºìÂÌÀ¶»ÆÇà×Ï°×
		{0,0,0,		0,0,0,		0,0,0,		0,0,0,		0,0,0,		0,0,0,					},//×Ô¶¨ÒåÌø±ä
		{153,153,153,		0,0,0,		0,0,0,		0,0,0,		0,0,0,		0,0,0,			},//×Ô¶¨ÒåºôÎü
};
//{red,gree,blue,white}
uint8 PWM_Light_RGBLight_Color_value_table[7][4]={
	{255,0,0,0},//ºì
	{0,255,0,0},//ÂÌ
	{0,0,255,0},//À¶
	{255,255,0,0},//»Æ
	{0,255,255,0},//Çà
	{255,0,255,0},//×Ï
	{153,153,153,0}//°×
};

void PWM_light_smooth_mode(void)
{
	hal_pwm_set_count_val(pwm_light_red_ch.pwmN,color_data.light_red,PWM_LIGHT_CNTTOPVALUE);
	hal_pwm_set_count_val(pwm_light_green_ch.pwmN,color_data.light_green,PWM_LIGHT_CNTTOPVALUE);
	hal_pwm_set_count_val(pwm_light_blue_ch.pwmN,color_data.light_blue,PWM_LIGHT_CNTTOPVALUE);
	hal_pwm_set_count_val(pwm_light_white_ch.pwmN,color_data.light_white,PWM_LIGHT_CNTTOPVALUE);
	
}

void PWM_light_all_off(void)
{
	  hal_pwm_set_count_val(pwm_light_red_ch.pwmN,0,PWM_LIGHT_CNTTOPVALUE);
		hal_pwm_set_count_val(pwm_light_green_ch.pwmN,0,PWM_LIGHT_CNTTOPVALUE);
		hal_pwm_set_count_val(pwm_light_blue_ch.pwmN,0,PWM_LIGHT_CNTTOPVALUE);
		hal_pwm_set_count_val(pwm_light_white_ch.pwmN,0,PWM_LIGHT_CNTTOPVALUE);
	  osal_stop_timerEx(PWM_TaskID,PWM_LIGTH_PERIOD_TIME_EVT);
}	

void PWM_light_strobe_mode(void)
{
	if(PWM_LIGHT_strobe_work_cnt>=6)//(color_data.color_num-1))//(PWM_LIGHT_COLOR_DEFAULT_NUM-1))
	{
		PWM_LIGHT_strobe_work_cnt=0;
	}	
	hal_pwm_set_count_val(pwm_light_red_ch.pwmN,PWM_Light_RGBLight_Color_value_table[PWM_LIGHT_strobe_work_cnt][0],PWM_LIGHT_CNTTOPVALUE);
	hal_pwm_set_count_val(pwm_light_green_ch.pwmN,PWM_Light_RGBLight_Color_value_table[PWM_LIGHT_strobe_work_cnt][1],PWM_LIGHT_CNTTOPVALUE);
	hal_pwm_set_count_val(pwm_light_blue_ch.pwmN,PWM_Light_RGBLight_Color_value_table[PWM_LIGHT_strobe_work_cnt][2],PWM_LIGHT_CNTTOPVALUE);
	hal_pwm_set_count_val(pwm_light_white_ch.pwmN,PWM_Light_RGBLight_Color_value_table[PWM_LIGHT_strobe_work_cnt][3],PWM_LIGHT_CNTTOPVALUE);
	PWM_LIGHT_strobe_work_cnt++;
}	



void PWM_light_flash_mode(void)
{
	static uint8 onoff=1;
	
	if(PWM_light_flash_mode_cnt>=6)//(color_data.color_num-1))//(PWM_LIGHT_COLOR_DEFAULT_NUM-1))
	{
		PWM_light_flash_mode_cnt=0;
	}
	if(PWM_light_flash_mode_cnt==0)
	{
		onoff=1;
	}	
	if(onoff)
	{	
		hal_pwm_set_count_val(pwm_light_red_ch.pwmN,PWM_Light_RGBLight_Color_value_table[PWM_light_flash_mode_cnt][0],PWM_LIGHT_CNTTOPVALUE);
		hal_pwm_set_count_val(pwm_light_green_ch.pwmN,PWM_Light_RGBLight_Color_value_table[PWM_light_flash_mode_cnt][1],PWM_LIGHT_CNTTOPVALUE);
		hal_pwm_set_count_val(pwm_light_blue_ch.pwmN,PWM_Light_RGBLight_Color_value_table[PWM_light_flash_mode_cnt][2],PWM_LIGHT_CNTTOPVALUE);
		hal_pwm_set_count_val(pwm_light_white_ch.pwmN,PWM_Light_RGBLight_Color_value_table[PWM_light_flash_mode_cnt][3],PWM_LIGHT_CNTTOPVALUE);
    onoff=0;
	}
  else
	{		
		 onoff=1;
		hal_pwm_set_count_val(pwm_light_red_ch.pwmN,0,PWM_LIGHT_CNTTOPVALUE);
		hal_pwm_set_count_val(pwm_light_green_ch.pwmN,0,PWM_LIGHT_CNTTOPVALUE);
		hal_pwm_set_count_val(pwm_light_blue_ch.pwmN,0,PWM_LIGHT_CNTTOPVALUE);
		hal_pwm_set_count_val(pwm_light_white_ch.pwmN,0,PWM_LIGHT_CNTTOPVALUE);			
	}
	
	PWM_light_flash_mode_cnt++;
}	

void PWM_light_fade_mode(void)
{
	   	switch(PWM_LIGHT_work_cnt)
			{
				case 0://ºì
					   if(PWM_light_fademode_reverse_flag)
						 {
               
							 hal_pwm_set_count_val(pwm_light_red_ch.pwmN,PWM_light_fademode_cnt,PWM_LIGHT_CNTTOPVALUE);
							 hal_pwm_set_count_val(pwm_light_green_ch.pwmN,0,PWM_LIGHT_CNTTOPVALUE);
							 hal_pwm_set_count_val(pwm_light_blue_ch.pwmN,0,PWM_LIGHT_CNTTOPVALUE);
							  PWM_light_fademode_cnt--;		
							 if(PWM_light_fademode_cnt==0)
							 {
								 PWM_light_fademode_reverse_flag=0;
								 PWM_LIGHT_work_cnt++;
							 }	 
						 }
             else
             {
                PWM_light_fademode_cnt++;
							 if(PWM_light_fademode_cnt<PWM_LIGHT_CNTTOPVALUE)
							 {
								   hal_pwm_set_count_val(pwm_light_red_ch.pwmN,PWM_light_fademode_cnt,PWM_LIGHT_CNTTOPVALUE);
									 hal_pwm_set_count_val(pwm_light_green_ch.pwmN,0,PWM_LIGHT_CNTTOPVALUE);
									 hal_pwm_set_count_val(pwm_light_blue_ch.pwmN,0,PWM_LIGHT_CNTTOPVALUE);								   
							 } 	
               if(PWM_light_fademode_cnt==PWM_LIGHT_CNTTOPVALUE)			
               {
								 PWM_light_fademode_reverse_flag=1;
							 }								 
							 
						 }							 
					   
             break;
				case 1: //ÂÌ
					   if(PWM_light_fademode_reverse_flag)
						 {
               
							 hal_pwm_set_count_val(pwm_light_red_ch.pwmN,0,PWM_LIGHT_CNTTOPVALUE);
							 hal_pwm_set_count_val(pwm_light_green_ch.pwmN,PWM_light_fademode_cnt,PWM_LIGHT_CNTTOPVALUE);
							 hal_pwm_set_count_val(pwm_light_blue_ch.pwmN,0,PWM_LIGHT_CNTTOPVALUE);
							  PWM_light_fademode_cnt--;		
							 if(PWM_light_fademode_cnt==0)
							 {
								 PWM_light_fademode_reverse_flag=0;
								 PWM_LIGHT_work_cnt++;
							 }	 
						 }
             else
             {
                PWM_light_fademode_cnt++;
							 if(PWM_light_fademode_cnt<PWM_LIGHT_CNTTOPVALUE)
							 {
								   hal_pwm_set_count_val(pwm_light_red_ch.pwmN,0,PWM_LIGHT_CNTTOPVALUE);
									 hal_pwm_set_count_val(pwm_light_green_ch.pwmN,PWM_light_fademode_cnt,PWM_LIGHT_CNTTOPVALUE);
									 hal_pwm_set_count_val(pwm_light_blue_ch.pwmN,0,PWM_LIGHT_CNTTOPVALUE);								   
							 } 	
               if(PWM_light_fademode_cnt==PWM_LIGHT_CNTTOPVALUE)			
               {
								 PWM_light_fademode_reverse_flag=1;
							 }								 
							 
						 }			
				
				
             break;
				case 2: //À¶
					    if(PWM_light_fademode_reverse_flag)
						 {
               
							 hal_pwm_set_count_val(pwm_light_red_ch.pwmN,0,PWM_LIGHT_CNTTOPVALUE);
							 hal_pwm_set_count_val(pwm_light_green_ch.pwmN,0,PWM_LIGHT_CNTTOPVALUE);
							 hal_pwm_set_count_val(pwm_light_blue_ch.pwmN,PWM_light_fademode_cnt,PWM_LIGHT_CNTTOPVALUE);
							  PWM_light_fademode_cnt--;		
							 if(PWM_light_fademode_cnt==0)
							 {
								 PWM_light_fademode_reverse_flag=0;
								 PWM_LIGHT_work_cnt++;
							 }	 
						 }
             else
             {
                PWM_light_fademode_cnt++;
							 if(PWM_light_fademode_cnt<PWM_LIGHT_CNTTOPVALUE)
							 {
								   hal_pwm_set_count_val(pwm_light_red_ch.pwmN,0,PWM_LIGHT_CNTTOPVALUE);
									 hal_pwm_set_count_val(pwm_light_green_ch.pwmN,0,PWM_LIGHT_CNTTOPVALUE);
									 hal_pwm_set_count_val(pwm_light_blue_ch.pwmN,PWM_light_fademode_cnt,PWM_LIGHT_CNTTOPVALUE);								   
							 } 	
               if(PWM_light_fademode_cnt==PWM_LIGHT_CNTTOPVALUE)			
               {
								 PWM_light_fademode_reverse_flag=1;
							 }								 
							 
						 }							     
             break;
				case 3: //»Æ
					   if(PWM_light_fademode_reverse_flag)
						 {
               
							 hal_pwm_set_count_val(pwm_light_red_ch.pwmN,PWM_light_fademode_cnt,PWM_LIGHT_CNTTOPVALUE);
							 hal_pwm_set_count_val(pwm_light_green_ch.pwmN,PWM_light_fademode_cnt,PWM_LIGHT_CNTTOPVALUE);
							 hal_pwm_set_count_val(pwm_light_blue_ch.pwmN,0,PWM_LIGHT_CNTTOPVALUE);
							  PWM_light_fademode_cnt--;		
							 if(PWM_light_fademode_cnt==0)
							 {
								 PWM_light_fademode_reverse_flag=0;
								 PWM_LIGHT_work_cnt++;
							 }	 
						 }
             else
             {
                PWM_light_fademode_cnt++;
							 if(PWM_light_fademode_cnt<PWM_LIGHT_CNTTOPVALUE)
							 {
								   hal_pwm_set_count_val(pwm_light_red_ch.pwmN,PWM_light_fademode_cnt,PWM_LIGHT_CNTTOPVALUE);
									 hal_pwm_set_count_val(pwm_light_green_ch.pwmN,PWM_light_fademode_cnt,PWM_LIGHT_CNTTOPVALUE);
									 hal_pwm_set_count_val(pwm_light_blue_ch.pwmN,0,PWM_LIGHT_CNTTOPVALUE);								   
							 } 	
               if(PWM_light_fademode_cnt==PWM_LIGHT_CNTTOPVALUE)			
               {
								 PWM_light_fademode_reverse_flag=1;
							 }								 
							 
						 }			
						 
             break;
				case 4: //Çà
					
				    if(PWM_light_fademode_reverse_flag)
						 {
               
							 hal_pwm_set_count_val(pwm_light_red_ch.pwmN,0,PWM_LIGHT_CNTTOPVALUE);
							 hal_pwm_set_count_val(pwm_light_green_ch.pwmN,PWM_light_fademode_cnt,PWM_LIGHT_CNTTOPVALUE);
							 hal_pwm_set_count_val(pwm_light_blue_ch.pwmN,PWM_light_fademode_cnt,PWM_LIGHT_CNTTOPVALUE);
							  PWM_light_fademode_cnt--;		
							 if(PWM_light_fademode_cnt==0)
							 {
								 PWM_light_fademode_reverse_flag=0;
								 PWM_LIGHT_work_cnt++;
							 }	 
						 }
             else
             {
                PWM_light_fademode_cnt++;
							 if(PWM_light_fademode_cnt<PWM_LIGHT_CNTTOPVALUE)
							 {
								   hal_pwm_set_count_val(pwm_light_red_ch.pwmN,0,PWM_LIGHT_CNTTOPVALUE);
									 hal_pwm_set_count_val(pwm_light_green_ch.pwmN,PWM_light_fademode_cnt,PWM_LIGHT_CNTTOPVALUE);
									 hal_pwm_set_count_val(pwm_light_blue_ch.pwmN,PWM_light_fademode_cnt,PWM_LIGHT_CNTTOPVALUE);								   
							 } 	
               if(PWM_light_fademode_cnt==PWM_LIGHT_CNTTOPVALUE)			
               {
								 PWM_light_fademode_reverse_flag=1;
							 }								 
							 
						 }			
						 
             break;
				case 5: //×Ï
					    if(PWM_light_fademode_reverse_flag)
						 {
               
							 hal_pwm_set_count_val(pwm_light_red_ch.pwmN,PWM_light_fademode_cnt,PWM_LIGHT_CNTTOPVALUE);
							 hal_pwm_set_count_val(pwm_light_green_ch.pwmN,0,PWM_LIGHT_CNTTOPVALUE);
							 hal_pwm_set_count_val(pwm_light_blue_ch.pwmN,PWM_light_fademode_cnt,PWM_LIGHT_CNTTOPVALUE);
							  PWM_light_fademode_cnt--;		
							 if(PWM_light_fademode_cnt==0)
							 {
								 PWM_light_fademode_reverse_flag=0;
								 PWM_LIGHT_work_cnt++;
							 }	 
						 }
             else
             {
                PWM_light_fademode_cnt++;
							 if(PWM_light_fademode_cnt<PWM_LIGHT_CNTTOPVALUE)
							 {
								   hal_pwm_set_count_val(pwm_light_red_ch.pwmN,PWM_light_fademode_cnt,PWM_LIGHT_CNTTOPVALUE);
									 hal_pwm_set_count_val(pwm_light_green_ch.pwmN,0,PWM_LIGHT_CNTTOPVALUE);
									 hal_pwm_set_count_val(pwm_light_blue_ch.pwmN,PWM_light_fademode_cnt,PWM_LIGHT_CNTTOPVALUE);								   
							 } 	
               if(PWM_light_fademode_cnt==PWM_LIGHT_CNTTOPVALUE)			
               {
								 PWM_light_fademode_reverse_flag=1;
							 }								 
							 
						 }			
             break;
				case 6: //°×
					   if(PWM_light_fademode_reverse_flag)
						 {
               
							 hal_pwm_set_count_val(pwm_light_red_ch.pwmN,PWM_light_fademode_cnt,PWM_LIGHT_CNTTOPVALUE);
							 hal_pwm_set_count_val(pwm_light_green_ch.pwmN,PWM_light_fademode_cnt,PWM_LIGHT_CNTTOPVALUE);
							 hal_pwm_set_count_val(pwm_light_blue_ch.pwmN,PWM_light_fademode_cnt,PWM_LIGHT_CNTTOPVALUE);
							  PWM_light_fademode_cnt--;		
							 if(PWM_light_fademode_cnt==0)
							 {
								 PWM_light_fademode_reverse_flag=0;
								 PWM_LIGHT_work_cnt=0;
							 }	 
						 }
             else
             {
                PWM_light_fademode_cnt++;
							 if(PWM_light_fademode_cnt<=PWM_LIGHT_RGB_WHITE_VALUE)
							 {
								   hal_pwm_set_count_val(pwm_light_red_ch.pwmN,PWM_light_fademode_cnt,PWM_LIGHT_CNTTOPVALUE);
									 hal_pwm_set_count_val(pwm_light_green_ch.pwmN,PWM_light_fademode_cnt,PWM_LIGHT_CNTTOPVALUE);
									 hal_pwm_set_count_val(pwm_light_blue_ch.pwmN,PWM_light_fademode_cnt,PWM_LIGHT_CNTTOPVALUE);								   
							 } 	
               if(PWM_light_fademode_cnt==PWM_LIGHT_RGB_WHITE_VALUE)			
               {
								 PWM_light_fademode_reverse_flag=1;
							 }								 
							 
						 }			
             break;
				
				default:
					   break;
			}	
			
}


//»ìÉ«Ä£Ê½
static uint8 fade_mode_temp_data=0;
void PWM_light_more_fademode(void)
{
	switch(PWM_LIGHT_work_cnt)
	{
		case 0://red=max green=0,blue=0;
		  fade_mode_temp_data=0;
			hal_pwm_set_count_val(pwm_light_red_ch.pwmN,PWM_LIGHT_MAX_DUTY,PWM_LIGHT_CNTTOPVALUE);
			hal_pwm_set_count_val(pwm_light_green_ch.pwmN,0,PWM_LIGHT_CNTTOPVALUE);
			hal_pwm_set_count_val(pwm_light_blue_ch.pwmN,0,PWM_LIGHT_CNTTOPVALUE);	
		  PWM_LIGHT_work_cnt++;
			break;
		case 1://red max green increase
			if(fade_mode_temp_data<PWM_LIGHT_MAX_DUTY)
			{	
				fade_mode_temp_data++;
				hal_pwm_set_count_val(pwm_light_red_ch.pwmN,PWM_LIGHT_MAX_DUTY,PWM_LIGHT_CNTTOPVALUE);
				hal_pwm_set_count_val(pwm_light_green_ch.pwmN,fade_mode_temp_data,PWM_LIGHT_CNTTOPVALUE);
				hal_pwm_set_count_val(pwm_light_blue_ch.pwmN,0,PWM_LIGHT_CNTTOPVALUE);
			}				
			if(fade_mode_temp_data==PWM_LIGHT_MAX_DUTY)						
			{	
				PWM_LIGHT_work_cnt++;
			}
			
			break;
		case 2://green max ,red decrease
			if(fade_mode_temp_data>0)
			{	
				fade_mode_temp_data--;
				hal_pwm_set_count_val(pwm_light_red_ch.pwmN,fade_mode_temp_data,PWM_LIGHT_CNTTOPVALUE);
				hal_pwm_set_count_val(pwm_light_green_ch.pwmN,PWM_LIGHT_MAX_DUTY,PWM_LIGHT_CNTTOPVALUE);
				hal_pwm_set_count_val(pwm_light_blue_ch.pwmN,0,PWM_LIGHT_CNTTOPVALUE);
			}				
			if(fade_mode_temp_data==0)
      {					
				PWM_LIGHT_work_cnt++;
			}
			break;
		case 3://red:0 green max ,blue increase
			if(fade_mode_temp_data<PWM_LIGHT_MAX_DUTY)
			{	
				fade_mode_temp_data++;
				hal_pwm_set_count_val(pwm_light_red_ch.pwmN,0,PWM_LIGHT_CNTTOPVALUE);
				hal_pwm_set_count_val(pwm_light_green_ch.pwmN,PWM_LIGHT_MAX_DUTY,PWM_LIGHT_CNTTOPVALUE);
				hal_pwm_set_count_val(pwm_light_blue_ch.pwmN,fade_mode_temp_data,PWM_LIGHT_CNTTOPVALUE);
			}				
			if(fade_mode_temp_data==PWM_LIGHT_MAX_DUTY)						
			{	
				PWM_LIGHT_work_cnt++;
			}
			break;
		case 4://red:0,green decrease blue max
			if(fade_mode_temp_data>0)
			{	
				fade_mode_temp_data--;
				hal_pwm_set_count_val(pwm_light_red_ch.pwmN,0,PWM_LIGHT_CNTTOPVALUE);
				hal_pwm_set_count_val(pwm_light_green_ch.pwmN,fade_mode_temp_data,PWM_LIGHT_CNTTOPVALUE);
				hal_pwm_set_count_val(pwm_light_blue_ch.pwmN,PWM_LIGHT_MAX_DUTY,PWM_LIGHT_CNTTOPVALUE);
			}				
			if(fade_mode_temp_data==0)
      {					
				PWM_LIGHT_work_cnt++;
			}
			break;
		case 5://green£º0£¬blue max. red increase
			if(fade_mode_temp_data<PWM_LIGHT_MAX_DUTY)
			{	
				fade_mode_temp_data++;
				hal_pwm_set_count_val(pwm_light_red_ch.pwmN,fade_mode_temp_data,PWM_LIGHT_CNTTOPVALUE);
				hal_pwm_set_count_val(pwm_light_green_ch.pwmN,0,PWM_LIGHT_CNTTOPVALUE);
				hal_pwm_set_count_val(pwm_light_blue_ch.pwmN,PWM_LIGHT_MAX_DUTY,PWM_LIGHT_CNTTOPVALUE);
			}				
			if(fade_mode_temp_data==PWM_LIGHT_MAX_DUTY)						
			{	
				PWM_LIGHT_work_cnt++;
			}
			break;
		case 6://green0,red:max,blue decrease
			if(fade_mode_temp_data>0)
			{	
				fade_mode_temp_data--;
				hal_pwm_set_count_val(pwm_light_red_ch.pwmN,PWM_LIGHT_MAX_DUTY,PWM_LIGHT_CNTTOPVALUE);
				hal_pwm_set_count_val(pwm_light_green_ch.pwmN,0,PWM_LIGHT_CNTTOPVALUE);
				hal_pwm_set_count_val(pwm_light_blue_ch.pwmN,fade_mode_temp_data ,PWM_LIGHT_CNTTOPVALUE);
			}				
			if(fade_mode_temp_data==0)
      {					
				PWM_LIGHT_work_cnt=1;
			}
			break;
		default:
			LOG("more fade error mode\n");
			break;
	}	
}	
