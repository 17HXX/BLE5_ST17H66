
/**************************************************************************************************
  Filename:       RF433_analysis.c
  Revised:        
  Revision:       
* @date		05. Sep. 2020
* @author	Duanyang.chen
  Description:    This file contains the RF433 application                 

**************************************************************************************************/
#include "types.h"
#include "bcomdef.h"
#include "OSAL.h"
#include "mcu.h"
//#include "ap_timer.h"
#include "pwrmgr.h"
#include "gpio.h"
#include "log.h"
//#include "common.h"
#define RF433_SHORT_TIME_LOW  480 
#define RF433_SHORT_TIME_UPPER  650  //us
#define RF433_LONG_TIME_LOW  1450
#define RF433_LONG_TIME_UP   1550  //us
#define RF433_DATA_PIN    P0
#define RF433_MAX_PLUS_NUM    24

extern uint32_t hal_read_current_time(void);
static uint32 key_num=0x00FFFFFF;
#define BIT_SET(data,bit)                ( data |= BIT(bit) )     //bit set
#define BIT_CLR(data,bit)                ( data &= ~BIT(bit) )    //bit clear

static uint8 receive_task_id;
static uint16 send_event;
static uint32 analysis_data=0;
uint32 RF433_Get_data(void)
{
	 return analysis_data;
}	

void RF433_NegInt_handle(GPIO_Pin_e pin, IO_Wakeup_Pol_e type)
{
	 static uint32 pre_time=0;
	 static  uint32 cur_time=0;
	 uint32 delta_time=0;
	 static uint8 plus_cnt=0;
	 static uint8 neg_first_flag=0;
//	 static uint8 set_event_flag=0;
	 if(pin != RF433_DATA_PIN )
		 return;
	if(type==NEGEDGE)
	 {
		  pre_time=hal_read_current_time();
		  neg_first_flag=1;
	 }	
   else if(type==POSEDGE)
   {
		 if(neg_first_flag==0)
		 {
				key_num=0x00FFFFFF;
		    plus_cnt=0; 
        return;
     }			 
		 cur_time=hal_read_current_time();
		 delta_time=TIME_DELTA(cur_time,pre_time);	 
		
	   if((delta_time>RF433_SHORT_TIME_LOW) && (delta_time<RF433_LONG_TIME_UP))
	   {
	
		   if(delta_time>RF433_LONG_TIME_LOW)
		  {
			     BIT_CLR(key_num,(23-plus_cnt));
		  } 
		 	 plus_cnt++;
		  if(plus_cnt==RF433_MAX_PLUS_NUM)
		  {
					 analysis_data=key_num;			 
					 osal_set_event(receive_task_id,send_event);
			     key_num=0x00FFFFFF;	
  			   plus_cnt=0;
		  }
	  }
   else
   {
		 key_num=0x00FFFFFF;
		 plus_cnt=0; 		  
	 }	
  neg_first_flag=0;	 
 }
}	
 
void RF433_analysis_init(uint8 taskid,uint16 event)
{
	 receive_task_id=taskid;
	 send_event=event;
	 hal_gpio_fmux(RF433_DATA_PIN,Bit_DISABLE);
	 hal_gpio_pin_init(RF433_DATA_PIN,IE);
	 hal_gpio_pull_set(RF433_DATA_PIN,STRONG_PULL_UP);
	// hal_gpioin_register(RF433_DATA_PIN,RF433_PosInt_handle,RF433_PosInt_handle);	
	hal_gpioin_register(RF433_DATA_PIN,RF433_NegInt_handle,RF433_NegInt_handle);	
	
	 LOG("RF433 analysis init RC taskid=%d,event=%x\n",receive_task_id,event);
}


