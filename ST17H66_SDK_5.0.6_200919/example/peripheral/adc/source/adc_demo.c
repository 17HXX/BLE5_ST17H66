/**************************************************************************************************
*******
**************************************************************************************************/

/**************************************************************************************************
  Filename:       adc_demo.c
  Revised:        $Date $
  Revision:       $Revision $


**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "OSAL.h"
#include "gpio.h"
#include "clock.h"
#include "adc.h"
#include "adc_demo.h"
#include "log.h"
/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
#define MAX_SAMPLE_POINT    64
uint16_t adc_debug[6][MAX_SAMPLE_POINT];
static uint8_t channel_done_flag = 0;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */
 
/*********************************************************************
 * LOCAL VARIABLES
 */
static uint8 adcDemo_TaskID;   // Task ID for internal task/event processing
/*
channel:
is_differential_mode:
is_high_resolution:
[bit7~bit2]=[p20,p15~p11],ignore[bit1,bit0]
when measure adc(not battery),we'd better use high_resolution.
when measure battery,we'd better use no high_resolution and keep the gpio alone.

differential_mode is rarely used,
if use please config channel as one of [ADC_CH3DIFF,ADC_CH2DIFF,ADC_CH1DIFF],
and is_high_resolution as one of [0x80,0x20,0x08],
then the pair of [P20~P15,P14~P13,P12~P11] will work.
other adc channel cannot work.
*/
adc_Cfg_t adc_cfg = {	
	.channel = ADC_BIT(ADC_CH3P_P20)|ADC_BIT(ADC_CH2P_P14)|ADC_BIT(ADC_CH3N_P15),	
	.is_continue_mode = FALSE,
	.is_differential_mode = 0x00,
	.is_high_resolution = 0x7f,
};




/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void adc_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void adcMeasureTask( void );

/*********************************************************************
 * PROFILE CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

void adc_Init( uint8 task_id )
{
	adcDemo_TaskID = task_id;
	adcMeasureTask();
}

uint16 adc_ProcessEvent( uint8 task_id, uint16 events )
{  
    VOID task_id; // OSAL required parameter that isn't used in this function
    //LOG("adc_ProcessEvent: 0x%x\n",events);
  
    if ( events & SYS_EVENT_MSG )
    {
        uint8 *pMsg;

        if ( (pMsg = osal_msg_receive( adcDemo_TaskID )) != NULL )
        {
            adc_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

            // Release the OSAL message
            VOID osal_msg_deallocate( pMsg );
        }

        // return unprocessed events
        return (events ^ SYS_EVENT_MSG);
    }
 
    if ( events & 0x20 )
    {
        // Perform periodic heart rate task
        //LOG("20\n");
        //osal_start_timerEx( adcDemo_TaskID, 0x20, 2000);    
        return (events ^ 0x20);
    }  

    if ( events & adcMeasureTask_EVT )
    {
        // Perform periodic heart rate task
        //LOG("adcMeasureTask_EVT\n");
        adcMeasureTask();    
        return (events ^ adcMeasureTask_EVT);
    }  
  
    // Discard unknown events
    return 0;
}

static void adc_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
	
}

static void adc_evt(adc_Evt_t* pev)
{
	float value = 0;
	int i = 0;
	bool is_high_resolution = FALSE;
	bool is_differential_mode = FALSE;
	uint8_t ch = 0;	
	
	if((pev->type != HAL_ADC_EVT_DATA) || (pev->ch < 2))
		return;
		
	osal_memcpy(adc_debug[pev->ch-2],pev->data,2*(pev->size));
	channel_done_flag |= BIT(pev->ch);		
		
	if(channel_done_flag == adc_cfg.channel)
	{        		
		for(i=2;i<8;i++)
		{
			if(channel_done_flag & BIT(i))
			{
				is_high_resolution = (adc_cfg.is_high_resolution & BIT(i))?TRUE:FALSE;
				is_differential_mode = (adc_cfg.is_differential_mode & BIT(i))?TRUE:FALSE;
				value = hal_adc_value_cal((adc_CH_t)i,adc_debug[i-2], pev->size, is_high_resolution,is_differential_mode);
				
				switch(i)
				{
					case ADC_CH1N_P11:
						ch=11;
						break;
					case ADC_CH1P_P23:
						ch=23;
						break;
					case ADC_CH2N_P24:
						ch=24;
						break;
					case ADC_CH2P_P14:
						ch=14;
						break;
					case ADC_CH3N_P15:
						ch=15;
						break;
					case ADC_CH3P_P20:
						ch=20;
						break;
					default:
						break;
				}
				
				if(ch!=0)
				{
					LOG("P%d %d mv ",ch,(int)(value*1000));
				}
				else
				{
					LOG("invalid channel\n");
				}					
			}
		}			
		LOG(" mode:%d \n",adc_cfg.is_continue_mode);		

		channel_done_flag = 0;
		
		if(adc_cfg.is_continue_mode == FALSE)
		{
			osal_start_timerEx(adcDemo_TaskID, adcMeasureTask_EVT,1000);
		}
	}    
}

static void adcMeasureTask( void )
{
	int ret;
	bool batt_mode = TRUE;
	uint8_t batt_ch = ADC_CH3P_P20;
	GPIO_Pin_e pin;
	
	//LOG("adcMeasureTask\n");
	if(FALSE == batt_mode)
	{
		ret = hal_adc_config_channel(adc_cfg, adc_evt);
	}
	else
	{
		if((((1 << batt_ch) & adc_cfg.channel) == 0) || (adc_cfg.is_differential_mode != 0x00))			
			return;

		pin = s_pinmap[batt_ch];		
		hal_gpio_cfg_analog_io(pin,Bit_DISABLE);
		hal_gpio_write(pin, 1);		

		ret = hal_adc_config_channel(adc_cfg, adc_evt);
		hal_gpio_cfg_analog_io(pin,Bit_DISABLE);	
	}	
	
	if(ret)
	{
		LOG("ret = %d\n",ret);
		return;
	}
	hal_adc_start();
}

