/**************************************************************************************************
THIS IS EMPTY HEADER
**************************************************************************************************/

/*******************************************************************************
* @file		adc.h
* @brief	Contains all functions support for adc driver
* @version	0.0
* @date		18. Oct. 2017
* @author	qing.han
* 
*
*******************************************************************************/
#ifndef __ADC__H__
#define __ADC__H__

#ifdef __cplusplus
extern "C" {
#endif


#include "types.h"
#include "bus_dev.h"
#include "gpio.h"

#define    MAX_ADC_SAMPLE_SIZE     32
#define    ADC_CH_BASE             (0x40050400UL)

#define    ENABLE_ADC_INT       AP_ADCC->intr_mask |= 0x000001ff
#define    MASK_ADC_INT         AP_ADCC->intr_mask &= 0xfffffe00

#define    CLEAR_ADC_INT(n)     AP_ADC->intr_clear |= BIT(n)


#define    IS_CLAER_ADC_INT_VOICE (AP_ADC->intr_clear & BIT(8))
#define    IS_CLAER_ADC_INT(n)    (AP_ADC->intr_clear & BIT(n))
    
#define    GET_IRQ_STATUS         (AP_ADCC->intr_status & 0x3ff)

#define    ENABLE_ADC             (AP_PCRM->ANA_CTL |= BIT(3))
#define    DISABLE_ADC            (AP_PCRM->ANA_CTL &= ~BIT(3))
    
#define    ADC_CLOCK_ENABLE       (AP_PCRM->CLKHF_CTL1 |= BIT(13))
#define    ADC_CLOCK_DISABLE       (AP_PCRM->CLKHF_CTL1 &= ~BIT(13))

#define ADC_USE_TIMEOUT 0
#define ADC_OP_TIMEOUT  100
#if(ADC_USE_TIMEOUT == 1)
  #define ADC_INIT_TOUT(to) int to = hal_systick()
  #define ADC_CHECK_TOUT(to, timeout, loginfo) {if(hal_ms_intv(to) > timeout){LOG(loginfo);return PPlus_ERR_TIMEOUT;}}
#else
  #define ADC_INIT_TOUT(to)
  #define ADC_CHECK_TOUT(to, timeout, loginfo)
#endif
	
/**************************************************************************************
 * @fn          hal_get_adc_int_source
 *
 * @brief       This function process for get adc interrupt source,such as adc channel NO 
 *
 * input parameters
 *
 * @param       None.      
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      adc interrupt source bit loaction(uint8_t) 
 **************************************************************************************/
typedef enum {
	ADC_CH0 = 0,
	ADC_CH1 = 1,                 ADC_CH0DIFF = 1,/*p18(positive),p25(gegative),only works in diff*/
	ADC_CH2 = 2,ADC_CH1N_P11 = 2,
	ADC_CH3 = 3,ADC_CH1P_P23 = 3,ADC_CH1DIFF = 3, 
	ADC_CH4 = 4,ADC_CH2N_P24 = 4,
	ADC_CH5 = 5,ADC_CH2P_P14 = 5,ADC_CH2DIFF = 5, 
	ADC_CH6 = 6,ADC_CH3N_P15 = 6,
	ADC_CH7 = 7,ADC_CH3P_P20 = 7,ADC_CH3DIFF = 7,
	ADC_CH_VOICE = 8,
	ADC_CH_NUM =9,
}adc_CH_t;

#define ADC_BIT(ch) (1<<ch)

enum{
  HAL_ADC_EVT_DATA = 1,
  HAL_ADC_EVT_FAIL = 0xff
};

typedef enum {
	HAL_ADC_CLOCK_80K = 0,
	HAL_ADC_CLOCK_160K = 1,
	HAL_ADC_CLOCK_320K = 2,
}adc_CLOCK_SEL_t;

typedef struct _adc_Cfg_t{
  uint8_t channel;
  bool  is_continue_mode; 
  uint8_t  is_differential_mode;
  uint8_t  is_high_resolution; 
}adc_Cfg_t;


typedef struct _adc_Evt_t{
  int       type;
  adc_CH_t  ch;
  uint16_t* data;
  uint8_t   size; //word size
}adc_Evt_t;

typedef void (*adc_Hdl_t)(adc_Evt_t* pev);

typedef struct _adc_Contex_t{
    bool        enable;
    uint8_t     all_channel;
		bool        continue_mode;
    adc_Hdl_t   evt_handler[ADC_CH_NUM];
}adc_Ctx_t;

extern gpio_pin_e s_pinmap[ADC_CH_NUM];
/**************************************************************************************
 * @fn          hal_adc_init
 *
 * @brief       This function process for adc initial 
 *
 * input parameters
 *
 * @param       ADC_CH_e adc_pin: adc pin select;ADC_CH0~ADC_CH7 and ADC_CH_VOICE
 *              ADC_SEMODE_e semode: single-end mode and diff mode select; 1:SINGLE_END(single-end mode) 0:DIFF(Diff mode)
 *              IO_CONTROL_e amplitude: input signal amplitude, 0:BELOW_1V,1:UP_1V
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 **************************************************************************************/
void hal_adc_init(void);

int hal_adc_config_channel(adc_Cfg_t cfg, adc_Hdl_t evt_handler);

int hal_adc_clock_config(adc_CLOCK_SEL_t clk);

int hal_adc_start(void);

int hal_adc_stop(void);

void __attribute__((weak)) hal_ADC_IRQHandler(void);

float hal_adc_value_cal(adc_CH_t ch,uint16_t* buf, uint32_t size, uint8_t high_resol, uint8_t diff_mode);

#ifdef __cplusplus
}
#endif

#endif
