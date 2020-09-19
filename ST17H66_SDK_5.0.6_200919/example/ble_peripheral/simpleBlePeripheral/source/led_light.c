/**************************************************************************************************
*******
**************************************************************************************************/

#include "led_light.h"
#include "pwm.h"
#include "OSAL.h"
#include "gpio.h"
#include "error.h"


#define GPIO_GREEN    P15
#define GPIO_BLUE     P18
#define GPIO_RED      P20




static uint16_t s_light[3];
static light_blink_cfg_t s_lightBlink;

static void light_start_timer(void)
{
  //osal_start_timerEx(AppWrist_TaskID, TIMER_LIGHT_EVT, 30*1000);
}
static void light_stop_timer(void)
{
  //osal_stop_timerEx(AppWrist_TaskID, TIMER_LIGHT_EVT);
}


void light_reflash(void)
{
  if(s_light[LIGHT_RED] + s_light[LIGHT_GREEN] + s_light[LIGHT_BLUE]){
    
    hal_pwm_close_channel(PWM_CH0);
    hal_pwm_destroy(PWM_CH0);
    hal_pwm_close_channel(PWM_CH1);
    hal_pwm_destroy(PWM_CH1);
    hal_pwm_close_channel(PWM_CH2);
    hal_pwm_destroy(PWM_CH2);
    hal_pwm_stop();
    hal_gpio_pin_init(GPIO_GREEN, IE);
    hal_gpio_pin_init(GPIO_RED, IE);
    hal_gpio_pin_init(GPIO_BLUE, IE);
    hal_gpio_pull_set(GPIO_GREEN, WEAK_PULL_UP);
    hal_gpio_pull_set(GPIO_RED, WEAK_PULL_UP);
    hal_gpio_pull_set(GPIO_BLUE, WEAK_PULL_UP);
    
    if(s_light[LIGHT_GREEN]){
      hal_pwm_init(PWM_CH0, PWM_CLK_DIV_64, PWM_CNT_UP, PWM_POLARITY_RISING);
      hal_pwm_set_count_val(PWM_CH0, s_light[LIGHT_GREEN], LIGHT_TOP_VALUE);
      hal_pwm_open_channel(PWM_CH0, GPIO_GREEN);
    }
    
    if(s_light[LIGHT_BLUE]){
      hal_pwm_init(PWM_CH1, PWM_CLK_DIV_64, PWM_CNT_UP, PWM_POLARITY_RISING);
      hal_pwm_set_count_val(PWM_CH1, s_light[LIGHT_BLUE], LIGHT_TOP_VALUE);
      hal_pwm_open_channel(PWM_CH1, GPIO_BLUE);
    }
    
    if(s_light[LIGHT_RED]){
      hal_pwm_init(PWM_CH2, PWM_CLK_DIV_64, PWM_CNT_UP, PWM_POLARITY_RISING);
      hal_pwm_set_count_val(PWM_CH2, s_light[LIGHT_RED], LIGHT_TOP_VALUE);
      hal_pwm_open_channel(PWM_CH2, GPIO_RED);
    }
    
    hal_pwm_start();
    light_stop_timer();
    light_start_timer();
  }
  else
  {
    hal_pwm_close_channel(PWM_CH0);
    hal_pwm_destroy(PWM_CH0);
    hal_pwm_close_channel(PWM_CH1);
    hal_pwm_destroy(PWM_CH1);
    hal_pwm_close_channel(PWM_CH2);
    hal_pwm_destroy(PWM_CH2);
    hal_pwm_stop();
    hal_gpio_pin_init(GPIO_GREEN, IE);
    hal_gpio_pin_init(GPIO_RED, IE);
    hal_gpio_pin_init(GPIO_BLUE, IE);
    hal_gpio_pull_set(GPIO_GREEN, WEAK_PULL_UP);
    hal_gpio_pull_set(GPIO_RED, WEAK_PULL_UP);
    hal_gpio_pull_set(GPIO_BLUE, WEAK_PULL_UP);
    light_stop_timer();
  }
}

void light_timeout_handle(void)
{
  s_light[0] = 0;
  s_light[1] = 0;
  s_light[2] = 0;
  hal_pwm_close_channel(PWM_CH0);
  hal_pwm_destroy(PWM_CH0);
  hal_pwm_close_channel(PWM_CH1);
  hal_pwm_destroy(PWM_CH1);
  hal_pwm_close_channel(PWM_CH2);
  hal_pwm_destroy(PWM_CH2);
  hal_pwm_stop();
  hal_gpio_pin_init(GPIO_GREEN, IE);
  hal_gpio_pin_init(GPIO_RED, IE);
  hal_gpio_pin_init(GPIO_BLUE, IE);
  hal_gpio_pull_set(GPIO_GREEN, WEAK_PULL_UP);
  hal_gpio_pull_set(GPIO_RED, WEAK_PULL_UP);
  hal_gpio_pull_set(GPIO_BLUE, WEAK_PULL_UP);
  
}

int light_config(uint8_t ch, uint16_t value)
{
  if(ch >2 || (value > LIGHT_TOP_VALUE))
    return PPlus_ERR_INVALID_PARAM;

  s_light[ch] = (uint16_t)value;
	return PPlus_SUCCESS;
}
int light_ctrl(uint8_t ch, uint16_t value)
{
  if(ch >2 || (value > LIGHT_TOP_VALUE))
    return PPlus_ERR_INVALID_PARAM;

  s_light[ch] = (uint16_t)value;

  light_reflash();
	return PPlus_SUCCESS;
}


int light_init(void)
{
  s_light[LIGHT_GREEN] = 0;
  s_light[LIGHT_BLUE] = 0;
  s_light[LIGHT_RED] = 0;
  hal_gpio_pin_init(GPIO_GREEN, IE);
  hal_gpio_pin_init(GPIO_RED, IE);
  hal_gpio_pin_init(GPIO_BLUE, IE);
  hal_gpio_pull_set(GPIO_GREEN, WEAK_PULL_UP);
  hal_gpio_pull_set(GPIO_RED, WEAK_PULL_UP);
  hal_gpio_pull_set(GPIO_BLUE, WEAK_PULL_UP);
  //hal_gpio_pull_set(GPIO_GREEN, STRONG_PULL_UP);
  //hal_gpio_pull_set(GPIO_RED, STRONG_PULL_UP);
  //hal_gpio_pull_set(GPIO_YELLOW, STRONG_PULL_UP);
  osal_memset(&s_lightBlink, 0, sizeof(s_lightBlink));
  s_lightBlink.val0=LIGHT_TURN_OFF;
  s_lightBlink.val1=LIGHT_TURN_ON;
  
  
  return PPlus_SUCCESS;
}

int light_blink_evt_cfg(uint8_t task_id,uint16_t event_id)
{
    if(s_lightBlink.status==0)
    {
        s_lightBlink.task_id=task_id;
        s_lightBlink.event_id=event_id;

        return PPlus_SUCCESS;
    }
    else
    {
        return PPlus_ERR_BUSY;
    }

}
int light_blink_set(uint8_t light,uint8 blinkIntv,uint8 blinkCnt)
{
    if(s_lightBlink.status==0)
    {
        
        s_lightBlink.light=light;
        s_lightBlink.tagCnt=blinkCnt;
        s_lightBlink.intv = blinkIntv;

        s_lightBlink.status=1;
        if(s_lightBlink.task_id>0 && s_lightBlink.event_id>0)
        {
            light_ctrl(LIGHT_RED,0);
            light_ctrl(LIGHT_GREEN,0);
            light_ctrl(LIGHT_BLUE,0);
            s_lightBlink.curCnt=0;
            osal_set_event( s_lightBlink.task_id, s_lightBlink.event_id);
        }
        else
        {
            return PPlus_ERR_NOT_FOUND;
        }
        return PPlus_SUCCESS;
    }
    else
    {
        return PPlus_ERR_BUSY;
    }
}
void light_blink_porcess_evt(void)
{
    if(s_lightBlink.curCnt==(s_lightBlink.tagCnt*2) )
    {
        light_ctrl(LIGHT_RED,0);
        light_ctrl(LIGHT_GREEN,0);
        light_ctrl(LIGHT_BLUE,0);
        osal_stop_timerEx( s_lightBlink.task_id, s_lightBlink.event_id);
        s_lightBlink.status=0;
    }
    else
    {

        if(s_lightBlink.curCnt&0x01)
        {
            light_ctrl(s_lightBlink.light,s_lightBlink.val1);
        }
        else
        {
            light_ctrl(s_lightBlink.light,s_lightBlink.val0);
        }
        s_lightBlink.curCnt++;
        osal_start_timerEx(s_lightBlink.task_id, s_lightBlink.event_id,s_lightBlink.intv*100);
    }
    

}

void light_color_quickSet(light_color_t color)
{
    switch ( color )
    {
        case LIGHT_COLOR_OFF:
            LIGHT_ON_OFF(0,0,0);
        break;
        case LIGHT_COLOR_RED:
            LIGHT_ONLY_RED_ON;
        break;

        case LIGHT_COLOR_GREEN:
            LIGHT_ONLY_GREEN_ON;
        break;

        case LIGHT_COLOR_BLUE:
            LIGHT_ONLY_BLUE_ON;
        break;

        case LIGHT_COLOR_CYAN:
            LIGHT_ON_CYAN;
        break;

        case LIGHT_COLOR_YELLOW:
            LIGHT_ON_YELLOW;
        break;

        case LIGHT_COLOR_MEGENTA:
            LIGHT_ON_MEGENTA;
        break;

        case LIGHT_COLOR_WHITE:
            LIGHT_ON_WHITE;
        break;

        default:
        break;
    }

}
