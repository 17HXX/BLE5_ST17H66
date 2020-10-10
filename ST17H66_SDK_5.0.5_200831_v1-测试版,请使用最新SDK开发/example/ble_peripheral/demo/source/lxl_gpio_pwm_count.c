#include <lxl_gpio_all_init.h>
static u16 lxl_pwm_value_light[3] ;   //pwm点亮的占空比




void lxl_pwm_timeout_handle(void)
{
	lxl_pwm_value_light[0] = 0;
	lxl_pwm_value_light[1] = 0;
	lxl_pwm_value_light[2] = 0;
	hal_pwm_close_channel(PWM_CH0);
	hal_pwm_destroy(PWM_CH0);
	hal_pwm_close_channel(PWM_CH1);
	hal_pwm_destroy(PWM_CH1);
	//hal_pwm_close_channel(PWM_CH2);
	//hal_pwm_destroy(PWM_CH2);
	hal_pwm_stop();
}

int lxl_pwm_ctrl(uint8_t ch, uint16_t value)
{
	if(ch >2 || (value > MY_PWM_CNTTOP_VALUE))
		return PPlus_ERR_INVALID_PARAM;
	switch (ch)
	{
		case 0:
			if(value==0){
				hal_gpio_write(MY_GPIO_PWM_M0_1, 0);
			}else{
				lxl_pwm_on(ch) ;
				hal_pwm_open_channel(PWM_CH0, MY_GPIO_PWM_M0_1);
				lxl_pwm_value_light[0] = value;
			}  
			hal_pwm_set_count_val(PWM_CH0, lxl_pwm_value_light[0], MY_PWM_CNTTOP_VALUE);
		break;
		case 1:
			if(value==0){
				hal_gpio_write(MY_GPIO_BACK_LED_1, 0);
			}else{
				lxl_pwm_on(ch) ;
				hal_pwm_open_channel(PWM_CH1, MY_GPIO_BACK_LED_1);
				lxl_pwm_value_light[0] = value;
			}  
			hal_pwm_set_count_val(PWM_CH1, lxl_pwm_value_light[1], 255);
		break;
		#if 0
		case 2:
			if(value==0){
				hal_gpio_write(GPIO_RED, 0);
			}else{
				hal_pwm_open_channel(PWM_CH2, GPIO_RED);
				s_light[2] = value;
			}
		break;
		#endif
		default:
		break;
	}
	return PPlus_SUCCESS;
}

int lxl_pwm_on(uint8_t ch)
{
	if(ch >3)
		return PPlus_ERR_INVALID_PARAM;
	lxl_pwm_value_light[ch] = (uint16_t)(MY_PWM_CNTTOP_VALUE>>1);
	switch(ch){
		case 0:
			hal_pwm_init(PWM_CH0, PWM_CLK_DIV_8, PWM_CNT_UP, PWM_POLARITY_RISING);   //16M/8 = 2M
			//hal_pwm_set_count_val(PWM_CH0, lxl_pwm_value_light[0], MY_PWM_CNTTOP_VALUE);  // 2M/MY_PWM_CNTTOP_VALUE = 频率
			hal_pwm_open_channel(PWM_CH0, MY_GPIO_PWM_M0_1);
		break;
		case 1:
			hal_pwm_init(PWM_CH1, PWM_CLK_DIV_8, PWM_CNT_UP, PWM_POLARITY_RISING);
			//hal_pwm_set_count_val(PWM_CH1, lxl_pwm_value_light[1], 255);
			hal_pwm_open_channel(PWM_CH1, 255);
		break;
		#if 0
		case 2:
			hal_pwm_init(PWM_CH2, PWM_CLK_DIV_64, PWM_CNT_UP, PWM_POLARITY_FALLING);
			hal_pwm_set_count_val(PWM_CH2, s_light[2], CNTTOPVALUE);
			hal_pwm_open_channel(PWM_CH2, GPIO_RED);
		break;
		#endif
		default:
		break;
	}
	hal_pwm_start();
	return PPlus_SUCCESS;
}

int lxl_pwm_off(uint8_t ch)
{
	if(ch >2)
		return PPlus_ERR_INVALID_PARAM;

	lxl_pwm_value_light[ch] = 0;
	switch(ch){
		case 0:
			lxl_pwm_value_light[0] = 0;
			hal_pwm_close_channel(PWM_CH0);
			hal_pwm_destroy(PWM_CH0);
			hal_gpio_write(MY_GPIO_PWM_M0_1, 0);
		break;
		case 1:
			lxl_pwm_value_light[1] = 0;
			hal_pwm_close_channel(PWM_CH1);
			hal_pwm_destroy(PWM_CH1);
			hal_gpio_write(MY_GPIO_BACK_LED_1, 0);
		break;
		#if 0
		case 2:
			s_light[2] = 0;
			hal_pwm_close_channel(PWM_CH2);
			hal_pwm_destroy(PWM_CH2);
		break;
		#endif
		default:
		break;
	}
	#if 0
	if((lxl_pwm_value_light[0] + lxl_pwm_value_light[1] + lxl_pwm_value_light[2])==0){
		hal_pwm_stop();
	}
	#else 
	if((lxl_pwm_value_light[0])==0){
		hal_pwm_stop();
	}
	#endif
	return PPlus_SUCCESS;
}

int lxl_pwm_init(void)
{
  lxl_pwm_value_light[0] = 0;
  lxl_pwm_value_light[1] = 0;
  lxl_pwm_value_light[2] = 0;

  return PPlus_SUCCESS;
}
 

