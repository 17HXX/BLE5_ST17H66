/**************************************************************************************************
THIS IS EMPTY HEADER
**************************************************************************************************/
#include "rom_sym_def.h"
#include "types.h"
#include "ll_sleep.h"
#include "bus_dev.h"
#include "string.h"

#include "pwrmgr.h"
#include "error.h"
#include "gpio.h"
#include "log.h"
#include "clock.h"
#include "jump_function.h"

#if(CFG_SLEEP_MODE == PWR_MODE_NO_SLEEP)
static uint8_t mPwrMode = PWR_MODE_NO_SLEEP;
#elif(CFG_SLEEP_MODE == PWR_MODE_SLEEP)
static uint8_t mPwrMode = PWR_MODE_SLEEP;
#elif(CFG_SLEEP_MODE == PWR_MODE_PWROFF_NO_SLEEP)
static uint8_t mPwrMode = PWR_MODE_PWROFF_NO_SLEEP;
#else
#error "CFG_SLEEP_MODE define incorrect"
#endif


typedef struct _pwrmgr_Context_t{
  MODULE_e     moudle_id;
  bool         lock;
  pwrmgr_Hdl_t sleep_handler;
  pwrmgr_Hdl_t wakeup_handler;
}pwrmgr_Ctx_t;

static pwrmgr_Ctx_t mCtx[HAL_PWRMGR_TASK_MAX_NUM];
static uint32_t sramRet_config;
static uint32_t s_config_swClk0 = DEF_CLKG_CONFIG_0;
  
uint32_t s_config_swClk1 = DEF_CLKG_CONFIG_1;
uint32_t s_gpio_wakeup_src_group1,s_gpio_wakeup_src_group2;


int hal_pwrmgr_init(void)
{
  memset(&mCtx, 0, sizeof(mCtx));
  switch(mPwrMode){
  case PWR_MODE_NO_SLEEP:
  case PWR_MODE_PWROFF_NO_SLEEP:
    disableSleep();
    break;
  case PWR_MODE_SLEEP:
    enableSleep();
    break;
  }
  return PPlus_SUCCESS;
}

int hal_pwrmgr_clk_gate_config(MODULE_e module)
{
  if (module < MOD_CP_CPU)
  {
    s_config_swClk0 |= BIT(module);
  }
  else if (module < MOD_PCLK_CACHE)
  {
    s_config_swClk1 |= BIT(module - MOD_CP_CPU);
  }
  return PPlus_SUCCESS;
}

bool hal_pwrmgr_is_lock(MODULE_e mod)
{
  int i;
  int ret = FALSE;
  if(mPwrMode == PWR_MODE_NO_SLEEP || mPwrMode == PWR_MODE_PWROFF_NO_SLEEP ){
    return TRUE;
  }
	
  HAL_ENTER_CRITICAL_SECTION();
  for(i = 0; i< HAL_PWRMGR_TASK_MAX_NUM; i++){
    if(mCtx[i].moudle_id == MOD_NONE)
      break;
    if(mCtx[i].moudle_id == mod){
      if(mCtx[i].lock == TRUE)
        ret = TRUE;
      break;
    }
  }
  HAL_EXIT_CRITICAL_SECTION();
  return ret;
}


int hal_pwrmgr_lock(MODULE_e mod)
{
  int i;
  int ret = PPlus_ERR_NOT_REGISTED;
  if(mPwrMode == PWR_MODE_NO_SLEEP || mPwrMode == PWR_MODE_PWROFF_NO_SLEEP ){
    disableSleep();
    return PPlus_SUCCESS;
  }
	
  HAL_ENTER_CRITICAL_SECTION();
  for(i = 0; i< HAL_PWRMGR_TASK_MAX_NUM; i++){
    if(mCtx[i].moudle_id == MOD_NONE)
      break;
    if(mCtx[i].moudle_id == mod){
      mCtx[i].lock = TRUE;
      disableSleep();
      //LOG("LOCK\n");
      ret = PPlus_SUCCESS;
      break;
    }
  }
  HAL_EXIT_CRITICAL_SECTION();
  return ret;
}

int hal_pwrmgr_unlock(MODULE_e mod)
{
  int i, cnt = 0;
  
  if(mPwrMode == PWR_MODE_NO_SLEEP || mPwrMode == PWR_MODE_PWROFF_NO_SLEEP ){
    disableSleep();
    return PPlus_SUCCESS;
  }

  HAL_ENTER_CRITICAL_SECTION();
  for(i = 0; i< HAL_PWRMGR_TASK_MAX_NUM; i++){
    if(mCtx[i].moudle_id == MOD_NONE)
      break;

    if(mCtx[i].moudle_id == mod){
      mCtx[i].lock = FALSE;
    }
    if(mCtx[i].lock)
      cnt ++;
  }
  if(cnt == 0)
    enableSleep();
  else
    disableSleep();
  HAL_EXIT_CRITICAL_SECTION();
	
	//LOG("sleep mode:%d\n", isSleepAllow());
  
  return PPlus_SUCCESS;
}

int hal_pwrmgr_register(MODULE_e mod, pwrmgr_Hdl_t sleepHandle, pwrmgr_Hdl_t wakeupHandle)
{
  int i;
  pwrmgr_Ctx_t* pctx = NULL;
  for(i = 0; i< HAL_PWRMGR_TASK_MAX_NUM; i++){
    if(mCtx[i].moudle_id == mod)
      return PPlus_ERR_INVALID_STATE;
    if(mCtx[i].moudle_id == MOD_NONE){
      pctx = &mCtx[i];
      break;
    }
  }
  if(pctx == NULL)
    return PPlus_ERR_NO_MEM;
  pctx->lock = FALSE;
  pctx->moudle_id = mod;
  pctx->sleep_handler = sleepHandle;
  pctx->wakeup_handler = wakeupHandle;
  return PPlus_SUCCESS;
}

int hal_pwrmgr_unregister(MODULE_e mod)
{
  int i;
  pwrmgr_Ctx_t* pctx = NULL;
  for(i = 0; i< HAL_PWRMGR_TASK_MAX_NUM; i++){
    if(mCtx[i].moudle_id == mod){
      pctx = &mCtx[i];
      break;
    }
    if(mCtx[i].moudle_id == MOD_NONE){
      return PPlus_ERR_NOT_REGISTED;
    }
  }

  if(pctx == NULL)
    return PPlus_ERR_NOT_REGISTED;

  HAL_ENTER_CRITICAL_SECTION();
  memcpy(pctx, pctx+1, sizeof(pwrmgr_Ctx_t)*(HAL_PWRMGR_TASK_MAX_NUM-i-1));
  HAL_EXIT_CRITICAL_SECTION();
  return PPlus_SUCCESS;
}


int __attribute__((used)) hal_pwrmgr_wakeup_process(void)
{
  int i;
  
  AP_PCR->SW_CLK  = s_config_swClk0;
  AP_PCR->SW_CLK1 = s_config_swClk1|0x01;//force set M0 CPU

  s_gpio_wakeup_src_group1 = AP_AON->GPIO_WAKEUP_SRC[0];
  s_gpio_wakeup_src_group2 = AP_AON->GPIO_WAKEUP_SRC[1];
    
  for(i = 0; i< HAL_PWRMGR_TASK_MAX_NUM; i++){
    if(mCtx[i].moudle_id == MOD_NONE){
      return PPlus_ERR_NOT_REGISTED;
    }
    if(mCtx[i].wakeup_handler)
      mCtx[i].wakeup_handler();
  }
  return PPlus_SUCCESS;
}

int __attribute__((used)) hal_pwrmgr_sleep_process(void)
{
  int i;
    //20181013 ZQ :
  hal_pwrmgr_RAM_retention_set();
    //LOG("Sleep\n");
  for(i = 0; i< HAL_PWRMGR_TASK_MAX_NUM; i++){
    if(mCtx[i].moudle_id == MOD_NONE){
      return PPlus_ERR_NOT_REGISTED;
    }
    if(mCtx[i].sleep_handler)
      mCtx[i].sleep_handler();
  }


  
  return PPlus_SUCCESS;
}

/**************************************************************************************
 * @fn          hal_pwrmgr_RAM_retention
 *
 * @brief       This function process for enable retention sram
 *
 * input parameters
 *
 * @param       uint32_t sram: sram bit map
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      refer error.h.
 **************************************************************************************/
int hal_pwrmgr_RAM_retention(uint32_t sram)
{
  if(sram & 0xffffffe0)
  {
    sramRet_config = 0x00;
    return PPlus_ERR_INVALID_PARAM;
  } 

  sramRet_config = sram;
  return PPlus_SUCCESS;
}

int hal_pwrmgr_RAM_retention_clr(void)
{
  subWriteReg(0x4000f01c,21,17,0);
  return PPlus_SUCCESS;
}

int hal_pwrmgr_RAM_retention_set(void)
{
  subWriteReg(0x4000f01c,21,17,sramRet_config);
  return PPlus_SUCCESS;
}

int hal_pwrmgr_LowCurrentLdo_enable(void)
{
    subWriteReg(0x4000f014,26,26, 1);
    return PPlus_SUCCESS;
}

int hal_pwrmgr_LowCurrentLdo_disable(void)
{
    subWriteReg(0x4000f014,26,26, 0);
    return PPlus_SUCCESS;
}

int hal_pwrmgr_poweroff(pwroff_cfg_t* pcfg, uint8_t wakeup_pin_num)
{ 

  subWriteReg(0x4000f01c,6,6,0x00);   //disable software control

  uint8_t i = 0;
  for(i = 0; i < wakeup_pin_num; i++){
    hal_gpio_wakeup_set(pcfg[i].pin, pcfg[i].type);
  }
  //system off
//  #warning need check cache spif status
  write_reg(0x4000f000,0x5a5aa5a5);
	return PPlus_SUCCESS;
}






