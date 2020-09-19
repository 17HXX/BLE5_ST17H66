/**************************************************************************************************
*******
**************************************************************************************************/

#include "bus_dev.h"
#include "gpio.h"
#include "clock.h"
#include "timer.h"
#include "jump_function.h"
#include "pwrmgr.h"
#include "mcu.h"
#include "gpio.h"
#include "log.h"
#include "rf_phy_driver.h"
#include "flash.h"
#include "fs.h"
#include "version.h"

#define DEFAULT_UART_BAUD   115200


/*********************************************************************
 * LOCAL FUNCTION PROTOTYPES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

extern void init_config(void);
extern int app_main(void);
extern void hal_rom_boot_init(void);
/*********************************************************************
*  CONNECTION CONTEXT RELATE DEFINITION
*/
#define   BLE_MAX_ALLOW_CONNECTION              1
#define   BLE_MAX_ALLOW_PKT_PER_EVENT_TX        3
#define   BLE_MAX_ALLOW_PKT_PER_EVENT_RX        3

#define   BLE_PKT_VERSION                       BLE_PKT_VERSION_4_0 //BLE_PKT_VERSION_5_1 //BLE_PKT_VERSION_5_1     



/* BLE_MAX_ALLOW_PER_CONNECTION 
{
    ...
    struct ll_pkt_desc *tx_conn_desc[MAX_LL_BUF_LEN];     // new Tx data buffer
    struct ll_pkt_desc *rx_conn_desc[MAX_LL_BUF_LEN];

    struct ll_pkt_desc *tx_not_ack_pkt;
    struct ll_pkt_desc *tx_ntrm_pkts[MAX_LL_BUF_LEN];  
    ...
 }   
 tx_conn_desc[] + tx_ntrm_pkts[]    --> BLE_MAX_ALLOW_PKT_PER_EVENT_TX * BLE_PKT_BUF_SIZE*2
 rx_conn_desc[]             --> BLE_MAX_ALLOW_PKT_PER_EVENT_RX * BLE_PKT_BUF_SIZE
 tx_not_ack_pkt             --> 1*BLE_PKT_BUF_SIZE
 
*/

#define   BLE_PKT_BUF_SIZE                  (((BLE_PKT_VERSION == BLE_PKT_VERSION_5_1) ? 1 : 0) *  BLE_PKT51_LEN \
	                                        + ((BLE_PKT_VERSION == BLE_PKT_VERSION_4_0) ? 1 : 0) * BLE_PKT40_LEN \
	                                        + (sizeof(struct ll_pkt_desc) - 2))
                                                
#define   BLE_MAX_ALLOW_PER_CONNECTION          ( (BLE_MAX_ALLOW_PKT_PER_EVENT_TX * BLE_PKT_BUF_SIZE*2) \
                                                 +(BLE_MAX_ALLOW_PKT_PER_EVENT_RX * BLE_PKT_BUF_SIZE)   \
                                                 + BLE_PKT_BUF_SIZE )
                                                 
#define   BLE_CONN_BUF_SIZE                 (BLE_MAX_ALLOW_CONNECTION * BLE_MAX_ALLOW_PER_CONNECTION)
                                                                                        

__align(4) uint8            g_pConnectionBuffer[BLE_CONN_BUF_SIZE];
llConnState_t               pConnContext[BLE_MAX_ALLOW_CONNECTION];

/*********************************************************************
*   CTE IQ SAMPLE BUF config
*/
//#define BLE_SUPPORT_CTE_IQ_SAMPLE TRUE
#ifdef BLE_SUPPORT_CTE_IQ_SAMPLE 
uint16 g_llCteSampleI[LL_CTE_MAX_SUPP_LEN * LL_CTE_SUPP_LEN_UNIT];
uint16 g_llCteSampleQ[LL_CTE_MAX_SUPP_LEN * LL_CTE_SUPP_LEN_UNIT];
#endif


/*********************************************************************
*  OSAL LARGE HEAP CONFIG
*/
#define     LARGE_HEAP_SIZE  (4*1024)
uint8       g_largeHeap[LARGE_HEAP_SIZE];

/*********************************************************************
 * GLOBAL VARIABLES
 */
volatile uint8 g_clk32K_config;
volatile sysclk_t g_spif_clk_config;


/*********************************************************************
 * EXTERNAL VARIABLES
 */
extern uint32_t  __initial_sp;


static void hal_low_power_io_init(void)
{
   //========= pull all io to gnd by default
    ioinit_cfg_t ioInit[]= {
        //TSOP6252 10 IO
       {GPIO_P02,   GPIO_FLOATING   },/*SWD*/
       {GPIO_P03,   GPIO_FLOATING   },/*SWD*/
       {GPIO_P09,   GPIO_PULL_UP    },/*UART TX*/
       {GPIO_P10,   GPIO_PULL_UP    },/*UART RX*/
       {GPIO_P11,   GPIO_PULL_DOWN  },
       {GPIO_P14,   GPIO_PULL_DOWN  },
       {GPIO_P15,   GPIO_PULL_DOWN  },
       {GPIO_P16,   GPIO_FLOATING   },
       {GPIO_P18,   GPIO_PULL_DOWN  },
       {GPIO_P20,   GPIO_PULL_DOWN  },
#if(SDK_VER_CHIP==__DEF_CHIP_QFN32__)
        //6222 23 IO  
       {GPIO_P00,   GPIO_PULL_DOWN  },
       {GPIO_P01,   GPIO_PULL_DOWN  },
       {GPIO_P07,   GPIO_PULL_DOWN  },
       {GPIO_P17,   GPIO_FLOATING   },/*32k xtal*/
       {GPIO_P23,   GPIO_PULL_DOWN  },
       {GPIO_P24,   GPIO_PULL_DOWN  },
       {GPIO_P25,   GPIO_PULL_DOWN  },
       {GPIO_P26,   GPIO_PULL_DOWN  },
       {GPIO_P27,   GPIO_PULL_DOWN  },
       {GPIO_P31,   GPIO_PULL_DOWN  },
       {GPIO_P32,   GPIO_PULL_DOWN  },
       {GPIO_P33,   GPIO_PULL_DOWN  },
       {GPIO_P34,   GPIO_PULL_DOWN  },
#endif  
    };
    for(uint8_t i=0;i<sizeof(ioInit)/sizeof(ioinit_cfg_t);i++)
        hal_gpio_pull_set(ioInit[i].pin,ioInit[i].type);

    DCDC_CONFIG_SETTING(0x0f);
    DIG_LDO_CURRENT_SETTING(0x01);
    //hal_pwrmgr_RAM_retention(RET_SRAM0|RET_SRAM1|RET_SRAM2);
    hal_pwrmgr_RAM_retention(RET_SRAM0|RET_SRAM1);
    hal_pwrmgr_RAM_retention_set(); 
    hal_pwrmgr_LowCurrentLdo_enable();
 
}

static void ble_mem_init_config(void)
{
    	
    osal_mem_set_heap((osalMemHdr_t *)g_largeHeap, LARGE_HEAP_SIZE);
    
    LL_InitConnectContext(pConnContext, 
                        g_pConnectionBuffer, 
                        BLE_MAX_ALLOW_CONNECTION, 
                        BLE_MAX_ALLOW_PKT_PER_EVENT_TX,
                        BLE_MAX_ALLOW_PKT_PER_EVENT_RX,
                        BLE_PKT_VERSION);
    
#ifdef	BLE_SUPPORT_CTE_IQ_SAMPLE 				
	LL_EXT_Init_IQ_pBuff(g_llCteSampleI,g_llCteSampleQ);
#endif
    
}

static void hal_rfphy_init(void)
{
    //============config the txPower
    g_rfPhyTxPower  = RF_PHY_TX_POWER_0DBM ;
    //============config BLE_PHY TYPE
    g_rfPhyPktFmt   = PKT_FMT_BLE1M;
    //============config RF Frequency Offset
    g_rfPhyFreqOffSet   =RF_PHY_FREQ_FOFF_00KHZ;
    //============config xtal 16M cap
    XTAL16M_CAP_SETTING(0x09);
    XTAL16M_CURRENT_SETTING(0x01);

    hal_rom_boot_init();

    NVIC_SetPriority((IRQn_Type)BB_IRQn,    IRQ_PRIO_REALTIME);
    NVIC_SetPriority((IRQn_Type)TIM1_IRQn,  IRQ_PRIO_HIGH);     //ll_EVT
    NVIC_SetPriority((IRQn_Type)TIM2_IRQn,  IRQ_PRIO_HIGH);     //OSAL_TICK
    NVIC_SetPriority((IRQn_Type)TIM4_IRQn,  IRQ_PRIO_HIGH);     //LL_EXA_ADV

    //ble memory init and config
    ble_mem_init_config();

}


static void hal_init(void)
{
    hal_low_power_io_init();
    clk_init(g_system_clk); //system init
    hal_rtc_clock_config((CLK32K_e)g_clk32K_config);

    hal_pwrmgr_init();
    
    xflash_Ctx_t cfg = 
    {
        .spif_ref_clk   =   SYS_CLK_DLL_64M,
        .rd_instr       =   XFRD_FCMD_READ_DUAL
    };
    hal_spif_cache_init(cfg);
    LOG_INIT();
    hal_gpio_init();				
    hal_fs_init(0x1103C000,3);
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////
int  main(void)  
{
    g_system_clk = SYS_CLK_XTAL_16M;//SYS_CLK_XTAL_16M;//SYS_CLK_DLL_48M;
    g_clk32K_config = CLK_32K_RCOSC;//CLK_32K_XTAL;//CLK_32K_XTAL,CLK_32K_RCOSC      
    
    drv_irq_init();
    
    init_config();
    
    hal_rfphy_init();
    
    hal_init();

#if(_BUILD_FOR_DTM_==1)
    //if(hal_gpio_read(P33)) 
        rf_phy_direct_test();
#endif

    
        
	LOG("SDK Version ID %08x \n",SDK_VER_RELEASE_ID);
    LOG("rfClk %d rcClk %d sysClk %d tpCap[%02x %02x]\n",g_rfPhyClkSel,g_clk32K_config,g_system_clk,g_rfPhyTpCal0,g_rfPhyTpCal1);

    LOG("sizeof(struct ll_pkt_desc) = %d, buf size = %d\n", sizeof(struct ll_pkt_desc), BLE_CONN_BUF_SIZE);
    LOG("sizeof(g_pConnectionBuffer) = %d, sizeof(pConnContext) = %d, sizeof(largeHeap)=%d \n",\
         sizeof(g_pConnectionBuffer), sizeof(pConnContext),sizeof(g_largeHeap));  
    app_main();	

}


/////////////////////////////////////  end  ///////////////////////////////////////









	









