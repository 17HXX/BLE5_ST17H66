/**************************************************************************************************
*******
**************************************************************************************************/

#ifndef __FS_TEST_H__
#define __FS_TEST_H__

//FS test config,please refer to Flash_Distribution
//#define FS_OFFSET_ADDRESS         0x1103c000 //for 256KB Flash
#define FS_OFFSET_ADDRESS         0x11034000 //for 512KB Flash
#define FS_SECTOR_NUM			  2


//#define FS_MODULE_TEST
#ifdef FS_MODULE_TEST
void ftcase_simple_write_test(void);
void ftcase_write_del_test(void);
void ftcase_write_del_and_ble_enable_test(void);
#endif

#define FS_EXAMPLE
#ifdef FS_EXAMPLE
void fs_example(void);
#endif

//#define FS_TIMING_TEST
#ifdef FS_TIMING_TEST
void fs_timing_test(void);
#endif

#endif
