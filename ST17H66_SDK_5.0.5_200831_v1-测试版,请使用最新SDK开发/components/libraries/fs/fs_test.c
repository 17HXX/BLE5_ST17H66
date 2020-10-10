/**************************************************************************************************
THIS IS EMPTY HEADER
**************************************************************************************************/
#include "rom_sym_def.h"
#include "osal.h"
#include "crc16.h"
#include "fs.h"
#include "error.h"
#include "fs_test.h"
#include "clock.h"
#include "log.h"

//#warning "this api will be defined in patch"
//extern uint16_t crc16(uint16_t seed, const volatile void * p_data, uint32_t size);
//extern void WaitMs(uint32_t msecond);

#ifdef FS_MODULE_TEST

#define FTST_MAX_FILE_CNT   256
#define FTST_MAX_FILE_SIZE  4095

typedef struct{
  uint16_t fid;
  uint16_t fsize;
  uint16_t fcrc;
  uint16_t del_flg;//0: file exist; 1: file deleted
}ftst_t;

static ftst_t s_fpool[FTST_MAX_FILE_CNT];
static uint16_t s_ftst_fid_num = 0;
uint8_t s_ftst_buf[4095];

static void fstest_init(void)
{
  int i = 0;
  osal_memset((void*)s_fpool, 0, sizeof(s_fpool));
  osal_memset(s_ftst_buf, 0, sizeof(s_ftst_buf));
  s_ftst_fid_num = 0;
  for(i = 0; i< FTST_MAX_FILE_CNT; i++)
  {
    s_fpool[i].fid = 0xffff;
  }
}

uint16_t fstest_gen_new_fid(uint16_t fid_num_limit)
{
	uint16_t id;
	uint32_t i;
	bool new_flag = false;
	
	do
	{
		id = (uint16_t)osal_rand();
		new_flag = true;

		for(i = 0; i< s_ftst_fid_num; i++)
		{
			if((s_fpool[i].fid == id)&&(s_fpool[i].del_flg == false))
			{
				new_flag = false;	
				break;
			}
		}
	}
	while(new_flag == false);
	
  if(s_ftst_fid_num >= fid_num_limit || s_ftst_fid_num >= FTST_MAX_FILE_CNT)
  {
    fid_num_limit = fid_num_limit>FTST_MAX_FILE_CNT ? FTST_MAX_FILE_CNT : fid_num_limit;
    id = id%s_ftst_fid_num;
    return s_fpool[id].fid;
  }
    
  if(id == 0xffff)
    id = id/2;

  return id;
   
}

static void fstest_gen_fsdata(ftst_t* pftst, uint16_t size_limit, uint16_t fid_num_limit)
{
  uint16_t size;
  uint16_t i;
	
  size_limit = size_limit >FTST_MAX_FILE_SIZE?FTST_MAX_FILE_SIZE:size_limit;
  size = 1+ osal_rand()%(size_limit-1);
	
  if(pftst->fid == 0xffff){
    pftst->fid = fstest_gen_new_fid(fid_num_limit);
  }

  for(i = 0; i< size; i++){
    s_ftst_buf[i] = (uint8_t)(osal_rand()&0xff);
  }

  pftst->del_flg = 0;
  pftst->fcrc = crc16(0, s_ftst_buf, size);
  pftst->fsize = size;
}


static uint16_t fstest_save_fsdata(ftst_t* pftst, uint16_t fid_num_limit)
{
  uint16_t i;
  fid_num_limit = (fid_num_limit > FTST_MAX_FILE_CNT )? FTST_MAX_FILE_CNT: fid_num_limit;
	
  for(i = 0; i< fid_num_limit; i++){
    if(s_fpool[i].fid == pftst->fid){
      s_fpool[i].del_flg = pftst->del_flg;
      s_fpool[i].fcrc = pftst->fcrc;
      s_fpool[i].fsize = pftst->fsize;
      return i;
    }
    if(s_fpool[i].fid == 0xffff){
			s_fpool[i].fid = pftst->fid;
      s_fpool[i].del_flg = pftst->del_flg;
      s_fpool[i].fcrc = pftst->fcrc;
      s_fpool[i].fsize = pftst->fsize;
      s_ftst_fid_num++;
      return i;
    }
  }

  LOG("file count out of test range\n");
	//while(1);;;
  return 0xffff;
}

bool fstest_validate_fsdata(ftst_t* pftst)
{
  int i;
  int ret;
  uint16_t crc = 0;
  ftst_t* pft;
	
  if(pftst){
    pft = pftst;
    ret = hal_fs_item_read(pft->fid, s_ftst_buf, pft->fsize, NULL);
    if(ret != 0){
      LOG("ftest_validate_fsdata failed!,fid[0x%x], fsize[%d]\n", pft->fid, pft->fsize);
			//while(1);;;
      return FALSE;
    }
    
    crc = crc16(0, s_ftst_buf, pft->fsize);
    if(crc != pft->fcrc){
      LOG("ftest_validate_fsdata CRC error!,fid[0x%x], fsize[%d]\n", pft->fid, pft->fsize);
			//while(1);;;
      return FALSE;
    }
    return TRUE;
  }
	
  for(i = 0; i< s_ftst_fid_num; i++){
    pft = &(s_fpool[i]);
    if(pft->del_flg)
      continue;
    ret = hal_fs_item_read(pft->fid, s_ftst_buf, pft->fsize, NULL);
    if(ret != 0){
      LOG("ftest_validate_fsdata failed!,fid[0x%x], fsize[%d]\n", pft->fid, pft->fsize);
			//hile(1);;;
      return FALSE;
    }
    
    crc = crc16(0, s_ftst_buf, pft->fsize);
    if(crc != pft->fcrc){
      LOG("ftest_validate_fsdata CRC error!,fid[0x%x], fsize[%d]\n", pft->fid, pft->fsize);
			//while(1);;;
      return FALSE;
    }
  }
	
  return TRUE;  
}

int fstest_del(void)
{
	int ret;
	uint32_t id_index;
	
	while(1){
		id_index = osal_rand()%s_ftst_fid_num;
		if(s_fpool[id_index].del_flg == false){
			break;
		}
	}
	
	ret = hal_fs_item_del(s_fpool[id_index].fid);
	if(ret == PPlus_SUCCESS){
		s_fpool[id_index].del_flg = true;
		return PPlus_SUCCESS;
	}
	
	return PPlus_ERR_FATAL;
}

void ftcase_simple_write_test(void)
{
  int i;
  int iret;
  uint16_t index;
  char* str_result = "Skip";
  ftst_t ftst_item;
  bool error_flag = false;
  
	fstest_init();

	//if(PPlus_ERR_FS_CONTEXT == hal_fs_init(FS_OFFSET_ADDRESS,FS_SECTOR_NUM))
	{
			iret = hal_fs_format(FS_OFFSET_ADDRESS,FS_SECTOR_NUM);
			if(iret != PPlus_SUCCESS){
				LOG("format error\n");
			while(1);
		}
	}
	
 	str_result = "Success";
  
	for(i = 0; i < 20; i++)
	{
		ftst_item.fid = 0xffff;
    fstest_gen_fsdata(&ftst_item, 256, 0xffff);//size_limit fid_num_limit
    iret = hal_fs_item_write(ftst_item.fid,s_ftst_buf,ftst_item.fsize);
    if(iret != 0){
      LOG("fs write error[%d], fid[0x%x], fsize[%d]\n",iret,ftst_item.fid,ftst_item.fsize);
      str_result = "FS data validation error";
			error_flag = true;
			//while(1);;;
      break;
    }
    index = fstest_save_fsdata(&ftst_item,256);
    if(index == 0xffff){
      LOG("fs save error[%d], fid[0x%x], fsize[%d]\n",index, ftst_item.fid, ftst_item.fsize);
      str_result = "FS data validation error";
			error_flag = true;
      break;
    }
    if(fstest_validate_fsdata(NULL) == FALSE){
      str_result = "FS data validation error";
			error_flag = true;
      break;
    }
  }

  LOG("fstest_simple_write_test %s\n", str_result);
	
	if(error_flag == true)
	{
		LOG("fs error!\n");
		while(1);;;
	}
}


void ftcase_write_del_test(void)
{
	int i;
	int iret;
	uint16_t index;
	uint32_t garbage_num = 0;
	char* str_result = "Skip";
	ftst_t ftst_item;
	bool error_flag = false;
  
	fstest_init();
	
		//if(PPlus_ERR_FS_CONTEXT == hal_fs_init(FS_OFFSET_ADDRESS,FS_SECTOR_NUM))
	{
			iret = hal_fs_format(FS_OFFSET_ADDRESS,FS_SECTOR_NUM);
			if(iret != PPlus_SUCCESS){
				LOG("format error\n");
			while(1);
		}
	}
	
 	str_result = "Success";
  
	for(i=0;;i++){
		ftst_item.fid = 0xffff;
    fstest_gen_fsdata(&ftst_item, 256, 0xfffe);
		
		if(ftst_item.fsize > hal_fs_get_free_size()){
			LOG("test end!\n");
			break;
		}
		
    iret = hal_fs_item_write(ftst_item.fid , s_ftst_buf, ftst_item.fsize);
    if(iret != 0){
      LOG("fs write error[%d], fid[0x%x], fsize[%d]\n", iret,ftst_item.fid, ftst_item.fsize);
      str_result = "FS data validation error";
			error_flag = true;
      break;
    }
		
    index = fstest_save_fsdata(&ftst_item, 256);
    if(index == 0xffff){
      LOG("fs save error[%d], fid[0x%x], fsize[%d]\n", iret,ftst_item.fid, ftst_item.fsize);
      str_result = "FS data validation error";
			error_flag = true;
      break;
    }
		
    if(fstest_validate_fsdata(NULL) == FALSE){
      str_result = "FS data validation error";
			error_flag = true;
      break;
    }
		
		if(((i%8)==0) && (i>0)){
			iret = fstest_del();
			if(iret != PPlus_SUCCESS){
				str_result = "FS del error";
				error_flag = true;
			}
		}
		
		if(hal_fs_get_garbage_size(&garbage_num) > 256){
			iret = hal_fs_garbage_collect();
			if(iret != PPlus_SUCCESS){
				str_result = "FS compresss error";
				error_flag = true;
			}
			else{
				LOG("compress_ok\n");
			}
		}
		
		WaitMs(1);
  }

  LOG("ftcase_write_del_test %s\n", str_result);
	
	if(error_flag == true)
	{
		LOG("fs error!\n");
		while(1);;;
	}
}


void ftcase_write_del_and_ble_enable_test(void)
{
  static int i;
  int iret;
  uint16_t index;
  ftst_t ftst_item;
  bool error_flag = false;
  static bool firstFlag = true;
	uint32_t garbage_num = 0;
	
	if(firstFlag == true)
	{
		fstest_init();
		
	//if(PPlus_ERR_FS_CONTEXT == hal_fs_init(FS_OFFSET_ADDRESS,FS_SECTOR_NUM))
		{
			iret = hal_fs_format(FS_OFFSET_ADDRESS,FS_SECTOR_NUM);
		
			if(iret != PPlus_SUCCESS){
				LOG("format error\n");
				while(1);
			}
		}
		firstFlag = false;
		i =0;
  }
	else
	{
		i++;
	}

	{
		ftst_item.fid = 0xffff;
    fstest_gen_fsdata(&ftst_item, 256, 0xfffe);
		if(ftst_item.fsize > hal_fs_get_free_size()){
			LOG("\nreinit_and_test:%d %d \n",ftst_item.fsize,hal_fs_get_free_size());
			firstFlag = true;
			return;
		}
		
    iret = hal_fs_item_write(ftst_item.fid , s_ftst_buf, ftst_item.fsize);
    if(iret != 0){
      LOG("fs write error[%d], fid[0x%x], fsize[%d],%d\n", iret,ftst_item.fid, ftst_item.fsize);
			iret = hal_fs_item_write(ftst_item.fid , s_ftst_buf, ftst_item.fsize);//to debug
			error_flag = true;
      while(1);;;
    }
		
    index = fstest_save_fsdata(&ftst_item, 256);
    if(index == 0xffff){
      LOG("fs save error[%d], fid[0x%x], fsize[%d]\n", iret,ftst_item.fid, ftst_item.fsize);
			error_flag = true;
      while(1);;;
    }
    if(fstest_validate_fsdata(NULL) == FALSE){
			LOG("FS data validation error\n");
			error_flag = true;
      while(1);;;
    }
		
		if(((i%4)==0) && (i>0)){
			iret = fstest_del();
			if(iret != PPlus_SUCCESS){
				LOG("FS del error\n");
				error_flag = true;
				while(1);;;
			}
			else{
				LOG(".");
			}
		}
		
		if(hal_fs_get_garbage_size(&garbage_num) > 1024){
			iret = hal_fs_garbage_collect();
			if(iret != PPlus_SUCCESS){
				LOG("FS compresss error,%d\n",iret);
				error_flag = true;
				while(1);
			}
			else{
				LOG("compress_ok\n");
			}
		}
  }

	if(error_flag == true)
	{
		LOG("fs error!\n");
		while(1);;;
	}
}

#endif

#ifdef FS_EXAMPLE

#define FS_ITEM_TEST2 127
uint8_t id_buf[4095];
void fs_example(void)
{
	int ret;
	uint32_t free_size;
	uint16 id = 1,id_len;
	uint16 i,file_len;
	static uint8_t testCase = 1;
	static uint8_t testCycle = 0;
	bool errFlag;
	uint32_t garbage_num = 0;
	
	if(testCycle >= 3){
		LOG("fs example end!\n");
		return;
	}
		
	if(hal_fs_initialized() == FALSE){
		ret = hal_fs_init(FS_OFFSET_ADDRESS,FS_SECTOR_NUM);
		if(PPlus_SUCCESS != ret)
			LOG("error:%d\n",ret);
	}
	
	osal_memset(id_buf,0x00,4095);
	for(i=0;i<4095;i++){
		id_buf[i] = i%256;
	}
	
	switch(testCase)
	{
		case 1://write two files to fs,one is the mix length,one is the max length	
		{
				LOG("\nfs_write................................................\n");
				free_size = hal_fs_get_free_size();
				LOG("free_size:%d\n",free_size);
			
				id = 1;
				id_len = 1; 
				if(id_len < free_size){
					ret = hal_fs_item_write(id,id_buf,id_len);
					if(PPlus_SUCCESS != ret)
						LOG("error:%d\n",ret);
					else
						LOG("write ok\n");
				}

				id = FS_ITEM_TEST2;
				id_len = FS_ITEM_TEST2; 
				if(id_len < free_size){
					ret = hal_fs_item_write(id,id_buf,id_len);
					if(PPlus_SUCCESS != ret)
						LOG("error:%d\n",ret);
					else
						LOG("write ok\n");
				}
				free_size = hal_fs_get_free_size();
				LOG("free_size:%d\n",free_size);			
				break;
		}
		
		case 2://read the two files
		{			
			LOG("\nfs_read................................................\n");
			osal_memset(id_buf,0x00,1);
			id = 1;
			ret = hal_fs_item_read(id,id_buf,1,&file_len);
			if(PPlus_SUCCESS != ret)
				LOG("error:%d\n",ret);
			
			LOG("id:%d\n",id);
			LOG("id len:%d\n",file_len);
			LOG("id data:\n");
			errFlag = FALSE;
			for(i=0;i<file_len;i++){
				if(id_buf[i] != i%256){
					errFlag = TRUE;
					break;
				}
			}
			if(errFlag) 
				LOG("error\n");
			else
				LOG("ok\n");
			
			osal_memset(id_buf,0x00,FS_ITEM_TEST2);
			id = FS_ITEM_TEST2;
			hal_fs_item_read(id,id_buf,FS_ITEM_TEST2,&file_len);
			if(PPlus_SUCCESS != ret)
				LOG("error:%d\n",ret);
			
			LOG("\nid:%d\n",id);
			LOG("id len:%d\n",file_len);
			LOG("id data:\n");
			errFlag = FALSE;
			for(i=0;i<file_len;i++){
				if(id_buf[i] != i%256){
					errFlag = TRUE;
					break;
				}
			}
			if(errFlag) 
				LOG("error\n");
			else
				LOG("ok\n");
			
			break;
		}
		
		case 3://delete the two files
		{

			LOG("\nfs_delete................................................\n");
			LOG("free_size:%d\n",hal_fs_get_free_size());
			LOG("garbage_size:%d garbage_num:%d\n",hal_fs_get_garbage_size(&garbage_num),garbage_num);
			
			id = 1;
			ret = hal_fs_item_del(id);
			if(PPlus_SUCCESS != ret)
				LOG("error:%d\n",ret);
			else{
				LOG("ok\n");
				ret = hal_fs_item_read(id,id_buf,4095,&file_len);
				if(ret != PPlus_ERR_FS_NOT_FIND_ID)
					LOG("error:%d\n",ret);
			}
			
			id = FS_ITEM_TEST2;
			ret = hal_fs_item_del(id);
			if(PPlus_SUCCESS != ret)
				LOG("error:%d\n",ret);
			else{
				LOG("ok\n");
				ret = hal_fs_item_read(id,id_buf,FS_ITEM_TEST2,&file_len);
				if(ret != PPlus_ERR_FS_NOT_FIND_ID)
					LOG("error:%d\n",ret);
			}
			
			LOG("free_size:%d\n",hal_fs_get_free_size());
			LOG("garbage_size:%d garbage_num:%d\n",hal_fs_get_garbage_size(&garbage_num),garbage_num);
			break;
		}
		
		case 4://garbage collect
		{
			LOG("\nfs_garbage_collect................................................\n");
			LOG("free_size:%d\n",hal_fs_get_free_size());
			LOG("garbage_size:%d garbage_num:%d\n",hal_fs_get_garbage_size(&garbage_num),garbage_num);

			ret = hal_fs_garbage_collect();
			if(PPlus_SUCCESS != ret)
				LOG("error:%d\n",ret);
			
			LOG("free_size:%d\n",hal_fs_get_free_size());
			LOG("garbage_size:%d garbage_num:%d\n",hal_fs_get_garbage_size(&garbage_num),garbage_num);
			break;
		}
		
		default:
			break;
	}

	testCase++;
	if(testCase > 4){
		testCase = 1;
		testCycle++;
		LOG("\nfs test cycle:%d................................................\n",testCycle);
	}
}
#endif


#ifdef FS_TIMING_TEST



#define TOGGLE_GPIO GPIO_P14
uint8_t id_buf[4095];
void fs_timing_test(void)
{
	uint8_t testCase = 4,data_wr[4]={0x12,0x34,0x56,0x78};
	uint16_t i,file_len;
	int ret;
	uint32_t garbage_size,garbage_num;
	
	hal_gpio_write(TOGGLE_GPIO,1);	
	WaitMs(1);
	
	testCase = 9;
	switch(testCase)
	{
		case 0:
			hal_gpio_write(TOGGLE_GPIO,0);
			flash_sector_erase(0x11005000);
			hal_gpio_write(TOGGLE_GPIO,1);
			WaitMs(1);
			break;
		
		case 1:
			hal_gpio_write(TOGGLE_GPIO,0);		
			spif_write(0x1100500c,data_wr,4);
			hal_gpio_write(TOGGLE_GPIO,1);
			WaitMs(1);
			break;
		
		case 2:
			hal_gpio_write(TOGGLE_GPIO,0);
			osal_memcpy((uint8_t*)id_buf,(uint8_t*)0x11005000,4);
			hal_gpio_write(TOGGLE_GPIO,1);
			WaitMs(1);
			break;
		
		case 3:
			hal_gpio_write(TOGGLE_GPIO,0);
			ret = hal_fs_format(0x11005000,3);
			hal_gpio_write(TOGGLE_GPIO,1);
			WaitMs(1);
			if(ret != PPlus_SUCCESS){
				LOG("ret:%d",ret);
			}
			break;
		
		case 4:
			hal_gpio_write(TOGGLE_GPIO,0);
			ret = hal_fs_init(0x11005000,3);
			hal_gpio_write(TOGGLE_GPIO,1);
			WaitMs(1);
			if(ret != PPlus_SUCCESS){
				LOG("ret:%d",ret);
			}
			ret = hal_fs_item_write(1,id_buf,1);
			ret = hal_fs_item_write(2,id_buf,2);
			ret = hal_fs_item_write(3,id_buf,3);
			ret = hal_fs_item_write(4,id_buf,4);
			ret = hal_fs_item_write(5,id_buf,5);
			ret = hal_fs_item_write(6,id_buf,6);
			ret = hal_fs_item_write(7,id_buf,7);
			ret = hal_fs_item_write(8,id_buf,8);
			ret = hal_fs_item_write(9,id_buf,9);
			ret = hal_fs_item_write(10,id_buf,10);
			WaitMs(1);
			break;
				
		case 5:
			hal_gpio_write(TOGGLE_GPIO,0);
			ret = hal_fs_init(0x11005000,3);
			hal_gpio_write(TOGGLE_GPIO,1);
			WaitMs(1);
			if(ret != PPlus_SUCCESS){
				LOG("ret:%d",ret);
			}
		break;
		
		case 6:
			LOG("write file\n");
			for(i=0;i<4095;i++)
				id_buf[i] = (i+1)%256;
		
			ret = hal_fs_format(0x11005000,3);
			if(ret != PPlus_SUCCESS){
				LOG("ret:%d",ret);
			}
			WaitMs(1);
		
			hal_gpio_write(TOGGLE_GPIO,0);	
			ret = hal_fs_item_write(1,id_buf,1);		
			hal_gpio_write(TOGGLE_GPIO,1);
			WaitMs(5);
		
			hal_gpio_write(TOGGLE_GPIO,0);	
			ret = hal_fs_item_write(2,id_buf,100);
			hal_gpio_write(TOGGLE_GPIO,1);
			WaitMs(5);
		
			hal_gpio_write(TOGGLE_GPIO,0);	
			ret = hal_fs_item_write(3,id_buf,100);
			hal_gpio_write(TOGGLE_GPIO,1);
			WaitMs(5);
		break;
		
		case 7:
			LOG("read file\n");
			ret = hal_fs_init(0x11005000,3);
			WaitMs(1);
		
			hal_gpio_write(TOGGLE_GPIO,0);		
		  ret = hal_fs_item_read(1,id_buf,4095,&file_len);
			hal_gpio_write(TOGGLE_GPIO,1);			
			WaitMs(1);
			LOG("ret:%d",ret);
		
			hal_gpio_write(TOGGLE_GPIO,0);		
			ret = hal_fs_item_read(2,id_buf,4095,&file_len);
			hal_gpio_write(TOGGLE_GPIO,1);
			WaitMs(1);
			LOG("ret:%d",ret);
		
			hal_gpio_write(TOGGLE_GPIO,0);		
			ret = hal_fs_item_read(3,id_buf,4095,&file_len);
			hal_gpio_write(TOGGLE_GPIO,1);
			WaitMs(1);
			LOG("ret:%d",ret);
			break;
		
		case 8:
			LOG("del file\n");
			ret = hal_fs_init(0x11005000,3);
			WaitMs(1);
		
			hal_gpio_write(TOGGLE_GPIO,0);		
		  ret = hal_fs_item_del(1);
			hal_gpio_write(TOGGLE_GPIO,1);			
			WaitMs(1);
			LOG("ret:%d",ret);
		
			hal_gpio_write(TOGGLE_GPIO,0);		
			ret = hal_fs_item_del(2);
			hal_gpio_write(TOGGLE_GPIO,1);
			WaitMs(1);
			LOG("ret:%d",ret);
		
			hal_gpio_write(TOGGLE_GPIO,0);		
			ret =hal_fs_item_del(3);
			hal_gpio_write(TOGGLE_GPIO,1);
			WaitMs(1);
			LOG("ret:%d",ret);
			break;
		
		case 9:
			LOG("garbage calc and collect\n");
			ret = hal_fs_init(0x11005000,3);
			if(ret != PPlus_SUCCESS){
				LOG("hal_fs_init error:%d\n",ret);
			}
			
			WaitMs(5);
			hal_gpio_write(TOGGLE_GPIO,0);	
			garbage_size = hal_fs_get_garbage_size(&garbage_num);
			hal_gpio_write(TOGGLE_GPIO,1);	
			LOG("garbage_num:%d garbage_size:%d\n",garbage_num,garbage_size);
			
			
			WaitMs(5);
			hal_gpio_write(TOGGLE_GPIO,0);
			ret = hal_fs_garbage_collect();
			hal_gpio_write(TOGGLE_GPIO,1);
			if(ret != PPlus_SUCCESS){
				LOG("hal_fs_garbage_collect error:%d\n",ret);
			}
			
			WaitMs(5);
			break;
		
		default:
			break;
	}
	while(1);;;
}

#endif
