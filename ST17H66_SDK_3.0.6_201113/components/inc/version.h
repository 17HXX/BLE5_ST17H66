/**************************************************************************************************
 
  Lenze Microelectronics Limited confidential and proprietary. 
  All rights reserved.
 
  IMPORTANT: All rights of this software belong to Lenze Microelectronics 
  Limited ("Lenze"). Your use of this Software is limited to those 
  specific rights granted under  the terms of the business contract, the 
  confidential agreement, the non-disclosure agreement and any other forms 
  of agreements as a customer or a partner of Lenze. You may not use this 
  Software unless you agree to abide by the terms of these agreements. 
  You acknowledge that the Software may not be modified, copied, 
  distributed or disclosed unless embedded on a Lenze Bluetooth Low Energy 
  (BLE) integrated circuit, either as a product or is integrated into your 
  products.  Other than for the aforementioned purposes, you may not use, 
  reproduce, copy, prepare derivative works of, modify, distribute, perform, 
  display or sell this Software and/or its documentation for any purposes.
 
  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  LENZE OR ITS SUBSIDIARIES BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
  
**************************************************************************************************/
 
/*******************************************************************************
* @file		sdk_version.h
* @brief	
* @author	
* 
*
*******************************************************************************/
#ifndef __SDK_VER_H__
#define __SDK_VER_H__
 
#define __DEF_CHIP_QFN32__                  (0x0001) 
#define __DEF_CHIP_TSOP16__                  (0x0002) 
#define SDK_VER_MAJOR                      3
#define SDK_VER_MINOR                      0
#define SDK_VER_REVISION                   6
#define SDK_VER_RELEASE_ID                 ((SDK_VER_MAJOR<<16)|(SDK_VER_MINOR<<8)|(SDK_VER_REVISION))
#define SDK_VER_CHIP                      __DEF_CHIP_TSOP16__
//#define SDK_VER_TEST_BUILD ""
#endif

