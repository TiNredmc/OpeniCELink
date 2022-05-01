/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_storage_if.c
  * @version        : v2.0_Cube
  * @brief          : Memory management layer.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "usbd_storage_if.h"

/* USER CODE BEGIN INCLUDE */
#include <stdio.h>
#include <stm32f1xx_hal.h>
#include <main.h>
/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device.
  * @{
  */

/** @defgroup USBD_STORAGE
  * @brief Usb mass storage device module
  * @{
  */

/** @defgroup USBD_STORAGE_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_STORAGE_Private_Defines
  * @brief Private defines.
  * @{
  */

#define STORAGE_LUN_NBR                  1
#define STORAGE_BLK_NBR                  0x10000
#define STORAGE_BLK_SIZ                  0x200

/* USER CODE BEGIN PRIVATE_DEFINES */
/* block size 512 bytes (STORAGE_BLK_SIZ = 0x200).
 * and block number is 128(STORAGE_BLK_NBR).
 */
#define SMALL_FS
// Redefine
#undef STORAGE_BLK_NBR
#undef STORAGE_BLK_SIZ
#ifdef SMALL_FS
#define STORAGE_BLK_NBR                  128//64
#define STORAGE_BLK_SIZ                  0x200//0x400
#else
#define STORAGE_BLK_NBR                  64
#define STORAGE_BLK_SIZ                  0x400
#endif
/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_STORAGE_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_STORAGE_Private_Variables
  * @brief Private variables.
  * @{
  */

/* USER CODE BEGIN INQUIRY_DATA_FS */
/** USB Mass storage Standard Inquiry Data. */
const int8_t STORAGE_Inquirydata_FS[] = {/* 36 */

  /* LUN 0 */
  0x00,
  0x80,
  0x02,
  0x02,
  (STANDARD_INQUIRY_DATA_LEN - 5),
  0x00,
  0x00,
  0x00,
  'T', 'L', 'H', 'X', ' ', ' ', ' ', ' ', /* Manufacturer : 8 bytes */
  'i', 'C', 'E', 'B', 'L', 'A', 'S', 'T', /* Product      : 16 Bytes */
  'E', 'R', ' ', ' ', ' ', ' ', ' ', ' ',
  '0', '.', '0' ,'1'                      /* Version      : 4 Bytes */
};
/* USER CODE END INQUIRY_DATA_FS */

/* USER CODE BEGIN PRIVATE_VARIABLES */
  static FLASH_EraseInitTypeDef USB_EraseInitStruct;
  uint32_t USB_PAGEError = 0;
/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_STORAGE_Exported_Variables
  * @brief Public variables.
  * @{
  */

extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_STORAGE_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t STORAGE_Init_FS(uint8_t lun);
static int8_t STORAGE_GetCapacity_FS(uint8_t lun, uint32_t *block_num, uint16_t *block_size);
static int8_t STORAGE_IsReady_FS(uint8_t lun);
static int8_t STORAGE_IsWriteProtected_FS(uint8_t lun);
static int8_t STORAGE_Read_FS(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len);
static int8_t STORAGE_Write_FS(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len);
static int8_t STORAGE_GetMaxLun_FS(void);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */


/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

USBD_StorageTypeDef USBD_Storage_Interface_fops_FS =
{
  STORAGE_Init_FS,
  STORAGE_GetCapacity_FS,
  STORAGE_IsReady_FS,
  STORAGE_IsWriteProtected_FS,
  STORAGE_Read_FS,
  STORAGE_Write_FS,
  STORAGE_GetMaxLun_FS,
  (int8_t *)STORAGE_Inquirydata_FS
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initializes over USB FS IP
  * @param  lun:
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t STORAGE_Init_FS(uint8_t lun)
{
  /* USER CODE BEGIN 2 */
	HAL_FLASH_Unlock();
  return (USBD_OK);
  /* USER CODE END 2 */
}

/**
  * @brief  .
  * @param  lun: .
  * @param  block_num: .
  * @param  block_size: .
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t STORAGE_GetCapacity_FS(uint8_t lun, uint32_t *block_num, uint16_t *block_size)
{
  /* USER CODE BEGIN 3 */
  *block_num  = STORAGE_BLK_NBR;
  *block_size = STORAGE_BLK_SIZ;
  return (USBD_OK);
  /* USER CODE END 3 */
}

/**
  * @brief  .
  * @param  lun: .
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t STORAGE_IsReady_FS(uint8_t lun)
{
  /* USER CODE BEGIN 4 */
  return (USBD_OK);
  /* USER CODE END 4 */
}

/**
  * @brief  .
  * @param  lun: .
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t STORAGE_IsWriteProtected_FS(uint8_t lun)
{
  /* USER CODE BEGIN 5 */
  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  .
  * @param  lun: .
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t STORAGE_Read_FS(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len)
{
  /* USER CODE BEGIN 6 */
	// convert block address (block number) into absolute address
#ifdef SMALL_FS
	uint32_t inbuf_addr = blk_addr << 9;// 1 sector has 512 bytes in size. 1 << 9 = 1*512.
#else
	uint32_t inbuf_addr = blk_addr << 10; // 1 sector has 1024 bytes in size. 1 << 10 = 1*1024.
#endif

	memcpy(buf, (uint8_t *)(FLASH_MEM_BASE_ADDR + inbuf_addr),  STORAGE_BLK_SIZ);
	return (USBD_OK);
  /* USER CODE END 6 */
}

/**
  * @brief  .
  * @param  lun: .
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t STORAGE_Write_FS(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len)
{
  /* USER CODE BEGIN 7 */
#ifdef SMALL_FS
	static uint8_t mod_space[FLASH_PAGE_SIZE]; // space to copy data from flash for modifying data before page erase and re-flash
	// convert sector number into physical address of the flash.
	// 1 sector has 512 bytes in size.

	uint32_t temp_buf = 0; // temp buffer to merge four bytes into 4 byte 32 bit (word).
	uint32_t cpy_addr = 0;

	// Unlock flash for writing
	HAL_FLASH_Unlock();

	// Since our flash page size is 1024 bytes, but fatfs can only write 512 bytes each time (512 bytes per 1 sector).
	// Which it means that 1 flash page fits 2 fatfs sectors.
	// Thus, in order to write properly. Even sector number will we written to lower half of flash page [0-511].
	// and Odd sector number will we written to upper half of flash page [512-1023].

	// read back from flash.
	// use sector number to calculate the physical page offset of flash of that sector number.
	// (sector/2)*FLASH_PAGE_SIZE always return page-aligned number a.k.a page starting address.
	cpy_addr = (FLASH_MEM_BASE_ADDR	+ ((blk_addr >> 1) << 10));
	memcpy(mod_space, (uint8_t*) cpy_addr,	FLASH_PAGE_SIZE);

	// modify data in buffer.
	// Even sector number reads from [0-511]. Odd sector number reads from [512-1023].
	memcpy(mod_space + ((blk_addr % 2) << 9), buf, 512);
	//buff+=512;

	// Page erase
	USB_EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES; // erase 1024 KBytes (which is the size of 1 page).
	USB_EraseInitStruct.PageAddress = cpy_addr; // We start erase from the beginning of sector.
	USB_EraseInitStruct.NbPages = 1; // this tells eraser for how many page we want to erase. Which is 1 page.

	HAL_FLASHEx_Erase(&USB_EraseInitStruct, &USB_PAGEError);

	// flash the modified data back to Flash memory.
	for (uint32_t i = 0; i < FLASH_PAGE_SIZE; i+=4) {
		temp_buf = mod_space[i] | mod_space[i+1] << 8 | mod_space[i+2] << 16
				| mod_space[i+3] << 24; // parse byte n n+1 n+2 and n+3
		//memcpy(&temp_buf, mod_space+(i*4), 4);// Copy 4 byte into one DWORD (unt32_t byte).
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
				USB_EraseInitStruct.PageAddress + i, temp_buf); // flash modified data onto Flash memory.

	}
#else
	static uint32_t temp_buf;// temp buffer to merge 4 8-bit (byte) into 32 bit (word).

	// convert block address (block number) into absolute address
	uint32_t inbuf_addr = blk_addr << 10; // 1 sector has 1024 bytes in size. 1 << 10 = 1*1024.

	// Unlock flash for writing
	HAL_FLASH_Unlock();

	// Page erase
	USB_EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES; // erase 1024 KBytes (which is the size of 1 page).
	USB_EraseInitStruct.PageAddress = FLASH_MEM_BASE_ADDR + inbuf_addr; // We start erase from the beginning of sector.
	USB_EraseInitStruct.NbPages = 1; // this tells eraser for how many page we want to erase.

	HAL_FLASHEx_Erase(&USB_EraseInitStruct, &USB_PAGEError);

	// flash the data to Flash memory
	for(uint32_t i = 0; i < STORAGE_BLK_SIZ;i+=4){
	temp_buf = *buf | (*(buf + 1) << 8) | (*(buf + 2) << 16)
			| (*(buf + 3) << 24); // parse byte n n+1 n+2 and n+3

	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
			USB_EraseInitStruct.PageAddress + i, temp_buf); // flash data onto Flash memory.

	buf += 4;
	}
#endif
  return (USBD_OK);
  /* USER CODE END 7 */
}

/**
  * @brief  .
  * @param  None
  * @retval .
  */
int8_t STORAGE_GetMaxLun_FS(void)
{
  /* USER CODE BEGIN 8 */
  return (STORAGE_LUN_NBR - 1);
  /* USER CODE END 8 */
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */

