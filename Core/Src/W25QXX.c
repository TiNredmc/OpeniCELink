/*
 * 25QXX.c
 *	Interface with Winbond SPI NOR flash that store FPGA's bitstream.
 *  Created on: Dec 14, 2021
 *      Author: TinLethax
 */
#include <W25QXX.h>
#include <main.h>


void w25_wr16(){}

void w25_rd16(){}

/* w25_init use for initialize the struct to store SPI bus, CS pin GPIO band and number.
 * also use for probing for the SPI NOR chip by detecting the JEDEC ID respond
 *
 */
uint8_t w25_init(W25QXX *w25q, SPI_HandleTypeDef *SPIbus,
		GPIO_TypeDef *CSBANK, uint16_t CS_PIN){
	// store to struct.
	w25q->SPIbus = SPIbus;
	w25q->CSBANK = CSBANK;
	w25q->CS_PIN = CS_PIN;

	// probe JEDEC
	uint8_t tempbuf[4] = {W25_CMD_JDEC, 0, 0, 0};
	// CE Low
	HAL_GPIO_WritePin(w25q->CSBANK, w25q->CS_PIN, GPIO_PIN_RESET);

	// Send command and receive the data at the same time.
	HAL_SPI_TransmitReceive(w25q->SPIbus, tempbuf, tempbuf, 4, 100);

	// CE High
	HAL_GPIO_WritePin(w25q->CSBANK, w25q->CS_PIN, GPIO_PIN_SET);

	// Chip checking.
	switch(tempbuf[1]){
		case(0xEF):// WinBond flash.
		case(0xC2):// Macronix flash.
		case(0xC8):// Gigadevice flash.
		case(0x9D):// ISSI flash.
			break;

		case(0x00):// Clearly no respond
		case(0xFF):// Also clearly no respond but with MISO line pull up variants that always read 1
			return 1;

		default:// else it's just unknown chip, maybe some other brand.
			break ;// Unknown chip.
	}

	// store the capacity in byte uint.
	w25q->flash_capa = 1 << tempbuf[3]; // store the flash size in byte unit.
	return 0;
}

/* w25_writeMode is to toggle write enable or write disable before and after writing to flash.
 * 1 to enable writing, 0 is vice versa.
 */
void w25_writeMode(W25QXX *w25q, uint8_t onoff){
	uint8_t CMD = onoff ? W25_CMD_WREN : W25_CMD_WRDS;

	// CE Low
	HAL_GPIO_WritePin(w25q->CSBANK, w25q->CS_PIN, GPIO_PIN_RESET);

	// send command to enable or disable flash writing.
	HAL_SPI_Transmit(w25q->SPIbus, &CMD, 1, 100);

	// CE High
	HAL_GPIO_WritePin(w25q->CSBANK, w25q->CS_PIN, GPIO_PIN_SET);
}

void w25_wait(W25QXX *w25q){
	uint8_t CMD = 0x05;
	uint8_t stat;

	// CE Low
	HAL_GPIO_WritePin(w25q->CSBANK, w25q->CS_PIN, GPIO_PIN_RESET);

	HAL_SPI_Transmit(w25q->SPIbus, &CMD, 1, 100);

	CMD = 0x00;// DUMMY BYTE

	do{
		HAL_SPI_TransmitReceive(w25q->SPIbus, &CMD, &stat, 1, 100);
	}while(stat & 0x01);


	// CE High
	HAL_GPIO_WritePin(w25q->CSBANK, w25q->CS_PIN, GPIO_PIN_SET);
}

/* w25_pageWrite is for writing data to NOR flash.
 * Select starting address (24 bit), buffer and the size no more than 256 bytes.
 */
uint8_t w25_pageWrite(W25QXX *w25q, uint32_t addr,
		uint8_t *buffer, uint16_t bufsize){
	if(bufsize > 256)// too much! one page is only 256 byte max.
		return 1;

	uint8_t CMD[4] = {
			W25_CMD_PGPR, // page write command.
			(uint8_t)(addr >> 16),// page address [23 - 16]
			(uint8_t)(addr >> 8),// page address [15 - 8]
			(uint8_t)(addr)// page address [7 - 0]
	};

	w25_wait(w25q);

	// Write enable == 1.
	w25_writeMode(w25q, 1);

	// CE Low
	HAL_GPIO_WritePin(w25q->CSBANK, w25q->CS_PIN, GPIO_PIN_RESET);

	// send page write command, page address and data.
	HAL_SPI_Transmit(w25q->SPIbus, CMD, 4, 100);
	HAL_SPI_Transmit(w25q->SPIbus, buffer, bufsize, 100);

	// CE High
	HAL_GPIO_WritePin(w25q->CSBANK, w25q->CS_PIN, GPIO_PIN_SET);

	w25_wait(w25q);

		return 0;
}

/* w25_read is for reading (sequentially) from the flash.
 * Select the address to start from, buffer to store read back and length that you want to read.
 */
uint8_t w25_read(W25QXX *w25q, uint32_t addr,
		uint8_t *buffer, uint32_t bufsize){
	if(bufsize > w25q->flash_capa)// never read beyond the boundary!
		return 1;
	if(addr > w25q->flash_capa)// never read from address beyond the boundary!
		return 1;

	uint8_t CMD[4] = {
			W25_CMD_READ, // read command.
			(uint8_t)(addr >> 16),// page address [23 - 16]
			(uint8_t)(addr >> 8),// page address [15 - 8]
			(uint8_t)(addr)// page address [7 - 0]
	};

	// CE Low
    HAL_GPIO_WritePin(w25q->CSBANK, w25q->CS_PIN, GPIO_PIN_RESET);

	// send read command, page address and wait for data
	HAL_SPI_TransmitReceive(w25q->SPIbus, CMD, CMD, 4, 100);
	HAL_SPI_Receive(w25q->SPIbus, buffer, bufsize, 100);

	// CE High
	HAL_GPIO_WritePin(w25q->CSBANK, w25q->CS_PIN, GPIO_PIN_SET);

	return 0;
}
/* Write from the beginning of the flash
 *
 */
uint8_t w25_write(W25QXX *w25q,uint8_t *data, uint32_t datsize){
	if(datsize > w25q->flash_capa)// never write beyond the flash capacity!
		return 1;

	uint16_t page_num = datsize / 256;// check for how many page-aligned data.
	uint8_t page_remain = datsize % 256;// if the data isn't page aligned, all the rest data will be flash separately.
	uint16_t last_page = 0;// store last page-aligned address for the non-aligned byte.

	for(uint16_t pages = 0; pages < page_num; pages++){
		w25_pageWrite(w25q, pages << 8, data, 256);
		data += 256;// move to next 256 bytes.
		last_page = pages;
	}

	if(page_remain)
		w25_pageWrite(w25q, (last_page+1) << 8, data, page_remain);

	return 0;
}

/* Erase data in 1 sector (4KBytes) size
 * Select the start address of that sector to erase.
 */
uint8_t w25_erase4K(W25QXX *w25q, uint32_t addr){
	if(addr > w25q->flash_capa)// never read from address beyond the boundary!
		return 1;

	uint8_t CMD[4] = {
				W25_CMD_ER_4K, // 4k sector erase command.
				(uint8_t)(addr >> 16),// page address [23 - 16]
				(uint8_t)(addr >> 8),// page address [15 - 8]
				(uint8_t)(addr)// page address [7 - 0]
		};

	w25_wait(w25q);

	// Write enable == 1.
	w25_writeMode(w25q, 1);

	// CE Low
	HAL_GPIO_WritePin(w25q->CSBANK, w25q->CS_PIN, GPIO_PIN_RESET);

	// send erase command and page address.
	HAL_SPI_Transmit(w25q->SPIbus, CMD, 4, 100);

	// CE High
	HAL_GPIO_WritePin(w25q->CSBANK, w25q->CS_PIN, GPIO_PIN_SET);

	return 0;
}

/* Erase data in 1 block (32KBytes) size
 * Select the start address of that sector to erase.
 */
uint8_t w25_erase32K(W25QXX *w25q, uint32_t addr){
	if(addr > w25q->flash_capa)// never read from address beyond the boundary!
			return 1;

	uint8_t CMD[4] = {
				W25_CMD_ER_32K, // 32K block erase command.
				(uint8_t)(addr >> 16),// page address [23 - 16]
				(uint8_t)(addr >> 8),// page address [15 - 8]
				(uint8_t)(addr)// page address [7 - 0]
		};

	w25_wait(w25q);

	// Write enable == 1.
	w25_writeMode(w25q, 1);

	// CE Low
	HAL_GPIO_WritePin(w25q->CSBANK, w25q->CS_PIN, GPIO_PIN_RESET);

	// send erase command and page address.
	HAL_SPI_Transmit(w25q->SPIbus, CMD, 4, 100);

	// CE High
	HAL_GPIO_WritePin(w25q->CSBANK, w25q->CS_PIN, GPIO_PIN_SET);

	return 0;
}

/* Erase data in 1 block (64KBytes) size
 * Select the start address of that sector to erase.
 */
uint8_t w25_erase64K(W25QXX *w25q, uint32_t addr){
	if(addr > w25q->flash_capa)// never read from address beyond the boundary!
			return 1;

	uint8_t CMD[4] = {
				W25_CMD_ER_64K, // 64K block erase command.
				(uint8_t)(addr >> 16),// page address [23 - 16]
				(uint8_t)(addr >> 8),// page address [15 - 8]
				(uint8_t)(addr)// page address [7 - 0]
		};

	w25_wait(w25q);

	// Write enable == 1.
	w25_writeMode(w25q, 1);

	// CE Low
	HAL_GPIO_WritePin(w25q->CSBANK, w25q->CS_PIN, GPIO_PIN_RESET);

	// send erase command and page address.
	HAL_SPI_Transmit(w25q->SPIbus, CMD, 4, 100);

	// CE High
	HAL_GPIO_WritePin(w25q->CSBANK, w25q->CS_PIN, GPIO_PIN_SET);

	return 0;
}

/* Erase entire chip, suitable for first use.
 */
void w25_erasewhole(W25QXX *w25q){
	uint8_t CMD = W25_CMD_CPER;

	w25_wait(w25q);

	// Write enable == 1.
	w25_writeMode(w25q, 1);

	// CE Low
	HAL_GPIO_WritePin(w25q->CSBANK, w25q->CS_PIN, GPIO_PIN_RESET);

	// send chip erase command.
	HAL_SPI_Transmit(w25q->SPIbus, &CMD, 1, 100);

	// CE High
	HAL_GPIO_WritePin(w25q->CSBANK, w25q->CS_PIN, GPIO_PIN_SET);
}

void w25_softreset(W25QXX *w25q){
	uint8_t CMD[2] = {0x66, 0x99};

	// CE Low
	HAL_GPIO_WritePin(w25q->CSBANK, w25q->CS_PIN, GPIO_PIN_RESET);

	// send chip erase command.
	HAL_SPI_Transmit(w25q->SPIbus, CMD, 1, 100);

	// CE High
	HAL_GPIO_WritePin(w25q->CSBANK, w25q->CS_PIN, GPIO_PIN_SET);

	// CE Low
	HAL_GPIO_WritePin(w25q->CSBANK, w25q->CS_PIN, GPIO_PIN_RESET);

	// send chip erase command.
	HAL_SPI_Transmit(w25q->SPIbus, CMD+1, 1, 100);

	// CE High
	HAL_GPIO_WritePin(w25q->CSBANK, w25q->CS_PIN, GPIO_PIN_SET);
}

void w25_sleep(W25QXX *w25q){
	uint8_t CMD = 0xB9;

	// CE Low
	HAL_GPIO_WritePin(w25q->CSBANK, w25q->CS_PIN, GPIO_PIN_RESET);

	// send chip erase command.
	HAL_SPI_Transmit(w25q->SPIbus, &CMD, 1, 100);

	// CE High
	HAL_GPIO_WritePin(w25q->CSBANK, w25q->CS_PIN, GPIO_PIN_SET);
}
