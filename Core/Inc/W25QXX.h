/*
 * 25QXX.h
 *
 *  Created on: Dec 14, 2021
 *      Author: TinLethax
 */

#ifndef INC_W25QXX_H_
#define INC_W25QXX_H_

#include "stm32f1xx_hal.h"
#include <stdbool.h>

/*command list*/
// JECED id
#define W25_CMD_JDEC	0x9F // use JEDEC ID request to Probe the chip.
// read
#define W25_CMD_READ	0x03 // read command.
// write
#define W25_CMD_WREN	0x06 // Write enable.
#define W25_CMD_WRDS	0x04 // Write disable.

#define W25_CMD_PGPR	0x02 // Page program.

#define W25_CMD_ER_4K	0x20 // 4KB sector erase.
#define W25_CMD_ER_32K	0x52 // 32KB block erase.
#define W25_CMD_ER_64K	0xD8 // 64KB block erase.

#define W25_CMD_CPER	0xC7 // Chip erase.

typedef struct{
	SPI_HandleTypeDef		*SPIbus;
	GPIO_TypeDef			*CSBANK;
	uint16_t 				CS_PIN;
	uint32_t 				flash_capa;// SPI flash capacity.
}W25QXX;


uint8_t w25_init(W25QXX *w25q, SPI_HandleTypeDef *SPIbus,GPIO_TypeDef *CSBANK, uint16_t CS_PIN);
void w25_writeMode(W25QXX *w25q, uint8_t onoff);
uint8_t w25_pageWrite(W25QXX *w25q, uint32_t addr, uint8_t *buffer, uint16_t bufsize);
uint8_t w25_read(W25QXX *w25q, uint32_t addr, uint8_t *buffer, uint32_t bufsize);
uint8_t w25_write(W25QXX *w25q,uint8_t *data, uint32_t datsize);

uint8_t w25_erase4K(W25QXX *w25q, uint32_t addr);
uint8_t w25_erase32K(W25QXX *w25q, uint32_t addr);
uint8_t w25_erase64K(W25QXX *w25q, uint32_t addr);
void w25_erasewhole(W25QXX *w25q);

#endif /* INC_W25QXX_H_ */
