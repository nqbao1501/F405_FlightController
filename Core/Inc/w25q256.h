/*
 * w25q256.h
 *
 *  Created on: Oct 31, 2025
 *      Author: Admin
 */

#ifndef INC_W25Q256_H_
#define INC_W25Q256_H_

#include "stm32f4xx_hal.h"
#include <stdbool.h>

#define FLASH_CS_PIN    GPIO_PIN_3
#define FLASH_CS_PORT	GPIOB

/* active information */
#define EXT_FLASH_PAGE_SIZE		0x0100		//256b 		page size (bits)
#define EXT_FLASH_SECTOR_SIZE	0x1000		//4kB 		sector size (bytes)
#define EXT_FLASH_BLOCK_SIZE	0x00010000	//64kB 		block size (bytes)
#define EXT_FLASH_SIZE			0X00100000	//1MB-8Mb	total size (bytes)
#define EXT_FLASH_PAGE_NUM		0x1000		//4096 		pages
#define EXT_FLASH_SECTOR_NUM	0x0100		//256 		sectors
#define EXT_FLASH_BLOCK_NUM		0x0010		//16 		blocks

/*||||||||||||||| DEVICE PARAMETERS ||||||||||||||||||*/
// W25QXX EEPROM family commands

#define W25_RESET_EN		0x66	//sequence is 0x66 + 0x99 + 30us delay
#define W25_RESET			0x99 	//sequence is 0x66 + 0x99 + 30us delay
#define W25_W_ENABLE		0x06
#define W25_READ 			0x03
#define W25_FREAD 			0x0B
#define W25_FREAD_DUAL		0x3B
#define W25_FREAD_QUAD		0x6B
#define W25_PAGE_P 			0x02
#define W25_S_ERASE4K 		0x20
#define W25_B_ERASE32K		0x52
#define W25_B_ERASE64K		0xD8
#define W25_CH_ERASE		0xC7
#define W25_POWERDOWN		0xB9
#define W25_POWERUP_ID		0xAB
#define W25_JEDEC_ID		0x9F
#define W25_R_SR1			0x05
#define W25_R_SFPD_REG		0x5A

void csLow(void);
void csHigh(void);
void W25Q_Reset (void);
uint32_t W25Q_ReadID (void);
void W25Q_Read(uint32_t byteAddress, uint32_t size, uint8_t *rData);
void W25Q_WaitBusy(void);
void W25Q_SectorErase(uint32_t addr);
void W25Q_BlockErase(uint32_t addr);
void W25Q_PageProgram(uint32_t addr, volatile uint8_t *data);
uint8_t W25Q_ReadStatus(void);
#endif /* INC_W25Q256_H_ */
