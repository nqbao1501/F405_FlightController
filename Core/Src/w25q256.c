/*
 * w25q256.c
 *
 *  Created on: Oct 31, 2025
 *      Author: Admin
 */
#include "w25q256.h"
#include "stm32f405xx.h"

extern SPI_HandleTypeDef hspi3;

void csLow(void){
	HAL_GPIO_WritePin(FLASH_CS_PORT, FLASH_CS_PIN, GPIO_PIN_RESET);
}
void csHigh(void){
	HAL_GPIO_WritePin(FLASH_CS_PORT, FLASH_CS_PIN, GPIO_PIN_SET);
}
uint8_t W25Q_ReadStatus1(void)
{
    uint8_t cmd = W25_R_SR1;
    uint8_t status = 0;

    csLow();
    HAL_SPI_Transmit(&hspi3, &cmd, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi3, &status, 1, HAL_MAX_DELAY);
    csHigh();

    return status;
}
void W25Q_Reset (void)
{
	uint8_t tData[2];

	tData[0] = W25_RESET_EN;  // enable Reset
	tData[1] = W25_RESET;  // Reset
    csLow();
    HAL_SPI_Transmit(&hspi3, tData, 2, 2000);
	csHigh(); // pull the HIGH
	HAL_Delay(100);
}

uint32_t W25Q_ReadID (void)
{
    uint8_t tx[4] = {W25_JEDEC_ID, 0xFF, 0xFF, 0xFF};
    uint8_t rx[4] = {0};

    csLow();
    HAL_SPI_TransmitReceive(&hspi3, tx, rx, sizeof(tx), HAL_MAX_DELAY);
    csHigh();

    // rx[0] = echo of 0x9F, ignore it
    uint32_t id = (rx[1] << 16) | (rx[2] << 8) | rx[3];
    return id;  // expected: 0xEF4019 for Winbond W25Q256
}
uint8_t W25Q_ReadStatus(void)
{
    uint8_t cmd = 0x05;
    uint8_t status = 0;

    csLow();
    HAL_SPI_Transmit(&hspi3, &cmd, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi3, &status, 1, HAL_MAX_DELAY);
    csHigh();

    return status;
}
void W25Q_Read(uint32_t byteAddress, uint32_t size, uint8_t *rData)
{
    uint8_t cmd[4];
    cmd[0] = W25_READ;
    cmd[1] = (byteAddress >> 16) & 0xFF;
    cmd[2] = (byteAddress >> 8) & 0xFF;
    cmd[3] = byteAddress & 0xFF;

    csLow();
    HAL_SPI_Transmit(&hspi3, cmd, sizeof(cmd), HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi3, rData, size, HAL_MAX_DELAY);
    csHigh();
}
void W25Q_EnableWrite(){
	uint8_t tData = W25_W_ENABLE;
	csLow();
	HAL_SPI_Transmit(&hspi3, &tData, 1, HAL_MAX_DELAY);
	csHigh();
}


void W25Q_WaitBusy(void)
{
    uint8_t cmd = W25_R_SR1;
    uint8_t status;

    csLow();
    HAL_SPI_Transmit(&hspi3, &cmd, 1, HAL_MAX_DELAY);
    do {
        HAL_SPI_Receive(&hspi3, &status, 1, HAL_MAX_DELAY);
        HAL_Delay(1);
    } while (status & 0x01);
    csHigh();
}

void W25Q_SectorErase(uint32_t addr)
{
    uint8_t cmd[4];
    cmd[0] = W25_S_ERASE4K;
    cmd[1] = (addr >> 16) & 0xFF;
    cmd[2] = (addr >> 8) & 0xFF;
    cmd[3] = addr & 0xFF;

    W25Q_EnableWrite();

    csLow();
    HAL_StatusTypeDef status;
    status = HAL_SPI_Transmit(&hspi3, cmd, 4, HAL_MAX_DELAY);
    csHigh();

    W25Q_WaitBusy();
}
void W25Q_BlockErase(uint32_t addr)
{
    uint8_t cmd[4];
    cmd[0] = W25_B_ERASE64K;
    cmd[1] = (addr >> 16) & 0xFF;
    cmd[2] = (addr >> 8) & 0xFF;
    cmd[3] = addr & 0xFF;
    HAL_StatusTypeDef status;

    W25Q_EnableWrite();

    csLow();
    status = HAL_SPI_Transmit(&hspi3, cmd, 4, HAL_MAX_DELAY);
    csHigh();

    W25Q_WaitBusy();
}
void W25Q_PageProgram(uint32_t addr, volatile uint8_t *data)
{
    uint8_t cmd[4];
    cmd[0] = 0x02;  // Page Program
    cmd[1] = (addr >> 16) & 0xFF;
    cmd[2] = (addr >> 8) & 0xFF;
    cmd[3] = addr & 0xFF;
    W25Q_EnableWrite();
    csLow();
    HAL_StatusTypeDef status;
    status = HAL_SPI_Transmit(&hspi3, cmd, 4, HAL_MAX_DELAY); // send command + address
    status = HAL_SPI_Transmit(&hspi3, data, 256, HAL_MAX_DELAY);
    csHigh();
    W25Q_WaitBusy();

}



