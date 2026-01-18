/*
 * flash_interface.c
 *
 *  Created on: Nov 1, 2025
 *      Author: Admin
 */


#include "blackbox.h"
void Blackbox_EraseAll(void)
{
    uint32_t address;

    for (address = BLACKBOX_START_ADDR; address < BLACKBOX_END_ADDR; address += BLOCK_SIZE) {
        W25Q_BlockErase(address);
    }
}
