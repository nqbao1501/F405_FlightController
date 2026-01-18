/*
 * flash_interface.h
 *
 *  Created on: Nov 1, 2025
 *      Author: Admin
 */

#ifndef INC_BLACKBOX_H_
#define INC_BLACKBOX_H_

#include "stm32f405xx.h"
#include "w25q256.h"

/*Allocate 2MB to blackbox, once out, stop sending to flash*/
#define BLACKBOX_START_ADDR 0x200000  // start at 2MB offset
#define BLACKBOX_END_ADDR   0x400000  // end at 4MB offset
#define SECTOR_SIZE           4096
#define BLOCK_SIZE 			64*1024

typedef struct __attribute__((packed)){
	uint32_t timestamp;
	float battery_voltage;
	float roll_target;
	float pitch_target;
	float yaw_target;
	float roll;
	float pitch;
	float yaw;
}flash_packet;

void Blackbox_EraseAll(void);


#endif /* INC_BLACKBOX_H_ */
