/*
 * CrsF.h
 *
 *  Created on: Aug 11, 2025
 *      Author: Precision
 */

#include"stdbool.h"
#include "string.h"
#include "stdint.h"
#include "stdio.h"

#ifndef INC_CRSF_H_
#define INC_CRSF_H_

#define CH_ROLL 0
#define CH_THROTTLE 1
#define CH_YAW 3
#define CH_PITCH 2
#define CH_ARM 4

#define CRSF_HEADER 0xC8
#define MAX_PAYLOAD 22
#define MAX_FRAME_SIZE 26
#define RC_Channel 0x16
#define CRSF_CRC_POLY 0xD5


typedef struct {
    uint8_t header;
    uint8_t length;
    uint8_t type;
    uint8_t data[22];
    uint8_t CRC_val;
}__attribute__((packed)) CrsF_Frame;

typedef enum {
    GPS = 0x02,//Vị trí GPS
    VARIO = 0x07, //Tốc độ thẳng đứng
    BATTERY_SENSOR = 0x08, //Dữ liệu về pin
    BARO_ALTITUDE = 0x09, //Chiều cao
    AIR_SPEED = 0x0A, //Tốc độ không
    HEARTBEAT = 0x0B, //Thời gian hoạt động
    RC_CHANNELS = 0x16, //Dữ liệu từ TX
    ATTITUDE = 0x1E, // góc ROLL, PITCH, YAW
    FLIGHT_MODE = 0x21, //Chế độ bay
}CRsF_Type;

typedef enum {
    FC_Address = 0xC8, //Địa chỉ của FC
    TX_Address = 0xEA, //Địa chỉ của TX
    RX_Address = 0xEE //Địa chỉ của RX
}CRsF_Address;

typedef enum {
    Wait_Header,
    Wait_Length,
    Wait_Type,
    Wait_PayLoad,
    Wait_CRC,
}CRsF_Status;
extern float ScaledControllerOutput[5];
float map_float(float x, float in_min, float in_max, float out_min, float out_max);
bool Check_valid_Frame(CrsF_Frame *frame);
void CrsF_Encode(CrsF_Frame *frame, CRsF_Address address, CRsF_Type type, uint8_t *data, uint8_t length);
void Payload_CRsF(uint8_t* data);
uint8_t getCRC8(uint8_t *buf, uint8_t size);
bool Check_Status(uint8_t byte , CrsF_Frame* frame);
void CRsF_Process(CrsF_Frame*frame);
void CRsF_Pack_Attitude(uint8_t* buffer, float pitch, float roll, float yaw);
void CRsF_Pack_Battery(uint8_t* buffer);
void CRsF_Pack_FlightMode(uint8_t* buffer, const char* mode);

#endif /* INC_CRSF_H_ */
