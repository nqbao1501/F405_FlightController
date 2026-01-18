/*
 * CRsF.c
 *
 *  Created on: Aug 11, 2025
 *      Author: Precision
 */
#include "CRsF.h"



uint16_t CRsFChannel[RC_Channel];

uint8_t Data_Frame[MAX_FRAME_SIZE];

static CRsF_Status status = Wait_Header;
static uint8_t frame_buff[MAX_FRAME_SIZE];
static uint8_t frame_index = 0;
static uint8_t expected_length = 0;

//typedef enum {
//    GPS = 0x02,//Vị trí GPS
//    VARIO = 0x07, //Tốc độ thẳng đứng
//    BATTERY_SENSOR = 0x08, //Dữ liệu về pin
//    BARO_ALTITUDE = 0x09, //Chiều cao
//    AIR_SPEED = 0x0A, //Tốc độ không
//    HEARTBEAT = 0x0B, //Thời gian hoạt động
//    RC_CHANNELS = 0x16, //Dữ liệu từ TX
//    ATTITUDE = 0x1E, // góc ROLL, PITCH, YAW
//    FLIGHT_MODE = 0x21, //Chế độ bay
//}CRsF_Type;

//typedef enum {
//    FC_Address = 0xC8, //Địa chỉ của FC
//    TX_Address = 0xEA, //Địa chỉ của TX
//    RX_Address = 0xEE //Địa chỉ của RX
//}CRsF_Address;

//typedef struct {
//    uint8_t header;
//    uint8_t length;
//    uint8_t type;
//    uint8_t data[MAX_PAYLOAD];
//    uint8_t CRC_val;
//}__attribute__((packed)) CrsF_Frame;


bool Check_valid_Frame(CrsF_Frame *frame){
    if(frame->header != CRSF_HEADER){
//        CDC_Transmit_FS((uint8_t *)"Invalid header", 16);
        return false;
    }
    if(frame ->length <2 || frame ->length > (MAX_PAYLOAD+2)){
//        CDC_Transmit_FS((uint8_t *)"Invalid length", 14);
        return false;
    }
    uint8_t calCRC = getCRC8(&frame->type , (frame->length)-1);
    if(calCRC != frame->CRC_val){
//        CDC_Transmit_FS((uint8_t *)"Invalid CRC", 12);
        return false;
    }
    return true;
}

void CrsF_Encode(CrsF_Frame *frame, CRsF_Address address, CRsF_Type type, uint8_t *data, uint8_t length)
{
    frame->header = CRSF_HEADER;
    frame->length = length+2;
    frame->type = type;
    if(data != NULL && length > 0){
        memcpy(frame->data, data, length);
    }
    frame->CRC_val = getCRC8(&frame->type, (frame->length)-1);
}

//Xử lý payload với 16 kênh điều khiển
void Payload_CRsF(uint8_t* data){
    CRsFChannel[0] = ((data[0]    | data[1] <<8)                     & 0x07FF);
    CRsFChannel[1] = ((data[1] >>3 | data[2] <<5)                     & 0x07FF);
    CRsFChannel[2] = ((data[2] >>6 | data[3] <<2 | data[4]<<10)      & 0x07FF);
    CRsFChannel[3] = ((data[4] >>1 | data[5] <<7)                     & 0x07FF);
    CRsFChannel[4] = ((data[5] >>4 | data[6] <<4)                     & 0x07FF);
    CRsFChannel[5] = ((data[6] >>7 | data[7] <<1 | data[8]<<9)       & 0x07FF);
    CRsFChannel[6] = ((data[8] >>2 | data[9] <<6)                    & 0x07FF);
    CRsFChannel[7] = ((data[9] >>5 | data[10] <<3)                   & 0x07FF);
    CRsFChannel[8] = ((data[11]   | data[12] <<8)                    & 0x07FF);
    CRsFChannel[9] = ((data[12] >>3 | data[13] <<5)                  & 0x07FF);
    CRsFChannel[10] = ((data[13] >>6 | data[14] <<2 | data[15] <<10) & 0x07FF);
    CRsFChannel[11] = ((data[15] >>1 | data[16] <<7)                 & 0x07FF);
    CRsFChannel[12] = ((data[16] >>4 | data[17] <<4)                 & 0x07FF);
    CRsFChannel[13] = ((data[17] >>7 | data[18] <<1 | data[19] <<9)  & 0x07FF);
    CRsFChannel[14] = ((data[19] >>2 | data[20] <<6)                 & 0x07FF);
    CRsFChannel[15] = ((data[20] >>5 | data[21] <<3)                 & 0x07FF);
}

//CRC8
 uint8_t getCRC8(uint8_t *buf, uint8_t size)
{
    uint8_t crc8 = 0x00;
    for (int i = 0; i < size; i++)
    {
        crc8 ^= buf[i];

        for (int j = 0; j < 8; j++)
        {
            if (crc8 & 0x80)
            {
                crc8 <<= 1;
                crc8 ^= CRSF_CRC_POLY;
            }
            else
            {
                crc8 <<= 1;
            }
        }
    }
    return crc8;
}

//typedef enum {
//    Wait_Header,
//    Wait_Length,
//    Wait_Type,
//    Wait_PayLoad,
//    Wait_CRC,
//}CRsF_Status;



bool Check_Status(uint8_t byte , CrsF_Frame* frame){
    switch(status){
        case Wait_Header:
            if(byte == CRSF_HEADER){
                frame_buff[0] = byte;
                frame_index = 1;
                status = Wait_Length;
            }
            break;

        case Wait_Length:
            if(byte >=2 && byte <= (MAX_PAYLOAD+2)){
                frame_buff[1] = byte;
                expected_length = byte;
                frame_index = 2;
                status = Wait_Type;
            }else{
                status = Wait_Header;
            }
            break;

        case Wait_Type:
            frame_buff[2] = byte;
            frame_index = 3;
            if(expected_length ==2){
                status = Wait_CRC;
            }else{
                status = Wait_PayLoad;
            }
            break;

        case Wait_PayLoad:
            frame_buff[frame_index] = byte;
            frame_index++;
            if(frame_index >= (expected_length + 1)){
                status = Wait_CRC;
            }
            break;

        case Wait_CRC:
            frame_buff[frame_index] = byte;
            memcpy(frame, frame_buff, (frame_index+1));
            status = Wait_Header;
            return true;
            break;
    }
    return false;
}
float map_float(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
void CRsF_Process(CrsF_Frame*frame){
    if(frame == NULL){
//        CDC_Transmit_FS((uint8_t*)"Frame NULL", 10);
        return;
    }

    if(!Check_valid_Frame(frame)){
//        CDC_Transmit_FS((uint8_t*)"Frame Invalid", 12);
        return;
    }

    uint8_t total_length = frame->length + 2;
    if(total_length > MAX_FRAME_SIZE){
//        CDC_Transmit_FS((uint8_t*)"Frame too long", 14);
        return;
    }

    Data_Frame[0] = frame->header;
    Data_Frame[1] = frame->length;
    Data_Frame[2] = frame->type;
    uint8_t payload_length = frame->length - 2;
    for(int i = 0 ; i<payload_length ; i++){
        Data_Frame[i+3] = frame->data[i];
    }
    Data_Frame[3+payload_length] = frame->CRC_val;

    if(frame->type == RC_Channel && payload_length >= 22){
        Payload_CRsF(frame->data);

    }
//    snprintf(logbuf,sizeof(logbuf),
//    		"THROTTLE = %d  ROLL = %d  PITCH = %d  YAW = %d  ARM = %d \r\n",
//			CRsFChannel[CH_THROTTLE],
//			CRsFChannel[CH_ROLL],
//			CRsFChannel[CH_PITCH],
//			CRsFChannel[CH_YAW],
//			CRsFChannel[CH_ARM]);
//    CDC_Transmit_FS((uint8_t*)logbuf, strlen(logbuf));
    ScaledControllerOutput[CH_THROTTLE] = map_float(CRsFChannel[CH_THROTTLE], 174, 1811, 0, 1600);
    ScaledControllerOutput[CH_ROLL] = map_float(CRsFChannel[CH_ROLL], 174, 1811, 1000, 2000);
    ScaledControllerOutput[CH_PITCH] = map_float(CRsFChannel[CH_PITCH], 174, 1811, 1000, 2000);
    ScaledControllerOutput[CH_YAW] = map_float(CRsFChannel[CH_YAW], 174, 1811, 1000, 2000);
    ScaledControllerOutput[CH_ARM] = CRsFChannel[CH_ARM];


}





