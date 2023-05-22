/*
 * vantay.h
 *
 *  Created on: Mar 6, 2023
 *      Author: DELL
 */

#ifndef VANTAY_H_
#define VANTAY_H_

#include "main.h"
#define FP_OK 0x00
#define FP_ERROR 0xFE
#define FP_NOFINGER 0x02
#define FP_FINGER_NOTMATCH 0x0A
#define FP_FINGER_NOTMATCH 0x0A
#define FP_FINGER_NOTFOUND 0x09
#define MP3COMMANDLEN 24

void SendFPHeader();
void SendFPGetImage();
void SendFPCreateCharFile1();
void SendFPCreateCharFile2();
void SendFPCreateTemplate();
void SendFPDeleteAllFinger();
void SendFPDSearchFinger();
void SendFGetNumberOfFinger();
void SendStoreFinger(uint16_t IDStore);
void SendDeleteFinger(uint16_t IDDelete);
uint8_t CheckFPRespsone(uint8_t MaxRead);
uint8_t GetNumberOfFinger();
uint8_t RegistryNewFinger(uint16_t LocationID);
//uint8_t CheckFinger();
uint8_t deleteallfinger();
uint8_t Delete_ID(uint16_t ID);

#endif /* VANTAY_H_ */
