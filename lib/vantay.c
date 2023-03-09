/*
 * vantay.c
 *
 *  Created on: Mar 6, 2023
 *      Author: DELL
 */

// this thread has been managed by trung
#include "stm32f1xx.h"
#include "vantay.h"

UART_HandleTypeDef huart1; // you need write "extern" in this line

uint8_t FPHeader[6] = {0xEF, 0x01, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t FPGetImage[6] = {0x01, 0x00, 0x03, 0x01, 0x00, 0x05};
uint8_t FPCreateCharFile1[7] = {0x01, 0x00, 0x04, 0x02, 0x01, 0x00, 0x08};
uint8_t FPCreateCharFile2[7] = {0x01, 0x00, 0x04, 0x02, 0x02, 0x00, 0x09};
uint8_t FPCreateTemplate[6] = {0x01, 0x00, 0x03, 0x05, 0x00, 0x09};
uint8_t FPDeleteAllFinger[6] = {0x01, 0x00, 0x03, 0x0D, 0x00, 0x11};
uint8_t FPSearchFinger[11] = {0x01, 0x00, 0x08, 0x04, 0x01, 0x00, 0x00, 0x00, 0x40, 0x00, 0x4E};
uint8_t FPGetNumberOfFinger[6] = {0x01, 0x00, 0x03, 0x1D, 0x00, 0x21};
uint8_t CurrentNumberFinger;

void SendFPHeader()
{
    HAL_UART_Transmit(&huart1, FPHeader, 6, 1000);
}

void SendFPGetImage()
{
    HAL_UART_Transmit(&huart1, FPGetImage, 6, 1000);
}

void SendFPCreateCharFile1()
{
    HAL_UART_Transmit(&huart1, FPCreateCharFile1, 7, 1000);
}

void SendFPCreateCharFile2()
{
    HAL_UART_Transmit(&huart1, FPCreateCharFile2, 7, 1000);
}

void SendFPCreateTemplate()
{
    HAL_UART_Transmit(&huart1, FPCreateTemplate, 6, 1000);
}

void SendFPDeleteAllFinger()
{
    HAL_UART_Transmit(&huart1, FPDeleteAllFinger, 6, 1000);
}

void SendFPDSearchFinger()
{
    HAL_UART_Transmit(&huart1, FPSearchFinger, 11, 1000);
}

void SendFGetNumberOfFinger()
{
    HAL_UART_Transmit(&huart1, FPGetNumberOfFinger, 6, 1000);
}

void SendStoreFinger(uint16_t IDStore)
{
    uint16_t Sum = 0;
    uint8_t DataSend[9] = {0};

    DataSend[0] = 0x01;
    Sum = Sum + DataSend[0];
    DataSend[1] = 0x00;
    Sum = Sum + DataSend[1];
    DataSend[2] = 0x06;
    Sum = Sum + DataSend[2];
    DataSend[3] = 0x06;
    Sum = Sum + DataSend[3];
    DataSend[4] = 0x01;
    Sum = Sum + DataSend[4];
    DataSend[5] = (uint8_t)(IDStore >> 8);
    Sum = Sum + DataSend[5];
    DataSend[6] = (uint8_t)(IDStore & 0xFF);
    Sum = Sum + DataSend[6];
    DataSend[7] = (uint8_t)(Sum >> 8);
    DataSend[8] = (uint8_t)(Sum & 0xFF);
    HAL_UART_Transmit(&huart1, DataSend, 9, 1000);
}

void SendDeleteFinger(uint16_t IDDelete)
{
    uint16_t Sum = 0;
    uint8_t DataSend[10] = {0};

    DataSend[0] = 0x01;
    Sum = Sum + DataSend[0];
    DataSend[1] = 0x00;
    Sum = Sum + DataSend[1];
    DataSend[2] = 0x07;
    Sum = Sum + DataSend[2];
    DataSend[3] = 0x0C;
    Sum = Sum + DataSend[3];
    DataSend[4] = (uint8_t)(IDDelete >> 8);
    Sum = Sum + DataSend[4];
    DataSend[5] = (uint8_t)(IDDelete & 0xFF);
    Sum = Sum + DataSend[5];
    DataSend[6] = 0x00;
    Sum = Sum + DataSend[6];
    DataSend[7] = 0x001;
    Sum = Sum + DataSend[7];
    DataSend[8] = (uint8_t)(Sum >> 8);
    DataSend[9] = (uint8_t)(Sum & 0xFF);
    HAL_UART_Transmit(&huart1, DataSend, 10, 1000);
}

uint8_t CheckFPRespsone(uint8_t MaxRead)
{
    uint8_t ByteCount = 0;
    uint8_t FPRXData[20] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t UARTData[1] = {0};
    uint32_t TimeOut = HAL_GetTick();
    uint8_t Result;
    IDFromFinger = 0xFF;
    while ((HAL_GetTick() - TimeOut < 1000) && ByteCount < MaxRead) // time out is 1000 ms
    {
        if (HAL_UART_Receive(&huart1, (uint8_t *)UARTData, 1, 1000) == HAL_OK)
        {
            FPRXData[ByteCount] = UARTData[0];
            ByteCount++;
        }
    }

    if (ByteCount == 0)
    {
        // FPRXData[0]=0xEE;
        // FPRXData[1]=0xEE;
        // HAL_UART_Transmit(&huart2,FPRXData,2,1000);
        Result = FP_ERROR;
        return Result;
    }
    else if (ByteCount < MaxRead)
    {
        Result = FP_ERROR;
        return Result;
    }
    else // vail data return
    {

        Result = FPRXData[9];
        IDFromFinger = FPRXData[11];
        // HAL_UART_Transmit(&huart2,FPRXData,MaxRead,1000);
        return Result;
    }
}

uint8_t GetNumberOfFinger()
{
    uint8_t Result;
    SendFPHeader();
    SendFGetNumberOfFinger();
    Result = CheckFPRespsone(14);
    if (Result != FP_OK)
        return 0xFF;

    return IDFromFinger;
}

uint8_t RegistryNewFinger(uint16_t LocationID)
{

    uint8_t Result = FP_NOFINGER;
    uint32_t TimeOut = HAL_GetTick();
    HAL_UART_Transmit(&huart2, (uint8_t *)RegistryFingger, MP3COMMANDLEN, 1000);
    HAL_UART_Transmit(&huart2, (uint8_t *)PutFinger, MP3COMMANDLEN, 1000);
    while (Result == FP_NOFINGER && (HAL_GetTick() - TimeOut < 5000)) // time out is 5000 ms
    {

        SendFPHeader();
        SendFPGetImage();
        Result = CheckFPRespsone(12);
    }
    if (Result != FP_OK)
        return FP_ERROR;
    // continue if detect finger;
    SendFPHeader();
    SendFPCreateCharFile1();
    Result = CheckFPRespsone(12);
    if (Result != FP_OK)
        return FP_ERROR;

    // second get image
    HAL_UART_Transmit(&huart2, (uint8_t *)RemoveFinger, MP3COMMANDLEN, 1000);
    HAL_Delay(2000);
    Result = FP_NOFINGER;
    TimeOut = HAL_GetTick();
    HAL_UART_Transmit(&huart2, (uint8_t *)ReputFinger, MP3COMMANDLEN, 1000);

    while (Result == FP_NOFINGER && (HAL_GetTick() - TimeOut < 5000)) // time out is 5000 ms
    {

        SendFPHeader();
        SendFPGetImage();
        Result = CheckFPRespsone(12);
    }
    if (Result != FP_OK)
        return FP_ERROR;

    // continue if detect finger;
    SendFPHeader();
    SendFPCreateCharFile2();
    Result = CheckFPRespsone(12);
    if (Result != FP_OK)
        return FP_ERROR;

    // Compare finger, create template
    SendFPHeader();
    SendFPCreateTemplate();
    Result = CheckFPRespsone(12);
    if (Result == FP_FINGER_NOTMATCH)
    {

        return FP_FINGER_NOTMATCH;
    }
    else if (Result != FP_OK)
        return FP_ERROR;

    // save finger
    SendFPHeader();
    SendStoreFinger(LocationID);
    Result = CheckFPRespsone(12);
    if (Result != FP_OK)
        return FP_ERROR;
    else
    {
        return FP_OK;
    }
}

uint8_t CheckFinger()
{
    uint8_t Result = FP_NOFINGER;
    uint32_t TimeOut = HAL_GetTick();
    HAL_UART_Transmit(&huart2, (uint8_t *)CheckingFinger, MP3COMMANDLEN, 1000);

    while (Result == FP_NOFINGER && (HAL_GetTick() - TimeOut < 5000)) // time out is 5000 ms
    {

        SendFPHeader();
        SendFPGetImage();
        Result = CheckFPRespsone(12);
    }
    if (Result != FP_OK)
        return FP_ERROR;
    // continue if detect finger;
    SendFPHeader();
    SendFPCreateCharFile1();
    Result = CheckFPRespsone(12);
    if (Result != FP_OK)
        return FP_ERROR;

    // Search Fingger
    SendFPHeader();
    SendFPDSearchFinger();
    Result = CheckFPRespsone(16);
    return Result;
}

uint8_t deleteallfinger()
{ // xoa all van tay
    uint8_t Result;
    SendFPHeader();
    SendFPDeleteAllFinger();
    Result = CheckFPRespsone(12);
    if (Result != FP_OK)
        return FP_ERROR;
    else
        return Result;
}

uint8_t Delete_ID(uint16_t ID) // xoa van tay theo id
{
    uint8_t Result;
    SendFPHeader();
    SendDeleteFinger(ID);
    Result = CheckFPRespsone(16);
    return Result;
}
// 156
