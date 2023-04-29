/*
 * rfid.c
 *
 *  Created on: Apr 29, 2023
 *      Author: DELL
 */

#include "rfid.h"
extern SPI_HandleTypeDef hspi2;

/*
 * Ten ham:Write_MFRC5200
 * Chuc nang: Viet 1 byte du lieu vao thanh ghi MFRC522
 * Input:addr-> DIa chi ghi, val-> Gia tri de ghi
 * Tra ve: Khong
 */
uint8_t SPI_Transfer(uint8_t data)
{
    uint8_t TxBuffer = data;
    uint8_t RxBuffer;
    HAL_SPI_Transmit(&hspi2, &TxBuffer, 1, 1000);

    HAL_SPI_Receive(&hspi2, &RxBuffer, 1, 1000);
    return RxBuffer;
}

uint8_t RC522_SPI_Transfer(uint8_t data)
{
    return SPI_Transfer(data);
}

/*
 * Ten ham:Write_MFRC5200
 * Chuc nang: Viet 1 byte du lieu vao thanh ghi MFRC522
 * Input:addr-> DIa chi ghi, val-> Gia tri de ghi
 * Tra ve: Khong
 */
void Write_MFRC522(uchar addr, uchar val)
{
    /* CS LOW */
    HAL_GPIO_WritePin(MFRC522_CS_GPIO_Port, MFRC522_CS_Pin, 0);

    // Dinh dang dia chi:0XXXXXX0
    RC522_SPI_Transfer((addr << 1) & 0x7E);
    RC522_SPI_Transfer(val);

    /* CS HIGH */
    HAL_GPIO_WritePin(MFRC522_CS_GPIO_Port, MFRC522_CS_Pin, 1);
}

/*
 * Ten ham:Read_MFRC522
 * Chuc nang:Doc 1 byte du lieu tu 1 thanh ghi MFRC522
 * Input:addr-> dia chi doc
 * Tra ve: Gia tri trong thanh ghi doc ve
 */
uchar Read_MFRC522(uchar addr)
{
    uchar val;

    /* CS LOW */
    HAL_GPIO_WritePin(MFRC522_CS_GPIO_Port, MFRC522_CS_Pin, 0);

    // Dinh dang dia chi:1XXXXXX0
    RC522_SPI_Transfer(((addr << 1) & 0x7E) | 0x80);
    val = RC522_SPI_Transfer(0x00);

    /* CS HIGH */
    HAL_GPIO_WritePin(MFRC522_CS_GPIO_Port, MFRC522_CS_Pin, 1);

    return val;
}

/*
 * Ten ham:SetBitMask
 * Chuc nang:Set bit trong mot thanh ghi MFRC522
 * Input:reg--Thanh ghi cai dat; mask--gia tri set
 * Tra ve: Khong
 */
void SetBitMask(uchar reg, uchar mask)
{
    uchar tmp;
    tmp = Read_MFRC522(reg);
    Write_MFRC522(reg, tmp | mask); // set bit mask
}

/*
 * Ten ham:ClearBitMask
 * Chuc nang:Reset bit trong thanh ghi MFRC522
 * Input:reg--Dia chi thanh ghi; mask--Gia tri bit can clear
 * Tra ve: Khong
 */
void ClearBitMask(uchar reg, uchar mask)
{
    uchar tmp;
    tmp = Read_MFRC522(reg);
    Write_MFRC522(reg, tmp & (~mask)); // clear bit mask
}

/*
 * Ten Ham:AntennaOn
 * Chuc Nang:Mo anten, nen co it nhat 1 ms
 * Input: khong
 * Tra ve: khong
 */
void AntennaOn(void)
{
    //  uchar temp;

    //  temp = Read_MFRC522(TxControlReg);
    //  if (!(temp & 0x03))
    //  {
    //      SetBitMask(TxControlReg, 0x03);
    //  }
    SetBitMask(TxControlReg, 0x03);
}

/*
 * Ten ham:AntennaOff
 * chuc nang:Dong Anten, nen co it nhat 1 ms
 * Input:khong
 * Tra ve: khong
 */
void AntennaOff(void)
{
    ClearBitMask(TxControlReg, 0x03);
}

/*
 * Ten ham:ResetMFRC522
 * Chuc nang:Khoi dong lai RC522
 * Input: Khong
 * Return: Khong
 */
void MFRC522_Reset(void)
{
    Write_MFRC522(CommandReg, PCD_RESETPHASE);
}

/*
 * Ten ham:InitMFRC522
 * Chuc nang:Khoi tao RC522
 * Input: Khong
 * Tra va: Khong
 */
void MFRC522_Init(void)
{

    HAL_GPIO_WritePin(MFRC522_CS_GPIO_Port, MFRC522_CS_Pin, 1);   // Activate the RFID reader
    HAL_GPIO_WritePin(MFRC522_RST_GPIO_Port, MFRC522_RST_Pin, 1); // not reset

    MFRC522_Reset();

    // Timer: TPrescaler*TreloadVal/6.78MHz = 24ms
    Write_MFRC522(TModeReg, 0x8D);      // Tauto=1; f(Timer) = 6.78MHz/TPreScaler
    Write_MFRC522(TPrescalerReg, 0x3E); // TModeReg[3..0] + TPrescalerReg
    Write_MFRC522(TReloadRegL, 30);
    Write_MFRC522(TReloadRegH, 0);

    Write_MFRC522(TxAutoReg, 0x40); // 100%ASK
    Write_MFRC522(ModeReg, 0x3D);   // CRC Gia tri ban dau 0x6363    ???

    // ClearBitMask(Status2Reg, 0x08);       //MFCrypto1On=0
    // Write_MFRC522(RxSelReg, 0x86);        //RxWait = RxSelReg[5..0]
    // Write_MFRC522(RFCfgReg, 0x7F);        //RxGain = 48dB

    AntennaOn(); // Mo Anten
}

/*
 * Ten ham:MFRC522_ToCard
 * Chuc nang:truyen thong giua RC522 va the ISO14443
 * Input:command--lenh gui den MF522,
 *           sendData--Du lieu gui den the bang MFRC522,
 *           sendLen--Chieu dai du lieu gui
 *           backData--Du lieu nhan duoc tro lai
 *           backLen--Tra ve do dai bit cua du lieu
 * Tra ve: MI_OK neu thanh cong
 */
uchar MFRC522_ToCard(uchar command, uchar *sendData, uchar sendLen, uchar *backData, unsigned int *backLen)
{
    uchar status = MI_ERR;
    uchar irqEn = 0x00;
    uchar waitIRq = 0x00;
    uchar lastBits;
    uchar n;
    unsigned int i;

    switch (command)
    {
    case PCD_AUTHENT: // Xac nhan the gan
    {
        irqEn = 0x12;
        waitIRq = 0x10;
        break;
    }
    case PCD_TRANSCEIVE: // Gui du lieu FIFO
    {
        irqEn = 0x77;
        waitIRq = 0x30;
        break;
    }
    default:
        break;
    }

    Write_MFRC522(CommIEnReg, irqEn | 0x80); // Yeu cau ngat
    ClearBitMask(CommIrqReg, 0x80);          // Clear tat ca cac bit yeu cau ngat
    SetBitMask(FIFOLevelReg, 0x80);          // FlushBuffer=1, Khoi tao FIFO

    Write_MFRC522(CommandReg, PCD_IDLE); // NO action; Huy bo lenh hien hanh  ???

    // Ghi du lieu vao FIFO
    for (i = 0; i < sendLen; i++)
    {
        Write_MFRC522(FIFODataReg, sendData[i]);
    }

    // chay
    Write_MFRC522(CommandReg, command);
    if (command == PCD_TRANSCEIVE)
    {
        SetBitMask(BitFramingReg, 0x80); // StartSend=1,transmission of data starts
    }

    // Cho doi de nhan duoc du lieu day du
    i = 2000; // i tuy thuoc tan so thach anh, thoi gian toi da cho the M1 la 25ms
    do
    {
        // CommIrqReg[7..0]
        // Set1 TxIRq RxIRq IdleIRq HiAlerIRq LoAlertIRq ErrIRq TimerIRq
        n = Read_MFRC522(CommIrqReg);
        i--;
    } while ((i != 0) && !(n & 0x01) && !(n & waitIRq));

    ClearBitMask(BitFramingReg, 0x80); // StartSend=0

    if (i != 0)
    {
        if (!(Read_MFRC522(ErrorReg) & 0x1B)) // BufferOvfl Collerr CRCErr ProtecolErr
        {
            status = MI_OK;
            if (n & irqEn & 0x01)
            {
                status = MI_NOTAGERR; //??
            }

            if (command == PCD_TRANSCEIVE)
            {
                n = Read_MFRC522(FIFOLevelReg);
                lastBits = Read_MFRC522(ControlReg) & 0x07;
                if (lastBits)
                {
                    *backLen = (n - 1) * 8 + lastBits;
                }
                else
                {
                    *backLen = n * 8;
                }

                if (n == 0)
                {
                    n = 1;
                }
                if (n > MAX_LEN)
                {
                    n = MAX_LEN;
                }

                // Doc FIFO trong cac du lieu nhan duoc
                for (i = 0; i < n; i++)
                {
                    backData[i] = Read_MFRC522(FIFODataReg);
                }
            }
        }
        else
        {
            status = MI_ERR;
        }
    }

    // SetBitMask(ControlReg,0x80);           //timer stops
    // Write_MFRC522(CommandReg, PCD_IDLE);

    return status;
}

/*
 * Ten ham:MFRC522_Request
 * Chuc nang:Phat hien the, doc loai the
 * Input:reqMode--Phat hien co the,
 *           TagType--Loai the tra ve
 *              0x4400 = Mifare_UltraLight
 *              0x0400 = Mifare_One(S50)
 *              0x0200 = Mifare_One(S70)
 *              0x0800 = Mifare_Pro(X)
 *              0x4403 = Mifare_DESFire
 * Return: MI_OK neu thanh cong
 */
uchar MFRC522_Request(uchar reqMode, uchar *TagType)
{
    uchar status;
    unsigned int backBits; // cac bit du lieu nhan duoc

    Write_MFRC522(BitFramingReg, 0x07); // TxLastBists = BitFramingReg[2..0] ???

    TagType[0] = reqMode;
    status = MFRC522_ToCard(PCD_TRANSCEIVE, TagType, 1, TagType, &backBits);

    if ((status != MI_OK) || (backBits != 0x10))
    {
        status = MI_ERR;
    }

    return status;
}

/*
 * Ten ham:MFRC522_Anticoll
 * Chuc nang:Phat hien chong va cham, chon the va doc so serial the
 * Input:serNum--Tra ve serial the 4 byte, byte 5 la ma checksum
 * Tra ve: MI_OK neu thanh cong
 */
uchar MFRC522_Anticoll(uchar *serNum)
{
    uchar status;
    uchar i;
    uchar serNumCheck = 0;
    unsigned int unLen;

    // ClearBitMask(Status2Reg, 0x08);       //TempSensclear
    // ClearBitMask(CollReg,0x80);           //ValuesAfterColl
    Write_MFRC522(BitFramingReg, 0x00); // TxLastBists = BitFramingReg[2..0]

    serNum[0] = PICC_ANTICOLL;
    serNum[1] = 0x20;
    status = MFRC522_ToCard(PCD_TRANSCEIVE, serNum, 2, serNum, &unLen);

    if (status == MI_OK)
    {
        // Kiem tra so serial the
        for (i = 0; i < 4; i++)
        {
            serNumCheck ^= serNum[i];
        }
        if (serNumCheck != serNum[i])
        {
            status = MI_ERR;
        }
    }

    // SetBitMask(CollReg, 0x80);        //ValuesAfterColl=1

    return status;
}
