
/**
 * @file ma800.c
 * @author Naim MASRI (naimmas@outlook.com)
 * @brief
 * @version 0.1
 * @date 2023-11-23
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "ma800.h"


/**
 * @brief spi hattini baslatir
 *
 * @param chipselect_pin    spi cs pini
 * @param clk_src           spi clock source (spi.h bkz)
 * @param clk_prescaler     spi clock hizi (source clock hizi/prescaler)
 */
void MA800_Init(chipSelect_t *chipselect_pin, SpiClkSource_t clk_src, uint16_t clk_prescaler)
{
    initSPI(chipselect_pin, MSB_FIRST, clk_src, clk_prescaler, DATA_8_BIT);
}

/**
 * @brief ani aciyi okur
 *
 * @return double derece cinsinden
 */
double MA800_GetAngle()
{
    return (MA800_GetRawAngle() * 360.0) / 256.0;
}

/**
 * @brief ani aciyi okur
 * @note 0-256 arasi deger dondurur derece cinsine cevrilmesi lazim
 * @return uint8_t
 */
uint8_t MA800_GetRawAngle()
{
    return SPI_ReadByte(0x0);
}

/**
 * @brief sifir gecis noktasini ayarlar. bu deger bir referans olur ve aci bu degere gore hesaplanir (0.005'lik cozunurlukte)
 * @note Test edilmesi lazim spi_readbyte yerine spi_readbuffer kullanilmasi gerekebilir (16bit olmasi icin)
 * @param angle istenilen aci derece cinsinden
 * @return OperationStatus_t ayarlanan deger tekrar okunur ve yazma islemi basarili olup olmadigini bildirir
 */
OperationStatus_t MA800_SetZeroAngle(float angle)
{
    OperationStatus_t ret_val = STATUS_FAILURE;
    uint16_t reg_val = 65536 - (uint16_t)((angle / 360.0) * 65536.0);
    uint8_t temp_val = 0;
    uint8_t reg_val_8[2] = {GET_HIGH_BYTE(SET_REG(ZERO_SET_REG_LOW, GET_LOW_BYTE(reg_val))), GET_LOW_BYTE(SET_REG(ZERO_SET_REG_LOW, GET_LOW_BYTE(reg_val)))};
    SPI_WriteBuffer(reg_val_8, 2);
    _delay_ms(40);
    temp_val = SPI_ReadByte(0x0);
    ret_val |= (temp_val == GET_LOW_BYTE(reg_val));

    reg_val_8[0] = GET_HIGH_BYTE(SET_REG(ZERO_SET_REG_HIGH, GET_HIGH_BYTE(reg_val)));
    reg_val_8[1] = GET_LOW_BYTE(SET_REG(ZERO_SET_REG_HIGH, GET_HIGH_BYTE(reg_val)));
    SPI_WriteBuffer(reg_val_8, 2);
    _delay_ms(40);
    temp_val = SPI_ReadByte(0x0);
    ret_val |= (OperationStatus_t)(temp_val == GET_HIGH_BYTE(reg_val));
    return ret_val;
}

/**
 * @brief sifir gecis noktasini okur
 * @note Test edilmesi lazim spi_readbyte yerine spi_readbuffer kullanilmasi gerekebilir (16bit olmasi icin)
 * @return float
 */
double MA800_GetZeroAngle()
{
    uint8_t reg_val_8[2] = {GET_HIGH_BYTE(GET_REG(ZERO_SET_REG_LOW)), 0x0};
    SPI_WriteBuffer(reg_val_8, 2);
    _delay_us(5);
    uint8_t ret_val_8[2];
    ret_val_8[0] = SPI_ReadByte(0x00);
    reg_val_8[0] = GET_REG(ZERO_SET_REG_HIGH);
    SPI_WriteBuffer(reg_val_8, 2);
    _delay_us(5);
    ret_val_8[1] = SPI_ReadByte(0x00);
    return ((double)((uint16_t)(reg_val_8[1] << 8 | reg_val_8[0]) * 360.0 / 65536.0));
}

/**
 * @brief MGL ve MGH pinleri ve regesterlerin 1 olmasi icin esik degerini ayarlar
 * @param low_threshold MGL icin esik degeri (3bit)  (datasheet s.19)
 * @param high_threshold MGH icin esik degeri (3bit) (datasheet s.19)
 * @return OperationStatus_t ayarlanan deger tekrar okunur ve yazma islemi basarili olup olmadigini bildirir
 */
OperationStatus_t MA800_SetSwtichMagneticThreshold(uint8_t low_threshold, uint8_t high_threshold)
{
    uint8_t temp_val = 0;
    uint8_t reg_val = (low_threshold & 0xE0) | (high_threshold & 0x1C);
    uint8_t reg_val_8[2] = {GET_HIGH_BYTE(SET_REG(MAG_THRESHOLD_SET, reg_val)), GET_LOW_BYTE(SET_REG(MAG_THRESHOLD_SET, reg_val))};
    SPI_WriteBuffer(reg_val_8, 2);
    _delay_ms(40);
    temp_val = SPI_ReadByte(0x0);
    return (OperationStatus_t)(temp_val == reg_val);
}

/**
 * @brief MGL ve MGH esik degerlerini okur
 *
 * @param low_threshold_buffer   MGL esik degerinin kaydedilecegi hafiza adresi 
 * @param high_threshold_buffer  MGH esik degerinin kaydedilecegi hafiza adresi
 */
void MA800_GetSwtichMagneticThreshold(uint8_t *low_threshold_buffer, uint8_t *high_threshold_buffer)
{
    uint8_t reg_val_8[2] = {GET_HIGH_BYTE(GET_REG(MAG_THRESHOLD_SET)), 0x0};
    SPI_WriteBuffer(reg_val_8, 2);
    _delay_us(5);
    uint8_t ret_val_8;
    ret_val_8 = SPI_ReadByte(0x00);
    *low_threshold_buffer = (ret_val_8 >> 5) & 0x7;
    *high_threshold_buffer = (ret_val_8 >> 2) & 0x7;
}

/**
 * @brief manyetik alanin donus yonunu okur
 *
 * @return true saat yonu CW
 * @return false saatin ters yonu CCW
 */
bool MA800_GetRotationDirection()
{
    uint16_t reg_val_8[2] = {GET_HIGH_BYTE(GET_REG(ROT_DIR_GET)), 0x0};
    SPI_WriteBuffer(reg_val_8, 2);
    _delay_us(5);
    uint8_t ret_val_8;
    ret_val_8 = SPI_ReadByte(0x00);
    return BIT_CHK(ret_val_8, 7);
}

/**
 * @brief MGL ve MGH degerlerini okur
 * @note MGL pini ve registerinde bazi aci gecislerinde pulse olusabilir (datasheet s.19) ondan dolayi sadece
 * @return 1  MGH=1 manyetik alan MGTH ustunde
 * @return 0 MGH=MGL=0 manyetik alan MGTL ile MGTH arasinda
 * @return -1 MGL=1 manyetik alan MGTL altinda
 */
uint8_t MA00_GetMagneticSwitchState()
{
    uint8_t ret_val = 0;
    uint16_t reg_val_8[2] = {GET_HIGH_BYTE(GET_REG(ROT_DIR_GET)), 0x0};
    SPI_WriteBuffer(reg_val_8, 2);
    _delay_us(5);
    uint8_t ret_val_8;
    ret_val_8 = SPI_ReadByte(0x00);
    if (BIT_CHK(ret_val_8, 6))
    {
        _delay_us(100);
        SPI_WriteBuffer(reg_val_8, 2);
        _delay_us(5);
        uint8_t tmp_ret_val_8;
        tmp_ret_val_8 = SPI_ReadByte(0x00);
       if(BIT_CHK(ret_val_8, 6))
        ret_val = -1;
    }
    else
        ret_val = BIT_CHK(ret_val_8, 7);
    return ret_val;
}