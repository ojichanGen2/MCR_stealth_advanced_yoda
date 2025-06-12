/**********************************************************************/
/**
 * @file	i2c_eeprom_lib.h
 *
 * @brief	2024 RA4M1用EEPROM(24C256B)I2Cプログラム.
 *
 * @author	月虹.
 *
 * Language	C
 *
 * --------------------------------------------------------------------
 * @note
 *
 * --------------------------------------------------------------------
 * Copyright (c) 2024 Suzaki. All Rights Reserved.
 */
/**********************************************************************/
#ifndef I2C_EEPROM_LIB_H__
#define I2C_EEPROM_LIB_H__


/**********************************************************************/
/*
 * 公開関数
 */
void initI2CEeprom( void );
void selectI2CEepromAddress( unsigned char address );
signed char readI2CEeprom( unsigned long address );
void writeI2CEeprom( unsigned long address, signed char write );
void setPageWriteI2CEeprom( unsigned long address, int count, signed char* data );
void I2CEepromProcess( void );
void clearI2CEeprom( void );
int checkI2CEeprom( void );

#endif // I2C_EEPROM_LIB_H__
