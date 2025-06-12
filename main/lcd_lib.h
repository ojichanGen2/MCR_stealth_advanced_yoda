/**********************************************************************/
/**
 * @file	lcd_lib.h
 *
 * @brief	2024 RA4M1用LCDプログラム.
 *
 * @author	Interface Co.,Ltd.
 *
 * Language	C++
 *
 * --------------------------------------------------------------------
 * @note
 *
 * --------------------------------------------------------------------
 * Copyright (c) 2024 Suzaki. All Rights Reserved.
 */
/**********************************************************************/
#ifndef LCD_LIB_H__
#define LCD_LIB_H__


/**********************************************************************/
/*
 * 公開関数
 */
void LcdInit(void);
void LcdClear(void);		/* 液晶のクリア */

//void lcdShowProcess( void );
int LcdPrintf(char *format, ...);
void LcdPosition(int x, int y);

#endif /* LCD_LIB_H__ */