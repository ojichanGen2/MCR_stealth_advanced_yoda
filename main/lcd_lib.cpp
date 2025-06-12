/**********************************************************************/
/**
 * @file	lcd_lib.cpp
 *
 * @brief	2024 RA4M1用LCDプログラム.
 *
 * @author	Suzaki.
 *
 * Language	CPP
 *
 * --------------------------------------------------------------------
 * @note
 *
 * --------------------------------------------------------------------
 * Copyright (c) 2024 Suzaki. All Rights Reserved.
 */
/**********************************************************************/

/**********************************************************************/
/*
 * インクルード
 */
#include <stdio.h>			// 標準Cﾗｲﾌﾞﾗﾘ 入出力
#include <stdarg.h>			// 標準Cﾗｲﾌﾞﾗﾘ 可変個数の実引数
#include "mcr_gpt_lib.h"	// MCR用RAライブラリ
#include "lcd_lib.h"		// 液晶関連処理


/**********************************************************************/
/*
 * マクロ/定数定義
 */

/*
 *	LCDピン定義
 *	@note sc1602bslb-xa-gb-k
 */
#define LCD_RS				38		// CN7  PIN5
#define LCD_R_W				37		// CN7  PIN4
#define LCD_E				36		// CN7  PIN3
#define LCD_DB4				42		// CN7  PIN9
#define LCD_DB5				41		// CN7  PIN8
#define LCD_DB6				40		// CN7  PIN7
#define LCD_DB7				39		// CN7  PIN6

/*
 *	液晶関連変数
 */
#define LCD_MAX_X			16		// 表示文字数 横 16
#define LCD_MAX_Y			 2		// 表示文字数 縦  2
#define DISP_BUFF_SIZE		(LCD_MAX_X * LCD_MAX_Y)


/**********************************************************************/
/*
 * プロトタイプ宣言
 */
static void out4(char code, char rs);	/* code 中の下位 4bit を LCD へ転送する rsは RS に渡すだけ */
static void out8(char code, char rs);	/* code 1Byte を LCD へ転送する rs は RS に渡すだけ */


/**********************************************************************/
/*
 * 変数定義
 */
static char	buffLcdData[DISP_BUFF_SIZE];		// 表示バッファ


/**********************************************************************/
/**
 *	4bitデータ出力.
 *
 *	@param[in]	code		出力コード
 *	@param[in]	rs			0:コマンド / 1:データ
 *
 *	@retval		パターン
 */
static void out4 (char code, char rs)
{
	// 出力ピン設定
	pinMode(LCD_DB4, OUTPUT);
	pinMode(LCD_DB5, OUTPUT);
	pinMode(LCD_DB6, OUTPUT);
	pinMode(LCD_DB7, OUTPUT);

	digitalWrite(LCD_DB4, (code >> 0) & 0x01);
	digitalWrite(LCD_DB5, (code >> 1) & 0x01);
	digitalWrite(LCD_DB6, (code >> 2) & 0x01);
	digitalWrite(LCD_DB7, (code >> 3) & 0x01);

	digitalWrite(LCD_E, LOW);
	delayMicroseconds(1);
	digitalWrite(LCD_E, HIGH);
	delayMicroseconds(1);
	digitalWrite(LCD_E, LOW);
	delayMicroseconds(100);
}

/**********************************************************************/
/**
 *	8bitデータ出力.
 *
 *	@param[in]	code		出力コード
 *	@param[in]	rs			0:コマンド / 1:データ
 */
static void out8 (char code, char rs)
{
	digitalWrite(LCD_RS, rs);
	digitalWrite(LCD_R_W, LOW);

	// ch(の下位 4bit) を出力
	out4(code >> 4, rs);
	// cl(の下位 4bit) を出力
	out4(code, rs);
}

/**********************************************************************/
/**
 *	LCD 初期化.
 */
void LcdInit(void)
{
	// 変数初期化
	memset(buffLcdData, 0x20, DISP_BUFF_SIZE);

	// 出力ピン設定
	pinMode(LCD_RS,  OUTPUT);
	pinMode(LCD_R_W, OUTPUT);
	pinMode(LCD_E, 	 OUTPUT);
	pinMode(LCD_DB4, OUTPUT);
	pinMode(LCD_DB5, OUTPUT);
	pinMode(LCD_DB6, OUTPUT);
	pinMode(LCD_DB7, OUTPUT);

	delay(50);
	digitalWrite(LCD_RS, LOW);
	digitalWrite(LCD_E, LOW);
	digitalWrite(LCD_R_W, LOW);

	delay(15);
	out4(0x03, 0);
	delay(5);
	out4(0x03, 0);
//	delay(5);
	out4(0x03, 0);
//	delay(1);
	out4(0x02, 0);

	out8(0x28, 0);
	out8(0x0C, 0);
	out8(0x06, 0);

	// 液晶クリア
	LcdClear();
}

/**********************************************************************/
/**
 *	LCDクリア.
 */
void LcdClear(void)
{
	/* 液晶クリア */
//	out8(0x01, 0);
	memset(buffLcdData, 0x20, DISP_BUFF_SIZE);
	LcdPosition(0, 0);
	LcdPrintf(buffLcdData);
	LcdPosition(0, 1);
	LcdPrintf(buffLcdData);
}

/**********************************************************************/
/**
 *	液晶出力.
 *	@note
 *		最後に出力した場所から出力
 *
 *	@param[in]	format		printfと同様
 *
 *	@retval		正常時：出力した文字列　異常時：負の数
 */
int LcdPrintf(char *format, ...)
{
	va_list argptr;
	volatile char    *p;
	volatile int     ret = 0;
	int i = LCD_MAX_X * LCD_MAX_Y + 10;

	memset(buffLcdData, 0x20, DISP_BUFF_SIZE);

	va_start(argptr, format);
	ret = vsprintf( buffLcdData, format, argptr );
	va_end(argptr);

	if (ret > 0) {
		/* vsprintfが正常なら液晶バッファへ転送 */
		p = buffLcdData;
		while (*p) {
			out8(*p, 1);
			p++;
		}
	}
	return ret;
}

/**********************************************************************/
/**
 *	カーソル移動.
 *
 *	@param[in]	x		X軸位置
 *	@param[in]	y		Y軸位置
 */
void LcdPosition(int x, int y)
{
	volatile unsigned char work = 0x80;

	/* xの計算 */
	work += x;

	/* yの計算 */
	if (y == 1) {
		work += 0x40;
	}
	else if (y == 2) {
		work += 0x14;
	}
	else if (y == 3) {
		work += 0x54;
	}

	/* カーソル移動 */
	out8(work, 0);
}
