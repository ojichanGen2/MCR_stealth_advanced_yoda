/**********************************************************************/
/**
 * @file	switch_lib.cpp
 *
 * @brief	2024 RA4M1用Switchプログラム.
 *
 * @author	Suzaki.
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

/**********************************************************************/
/*
 * インクルード
 */
#include "mcr_gpt_lib.h"	// MCR用RAライブラリ
#include "switch_lib.h"		// スイッチ制御ライブラリ


/**********************************************************************/
/*
 * マクロ/定数定義
 */

/*
 *	Switchピン定義
 */
#define SWITCH_MODE_DOWN		42		// CN7  PIN9
#define SWITCH_MODE_UP			41		// CN7  PIN8
#define SWITCH_DOWN				40		// CN7  PIN7
#define SWITCH_UP				39		// CN7  PIN6
#define SWITCH_CENTER			35		// CN7  PIN2


/**********************************************************************/
/*
 * プロトタイプ宣言
 */


/**********************************************************************/
/*
 * 変数定義
 */


/**********************************************************************/
/**
 *	SWITCH 初期化.
 */
void SwitchInit(void)
{

}

/**********************************************************************/
/**
 *	スイッチ状態取得.
 *
 *	@retval		スイッチ状態
 */
uint8_t SwitchGetState(void)
{
	uint8_t sw;

	// 入力ピン設定
	pinMode(SWITCH_MODE_DOWN,	INPUT);
	pinMode(SWITCH_MODE_UP,		INPUT);
	pinMode(SWITCH_DOWN,		INPUT);
	pinMode(SWITCH_UP,			INPUT);
	pinMode(SWITCH_CENTER,		INPUT);
	delayMicroseconds(1);

	// センサ値取得
	sw =	(digitalRead(SWITCH_CENTER)		<< 4) |
			(digitalRead(SWITCH_UP)			<< 3) |
			(digitalRead(SWITCH_DOWN)		<< 2) |
			(digitalRead(SWITCH_MODE_UP)	<< 1) |
			(digitalRead(SWITCH_MODE_DOWN)	<< 0);

	// マスク処理
	sw = (~sw & 0x1f);
  delay(100);

	return sw;
}
