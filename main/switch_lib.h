/**********************************************************************/
/**
 * @file	switch_lib.h
 *
 * @brief	2024 RA4M1用Switchプログラム.
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
#ifndef SWITCH_LIB_H__
#define SWITCH_LIB_H__


/**********************************************************************/
/*
 * マクロ/定数定義
 */

/*
 *	各スイッチビット情報
 */
#define	MENU_DOWN	(0x01 << 0)			// bit0
#define	MENU_UP		(0x01 << 1)			// bit1
#define	DATA_DOWN	(0x01 << 2)			// bit2
#define	DATA_UP		(0x01 << 3)			// bit3
#define	SET			(0x01 << 4)			// bit4

/**********************************************************************/
/*
 * 公開関数
 */
void SwitchInit(void);
uint8_t SwitchGetState(void);

#endif /* SWITCH_LIB_H__ */
