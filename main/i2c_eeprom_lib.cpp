/**********************************************************************/
/**
 * @file	i2c_eeprom_lib.cpp
 *
 * @brief	2024 RA4M1用EEPROM(24C256B)I2Cプログラム.
 *
 * @author	月虹.
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
#include "Arduino.h"
#include "i2c_eeprom_lib.h"		// EEP-ROM関連処理


/**********************************************************************/
/*
 * マクロ/定数定義
 */

/* 
 *	setPageWriteI2CEeprom関数１回で保存出来るサイズ
 */
#define EEP_SAVE_SIZE		16			// 最大保存数

/*
 *	その他定義
 */
#define ACK					0			// リード時のACK有効(ACK=0)
#define NO_ACK				1			// リード時のACK無効(ACK=1)

#define STOP_RD				0			// ストップの直前は読み込み
#define STOP_WR				1			// ストップの直前は書き込み

#define CLOCK				6			//10


/**********************************************************************/
/*
 * プロトタイプ宣言
 */


/**********************************************************************/
/*
 * 変数定義
 */

static unsigned char eep_address;		// アドレスの選択
static int			 write_mode;		// 処理内容
static unsigned char write_eep_address;	// 書き込みEEP-ROMのアドレス
static unsigned int	 write_address;		// 書き込みアドレス
static int			 write_count;		// 書き込み個数
static signed char	 write_buff[EEP_SAVE_SIZE];	// 書き込みデータ保存バッファ
static signed char	*write_buff_p;		// 書き込みデータ読み込み位置
volatile int		j;


/**********************************************************************/
/**
 *	I2Cスタート
 *	@note
 *		スタート関連レジスタ設定、スタート信号送信
 */
static void i2c_start(void)
{
	R_PFS->PORT[4].PIN[7].PmnPFS_b.PDR = 1;		// 出力

	R_PORT4->PODR_b.PODR7 = 1;
	R_PORT4->PODR_b.PODR8 = 1;
	j = CLOCK >> 1;
	while (j--);
//	for (j = CLOCK >> 1; j > 0; j--);

	R_PORT4->PODR_b.PODR7 = 0;
	j = CLOCK >> 1;
	while (j--);
//	for (j = CLOCK >> 1; j > 0; j--);

	R_PORT4->PODR_b.PODR8 = 0;
	j = CLOCK;
	while (j--);
//	for (j = CLOCK; j > 0; j--);
}

/**********************************************************************/
/**
 *	I2Cリスタート
 *	@note
 *		リスタート信号送信
 */
static void i2c_restart(void)
{
	R_PFS->PORT[4].PIN[7].PmnPFS_b.PDR = 1;		// 出力

	R_PORT4->PODR_b.PODR7 = 1;
	R_PORT4->PODR_b.PODR8 = 1;
	j = CLOCK >> 1;
	while (j--);

	R_PORT4->PODR_b.PODR7 = 0;
	j = CLOCK >> 1;
	while (j--);

	R_PORT4->PODR_b.PODR8 = 0;
	j = CLOCK;
	while (j--);

//	int    i2c_ak = 1;

	j = CLOCK >> 1;
	while (j--);

	R_PORT4->PODR_b.PODR8 = 1;
	j = CLOCK >> 1;
	while (j--);

//	i2c_ak = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7);
	j = CLOCK >> 1;
	while (j--);

	R_PORT4->PODR_b.PODR8 = 0;
	j = CLOCK >> 1;
	while (j--);

	R_PORT4->PODR_b.PODR7 = 1;
	j = CLOCK >> 1;
	while (j--);

	R_PORT4->PODR_b.PODR8 = 1;
	j = CLOCK >> 1;
	while (j--);

	R_PORT4->PODR_b.PODR7 = 0;
	j = CLOCK >> 1;
	while (j--);

	R_PORT4->PODR_b.PODR8 = 0;
	j = CLOCK >> 1;
	while (j--);

//	return i2c_ak;
}

/**********************************************************************/
/**
 *	I2Cストップ
 *	@note
 *		ストップ信号送信
 */
void i2c_stop(int mode)
{
	R_PFS->PORT[4].PIN[7].PmnPFS_b.PDR = 1;		// 出力

	R_PORT4->PODR_b.PODR7 = 0;
	R_PORT4->PODR_b.PODR8 = 1;
	j = CLOCK >> 1;
	while (j--);

	R_PORT4->PODR_b.PODR7 = 1;
	j = CLOCK >> 1;
	while (j--);

	R_PORT4->PODR_b.PODR8 = 0;
	j = CLOCK >> 1;
	while (j--);
}

/**********************************************************************/
/**
 *	I2C 1バイト書き込み
 *
 *	@param[in]	data	書き込みデータ
 *
 *	@retval		ACK
 */
int i2c_write(signed char data)
{
	volatile int i;

	R_PFS->PORT[4].PIN[7].PmnPFS_b.PDR = 1;		// 出力

	for (i = 0; i < 8; i++)
	{
		if (data & (0x80 >> i))
		{
			R_PORT4->PODR_b.PODR7 = 1;
		}
		else
		{
			R_PORT4->PODR_b.PODR7 = 0;
		}
		R_PORT4->PODR_b.PODR8 = 1;
		j = CLOCK;
		while (j--);

		R_PORT4->PODR_b.PODR8 = 0;
		j = CLOCK;
		while (j--);
	}
}

/**********************************************************************/
/**
 *	I2C 1バイト読み込み
 *
 *	@param[in]	ack		ACK or NO_ACK
 *
 *	@retval		読み込みデータ
 */
signed char i2c_read(int ack)
{
	volatile int i;
	signed char ret;
	ret &= 0x00;
  
	// P407
	R_PFS->PORT[4].PIN[7].PmnPFS_b.PDR = 0;		// 入力

	for (i = 0; i < 8; i++)
	{
		R_PORT4->PODR_b.PODR8 = 1;
		j = CLOCK >> 1;
		while (j--);
		if (R_PORT4->PIDR_b.PIDR7)
		{
			ret |= 0x80 >> i;
		}
		j = CLOCK >> 1;
		while (j--);
		R_PORT4->PODR_b.PODR8 = 0;
		j = CLOCK;
		while (j--);
	}

	return ret;
}

/**********************************************************************/
/**
 *	EEPROM 初期化
 */
void initI2CEeprom(void)
{
	// P407
	//	pinMode(54, OUTPUT);
	R_PFS->PORT[4].PIN[7].PmnPFS_b.PDR = 1;		// 出力
	R_PFS->PORT[4].PIN[7].PmnPFS_b.PMR = 0;		// 汎用端子
	R_PFS->PORT[4].PIN[7].PmnPFS_b.PCR = 1;		// 内部プルアップ

	// P408
	//	pinMode(53, OUTPUT);
	R_PFS->PORT[4].PIN[8].PmnPFS_b.PDR = 1;		// 出力
	R_PFS->PORT[4].PIN[8].PmnPFS_b.PMR = 0;		// 汎用端子
	R_PFS->PORT[4].PIN[8].PmnPFS_b.PCR = 1;		// 内部プルアップ

	// アドレスの選択
	selectI2CEepromAddress(0);
	write_mode = 0;
}

/**********************************************************************/
/**
 *	EEPROM アドレス選択
 *
 *	@param[in]	address		アドレス0～3
 */
void selectI2CEepromAddress(unsigned char address)
{
	address &= 0x03;
	eep_address = address << 1;
}

/**********************************************************************/
/**
 *	EEPROM 読み込み
 *
 *	@param[in]	address		アドレス0x00～0x7FFF
 *
 *	@retval		読み込みデータ
 */
signed char readI2CEeprom(unsigned long address)
{
	volatile signed char ret;

	// 全体の割り込み禁止
//	noInterrupts();

	i2c_start();
	// デバイスアドレス(write)
	i2c_write(0xa0 | eep_address);
	// 1stアドレス
	i2c_write((address >> 8) & 0xff);
	// 2ndアドレス
	i2c_write(address & 0xff);
	// リスタートコンディション発行
	i2c_restart();
	// デバイスアドレス(read)
	i2c_write(0xa1 | eep_address);
	// データ読み出し
	ret = i2c_read(NO_ACK);
	i2c_stop(STOP_RD);

	// 全体の割り込み許可
//	interrupts();

	return ret;
}

/**********************************************************************/
/**
 *	EEPROM 書き込み
 *
 *	@param[in]	address		アドレス0x00～0x7FFF
 *	@param[in]	write		書き込みデータ
 */
void writeI2CEeprom(unsigned long address, signed char write)
{
	volatile int i;

	// 全体の割り込み禁止
//	noInterrupts();

	i2c_start();
	// デバイスアドレス(write)
	i2c_write(0xa0 | eep_address);
	// 1stアドレス
	i2c_write((address >> 8) & 0xff);
	// 2ndアドレス
	i2c_write(address & 0xff);
	// データ書き込み
	i2c_write(write);
		i2c_stop(STOP_WR);

	// 全体の割り込み許可
//	interrupts();
}

/**********************************************************************/
/**
 *	EEPROM ページ書き込み
 *
 *	@param[in]	address		アドレス0x00～0x7FFF
 *	@param[in]	count		書き込みデータ数
 *	@param[in]	data		書き込みデータ
 */
void setPageWriteI2CEeprom(unsigned long address, int count, signed char *data)
{
	// 書き込み中ならこの関数は中止
	if (write_mode != 0) return;

	write_mode = 1;
	write_eep_address = eep_address;
	write_address = address;
	write_count = count;
	write_buff_p = write_buff;

	if (count >= EEP_SAVE_SIZE) count = EEP_SAVE_SIZE;
	do {
		*write_buff_p++ = *data++;
	} while (--count);

	write_buff_p = write_buff;
}

/**********************************************************************/
/**
 *	EEPROM 処理
 *	@note
 *		書き込み処理を分割して行うため最低でも6回回らないと書き込めない！！
 */
void I2CEepromProcess(void)
{
	// 全体の割り込み禁止
//	noInterrupts();

	switch (write_mode)
	{
	case 1:
		i2c_start();
		write_mode = 2;
		break;
	/*
	 *	device address(write)
	 */
	case 2:
		i2c_write(0xa0 | write_eep_address);
		write_mode = 3;
		break;
	/*
	 * first address
	 */
	case 3:
		i2c_write((write_address >> 8) & 0xff);
		write_mode = 4;
		break;
	/*
	 * second address
	 */
	case 4:
		i2c_write(write_address & 0xff);
		write_mode = 5;
		break;
	/*
	 * data write
	 */
	case 5:
		i2c_write(*write_buff_p++);
		if (!(--write_count)) write_mode = 6;
		break;
	case 6:
		i2c_stop(STOP_WR);
		write_mode = 0;
		break;
	}

	// 全体の割り込み許可
//	interrupts();
}

/**********************************************************************/
/**
 *	EEPROM オールクリア
 */
void clearI2CEeprom(void)
{
	unsigned int address = 0;
	int i;

	// 全体の割り込み禁止
//	noInterrupts();

	while (address < 32768)
	{
		i2c_start();
		// device address(write)
		i2c_write(0xa0 | eep_address);
		// first address
		i2c_write(address >> 8);
		// second address
		i2c_write(address & 0xff);
		// data write
		for (i = 0; i < 64; i++)
		{
			i2c_write(0);
		}
		i2c_stop(STOP_WR);

		// wait
		while (!checkI2CEeprom());

		address += 64;
	}

	// 全体の割り込み許可
//	interrupts();
}

/**********************************************************************/
/**
 *	EEPROM アクセスチェック
 *
 *	@retval		1:アクセス可 / 2:アクセス不可
 */
int checkI2CEeprom(void)
{
	int ret;

	if (write_mode != 0) return 0;

	// 全体の割り込み禁止
//	noInterrupts();

	i2c_start();
	// device address(write)
	ret = !i2c_write(0xa0 | eep_address);
	i2c_stop(STOP_WR);

	// 全体の割り込み許可
//	interrupts();

	return ret;
}
