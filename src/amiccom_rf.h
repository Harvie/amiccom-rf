/** @file
 * @defgroup Amiccom_RF Amiccom RF
 * Driver supporting Amiccom wireless transciever ICs
 * @copyright
 * SPDX-FileCopyrightText:  2019-2020 Tomas Mudrunka <harvie@github>
 * SPDX-License-Identifier: GPL-2.0
 * @{
 */

#pragma once

#include <amiccom_rf_hal.h>
#include <stdint.h>

/// @defgroup Amiccom_RF_CONST Datasheet constants
/// @ingroup Amiccom_RF
/// @{

//Registers
#define REG_SYSTEMCLOCK 0x00
#define REG_PLL1	0x01
#define REG_PLL2	0x02
#define REG_PLL3	0x03
#define REG_PLL4	0x04
#define REG_PLL5	0x05
#define REG_PLL6	0x06
#define REG_CRYSTAL	0x07
#define REG_PAGEA	0x08
#define REG_PAGEB	0x09
#define REG_RX1		0x0A
#define REG_RX2		0x0B
#define REG_ADC		0x0C
#define REG_PIN		0x0D
#define REG_CALIBRATION 0x0E
#define REG_MODE	0x0F

//Pages
#define ROT_PAGE_A 12
#define ROT_PAGE_B 7
#define ROT_PAGE(p) ((p) == REG_PAGEA ? ROT_PAGE_A : ROT_PAGE_B)

//Paged Registers
//PAGEA (Address: 08h), PAGEB (Address: 09h)
#define PAGEA_TX1  0x00
#define PAGEA_WOR1 0x01
#define PAGEA_WOR2 0x02
#define PAGEA_RFI  0x03
#define PAGEA_PM   0x04
#define PAGEA_RTH  0x05
#define PAGEA_AGC1 0x06
#define PAGEA_AGC2 0x07
#define PAGEA_GIO  0x08
#define PAGEA_CKO  0x09
#define PAGEA_VCB  0x0A
#define PAGEA_CHG1 0x0B
#define PAGEA_CHG2 0x0C
#define PAGEA_FIFO 0x0D
#define PAGEA_CODE 0x0E
#define PAGEA_WCAL 0x0F
#define PAGEB_TX2  0x00
#define PAGEB_IF1  0x01
#define PAGEB_IF2  0x02
#define PAGEB_ACK  0x03
#define PAGEB_ART  0x04

//Commands
#define CMD_REG_W      0x00 ///< 000x,xxxx	control register write
#define CMD_REG_R      0x80 ///< 100x,xxxx	control register read
#define CMD_ID_W       0x20 ///< 001x,xxxx	ID write
#define CMD_ID_R       0xA0 ///< 101x,xxxx	ID Read
#define CMD_FIFO_W     0x40 ///< 010x,xxxx	TX FIFO Write
#define CMD_FIFO_R     0xC0 ///< 110x,xxxx	RX FIFO Read
#define CMD_RF_RST     0xFF ///< 1111,1111	RF reset
#define CMD_TFR	       0x60 ///< 0110,xxxx	TX FIFO address pointrt reset
#define CMD_RFR	       0xE0 ///< 1110,xxxx	RX FIFO address pointer reset
#define CMD_SLEEP      0x10 ///< 0001,0000	SLEEP mode
#define CMD_IDLE       0x12 ///< 0001,0010	IDLE mode
#define CMD_STBY       0x14 ///< 0001,0100	Standby mode
#define CMD_PLL	       0x16 ///< 0001,0110	PLL mode
#define CMD_RX	       0x18 ///< 0001,1000	RX mode
#define CMD_TX	       0x1A ///< 0001,1010	TX mode
#define CMD_DEEP_SLEEP 0x1F ///< 0001,1111	Deep Sleep mode(pull-high)
#define CMD_TRI_SLEEP  0x1C ///< 0001,1100	Deep Sleep mode(tri-state)

#define AMICCOM_RF_MAX_FIFO_LENGTH 64 ///< Size of internal hardware buffer of RF IC

/// @}

#ifdef __cplusplus
extern "C" {
#endif

/// @defgroup Amiccom_RF_BIT_MAGIC Bit shuffling magic
/// @ingroup Amiccom_RF
/// @{

/**
 * Convert unsigned integers of various sizes to bytes and back
 * We use this for converting RF network and device IDs to single number and back
 */
uint8_t inline uint2bytes(uintmax_t num, uint8_t bytepos)
{
	//LITTLE Endian
	return (num >> (bytepos * 8)) & 0xFF;
}

/// @see uint2bytes()
uintmax_t inline bytes2uint(uint8_t *bytes, uint8_t len)
{
	//BIG Endian
	uintmax_t ret = 0;
	for (uint8_t i = 0; i < len; i++) { ret |= bytes[i] << ((len - 1 - i) * 8); }
	return ret;
}

/**
 * Convert byte to byte suitable for use as first byte of A7129 network ID code.
 * Quote from datasheet: "Recommend to set ID Byte 0 = 5xh or Axh"
 * This is probably for better clock/timing synchronization
 *
 * If MSB is 0, first 4 bits are set to 0x5 (0101)
 * If MSB is 1, first 4 bits are set to 0xA (1010)
 *
 * @see AmiccomRF::WriteID
 */
uint8_t inline syncbyte(uint8_t byte)
{
	if (byte >> 7) {
		return (byte & 0x0F) | 0xA0;
	} else {
		return (byte & 0x0F) | 0x50;
	}
}

/// @see syncbyte()
uint32_t inline syncbyte32(uint32_t byte)
{
	//TODO: rewrite for variable id lenght (to merge with syncbyte())
	if (byte >> 31) {
		return (byte & 0x0FFFFFFF) | 0xA0000000;
	} else {
		return (byte & 0x0FFFFFFF) | 0x50000000;
	}
}

/// @}

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

class AmiccomRF
{
	/// @defgroup Amiccom_RF_STATE State variables
	/// @ingroup Amiccom_RF
	/// @{

      public:
	int rf_chip;
	int pin_rf_sdio;
	int pin_rf_sck;
	int pin_rf_scs;
	int pin_rf_gio1; //SDO?
	int rf_freq_band;

	//unsigned int  ConfigRegisters[];        //868MHz, 50kbps (IFBW = 100KHz, Fdev = 37.5KHz), Crystal=12.8MHz
	///* const */ unsigned int  ConfigRegisters_PageA[];   //868MHz, 10kbps (IFBW = 100KHz, Fdev = 37.5KHz), Crystal=12.8MHz
	///* const */ unsigned int  ConfigRegisters_PageB[];   //868MHz, 10kbps (IFBW = 100KHz, Fdev = 37.5KHz), Crystal=12.8MHz

	uint32_t ConfigID;

	//Default 433MHz config:
	unsigned int ConfigRegisters[16] = //433MHz, 10kbps (IFBW = 100KHz, Fdev = 37.5KHz), Crystal=12.8MHz
		{
			0x0221, //SYSTEM CLOCK register,(0x1221 rate=10k)(0x0221 rate=50k) (0x0021 rate=100k)
			0x0a21, //PLL1 register,
			0xC000, //PLL2 register,    432MHz
			0x0000, //PLL3 register,
			0x1B20, //PLL4 register,
			0x0024, //PLL5 register,
			0x0000, //PLL6 register,
			0x0011, //CRYSTAL register,
			0x0000, //PAGEA,
			0x0000, //PAGEB,
			0x18D4, //RX1 register,     IFBW=100KHz   //ID tolerance, 18D4 = 1bit (default), 1854 = 0bit, 19D4 = 3bit
			0x7009, //RX2 register,     by preamble
			0xC000, //ADC register,
			0x0800, //PIN CONTROL register,     Use Strobe CMD
			0xCC45, //CALIBRATION register,     0x4C45:No CRC data filtering    0xCC45:Open CRC Data Filtering
			0x20C0	//MODE CONTROL register,    Use FIFO mode
	};

	unsigned int ConfigRegisters_PageA[16] = //433MHz, 10kbps (IFBW = 100KHz, Fdev = 37.5KHz), Crystal=12.8MHz
		{
			0xF706, //TX1 register,     Fdev = 37.5kHz
			0x0000, //WOR1 register,
			0xF800, //WOR2 register,
			0x1107, //RFI register,     Enable Tx Ramp up/down
			0x9970, //PM register,      CST=1
			0x0302, //RTH register,
			0x406F, //AGC1 register,
			0x2AC0, //AGC2 register,
			0x0001, //GIO register,     GIO2=WTR, GIO1=FSYNC
			0xD181, //CKO register
			0x0004, //VCB register,
			0x0A21, //CHG1 register,    430MHz    0x0A21
			0x0022, //CHG2 register,    435MHz    0x0022
			0x0004, //FIFO register,    FEP=4+1=5bytes
			0x152f, //CODE register,    Preamble=4bytes, ID=4bytes     0x1507:No CRC  0x150f:Open CRC    0x152f:Open data whitening   0x150f:No data whitening
			0x0000	//WCAL register,
	};

	unsigned int ConfigRegisters_PageB[16] = //868MHz, 10kbps (IFBW = 100KHz, Fdev = 37.5KHz), Crystal=12.8MHz
		{
			0x0337, //TX2 register,
			0x8400, //IF1 register,     Enable Auto-IF, IF=200KHz
			0x0000, //IF2 register,
			0x0000, //ACK register,
			0x0000	//ART register,
	};

	/// @}


	/// @defgroup Amiccom_RF_SW_SPI Software SPI primitives
	/// @ingroup Amiccom_RF
	/// @{
	void	      SendBitsRead(uint32_t src, unsigned char n, char read);
	void	      SendBits(uint32_t src, unsigned char n);
	uint32_t      ReadBits(unsigned char n);
	void	      ByteSend(unsigned char src);
	unsigned char ByteRead(void);
	/// @}

	/// @defgroup Amiccom_RF_SPI Amiccom-specific SPI primitives
	/// @ingroup Amiccom_RF
	/// @{
	void	      StrobeCMD(unsigned char cmd);
	void	      WriteReadCMD(unsigned char src);
	void	      WriteReg(unsigned char address, unsigned int dataWord);
	unsigned int  ReadReg(unsigned char address);
	void	      WritePage(unsigned char page, unsigned char address, unsigned int dataWord);
	unsigned int  ReadPage(unsigned char page, unsigned char address);
	unsigned char WriteID(void);
	unsigned char WriteID(uint32_t id);
	void	      WriteFIFO(unsigned char len, unsigned char *pdata);
	/// @}

	/// @defgroup Amiccom_RF_INIT IC initialization
	/// @ingroup Amiccom_RF
	/// @{
	AmiccomRF(int chip, int sdio, int sck, int scs, int gio1, int band);
	#ifdef MGOS_HAVE_MONGOOSE
		AmiccomRF(int band);
	#endif
	void          begin(void);
	void          begin(unsigned int freq_pll2);
	unsigned char Init(void);
	void	      InitIO(void);
	unsigned char Config(void);
	unsigned char Calibrate(void);
	/// @}

	/// @defgroup Amiccom_RF_TUNING RF Frequency tuning utilities
	/// @ingroup Amiccom_RF
	/// @{
	double	      Ch2Freq(int channel);
	uint16_t      Freq2FP(double freq);
	void          Tune2Band(int band);
	void          Tune(unsigned int freq_pll2);
	void          Tune2Freq(double freq);
	void          Tune2Ch(int channel);
	/// @}

	/// @defgroup Amiccom_RF_RXTX Wireless packet transfer
	/// @ingroup Amiccom_RF
	/// @{
	unsigned int  TxPacket(unsigned char len, unsigned char *pdata, int sync);
	void	      RxPacket(unsigned char len, unsigned char *pdata);
	unsigned int  RxPacketRequest(unsigned char len);
	unsigned int  RxPacketAvailable();
	unsigned int  RxPacketAvailableWait(unsigned long timeout);
	unsigned int  RxPacketWait(unsigned char len, unsigned char *pdata, unsigned long timeout);
	/// @}

	/// @defgroup Amiccom_RF_CW Continuous RF carrier support
	/// @ingroup Amiccom_RF
	/// @{
	void          ContinuousTX(void);
	void          TestTX(void);
	void          TestTXStop(void);
	/// @}

	/// @defgroup Amiccom_RF_PM Power Management
	/// @ingroup Amiccom_RF
	/// @{
	void          Sleep(void);
	void          Wake(void);
	/// @}

      private:
	/// How long we wait for unresponsive A7129 IC after strobe cmd before declaring failure
	const unsigned long ic_strobe_cmd_timeout_ms = 5;
};

#endif //__cplusplus

/// @}
