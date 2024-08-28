/** @file
 * @copyright
 * SPDX-FileCopyrightText:  2019-2020 Tomas Mudrunka <harvie@github>
 * SPDX-License-Identifier: GPL-2.0
 */

#include "amiccom_rf.h"
#include <math.h>

#ifdef MGOS_HAVE_MONGOOSE
#include "mgos.h"


/// Dummy mongoose OS init function. This is called by mongoose during boot. Does nothing.
extern "C" bool mgos_amiccom_rf_init(void) { return true; }


/// Constructor overloaded to get defaults from mos.yml
AmiccomRF::AmiccomRF(int band) : AmiccomRF(
	mgos_sys_config_get_amiccom_rf_chip(),
	mgos_sys_config_get_amiccom_rf_pin_sdio(),
	mgos_sys_config_get_amiccom_rf_pin_sck(),
	mgos_sys_config_get_amiccom_rf_pin_scs(),
	mgos_sys_config_get_amiccom_rf_pin_gio1(),
	band
	) {}
#endif


/// Module constructor
AmiccomRF::AmiccomRF(int chip, int sdio, int sck, int scs, int gio1, int band)
    : rf_chip(chip), pin_rf_sdio(sdio), pin_rf_sck(sck), pin_rf_scs(scs), pin_rf_gio1(gio1)
{
	//if(rf_chip != 7129) throw 7129; //Currently only amiccom A7129 chip is supported

	Tune2Band(band);
	Tune2Ch(0);

	Init();
}


/// Send a byte in SPI sequence, does not finish the transmission, end SCK = 0;
void IRAM_ATTR AmiccomRF::SendBitsRead(uint32_t src, unsigned char n, char read)
{
	SET_SCS(0);
	unsigned char i;
	uint32_t      b = (0x1 << (n - 1));
	for (i = 0; i < n; i++) {
		if (src & b) {
			SET_SDIO(1);
		} else {
			SET_SDIO(0);
		}
		/*_nop_();*/
		SET_SCK(1);
		/*_nop_();*/
		if (read && (i == (n - 1))) DIR_SDIO(DIR_IN);
		SET_SCK(0);
		src <<= 1;
	}
}


/// Send a byte in SPI sequence, does not finish the transmission, end SCK = 0;
void IRAM_ATTR AmiccomRF::SendBits(uint32_t src, unsigned char n)
{
	SendBitsRead(src, n, 0);
}


/// Read n bits from SPI and return them
uint32_t IRAM_ATTR AmiccomRF::ReadBits(unsigned char n)
{
	unsigned char i;
	uint32_t tmp = 0;
	/*_nop_();*/
	for (i = 0; i < n; i++) {
		SET_SCK(1);
		if (GET_SDIO())
			tmp = (tmp << 1) | 0x01;
		else
			tmp = tmp << 1;
		SET_SCK(0);
		/*_nop_();*/
	}
	return tmp;
}


/// Send a byte to IC in SPI sequence, does not finish the transmission, end SCK = 0;
void IRAM_ATTR AmiccomRF::ByteSend(unsigned char src)
{
	SendBits(src, 8);
}


/// Read byte. does not finish the transmission, end SCK = 0;
unsigned char IRAM_ATTR AmiccomRF::ByteRead(void)
{
	return ReadBits(8);
}


/// Just send single-byte "strobe" command, without follow-up data, first SCK = 1; SCS = 0, end SCK = 0; SCS = 1;
void IRAM_ATTR AmiccomRF::StrobeCMD(unsigned char cmd)
{
	SET_SCS(0);
	ByteSend(cmd);
	SET_SCS(1);
}


/// Send a byte write-read command in SPI sequence.
/// Start SCS = 0; does not finish the transmission, End SCK = 0;
void IRAM_ATTR AmiccomRF::WriteReadCMD(unsigned char src)
{
	SendBitsRead(src, 8, 1);
}


/// Write data word to configuration register specified by address.
/// This is how most features of RF IC get configured, adresses and values are described in datasheet.
void IRAM_ATTR AmiccomRF::WriteReg(unsigned char address, unsigned int dataWord)
{
	SET_SCS(0);
	address |= CMD_REG_W;
	ByteSend(address);
	/*_nop_();*/
	SendBits(dataWord, 16);
	SET_SCS(1);
}


/// Read data word at specified register of IC @see WriteReg()
unsigned int IRAM_ATTR AmiccomRF::ReadReg(unsigned char address)
{
	unsigned int  tmp = 0;
	address |= CMD_REG_R;
	WriteReadCMD(address);
	tmp = ReadBits(16);
	SET_SCS(1);
	DIR_SDIO(DIR_OUT);
	return tmp;
}


/// Write data to paged register.
/// Some IC registers are only available after selecting proper page by seting page register.
/// This method can conveniently access them.
/// @see ReadPage()
/// @see WriteReg()
void IRAM_ATTR AmiccomRF::WritePage(unsigned char page, unsigned char address, unsigned int dataWord)
{
	unsigned int tmp;
	tmp = address;
	tmp = ((tmp << ROT_PAGE(page)) | ConfigRegisters[REG_CRYSTAL]);
	WriteReg(REG_CRYSTAL, tmp);
	WriteReg(page, dataWord);
}


/// Reads data from paged register. @see WritePage() @see ReadReg()
unsigned int IRAM_ATTR AmiccomRF::ReadPage(unsigned char page, unsigned char address)
{
	unsigned int tmp;
	tmp = address;
	tmp = ((tmp << ROT_PAGE(page)) | ConfigRegisters[REG_CRYSTAL]);
	WriteReg(REG_CRYSTAL, tmp);
	tmp = ReadReg(page);
	return tmp;
}


unsigned char IRAM_ATTR AmiccomRF::WriteID(void)
{
	uint32_t id;
	SET_SCS(0);
	ByteSend(CMD_ID_W);
	SendBits(ConfigID, 32);
	SET_SCS(1);
	WriteReadCMD(CMD_ID_R);
	id = ReadBits(32);
	SET_SCS(1);
	DIR_SDIO(DIR_OUT);
	if (id != ConfigID) { return 1; }
	return 0;
}


/// Set 32b ID to the RF IC specified as uint32_t.
/// RF IC will only send/receive RF packets with this id in preamble.
/// @return 0 = normal, 1 = exception
unsigned char IRAM_ATTR AmiccomRF::WriteID(uint32_t id)
{
	ConfigID = syncbyte32(id);
	return WriteID();
}


/// Write to RF ICs FIFO (hardware buffer used to store packets in the RF IC)
void IRAM_ATTR AmiccomRF::WriteFIFO(unsigned char len, unsigned char *pdata)
{
	unsigned char i;
	StrobeCMD(CMD_TFR); //TX FIFO address pointer reset  0X60
	SET_SCS(0);
	ByteSend(CMD_FIFO_W); //TX FIFO write command         0X40
	for (i = 0; i < len; i++) { ByteSend(*(pdata++)); }
	SET_SCS(1);
}


/// @deprecated Probably obsolete, might use #Init() instead?
void IRAM_ATTR AmiccomRF::begin(void) //TODO: zrusit?
{
	Init();
	//InitIO(); //Uz je v Init()
	//Config(); //Uz je v Init()
	//WriteID(); //Uz je v Init()
	//Calibrate(); //Uz je v Init()
	Wake(); //Is really needed???
}


/// Init IC and tune to specified frequency in single step
/// @see Tune() @see Init()
void IRAM_ATTR AmiccomRF::begin(unsigned int freq_pll2)
{
	ConfigRegisters[REG_PLL2] = freq_pll2;
	begin();
}


/// Reconfigures the RF IC into sane initial config
unsigned char IRAM_ATTR AmiccomRF::Init(void)
{
	InitIO();
	delayms(1);		       //delay 1ms for regulator stabilized
	StrobeCMD(CMD_RF_RST);	       //reset A7129 chip
	if (Config()) { return 1; }    //config A7129 chip
	if (WriteID()) { return 2; }   //write ID code
	delayms(1);		       //delay 800us for crystal stabilized
	if (Calibrate()) { return 3; } //IF and VCO Calibration
	return 0;
}


/// Sets up GPIO for communication with RF IC
void IRAM_ATTR AmiccomRF::InitIO(void)
{
	SET_SCS(1);
	DIR_SCS(DIR_OUT);
	SET_SCK(0);
	DIR_SCK(DIR_OUT);
	SET_SDIO(1);
	DIR_SDIO(DIR_OUT);
	SET_GIO1(1);
	DIR_GIO1(DIR_IN);
	PULL_GIO1(PULL_NO);
}


/// Sync RF configuration from this object to IC
/// @return 0 = normal, 1 = exception
unsigned char IRAM_ATTR AmiccomRF::Config(void)
{
	unsigned char i;
	unsigned int  tmp;
	for (i = 0;  i < 8;  i++) { WriteReg(i, ConfigRegisters[i]); }
	for (i = 10; i < 16; i++) { WriteReg(i, ConfigRegisters[i]); }
	for (i = 0;  i < 16; i++) { WritePage(REG_PAGEA, i, ConfigRegisters_PageA[i]); }
	for (i = 0;  i < 5;  i++) { WritePage(REG_PAGEB, i, ConfigRegisters_PageB[i]); }
	tmp = ReadReg(REG_SYSTEMCLOCK);
	if (tmp != ConfigRegisters[REG_SYSTEMCLOCK]) { return 1; }
	return 0;
}


/// Performs all RF calibration routines
/// @return 0 = normal, 1 = exception
unsigned char IRAM_ATTR AmiccomRF::Calibrate(void)
{
	unsigned char fbcf; //IF Filter
	unsigned char vbcf; //VCO Current
	unsigned char vccf; //VCO Band
	unsigned int  tmp;
	WriteReg(REG_MODE, ConfigRegisters[REG_MODE] | 0x0802); /// * IF Filter & VCO Current Calibration
	do {							//VB value read here
		tmp = ReadReg(REG_MODE);
	} while (tmp & 0x0802); //TODO timeout???

	tmp  = ReadReg(REG_CALIBRATION);
	fbcf = (tmp >> 4) & 0x01;
	if (fbcf) { return 1; }
	tmp  = ReadPage(REG_PAGEA, PAGEA_VCB);
	vccf = (tmp >> 4) & 0x01;
	if (vccf) { return 1; }
	WriteReg(REG_ADC, 0x4C00);				/// * set ADC average=64
	WriteReg(REG_MODE, ConfigRegisters[REG_MODE] | 0x1000); /// * RSSI Calibration
	do {
		tmp = ReadReg(REG_MODE);
	} while (tmp & 0x1000); //TODO timeout???
	WriteReg(REG_ADC, ConfigRegisters[REG_ADC]);
	WriteReg(REG_PLL1, ConfigRegisters[REG_PLL1]);
	WriteReg(REG_PLL2, ConfigRegisters[REG_PLL2]);
	WriteReg(REG_MODE, ConfigRegisters[REG_MODE] | 0x0004); /// * VCO Band Calibration
	do {
		tmp = ReadReg(REG_MODE);
	} while (tmp & 0x0004); //TODO timeout???
	tmp  = ReadReg(REG_CALIBRATION);
	vbcf = (tmp >> 8) & 0x01;
	if (vbcf) { return 1; }
	return 0;
}


/// Convert integer channel number to floating point frequency in MHz.
/// Results vary based on RF band currently selected using ::Tune2Band()
double IRAM_ATTR AmiccomRF::Ch2Freq(int channel)
{
	// clang-format off
	const double freqlist_868[11] = {
		// 0        1        2        3        4        5        6        7        8        9        10
		   868.065, 868.112, 868.159, 868.206, 868.253, 868.488, 868.347, 868.394, 868.441, 868.300, 868.535
		// 0xD14D,  0xD23E,  0xD32E,  0xD41F,  0xD510,  0xD9C3,  0xD6F1,  0xD7E1,  0xD8D2,  0xD600,  0xDAB3
	};
	const double freqlist_433[11] = {
		// 0        1        2        3        4        5        6        7        8        9        10
		   433.150, 433.305, 433.460, 433.615, 433.770, 434.545, 434.080, 434.235, 434.390, 433.925, 434.700
		// 0xD700,  0xDA1A,  0xDD33,  0xE04D,  0xE366,  0xF2E6,  0xE99A,  0xECB3,  0xEFCD,  0xE680,  0xF600
	};
	// clang-format on

	switch (this->rf_freq_band) {
	case 868: return freqlist_868[channel]; break;
	case 433:
	default: return freqlist_433[channel]; break;
	}
}


/// Convert frequency in MHz to FP register settings
uint16_t IRAM_ATTR AmiccomRF::Freq2FP(double freq) {
	int8_t IP = ConfigRegisters[REG_PLL1] & 0xFF; //0x43
	double PFD = 12.8; //XTAL frequency MHz

	//TODO: convert this to fixed decimal point arithmetics
	//TODO: check this formula again in IC datasheet, might be off by one or something...
	//(Probably does not matter if you are using the same formula on both RX and TX side)
	return round((        (0xFFFF) * (freq-(IP*PFD))       ) /       PFD     )+1;
}


/// Tune the chip to frequency band (specified as integer)
void IRAM_ATTR AmiccomRF::Tune2Band(int band)
{
	rf_freq_band = band;

	//TODO: call this automaticaly when tuning to frequency

	/// Currently only following bands are supported:
	switch (this->rf_freq_band) {
	case 868:
		/// * 868MHz band (assuming 12.8MHz crystal)
		ConfigRegisters[REG_PLL1]	  = 0x0A43; //PLL1 register
		ConfigRegisters_PageA[PAGEA_CHG1] = 0x0343; //CHG1 register,    860MHz
		ConfigRegisters_PageA[PAGEA_CHG2] = 0x0044; //CHG2 register,    870MHz
		break;
	case 433:
	default:
		/// * 433MHz band (assuming 12.8MHz crystal)
		ConfigRegisters[REG_PLL1]	  = 0x0A21; //PLL1 register
		ConfigRegisters_PageA[PAGEA_CHG1] = 0x0A21; //CHG1 register,    430MHz
		ConfigRegisters_PageA[PAGEA_CHG2] = 0x0022; //CHG2 register,    435MHz
		break;
	}

	Config();
}


/// Tune the RF IC frequency by writing data to IC registers.
/// @see Tune2Freq() @see Tune2Ch()
void IRAM_ATTR AmiccomRF::Tune(unsigned int freq_pll2)
{
	ConfigRegisters[REG_PLL2] = freq_pll2;
	WriteReg(REG_PLL2, freq_pll2);
}


/// Tunes RF IC to frequency specified in MHz
void IRAM_ATTR AmiccomRF::Tune2Freq(double freq) { Tune(Freq2FP(freq)); }


/// Tunes RF IC to specified channel of currently selected band (resolves frequency in channel table and tunes to it)
void IRAM_ATTR AmiccomRF::Tune2Ch(int channel) { Tune2Freq(Ch2Freq(channel)); }


/**
 * TxPacket
 *
 * Sends RF packet and waits for transmit to finish
 *
 * @param len - number of bytes to send
 * @param pdata - pointer to data to send
 * @param sync - wait for TX to finish? (reccomended)
 * @return 1 if succesfull, otherwise 0
 * @see RxPacket()
 */
unsigned int IRAM_ATTR AmiccomRF::TxPacket(unsigned char len, unsigned char *pdata, int sync)
{
	DIR_GIO1(DIR_IN);
	StrobeCMD(CMD_STBY);
	WriteFIFO(len, pdata);
	StrobeCMD(CMD_TX);

	unsigned long startmillis;

	//Wait for TX to start
	startmillis = millis();
	while (GET_GIO1() == 0)
		if ((millis() - startmillis) > ic_strobe_cmd_timeout_ms) return 0;
	startmillis = millis();
	while (GET_GIO1() == 0)
		if ((millis() - startmillis) > ic_strobe_cmd_timeout_ms) return 0;

	//Wait for TX to finish (if requested)
	startmillis = millis();
	while (sync && (GET_GIO1() == 1))
		if ((millis() - startmillis) > ic_strobe_cmd_timeout_ms) return 0;

	return 1;
}


/// Receive RF packet (has to be requested and readily received in RF IC buffer first)
/// @see RxPacketWait() @see RxPacketRequest() @see RxPacketAvailable() @see TxPacket()
void IRAM_ATTR AmiccomRF::RxPacket(unsigned char len, unsigned char *pdata)
{
	unsigned char i;
	StrobeCMD(CMD_RFR); //RX FIFO address pointer reset
	WriteReadCMD(CMD_FIFO_R);
	for (i = 0; i < len; i++) { *(pdata++) = ByteRead(); }
	SET_SCS(1);
	DIR_SDIO(DIR_OUT);
}


/// Request RF IC to start receiving RF packet of specified length
unsigned int IRAM_ATTR AmiccomRF::RxPacketRequest(unsigned char len)
{
	DIR_GIO1(DIR_IN);
	StrobeCMD(CMD_STBY);
	WritePage(REG_PAGEA, 13, len - 1); //FIFO length
	StrobeCMD(CMD_RX);
	unsigned long startmillis = millis();
	while (GET_GIO1() == 0)
		if ((millis() - startmillis) > ic_strobe_cmd_timeout_ms) return 0;
	return 1;
}


/// Check if requested RF packet was already received into the hardware buffer of IC
/// @see RxPacketAvailableWait()
unsigned int IRAM_ATTR AmiccomRF::RxPacketAvailable() { return (GET_GIO1() == 0); }


/// BusyWait for RF IC to receive requested packet. Optionaly timeout after specified miliseconds.
/// @see RxPacketAvailable()
unsigned int IRAM_ATTR AmiccomRF::RxPacketAvailableWait(unsigned long timeout)
{
	unsigned long startmillis = millis();
	while (!RxPacketAvailable()) {
		if ((timeout != 0) && ((millis() - startmillis) > timeout)) return 0;
	}
	return 1;
}


/// Receive RF packet all-in-one solution. (request, busy-waits and receives)
/// @see RxPacket() @see TxPacket()
unsigned int IRAM_ATTR AmiccomRF::RxPacketWait(unsigned char len, unsigned char *pdata, unsigned long timeout)
{
	if (!RxPacketRequest(len)) return 0;
	if (!RxPacketAvailableWait(timeout)) return 0;
	RxPacket(len, pdata);
	return 1;
}


/// Prepare chip for continuous carrier transmiting (eg. for compliance testing)
/// @warning This might cause illegal jamming of RF spectrum!
/// @see TestTX() @see TestTXStop()
void IRAM_ATTR AmiccomRF::ContinuousTX(void) {
	StrobeCMD(CMD_STBY);
	WriteReg(REG_MODE, 0b0000010001111000); //Mode control register to transmit indefinetely after CMD_TX
}


/// @see ContinuousTX()
void IRAM_ATTR AmiccomRF::TestTX(void) {
	ContinuousTX();
	StrobeCMD(CMD_TX); /// Start transmitting continuously
}


/// @see ContinuousTX()
void IRAM_ATTR AmiccomRF::TestTXStop(void) {
	StrobeCMD(CMD_STBY); //Stop transmitting
	Config(); /// Revert back to original config (no continuous TX)
}


/// Put the RF IC into the low power sleep mode
void IRAM_ATTR AmiccomRF::Sleep(void)
{
	SET_SCS(1);
	SET_SCK(0);
	SET_SDIO(0);
	StrobeCMD(CMD_SLEEP); //entry sleep mode
}


/// Wake the RF IC up
void IRAM_ATTR AmiccomRF::Wake(void)
{
	StrobeCMD(CMD_STBY); //wake up
	SET_SCS(1);
	SET_SCK(0);
	SET_SDIO(1);
}
