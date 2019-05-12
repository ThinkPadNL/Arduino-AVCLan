/*
  AVCLanDrv.h - AVCLan Library for 'duino / Wiring
  Created by Kochetkov Aleksey, 04.08.2010
  Version 0.3.1
*/

#ifndef AVCLanDrv_h
#define AVCLanDrv_h

//#include "WProgram.h"
#include "config.h"
#include <stdint.h>
#define AVCLANDRV_VERSION "0.3.1"


#ifdef AVCLAN_RESISTOR
// avclan driver on resistor
#define INPUT_IS_SET   (ACSR & _BV(ACO))
#define INPUT_IS_CLEAR (!(ACSR & _BV(ACO)))
#define OUTPUT_SET_1   sbi(PORTD, DATAOUT);
#define OUTPUT_SET_0   cbi(PORTD, DATAOUT);
#define AVC_OUT_EN     sbi(PORTD, DATAOUT); sbi(DDRD, DATAOUT);  sbi(DDRD, DATAIN); sbi(ACSR, ACD); 
#define AVC_OUT_DIS    cbi(PORTD, DATAOUT); cbi(DDRD, DATAOUT);  cbi(DDRD, DATAIN); cbi(ACSR, ACD);
#else
#ifdef AVCLAN_ST485
// avclan driver on ST485
#define INPUT_IS_SET   (bit_is_clear(DATAIN_PIN, DATAIN))
#define INPUT_IS_CLEAR (bit_is_set(DATAIN_PIN, DATAIN))
#define OUTPUT_SET_1   (cbi(DATAOUT_PORT, DATAOUT));
#define OUTPUT_SET_0   (sbi(DATAOUT_PORT, DATAOUT));
#define AVC_OUT_EN     (sbi(OUTEN_PORT, OUTEN));; 
#define AVC_OUT_DIS    (cbi(OUTEN_PORT, OUTEN));;
#else
//avclan driver on PCA82C250 & LM239N
#define INPUT_IS_SET   (bit_is_set(DATAIN_PIN, DATAIN))
#define INPUT_IS_CLEAR (bit_is_clear(DATAIN_PIN, DATAIN))
#define OUTPUT_SET_1   (cbi(DATAOUT_PORT, DATAOUT));
#define OUTPUT_SET_0   (sbi(DATAOUT_PORT, DATAOUT));
#define AVC_OUT_EN     ; 
#define AVC_OUT_DIS    ;
#endif
#endif

#define AVC_NORMAL_BIT_LENGTH           	37  // 37 * (F_CPU / 1000000L / 8) 
#define AVC_BIT_1_HOLD_ON_LENGTH			20  // 20 uS * (F_CPU / 1000000L / 8) 
#define AVC_BIT_0_HOLD_ON_LENGTH			32  // 32 uS * (F_CPU / 1000000L / 8)
//#define AVC_BIT_0_HOLD_ON_MIN_LENGTH		0x34  // 26 uS * (F_CPU / 1000000L / 8)    Compare half way between a '1' (20 us) and a '0' (32 us ): 32 - (32 - 20) /2 = 26 us
#define AVC_BIT_0_HOLD_ON_MIN_LENGTH		26  // 30 uS * (F_CPU / 1000000L / 8)    Compare half way between a '1' (20 us) and a '0' (32 us ): 32 - (32 - 20) /2 = 26 us
#define AVC_START_BIT_LENGTH				186  // 186 uS  * (F_CPU / 1000000L / 8) ,  prescaler 8
#define AVC_START_BIT_HOLD_ON_LENGTH		168  // 168 uS * (F_CPU / 1000000L / 8)    prescaler 8
#define AVC_START_BIT_HOLD_ON_MIN_LENGTH	44  // 44 uS * (F_CPU / 1000000L / 8)
#define AVC_1U_LENGTH	                    1  // 1 uS * (F_CPU / 1000000L / 8)      

#define AVC_MAXMSGLEN		32
#define AVC_CONTROL_FLAGS	0xF

typedef enum
{   // No this is not a mistake, broadcast = 0!
    AVC_MSG_DIRECT    = 1,
    AVC_MSG_BROADCAST = 0
} AvcTransmissionMode;

#define ACT_NONE 0  // no action
#define EV_NONE	 0	// no event

typedef struct
{
	uint8_t	actionID;           // Action id
	uint8_t	dataSize;           // message size (uint8_ts)
	uint8_t	data[12];           // message
} AvcInMessageTable;

typedef struct
{
	uint8_t	actionID;           // Action id
	uint8_t	dataSize;           // message size (uint8_ts)
	uint8_t	data[14];           // message
	uint16_t	mask;				// mask, set bit = 1 in not checked position (1<<5 or _BV(5) - datap[5] not checked)
} AvcInMaskedMessageTable;

typedef struct
{
	AvcTransmissionMode broadcast;          // Transmission mode: normal (1) or broadcast (0).
    uint8_t                dataSize;           // message size (uint8_ts)
    uint8_t                data[14];           // message
} AvcOutMessage;

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

class AVCLanDrv{
	public:
		bool	broadcast;
		uint16_t	masterAddress;
		uint16_t	slaveAddress;
		uint16_t	deviceAddress;
		uint16_t	headAddress;
		uint8_t	dataSize;
		uint8_t	message[AVC_MAXMSGLEN];
		uint8_t	event;
		uint8_t	actionID;
		bool	readonly;
		void	begin ();
		uint8_t	readMessage (void);
		uint8_t	sendMessage (void);
		uint8_t	sendMessage (const AvcOutMessage*);
		void 	printMessage (bool incoming);
		bool	isAvcBusFree (void);
		uint8_t	getActionID (const AvcInMessageTable messageTable[], uint8_t mtSize);
		uint8_t	getActionID (const AvcInMaskedMessageTable messageTable[], uint8_t mtSize);
		void	loadMessage (const AvcOutMessage*);
	private:
		bool	_parityBit;
		uint16_t	readBits (uint8_t nbBits);
		uint8_t	_readMessage (void);
		uint8_t	_sendMessage (void);
		void	sendStartBit (void);
		void	send1BitWord (bool data);
		void	send4BitWord (uint8_t data);
		void	send8BitWord (uint8_t data);
		void	send12BitWord (uint16_t data);
		bool	readAcknowledge (void);
		bool	handleAcknowledge (void);
};

extern AVCLanDrv avclan;
#endif
