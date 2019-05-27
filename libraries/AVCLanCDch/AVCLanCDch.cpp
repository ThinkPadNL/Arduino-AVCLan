/*
  AVCLanDrv.cpp - AVCLan CD changer library for 'duino / Wiring
  Created by Kochetkov Aleksey, 04.08.2010
  Version 0.2.2
*/

#include "../AVCLanDrv/AVCLanDrv.h"
#include "AVCLanCDch.h"
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include "../../src/BuffSerial/BuffSerial.h"

const AvcInMessageTable  mtMain[] PROGMEM = {
	{ACT_REGISTER,       0x03, {0x12, 0x01, 0x00}},
	{ACT_PLAY_REQ1,      0x04, {0x00, 0x25, 0x43, 0x80}},
	{ACT_PLAY_IT,        0x04, {0x12, 0x01, 0x45, 0x43 }},
	{ACT_LAN_STATUS1,    0x03, {0x00, 0x01, 0x0A}},
	{ACT_LAN_STATUS2,    0x03, {0x00, 0x01, 0x08}},
	{ACT_LAN_STATUS3,    0x03, {0x00, 0x01, 0x0D}},
	{ACT_LAN_STATUS4,    0x03, {0x00, 0x01, 0x0C}},
	{ACT_LAN_STATUS5,    0x04, {0x00, 0x00, 0x01, 0x08}},
	{ACT_SCAN_ON,        0x04, {0x00, 0x25, 0x43, 0xA6}},
	{ACT_SCAN_OFF,       0x04, {0x00, 0x25, 0x43, 0xA7}},
	{ACT_SCAN_D_ON,      0x04, {0x00, 0x25, 0x43, 0xA9}},
	{ACT_SCAN_D_OFF,     0x04, {0x00, 0x25, 0x43, 0xAA}},
	{ACT_REPEAT_ON,      0x04, {0x00, 0x25, 0x43, 0xA0}},
	{ACT_REPEAT_OFF,     0x04, {0x00, 0x25, 0x43, 0xA1}},
	{ACT_REPEAT_D_ON,    0x04, {0x00, 0x25, 0x43, 0xA3}},
	{ACT_REPEAT_D_OFF,   0x04, {0x00, 0x25, 0x43, 0xA4}},
	{ACT_RANDOM_ON,      0x04, {0x00, 0x25, 0x43, 0xB0}},
	{ACT_RANDOM_OFF,     0x04, {0x00, 0x25, 0x43, 0xB1}},
	{ACT_RANDOM_D_ON,    0x04, {0x00, 0x25, 0x43, 0xB3}},
	{ACT_RANDOM_D_OFF,   0x04, {0x00, 0x25, 0x43, 0xB4}},
    {ACT_EJECT_CD,       0x04, {0x62, 0x31, 0x9F, 0x07}},
    {ACT_NOTIFY_NO_CD,   0x04, {0x62, 0x31, 0x9F, 0x00}},
	// power off 0401015F01
};
const uint8_t mtMainSize = sizeof(mtMain) / sizeof(AvcInMessageTable);

const AvcInMaskedMessageTable  mtMaskedMain[] PROGMEM = {
	{ACT_DEVSTATUS_E0, 0x04, {0x00, 0, 0x43, 0xE0}, _BV(1)},
	{ACT_DEVSTATUS_E2, 0x04, {0x00, 0, 0x43, 0xE2}, _BV(1)},
	{ACT_DEVSTATUS_E4, 0x04, {0x00, 0, 0x43, 0xE4}, _BV(1)},
	{ACT_LAN_CHECK,    0x04, {0x12, 0x01, 0x20, 0}, _BV(3)},
	{ACT_PLAY_REQ2,    0x06, {0x00, 0x12, 0x43, 0x42, 0, 0x00}, _BV(4)},
	{ACT_STOP_REQ1,     0x05, {0x00, 0x12, 0x43, 0x43, 0}, _BV(4)},
	{ACT_STOP_REQ2,     0x06, {0x00, 0x12, 0x43, 0x43, 0, 0x00}, _BV(4)},
    {ACT_VOLUME_STATUS,  0x13, {0x74, 0x31, 0xF1, 0x90, 0, 0, 0, 0, 0, 0, 0, 0x0F, 0x00, 0x00/*, 0x03, 0x08, 0x00, 0x00, 0x00*/ }, _BV(4) | _BV(5) | _BV(6) | _BV(7) | _BV(8)| _BV(9) | _BV(10)},
};
const uint8_t mtMaskedMainSize = sizeof(mtMaskedMain) / sizeof(AvcInMaskedMessageTable);

const AvcInMessageTable  mtSearchHead[] PROGMEM = {
	{ACT_REGISTER,  0x03, {0x12, 0x01, 0x00}},           // AVC LAN register
	{ACT_REGISTER,  0x03, {0x12, 0x01, 0x01}},           // AVC LAN init
	{ACT_REGISTER,  0x03, {0x01, 0x01, 0x58}},
	{ACT_REGISTER,  0x03, {0x01, 0x01, 0x5B}},
	{ACT_REGISTER,  0x04, {0x01, 0x01, 0x5F, 0x01}},
	{ACT_REGISTER,  0x04, {0x01, 0x01, 0x5F, 0x00}},
};
const uint8_t mtSearchHeadSize = sizeof(mtSearchHead) / sizeof(AvcInMessageTable);

const AvcOutMessage CmdReset         PROGMEM =  {AVC_MSG_BROADCAST,  0x05, {0x00, 0x00, 0x00, 0x00, 0x00}}; // reset AVCLan. This causes HU to send ACT_REGISTER
const AvcOutMessage CmdRegister      PROGMEM =  {AVC_MSG_DIRECT,     0x05, {0x00, 0x01, 0x12, 0x10, 0x43}}; // register CD-changer
const AvcOutMessage CmdInit1         PROGMEM =  {AVC_MSG_BROADCAST,  0x05, {0x43, 0x31, 0xF7, 0x00, 0x43}}; // init command 1
const AvcOutMessage CmdInit2         PROGMEM =  {AVC_MSG_BROADCAST,  0x0B, {0x43, 0x31, 0xF1, 0x00, 0x30, 0x01, 0x01, 0x00, 0x01, 0x00, 0x80}}; // init command 2
const AvcOutMessage CmdInit3         PROGMEM =  {AVC_MSG_BROADCAST,  0x0A, {0x43, 0x31, 0xF3, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x03}}; // init command 3
const AvcOutMessage CmdDevStatusE0   PROGMEM =  {AVC_MSG_DIRECT,     0x06, {0x00, 0x43, 0, 0xF0, 0x06, 0xA0}}; // Device status E0
const AvcOutMessage CmdDevStatusE2   PROGMEM =  {AVC_MSG_DIRECT,     0x0C, {0x00, 0x43, 0, 0xF2, 0x00, 0x30, 0x01, 0x01, 0x00, 0x01, 0x00, 0x80}}; // Device status  E2
const AvcOutMessage CmdDevStatusE4   PROGMEM =  {AVC_MSG_DIRECT,     0x0B, {0x00, 0x43, 0, 0xF4, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x03}}; // Device status  E4
const AvcOutMessage CmdPlayOk1       PROGMEM =  {AVC_MSG_DIRECT,     0x05, {0x00, 0x43, 0x12, 0x50, 0x01}}; // Play begin message 1
const AvcOutMessage CmdPlayOk2       PROGMEM =  {AVC_MSG_DIRECT,     0x05, {0x00, 0x43, 0x12, 0x52, 0x01}}; // Play begin message 2
const AvcOutMessage CmdPlayOk3       PROGMEM =  {AVC_MSG_BROADCAST,  0x0B, {0x43, 0x31, 0xF1, 0x05, 0x00, 0x01, 0xFF, 0xFF, 0xFF, 0x00, 0x80}}; // Play begin message 3, ch changer open
const AvcOutMessage CmdPlayOk4       PROGMEM =  {AVC_MSG_BROADCAST,  0x0B, {0x43, 0x31, 0xF1, 0x05, 0x28, 0x01, 0x02, 0x00, 0x25, 0x00, 0x80}}; // Play begin message 4
const AvcOutMessage CmdPlayOk5       PROGMEM =  {AVC_MSG_BROADCAST,  0x05, {0x43, 0x31, 0xF7, 0x05, 0x43}}; // Play begin message 5
const AvcOutMessage CmdStopOk1       PROGMEM =  {AVC_MSG_DIRECT,     0x05, {0x00, 0x43, 0x12, 0x53, 0x01}}; // Stop ok message 1
const AvcOutMessage CmdStopOk2       PROGMEM =  {AVC_MSG_BROADCAST,  0x0B, {0x43, 0x31, 0xF1, 0x00, 0x10, 0x01, 0x01, 0x00, 0x00, 0x00, 0x80}}; // Stop ok message 2
const AvcOutMessage CmdPlayStatus    PROGMEM =  {AVC_MSG_BROADCAST,  0x0B, {0x43, 0x31, 0xF1, 0x05, 0x10, 0x01, 0x01, 0x00, 0x00, 0x00, 0x80}}; // CD-changer play status
const AvcOutMessage CmdLanStatus1    PROGMEM =  {AVC_MSG_DIRECT,     0x04, {0x00, 0x01, 0x00, 0x1A}}; // Lan status 1
const AvcOutMessage CmdLanStatus2    PROGMEM =  {AVC_MSG_DIRECT,     0x04, {0x00, 0x01, 0x00, 0x18}}; // Lan status 2
const AvcOutMessage CmdLanStatus3    PROGMEM =  {AVC_MSG_DIRECT,     0x04, {0x00, 0x01, 0x00, 0x1D}}; // Lan status 3
const AvcOutMessage CmdLanStatus4    PROGMEM =  {AVC_MSG_DIRECT,     0x05, {0x00, 0x01, 0x00, 0x1C, 0x00}}; // Lan status 4
const AvcOutMessage CmdLanStatus5    PROGMEM =  {AVC_MSG_DIRECT,     0x04, {0x00, 0x01, 0x00, 0x18}}; // Lan status 5
const AvcOutMessage CmdLanCheckOk    PROGMEM =  {AVC_MSG_DIRECT,     0x06, {0x00, 0x01, 0x12, 0x30, 0, 0x00}}; // Check, device ok

// AVCLan CDchanger  & timer1 init, 
void AVCLanCDch::begin(){
	avclan.deviceAddress = 0x0240;
	
	// timer1 setup, prescaler factor - 1024
	TCCR1A = 0;     // normal mode
	TCCR1B = 5;     // Prescaler /1024
	TCNT1H = TI1_H; // Load counter value hi
	TCNT1L = TI1_L;	// Load counter value lo
	DISABLE_TIMER1_INT;
	AZFM_INIT;
	cd_min = cd_sec = cd_playmode = 0;
	cd_status = stWait;
}

// Use the last received message to determine the corresponding action ID, store it in avclan object
void AVCLanCDch::getActionID(){
	if (avclan.headAddress == 0){
		avclan.actionID = avclan.getActionID(mtSearchHead, mtSearchHeadSize);
	}else{
		avclan.actionID = avclan.getActionID(mtMain, mtMainSize);
		if (avclan.actionID == ACT_NONE) avclan.actionID = avclan.getActionID(mtMaskedMain, mtMaskedMainSize);
	}
}; 

// process action
void AVCLanCDch::processAction(AvcActionID ActionID){
	uint8_t r;
	switch (ActionID){
        case ACT_VOLUME_STATUS:
            bSerial.println("do ACT_VOLUME_STATUS");
            break;
        case ACT_EJECT_CD:
        case ACT_NOTIFY_NO_CD:
            bSerial.println("do ACT_EJECT_CD/ACT_NOTIFY_NO_CD -> register");
            // fallthrough
		case ACT_REGISTER:                                 // register device
            bSerial.println("do ACT_REGISTER");
			if (avclan.headAddress == 0) avclan.headAddress = avclan.masterAddress;
			avclan.sendMessage(&CmdRegister);
			break;
		case ACT_INIT:                                     // init device
            bSerial.println("do ACT_INIT");
			r = avclan.sendMessage(&CmdInit1);
			if (!r) r = avclan.sendMessage(&CmdInit2);
			if (!r) r = avclan.sendMessage(&CmdInit3);
			break;
		case ACT_DEVSTATUS_E0:                             // Device status E0
            bSerial.println("do ACT_DEVSTATUS_E0");
			r = avclan.message[1];
			avclan.loadMessage(&CmdDevStatusE0);
			avclan.message[2] = r;
			avclan.sendMessage();
			break;
		case ACT_DEVSTATUS_E2:                             // Device status E2
            bSerial.println("do ACT_DEVSTATUS_E2");
			r = avclan.message[1];
			avclan.loadMessage(&CmdDevStatusE2);
			avclan.message[2] = r;
			avclan.sendMessage();
			break;
		case ACT_DEVSTATUS_E4:                             // Device status E4
            bSerial.println("do ACT_DEVSTATUS_E4");
			r = avclan.message[1];
			avclan.loadMessage(&CmdDevStatusE4);
			avclan.message[2] = r;
			avclan.sendMessage();
			break;
		case ACT_PLAY_REQ1:                                // Play request 1
            bSerial.println("do ACT_PLAY_REQ1");
			avclan.sendMessage(&CmdPlayOk1);
			break;
		case ACT_PLAY_REQ2:                                // Play request 2
            bSerial.println("do ACT_PLAY_REQ2");
			r = avclan.sendMessage(&CmdPlayOk2);
			//if (!r) avclan.sendMessage(&CmdPlayOk3);
			break;
		case ACT_PLAY_IT:                                  // device play
            bSerial.println("do ACT_PLAY_IT");
			if (cd_status != stPlay || (cd_min == 0 && cd_sec == 0)){
				avclan.loadMessage(&CmdPlayOk4);
				avclan.message[5] = 1;       // cd disk
				avclan.message[6] = 1;       // cd track
				avclan.message[7] = cd_min;  // play tme min
				avclan.message[8] = cd_sec;  // play time sec
				r = avclan.sendMessage(false);
			}
			if (!r && cd_status != stPlay) avclan.sendMessage(&CmdPlayOk5);
			ENABLE_TIMER1_INT;
			AZFM_ON;
			cd_status = stPlay;
			break;
		case ACT_STOP_REQ1:                                // Stop request
		case ACT_STOP_REQ2:
            bSerial.println("do ACT_STOP_REQ*");
			avclan.sendMessage(&CmdStopOk1);
			DISABLE_TIMER1_INT;
			AZFM_OFF;
			cd_status = stStop;
			cd_min = cd_sec = 0;
			avclan.sendMessage(&CmdStopOk2);
			avclan.sendMessage(&CmdInit1);
			avclan.loadMessage(&CmdStopOk2);
			avclan.message[4] = 0x30;
			avclan.sendMessage();
			break;
		case ACT_LAN_STATUS1:                              // Lan status 1
            bSerial.println("do ACT_LAN_STATUS1");
			avclan.sendMessage(&CmdLanStatus1);
			DISABLE_TIMER1_INT;
			break;
		case ACT_LAN_STATUS2:                              // Lan status 2
            bSerial.println("do ACT_LAN_STATUS2");
			avclan.sendMessage(&CmdLanStatus2);
			DISABLE_TIMER1_INT;
			break;
		case ACT_LAN_STATUS3:                              // Lan status 3
            bSerial.println("do ACT_LAN_STATUS3");
			avclan.sendMessage(&CmdLanStatus3);
			break;
		case ACT_LAN_STATUS4:                              // Lan status 4
            bSerial.println("do ACT_LAN_STATUS4");
			avclan.sendMessage(&CmdLanStatus4);
			break;
		case ACT_LAN_STATUS5:                              // Lan status 5
            bSerial.println("do ACT_LAN_STATUS5");
			avclan.sendMessage(&CmdLanStatus5);
			break;
		case ACT_LAN_CHECK:                                // Lan status 5
            bSerial.println("do ACT_LAN_CHECK");
			r = avclan.message[3];
			avclan.loadMessage(&CmdLanCheckOk);
			avclan.message[4] = r;
			avclan.sendMessage(false);
			break;
		case ACT_SCAN_ON:                                  // Scan mode on
            bSerial.println("do ACT_SCAN_ON");
			cd_playmode |= pmScan;
			sendStatus();
			break;
		case ACT_SCAN_OFF:                                 // Scan mode off
            bSerial.println("do ACT_SCAN_OFF");
			cd_playmode &= ~pmScan;
			sendStatus();
			break;
		case ACT_SCAN_D_ON:                                // Scan directory mode on
            bSerial.println("do ACT_SCAN_D_ON");
			cd_playmode |= pmScanD;
			sendStatus();
			break;
		case ACT_SCAN_D_OFF:                               // Scan directory mode off
            bSerial.println("do ACT_SCAN_D_OFF");
			cd_playmode &= ~pmScanD;
			sendStatus();
			break;
		case ACT_REPEAT_ON:                                // Repeat mode on
            bSerial.println("do ACT_REPEAT_ON");
			cd_playmode |= pmRepeat;
			sendStatus();
			break;
		case ACT_REPEAT_OFF:                               // Repeat mode off
            bSerial.println("do ACT_REPEAT_OFF");
			cd_playmode &= ~pmRepeat;
			sendStatus();
			break;
		case ACT_REPEAT_D_ON:                              // Repeat directory mode on
            bSerial.println("do ACT_REPEAT_D_ON");
			cd_playmode |= pmRepeatD;
			sendStatus();
			break;
		case ACT_REPEAT_D_OFF:                             // Repeat directory mode off
            bSerial.println("do ACT_REPEAT_D_OFF");
			cd_playmode &= ~pmRepeatD;
			sendStatus();
			break;
		case ACT_RANDOM_ON:                                // Random mode on
            bSerial.println("do ACT_RANDOM_ON");
			cd_playmode |= pmRandom;
			sendStatus();
			break;
		case ACT_RANDOM_OFF:                               // Random mode off
            bSerial.println("do ACT_RANDOM_OFF");
			cd_playmode &= ~pmRandom;
			sendStatus();
			break;
		case ACT_RANDOM_D_ON:                              // Random directory mode on
            bSerial.println("do ACT_RANDOM_D_ON");
			cd_playmode |= pmRandomD;
			sendStatus();
			break;
		case ACT_RANDOM_D_OFF:                             // Random directory mode off
            bSerial.println("do ACT_RANDOM_D_OFF");
			cd_playmode &= ~pmRandomD;
			sendStatus();
			break;
	}
};

// process event
void AVCLanCDch::processEvent(AvcEventID EventID){
	switch (EventID){
		case EV_STATUS:
			sendStatus();
			avclan.event &= ~EV_STATUS;
			break;
	}
};

// send CD-changer status to head
uint8_t AVCLanCDch::sendStatus(){
	avclan.loadMessage(&CmdPlayStatus);
	avclan.message[4] = cd_status;   // cd changer status: 10-play, 80-load, 01-open, 02=err1, 03-wait
	avclan.message[5] = 1;           // cd disk
	avclan.message[6] = 1;           // cd track
	avclan.message[7] = cd_min;      // play tme min
	avclan.message[8] = cd_sec;      // play time sec
	avclan.message[9] = cd_playmode; // play mode: 0-normal, 2-disc rand, 4-rand, 8-disc repeat, 10-repeat, 20-disc scan, 40-scan
	
	return avclan.sendMessage();
}

uint8_t AVCLanCDch::hexInc(uint8_t data){
	if ((data & 0x9) == 0x9) return (data + 7);
	return (data + 1);
}

uint8_t AVCLanCDch::hexDec(uint8_t data){
	if ((data & 0xF) == 0) return (data - 7);
	return (data - 1);
}


// timer1 overflow
SIGNAL(TIMER1_OVF_vect){
	TCNT1H = TI1_H; // Load counter value hi
	TCNT1L = TI1_L;	// Load counter value lo
	avclanDevice.cd_sec = avclanDevice.hexInc(avclanDevice.cd_sec);
	if (avclanDevice.cd_sec == 0x60) {
		avclanDevice.cd_sec = 0;
		avclanDevice.cd_min = avclanDevice.hexInc(avclanDevice.cd_min);
		if (avclanDevice.cd_min == 0xA0) {
			avclanDevice.cd_min=0x0;
		}
	}
	avclan.event = EV_STATUS;
}

AVCLanCDch avclanDevice;
