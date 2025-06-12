/*
 * midi.c
 *
 *  Created on: Feb 14, 2025
 *      Author: Joshua
 */

#include "midi.h"


uint8_t midiTx[maxTxSize] = {0};

uint8_t txPtrIn = 0;


const uint8_t midiCable = 0;
const uint8_t midiChannel = 0;
const uint8_t NoteOn = 0b1001;
const uint8_t NoteOff = 0b1000;

const uint8_t NoteAT = 0b1010;
const uint8_t MidiCtrl = 0b1011;



void midiInit()
{

}


void midiAddMsg(uint8_t *msg, uint8_t length)
{
	uint8_t i = 0;
	while(i < length)
	{
		midiTx[txPtrIn] = *(msg + i);
		txPtrIn ++;
		i++;

		//avoid buffer overflow
		if(txPtrIn >= maxTxSize)
		{
			txPtrIn = 0;
		}
	}

}

void MidiSend()
{

	//blocking wait for usb device to become free if busy
	  while( ((USBD_HandleTypeDef*)hUsbDeviceFS.pClassData)->dev_state == USBD_HID_BUSY){}

	  USBD_HID_SendReport(&hUsbDeviceFS, &midiTx, txPtrIn);

	  //reset buffer position after sending
	  txPtrIn = 0;
}


void MidiNoteOff(uint8_t p,uint8_t v)
{

	uint8_t msg[midiNoteLength];

	msg[0] = ((midiCable << 4) | NoteOff);
	msg[1] = ((NoteOff << 4) | midiChannel);
	msg[2] = p;
	msg[3] = v;

	midiAddMsg(msg,midiNoteLength);
}
void MidiNoteOn(uint8_t p,uint8_t v)
{
	uint8_t msg[midiNoteLength];

	msg[0] = ((midiCable << 4) | NoteOn);
	msg[1] = ((NoteOn << 4) | midiChannel);
	msg[2] = p;
	msg[3] = v;

	midiAddMsg(msg,midiNoteLength);

}
void MidiNoteSus(uint8_t p,uint8_t v)
{
	uint8_t msg[midiNoteLength];

	msg[0] = ((midiCable << 4) | NoteAT);
	msg[1] = ((NoteAT << 4) | midiChannel);
	msg[2] = p;
	msg[3] = v;

	midiAddMsg(msg,midiNoteLength);

}

void MidiCC(uint8_t c, uint8_t v)
{

	uint8_t msg[midiNoteLength];

	msg[0] = ((midiCable << 4) | MidiCtrl);
	msg[1] = ((MidiCtrl << 4) | midiChannel);
	msg[2] = c;
	msg[3] = v;

	midiAddMsg(msg,midiNoteLength);


}
