/*
 * midi.h
 *
 *  Created on: Feb 14, 2025
 *      Author: Joshua
 */

#ifndef INC_MIDI_H_
#define INC_MIDI_H_


#include "stm32f4xx_hal.h"
#include "usbd_hid.h"


extern USBD_HandleTypeDef hUsbDeviceFS;

#define maxMsg 16
#define midiNoteLength 4
#define maxTxSize (maxMsg * midiNoteLength)

extern const uint8_t midiCable;
extern const uint8_t midiChannel;
extern const uint8_t NoteOn;
extern const uint8_t NoteOff;
extern const uint8_t NoteAT;
extern const uint8_t MidiCtrl;




extern uint8_t midiTx[maxTxSize];

extern uint8_t txPtrIn;


void midiInit();

void MidiNoetOff(uint8_t p,uint8_t v);
void MidiNoteOn(uint8_t p,uint8_t v);
void MidiNoteSus(uint8_t p,uint8_t v);
void MidiCC(uint8_t c,uint8_t v);

void midiAddMsg(uint8_t *msg, uint8_t length);


void MidiSend();

#endif /* INC_MIDI_H_ */
