/*
 * note.h
 *
 *  Created on: Feb 13, 2025
 *      Author: Joshua
 */

#ifndef INC_NOTE_H_
#define INC_NOTE_H_

#include "stm32f4xx_hal.h"
#include "sk6812MiniE.h"
#include "stdbool.h"

#define numKeys 16
#define numHalfKeys 8

#define trigThresh 40
#define trigIncrement 6

#define maxVel 150

#define lerpAmt 0.07

extern uint8_t numActive;


extern uint8_t ADC_COMPLETE_FLAG;

extern uint32_t keyData[numKeys];
extern uint8_t keyCount;

#define selectpinsPort GPIOB

extern const int selectPins[3];

extern const int Leds[numKeys];

extern const int notePitches[numKeys];


extern ADC_HandleTypeDef hadc1;

extern int8_t octave;
//variable for averege of thresholds
extern uint16_t globalThresh;




typedef struct note_Struct
{
	uint16_t rawVal,restVal;

	float vel;
	float starVel;
	uint8_t pitch;

	//interpolation coeficcients
	float vA,vB;

	uint8_t ccNum;


	uint8_t ledIndex;
	bool isHeld;

	bool isTrig;

	//flag for checking if octave needs updating
	bool octUpdate;

	int8_t oct;

	void(*onTrigger)(struct note_Struct*);
	void(*onHold)(struct note_Struct*);
	void(*onRelease)(struct note_Struct*);

}note;

extern note noteArray[numKeys];

void triggerFunction(note *currentNote);
void holdFunction(note *currentNote);
void releaseFunction(note *currentNote);

void controlFunction(note *currentNote);



void octUpTrig(note *currentNote);
void octDownTrig(note *currentNote);
void octDownRel(note *currentNote);
void controlTrig(note *currentNote);
void controlRel(note *currentNote);


void octaveUpdate();

void noteInit();

void selectMuxPin(int pin);

void noteScan();

//linear interpolation function

float lerpF(float a,float b);




#endif /* INC_NOTE_H_ */
