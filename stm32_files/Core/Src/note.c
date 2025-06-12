/*
 * note.c
 *
 *  Created on: Feb 13, 2025
 *      Author: Joshua
 */


#include "note.h"



uint8_t numActive = 0;

uint8_t ADC_COMPLETE_FLAG = 0;

int8_t octave = 0;


uint32_t keyData[numKeys] = {0};
uint8_t keyCount = 0;

note noteArray[numKeys];

const int Leds[numKeys] = {6,7,4,5,3,2,0,1,14,15,12,13,11,10,9,8};

//const int notePitches[numKeys] = {39, 38, 41 , 40, 43, 42 ,36,37, 49, 48, 47, 46, 45, 44, 50, 51};

//const int notePitches[numKeys] = {14,13,12,11,10,9,15,16,7,8,2,1,4,3,6,5};

//const int notePitches[numKeys] = {12,11,14,13,16,15,9,10,6,5,4,3,2,1,7,8};

//const int notePitches[numKeys] = {8,7,10,9,12,11,60,60,6,5,4,3,2,1,60,60};

const int notePitches[numKeys] = {43, 42, 45, 44, 47, 46, 95, 95, 41, 40, 39, 38, 37, 36, 95, 95};


uint16_t globalThresh;


void noteInit()
{
	//start dma and get initial readings
	ADC_COMPLETE_FLAG = 0;


	//get initial readings
	keyCount = 0;
	selectMuxPin(0);
	HAL_Delay(50);

	HAL_ADC_Start_DMA(&hadc1,&keyData[0], 2);
	HAL_Delay(50);

	//blocking wait for dma to finish
	while(ADC_COMPLETE_FLAG != 1)
	{
		HAL_Delay(1);

	}
	ADC_COMPLETE_FLAG = 0;

	HAL_Delay(50);

	globalThresh = 0;

	for(int i = 0; i < numKeys;i++)
	{
		noteArray[i].restVal = noteArray[i].rawVal;
		noteArray[i].ledIndex = Leds[i];

		noteArray[i].pitch = notePitches[i];
		noteArray[i].isHeld = false;
		noteArray[i].isTrig = false;
		noteArray[i].starVel = 0;


		noteArray[i].oct = 0;

		noteArray[i].onTrigger = &triggerFunction;
		noteArray[i].onRelease = &releaseFunction;
		noteArray[i].onHold = &holdFunction;

		noteArray[i].octUpdate = false;


		if(i == 14)
		{
			noteArray[i].onTrigger = &octDownTrig;
			noteArray[i].onRelease = &octDownRel;
			noteArray[i].onHold = NULL;
			noteArray[i].isTrig = true;


		}
		else if(i==7)
		{


			noteArray[i].onTrigger = &octUpTrig;
			noteArray[i].onRelease = &octDownRel;
			noteArray[i].onHold = NULL;
			noteArray[i].isTrig = true;

		}
		else if(i == 15 || i ==6)
		{
			noteArray[i].onTrigger = &controlTrig;
			noteArray[i].onHold = &controlFunction;
			noteArray[i].onRelease = &controlRel;
			noteArray[i].isTrig = true;

		}
		if(i == 6)
		{
			noteArray[i].ccNum = 16;
		}
		else if(i == 15)
		{
			noteArray[i].ccNum = 1;
		}



		globalThresh += noteArray[i].restVal;

	}

	globalThresh = globalThresh/numKeys;

}



void noteScan()
{


	  for(uint8_t i = 0; i < numKeys;i++)
	  {

		  note* currentNote = &noteArray[i];

		  uint16_t threshold = (currentNote->restVal + trigThresh + (numActive * trigIncrement));

		  //check for note above threshold
		  if(currentNote->rawVal > threshold)
		  {

			  float velo = (float)((currentNote->rawVal - threshold) * 0.85);

			  currentNote->vel = (velo > 127) ? 127 : velo;


			  if(currentNote->isHeld == false)
			  {

				  if(currentNote->isTrig == false)
				  {

					  currentNote->starVel = currentNote->vel;
					  currentNote->isTrig = true;
				  }
				  else
				  {

					  currentNote->onTrigger(currentNote);
				  }
//				  if(currentNote->)
//				  currentNote->onTrigger(currentNote);


			  }
			  else
			  {
				  //send pressure data
				  if(currentNote->onHold)
				  {
					 currentNote->onHold(currentNote);
				  }
			  }

		  }
		  else
		  {
			  //if note is below threshold and held turn off
			  if(currentNote->isHeld == true)
			  {

				  currentNote->onRelease(currentNote);

			  }
		  }





//
//		  if(currentNote->rawVal > (currentNote->restVal + trigThresh))
//		  {
//			  SK6812_SetColour(currentNote->ledIndex, &Colours[COLOUR_BLUE]);
//		  }
//		  else
//		  {
//			  SK6812_SetColour(currentNote->ledIndex, &Colours[COLOUR_OFF]);
//		  }

	  }
}



void triggerFunction(note *currentNote)
{

	//note is active
	currentNote->isHeld = true;
	//increment active notes
	numActive++;

	//calculate velocity and trigger note
	uint8_t triggerVelocity = 2.54 * (currentNote->vel - currentNote->starVel);
	if(triggerVelocity > 127){triggerVelocity = 127;}

//	currentNote->vA = triggerVelocity;

//	currentNote->velF = triggerVelocity;

	MidiNoteOn(currentNote->pitch + currentNote->oct, triggerVelocity);
	currentNote->vA = triggerVelocity;

	//update led
	SK6812_SetColour(currentNote->ledIndex, &Colours[COLOUR_BLUE]);

}
void holdFunction(note *currentNote)
{

	currentNote->vA = lerpF(currentNote->vA,currentNote->vel);

	MidiNoteSus(currentNote->pitch + currentNote->oct, (uint8_t)currentNote->vA);
//	SK6812_SetColour(currentNote->ledIndex, &(interFaceCol){(uint8_t)(currentNote->vA * 2.0), 0, 255});

}
void releaseFunction(note *currentNote)
{

	currentNote->isHeld = false;
	currentNote->isTrig = false;

	numActive--;
	MidiNoteOff(currentNote->pitch + currentNote->oct, 127);

	//update octave if nececary

	if(currentNote->octUpdate == true)
	{
		currentNote->octUpdate = false;
		currentNote->oct = octave;
	}

	SK6812_SetColour(currentNote->ledIndex, &Colours[COLOUR_OFF]);

}

void octDownTrig(note *currentNote)
{
	//note is active
	currentNote->isHeld = true;
	//increment active notes
//	numActive++;

	//change octave
	if(octave <= -24)
	{

	}
	else
	{
		octave -= 12;
		octaveUpdate();

	}


}

void octUpTrig(note *currentNote)
{
	//note is active
	currentNote->isHeld = true;
	//increment active notes
//	numActive++;

	//change octave
	if(octave >= 24)
	{

	}
	else
	{
		octave += 12;
		octaveUpdate();
	}

}


void octDownRel(note *currentNote)
{
	currentNote->isHeld = false;
}

void controlFunction(note *currentNote)
{
	currentNote->vA = lerpF(currentNote->vA, currentNote->vel);

	MidiCC(currentNote->ccNum, (uint8_t)currentNote->vA);
	uint8_t colL = (uint8_t)(currentNote->vA * 2.0);
	SK6812_SetColour(currentNote->ledIndex, &(interFaceCol){0, colL, 255 - colL});
}

void controlTrig(note *currentNote)
{
	currentNote->isHeld = true;
	MidiCC(currentNote->ccNum, currentNote->vel);
	currentNote->vA = currentNote->vel;
	numActive++;
	uint8_t colL = (uint8_t)(currentNote->vA * 2.0);
	SK6812_SetColour(currentNote->ledIndex, &(interFaceCol){0, colL, 255 - colL});
}
void controlRel(note *currentNote)
{
	currentNote->isHeld = false;
	MidiCC(currentNote->ccNum, (uint8_t)0);
	SK6812_SetColour(currentNote->ledIndex, &Colours[COLOUR_OFF]);


	numActive--;
}

void octaveUpdate()
{
	//update all notes

	for(uint8_t i = 0; i< numKeys;i++)
	{
		if(noteArray[i].isHeld)
		{
			//flag octave for update in note release
			noteArray[i].octUpdate = true;
		}
		else
		{
			noteArray[i].oct = octave;
		}
	}


	switch (octave) {
		case 0:
				SK6812_SetColour(noteArray[14].ledIndex,&Colours[COLOUR_OFF]);

				SK6812_SetColour(noteArray[7].ledIndex,&Colours[COLOUR_OFF]);
			break;
		case -12:
				SK6812_SetColour(noteArray[14].ledIndex,&Colours[COLOUR_ORANGE]);
			break;
		case -24:
				SK6812_SetColour(noteArray[14].ledIndex,&Colours[COLOUR_RED]);
			break;
		case 12:
				SK6812_SetColour(noteArray[7].ledIndex,&Colours[COLOUR_ORANGE]);
			break;
		case 24:
				SK6812_SetColour(noteArray[7].ledIndex,&Colours[COLOUR_RED]);
			break;
	}
}






void selectMuxPin(int pin)
{
  for (int i=0; i<3; i++)
  {
    if (pin & (1<<i))
      HAL_GPIO_WritePin(selectpinsPort, selectPins[i], GPIO_PIN_SET);
    else
      HAL_GPIO_WritePin(selectpinsPort, selectPins[i], GPIO_PIN_RESET);
  }
}


float lerpF(float a,float b)
{
	return a + lerpAmt * (b - a);
}

