#ifndef SK6812MINIE
#define SK6812MINIE

#include "stm32f4xx_hal.h"
#include "stdbool.h"

#define SK6812_NUM_LEDS 16


#define SK6812_TIM htim2
#define SK6812_TIM_CHANNEL TIM_CHANNEL_3

#define SK6812_HI_VAL 60
#define SK6812_LO_VAL 30
#define SK6812_RST_PERIODS 80

#define SK6812_BITS_PER_LED 24

#define COLOUR_COUNT 9

#define SK6812_DMA_BUF_LEN ((SK6812_NUM_LEDS *SK6812_BITS_PER_LED ) + SK6812_RST_PERIODS)


typedef union 
{
    struct 
    {
        uint8_t g;
        uint8_t r;
        uint8_t b;
    }colour;

    uint32_t data;
}SK6812_LEDDATARGB;


typedef struct
{
    uint8_t r;
    uint8_t g;
    uint8_t b;

}interFaceCol;



typedef enum {
    COLOUR_RED,
    COLOUR_GREEN,
    COLOUR_BLUE,
    COLOUR_YELLOW,
    COLOUR_MAGENTA,
    COLOUR_CYAN,
    COLOUR_WHITE,
    COLOUR_OFF,
	COLOUR_ORANGE
} ColourIndex;

// Define an array of colors, indexed by the enum
extern const interFaceCol Colours[COLOUR_COUNT];

extern SK6812_LEDDATARGB SK6812_LEDDATA[SK6812_NUM_LEDS];
extern uint32_t SK6812_DMA_BUF[SK6812_DMA_BUF_LEN];
//replaced with semaphore
extern volatile uint8_t SK6812_DMA_COMPLETE_FLAG;

//extern osSemaphoreId SK6812_DMA_CompleteHandle;

extern TIM_HandleTypeDef htim2;

extern bool SK6812_UPDATE_READY;

HAL_StatusTypeDef SK6812_Init();
void SK6812_SetColour(uint8_t index,const interFaceCol *col);
HAL_StatusTypeDef SK6812_Update();
void SK6812_Callback();


#endif
