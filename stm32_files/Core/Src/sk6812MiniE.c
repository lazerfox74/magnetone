#include "sk6812MiniE.h"


volatile uint8_t SK6812_DMA_COMPLETE_FLAG;


SK6812_LEDDATARGB SK6812_LEDDATA[SK6812_NUM_LEDS];
uint32_t SK6812_DMA_BUF[SK6812_DMA_BUF_LEN];

bool SK6812_UPDATE_READY;


const interFaceCol Colours[COLOUR_COUNT] = {
    {255, 0, 0},   // Red
    {0, 255, 0},   // Green
    {255, 0, 0},   // Blue
    {255, 255, 0}, // Yellow
    {255, 0, 255}, // Magenta
    {0, 255, 255}, // Cyan
    {255, 255, 255}, // White
    {0, 0, 0},      // Black
    {255,165,0} // Orange
};

HAL_StatusTypeDef SK6812_Init()
{
    HAL_StatusTypeDef halStatus = HAL_TIM_PWM_Init(&SK6812_TIM);

    //clear buffer
    for(uint16_t i = 0; i < SK6812_DMA_BUF_LEN;i++)
    {
        SK6812_DMA_BUF[i] = 0;
    }

    //set ready flag
    SK6812_DMA_COMPLETE_FLAG = 1;
    SK6812_UPDATE_READY = true;

    return halStatus;
}

void SK6812_SetColour(uint8_t index,const interFaceCol *col)
{
    SK6812_LEDDATA[index].colour.r = col->r;
    SK6812_LEDDATA[index].colour.g = col->g;
    SK6812_LEDDATA[index].colour.b = col->b;

    //set updateready flag to signal an update is needed
    SK6812_UPDATE_READY = true;

}


HAL_StatusTypeDef SK6812_Update()
{
    if(!SK6812_DMA_COMPLETE_FLAG)
    {
        return HAL_BUSY;
    }

//    xSemaphoreTake(SK6812_DMA_CompleteHandle,portMAX_DELAY);

    uint16_t bufIndex = 0;

    for(uint8_t ledIndex = 0;ledIndex < SK6812_NUM_LEDS;ledIndex++)
    {
        for(uint8_t bitIndex = 0; bitIndex<SK6812_BITS_PER_LED;bitIndex++)
        {

            if((SK6812_LEDDATA[ledIndex].data >> bitIndex) & 0x01)
            {
                SK6812_DMA_BUF[bufIndex] = SK6812_HI_VAL;
            }
            else
            {
                SK6812_DMA_BUF[bufIndex] = SK6812_LO_VAL;
            }

            bufIndex++;
        }

    }

    HAL_StatusTypeDef halStatus = HAL_TIM_PWM_Start_DMA(&SK6812_TIM,SK6812_TIM_CHANNEL,(uint32_t *)SK6812_DMA_BUF,SK6812_DMA_BUF_LEN);
    if (halStatus == HAL_OK)
    {
        SK6812_DMA_COMPLETE_FLAG = 0;
        //signal that update is not needed
        SK6812_UPDATE_READY = false;
    }

    return halStatus;
    
}

void SK6812_Callback()
{
    HAL_TIM_PWM_Stop_DMA(&SK6812_TIM,SK6812_TIM_CHANNEL);
    SK6812_DMA_COMPLETE_FLAG = 1;
//    BaseType_t xHigherPriorityTaskWoken;
//    xHigherPriorityTaskWoken = pdFALSE;
//    xSemaphoreGiveFromISR(SK6812_DMA_CompleteHandle,&xHigherPriorityTaskWoken);
//    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}
