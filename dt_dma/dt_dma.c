#include "dt_dma.h"

void DMA_Init(DMA_Channel_TypeDef* channel, uint32_t perif, uint32_t mem, uint16_t size, uint16_t conf)
{
	uint32_t tmp = 0;

	tmp = channel->CCR;
	tmp &= CCR_CLEAR_Mask;
	tmp |= conf;

	channel->CNDTR = size;
	channel->CPAR = perif;
	channel->CMAR = mem;
	channel->CCR = tmp;
}

uint32_t DMA_GetCurrentDataCounter(DMA_Channel_TypeDef* channel)
{
	return channel->CNDTR;
}

void DMA_Enable(DMA_Channel_TypeDef* channel)
{
    channel->CCR |= DMA_CCR1_EN;
}

void DMA_Disable(DMA_Channel_TypeDef* channel)
{
    channel->CCR &= (uint16_t)(~DMA_CCR1_EN);
}

void DMA_DeInit(DMA_Channel_TypeDef* channel)
{
    channel->CCR &= (uint16_t)(~DMA_CCR1_EN);
    channel->CCR = 0;
    channel->CNDTR = 0;
    channel->CPAR = 0;
    channel->CMAR = 0;

	if (channel == DMA1_Channel1){
		/* Reset interrupt pending bits for DMA1 Channel1 */
        DMA1->IFCR |= DMA1_Channel1_IT_Mask;
    }
	else if (channel == DMA1_Channel2){
        /* Reset interrupt pending bits for DMA1 Channel2 */
        DMA1->IFCR |= DMA1_Channel2_IT_Mask;
	}
	else if (channel == DMA1_Channel3){
        /* Reset interrupt pending bits for DMA1 Channel3 */
        DMA1->IFCR |= DMA1_Channel3_IT_Mask;
	}
	else if (channel == DMA1_Channel4){
        /* Reset interrupt pending bits for DMA1 Channel4 */
        DMA1->IFCR |= DMA1_Channel4_IT_Mask;
	}
	else if (channel == DMA1_Channel5){
        /* Reset interrupt pending bits for DMA1 Channel5 */
        DMA1->IFCR |= DMA1_Channel5_IT_Mask;
	}
	else if (channel == DMA1_Channel6){
        /* Reset interrupt pending bits for DMA1 Channel6 */
        DMA1->IFCR |= DMA1_Channel6_IT_Mask;
	}
	else if (channel == DMA1_Channel7){
        /* Reset interrupt pending bits for DMA1 Channel7 */
        DMA1->IFCR |= DMA1_Channel7_IT_Mask;
	}
	else if (channel == DMA2_Channel1){
        /* Reset interrupt pending bits for DMA2 Channel1 */
        DMA2->IFCR |= DMA2_Channel1_IT_Mask;
	}
	else if (channel == DMA2_Channel2){
        /* Reset interrupt pending bits for DMA2 Channel2 */
        DMA2->IFCR |= DMA2_Channel2_IT_Mask;
	}
	else if (channel == DMA2_Channel3){
        /* Reset interrupt pending bits for DMA2 Channel3 */
        DMA2->IFCR |= DMA2_Channel3_IT_Mask;
	}
	else if (channel == DMA2_Channel4){
        /* Reset interrupt pending bits for DMA2 Channel4 */
        DMA2->IFCR |= DMA2_Channel4_IT_Mask;
	}
	else if (channel == DMA2_Channel5){
		/* Reset interrupt pending bits for DMA2 Channel5 */
		DMA2->IFCR |= DMA2_Channel5_IT_Mask;
	}
}
