/*
 * common.c
 *
 *  Created on: Nov 14, 2012
 *      Author: andrei
 */


#include "common.h"


__IO uint32_t TimingDelay = 0;


/**
 **===========================================================================
 **
 **  Abstract: SysTick interrupt handler
 **
 **===========================================================================
 */
void SysTick_Handler(void)
{
	if (TimingDelay != 0x00)
	{
		TimingDelay--;
	}
}

/**
 * @brief  Inserts a delay time.
 * @param  ms: specifies the delay time length, in ms.
 * @retval None
 */
void _delay_ms(__IO uint32_t ms)
{
	TimingDelay = ms * 1000;
	while(TimingDelay != 0);
}

/**
 * @brief  Inserts a delay time.
 * @param  ms: specifies the delay time length, in ms.
 * @retval None
 */
void _delay_us(__IO uint32_t us)
{
	TimingDelay = us;
	while(TimingDelay != 0);
}

