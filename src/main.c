/**
 *****************************************************************************
 **
 **  File        : main.c
 **
 **  Abstract    : main function.
 **
 **  Functions   : main
 **
 **  Environment : Atollic TrueSTUDIO(R)
 **
 **  Distribution: The file is distributed �as is,� without any warranty
 **                of any kind.
 **
 **  (c)Copyright Atollic AB.
 **  You may use this file as-is or modify it according to the needs of your
 **  project. Distribution of this file (unmodified or modified) is not
 **  permitted. Atollic AB permit registered Atollic TrueSTUDIO(R) users the
 **  rights to distribute the assembled, compiled & linked contents of this
 **  file as part of an application binary file, provided that it is built
 **  using the Atollic TrueSTUDIO(R) toolchain.
 **
 *****************************************************************************
 */

/* Includes */
#include "stm32f30x.h"
#include "stm32f3_discovery.h"
#include "common.h"

/* Private typedef */

/* Private define  */
#define DELAY	(1000)

/* Private macro */

/* Private variables */
float a, b, c, d;

/* Private function prototypes */

/* Private functions */

/* Global variables */




/**
 **===========================================================================
 **
 **  Abstract: main program
 **
 **===========================================================================
 */
int main(void)
{
	uint32_t ii;

	/* Example use SysTick timer and read System core clock */
	SysTick_Config(72);  /* 1 us if clock frequency 72 MHz */

	SystemCoreClockUpdate();
	ii = SystemCoreClock;   /* This is a way to read the System core clock */

	/* Initialize LEDs and User Button available on STM32F3-Discovery board */
	STM_EVAL_LEDInit(LED3);
	STM_EVAL_LEDInit(LED4);
	STM_EVAL_LEDInit(LED5);
	STM_EVAL_LEDInit(LED6);
	STM_EVAL_LEDInit(LED7);
	STM_EVAL_LEDInit(LED8);
	STM_EVAL_LEDInit(LED9);
	STM_EVAL_LEDInit(LED10);

	//STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_EXTI);

	while (1)
	{
		/* Waiting User Button is pressed */
		//while (UserButtonPressed == 0x00)
		{
			/* Toggle LD3 */
			STM_EVAL_LEDToggle(LED3);
			_delay_ms(DELAY);

			/* Toggle LD5 */
			STM_EVAL_LEDToggle(LED5);
			_delay_ms(DELAY);

			/* Toggle LD7 */
			STM_EVAL_LEDToggle(LED7);
			_delay_ms(DELAY);

			/* Toggle LD9 */
			STM_EVAL_LEDToggle(LED9);
			_delay_ms(DELAY);

			/* Toggle LD10 */
			STM_EVAL_LEDToggle(LED10);
			_delay_ms(DELAY);

			/* Toggle LD8 */
			STM_EVAL_LEDToggle(LED8);
			_delay_ms(DELAY);

			/* Toggle LD6 */
			STM_EVAL_LEDToggle(LED6);
			_delay_ms(DELAY);

			/* Toggle LD4 */
			STM_EVAL_LEDToggle(LED4);
			_delay_ms(DELAY);
		}
	}

	/* Program will never run to this line */
	return 0;
}
