/*
 * "Hello World" example.
 *
 * This example prints 'Hello from Nios II' to the STDOUT stream. It runs on
 * the Nios II 'standard', 'full_featured', 'fast', and 'low_cost' example
 * designs. It runs with or without the MicroC/OS-II RTOS and requires a STDOUT
 * device in your system's hardware.
 * The memory footprint of this hosted application is ~69 kbytes by default
 * using the standard reference design.
 *
 * For a reduced footprint version of this template, and an explanation of how
 * to reduce the memory footprint for a given application, see the
 * "small_hello_world" template.
 *
 */

#include <stdio.h>
#include "system.h"
#include "altera_avalon_pio_regs.h"


int main()
{
	IOWR(LED_PIO_BASE,0,0x0);
	while (1)
	{
		int buttons = IORD(BUTTON_PIO_BASE,0);
		// if buttons = b'1110 ;  button is on
		int switches = IORD(SWITCH_PIO_BASE,0 );
		// if switches = b'1111 1110 ; switch 0 is "down"
		if(buttons == 14)
		{
			IOWR(LED_PIO_BASE,0,0xF);
		}
		if(buttons == 13)
		{
			IOWR(LED_PIO_BASE,0,0x0);
		}
	}
	return 0;
}
