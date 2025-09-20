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
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include <system.h>
#include "altera_avalon_pio_regs.h"
#include "sys/alt_irq.h"

uint8_t pb_val = 0;

static void button_ISR(void *context, alt_u32 id)
{
	printf("BUTTON INTERRUPT\n");
	// disable button interrupts
	IOWR(BUTTON_PIO_BASE, 2, 0x0);

	// set period
	IOWR(TIMER_0_BASE, 0x02, 0xE360);
    IOWR(TIMER_0_BASE, 0x03, 0x0016);

    // Enable Timer
    IOWR(TIMER_0_BASE, 0x01, 0x05);
}

static void timer_ISR(void *context, alt_u32 id)
{
	printf("TIMER INTERRUPT\n");
	// read data from button
	uint8_t data = IORD(BUTTON_PIO_BASE,0);
	// if button is not default, store data in global variable
	if(data!=15)
	{
		pb_val = data;
		printf("BUTTON VALUE: %d\n", pb_val);
	}

	// acknowledge interrupt request by clearing status register
	IOWR(TIMER_0_BASE,0,0);
	IOWR(TIMER_0_BASE,1,0);

	// clear button interrupt
	IOWR(BUTTON_PIO_BASE, 0x03, 0x0);
	// enable button interrupt
	IOWR(BUTTON_PIO_BASE, 0x02, 0x0F);

}



int main()
{
	printf("Hello from Nios II!\n");


	// register isr
	alt_irq_register( BUTTON_PIO_IRQ, (void *)0, button_ISR);
	alt_irq_register( TIMER_0_IRQ, (void *)0, timer_ISR);

	// clear previous timer
	IOWR(TIMER_0_BASE, 0x01, 0x0);

	// enable button_IRQ
	IOWR(BUTTON_PIO_BASE, 0x02, 0x0F);
	while(1);

  return 0;
}
