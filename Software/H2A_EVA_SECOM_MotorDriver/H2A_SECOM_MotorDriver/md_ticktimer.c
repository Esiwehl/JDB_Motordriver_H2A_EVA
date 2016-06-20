/*
 * md_ticktimer.c
 *
 * Created: 04/24/2013 9:45:43 PM
 *  Author: bakker
 */ 


#define TICKTIMER_PERIOD	65535 /* At 32MHz/DIV8 this overflows every 16.384ms */

#define US_PER_TICK	((F_CPU / 8) / 1000000UL)


#include <stdio.h>
#include <stddef.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "md_ticktimer.h"


void InitTimer(void) {

	TCE0.CTRLA = TC_CLKSEL_DIV8_gc;
	TCE0.CTRLB = 0x00; /* No input capture, normal mode */
	/* CTRLC is of no interest to us */
	TCE0.CTRLD = TC_EVACT_OFF_gc | TC_EVSEL_OFF_gc; /* No events */
	TCE0.CTRLE = 0x00; /* No byte mode */
	TCE0.PER = TICKTIMER_PERIOD;
	TCE0.INTCTRLA = TC_ERRINTLVL_OFF_gc | TC_OVFINTLVL_OFF_gc; /* No interrupts (for now) */
	TCE0.INTCTRLB = 0x00; /* Disable Compare/Capture interrupts */
	TCE0.CNT = 0;

} /* InitTimer */

timetick_t GetTicks(void) {
	return TCE0.CNT;
} /* GetTicks */


timetick_t BusyWaitTillAfter(timetick_t then, int usWait) {
	timetick_t tickWait = usWait * US_PER_TICK, delta, now;
	
	do {
		now = GetTicks();
		if(now < then) // Fix wraparound
		now += TICKTIMER_PERIOD;
		delta = now - then;
	} while(delta < tickWait);
	
	return now;
	
} /* BusyWaitTillAfter */

