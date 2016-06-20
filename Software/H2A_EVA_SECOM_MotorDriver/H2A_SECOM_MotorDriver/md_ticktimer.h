/*
 * md_ticktimer.h
 *
 * Created: 4/30/2013 9:49:21 PM
 *  Author: bakker
 */ 


#ifndef MD_TICKTIMER_H_
#define MD_TICKTIMER_H_


#define F_CPU 32000000UL

#include <stdint.h>

typedef uint16_t timetick_t;

void InitTimer(void);
timetick_t GetTicks(void);
timetick_t BusyWaitTillAfter(timetick_t then, int usWait);


#endif /* MD_TICKTIMER_H_ */