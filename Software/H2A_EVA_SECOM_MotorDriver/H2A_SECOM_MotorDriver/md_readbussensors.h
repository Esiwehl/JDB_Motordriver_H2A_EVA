/*
 * md_readbussensors.h
 *
 * Created: 13-5-2013 13:57:05
 *  Author: bakjd
 */ 


#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>


#ifndef MD_READBUSSENSORS_H_
#define MD_READBUSSENSORS_H_

void InitReadBussensors(void);
void PrintBussensors(FILE *fp);


#endif /* MD_READBUSSENSORS_H_ */