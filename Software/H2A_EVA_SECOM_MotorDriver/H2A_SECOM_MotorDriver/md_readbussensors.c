/*
 * md_readbussensors.c
 *
 * Created: 13-5-2013 13:56:23
 *  Author: bakjd
 */ 


#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>

#include "md_readbussensors.h"
#include "DataInPrivate.h"
#include "AutoCruiseControl.h"


/* Go waste some memory */
static char sGPSPos[MAXDATA] = ",", sGPSDirection[MAXDATA] = "", sGPSSpeed[MAXDATA] = "", sGPSTime[MAXDATA] = "";


static int AcceptDataGPSPosition(const char *adr, char *inData);
static int AcceptDataGPSDirection(const char *adr, char *inData);
static int AcceptDataGPSSpeed(const char *adr, char *inData);
static int AcceptDataGPSTime(const char *adr, char *inData);

void InitReadBussensors(void) {
	
	AddWantedSensor("LL01", AcceptDataGPSPosition);
	AddWantedSensor("GR01", AcceptDataGPSDirection);
	AddWantedSensor("SG01", AcceptDataGPSSpeed);
	AddWantedSensor("GT01", AcceptDataGPSTime);
	
} /* InitReadBussensors */


void PrintBussensors(FILE *fp) {
	
	fprintf(fp, "%s,%s,%s,%s,", sGPSPos, sGPSDirection, sGPSSpeed, sGPSTime);
	
} /* PrintBussensors */


static void CopySensorData(char *dest, const char *src) {
	uint8_t i = 0;
	
	while(i < MAXDATA && src[i] != '\'' && src[i] != '%' && src[i] != '\0') {
		dest[i] = src[i];
		i++;
	}		
	
	if(i < MAXDATA)
		dest[i] = '\0';
	else
		dest[MAXDATA - 1] = '\0';
	
} /* CopySensorData */


static int AcceptDataGPSPosition(const char *adr, char *inData) {
	CopySensorData(sGPSPos, inData);
	load_nmeadata(sGPSPos);

	return 0;
	
} /* AcceptDataGPSPosition */


static int AcceptDataGPSDirection(const char *adr, char *inData) {
	
	CopySensorData(sGPSDirection, inData);
	return 0;
	
} /* AcceptDataGPSDirection */


static int AcceptDataGPSSpeed(const char *adr, char *inData) {
	
	CopySensorData(sGPSSpeed, inData);
	return 0;
	
} /* AcceptDataGPSSpeed */


static int AcceptDataGPSTime(const char *adr, char *inData) {
	
	CopySensorData(sGPSTime, inData);
	return 0;
		
} /* AcceptDataGPSTime */


