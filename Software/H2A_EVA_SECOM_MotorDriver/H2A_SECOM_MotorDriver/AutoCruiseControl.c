/*
* AutoCruiseControl.c
*
* Created: 25-6-2018 18:27:53
*  Author: felix & Rowan
*/
#include <avr/io.h>

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <avr/io.h>
#include "AutoCruiseControl.h"
#include "md_serial.h"

typedef struct {
	double lat;
	double lon;
} tRMC;

struct NMEA
{
	char cur_lat[STRLENGPS];
	char cur_long[STRLENGPS];	
};
struct NMEA NMEA_use;

static tRMC cur_pos;
static tRMC *cur_posPtr = &cur_pos;
static tRMC *Point_Ptr;
static tRMC Point1 = {52.34565167,4.9144633};
static tRMC Point2 = {52.302805,4.56847};

static int ready2calculate=0;
static uint32_t sAfstandafgelegd;


static double Distance(const tRMC *orig, const tRMC *dest)
{
	double phi1 = DEG2RAD(orig->lat);
	double phi2 = DEG2RAD(dest->lat);
	double deltaphi = DEG2RAD(dest->lat - orig->lat);
	double deltalambda = DEG2RAD(dest->lon - orig->lon);

	double a = sin(deltaphi / 2.0) * sin(deltaphi / 2.0) + cos(phi1) * cos(phi2) * sin(deltalambda / 2) * sin(deltalambda / 2);
	double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));

	return EARTH_RADIUS * c;
	} /* Distance */

static double Bearing(const tRMC *orig, const tRMC *dest) {

		double phi1 = DEG2RAD(orig->lat);
		double phi2 = DEG2RAD(dest->lat);
		double deltalambda = DEG2RAD(dest->lon - orig->lon);

		double y = sin(deltalambda) * cos(phi2);
		double x = cos(phi1) * sin(phi2) - sin(phi1) * cos(phi2) * cos(deltalambda);

		double bearing = RAD2DEG(atan2(y, x));

		return (bearing >= 0.0) ? bearing : bearing + 360.0;
} /* Bearing */

void load_nmeadata (char *here)
{
	//points to all variables in the struct
	char *fields[2] = {NMEA_use.cur_long, NMEA_use.cur_lat};
	int j = 0;
	
	for (int i = 0; i < 2; i ++)
	{
		while(*here != ',' && *here != '\0' && j < STRLENGPS && *here != NMEACHECKSUMSIGN)	//j represents the maximum length of the string in the struct
		{
			(fields[i])[j++] = *here++;		//fills struct-string i with the characters of NMEA_str with *here
			
		}
		(fields[i])[j] = '\0';		//when the whileloop isn't true, this makes sure that the last character of the string is \0
		
		if(*here == ',') //if the while loop isn't true and the character pointed to is a comma then:
		{
			*here++; //move to the next character after the comma
			j = 0;	//fill struct-string i from point j=0 again (start filling aray from 0 again)
		}
	}
	ready2calculate = 1;
}

static double decimal_lat_or_long(char *input)
{

	char min_str[10];
	int degrees = 0, i = 0;
	char *minPtr = input, *degrPtr = input; //minPtr and degrPtr are the now both the first value of NMEA longitude/latitude
	float decimal;
	while (*minPtr != '.' && *minPtr != '\0') minPtr++; //find the dot in their string

	minPtr = minPtr-2; //both longitude and latitude start their min 2 positions away from the dot
 	while (degrPtr != minPtr) {	//as long as the degrPtr isn't in the same position as minPtr, the number pointed to is part of the degree
 		degrees = (10 * degrees) + *degrPtr - '0';
 		degrPtr++;
 	}
	while (*minPtr != '\0') min_str[i++] = *minPtr++; //fill the min_str
	min_str[i] = '\0';
	decimal = (float)(atof(min_str) / 60) + degrees; //decimal longitude or latitude is degrees + min/60

	return decimal;
}



static int Location_cmp(const tRMC *point, const tRMC *curPos)
{
	if(Distance(curPos, point)<= DETECTRADIUS)
		return 1;
	else
		return 0;
}

void load_distance(uint32_t PositivePulses)
{
	sAfstandafgelegd = PositivePulses * FIETS_WHEEL_METER_PER_PULSE;	
}

uint32_t DistanceCorrect()
{
	if (ready2calculate)
	{
		cur_posPtr->lat = decimal_lat_or_long(NMEA_use.cur_lat);
		cur_posPtr->lon = decimal_lat_or_long(NMEA_use.cur_long);
		
		Point_Ptr = &Point1;
 		if (Location_cmp(Point_Ptr, cur_posPtr))		//na 100 meter			return 100; 		else
 			return sAfstandafgelegd;
		Point_Ptr = &Point2;							//na 200 meter		if (Location_cmp(Point_Ptr, cur_posPtr))			return 200;		else 			return sAfstandafgelegd;
		ready2calculate = 0;
	}
	else return sAfstandafgelegd;
}

uint8_t CCspeed()
{
	uint32_t d = DistanceCorrect();
	
	if (d < 100)
		return SPEEDSECTOR1;
	else if(d < 150)
		return SPEEDSECTOR2;
	else if(d < 200)
		return SPEEDSECTOR3;	
	else
		return 8;
}
