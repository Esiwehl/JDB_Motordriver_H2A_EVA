/*
 * AutoCruiseControl.h
 *
 * Created: 25-6-2018 18:28:20
 *  Author: felix
 */ 


#ifndef AUTOCRUISECONTROL_H_
#define AUTOCRUISECONTROL_H_

#define STRLENGPS			15
#define NMEACHECKSUMSIGN	'*'
#define EARTH_RADIUS 6371e3
#define DEG2RAD(x) ((x) * M_PI / 180.0)
#define RAD2DEG(x) ((x) * 180.0 / M_PI)
#define DETECTRADIUS		3
#define FIETS_WHEEL_METER_PER_ROT	3.491f
#define FIETS_WHEEL_PULSE_PER_ROT	64
#define FIETS_WHEEL_METER_PER_PULSE	(FIETS_WHEEL_METER_PER_ROT / FIETS_WHEEL_PULSE_PER_ROT)


#define SPEEDSECTOR1	10
#define SPEEDSECTOR2	15
#define SPEEDSECTOR3	30


//uint32_t DistanceCorrect();
uint8_t CCspeed();
void load_distance(uint32_t PositivePulses);
void load_nmeadata (char GPSpos[]);
void transferbearing(float bearingshit);
void transferdata(char data[]);


#endif /* AUTOCRUISECONTROL_H_ */