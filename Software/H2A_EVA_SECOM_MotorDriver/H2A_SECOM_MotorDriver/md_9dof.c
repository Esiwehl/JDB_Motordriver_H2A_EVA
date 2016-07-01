/*
 * md_9dof.c
 *
 * Created: 6/26/2016 11:32:51 PM
 *  Author: bakker
 *  Based on gyro.c by Marnix Davidson
 */ 

#include <stdio.h>
#include <stddef.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>

#include "md_9dof.h"


#define SDA_POORT 	 	PORTE
#define SDA		 	 	PIN6_bm

#define SDO_G_POORT 	PORTF
#define SDO_G		 	PIN0_bm
#define SDO_XM_POORT 	PORTE
#define SDO_XM 			PIN7_bm

#define CS_G_POORT 		PORTC
#define CS_G		 	PIN0_bm
#define CS_XM_POORT 	PORTC
#define CS_XM			PIN7_bm
#define SCL_POORT 		PORTD
#define SCL			 	PIN0_bm

#define SELECT_XM		0
#define SELECT_G		1

#define GET_X			0
#define GET_Y			1
#define GET_Z			2


// Linear Acceleration: mg per LSB
#define LSM9DS0_ACCEL_MG_LSB_2G (0.061)
#define LSM9DS0_ACCEL_MG_LSB_4G (0.122)
#define LSM9DS0_ACCEL_MG_LSB_6G (0.183)
#define LSM9DS0_ACCEL_MG_LSB_8G (0.244)
#define LSM9DS0_ACCEL_MG_LSB_16G (0.732) // Is this right? Was expecting 0.488F

// Magnetic Field Strength: gauss range
#define LSM9DS0_MAG_MGAUSS_2GAUSS      (0.08)
#define LSM9DS0_MAG_MGAUSS_4GAUSS      (0.16)
#define LSM9DS0_MAG_MGAUSS_8GAUSS      (0.32)
#define LSM9DS0_MAG_MGAUSS_12GAUSS     (0.48)

// Angular Rate: dps per LSB
#define LSM9DS0_GYRO_DPS_DIGIT_245DPS      (0.00875)
#define LSM9DS0_GYRO_DPS_DIGIT_500DPS      (0.01750)
#define LSM9DS0_GYRO_DPS_DIGIT_2000DPS     (0.07000)

#define SENSORS_GRAVITY_EARTH	9.81
#define GYRO_RES	0.0074770348//245/32768


static void S9DOFWrite(char Address, char Data, uint8_t xm_or_g);
static uint16_t S9DOFRead(char Address, uint8_t bytes, uint8_t xm_or_g);


void Init9DOF(void) {
	
	SDA_POORT.DIRSET	=	SDA;
	CS_G_POORT.DIRSET	=	CS_G;
	CS_XM_POORT.DIRSET	=	CS_XM;
	SCL_POORT.DIRSET	=	SCL;

	SCL_POORT.OUTSET	=	SCL;
	SDA_POORT.OUTCLR	=	SDA;
	CS_G_POORT.OUTSET	=	CS_G;
	CS_XM_POORT.OUTSET	=	CS_XM;

	SDO_G_POORT.DIRCLR	=	SDO_G;
	SDO_XM_POORT.DIRCLR	=	SDO_XM;

	S9DOFWrite(0x20, 0x47, SELECT_XM);
	S9DOFWrite(0x21, 0x08, SELECT_XM);
	S9DOFWrite(0x24, 0x94, SELECT_XM);//was 0x10
	S9DOFWrite(0x25, 0x20, SELECT_XM);
	S9DOFWrite(0x26, 0x00, SELECT_XM);

	S9DOFWrite(0x20, 0x8F, SELECT_G);
	S9DOFWrite(0x21, 0x00, SELECT_G);
	
} /* Init9DOF */


void Process9DOF(void) {
	
} /* Process9DOF */


void PrintCSV_9DOF(FILE *fd) {
	
	// Dummy print for now. Format will be: Accel[XYZ],Magneto[XYZ],Gyro[XYZ],Temp
	fprintf(fd, "%.3f,%.3f,%.3f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,",
#if 0
		0.0,0.0,0.0,
		0.0,0.0,0.0,
		0.0,0.0,0.0,
		0.0
#else
		GyroGetAcceleration(GET_X), GyroGetAcceleration(GET_Y), GyroGetAcceleration(GET_Z),
		GyroGetMagnetic(GET_X), GyroGetMagnetic(GET_Y), GyroGetMagnetic(GET_Z),
		GyroGetGyro(GET_X), GyroGetGyro(GET_Y), GyroGetGyro(GET_Z),
		GyroGetTemp()
#endif
		);
	
} /* PrintCSV_9DOF */


static void S9DOFWrite(char Address, char Data, uint8_t xm_or_g) {
	uint8_t bit = 0, i;

	if (xm_or_g == SELECT_G)
		CS_G_POORT.OUTCLR = CS_G;
	else 
		CS_XM_POORT.OUTCLR = CS_XM;

	for(i = 0; i < 8; i++) {
		bit = ((Address << i) & 0x80);
		if (bit) {
			SDA_POORT.OUTSET = SDA;
		}
		else {
			SDA_POORT.OUTCLR = SDA;
		}
		SCL_POORT.OUTCLR = SCL;
		SCL_POORT.OUTSET = SCL;
	}


	for(i = 0; i < 8; i++) {
		bit = ((Data << i) & 0x80);
		if (bit) {
			SDA_POORT.OUTSET = SDA;
		}
		else {
			SDA_POORT.OUTCLR = SDA;
		}
		SCL_POORT.OUTCLR = SCL;
		SCL_POORT.OUTSET = SCL;
	}

	if (xm_or_g == SELECT_G)
		CS_G_POORT.OUTSET = CS_G;
	else 
		CS_XM_POORT.OUTSET = CS_XM;
		
} /* S9DOFWrite */


static uint16_t S9DOFRead(char Address, uint8_t bytes, uint8_t xm_or_g) {
	uint8_t bit, i;
	int8_t bitG = 0;
	int8_t bitXM = 0;
	uint16_t data = 0;
	uint16_t byteGyroG = 0, byte2GyroG =0;
	uint16_t byteGyroXM = 0, byte2GyroXM = 0;

	if (xm_or_g == SELECT_G)
		CS_G_POORT.OUTCLR = CS_G;
	else 
		CS_XM_POORT.OUTCLR = CS_XM;

	Address |= 0x80;

	for(i = 0; i < 8; i++) {
		bit = !!((Address << i) & 0x80);
		if (bit) {
			SDA_POORT.OUTSET = SDA;
		}
		else {
			SDA_POORT.OUTCLR = SDA;
		}
		SCL_POORT.OUTCLR = SCL;
		SCL_POORT.OUTSET = SCL;
	}
	SDA_POORT.OUTSET = SDA;

	for(i = 0; i < 8; i++) {
		SCL_POORT.OUTCLR = SCL;

		if(SDO_G_POORT.IN & SDO_G)
			bitG = 1;
		else 
			bitG = 0;
		byteGyroG = ((byteGyroG&0x7F)<<1)|bitG;

		if(SDO_XM_POORT.IN & SDO_XM)
			bitXM = 1;
		else
			bitXM = 0;
		byteGyroXM = ((byteGyroXM&0x7F)<<1)|bitXM;

		SCL_POORT.OUTSET = SCL;
	}

	if(bytes>1) {
		for(int count = 0; count < 8; count++) {
			SCL_POORT.OUTCLR = SCL;

			if(SDO_G_POORT.IN & SDO_G)	bitG = 1;
			else bitG = 0;
			byte2GyroG = ((byte2GyroG&0x7F)<<1)|bitG;

			if(SDO_XM_POORT.IN & SDO_XM)
				bitXM = 1;
			else 
				bitXM = 0;
			byte2GyroXM = ((byte2GyroXM &0x7F)<<1)|bitXM;

			SCL_POORT.OUTSET = SCL;
		}
	}

	if (xm_or_g == SELECT_G)
		CS_G_POORT.OUTSET = CS_G;
	else 
		CS_XM_POORT.OUTSET = CS_XM;

	if (xm_or_g == SELECT_G) 
		data = (((uint16_t) byte2GyroG) << 8) | byteGyroG;
	else 
		data = (((uint16_t) byte2GyroXM) << 8) | byteGyroXM;

	return data;
}


float GyroGetTemp(void) {
	int16_t data = 0;

	data = (int16_t)S9DOFRead(0xC5,2,SELECT_XM);

	return data / 8.0f;
}


float GyroGetMagnetic(uint8_t X_Y_Z)
{
	int16_t data = 0;
	if (X_Y_Z == GET_X)
	{
		data = (int16_t) S9DOFRead(0xC8,2,SELECT_XM);
	}
	if (X_Y_Z == GET_Y)
	{
		data = (int16_t) S9DOFRead(0xCA,2,SELECT_XM);
	}
	if (X_Y_Z == GET_Z)
	{
		data = (int16_t) S9DOFRead(0xCC,2,SELECT_XM);
	}

//if(data||0x8000)data = (~data +1)*(-1);

	float return_data = ((float)data / 1000)*LSM9DS0_MAG_MGAUSS_4GAUSS;

	return return_data;
}


float GyroGetAcceleration(uint8_t X_Y_Z) {
	int16_t data = 0;
	
	if (X_Y_Z == GET_X)
	{
		data = (int16_t) S9DOFRead(0xE8,2,SELECT_XM);
	}
	if (X_Y_Z == GET_Y)
	{
		data = (int16_t) S9DOFRead(0xEA,2,SELECT_XM);
	}
	if (X_Y_Z == GET_Z)
	{
		data = (int16_t) S9DOFRead(0xEC,2,SELECT_XM);
	}

	//if(data||0x8000)data = (~data +1)*(-1);

	float return_data = (((float)data / 1000) * SENSORS_GRAVITY_EARTH)*LSM9DS0_ACCEL_MG_LSB_4G; // factor voor

	return return_data;
}


float GyroGetGyro(uint8_t X_Y_Z) {
	int16_t data = 0;

	if (X_Y_Z == GET_X)
	{
		data = (int16_t) S9DOFRead(0x68,2,SELECT_G);
	}
	if (X_Y_Z == GET_Y)
	{
		data = (int16_t) S9DOFRead(0x6A,2,SELECT_G);
	}
	if (X_Y_Z == GET_Z)
	{
		data = (int16_t) S9DOFRead(0x6C,2,SELECT_G);
	}

//if(data||0x8000)data = (~data +1)*(-1);

	float return_data = ((float)data)*LSM9DS0_GYRO_DPS_DIGIT_245DPS;

	return return_data;
}

