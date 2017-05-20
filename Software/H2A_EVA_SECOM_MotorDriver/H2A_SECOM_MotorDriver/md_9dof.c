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


#define LSM9DS0_WHO_AM_I_G	0x0F
#define LSM9DS0_CTRL_REG1_G	0x20
#define LSM9DS0_CTRL_REG2_G	0x21
#define LSM9DS0_CTRL_REG3_G	0x22
#define LSM9DS0_CTRL_REG4_G	0x23
#define LSM9DS0_CTRL_REG5_G	0x24
#define LSM9DS0_REFERENCE_G	0x25
#define LSM9DS0_STATUS_REG_G	0x27
#define LSM9DS0_OUT_X_L_G	0x28
#define LSM9DS0_OUT_X_H_G	0x29
#define LSM9DS0_OUT_Y_L_G	0x2A
#define LSM9DS0_OUT_Y_H_G	0x2B
#define LSM9DS0_OUT_Z_L_G	0x2C
#define LSM9DS0_OUT_Z_H_G	0x2D
#define LSM9DS0_FIFO_CTRL_REG_G	0x2E
#define LSM9DS0_FIFO_SRC_REG_G	0x2F
#define LSM9DS0_INT1_CFG_G	0x30
#define LSM9DS0_INT1_SRC_G	0x31
#define LSM9DS0_TSH_XH_G	0x32
#define LSM9DS0_TSH_XL_G	0x33
#define LSM9DS0_TSH_YH_G	0x34
#define LSM9DS0_TSH_YL_G	0x35
#define LSM9DS0_TSH_ZH_G	0x36
#define LSM9DS0_TSH_ZL_G	0x37
#define LSM9DS0_INT1_DURATION_G	0x38

#define LSM9DS0_OUT_TEMP_L_XM	0x05
#define LSM9DS0_OUT_TEMP_H_XM	0x06
#define LSM9DS0_STATUS_REG_M	0x07
#define LSM9DS0_OUT_X_L_M	0x08
#define LSM9DS0_OUT_X_H_M	0x09
#define LSM9DS0_OUT_Y_L_M	0x0A
#define LSM9DS0_OUT_Y_H_M	0x0B
#define LSM9DS0_OUT_Z_L_M	0x0C
#define LSM9DS0_OUT_Z_H_M	0x0D
#define LSM9DS0_WHO_AM_I_XM	0x0F
#define LSM9DS0_INT_CTRL_REG_M	0x12
#define LSM9DS0_INT_SRC_REG_M	0x13
#define LSM9DS0_INT_THS_L_M	0x14
#define LSM9DS0_INT_THS_H_M	0x15
#define LSM9DS0_OFFSET_X_L_M	0x16
#define LSM9DS0_OFFSET_X_H_M	0x17
#define LSM9DS0_OFFSET_Y_L_M	0x18
#define LSM9DS0_OFFSET_Y_H_M	0x19
#define LSM9DS0_OFFSET_Z_L_M	0x1A
#define LSM9DS0_OFFSET_Z_H_M	0x1B
#define LSM9DS0_REFERENCE_X	0x1C
#define LSM9DS0_REFERENCE_Y	0x1D
#define LSM9DS0_REFERENCE_Z	0x1E
#define LSM9DS0_CTRL_REG0_XM	0x1F
#define LSM9DS0_CTRL_REG1_XM	0x20
#define LSM9DS0_CTRL_REG2_XM	0x21
#define LSM9DS0_CTRL_REG3_XM	0x22
#define LSM9DS0_CTRL_REG4_XM	0x23
#define LSM9DS0_CTRL_REG5_XM	0x24
#define LSM9DS0_CTRL_REG6_XM	0x25
#define LSM9DS0_CTRL_REG7_XM	0x26
#define LSM9DS0_STATUS_REG_A	0x27
#define LSM9DS0_OUT_X_L_A	0x28
#define LSM9DS0_OUT_X_H_A	0x29
#define LSM9DS0_OUT_Y_L_A	0x2A
#define LSM9DS0_OUT_Y_H_A	0x2B
#define LSM9DS0_OUT_Z_L_A	0x2C
#define LSM9DS0_OUT_Z_H_A	0x2D
#define LSM9DS0_FIFO_CTRL_REG	0x2E
#define LSM9DS0_FIFO_SRC_REG	0x2F
#define LSM9DS0_INT_GEN1_REG	0x30
#define LSM9DS0_INT_GEN1_SRC	0x31
#define LSM9DS0_INT_GEN1_THS	0x32
#define LSM9DS0_INT_GEN1_DURATION	0x33
#define LSM9DS0_INT_GEN2_REG	0x34
#define LSM9DS0_INT_GEN2_SRC	0x35
#define LSM9DS0_INT_GEN2_THS	0x36
#define LSM9DS0_INT_GEN2_DURATION	0x37
#define LSM9DS0_CLICK_CFG	0x38
#define LSM9DS0_CLICK_SRC	0x39
#define LSM9DS0_CLICK_THS	0x3A
#define LSM9DS0_TIME_LIMIT	0x3B
#define LSM9DS0_TIME_LATENCY	0x3C
#define LSM9DS0_TIME_WINDOW	0x3D
#define LSM9DS0_ACT_THS	0x3E
#define LSM9DS0_ACT_DUR	0x3F

#define LSM9DS0_READ_BIT	0x80
#define LSM9DS0_ADDRINC_BIT	0x40

// Linear Acceleration: mg per LSB
#define LSM9DS0_ACCEL_MG_LSB_2G (0.061f)
#define LSM9DS0_ACCEL_MG_LSB_4G (0.122f)
#define LSM9DS0_ACCEL_MG_LSB_6G (0.183f)
#define LSM9DS0_ACCEL_MG_LSB_8G (0.244f)
#define LSM9DS0_ACCEL_MG_LSB_16G (0.732f) // Is this right? Was expecting 0.488F

// Magnetic Field Strength: gauss range
#define LSM9DS0_MAG_MGAUSS_2GAUSS      (0.08f)
#define LSM9DS0_MAG_MGAUSS_4GAUSS      (0.16f)
#define LSM9DS0_MAG_MGAUSS_8GAUSS      (0.32f)
#define LSM9DS0_MAG_MGAUSS_12GAUSS     (0.48f)

// Angular Rate: dps per LSB
#define LSM9DS0_GYRO_DPS_DIGIT_245DPS      (0.00875f)
#define LSM9DS0_GYRO_DPS_DIGIT_500DPS      (0.01750f)
#define LSM9DS0_GYRO_DPS_DIGIT_2000DPS     (0.07000f)

#define LSM9DS0_TEMP_DEG_PER_LSB	(0.125f)

#define SENSORS_GRAVITY_EARTH	9.81f

#define LSM9DS0_NUMAXES	3
#define LSM9DS0_BYTES_PER_SAMPLE 2
#define LSM9DS0_RAWDATASZ	(LSM9DS0_NUMAXES * LSM9DS0_BYTES_PER_SAMPLE)

static uint8_t sRawAccelData[LSM9DS0_RAWDATASZ], sRawGyroData[LSM9DS0_RAWDATASZ];
static uint8_t sRawMagnetoData[LSM9DS0_RAWDATASZ], sRawMagnetoOffset[LSM9DS0_RAWDATASZ];
static uint8_t sRawTemperatureData[LSM9DS0_BYTES_PER_SAMPLE];

#define CONCATU8TOU16(d) ((uint16_t)*((d)+1) << 8 | *(d))

static void LSM9DS0Write(uint8_t addr, uint8_t *data, uint8_t numBytes, uint8_t xm_or_g);
static void LSM9DS0WriteOne(uint8_t addr, uint8_t data, uint8_t xm_or_g);
static void LSM9DS0Read(uint8_t addr, uint8_t *data, uint8_t numBytes, uint8_t xm_or_g);
static uint16_t LSM9DS0ReadU16(uint8_t addr, uint8_t xm_or_g);

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

	LSM9DS0WriteOne(LSM9DS0_CTRL_REG1_XM, 0xA7, SELECT_XM); // 0xA7: 1600Hz update rate, continuous update, XYZ all enabled
	LSM9DS0WriteOne(LSM9DS0_CTRL_REG2_XM, 0x08, SELECT_XM); // 0x08: Anti-alias 773Hz, 4G full scale, self-test off
	LSM9DS0WriteOne(LSM9DS0_CTRL_REG5_XM, 0x94, SELECT_XM); // 0x94: temp sensor on, mag res low, 100Hz mag data rate
	LSM9DS0WriteOne(LSM9DS0_CTRL_REG6_XM, 0x20, SELECT_XM); // 0x20: 4Gauss full-scale magnetics
	LSM9DS0WriteOne(LSM9DS0_CTRL_REG7_XM, 0x00, SELECT_XM); // 0x00: high-pass filter off, mag normal power, mag continuous conversion

	LSM9DS0WriteOne(LSM9DS0_CTRL_REG1_G, 0xFF, SELECT_G); // 0xFF: ODR 760Hz, Cutoff 100Hz, normal mode, XYZ on
	LSM9DS0WriteOne(LSM9DS0_CTRL_REG2_G, 0x09, SELECT_G); // 0x09: HPF normal mode, HPF cutoff 0.09Hz
	
} /* Init9DOF */


void Process9DOF(void) {
	
	LSM9DS0Read(LSM9DS0_OUT_X_L_A, sRawAccelData, LSM9DS0_RAWDATASZ, SELECT_XM);
	LSM9DS0Read(LSM9DS0_OUT_X_L_G, sRawGyroData, LSM9DS0_RAWDATASZ, SELECT_G);
	LSM9DS0Read(LSM9DS0_OUT_X_L_M, sRawMagnetoData, LSM9DS0_RAWDATASZ, SELECT_XM);
	LSM9DS0Read(LSM9DS0_OFFSET_X_L_M, sRawMagnetoOffset, LSM9DS0_RAWDATASZ, SELECT_XM);
	LSM9DS0Read(LSM9DS0_OUT_TEMP_L_XM, sRawTemperatureData, LSM9DS0_BYTES_PER_SAMPLE, SELECT_XM);
	
} /* Process9DOF */


void PrintCSV_9DOF(FILE *fd) {
	
	// Dummy print for now. Format will be: Accel[XYZ],Magneto[XYZ],Gyro[XYZ],Temp
	fprintf(fd, "%.3f,%.3f,%.3f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,",
		GyroGetAcceleration(GET_X), GyroGetAcceleration(GET_Y), GyroGetAcceleration(GET_Z),
		GyroGetMagnetic(GET_X), GyroGetMagnetic(GET_Y), GyroGetMagnetic(GET_Z),
		GyroGetGyro(GET_X), GyroGetGyro(GET_Y), GyroGetGyro(GET_Z),
		GyroGetTemp());
	
} /* PrintCSV_9DOF */


static void LSM9DS0Write(uint8_t addr, uint8_t *data, uint8_t numBytes, uint8_t xm_or_g) {
	uint8_t byteIdx = 0, i;

	if (xm_or_g == SELECT_G)
		CS_G_POORT.OUTCLR = CS_G;
	else 
		CS_XM_POORT.OUTCLR = CS_XM;

	if(numBytes > 1)
		addr |=LSM9DS0_ADDRINC_BIT;

	for(i = 0x80; i > 0; i >>= 1) {
		SCL_POORT.OUTCLR = SCL;
		if (addr & i) 
			SDA_POORT.OUTSET = SDA;
		else 
			SDA_POORT.OUTCLR = SDA;
		SCL_POORT.OUTSET = SCL;
	}

	for(byteIdx = 0; byteIdx < numBytes; byteIdx++) {
		for(i = 0x80; i > 0; i >>= 1) {
			SCL_POORT.OUTCLR = SCL;
			if (data[byteIdx] & i)
				SDA_POORT.OUTSET = SDA;
			else 
				SDA_POORT.OUTCLR = SDA;
			SCL_POORT.OUTSET = SCL;
		}
	}

	if (xm_or_g == SELECT_G)
		CS_G_POORT.OUTSET = CS_G;
	else 
		CS_XM_POORT.OUTSET = CS_XM;
		
} /* LSM9DS0Write */


static void LSM9DS0WriteOne(uint8_t addr, uint8_t data, uint8_t xm_or_g) {
	
	LSM9DS0Write(addr, &data, 1, xm_or_g);
	
} /* LSM9DS0WriteOne */


static void LSM9DS0Read(uint8_t addr, uint8_t *data, uint8_t numBytes, uint8_t xm_or_g) {
	uint8_t byteIdx, i;

	if (xm_or_g == SELECT_G)
		CS_G_POORT.OUTCLR = CS_G;
	else 
		CS_XM_POORT.OUTCLR = CS_XM;

	addr |= LSM9DS0_READ_BIT;
	if(numBytes > 1)
		addr |= LSM9DS0_ADDRINC_BIT;

	for(i = 0x80; i > 0; i >>= 1) {
		if (addr & i)
			SDA_POORT.OUTSET = SDA;
		else
			SDA_POORT.OUTCLR = SDA;
		SCL_POORT.OUTCLR = SCL;
		SCL_POORT.OUTSET = SCL;
	}
	SDA_POORT.OUTSET = SDA;

	for(byteIdx = 0; byteIdx < numBytes; byteIdx++) {
		data[byteIdx] = 0;
		for(i = 0; i < 8; i++) {
			SCL_POORT.OUTCLR = SCL;
			SCL_POORT.OUTSET = SCL;
		
			data[byteIdx] <<= 1; 
			data[byteIdx] |= (xm_or_g == SELECT_XM) ? !!(SDO_XM_POORT.IN & SDO_XM) : !!(SDO_G_POORT.IN & SDO_G);
		}
	}
	
	if (xm_or_g == SELECT_G)
		CS_G_POORT.OUTSET = CS_G;
	else
		CS_XM_POORT.OUTSET = CS_XM;
	
} /* LSM9DS0Read */


static uint16_t LSM9DS0ReadU16(uint8_t addr, uint8_t xm_or_g) {

	uint8_t data[2];
	
	LSM9DS0Read(addr, data, 2, xm_or_g);
	
	return ((uint16_t) data[1] << 8) | data[0];
	
} /* LSM9DS0ReadU16 */


float GyroGetTemp(void) {
	int16_t data = 0;

	data = CONCATU8TOU16(sRawTemperatureData);

	return data * LSM9DS0_TEMP_DEG_PER_LSB;
}


float GyroGetMagnetic(uint8_t X_Y_Z) {
	int16_t data;
	
	if(X_Y_Z > GET_Z)
		X_Y_Z = GET_Z;
	
	data = CONCATU8TOU16(sRawMagnetoData + LSM9DS0_BYTES_PER_SAMPLE * X_Y_Z);

	return (float)data * LSM9DS0_MAG_MGAUSS_4GAUSS;
}


float GyroGetAcceleration(uint8_t X_Y_Z) {
	int16_t data;
	
	if(X_Y_Z > GET_Z)
		X_Y_Z = GET_Z;
		
	data = CONCATU8TOU16(sRawAccelData + LSM9DS0_BYTES_PER_SAMPLE * X_Y_Z);
	
	return (((float)data / 1000) * SENSORS_GRAVITY_EARTH)*LSM9DS0_ACCEL_MG_LSB_4G;

}


float GyroGetGyro(uint8_t X_Y_Z) {
	int16_t data;
	
	if(X_Y_Z > GET_Z)
	X_Y_Z = GET_Z;
	
	data = CONCATU8TOU16(sRawGyroData + LSM9DS0_BYTES_PER_SAMPLE * X_Y_Z);

	return ((float)data)*LSM9DS0_GYRO_DPS_DIGIT_245DPS;
}

