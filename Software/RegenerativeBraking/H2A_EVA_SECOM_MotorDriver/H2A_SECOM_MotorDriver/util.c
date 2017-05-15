/*
 * util.c
 *
 * Created: 2/23/2013 5:00:34 PM
 *  Author: bakker
 */ 

#include <stdio.h>
#include <stddef.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>

#include "util.h"

#define SLAVE_ADDRESS_H2A	3
#define SLAVE_ADDRESS_EVA_L	5
#define SLAVE_ADDRESS_EVA_R	6
#define SLAVE_ADDRESS_EVA_M	7

#define PROCID_LEN 11

static uint8_t sProcID[PROCID_LEN];

void InitUtil(void) {

	PORTD.DIRCLR = BOARDID_BIT1 | BOARDID_BIT0;

	PORTD.PIN4CTRL = PORT_OPC_PULLDOWN_gc;
	PORTD.PIN5CTRL = PORT_OPC_PULLDOWN_gc;

	/* Read and save processor serial #. Source: http://www.avrfreaks.net/forum/tutc-introduction-offsetof-reading-xmega-sn */
	(void) ReadCalibrationByte( offsetof( NVM_PROD_SIGNATURES_t, LOTNUM0 ) ); 
	/* First read after reset or possibly Power Up returns zero, so read then toss this value */ 

	sProcID[0] = ReadCalibrationByte( offsetof( NVM_PROD_SIGNATURES_t, LOTNUM0 ) ) ;
	sProcID[1] = ReadCalibrationByte( offsetof( NVM_PROD_SIGNATURES_t, LOTNUM1 ) ) ;
	sProcID[2] = ReadCalibrationByte( offsetof( NVM_PROD_SIGNATURES_t, LOTNUM2 ) ) ;
	sProcID[3] = ReadCalibrationByte( offsetof( NVM_PROD_SIGNATURES_t, LOTNUM3 ) ) ;
	sProcID[4] = ReadCalibrationByte( offsetof( NVM_PROD_SIGNATURES_t, LOTNUM4 ) ) ;
	sProcID[5] = ReadCalibrationByte( offsetof( NVM_PROD_SIGNATURES_t, LOTNUM5 ) ) ;
	sProcID[6] = ReadCalibrationByte( offsetof( NVM_PROD_SIGNATURES_t, WAFNUM ) ); 
	sProcID[7] = ReadCalibrationByte( offsetof( NVM_PROD_SIGNATURES_t, COORDX0 ) ); 
	sProcID[8] = ReadCalibrationByte( offsetof( NVM_PROD_SIGNATURES_t, COORDX1 ) ); 
	sProcID[9] = ReadCalibrationByte( offsetof( NVM_PROD_SIGNATURES_t, COORDY0 ) ); 
	sProcID[10] = ReadCalibrationByte( offsetof( NVM_PROD_SIGNATURES_t, COORDY1 ) );	

} /* InitUtil */


uint8_t GetBusID(void) {

	switch(BOARDID) {
		case BOARDID_H2A:
			return SLAVE_ADDRESS_H2A;
			break;
		case BOARDID_EVA_L:
			return SLAVE_ADDRESS_EVA_L;
			break;
		case BOARDID_EVA_R:
			return SLAVE_ADDRESS_EVA_R;
			break;
		case BOARDID_EVA_M:
		default:
			return SLAVE_ADDRESS_EVA_M;
			break;
	}
	
} /* GetBusID */


void PrintBoardType(FILE *fp) {
	
	switch(BOARDID) {
		case BOARDID_H2A:
			fprintf(fp, "H2A");
			break;
		case BOARDID_EVA_L:
			fprintf(fp, "EVA Left");
			break;
		case BOARDID_EVA_R:
			fprintf(fp, "EVA Right");
			break;
		case BOARDID_EVA_M:
			fprintf(fp, "EVA Measurement");
			break;
		default:
			fprintf(fp, "unknown");
			break;
	}
	
} /* PrintBoardType */


void PrintProcessorID(FILE *fp) {
	
	int8_t i;
	
	fprintf(fp, "cpu %02X%02X%02X%02X", MCU.DEVID0, MCU.DEVID1, MCU.DEVID2, MCU.REVID);
	for(i = 0; i < PROCID_LEN; i++)
		fprintf(fp, "%02X", sProcID[i]);
	
} /* PrintProcessorID */


uint8_t TryReadEEPROM(void *dest, const void *eesrc, size_t sz) {
	
	uint8_t buf[EE_MAX_ELEMENT_SIZE];
	int8_t i, err = 0;
	
	eeprom_read_block((void *) buf, eesrc, 2 * sz);
	for(i = 0; i < sz && !err; i++)
		if(buf[i] != (uint8_t) ~buf[sz + i]) {
			err = 1;
		}
	
	if(!err) {
		for(i = 0; i < sz; i++)
			((uint8_t *) dest)[i] = buf[i];
	}
			
	return err;
	
} /* TryReadEEPROM */


void UpdateEEPROM(const void *src, void *eedest, size_t sz) {

	uint8_t buf[EE_MAX_ELEMENT_SIZE];
	int8_t i;
	
	for(i = 0; i < sz; i++) {
		buf[i] = ((const uint8_t *) src)[i];
		buf[i + sz] = ~((const uint8_t *) src)[i];
	}
	eeprom_update_block((const void *) buf, eedest, sz * 2);

} /* UpdateEEPROM */


void FinishEEPROMOperations(void) {
	
	eeprom_busy_wait();
	
} /* FinishEEPROMOperations */


uint8_t ReadCalibrationByte(uint8_t index) {
	uint8_t result;
	
	NVM.CMD = NVM_CMD_READ_CALIB_ROW_gc;
	result = pgm_read_byte(index);
	NVM.CMD = NVM_CMD_NO_OPERATION_gc;
	
	return result;
} /* ReadCalibrationByte */
