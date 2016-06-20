/*
 * md_fccomm.c
 *
 * Created: 8-5-2013 20:47:17
 *  Author: bakjd
 */ 

#include "md_serial.h"
#include "md_fccomm.h"
#include "DataInPrivate.h"

#define ZBT_MESSAGESIZE 26


typedef struct {
	uint8_t wd, st, err;
	uint16_t temp;
	uint16_t v;
	uint16_t i;
	uint16_t p;
	uint16_t flow;
	uint32_t time;
	uint32_t zbtBytesSkipped, zbtValidMessages;
} tZBTData;


static uint8_t sFCDataBuffer[ZBT_MESSAGESIZE];
static uint8_t sFCDataCount;
static tZBTData sFCDecodedData, sFCDecodedDataSnapshot, sFCDecodedDataSnapshot485;


static int GetFCTemperature(const char *subadress, char *printbuf, int maxChars);
static int FCTakeSnapshot485(const char *subadress);

static uint8_t IsValidZBTMessage(uint8_t *buffer);
void DecodeZBTMessage(uint8_t *raw, tZBTData *decoded);


void InitFCComm(void) {
	
	AddSlaveOwnSensor("TB01", GetFCTemperature, FCTakeSnapshot485, 1);
	
} /* InitFCComm */


void ProcessFCComm(void) {
	uint8_t i;
	
	while(CanRead_FC()) {
		sFCDataBuffer[sFCDataCount++] = ReadByte_FC();
		if(sFCDataCount >= ZBT_MESSAGESIZE) {
			if(IsValidZBTMessage(sFCDataBuffer)) {
				DecodeZBTMessage(sFCDataBuffer, &sFCDecodedData);
				sFCDecodedData.zbtValidMessages++;
				sFCDataCount = 0;
			}
			else {
				for(i = 1; i < ZBT_MESSAGESIZE; i++) {
					sFCDataBuffer[i-1] = sFCDataBuffer[i];
				}
				sFCDecodedData.zbtBytesSkipped++;
				sFCDataCount--;
			}
		}		
	}
} /* ProcessFCComm */


void FCTakeSnapshot(void) {

	sFCDecodedDataSnapshot = sFCDecodedData;

} /* FCTakeSnapshot */


void FCPrintDataCSV(FILE *fp) {
	
	fprintf(fp,"%u,%u,%u,%u,%u,%u,%u,%u,%lu,%lu,%lu,",
		(uint16_t) sFCDecodedDataSnapshot.wd,
		(uint16_t) sFCDecodedDataSnapshot.st,
		(uint16_t) sFCDecodedDataSnapshot.err,
		sFCDecodedDataSnapshot.temp,
		sFCDecodedDataSnapshot.v,
		sFCDecodedDataSnapshot.i,
		sFCDecodedDataSnapshot.p,
		sFCDecodedDataSnapshot.flow,
		sFCDecodedDataSnapshot.time,
		sFCDecodedDataSnapshot.zbtBytesSkipped,
		sFCDecodedDataSnapshot.zbtValidMessages);
		
} /* FCPrintDataCSV */


static int FCTakeSnapshot485(const char *subadress) {

	sFCDecodedDataSnapshot485 = sFCDecodedData;
	return 0;

} /* FCTakeSnapshot485 */


static int GetFCTemperature(const char *subadress, char *printbuf, int maxChars) {
	int err = 1;

	if(snprintf(printbuf, maxChars, "%.2f", sFCDecodedDataSnapshot.temp / 100.0) >= maxChars)
		printbuf[0] = '\0';
	else
		err = 0;
	
	return err;
} /* GetFCTemperature */


static uint8_t IsValidZBTMessage(uint8_t *buffer) {

	return (buffer[0] == 104)
	&& (buffer[1] == 19)
	&& (buffer[2] == 19)
	&& (buffer[3] == 104)
	&& (buffer[4] == 19)
	&& (buffer[ZBT_MESSAGESIZE - 1] == 22);

	/* maybe a checksum check here too? */

} /* IsValidZBTMessage */


void DecodeZBTMessage(uint8_t *raw, tZBTData *decoded) {
	
	decoded->wd = raw[5];
	decoded->st = raw[6];
	decoded->err = raw[7];
	
	decoded->temp = ((uint16_t) raw[10]) * 256 | ((uint16_t) raw[11]);
	decoded->v = ((uint16_t) raw[12]) * 256 | ((uint16_t) raw[13]);
	decoded->i = ((uint16_t) raw[14]) * 256 | ((uint16_t) raw[15]);
	decoded->p = ((uint16_t) raw[16]) * 256 | ((uint16_t) raw[17]);
	decoded->flow = ((uint16_t) raw[18]) * 256 | ((uint16_t) raw[19]);
	
	decoded->time = ((uint32_t) raw[20]) << 24u | ((uint32_t) raw[21]) << 16u | ((uint32_t) raw[22]) << 8u | raw[23];
	
} /* DecodeZBTMessage */