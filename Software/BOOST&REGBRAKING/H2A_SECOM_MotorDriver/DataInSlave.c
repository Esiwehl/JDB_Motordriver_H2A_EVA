/*
 * DataInSlave.c
 *
 * Created: 11-4-2013 14:54:30
 *  Author: Khaoula
 */

#define F_CPU 32000000UL

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <avr/io.h>

#include "DataInPrivate.h"
#include "md_serial.h"
//#include "md_timer.h"


typedef struct {
	char subadress[SUBADR_LENG];
	int measure_interval;
	int (*fpGetData)(const char*, char*, int);
	//the value of the sensor
	int (*fpTakeSnapshot)(const char*);
	//Get the last value of the sensor
} tOwnSensor;

typedef struct {
	char subadress[SUBADR_LENG];
	int (*fpSetData)(const char*, char*);
	//Get the last value of the sensor
} tWantedSensor;

/* Static variables, private to this slave */
static int sMyAdress;
static int sNumOwnSensor ;
static int sNumWantedSensor;
static tOwnSensor *sOwnSensor;
static tWantedSensor *sWantedSensor;

static int GetIndexFromMaster (char* dataMaster);
static int FindOwnSensorIndex(char *adressSensor);
static int FindWantedSensorIndex(char *adressSensor);
static void HandleCmdA(char* response);
static void HandleCmdI(char* response, char* dataFromMaster);
static void HandleCmdW(char* response, char* dataFromMaster);
static void HandleCmdG(char* response, char* dataFromMaster);
static void HandleCmdS(char* response, char* dataFromMaster);
static uint8_t DataChecksum(char *dataMaster);
static int CheckChecksum(char* dataMaster);
static int GetAdressFromMaster(char* dataMaster);

void HandleSnapshot();

static uint8_t FindSensorOK( char *subadres, char *sensor );

/** \brief Initialize the slave elements
 *
 * \param 1 adr: the address of the slave
 *
 */

void InitSlave (int adr){
    sMyAdress = adr;
    sNumOwnSensor = 0;
    sNumWantedSensor = 0;
    sOwnSensor = NULL;
    sWantedSensor = NULL;
}

/** \brief Add a sensor to the slave
 *
 * \param 1 subadress: the sub address of the sensor like "TM01","TZ03": take a look at the protocol specifications
 * \param 2 (*fpGetData): the function pointer to get the value of the sensor & eventuel comments
 * \param 3 (*fpTakeSnapshot):
 * \param 4 measure: How often the value of the sensor can be given
 *
 * \return 0 : if there is no fault while adding a sensor
 *         1 : if the buffer of the info is full
 *		     : if there is no more available memory
 *		     : if the function pointer does not match with the sub address of the sensor
 */
int AddSlaveOwnSensor (const char *subadress, int (*fpGetData)(const char*,char*,int),int(*fpTakeSnapshot)(const char*),int measure) {

    int status= 0;

    sOwnSensor = (tOwnSensor *) realloc (sOwnSensor, (sNumOwnSensor + 1) * sizeof(tOwnSensor));
    if(sOwnSensor != NULL){
        strncpy((sOwnSensor)[sNumOwnSensor].subadress,subadress, SUBADR_LENG - 1);
        sOwnSensor[sNumOwnSensor].subadress[SUBADR_LENG - 1] = '\0';
		sOwnSensor[sNumOwnSensor].fpTakeSnapshot = fpTakeSnapshot;
		sOwnSensor[sNumOwnSensor].fpGetData= fpGetData;
		sOwnSensor[sNumOwnSensor].measure_interval = measure;
		sNumOwnSensor++;
	}

	else{
		// led blink
		// printf("\nReset, Memory full\n");
		status = 1;
    }

    return status;
}

/** \brief Add a wanted sensor to the slave
 *
 * \param 1 subadress: the sub address of the sensor like "TM01","TZ03": take a look at the protocol specifications
 * \param 2 (*fpSetData): the function pointer to register the value of the sensor
 *
 * \return 0 : if there is no fault while adding a sensor
 *         1 : if there is no more available memory
			 : if the function pointer does not match with the sub address of the sensor
 */
int AddWantedSensor (const char *subadress, int (*fpSetData)(const char*,char*)){
	int status = 0;

		sWantedSensor = (tWantedSensor *) realloc (sWantedSensor, ((sNumWantedSensor) + 1) * sizeof(tWantedSensor));
		if((sWantedSensor)!= NULL){
		   strncpy(sWantedSensor[sNumWantedSensor].subadress, subadress, SUBADR_LENG - 1);
		  sWantedSensor[sNumWantedSensor].subadress[SUBADR_LENG - 1] = '\0';
		  sWantedSensor[sNumWantedSensor].fpSetData = fpSetData;
		  sNumWantedSensor++;
		}
		else {
			// led blink
			//printf("Reset, Memory full\n");
			return 1;
		}
	return status;
}


/** \brief Free the used memory
 *
 */
void freeSlave (){

	free(sOwnSensor);
	free(sWantedSensor);
}


/** \brief Give the index from the master's data in integer
 *
 * \param dataMaster:  the data from the master
 *
 * \return int: the index
 */
static int GetIndexFromMaster (char* dataFromMaster){
    return strtol(dataFromMaster + INDEX_SUBADR_POS,NULL,10);
}


/** \brief Search if the subadress of the sensor already exists in the list of the slave's own sensors
 *
 * \param 1 adressSensor : the given sub-address of the sensor
 *
 * \return position : the position of the sensor if it is found
 *					 -1 if sensor is not found
 */

static int FindOwnSensorIndex(char *adressSensor){
	int i=0;
	int position = -1;

	for(i=0; i < sNumOwnSensor; i++){
		if(!strncmp(adressSensor,sOwnSensor[i].subadress,strlen(adressSensor))){
			position = i;
		}
	}
	return position;
}

/** \brief Search if the subadress of the wanted sensor already exists in the list of the slave's wanted sensors
 *
 * \param 1 adressSensor : the given sub-address of the sensor
 *
 * \return position : the position of the sensor if it is found
 *					 -1 if sensor is not found
 */

static int FindWantedSensorIndex(char *adressSensor){
	int i=0;
	int position = -1;

	for(i=0; i<sNumWantedSensor;i++){
		if(/*!strncmp(adressSensor,sWantedSensor[i].subadress,SUBADR_LENG-1)*/ FindSensorOK( adressSensor, sWantedSensor[i].subadress )){

			position = i;
		}
	}
	return position;
}

/** \brief Handling of the Available command
 *
 * \param 1 response : the buffer where the response is saved
 *
 */
static void HandleCmdA(char* response){

    snprintf(response,MAXDATA-1,"$%03dA'Slave available'",sMyAdress);
}

/** \brief Handling of the Inventory command
 *
 * \param 1 response      :  the buffer where the response is saved
 * \param 2 dataFromMaster: the received message from the master
 *
 */
static void HandleCmdI(char* response, char* dataFromMaster){
	static int index =0;

	index = GetIndexFromMaster(dataFromMaster);

    if(index > 0 && index <= sNumOwnSensor) {
        snprintf(response,MAXDATA-1,"$%03dI%02d%s,%02d'Sensor %d'",sMyAdress,index,sOwnSensor[index-1].subadress,sOwnSensor[index-1].measure_interval,index);
    }
// the slave sends 0000,0000 if all the sensors that exists are already given
    else{
        sprintf(response,"$%03dI%02d0000,0000'No sensor'", sMyAdress,index);
    }
}

/** \brief Handling of the WhatDoYouWant command
 *
 * \param 1 response      :  the buffer where the response is saved
 * \param 2 dataFromMaster: the received message from the master
 *
 */
static void HandleCmdW(char* response, char* dataFromMaster){

    int index = GetIndexFromMaster(dataFromMaster);

    if(index > 0 && index <= sNumWantedSensor){
        snprintf(response,MAXDATA-1,"$%03dW%02d%s'Wants slave'",sMyAdress,index,sWantedSensor[index-1].subadress);
    }
// the slave sends 0000 if all sensor that exists in the buffer are given
    else {
       snprintf(response,MAXDATA-1,"$%03dW%02d0000'All done'", sMyAdress,index);
    }
}

/** \brief Handling of the Get command
 *
 * \param 1 response      :  the buffer where the response is saved
 * \param 2 dataFromMaster: the received message from the master
 *
 */
static void HandleCmdG(char* response, char* dataFromMaster){
    int sensorIndex, err;
    char adr[SUBADR_LENG];

    strncpy(adr,dataFromMaster + INDEX_SUBADR_POS, SUBADR_LENG-1);
	adr[SUBADR_LENG-1] = '\0';

    sensorIndex = FindOwnSensorIndex(adr);

    if(sensorIndex !=-1){
		//sensorIndex is -1 if sensor not found
        sprintf(response,"$%03dG",sMyAdress);
        err= sOwnSensor[sensorIndex].fpGetData(adr,response + strlen(response), MAXDATA-1-strlen(response) );
        if(err){
			sprintf(response,"$%03dGF'Memory fault'",sMyAdress);
		}
    }
    else{
        sprintf(response,"$%03dGF'Sensor not found'",sMyAdress);
    }
}

//update value of sensors of sensors of a slave
void HandleSnapshot(){

	int sensorIndex;

	for(sensorIndex=0; sensorIndex < sNumOwnSensor; sensorIndex++){
		if(sOwnSensor[sensorIndex].fpTakeSnapshot != NULL)
			sOwnSensor[sensorIndex].fpTakeSnapshot(sOwnSensor[sensorIndex].subadress);
	}
}


/** \brief Handling of the Set command
 *
 * \param 1 response      :  the buffer where the response is saved
 * \param 2 dataFromMaster: the received message from the master
 *
 */
static void HandleCmdS(char* response, char* dataFromMaster){
	int position=0, err =0;
	char adr[SUBADR_LENG];

	strncpy(adr,dataFromMaster + SENSADR_POS, SUBADR_LENG-1);
	adr[SUBADR_LENG-1] = '\0';

	if(strncmp(adr,"SN00",SUBADR_LENG-1)==0){

		HandleSnapshot();
	}
	else{
		position = FindWantedSensorIndex(adr);
		if(position != -1){
			err = (*sWantedSensor[position].fpSetData)(adr,dataFromMaster + SETDATA_POS);

			if(err==0){
				snprintf(response,MAXDATA-1,"$%03dSOK",sMyAdress);
			}
			else{
				snprintf(response,MAXDATA-strlen(response),"$%03dSF'Fault'",sMyAdress);
			}
		}
		else{
			snprintf(response,MAXDATA-strlen(response),"$%03dSF'Fault wrong sensor'",sMyAdress);
		}
	}
}


/** \brief Checksum handling/calculating
 *
 * \param 1 dataMaster: data from master of from the slave
 *
 * \return checksum value
 */

static uint8_t DataChecksum(char *dataMaster){
    uint8_t sChecksum = 0;
	dataMaster++;
    while(*dataMaster != CHECKSUMSIGN && *dataMaster != '\0'){
		sChecksum^=*dataMaster++;
	}
    return sChecksum;
}

/** \brief Checking the checksum
 *
 * \param 1 dataMaster : data from master
 *
 * \return 1 if the checksum is correct
 *		   0  if there is a fault in the checksum
 */
static int CheckChecksum(char* dataMaster){
    uint8_t checksum = 0;
	char *positionOfPercent;

    checksum = DataChecksum(dataMaster);
    //check if the the sentence begins with '#' or '$'
	if ((dataMaster[0] == MASTERSENTENCE) || (dataMaster[0] == SLAVESENTENCE)){
	    //searching for '%' and return the position of it in the sentence
	  if((positionOfPercent = strchr(dataMaster,CHECKSUMSIGN))!= NULL){
        //place the checksum found in the variable checksum
        //check if the two characters of the checksum are valid
	    if (checksum == strtol(positionOfPercent + 1,NULL,16) && (isxdigit(positionOfPercent[1])) && (isxdigit(positionOfPercent[2]))){
		    return 1;
	    }
	  }
	}

	return 0;
}

/** \brief  Gives the adress of the slave from the master in integer
 *
 * \param 1 dataMaster: data from the master
 * \return adress of the slave in integer
 *
 */
static int GetAdressFromMaster (char* dataMaster){

   return strtol(dataMaster + 1,NULL,10);
}

/** \brief The data composed by the slave is sent to the line of RS485
 *
 * \param 1 data : data from the slave
 *
 */

void SendLineRS485(char* data){

	while(*data != '\0'){
		WriteByte_Comm485(*data++);
	}
	// add \r\n to the sentence
	WriteByte_Comm485(0x0D);
	WriteByte_Comm485(0x0A);
}

/** \brief Read the data from the line of RS485
 *
 * \param 1 data : data from the master
 *
 */

uint8_t ReadLineRS485( char *Data ){
	static char getKarakter;
	uint8_t len, returnValue = 0;

	while( CanRead_Comm485() != 0 && returnValue == 0 ){
		getKarakter = (char )ReadByte_Comm485();
		if( getKarakter == SLAVESENTENCE || getKarakter == MASTERSENTENCE ){
			Data[0] = getKarakter;
			Data[1] = '\0';
		}
		else if( Data[0] == SLAVESENTENCE || Data[0] == MASTERSENTENCE ){
			len = strlen(Data);
			if(len < MAXCHARACTERSSENTENCE - 1) {
				Data[len] = getKarakter;
				Data[len + 1] = '\0';
				if( getKarakter == 0x0A ){
					returnValue = 1;
				}
				else{
					returnValue = 0;
				}
			}			
		}
	}
	return returnValue;
}


/** \brief The data from the master is scanned and the slave sends the answer
 *         the answer of the slave is in responseSlave stored
 *         then the checksum is added
 *         finally is the data sent to the line of RS485
 *
 * \param 1 dataMaster : data from the master
 * \return 0: if there is a fault in the adress of the slave
 *            or the checksum of the master or the message is not from the master
 *            else
 * \return 1
 */
uint8_t ScanDataInSlave (char* dataFromMaster){
	int err = 0;
    int adress = GetAdressFromMaster(dataFromMaster);
    char cmd = dataFromMaster[COMMAND];
    static char responseSlave[MAXCHARACTERSSENTENCE];
	/*
	 * check if the first character is '#'
	 * check if the adress of the slave is equal to the slave's adress
	 * check if the adress from the master is for the broadcast ('000') or not
	 * check if the checksum  of the master's sentence is equal to the calculated checksum
	 * if one of these conditions is false then return 0
	 * else if the adress is a for the broadcast the the command must be 'SET'
	*/
    if((dataFromMaster[KARAKTERSOORTZIN] != MASTERSENTENCE) || ((adress != sMyAdress) && (adress != BROADCAST))/* || (!CheckChecksum(dataFromMaster))*/){
       err = 0;
    }
    else{
	 if(adress == BROADCAST) {
		if(cmd =='S'){
			HandleCmdS(responseSlave,dataFromMaster);
		}
		err =1;
	 }
	 else{

      switch(cmd){
      case 'A': HandleCmdA(responseSlave);
                break;

      case 'I': HandleCmdI(responseSlave,dataFromMaster);
                break;

      case 'W': HandleCmdW(responseSlave,dataFromMaster);
                break;

      case 'G': HandleCmdG(responseSlave,dataFromMaster);
                break;

      case 'S': HandleCmdS(responseSlave,dataFromMaster);
                break;

      default:  break;
    }
    // add '%' and the checksum to the sentence and then send it through the line of the RS485
    snprintf(responseSlave+strlen(responseSlave),MAXCHARACTERSSENTENCE-strlen(responseSlave),"%%%02X",DataChecksum(responseSlave));
    SendLineRS485(responseSlave);
    err =1;
	}
	dataFromMaster[0] = '\0';
	}
	return err;
}


/** \brief searching for the sensor using '*'
 *
 * \param 1 subadres : the adress of the sensor from the master
 * \param 1 sensor : the adress of the sensor
 *
 *\return 0 if the sensor is found else 1
 */

static uint8_t FindSensorOK( char *subadres, char *sensor ){
	uint8_t err = 0;

	if( sensor[0] == '*' || sensor[0] == subadres[0] ){
		if( sensor[1] == '*' || sensor[1] == subadres[1] ){
			if( sensor[2] == '*' || sensor[2] == subadres[2] ){
				if( sensor[3] == '*' || sensor[3] == subadres[3] ){
					err = 1;
				}
			}
		}
	}
	return err;
}
