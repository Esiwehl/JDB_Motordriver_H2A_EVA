#ifndef __DATA_IN_PRIVATE_H__
#define __DATA_IN_PRIVATE_H__

//Positions
#define KARAKTERSOORTZIN        0
#define COMMAND                 4
#define ADRESSLAVE_POS          1
#define INDEX_SUBADR_POS        5
#define SENSADR_POS             8
#define SETDATA_POS             12

//Length

#define MAXCHARACTERSSENTENCE   83
#define MAXSIZEDATA             10
#define MAXNUMBERSOFSLAVES      3
#define SENSOR_NAMELENGTE       4
#define MAXAANTALKEERVERZENDEN  3
#define MAXDATA                 78 // without '%CH\n\r'
#define SUBADR_LENG             5

//Characters
#define SLAVESENTENCE           '$'
#define MASTERSENTENCE			'#'
#define CHECKSUMSIGN            '%'
#define COMMENT                 0x27
#define BROADCAST                0


int AddSlaveOwnSensor (const char *subadress, int (*fpGetData)(const char*,char*,int),int(*fpTakeSnapshot)(const char*),int measure);
int AddWantedSensor (const char *subadress, int (*fpSetData)(const char*, char*));
void freeSlave ();
uint8_t ScanDataInSlave(char* dataFromMaster);
void InitRS485();
void GetData();
void InitSlave (int adr);
void SendLineRS485(char* data);
uint8_t ReadLineRS45( char *Data );




#endif // __DATA_IN_PRIVATE_H__



