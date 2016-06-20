/*
 * md_fccomm.h
 *
 * Created: 8-5-2013 20:47:47
 *  Author: bakjd
 */ 


#ifndef MD_FCCOMM_H_
#define MD_FCCOMM_H_

void InitFCComm(void);
void ProcessFCComm(void);
void FCTakeSnapshot(void);
void FCPrintDataCSV(FILE *fp);

#endif /* MD_FCCOMM_H_ */