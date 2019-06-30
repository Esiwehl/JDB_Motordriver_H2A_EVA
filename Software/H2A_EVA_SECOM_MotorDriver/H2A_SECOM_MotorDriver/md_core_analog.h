/*
 * md_core_analog.h
 *
 * Created: 11/30/2012 4:32:56 PM
 *  Author: bakker
 */ 


#ifndef MD_CORE_ANALOG_H_
#define MD_CORE_ANALOG_H_

#define CYCLES_PER_SECOND 5000

/* Simple fast linear congruential RNG, from http://www.cse.yorku.ca/~oz/marsaglia-rng.html */
#define DO_LC_RNG(x) do { x = (uint32_t) 69069 * (x) + (uint32_t) 1234567; } while(0)

/* Simple first-order filters */
#define FILTER32(newData,filt) do { filt -= filt >> 8; filt += ((int32_t) (newData)) << 8; } while(0)
#define FILTER32SQ(newData,filt) do { filt -= filt >> 8; filt += ((int32_t) (newData)) * ((int32_t) (newData)); } while(0)
#define FILTER32PWR(newData,filt) do { filt -= filt >> 8; filt += ((int32_t) (newData)); } while(0)
#define FILTER32_SHIFT10_DITHER(newData,filt) do {	\
	static uint32_t rng;							\
	DO_LC_RNG(rng);									\
	filt += (((int32_t) (newData)) - (((int32_t) filt) >> 5) + (((int32_t) rng) >> 21)) >> 5; } while(0)

void InitCoreAnalog(void);
void CalibrateChannel(FILE *fp);
void receiveSpeedfromDebug(FILE *fp);
float GetSpeedfromMotorVoltage(void);
void TakeSnapshot(void);
uint8_t IsSnapshotDone(void);
uint32_t GetSessionCycleCount(void);
void PrintCSV_H2A(FILE *fp);
void PrintCSV_EVA(FILE *fp);
void PrintEVA(FILE *fp);
void PrintResetHeader(FILE *fp);

#endif /* MD_CORE_ANALOG_H_ */