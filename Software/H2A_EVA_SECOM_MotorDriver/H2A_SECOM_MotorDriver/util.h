/*
 * jc_util.h
 *
 * Created: 2/23/2013 5:00:34 PM
 *  Author: bakker
 */ 

#ifndef MD_UTIL_H_
#define MD_UTIL_H_

#define BOARDID_BIT1 PIN5_bm
#define BOARDID_BIT0 PIN4_bm
#define BOARDID_bm   (BOARDID_BIT1 | BOARDID_BIT0)

#define BOARDID_H2A		0x00
#define BOARDID_EVA_L	BOARDID_BIT0
#define BOARDID_EVA_R	BOARDID_BIT1
#define BOARDID_EVA_M	(BOARDID_BIT1 | BOARDID_BIT0) /* M = Measurement, not Middle! */

#define BOARDID (PORTD.IN & BOARDID_bm)

#define I_AM_H2A	(BOARDID == BOARDID_H2A)
#define I_AM_EVA	(!I_AM_H2A)
#define I_AM_EVA_L	(BOARDID == BOARDID_EVA_L)
#define I_AM_EVA_R	(BOARDID == BOARDID_EVA_R)
#define I_AM_EVA_M	(BOARDID == BOARDID_EVA_M)


#define EE_FC_VSCALE	((void *)0x00)
#define EE_FC_ISCALE	((void *)0x10)
#define EE_SC_VSCALE	((void *)0x20)
#define EE_SC_ISCALE	((void *)0x30)
#define EE_FC_VOFFSET	((void *)0x40)
#define EE_FC_IOFFSET	((void *)0x44)
#define EE_SC_VOFFSET	((void *)0x48)
#define EE_SC_IOFFSET	((void *)0x4C)

#define EE_M_VSCALE		((void *)0x50)
#define EE_M_ISCALE		((void *)0x60)
#define EE_IN_VSCALE	((void *)0x70)
#define EE_IN_ISCALE	((void *)0x80)
#define EE_M_VOFFSET	((void *)0x90)
#define EE_M_IOFFSET	((void *)0x94)
#define EE_IN_VOFFSET	((void *)0x98)
#define EE_IN_IOFFSET	((void *)0x9C)

#define EE_END			((void *)0xA0)

#define EE_MAX_ELEMENT_SIZE (2*sizeof(double))

void InitUtil();

uint8_t GetBusID(void);

void PrintBoardType(FILE *fp);
void PrintProcessorID(FILE *fp);

uint8_t TryReadEEPROM(void *dest, const void *eesrc, size_t sz);
void UpdateEEPROM(const void *src, void *eedest, size_t sz);
void FinishEEPROMOperations(void);

uint8_t ReadCalibrationByte(uint8_t index);

#define TRYREADEEPROM(d, s) TryReadEEPROM((void *) &(d), s, sizeof(d))
#define UPDATEEEPROM(s, d) UpdateEEPROM((void *) &(s), d, sizeof(s))

#endif