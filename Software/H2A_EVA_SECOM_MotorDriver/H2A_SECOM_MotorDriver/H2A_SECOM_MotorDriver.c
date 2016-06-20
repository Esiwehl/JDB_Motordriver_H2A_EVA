/*
 * H2A_SECOM_MotorDriver.c
 *
 * Created: 4/25/2013 1:00:12 AM
 *  Author: bakker
 */ 


#define F_CPU 32000000UL  // 32 MHz


#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>

#include "md_core_analog.h"
#include "md_serial.h"
#include "md_ticktimer.h"
#include "md_fccomm.h"
#include "clksys_driver.h"
#include "md_readbussensors.h"
#include "DataInPrivate.h"
#include "util.h"


static void InitClocks(void);
static void InitIO(void);

#define PRINTCSV_LINELEN 150 /* Approximate line length of the PrintCSV() call. 
								No point in generating a new line if there are not at least this many bytes free */

#define DEBUGPRINT_START	0
#define DEBUGPRINT_ANALOG	1
#define DEBUGPRINT_FC		2
#define DEBUGPRINT_BUS		3
#define DEBUGPRINT_DONE		4

#define ESC_TIMEOUT			(CYCLES_PER_SECOND)
#define PRINT_ID_INTERVAL	((uint32_t)10*60*(CYCLES_PER_SECOND))

int main(void)
{
	uint32_t prev = 0, prevIDPrint = 0, now;
	uint8_t debugPrintstate = DEBUGPRINT_START, escTimeoutActive = 0;
	static char slaveData[MAXCHARACTERSSENTENCE];
	
	InitClocks();
	InitIO();
	InitUtil();
	InitTimer();
	InitSerial();
	InitSlave(GetBusID());
	InitCoreAnalog();
	if(I_AM_H2A)
		InitFCComm();
	InitReadBussensors();
	
	sei();
	
	PrintResetHeader(&gCtrl_IO);
	while(1) {
		ProcessFCComm();
		
		if(CanRead_Ctrl()) {
			switch(ReadByte_Ctrl()) {
				case 0x1B:
					/* ESC -- sync character for avrdude. The programmer is trying to talk to us, so reset the chip */
					CCPWrite( &RST.CTRL, RST_SWRST_bm );
					break;
				case 'c':
				case 'C':
					CalibrateChannel(&gCtrl_IO);
					break;
			}
			escTimeoutActive = 1;
			prev = GetSessionCycleCount();
		}
		now = GetSessionCycleCount();
		if(escTimeoutActive) {
			if(now - prev >= ESC_TIMEOUT)
				escTimeoutActive = 0;
		}

		/* Dump data on the serial (debug) port */
		if(!escTimeoutActive && CanWrite_Ctrl() >= PRINTCSV_LINELEN) {
			switch(debugPrintstate) {
				case DEBUGPRINT_START:
					if(now - prevIDPrint >= PRINT_ID_INTERVAL) {
						prevIDPrint += PRINT_ID_INTERVAL;
						PrintResetHeader(&gCtrl_IO);
					}
					else {
						TakeSnapshot();
 						if(I_AM_H2A)
 							FCTakeSnapshot();
						debugPrintstate = DEBUGPRINT_ANALOG;
					}
					break;
				case DEBUGPRINT_ANALOG:
					if(I_AM_H2A) {
						PrintCSV_H2A(&gCtrl_IO);
						debugPrintstate = DEBUGPRINT_FC;
					}
					else if(I_AM_EVA_M) {
						PrintCSV_H2A(&gCtrl_IO);
						debugPrintstate = DEBUGPRINT_BUS;
					}
					else {
						PrintCSV_EVA(&gCtrl_IO);
						debugPrintstate = DEBUGPRINT_BUS;
					}
					break;
				case DEBUGPRINT_FC:
					FCPrintDataCSV(&gCtrl_IO);
					debugPrintstate = DEBUGPRINT_BUS;
					break;
				case DEBUGPRINT_BUS:
					PrintBussensors(&gCtrl_IO);
					debugPrintstate = DEBUGPRINT_DONE;
				case DEBUGPRINT_DONE:
					fprintf(&gCtrl_IO, "<\r\n");
				default:
					debugPrintstate = DEBUGPRINT_START;
			}
		}

		/* Handle the slave code */
		if(CanRead_Comm485())
			PORTF.OUTCLR = PIN4_bm | PIN5_bm;
		else
			PORTF.OUTSET = PIN4_bm | PIN5_bm;
		if ( ReadLineRS45( slaveData ) == 1 ){
			ScanDataInSlave(slaveData);
		}
	}
}


static void InitClocks(void)
{
	CLKSYS_XOSC_Config( OSC_FRQRANGE_12TO16_gc, 0, OSC_XOSCSEL_XTAL_16KCLK_gc );
	CLKSYS_Enable( OSC_XOSCEN_bm );
// do {} while ( CLKSYS_IsReady( OSC_XOSCRDY_bm ) == 0 );
	do {} while ( CLKSYS_IsReady( OSC_XOSCRDY_bm ) == 0 );
	CLKSYS_PLL_Config( OSC_PLLSRC_XOSC_gc, 2);
	CLKSYS_Enable( OSC_PLLEN_bm );
	do {} while ( CLKSYS_IsReady( OSC_PLLRDY_bm ) == 0 );
	CLKSYS_Main_ClockSource_Select( CLK_SCLKSEL_PLL_gc );
	CLKSYS_Disable( OSC_RC2MEN_bm );
	CLKSYS_Disable( OSC_RC32MEN_bm );
	// CLKSYS_AutoCalibration_Enable(OSC_RC32MCREF_bm, 1);
	
} /* InitClocks */


static void InitIO(void) {
	PORTC.DIRCLR = PIN0_bm | PIN1_bm | PIN2_bm | PIN3_bm | PIN4_bm | PIN5_bm | PIN6_bm;
	PORTC.PIN0CTRL = PORT_OPC_PULLDOWN_gc;
	
	PORTD.DIRCLR = PIN0_bm;
	PORTD.PIN0CTRL = PORT_OPC_PULLDOWN_gc;
		
	PORTE.DIR = PIN0_bm | PIN1_bm | PIN2_bm | PIN3_bm | PIN6_bm | PIN7_bm;
	PORTE.OUTCLR = PIN0_bm | PIN1_bm | PIN2_bm | PIN3_bm;
	
	PORTF.DIR = PIN0_bm | PIN3_bm | PIN4_bm | PIN5_bm | PIN6_bm | PIN7_bm;
	PORTF.OUTCLR = PIN3_bm | PIN4_bm | PIN5_bm;
}