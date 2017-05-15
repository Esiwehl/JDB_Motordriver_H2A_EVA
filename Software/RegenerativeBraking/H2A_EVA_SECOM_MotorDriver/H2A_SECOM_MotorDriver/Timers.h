/*
 * Timers.h
 *
 * Created: 28-1-2017 17:04:05
 *  Author: Willem van der Kooij
 */ 


#ifndef TIMERS_H_
#define TIMERS_H_

//waveform generation mode
#define TC_WGMODE_NORMAL_gc			(0x00 << 0) //normal mode
#define TC_WGMODE_FRQ_gc			(0x01 << 0) //frequency mode
#define TC_WGMODE_SINGLESLOPE_gc	(0x03 << 0) //single slope mode
#define TC_WGMODE_SS_gc				(0x03 << 0) //single slope mode
#define TC_WGMODE_DSTOP_gc			(0x05 << 0) //dual slope, update on top
#define TC_WGMODE_DS_T_gc			(0x05 << 0) //dual slope, update on top
#define TC_WGMODE_DSBOTH_gc			(0x06 << 0) //dual slope, update on both
#define TC_WGMODE_DS_TB_gc			(0x06 << 0) //dual slope, update on both
#define TC_WGMODE_DSBOTTOM_gc		(0x07 << 0) //dual slope, update on bottom
#define TC_WGMODE_DS_B_gc			(0x07 << 0) //dual slope, update on bottom

//clock selection
#define TC_CLKSEL_OFF_gc			(0x00 << 0) //timer off
#define TC_CLKSEL_DIV1_gc			(0x01 << 0) //prescaler 1
#define TC_CLKSEL_DIV2_gc			(0x02 << 0) //prescaler 2
#define TC_CLKSEL_DIV4_gc			(0x03 << 0) //prescaler 4
#define TC_CLKSEL_DIV8_gc			(0x04 << 0) //prescaler 8
#define TC_CLKSEL_DIV64_gc			(0x05 << 0) //prescaler 64
#define TC_CLKSEL_DIV256_gc			(0x06 << 0) //prescaler 256
#define TC_CLKSEL_DIV1024_gc		(0x07 << 0) //prescaler 1024
#define TC_CLKSEL_EVCH0_gc			(0x08<<0)	/* Event Channel 0 */
#define TC_CLKSEL_EVCH1_gc			(0x09<<0)	/* Event Channel 1 */
#define TC_CLKSEL_EVCH2_gc			(0x0A<<0)	/* Event Channel 2 */
#define TC_CLKSEL_EVCH3_gc			(0x0B<<0)	/* Event Channel 3 */
#define TC_CLKSEL_EVCH4_gc			(0x0C<<0)	/* Event Channel 4 */
#define TC_CLKSEL_EVCH5_gc			(0x0D<<0)	/* Event Channel 5 */
#define TC_CLKSEL_EVCH6_gc			(0x0E<<0)	/* Event Channel 6 */
    #define TC_CLKSEL_EVCH7_gc		(0x0F<<0)	/* Event Channel 7 */

//error interrupt level
#define TC_ERRINTLVL_OFF_gc			(0x00<<2)  /* Interrupt Disabled */
#define TC_ERRINTLVL_LO_gc			(0x01<<2)  /* Low Level */
#define TC_ERRINTLVL_MED_gc			(0x02<<2)  /* Medium Level */
#define TC_ERRINTLVL_HI_gc			(0x03<<2)  /* High Level */

//overflow interrupt level
#define TC_OVFINTLVL_OFF_gc			(0x00 << 0) //interrupt disabled
#define TC_OVFINTLVL_LO_gc			(0x01 << 0) //interrupt low level
#define TC_OVFINTLVL_MED_gc			(0x02 << 0) //interrupt medium level
#define TC_OVFINTLVL_HI_gc			(0x03 << 0) //interrupt high level

//compare or capture D interrupt level
#define TC_CCDINTLVL_OFF_gc			(0x00<<6)  /* Interrupt Disabled */
#define TC_CCDINTLVL_LO_gc			(0x01<<6)  /* Low Level */
#define TC_CCDINTLVL_MED_gc			(0x02<<6)  /* Medium Level */
#define TC_CCDINTLVL_HI_gc			(0x03<<6)  /* High Level */

//compare or capture C interrupt level
#define TC_CCCINTLVL_OFF_gc			(0x00<<4)  /* Interrupt Disabled */
#define TC_CCCINTLVL_LO_gc			(0x01<<4)  /* Low Level */
#define TC_CCCINTLVL_MED_gc			(0x02<<4)  /* Medium Level */
#define TC_CCCINTLVL_HI_gc			(0x03<<4)  /* High Level */

//compare or capture B interrupt level
#define TC_CCBINTLVL_OFF_gc			(0x00<<2)  /* Interrupt Disabled */
#define TC_CCBINTLVL_LO_gc			(0x01<<2)  /* Low Level */
#define TC_CCBINTLVL_MED_gc			(0x02<<2)  /* Medium Level */
#define	TC_CCBINTLVL_HI_gc			(0x03<<2)  /* High Level */

//compare or capture A interrupt level
#define TC_CCAINTLVL_OFF_gc			(0x00<<0)  /* Interrupt Disabled */
#define TC_CCAINTLVL_LO_gc			(0x01<<0)  /* Low Level */
#define TC_CCAINTLVL_MED_gc			(0x02<<0)  /* Medium Level */
#define TC_CCAINTLVL_HI_gc			(0x03<<0)  /* High Level */

//Timer/Counter command
#define TC_CMD_NONE_gc				(0x00<<2)  /* No Command */
#define TC_CMD_UPDATE_gc			(0x01<<2)  /* Force Update */
#define TC_CMD_RESTART_g			(0x02<<2)  /* Force Restart */
#define TC_CMD_RESET_gc				(0x03<<2)  /* Force Hard Reset */

//Timer/Counter Event Action
#define TC_EVACT_OFF_gc				(0x00 << 5) //no event action
#define TC_EVACT_CAPT_gc			(0x01 << 5) //input capture
#define TC_EVACT_UPDOWN_gc			(0x02 << 5) //externally controlled up/down count
#define TC_EVACT_QDEC_gc			(0x03 << 5) //quadrature decode
#define TC_EVACT_RESTART_gc			(0x04 << 5) //restart
#define TC_EVACT_FRQ_gc				(0x05 << 5) //frequency capture
#define TC_EVACT_PW_gc				(0x06 << 5) //pulse-width capture

//Timer/Counter Byte Mode
#define TC_BYTEM_NORMAL_gc			(0x00<<0)  /* 16-bit mode */
#define TC_BYTEM_BYTEMODE_gc		(0x01<<0)  /* Timer/Counter operating in byte mode only */
#define TC_BYTEM_SPLITMODE_gc		(0x02<<0)  /* Timer/Counter split into two 8-bit Counters (TC2) */


//Timer/Counter Event Selection
#define TC_EVSEL_OFF_gc  (0x00<<0)  /* No Event Source */
#define TC_EVSEL_CH0_gc  (0x08<<0)  /* Event Channel 0 */
#define TC_EVSEL_CH1_gc  (0x09<<0)  /* Event Channel 1 */
#define TC_EVSEL_CH2_gc  (0x0A<<0)  /* Event Channel 2 */
#define TC_EVSEL_CH3_gc  (0x0B<<0)  /* Event Channel 3 */
#define TC_EVSEL_CH4_gc  (0x0C<<0)  /* Event Channel 4 */
#define TC_EVSEL_CH5_gc  (0x0D<<0)  /* Event Channel 5 */
#define TC_EVSEL_CH6_gc  (0x0E<<0)  /* Event Channel 6 */
#define TC_EVSEL_CH7_gc  (0x0F<<0)  /* Event Channel 7 */

#endif /* TIMERS_H_ */