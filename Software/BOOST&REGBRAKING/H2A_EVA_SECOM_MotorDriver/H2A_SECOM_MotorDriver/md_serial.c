/*
 * md_serial.c
 *
 * Created: 4/24/2013 9:05:29 PM
 *  Author: bakker
 */ 


#include "md_ticktimer.h"
#include "md_serial.h"

#include <stdio.h>
#include <stddef.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>


#define UART_230K4_BSEL_VALUE		245
#define UART_230K4_BSCALE_VALUE		0x0B

#define UART_115K2_BSEL_VALUE		524
#define UART_115K2_BSCALE_VALUE		0x0B

#define UART_38K4_BSEL_VALUE		51
#define UART_38K4_BSCALE_VALUE		0

#define PD_DRIVER_ENABLE_COMM485	PIN1_bm
#define PD_UART_D0_RXD_COMM485		PIN2_bm
#define PD_UART_D0_TXD_COMM485		PIN3_bm

#define PD_UART_D1_RXD_CTRL			PIN6_bm
#define PD_UART_D1_TXD_CTRL			PIN7_bm

#define PF_UART_F0_RXD_FC			PIN2_bm
#define PF_UART_F0_TXD_FC			PIN3_bm
	
#define TXBUF_DEPTH_COMM485	180
#define RXBUF_DEPTH_COMM485 180
#define TXBUF_DEPTH_CTRL	250
#define RXBUF_DEPTH_CTRL	80
#define TXBUF_DEPTH_FC		100
#define RXBUF_DEPTH_FC		100


static int Comm485_putchar(char c, FILE *stream);
static int Ctrl_putchar(char c, FILE *stream);

FILE gComm485_IO = FDEV_SETUP_STREAM(Comm485_putchar, NULL, _FDEV_SETUP_WRITE);
FILE gCtrl_IO = FDEV_SETUP_STREAM(Ctrl_putchar, NULL, _FDEV_SETUP_WRITE);


static volatile uint8_t tx_485_wridx, tx_485_rdidx, tx_485_buf[TXBUF_DEPTH_COMM485], tx_485_busy;
static volatile uint8_t rx_485_wridx, rx_485_rdidx, rx_485_buf[RXBUF_DEPTH_COMM485];
static volatile uint8_t tx_ctrl_wridx, tx_ctrl_rdidx, tx_ctrl_buf[TXBUF_DEPTH_CTRL];
static volatile uint8_t rx_ctrl_wridx, rx_ctrl_rdidx, rx_ctrl_buf[RXBUF_DEPTH_CTRL];
static volatile uint8_t tx_fc_wridx, tx_fc_rdidx, tx_fc_buf[TXBUF_DEPTH_FC];
static volatile uint8_t rx_fc_wridx, rx_fc_rdidx, rx_fc_buf[RXBUF_DEPTH_FC];


void InitSerial(void)
{
	/* Shut down UARTs which might be left running by the bootloader */
	
	cli();
	
	USARTC0.CTRLB = 0;
 	USARTC1.CTRLB = 0;
 	USARTD0.CTRLB = 0;
 	USARTD1.CTRLB = 0;
 	USARTE0.CTRLB = 0;
 	USARTE1.CTRLB = 0;
 	USARTF0.CTRLB = 0;
	
	/* Pull-up on RX ports */
	PORTD.PIN2CTRL = PORT_OPC_PULLUP_gc;
	PORTD.PIN6CTRL = PORT_OPC_PULLUP_gc;
	PORTF.PIN2CTRL = PORT_OPC_PULLUP_gc;
	
	/* Activate ports for RS485 (USARTD0), serial comms (USARTD1) and FC comms (USARTF0), disable RS485 TX driver */
	PORTD.OUTSET = PD_UART_D0_TXD_COMM485 | PD_UART_D1_TXD_CTRL; 
	PORTD.OUTCLR = PD_DRIVER_ENABLE_COMM485;
	PORTD.DIRSET = PD_UART_D0_TXD_COMM485 | PD_DRIVER_ENABLE_COMM485 | PD_UART_D1_TXD_CTRL;
	PORTD.DIRCLR = PD_UART_D0_RXD_COMM485 | PD_UART_D1_RXD_CTRL; 
	PORTF.OUTSET = PF_UART_F0_TXD_FC;
	PORTF.DIRSET = PF_UART_F0_TXD_FC;
	PORTF.DIRCLR = PF_UART_F0_RXD_FC;

	PORTD.PIN0CTRL = PORT_OPC_PULLDOWN_gc;
	PORTD.PIN4CTRL = PORT_OPC_PULLDOWN_gc;
	PORTD.PIN5CTRL = PORT_OPC_PULLDOWN_gc;
		
	USARTD0.BAUDCTRLA = (UART_230K4_BSEL_VALUE & USART_BSEL_gm);
	USARTD0.BAUDCTRLB = ((UART_230K4_BSCALE_VALUE << USART_BSCALE_gp) & USART_BSCALE_gm) | ((UART_230K4_BSEL_VALUE >> 8) & ~USART_BSCALE_gm);
		
	USARTD1.BAUDCTRLA = (UART_230K4_BSEL_VALUE & USART_BSEL_gm);
	USARTD1.BAUDCTRLB = ((UART_230K4_BSCALE_VALUE << USART_BSCALE_gp) & USART_BSCALE_gm) | ((UART_230K4_BSEL_VALUE >> 8) & ~USART_BSCALE_gm);

	USARTF0.BAUDCTRLA = (UART_115K2_BSEL_VALUE & USART_BSEL_gm);
	USARTF0.BAUDCTRLB = ((UART_115K2_BSCALE_VALUE << USART_BSCALE_gp) & USART_BSCALE_gm) | ((UART_115K2_BSEL_VALUE >> 8) & ~USART_BSCALE_gm);
	
	USARTD0.CTRLB = USART_RXEN_bm | USART_TXEN_bm;
 	USARTD1.CTRLB = USART_RXEN_bm | USART_TXEN_bm;
 	USARTF0.CTRLB = USART_RXEN_bm | USART_TXEN_bm;

	USARTD0.CTRLA = USART_RXCINTLVL_MED_gc | USART_TXCINTLVL_OFF_gc | USART_DREINTLVL_OFF_gc;
	USARTD1.CTRLA = USART_RXCINTLVL_MED_gc | USART_TXCINTLVL_OFF_gc | USART_DREINTLVL_OFF_gc;
	USARTF0.CTRLA = USART_RXCINTLVL_MED_gc | USART_TXCINTLVL_OFF_gc | USART_DREINTLVL_OFF_gc;
	
	PMIC.CTRL |= PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm;
	
} /* InitSerial */


uint8_t CanRead_Comm485(void) {
	uint8_t wridx = rx_485_wridx, rdidx = rx_485_rdidx;
	
	if(wridx >= rdidx)
		return wridx - rdidx;
	else
		return wridx - rdidx + RXBUF_DEPTH_COMM485;

} /* CanRead_Comm485 */


uint8_t ReadByte_Comm485(void) {
	uint8_t res, curSlot, nextSlot;
	
	curSlot = rx_485_rdidx;
	/* Busy-wait for a byte to be available. Should not be necessary if the caller calls CanRead_xxx() first */
	while(!CanRead_Comm485()) ;
	
	res = rx_485_buf[curSlot];

	nextSlot = curSlot + 1;
	if(nextSlot >= RXBUF_DEPTH_COMM485)
		nextSlot = 0;
	rx_485_rdidx = nextSlot;
	
	return res;
} /* ReadByte_Comm485 */


uint8_t CanWrite_Comm485(void) {
	uint8_t wridx1 = tx_485_wridx + 1, rdidx = tx_485_rdidx;
	
	if(wridx1 >= TXBUF_DEPTH_COMM485)
		wridx1 -= TXBUF_DEPTH_COMM485;
	if(rdidx >= wridx1)
		return rdidx - wridx1;
	else
		return rdidx - wridx1 + TXBUF_DEPTH_COMM485;
	
} /* CanWrite_Comm485 */


void WriteByte_Comm485(uint8_t data) {
	uint8_t curSlot, nextSlot, savePMIC;
	timetick_t now;
	
	/* Busy-wait for a byte to be available. Should not be necessary if the caller calls CanWrite_xxx() first */
	while(!CanWrite_Comm485()) 
		USARTD0.CTRLA = USART_RXCINTLVL_MED_gc | USART_TXCINTLVL_OFF_gc | USART_DREINTLVL_LO_gc;
	
	curSlot = tx_485_wridx;
	tx_485_buf[curSlot] = data;
	
	nextSlot = curSlot + 1;
	if(nextSlot >= TXBUF_DEPTH_COMM485)
		nextSlot = 0;

	savePMIC = PMIC.CTRL;
	PMIC.CTRL = savePMIC & ~PMIC_LOLVLEN_bm;
	if(tx_485_busy) { /* We still have characters buffered, so safe to just update the write index */ 
		tx_485_wridx = nextSlot;
		PMIC.CTRL = savePMIC;
	}
	else {
		PORTD.OUTSET = PD_DRIVER_ENABLE_COMM485;
		now = GetTicks();
		tx_485_busy = 1;
		tx_485_wridx = nextSlot;
		USARTD0.CTRLA = USART_RXCINTLVL_MED_gc | USART_TXCINTLVL_OFF_gc | USART_DREINTLVL_OFF_gc;
		PMIC.CTRL = savePMIC;
		/* Busy-wait for the output enable time of the RS485 driver to pass */
		BusyWaitTillAfter(now, 3);
		USARTD0.CTRLA = USART_RXCINTLVL_MED_gc | USART_TXCINTLVL_OFF_gc | USART_DREINTLVL_LO_gc;
	}

} /* WriteByte_Comm485 */


uint8_t CanRead_Ctrl(void) {
	uint8_t wridx = rx_ctrl_wridx, rdidx = rx_ctrl_rdidx;
	
	if(wridx >= rdidx)
		return wridx - rdidx;
	else
		return wridx - rdidx + RXBUF_DEPTH_CTRL;
	
} /* CanRead_Ctrl */


uint8_t ReadByte_Ctrl(void) {
	uint8_t res, curSlot, nextSlot;
	
	curSlot = rx_ctrl_rdidx;
	/* Busy-wait for a byte to be available. Should not be necessary if the caller calls CanRead_xxx() first */
	while(!CanRead_Ctrl()) ;
	
	res = rx_ctrl_buf[curSlot];

	nextSlot = curSlot + 1;
	if(nextSlot >= RXBUF_DEPTH_CTRL)
		nextSlot = 0;
	rx_ctrl_rdidx = nextSlot;
	
	return res;
} /* ReadByte_Ctrl */


uint8_t CanWrite_Ctrl(void) {
	uint8_t wridx1 = tx_ctrl_wridx + 1, rdidx = tx_ctrl_rdidx;
	
	if(wridx1 >= TXBUF_DEPTH_CTRL)
		wridx1 -= TXBUF_DEPTH_CTRL;
	if(rdidx >= wridx1)
		return rdidx - wridx1;
	else
		return rdidx - wridx1 + TXBUF_DEPTH_CTRL;
	
} /* CanWrite_Ctrl */


void WriteByte_Ctrl(uint8_t data) {
	uint8_t curSlot, nextSlot, savePMIC;
	
	/* Busy-wait for a byte to be available. Should not be necessary if the caller calls CanWrite_xxx() first */
	while(!CanWrite_Ctrl()) 
		USARTD1.CTRLA = USART_RXCINTLVL_MED_gc | USART_TXCINTLVL_OFF_gc | USART_DREINTLVL_LO_gc;
	
	curSlot = tx_ctrl_wridx;
	tx_ctrl_buf[curSlot] = data;
	
	nextSlot = curSlot + 1;
	if(nextSlot >= TXBUF_DEPTH_CTRL)
		nextSlot = 0;

	savePMIC = PMIC.CTRL;
	PMIC.CTRL = savePMIC & ~PMIC_LOLVLEN_bm;
	tx_ctrl_wridx = nextSlot;
	USARTD1.CTRLA = USART_RXCINTLVL_MED_gc | USART_TXCINTLVL_OFF_gc | USART_DREINTLVL_LO_gc;
	PMIC.CTRL = savePMIC;

} /* WriteByte_Ctrl */


static int Comm485_putchar(char c, FILE *stream) {
	
	WriteByte_Comm485((uint8_t) c);
	
	return 0;
	
} /* Comm485_putchar */


#define TOHEX(x) (((x) & 0x0F) <= 9 ? ('0' + ((x) & 0x0F)) : ('A' + ((x) & 0x0F) - 10))

static int Ctrl_putchar(char c, FILE *stream) {
	
	static uint8_t checksum;
	
	WriteByte_Ctrl((uint8_t) c);
	
	if(c == '>') {
		checksum = 0;
	}
	else if(c == '<') {
		WriteByte_Ctrl(TOHEX((uint8_t) checksum >> 4));
		WriteByte_Ctrl(TOHEX((uint8_t) checksum));
	}
	else {
		checksum ^= (uint8_t) c;
	}
	
	return 0;
	
} /* Ctrl_putchar */


uint8_t CanRead_FC(void) {
	uint8_t wridx = rx_fc_wridx, rdidx = rx_fc_rdidx;
	
	if(wridx >= rdidx)
		return wridx - rdidx;
	else
		return wridx - rdidx + RXBUF_DEPTH_FC;
	
} /* CanRead_FC */


uint8_t ReadByte_FC(void) {
	uint8_t res, curSlot, nextSlot;
	
	curSlot = rx_fc_rdidx;
	/* Busy-wait for a byte to be available. Should not be necessary if the caller calls CanRead_xxx() first */
	while(!CanRead_FC()) ;
	
	res = rx_fc_buf[curSlot];

	nextSlot = curSlot + 1;
	if(nextSlot >= RXBUF_DEPTH_FC)
		nextSlot = 0;
	rx_fc_rdidx = nextSlot;
	
	return res;
} /* ReadByte_FC */


uint8_t CanWrite_FC(void) {
	uint8_t wridx1 = tx_fc_wridx + 1, rdidx = tx_fc_rdidx;
	
	if(wridx1 >= TXBUF_DEPTH_FC)
		wridx1 -= TXBUF_DEPTH_FC;
	if(rdidx >= wridx1)
		return rdidx - wridx1;
	else
		return rdidx - wridx1 + TXBUF_DEPTH_FC;
	
} /* CanWrite_FC */


void WriteByte_FC(uint8_t data) {
	uint8_t curSlot, nextSlot, savePMIC;
	
	/* Busy-wait for a byte to be available. Should not be necessary if the caller calls CanWrite_xxx() first */
	while(!CanWrite_FC())
		USARTF0.CTRLA = USART_RXCINTLVL_MED_gc | USART_TXCINTLVL_OFF_gc | USART_DREINTLVL_LO_gc;
	
	curSlot = tx_fc_wridx;
	tx_fc_buf[curSlot] = data;
	
	nextSlot = curSlot + 1;
	if(nextSlot >= TXBUF_DEPTH_FC)
		nextSlot = 0;

	savePMIC = PMIC.CTRL;
	PMIC.CTRL = savePMIC & ~PMIC_LOLVLEN_bm;
	tx_fc_wridx = nextSlot;
	USARTF0.CTRLA = USART_RXCINTLVL_MED_gc | USART_TXCINTLVL_OFF_gc | USART_DREINTLVL_LO_gc;
	PMIC.CTRL = savePMIC;

} /* WriteByte_FC */


ISR(USARTD0_RXC_vect) {
	
	uint8_t curSlot, nextSlot;
	
	curSlot = rx_485_wridx;
	rx_485_buf[curSlot] = USARTD0.DATA;
	
	nextSlot = curSlot + 1;
	if(nextSlot >= RXBUF_DEPTH_COMM485)
	nextSlot = 0;
	
	if(nextSlot != rx_485_rdidx)
	rx_485_wridx = nextSlot;
	
} /* ISR(USARTD0_RXC_vect) */


ISR(USARTD0_DRE_vect) {
	
	uint8_t curSlot, nextSlot, lastSlot;
	
	nextSlot = curSlot = tx_485_rdidx;
	lastSlot = tx_485_wridx;
	
	if(curSlot != lastSlot) {
		USARTD0.DATA = tx_485_buf[curSlot];
		if(++nextSlot >= TXBUF_DEPTH_COMM485)
		nextSlot = 0;
	}
	if(nextSlot == lastSlot)
	USARTD0.CTRLA = USART_RXCINTLVL_MED_gc | USART_TXCINTLVL_LO_gc | USART_DREINTLVL_OFF_gc;
	
	tx_485_rdidx = nextSlot;
	
} /* ISR(USARTD0_DRE_vect) */


ISR(USARTD0_TXC_vect) {
	
	if(tx_485_rdidx == tx_485_wridx) {/* Still no bytes to tx ? */
		PORTD.OUTCLR = PD_DRIVER_ENABLE_COMM485; /* Transmission complete; we can turn off the driver enable line */
		tx_485_busy = 0;
	}	
	else /* Re-enable TX ISR */
		USARTD0.CTRLA = USART_RXCINTLVL_MED_gc | USART_TXCINTLVL_OFF_gc | USART_DREINTLVL_LO_gc;
	
} /* ISR(USARTD0_TXC_vect) */


ISR(USARTD1_RXC_vect) {
	
	uint8_t curSlot, nextSlot;
	
	curSlot = rx_ctrl_wridx;
	rx_ctrl_buf[curSlot] = USARTD1.DATA;
	
	nextSlot = curSlot + 1;
	if(nextSlot >= RXBUF_DEPTH_CTRL)
		nextSlot = 0;
		
	if(nextSlot != rx_ctrl_rdidx)
		rx_ctrl_wridx = nextSlot;
	
} /* ISR(USARTD1_RXC_vect) */


ISR(USARTD1_DRE_vect) {
	
	uint8_t curSlot, nextSlot, lastSlot;
	
	nextSlot = curSlot = tx_ctrl_rdidx;
	lastSlot = tx_ctrl_wridx;
	
	if(curSlot != lastSlot) {
		USARTD1.DATA = tx_ctrl_buf[curSlot];
		nextSlot = curSlot + 1;
		if(nextSlot >= TXBUF_DEPTH_CTRL)
			nextSlot = 0;
	}
	if(nextSlot == lastSlot)
		USARTD1.CTRLA = USART_RXCINTLVL_MED_gc | USART_TXCINTLVL_OFF_gc | USART_DREINTLVL_OFF_gc;
	
	tx_ctrl_rdidx = nextSlot;
	
} /* ISR(USARTD1_DRE_vect) */


ISR(USARTF0_RXC_vect) {
	
	uint8_t curSlot, nextSlot;
	
	curSlot = rx_fc_wridx;
	rx_fc_buf[curSlot] = USARTF0.DATA;
	
	nextSlot = curSlot + 1;
	if(nextSlot >= RXBUF_DEPTH_FC)
	nextSlot = 0;
	
	if(nextSlot != rx_fc_rdidx)
	rx_fc_wridx = nextSlot;
	
} /* ISR(USARTF0_RXC_vect) */


ISR(USARTF0_DRE_vect) {
	
	uint8_t curSlot, nextSlot, lastSlot;
	
	nextSlot = curSlot = tx_fc_rdidx;
	lastSlot = tx_fc_wridx;
	
	if(curSlot != lastSlot) {
		USARTF0.DATA = tx_fc_buf[curSlot];
		nextSlot = curSlot + 1;
		if(nextSlot >= TXBUF_DEPTH_FC)
		nextSlot = 0;
	}
	if(nextSlot == lastSlot)
	USARTF0.CTRLA = USART_RXCINTLVL_MED_gc | USART_TXCINTLVL_OFF_gc | USART_DREINTLVL_OFF_gc;
	
	tx_fc_rdidx = nextSlot;
	
} /* ISR(USARTF0_DRE_vect) */
