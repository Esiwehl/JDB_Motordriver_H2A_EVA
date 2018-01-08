// This file has been prepared for Doxygen automatic documentation generation.
/*! \file *********************************************************************
 *
 * \brief  XMega low power macros
 *
 *      This file defines the macros used by LOWPOWER_Init(), tailored to
 *      the individual XMEGA variants.
 *      Six macros are defined, of which the last five are device specific:
 *      <pre>
 *      DISABLE_JTAG() - disables JTAG interface
 *      DISABLE_GEN() - disables crypto, EBI, DMA and event system
 *      DISABLE_TC() - disables timer/counters
 *      DISABLE_COM() - disables TWI, SPI and UARTs
 *      DISABLE_ANLG() - disables ADC, DAC and comparators
 *      ENABLE_PULLUP() - enables pull-up on all available I/O pins
 *      </pre>
 *
 *      \note These macros are not created in lowpower.h because they are only
 *      meant for use in lowpower.c.
 *
 * \par Application note:
 *      AVR1010: Minimizing the power consumption of XMEGA devices
 *
 * \par Documentation
 *      For comprehensive code documentation, supported compilers, compiler
 *      settings and supported devices see readme.html
 *
 * \author
 *      Atmel Corporation: http://www.atmel.com \n
 *      Support email: avr@atmel.com
 *
 * $Revision: 2770 $
 * $Date: 2009-09-11 10:55:22 +0200 (fr, 11 sep 2009) $  \n
 *
 * Copyright (c) 2008, Atmel Corporation All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. The name of ATMEL may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY AND
 * SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#ifndef LOWPOWER_MACROS_H
#define LOWPOWER_MACROS_H


/*============================ MACROS ========================================*/
//! Convenience macro for disabling the JTAG interface.
#define DISABLE_JTAG( ) { \
	ENTER_CRITICAL_REGION(); \
	CCP = 0xD8; \
	MCU_MCUCR = MCU_JTAGD_bm; \
	LEAVE_CRITICAL_REGION(); \
}

//! Convenience macro for enabling pull-ups on specified pins on any port.
#define __PORT_PULLUP(port, mask) { \
	PORTCFG.MPCMASK = mask ; \
	port.PIN0CTRL = PORT_OPC_PULLUP_gc; \
}


//! Pull down
#define __PORT_PULLDOWN(port, mask) { \
	PORTCFG.MPCMASK = mask ; \
	port.PIN0CTRL = PORT_OPC_PULLDOWN_gc; \
}

/* Create macros according to XMEGA device selection.
 *
 * Create macros that only set the PRR-bits for peripherals and enable pullup
 * on ports/pins that actually exist on the device, according to datasheets.
 * (This is a very cautious approach.)
 */


// A3

#define DISABLE_GEN( ) { \
	PR.PRGEN |= PR_AES_bm | PR_DMA_bm | PR_RTC_bm; \
}

#define DISABLE_TC( ) { \
	PR.PRPD |= PR_HIRES_bm | PR_TC0_bm | PR_TC1_bm; \
	PR.PRPE |= PR_HIRES_bm | PR_TC0_bm | PR_TC1_bm; \
	PR.PRPF |= PR_HIRES_bm | PR_TC0_bm; \
}

#define DISABLE_COM( ) { \
	PR.PRPC |= PR_SPI_bm | PR_TWI_bm | PR_USART0_bm | PR_USART1_bm; \
	PR.PRPD |= PR_SPI_bm ; \
	PR.PRPE |= PR_SPI_bm | PR_TWI_bm | PR_USART0_bm | PR_USART1_bm; \
}

#define DISABLE_ANLG( ) { \
	PR.PRPA |= PR_AC_bm; \
	PR.PRPB |= PR_AC_bm; \
}

#define ENABLE_PULLUP( ) { \
	__PORT_PULLUP(PORTA, 0xFF); \
	__PORT_PULLUP(PORTB, 0xFF); \
	__PORT_PULLUP(PORTC, 0xFF); \
	__PORT_PULLUP(PORTD, 0xFF); \
	__PORT_PULLUP(PORTE, 0xFF); \
	__PORT_PULLUP(PORTF, 0xFF); \
	__PORT_PULLUP(PORTR, 0x03); \
}

#define ENABLE_PULLDOWN( ) { \
	__PORT_PULLDOWN(PORTA, 0x01); \
	__PORT_PULLDOWN(PORTB, 0xFF); \
	__PORT_PULLDOWN(PORTC, 0xFF); \
	__PORT_PULLDOWN(PORTD, 0xFF); \
	__PORT_PULLDOWN(PORTE, 0xFF); \
	__PORT_PULLDOWN(PORTF, 0xFF); \
	__PORT_PULLDOWN(PORTR, 0x03); \
}


#endif // LOWPOWER_MACROS_H
