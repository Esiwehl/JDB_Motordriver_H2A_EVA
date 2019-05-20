/*
 * md_core_analog.c
 * 
 * Contains the core routines driving the analog data capture and cruise control in the H2A/EVA SECOM Motordriver
 *
 * Created: 4/24/2013 3:59:51 PM
 *  Author: bakker
 */ 

#include <stdio.h>
#include <stddef.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include "md_core_analog.h"
#include "md_serial.h"
#include "DataInPrivate.h"
#include "util.h"

#define TIMER_PERIOD 6400 /* Number of clock cycles per timer cycle, for 5kHz at a 32MHz clock */

#define SPEEDSENSOR_MIDWAY_VAL		727 /* Calculated offset for a point halfway the expected sensor input extremes */
#define SPEEDSENSOR_DEGLITCH		(CYCLES_PER_SECOND / 2000)	/* Number of valid over-threshold values that need to be seen before an edge is recognized */
#define SPEEDSENSOR_MAX_INTERVAL	(CYCLES_PER_SECOND / 3)	/* Max time between edges before we assume we're standing still */
#define SPEEDSENSOR_MIN_SPEED		1.0f /* Below this speed in km/h we report to be standing still */

/* Wheel constants */

#define H2A_WHEEL_METER_PER_ROT		1.488f
#define EVA_WHEEL_METER_PER_ROT		1.753f
#define H2A_WHEEL_PULSE_PER_ROT		24
#define EVA_WHEEL_PULSE_PER_ROT		15
#define WHEEL_MS_TO_KMH				3.6f
#define H2A_WHEEL_METER_PER_PULSE	(H2A_WHEEL_METER_PER_ROT / H2A_WHEEL_PULSE_PER_ROT)
#define EVA_WHEEL_METER_PER_PULSE	(EVA_WHEEL_METER_PER_ROT / EVA_WHEEL_PULSE_PER_ROT)

#define	H2A_GEAR_RATIO				0.067f
#define	EVA_GEAR_RATIO				0.067f // Voor nu H2A want eva niet bekend

#define H2A_MOTOR_SPEED_CONSTANT	248.0f			// rpm/v
#define EVA_MOTOR_SPEED_CONSTANT	178.0f			// rpm/v

#define CC2_MINIMUM_SPEED_KMH	15

#define CC_REG_CYCLES		CYCLES_PER_SECOND /* Ticks between runs of the CC algorithm */
#define H2A_CC_MAX_INTERVAL	(CYCLES_PER_SECOND / H2A_WHEEL_PULSE_PER_ROT) /* Do not attempt CC below ~5 km/h */
#define EVA_CC_MAX_INTERVAL	(CYCLES_PER_SECOND / EVA_WHEEL_PULSE_PER_ROT) /* Do not attempt CC below ~5 km/h */ 
#define CC_MAX_POWER		7
#define REGBRAKE_LEVEL		8				// 8 wasn't used before				

#define CC_TURBO_BOOST		PIN3_bm
#define REGBRAKE_PIN		PIN3_bm
#define CC_DEFAULT_POWER	2
#define CC_PINS				(PIN0_bm | PIN1_bm | PIN2_bm)

#define MOTORTEMP_SCALE (40.0f / 2.4f)


#define PWM_DC_FS	0xFF
#define PWM_FREQ_FILTER_SHIFT 3

#define PWM_FREQ_SCALE ((1<<PWM_FREQ_FILTER_SHIFT) / (float) CYCLES_PER_SECOND)
#define PWM_DC_SCALE (PWM_DC_FS / 100.0f)

#define MINMAX_RESET_PERIOD CYCLES_PER_SECOND


typedef struct {
	int32_t fcVoltageFiltered, fcCurrentFiltered, scVoltageFiltered, scCurrentFiltered;
	int32_t fcPowerFiltered, scPowerFiltered;
	int64_t fcEnergy, scEnergy;
	
	uint8_t idealDiodeState;
	uint32_t idealDiodeTimestamp;
			
} tCoreAnalogH2AData;

typedef struct {

	int32_t motorTempFrontFiltered, motorTempRearFiltered;
	int32_t angSenseFiltered, angFSFiltered;
	
} tCoreAnalogEVAData;

typedef struct {
	
	union {
		tCoreAnalogH2AData h2a;
		tCoreAnalogEVAData eva;
	} adc;
	
	int32_t driverTempFiltered, motorVoltageFiltered, motorCurrentFiltered, inVoltageFiltered, inCurrentFiltered;
	int32_t inVoltageMin, inVoltageMax;
	int32_t inCurrentMin, inCurrentMax;
	int32_t inVoltageMinTimestamp, inVoltageMaxTimestamp;
	int32_t inCurrentMinTimestamp, inCurrentMaxTimestamp;
	int32_t motorPowerFiltered, inPowerFiltered;
	int64_t motorEnergy, inEnergy;
	int32_t speedSensorPulseInterval;
	uint32_t speedSensorPositivePulsesSeen;
	int16_t speedSensorLastValidInterval;
	uint32_t speedSensorPreviousValidEdgeTimestamp;

	uint8_t selFPState, selCCState, selCC2State;
	uint32_t selFPTimestamp, selCCTimestamp, selCC2Timestamp;

	uint16_t pwmFrequency, pwmDutyCycle;

	int32_t ccTargetSpeed;
	uint8_t ccPower;
	
} tCoreAnalogSensorData;


typedef struct {
	
	int16_t fcVoltageOffset, fcCurrentOffset, scVoltageOffset, scCurrentOffset;
	double fcVoltageScale, fcCurrentScale, scVoltageScale, scCurrentScale;
	
	int16_t motorVoltageOffset, motorCurrentOffset, inVoltageOffset, inCurrentOffset;
	double motorVoltageScale, motorCurrentScale, inVoltageScale, inCurrentScale;
	
} tCoreAnalogCalibration;


/* Variables used to communicate between user code and interrupt routines */
static volatile tCoreAnalogCalibration sCal;

static volatile tCoreAnalogSensorData sSensorDataSnapshot;
static volatile uint32_t sSessionCycleCountSnapshot;
static volatile uint8_t sTakeSnapshot;

static volatile tCoreAnalogSensorData sSensorDataSnapshot485;
static volatile uint32_t sSessionCycleCountSnapshot485;
static volatile uint8_t sTakeSnapshot485;

static volatile uint32_t sSessionCycleCount;

static volatile uint8_t sCC2OverrideEnable, sCC2TargetSpeedUpdate;
static volatile int32_t sCC2MinSpeed, sCC2TargetSpeed;

static volatile float aTargetSpeed;		// Variable for saving target speed received from serial debug


/* Variables used to communicate between the interrupt routines */


/* Static functions */

static void InitCoreAnalogADC(void);
static void InitCoreAnalogCalibration(void);
static void InitCoreAnalogTimer(void);
static void InitCoreAnalogSensors(void);

static float GetProcessedSpeed(int32_t speedSensorPulseInterval, float wheelMeterPerPulse);

void InitCoreAnalog(void) {
	
	InitCoreAnalogADC();
	InitCoreAnalogCalibration();
	InitCoreAnalogTimer();
	InitCoreAnalogSensors();

	if(I_AM_EVA_L || I_AM_EVA_R)
		sCC2MinSpeed = (65536.0 * EVA_WHEEL_METER_PER_PULSE * WHEEL_MS_TO_KMH * CYCLES_PER_SECOND) /  CC2_MINIMUM_SPEED_KMH;
	else
		sCC2MinSpeed = (65536.0 * H2A_WHEEL_METER_PER_PULSE * WHEEL_MS_TO_KMH * CYCLES_PER_SECOND) /  CC2_MINIMUM_SPEED_KMH;
	
} /* InitCoreAnalog */


static void InitCoreAnalogADC(void) {

	ADCA.CALL = ReadCalibrationByte(offsetof(NVM_PROD_SIGNATURES_t, ADCACAL0));
	ADCA.CALH = ReadCalibrationByte(offsetof(NVM_PROD_SIGNATURES_t, ADCACAL1));
	ADCB.CALL = ReadCalibrationByte(offsetof(NVM_PROD_SIGNATURES_t, ADCBCAL0));
	ADCB.CALH = ReadCalibrationByte(offsetof(NVM_PROD_SIGNATURES_t, ADCBCAL1));
	
	PORTA.OUT = 0x00;
	PORTA.DIR = 0x00;
	
	PORTB.OUT = 0x00;
	PORTB.DIR = 0x00;
	
	/* Disable input buffers on ADC ports to reduce leakage (I hope -- not verified) */
	PORTA.PIN0CTRL = PORT_ISC_INPUT_DISABLE_gc;
	PORTA.PIN1CTRL = PORT_ISC_INPUT_DISABLE_gc;
	PORTA.PIN2CTRL = PORT_ISC_INPUT_DISABLE_gc;
	PORTA.PIN3CTRL = PORT_ISC_INPUT_DISABLE_gc;
	PORTA.PIN4CTRL = PORT_ISC_INPUT_DISABLE_gc;
	PORTA.PIN5CTRL = PORT_ISC_INPUT_DISABLE_gc;
	PORTA.PIN6CTRL = PORT_ISC_INPUT_DISABLE_gc;

	PORTB.PIN0CTRL = PORT_ISC_INPUT_DISABLE_gc;
	PORTB.PIN1CTRL = PORT_ISC_INPUT_DISABLE_gc;
	PORTB.PIN2CTRL = PORT_ISC_INPUT_DISABLE_gc;
	PORTB.PIN3CTRL = PORT_ISC_INPUT_DISABLE_gc;
	PORTB.PIN4CTRL = PORT_ISC_INPUT_DISABLE_gc;
	PORTB.PIN5CTRL = PORT_ISC_INPUT_DISABLE_gc;
	PORTB.PIN6CTRL = PORT_ISC_INPUT_DISABLE_gc;
	PORTB.PIN7CTRL = PORT_ISC_INPUT_DISABLE_gc;
	
	/* Pull-downs on remaining port pins */
	PORTA.PIN7CTRL = PORT_OPC_PULLDOWN_gc;
		
	/* Init the ADCs */
	ADCA.CTRLB = ADC_CURRLIMIT_NO_gc | ADC_RESOLUTION_12BIT_gc | ADC_CONMODE_bm; /* Enable signed mode, 12-bit conversion */
	ADCA.REFCTRL = ADC_REFSEL_AREFA_gc; /* REF = external on pin PA0; bandgap off, temp sensor off */
	ADCA.EVCTRL = ADC_SWEEP_0123_gc | ADC_EVSEL_0123_gc | ADC_EVACT_SYNCSWEEP_gc; /* Sweep channels 0-3, trigger using event channel 0, sync sweep on event */
	ADCA.PRESCALER = ADC_PRESCALER_DIV32_gc; /* Divide peripheral clock by 32. */
	
	ADCA.CH0.CTRL = ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_DIFF_gc; /* Diff in, not using the gainblock */
	ADCA.CH1.CTRL = ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_DIFF_gc; /* Diff in, not using the gainblock */
	ADCA.CH2.CTRL = ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_DIFF_gc; /* Diff in, not using the gainblock */
	ADCA.CH3.CTRL = ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_DIFF_gc; /* Diff in, not using the gainblock */

	ADCA.CH0.MUXCTRL = ADC_CH_MUXPOS_PIN5_gc | ADC_CH_MUXNEG_PIN1_gc; /* Measure ADC_FC1 on PA5 vs V33/2 on PA1 */
	ADCA.CH1.MUXCTRL = ADC_CH_MUXPOS_PIN3_gc | ADC_CH_MUXNEG_PIN1_gc; /* Measure ADC_FC3 on PA3 vs V33/2 on PA1 */
	ADCA.CH2.MUXCTRL = ADC_CH_MUXPOS_PIN6_gc | ADC_CH_MUXNEG_PIN1_gc; /* Measure ADC_SP_RAW on PA6 vs V33/2 on PA1 */
	ADCA.CH3.MUXCTRL = ADC_CH_MUXPOS_PIN12_gc | ADC_CH_MUXNEG_PIN1_gc; /* Measure ADC_VMOTOR on PB4 vs V33/2 on PA1 */
	
	ADCA.CH0.INTCTRL = ADC_CH_INTMODE_COMPLETE_gc | ADC_CH_INTLVL_LO_gc; /* Trigger a low-level interrupt on completion of the CH0 conversion */

	ADCA.CTRLA = ADC_ENABLE_bm; /* Enable ADC */

	ADCB.CTRLB = ADC_CURRLIMIT_NO_gc | ADC_RESOLUTION_12BIT_gc | ADC_CONMODE_bm; /* Enable signed mode, 12-bit conversion */
	ADCB.REFCTRL = ADC_REFSEL_AREFA_gc; /* REF = external on pin PA0; bandgap off, temp sensor off */
	ADCB.EVCTRL = ADC_SWEEP_0123_gc | ADC_EVSEL_0123_gc | ADC_EVACT_SYNCSWEEP_gc; /* Sweep channels 0-3, trigger using event channel 0, sync sweep on event */
	ADCB.PRESCALER = ADC_PRESCALER_DIV32_gc; /* Divide peripheral clock by 32. */
	
	ADCB.CH0.CTRL = ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_DIFF_gc; /* Diff in, not using the gainblock */
	ADCB.CH1.CTRL = ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_DIFF_gc; /* Diff in, not using the gainblock */
	ADCB.CH2.CTRL = ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_DIFF_gc; /* Diff in, not using the gainblock */
	ADCB.CH3.CTRL = ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_DIFF_gc; /* Diff in, not using the gainblock */

	ADCB.CH0.MUXCTRL = ADC_CH_MUXPOS_PIN12_gc | ADC_CH_MUXNEG_PIN1_gc; /* Measure ADC_FC2 on PA4 vs V33/2 on PB1 */
	ADCB.CH1.MUXCTRL = ADC_CH_MUXPOS_PIN10_gc | ADC_CH_MUXNEG_PIN1_gc; /* Measure ADC_FC4 on PA2 vs V33/2 on PB1 */
	ADCB.CH2.MUXCTRL = ADC_CH_MUXPOS_PIN6_gc | ADC_CH_MUXNEG_PIN1_gc; /* Measure ADC_TEMP on PB6 vs V33/2 on PB1 */
	ADCB.CH3.MUXCTRL = ADC_CH_MUXPOS_PIN5_gc | ADC_CH_MUXNEG_PIN1_gc; /* Measure ADC_IMOTOR on PB5 vs V33/2 on PB1 */

	ADCB.CTRLA = ADC_ENABLE_bm; /* Enable ADC */

	PMIC.CTRL |= PMIC_LOLVLEN_bm; /* Enable lo-level interrupt (ADC completion) */

} /* InitCoreAnalogADC */


static void InitCoreAnalogTimer(void) {

	/* Init the timer/PWM and associated I/Os */
	TCC0.CTRLB = 0x00; /* No input capture, no PWM, normal mode */
	/* CTRLC is of no interest to us */
	TCC0.CTRLD = TC_EVACT_OFF_gc | TC_EVSEL_OFF_gc; /* No events */
	TCC0.CTRLE = 0x00; /* No byte mode */
	TCC0.PER = TIMER_PERIOD;
	TCC0.INTCTRLA = TC_ERRINTLVL_OFF_gc | TC_OVFINTLVL_OFF_gc; /* All timer interrupts off */
	TCC0.INTCTRLB = 0x00; /* Disable Compare/Capture interrupts */
	TCC0.CNT = 0;
	TCC0.CTRLA = TC_CLKSEL_DIV1_gc; /* Start the timer with a clock divider of 1 */
	
	EVSYS.CH0MUX = EVSYS_CHMUX_TCC0_OVF_gc; /* Connect TCC0 overflow to event channel 0, thus triggering an ADC sweep on ADCs 0 and 1 */
	
	/* Use TCC1 to count PWM cycles */
	TCC1.CTRLA = TC_CLKSEL_EVCH4_gc;
	TCC1.CTRLB = TC_WGMODE_NORMAL_gc;
	/* No need to modify CTRLC */
	TCC1.CTRLD = TC_EVACT_OFF_gc | TC_EVSEL_OFF_gc;
	TCC1.CTRLE = TC_BYTEM_NORMAL_gc;
	TCC1.INTCTRLA = TC_ERRINTLVL_OFF_gc | TC_OVFINTLVL_OFF_gc;
	TCC1.INTCTRLB = TC_CCAINTLVL_OFF_gc | TC_CCBINTLVL_OFF_gc | TC_CCCINTLVL_OFF_gc | TC_CCDINTLVL_OFF_gc;
	TCC1.CTRLFCLR = TC0_DIR_bm;
	TCC1.PER = 0xFFFF;
	
	PORTC.DIRCLR = PIN6_bm;
	PORTC.PIN6CTRL = PORT_ISC_RISING_gc;
	
	EVSYS.CH4MUX = EVSYS_CHMUX_PORTC_PIN6_gc;
	EVSYS.CH4CTRL = EVSYS_DIGFILT_3SAMPLES_gc;

} /* InitCoreAnalogTimer */


static void InitCoreAnalogCalibration(void) {

	sCal.fcVoltageScale = (1000.0f / (2.4f * (20.0f / 2.0f)));
	sCal.fcCurrentScale = ((2.58f / 2.388f) * 1000.0f / (2.4f * (10.0f / 2.0f)));
	sCal.scVoltageScale = (1000.0f / (2.4f * (20.0f / 2.0f)));
	sCal.scCurrentScale = ((2.53f / 2.3883f) * 1000.0f / (2.4f * (10.0f / 2.0f)));

	sCal.motorVoltageScale = (1000.0f / (24.0f));
	sCal.motorCurrentScale = (1000.0f / (16.7f));
	sCal.inVoltageScale = (1000.0f / (24.0f));
	sCal.inCurrentScale = (1000.0f / (20.0f));
	
	TRYREADEEPROM(sCal.fcVoltageScale, EE_FC_VSCALE);
	TRYREADEEPROM(sCal.fcCurrentScale, EE_FC_ISCALE);
	TRYREADEEPROM(sCal.scVoltageScale, EE_SC_VSCALE);
	TRYREADEEPROM(sCal.scCurrentScale, EE_SC_ISCALE);

	TRYREADEEPROM(sCal.fcVoltageOffset, EE_FC_VOFFSET);
	TRYREADEEPROM(sCal.fcCurrentOffset, EE_FC_IOFFSET);
	TRYREADEEPROM(sCal.scVoltageOffset, EE_SC_VOFFSET);
	TRYREADEEPROM(sCal.scCurrentOffset, EE_SC_IOFFSET);

	TRYREADEEPROM(sCal.motorVoltageScale, EE_M_VSCALE);
	TRYREADEEPROM(sCal.motorCurrentScale, EE_M_ISCALE);
	TRYREADEEPROM(sCal.inVoltageScale, EE_IN_VSCALE);
	TRYREADEEPROM(sCal.inCurrentScale, EE_IN_ISCALE);

	TRYREADEEPROM(sCal.motorVoltageOffset, EE_M_VOFFSET);
	TRYREADEEPROM(sCal.motorCurrentOffset, EE_M_IOFFSET);
	TRYREADEEPROM(sCal.inVoltageOffset, EE_IN_VOFFSET);
	TRYREADEEPROM(sCal.inCurrentOffset, EE_IN_IOFFSET);

	/* DAC for trimming */
	DACB.CTRLA = DAC_CH0EN_bm | DAC_ENABLE_bm;
	DACB.CTRLB = DAC_CHSEL_SINGLE_gc;
	DACB.CTRLC = DAC_REFSEL_AREFA_gc;

	DACB.CH0DATA = 2 * SPEEDSENSOR_MIDWAY_VAL; /* Calculated offset for a point halfway the expected sensor input extremes */

} /* InitCoreAnalogCalibration */


typedef struct {
	const char *name;
	volatile int32_t *valFiltered;
	volatile double *scale;
	volatile int16_t *offset;
	void *ee_scale_addr, *ee_offset_addr;
} tCal;

#define NUM_CAL_OPTS_H2A 8
#define NUM_CAL_OPTS_EVA 4
#define MAX_CAL_POINTS 5
#define MAX_WHEEL_PULSES_SEEN_BEFORE_CAL 100


void receiveSpeedfromDebug(FILE *fp){
	uint8_t inputChar = 0;
	uint32_t totalChar = 0;

	while(CanRead_Ctrl())
	ReadByte_Ctrl();
		
	do {
		inputChar = ReadByte_Ctrl();
		if(isdigit(inputChar)) {
			totalChar = 10 * totalChar + inputChar - '0';
		}
	} while(isdigit(inputChar));
	
	
	if(inputChar == 'a' || inputChar == 'A'){
		aTargetSpeed = (totalChar / 1000.0f);		// m/h to km/h
	} else{
		aTargetSpeed = 0;
	}

	
}


void CalibrateChannel(FILE *fp) {
	
	tCal calChan[NUM_CAL_OPTS_H2A] = {
		{ "Input Voltage", &(sSensorDataSnapshot.inVoltageFiltered), &(sCal.inVoltageScale), &(sCal.inVoltageOffset), EE_IN_VSCALE, EE_IN_VOFFSET },
		{ "Input Current", &(sSensorDataSnapshot.inCurrentFiltered), &(sCal.inCurrentScale), &(sCal.inCurrentOffset), EE_IN_ISCALE, EE_IN_IOFFSET },
		{ "Motor Voltage", &(sSensorDataSnapshot.motorVoltageFiltered), &(sCal.motorVoltageScale), &(sCal.motorVoltageOffset), EE_M_VSCALE, EE_M_VOFFSET },
		{ "Motor Current", &(sSensorDataSnapshot.motorCurrentFiltered), &(sCal.motorCurrentScale), &(sCal.motorCurrentOffset), EE_M_ISCALE, EE_M_IOFFSET },
		{ "FC Voltage", &(sSensorDataSnapshot.adc.h2a.fcVoltageFiltered), &(sCal.fcVoltageScale), &(sCal.fcVoltageOffset), EE_FC_VSCALE, EE_FC_VOFFSET },
		{ "FC Current", &(sSensorDataSnapshot.adc.h2a.fcCurrentFiltered), &(sCal.fcCurrentScale), &(sCal.fcCurrentOffset), EE_FC_ISCALE, EE_FC_IOFFSET },
		{ "Supercap Voltage", &(sSensorDataSnapshot.adc.h2a.scVoltageFiltered), &(sCal.scVoltageScale), &(sCal.scVoltageOffset), EE_SC_VSCALE, EE_SC_VOFFSET },
		{ "Supercap Current", &(sSensorDataSnapshot.adc.h2a.scCurrentFiltered), &(sCal.scCurrentScale), &(sCal.scCurrentOffset), EE_SC_ISCALE, EE_SC_IOFFSET },
	};
	double meas[MAX_CAL_POINTS], cal[MAX_CAL_POINTS];
	int32_t curCal;
	int16_t newOffset_ee;
	int8_t selectedCal = -1, numPoints, i, thisChar, done;
	double xavg, yavg, ss_xx, ss_xy, newOffset, newScale, newScale_ee;
	
	TakeSnapshot();
	while(!IsSnapshotDone());
	if(sSensorDataSnapshot.speedSensorPositivePulsesSeen > MAX_WHEEL_PULSES_SEEN_BEFORE_CAL) {
		fprintf(fp, "# *** Too many wheel revolutions seen for calibration -- aborted\r\n");
		return;
	}
	fprintf(fp, "# Calibration started.\r\n");
	while(selectedCal < 0) {
		fprintf(fp, "# Please select channel to calibrate:\r\n");
		for(i = 0; i < ((I_AM_EVA_L | I_AM_EVA_R) ? NUM_CAL_OPTS_EVA : NUM_CAL_OPTS_H2A); i++)
			fprintf(fp, "#  %d) %s\r\n", i + 1, calChan[i].name);
		fprintf(fp, "# :");
		while(CanRead_Ctrl())
			ReadByte_Ctrl();
		selectedCal = ReadByte_Ctrl() - '1';
		fprintf(fp, "\r\n");
		if(selectedCal >= i)
			selectedCal = -1;
		if(selectedCal < 0)
			fprintf(fp, "# *** Invalid channel selected\r\n");
	}
	fprintf(fp, "# Channel %s selected.\r\n", calChan[selectedCal].name);
	fprintf(fp, "# Please enter up to %d calibration points.\r\n", MAX_CAL_POINTS);
	numPoints = 0;
	done = 0;
	do {
		while(CanRead_Ctrl())
			ReadByte_Ctrl();
		fprintf(fp, "# Calibration point #%d, in %s (no decimal point!): ", numPoints + 1, selectedCal & 1 ? "mA" : "mV");
		curCal = 0;
		do {
			thisChar = ReadByte_Ctrl();
			if(isdigit(thisChar)) {
				fprintf(fp, "%c", thisChar);
				curCal = 10 * curCal + thisChar - '0';
			}
		} while(isdigit(thisChar));
		cal[numPoints] = curCal / 1000.0f;
		TakeSnapshot();
		while(!IsSnapshotDone()) ;
		meas[numPoints] = *(calChan[selectedCal].valFiltered) / 65536.0f;
		fprintf(fp, "\r\n# Calibration value %.3f; measured %.3f (raw ADC: %.3fV). Is this correct (y/n)? ", 
			        cal[numPoints], meas[numPoints] / *(calChan[selectedCal].scale), meas[numPoints] / 1000.0f);
		do {
			thisChar = ReadByte_Ctrl();
		} while(thisChar != 'y' && thisChar != 'Y' && thisChar != 'n' && thisChar != 'N' && thisChar != 0x1B);
		fprintf(fp, "%c\r\n", thisChar);
		if(thisChar == 0x1B)
			done = 1;
		else if(thisChar == 'y' || thisChar == 'Y') {
			numPoints++;
			if(numPoints < MAX_CAL_POINTS) {
				fprintf(fp, "# Add another calibration point (y/n)? ");
				do {
					thisChar = ReadByte_Ctrl();
				} while(thisChar != 'y' && thisChar != 'Y' && thisChar != 'n' && thisChar != 'N' && thisChar != 0x1B);
				fprintf(fp, "%c\r\n", thisChar);
				if(thisChar != 'y' && thisChar != 'Y')
					done = 1;
			}
		}
	} while(numPoints < MAX_CAL_POINTS && !done);
	
	if(numPoints < 2) {
		fprintf(fp, "# *** At least two calibration points are required to determine offset and gain!\r\n");
		return;
	}
	
	xavg = yavg = 0.0f;
	for(i = 0; i < numPoints; i++) {
		xavg += cal[i];
		yavg += meas[i];
	}
	xavg /= numPoints;
	yavg /= numPoints;
	
	ss_xx = ss_xy = 0.0f;
	for(i = 0; i < numPoints; i++) {
		ss_xx += (cal[i] - xavg) * (cal[i] - xavg);
		ss_xy += (cal[i] - xavg) * (meas[i] - yavg);
	}
	
	newScale = ss_xy / ss_xx;
	newOffset = yavg - newScale * xavg;
	
	while(CanRead_Ctrl())
		ReadByte_Ctrl();
	fprintf(fp, "# New calibration for channel %s:\r\n", calChan[selectedCal].name);
	fprintf(fp, "# offset %.0f (was %d), scale %.4f (was %.4f). Apply new values (y/n)?",
		*calChan[selectedCal].offset + round(newOffset), *calChan[selectedCal].offset, newScale, *calChan[selectedCal].scale);
	do {
		thisChar = ReadByte_Ctrl();
	} while(thisChar != 'y' && thisChar != 'Y' && thisChar != 'n' && thisChar != 'N' && thisChar != 0x1B);
	fprintf(fp, "%c\r\n", thisChar);
	if(thisChar == 'y' || thisChar == 'Y') {
		*calChan[selectedCal].offset += round(newOffset);
		*calChan[selectedCal].scale = newScale;
		UPDATEEEPROM(*calChan[selectedCal].offset, calChan[selectedCal].ee_offset_addr);
		UPDATEEEPROM(*calChan[selectedCal].scale, calChan[selectedCal].ee_scale_addr);
		fprintf(fp, "# New cal value (struct): %d / %.4f\r\n", *calChan[selectedCal].offset, *calChan[selectedCal].scale);
		TRYREADEEPROM(newOffset_ee, calChan[selectedCal].ee_offset_addr);
		TRYREADEEPROM(newScale_ee, calChan[selectedCal].ee_scale_addr);
		fprintf(fp, "# New cal value (eeprom): %d / %.4f\r\n",newOffset_ee, newScale_ee);
		fprintf(fp, "# Calibration applied and saved to EEPROM.\r\n");
	}
	
} /* CalibrateChannel */


static int CoreAnalogTakeSnapshot485(const char *subadress) {
	
	sTakeSnapshot485 = 1;
	return 0;
	
} /* CoreAnalogTakeSnapshot485 */


static int GetWheelSpeedH2A(const char *subadress, char *printbuf, int maxChars) {
	int err = 1;

	if(snprintf(printbuf, maxChars, "%.3f", GetProcessedSpeed(sSensorDataSnapshot485.speedSensorPulseInterval, H2A_WHEEL_METER_PER_PULSE)) >= maxChars)
		printbuf[0] = '\0';
	else
		err = 0;
	
	return err;
} /* GetWheelSpeed */


static int GetWheelSpeedEVA(const char *subadress, char *printbuf, int maxChars) {
	int err = 1;

	if(snprintf(printbuf, maxChars, "%.3f", GetProcessedSpeed(sSensorDataSnapshot485.speedSensorPulseInterval, EVA_WHEEL_METER_PER_PULSE)) >= maxChars)
		printbuf[0] = '\0';
	else
		err = 0;
	
	return err;
} /* GetWheelSpeedEVA */


static int GetWheelDistance(const char *subadress, char *printbuf, int maxChars) {
	int err = 1;

	if(snprintf(printbuf, maxChars, "%.3f", sSensorDataSnapshot485.speedSensorPositivePulsesSeen * EVA_WHEEL_METER_PER_PULSE) >= maxChars)
		printbuf[0] = '\0';
	else
		err = 0;
	
	return err;
} /* GetWheelDistance */


static int GetFuelCellVoltage(const char *subadress, char *printbuf, int maxChars) {
	int err = 1;

	if(snprintf(printbuf, maxChars, "%.3f", sSensorDataSnapshot485.adc.h2a.fcVoltageFiltered / (65536.0f * sCal.fcVoltageScale)) >= maxChars)
		printbuf[0] = '\0';
	else
		err = 0;
	
	return err;
} /* GetFuelCellVoltage */


static int GetFuelCellCurrent(const char *subadress, char *printbuf, int maxChars) {
	int err = 1;

	if(snprintf(printbuf, maxChars, "%.3f", sSensorDataSnapshot485.adc.h2a.fcCurrentFiltered / (65536.0f * sCal.fcCurrentScale)) >= maxChars)
		printbuf[0] = '\0';
	else
		err = 0;
	
	return err;
} /* GetFuelCellCurrent */


static int GetFuelCellPower(const char *subadress, char *printbuf, int maxChars) {
	int err = 1;

	if(snprintf(printbuf, maxChars, "%.3f", sSensorDataSnapshot485.adc.h2a.fcPowerFiltered / (256.0f * sCal.fcVoltageScale * sCal.fcCurrentScale)) >= maxChars)
		printbuf[0] = '\0';
	else
		err = 0;
	
	return err;
} /* GetFuelCellPower */


static int GetFuelCellEnergy(const char *subadress, char *printbuf, int maxChars) {
	int err = 1;

	if(snprintf(printbuf, maxChars, "%.3f", sSensorDataSnapshot485.adc.h2a.fcEnergy / (CYCLES_PER_SECOND * sCal.fcVoltageScale * sCal.fcCurrentScale)) >= maxChars)
		printbuf[0] = '\0';
	else
		err = 0;
	
	return err;
} /* GetFuelCellCurrent */


static int GetSupercapVoltage(const char *subadress, char *printbuf, int maxChars) {
	int err = 1;

	if(snprintf(printbuf, maxChars, "%.3f", sSensorDataSnapshot485.adc.h2a.scVoltageFiltered / (65536.0f * sCal.scVoltageScale)) >= maxChars)
		printbuf[0] = '\0';
	else
		err = 0;
	
	return err;
} /* GetSupercapVoltage */


static int GetSupercapCurrent(const char *subadress, char *printbuf, int maxChars) {
	int err = 1;

	if(snprintf(printbuf, maxChars, "%.3f", sSensorDataSnapshot485.adc.h2a.scCurrentFiltered / (65536.0f * sCal.scCurrentScale)) >= maxChars)
		printbuf[0] = '\0';
	else
		err = 0;
	
	return err;
} /* GetSupercapCurrent */


static int GetSupercapPower(const char *subadress, char *printbuf, int maxChars) {
	int err = 1;

	if(snprintf(printbuf, maxChars, "%.3f", sSensorDataSnapshot485.adc.h2a.scPowerFiltered / (256.0f * sCal.scVoltageScale * sCal.scCurrentScale)) >= maxChars)
		printbuf[0] = '\0';
	else
		err = 0;
	
	return err;
} /* GetSupercapPower */


static int GetSupercapEnergy(const char *subadress, char *printbuf, int maxChars) {
	int err = 1;

	if(snprintf(printbuf, maxChars, "%.3f", sSensorDataSnapshot485.adc.h2a.scEnergy / (CYCLES_PER_SECOND * sCal.scVoltageScale * sCal.scCurrentScale)) >= maxChars)
		printbuf[0] = '\0';
	else
		err = 0;
	
	return err;
} /* GetSupercapEnergy */


static int GetMotorVoltage(const char *subadress, char *printbuf, int maxChars) {
	int err = 1;

	if(snprintf(printbuf, maxChars, "%.3f", sSensorDataSnapshot485.motorVoltageFiltered / (65536.0f * sCal.motorVoltageScale)) >= maxChars)
		printbuf[0] = '\0';
	else
		err = 0;
	
	return err;
} /* GetMotorVoltage */


static int GetMotorCurrent(const char *subadress, char *printbuf, int maxChars) {
	int err = 1;

	if(snprintf(printbuf, maxChars, "%.3f", 
	   sSensorDataSnapshot485.motorCurrentFiltered / (65536.0f * 200.0f) 
	  /* + (0.22f / 28.4f) * sSensorDataSnapshot485.inVoltageFiltered * 22.0f */ / (65536.0f * 1000.0f)) >= maxChars)
		printbuf[0] = '\0';
	else
		err = 0;
	
	return err;
} /* GetMotorCurrent */


static int GetMotorDriverTemp(const char *subadress, char *printbuf, int maxChars) {
	int err = 1;

	if(snprintf(printbuf, maxChars, "%.3f", sSensorDataSnapshot485.driverTempFiltered / (65536.0f * 10.0f)) >= maxChars)
		printbuf[0] = '\0';
	else
		err = 0;
	
	return err;
} /* GetMotorDriverTemp */


static int GetTimeSnapshot(const char *subadress, char *printbuf, int maxChars) {
	int err = 1;

	if(snprintf(printbuf, maxChars, "%.4f", (float) sSessionCycleCountSnapshot485 / CYCLES_PER_SECOND) >= maxChars)
		printbuf[0] = '\0';
	else
		err = 0;
	
	return err;
} /* GetTimeSnapshot */


static int GetDiodeStatus(const char *subadress, char *printbuf, int maxChars) {
	int err = 1;

	if(snprintf(printbuf, maxChars, "%d,%.4f",
			!!sSensorDataSnapshot485.adc.h2a.idealDiodeState, ((float) sSensorDataSnapshot485.adc.h2a.idealDiodeTimestamp / CYCLES_PER_SECOND)) >= maxChars)
		printbuf[0] = '\0';
	else
		err = 0;
	
	return err;
} /* GetDiodeStatus */


static int GetFullPowerButtonStatus(const char *subadress, char *printbuf, int maxChars) {
	int err = 1;

	if(snprintf(printbuf, maxChars, "%d,%.4f",
			!sSensorDataSnapshot485.selFPState, ((float) sSensorDataSnapshot485.selFPTimestamp / CYCLES_PER_SECOND)) >= maxChars)
		printbuf[0] = '\0';
	else
		err = 0;
	
	return err;
} /* GetFullPowerButtonStatus */


static int GetCruiseControlButtonStatus(const char *subadress, char *printbuf, int maxChars) {
	int err = 1;

	if(snprintf(printbuf, maxChars, "%d,%.4f",
			!sSensorDataSnapshot485.selCCState, ((float) sSensorDataSnapshot485.selCCTimestamp / CYCLES_PER_SECOND)) >= maxChars)
	printbuf[0] = '\0';
	else
		err = 0;
	
	return err;
} /* GetCruiseControlButtonStatus */


static int GetUnimplementedSensor(const char *subadress, char *printbuf, int maxChars) {
	int err = 1;

	if(snprintf(printbuf, maxChars, "0") >= maxChars)
		printbuf[0] = '\0';
	else {
		strncat(printbuf, "'Unimplemented'", maxChars-(strlen(printbuf)-1));
		err = 0;
	}
	
	return err;
} /* GetUnimplementedSensor */


static void InitCoreAnalogSensors(void) {
	
	if(I_AM_EVA_L | I_AM_EVA_R) {
		AddSlaveOwnSensor(I_AM_EVA_L ? "SM01" : "SM02", GetWheelSpeedEVA, CoreAnalogTakeSnapshot485, 1);
		AddSlaveOwnSensor(I_AM_EVA_L ? "AF01" : "AF02", GetWheelDistance, NULL, 1);
	
		AddSlaveOwnSensor(I_AM_EVA_L ? "VM01" : "VM02", GetMotorVoltage, NULL, 1);		
		AddSlaveOwnSensor(I_AM_EVA_L ? "CM01" : "CM02", GetMotorCurrent, NULL, 1);
//		AddSlaveOwnSensor(IS_EVA_LEFT ? "TM01" : "TM02", GetMotorDriverTemp, NULL, 10);
	
//		AddSlaveOwnSensor(IS_EVA_LEFT ? "TS01" : "TS02", GetTimeSnapshot, NULL, 1);
	
//		AddSlaveOwnSensor(IS_EVA_LEFT ? "SG02" : "SG03", GetFullPowerButtonStatus, NULL, 5);
//		AddSlaveOwnSensor(IS_EVA_LEFT ? "SC01" : "SC02", GetCruiseControlButtonStatus, NULL, 5);
	}
	else { /* H2A */
		
		AddSlaveOwnSensor("VB01", GetFuelCellVoltage, NULL, 1);
		AddSlaveOwnSensor("CB01", GetFuelCellCurrent, NULL, 1);
		AddSlaveOwnSensor("PB01", GetFuelCellPower, NULL, 1);
		AddSlaveOwnSensor("EB01", GetFuelCellEnergy, NULL, 5);
		
		AddSlaveOwnSensor("VS01", GetSupercapVoltage, NULL, 1);
		AddSlaveOwnSensor("CS01", GetSupercapCurrent, NULL, 1);
		AddSlaveOwnSensor("PS01", GetSupercapPower, NULL, 1);
		AddSlaveOwnSensor("ES01", GetSupercapEnergy, NULL, 5);
		
		AddSlaveOwnSensor("TS01", GetTimeSnapshot, NULL, 1);
		
		AddSlaveOwnSensor("SD01", GetDiodeStatus, NULL, 10);
		if(I_AM_H2A) {
			AddSlaveOwnSensor("SM01", GetWheelSpeedH2A, CoreAnalogTakeSnapshot485, 1);
			AddSlaveOwnSensor("AF01", GetWheelDistance, NULL, 1);

			AddSlaveOwnSensor("VM01", GetMotorVoltage, NULL, 1);
			AddSlaveOwnSensor("CM01", GetMotorCurrent, NULL, 1);
			AddSlaveOwnSensor("TM01", GetMotorDriverTemp, NULL, 10);
		
			AddSlaveOwnSensor("SG02", GetFullPowerButtonStatus, NULL, 5);
			AddSlaveOwnSensor("SC01", GetCruiseControlButtonStatus, NULL, 5);
		}
	}	

//	AddSlaveOwnSensor("CV03", GetUnimplementedSensor, NULL, 0);
//	AddSlaveOwnSensor("XI03", GetUnimplementedSensor, NULL, 0);
		
} /* InitCoreAnalogSensors */


void TakeSnapshot(void) {
	
	sTakeSnapshot = 1;
	
} /* TakeSnapshotCoreAnalog */


uint8_t IsSnapshotDone(void) {
	
	return !sTakeSnapshot;
	
} /* IsSnapshotCoreAnalogDone */


uint32_t GetSessionCycleCount(void) {
	
	uint32_t res, prev;
	
	res = sSessionCycleCount;
	
	do {
		prev = res;
		res = sSessionCycleCount;
	} while (res != prev);
	
	return res;
	
} /* GetSessionCycleCount */


static float GetProcessedSpeed(int32_t speedSensorPulseInterval, float wheelMeterPerPulse) {
	
	float res;
	
	if(speedSensorPulseInterval) {
		res = wheelMeterPerPulse * WHEEL_MS_TO_KMH * CYCLES_PER_SECOND / (speedSensorPulseInterval / 65536.0f);
		if(res < SPEEDSENSOR_MIN_SPEED)
		res = 0.0f;
	}
	else
	res = 0.0f;

	return res;
} /* GetProcessedSpeed */

float GetSpeedfromMotorVoltage(void){	// Only for testing
	float res;
	
	res = (sSensorDataSnapshot.motorVoltageFiltered / (65536.0f * sCal.motorVoltageScale)) * (I_AM_H2A ? H2A_MOTOR_SPEED_CONSTANT : EVA_MOTOR_SPEED_CONSTANT) * (I_AM_H2A ? H2A_GEAR_RATIO : EVA_GEAR_RATIO ) * WHEEL_MS_TO_KMH / (60.0f);
	
	if (res < SPEEDSENSOR_MIN_SPEED) res = 0;
	
	return res;
}

void PrintCSV_H2A(FILE *fp) {
	/* Assume the calling code has already initiated a snapshot */
	while(!(IsSnapshotDone())) ; /* Wait for the snapshot to be taken */

	fprintf(fp, ">03|03:%.4f,%.3f,%.3f,%.2f,%.0f,%.3f,%.3f,%.2f,%.0f,%.3f,%.3f,%.1f,%.0f,%.3f,%.3f,%.1f,%.0f,%.2f,%.0f,%.0f,%.3f,%.2f,%.4f,%.4f,%d,%.3f,%d,%.3f,%d,%.3f,%d,%.3f,%d,%.3f",
		(float) sSessionCycleCountSnapshot / CYCLES_PER_SECOND,
		sSensorDataSnapshot.adc.h2a.fcVoltageFiltered / (65536.0f * sCal.fcVoltageScale),
		sSensorDataSnapshot.adc.h2a.fcCurrentFiltered / (65536.0f * sCal.fcCurrentScale),
		sSensorDataSnapshot.adc.h2a.fcPowerFiltered / (256.0f * sCal.fcVoltageScale * sCal.fcCurrentScale), /* Power */
		sSensorDataSnapshot.adc.h2a.fcEnergy / (CYCLES_PER_SECOND * sCal.fcVoltageScale * sCal.fcCurrentScale), /* Energy */
		sSensorDataSnapshot.adc.h2a.scVoltageFiltered / (65536.0f * sCal.scVoltageScale),
		sSensorDataSnapshot.adc.h2a.scCurrentFiltered / (65536.0f * sCal.scCurrentScale),
		sSensorDataSnapshot.adc.h2a.scPowerFiltered / (256.0f * sCal.scVoltageScale * sCal.scCurrentScale), /* Power */
		sSensorDataSnapshot.adc.h2a.scEnergy / (CYCLES_PER_SECOND * sCal.scVoltageScale * sCal.scCurrentScale), /* Energy */
		sSensorDataSnapshot.motorVoltageFiltered / (65536.0f * sCal.motorVoltageScale),
		sSensorDataSnapshot.motorCurrentFiltered / (65536.0f * sCal.motorCurrentScale),
		sSensorDataSnapshot.motorPowerFiltered / (256.0f * sCal.motorVoltageScale * sCal.motorCurrentScale),
		sSensorDataSnapshot.motorEnergy / (CYCLES_PER_SECOND * sCal.motorVoltageScale * sCal.motorCurrentScale),
		sSensorDataSnapshot.inVoltageFiltered / (65536.0f * sCal.inVoltageScale),
		sSensorDataSnapshot.inCurrentFiltered / (65536.0f * sCal.inCurrentScale),
		sSensorDataSnapshot.inPowerFiltered / (256.0f * sCal.inVoltageScale * sCal.inCurrentScale),
		sSensorDataSnapshot.inEnergy / (CYCLES_PER_SECOND * sCal.inVoltageScale * sCal.inCurrentScale),
		sSensorDataSnapshot.driverTempFiltered / (65536.0f * 10.0f),
		sSensorDataSnapshot.pwmFrequency / (256.0f * PWM_FREQ_SCALE),
		sSensorDataSnapshot.pwmDutyCycle / (256.0f * PWM_DC_SCALE),
		GetProcessedSpeed(sSensorDataSnapshot.speedSensorPulseInterval, H2A_WHEEL_METER_PER_PULSE),
		sSensorDataSnapshot.speedSensorPositivePulsesSeen * H2A_WHEEL_METER_PER_PULSE,
		(float) sSensorDataSnapshot.speedSensorLastValidInterval / CYCLES_PER_SECOND,
		(float) sSensorDataSnapshot.speedSensorPreviousValidEdgeTimestamp / CYCLES_PER_SECOND,
		(int16_t)!!sSensorDataSnapshot.adc.h2a.idealDiodeState,
		((float) sSensorDataSnapshot.adc.h2a.idealDiodeTimestamp / CYCLES_PER_SECOND),
		(int16_t)!sSensorDataSnapshot.selFPState,
		((float) sSensorDataSnapshot.selFPTimestamp / CYCLES_PER_SECOND),
		(int16_t)!sSensorDataSnapshot.selCCState,
		((float) sSensorDataSnapshot.selCCTimestamp / CYCLES_PER_SECOND),
		(int16_t)sSensorDataSnapshot.ccPower,
		sSensorDataSnapshot.ccTargetSpeed ? (H2A_WHEEL_METER_PER_PULSE * WHEEL_MS_TO_KMH * CYCLES_PER_SECOND / (sSensorDataSnapshot.ccTargetSpeed  / 65536.0f)) : 0.0f,
		(int16_t)!sSensorDataSnapshot.selCC2State,
		((float) sSensorDataSnapshot.selCC2Timestamp / CYCLES_PER_SECOND)
		);
	
} /* PrintCSV_H2A */


void PrintCSV_EVA(FILE *fp) {
	/* Assume the calling code has already initiated a snapshot */	
	while(!(IsSnapshotDone())) ; /* Wait for the snapshot to be taken */
	fprintf(fp, ">04|06:%.4f,%.3f,%.3f,%.2f,%.2f,%.3f,%.3f,%.1f,%.0f,%.3f,%.3f,%.1f,%.0f,%.2f,%.0f,%.0f,%.3f,%.2f,%.4f,%.4f,%d,%.3f,%d,%.3f,%d,%.3f,%d,%.3f,",
	(float) sSessionCycleCountSnapshot / CYCLES_PER_SECOND,
	(float) sSensorDataSnapshot.inVoltageMin / (sCal.inVoltageScale),
	(float) sSensorDataSnapshot.inVoltageMax / (sCal.inVoltageScale),
	(float) sSensorDataSnapshot.inCurrentMin / (sCal.inCurrentScale),
	(float) sSensorDataSnapshot.inCurrentMax / (sCal.inCurrentScale),
	sSensorDataSnapshot.motorVoltageFiltered / (65536.0f * sCal.motorVoltageScale),
	sSensorDataSnapshot.motorCurrentFiltered / (65536.0f * sCal.motorCurrentScale),
	sSensorDataSnapshot.motorPowerFiltered / (256.0f * sCal.motorVoltageScale * sCal.motorCurrentScale),
	sSensorDataSnapshot.motorEnergy / (CYCLES_PER_SECOND * sCal.motorVoltageScale * sCal.motorCurrentScale),
	sSensorDataSnapshot.inVoltageFiltered / (65536.0f * sCal.inVoltageScale),
	sSensorDataSnapshot.inCurrentFiltered / (65536.0f * sCal.inCurrentScale),
	sSensorDataSnapshot.inPowerFiltered / (256.0f * sCal.inVoltageScale * sCal.inCurrentScale),
	sSensorDataSnapshot.inEnergy / (CYCLES_PER_SECOND * sCal.inVoltageScale * sCal.inCurrentScale),
	sSensorDataSnapshot.driverTempFiltered / (65536.0f * 10.0f),
	sSensorDataSnapshot.pwmFrequency / (256.0f * PWM_FREQ_SCALE),
	sSensorDataSnapshot.pwmDutyCycle / (256.0f * PWM_DC_SCALE),
	GetProcessedSpeed(sSensorDataSnapshot.speedSensorPulseInterval, EVA_WHEEL_METER_PER_PULSE),
	sSensorDataSnapshot.speedSensorPositivePulsesSeen * EVA_WHEEL_METER_PER_PULSE,
	(float) sSensorDataSnapshot.speedSensorLastValidInterval / CYCLES_PER_SECOND,
	(float) sSensorDataSnapshot.speedSensorPreviousValidEdgeTimestamp / CYCLES_PER_SECOND,
	(int16_t)!sSensorDataSnapshot.selFPState,
	((float) sSensorDataSnapshot.selFPTimestamp / CYCLES_PER_SECOND),
	(int16_t)!sSensorDataSnapshot.selCCState,
	((float) sSensorDataSnapshot.selCCTimestamp / CYCLES_PER_SECOND),
	(int16_t)sSensorDataSnapshot.ccPower,
	sSensorDataSnapshot.ccTargetSpeed ? (EVA_WHEEL_METER_PER_PULSE * WHEEL_MS_TO_KMH * CYCLES_PER_SECOND / (sSensorDataSnapshot.ccTargetSpeed  / 65536.0f)) : 0.0f,
	(int16_t)!sSensorDataSnapshot.selCC2State,
	((float) sSensorDataSnapshot.selCC2Timestamp / CYCLES_PER_SECOND)
	);
	
} /* PrintCSV_EVA */

// void PrintCSV_EVA(FILE *fp) {
// 	/* Assume the calling code has already initiated a snapshot */
// 	while(!(IsSnapshotDone())) ; /* Wait for the snapshot to be taken */
// 	fprintf(fp, ">04|05:%.4f,%.3f,%.3f,%.2f,%.2f,%.3f,%.3f,%.1f,%.0f,%.3f,%.3f,%.1f,%.0f,%.2f,%.0f,%.0f,%.3f,%.2f,%.4f,%.4f,%d,%.3f,%d,%.3f,%d,%.3f,%d,%.3f,",
// 	(float) sSessionCycleCountSnapshot / CYCLES_PER_SECOND,
// 	sSensorDataSnapshot.adc.eva.angSenseFiltered / (65536.0f),
// 	sSensorDataSnapshot.adc.eva.angFSFiltered / (65536.0f),
// 	sSensorDataSnapshot.adc.eva.motorTempFrontFiltered / (65536.0f * MOTORTEMP_SCALE),
// 	sSensorDataSnapshot.adc.eva.motorTempRearFiltered / (65536.0f * MOTORTEMP_SCALE),
// 	sSensorDataSnapshot.motorVoltageFiltered / (65536.0f * sCal.motorVoltageScale),
// 	sSensorDataSnapshot.motorCurrentFiltered / (65536.0f * sCal.motorCurrentScale),
// 	sSensorDataSnapshot.motorPowerFiltered / (256.0f * sCal.motorVoltageScale * sCal.motorCurrentScale),
// 	sSensorDataSnapshot.motorEnergy / (CYCLES_PER_SECOND * sCal.motorVoltageScale * sCal.motorCurrentScale),
// 	sSensorDataSnapshot.inVoltageFiltered / (65536.0f * sCal.inVoltageScale),
// 	sSensorDataSnapshot.inCurrentFiltered / (65536.0f * sCal.inCurrentScale),
// 	sSensorDataSnapshot.inPowerFiltered / (256.0f * sCal.inVoltageScale * sCal.inCurrentScale),
// 	sSensorDataSnapshot.inEnergy / (CYCLES_PER_SECOND * sCal.inVoltageScale * sCal.inCurrentScale),
// 	sSensorDataSnapshot.driverTempFiltered / (65536.0f * 10.0f),
// 	sSensorDataSnapshot.pwmFrequency / (256.0f * PWM_FREQ_SCALE),
// 	sSensorDataSnapshot.pwmDutyCycle / (256.0f * PWM_DC_SCALE),
// 	GetProcessedSpeed(sSensorDataSnapshot.speedSensorPulseInterval, EVA_WHEEL_METER_PER_PULSE),
// 	sSensorDataSnapshot.speedSensorPositivePulsesSeen * EVA_WHEEL_METER_PER_PULSE,
// 	(float) sSensorDataSnapshot.speedSensorLastValidInterval / CYCLES_PER_SECOND,
// 	(float) sSensorDataSnapshot.speedSensorPreviousValidEdgeTimestamp / CYCLES_PER_SECOND,
// 	(int16_t)!sSensorDataSnapshot.selFPState,
// 	((float) sSensorDataSnapshot.selFPTimestamp / CYCLES_PER_SECOND),
// 	(int16_t)!sSensorDataSnapshot.selCCState,
// 	((float) sSensorDataSnapshot.selCCTimestamp / CYCLES_PER_SECOND),
// 	(int16_t)sSensorDataSnapshot.ccPower,
// 	sSensorDataSnapshot.ccTargetSpeed ? (EVA_WHEEL_METER_PER_PULSE * WHEEL_MS_TO_KMH * CYCLES_PER_SECOND / (sSensorDataSnapshot.ccTargetSpeed  / 65536.0f)) : 0.0f,
// 	(int16_t)!sSensorDataSnapshot.selCC2State,
// 	((float) sSensorDataSnapshot.selCC2Timestamp / CYCLES_PER_SECOND)
// 	);
// 	
// 	} /* PrintCSV_EVA */

void PrintResetHeader(FILE *fp) {
	
	fprintf(fp, "# HvA SECOM Motor driver board ");
	PrintBoardType(fp);
	fprintf(fp, "busID %d cpu ", GetBusID());
	PrintProcessorID(fp);
	fprintf(fp, "# code " __DATE__ " " __TIME__ "\r\n");
	
} /* PrintResetHeader */


static tCoreAnalogSensorData sSensorData = { .speedSensorLastValidInterval = SPEEDSENSOR_MAX_INTERVAL, .inVoltageMin = 100, .inCurrentMin = 100 };

#define SET_CC_DRIVE(x) do { \
	if((x) == 0) { PORTE.OUTCLR = CC_PINS | CC_TURBO_BOOST; } \
	else if((x) == CC_MAX_POWER) { PORTE.OUTSET = CC_MAX_POWER; PORTE.OUTCLR = CC_TURBO_BOOST; } \
	else if((x) == CC_TURBO_BOOST) { PORTE.OUTSET = CC_MAX_POWER | CC_TURBO_BOOST; } \
	else if((x) == REGBRAKE_LEVEL) { PORTE.OUTSET = REGBRAKE_PIN; PORTE.OUTCLR = CC_PINS; } \
	else { PORTE.OUTSET = ~(x) & CC_PINS;  PORTE.OUTCLR = CC_TURBO_BOOST | ((x) & CC_PINS); } \
} while(0)

static inline void ISRReadADC_H2A(void) {

	int16_t fcVoltageSample = ADCA.CH0RES - sCal.fcVoltageOffset, fcCurrentSample, scVoltageSample, scCurrentSample;
	int32_t fcPower, scPower;

	uint8_t idealDiodePin = PORTE.IN & PIN4_bm;
	
	FILTER32(fcVoltageSample, sSensorData.adc.h2a.fcVoltageFiltered);
	
	while(!(ADCB.CH0.INTFLAGS & 0x01)) ; /* Should not be necessary, as ADCB.CH0 is expected to be done simultaneously with ADCA.CH0 */
	ADCB.CH0.INTFLAGS = 0x01;
	fcCurrentSample = ADCB.CH0RES - sCal.fcCurrentOffset;
	FILTER32(fcCurrentSample, sSensorData.adc.h2a.fcCurrentFiltered);
	
	fcPower = ((int32_t) fcVoltageSample) * ((int32_t) fcCurrentSample);
	FILTER32PWR(fcPower, sSensorData.adc.h2a.fcPowerFiltered);
	sSensorData.adc.h2a.fcEnergy += fcPower;
	
	while(!(ADCA.CH1.INTFLAGS & 0x01)) ; /* Should also not be necessary, by this time the conversion should be long over */
	ADCA.CH1.INTFLAGS = 0x01;
	scVoltageSample = ADCA.CH1RES - sCal.scVoltageOffset;
	FILTER32(scVoltageSample, sSensorData.adc.h2a.scVoltageFiltered);
	
	while(!(ADCB.CH1.INTFLAGS & 0x01)) ;
	ADCB.CH1.INTFLAGS = 0x01;
	scCurrentSample = ADCB.CH1RES - sCal.scCurrentOffset;
	FILTER32(scCurrentSample, sSensorData.adc.h2a.scCurrentFiltered);

	scPower = ((int32_t) scVoltageSample) * ((int32_t) scCurrentSample);
	FILTER32PWR(scPower, sSensorData.adc.h2a.scPowerFiltered);
	sSensorData.adc.h2a.scEnergy += scPower;
	
	if(sSensorData.adc.h2a.idealDiodeState != idealDiodePin) {
		sSensorData.adc.h2a.idealDiodeState = idealDiodePin;
		sSensorData.adc.h2a.idealDiodeTimestamp = sSessionCycleCount;
	}
	
} /* ISRReadADC_H2A */


static inline void ISRReadADC_EVA(void) {
	
	int16_t motorTempFront = ADCA.CH0RES, motorTempRear, angSample, angFSSample;

	FILTER32(motorTempFront, sSensorData.adc.eva.motorTempFrontFiltered);
	
	while(!(ADCB.CH0.INTFLAGS & 0x01)) ; /* Should not be necessary, as ADCB.CH0 is expected to be done simultaneously with ADCA.CH0 */
	ADCB.CH0.INTFLAGS = 0x01;
	motorTempRear = ADCB.CH0RES;
	FILTER32(motorTempRear, sSensorData.adc.eva.motorTempRearFiltered);
	
	while(!(ADCA.CH1.INTFLAGS & 0x01)) ;
	ADCA.CH1.INTFLAGS = 0x01;
	angSample = ADCA.CH1RES;
	FILTER32(angSample, sSensorData.adc.eva.angSenseFiltered);
	
	while(!(ADCB.CH1.INTFLAGS & 0x01)) ;
	ADCB.CH1.INTFLAGS = 0x01;
	angFSSample = ADCB.CH1RES;
	FILTER32(angFSSample, sSensorData.adc.eva.angFSFiltered);
	
} /* ISRReadADC_EVA */


ISR(ADCA_CH0_vect) {
	
	static uint8_t sSamplingVin;
	
	static uint8_t sPrevPWMCycles;
	
	static uint8_t sCCIsOn;
	static uint16_t sCCRunTimer;
	
	static uint8_t sSpeedSensorPreviousState, sSpeedSensorPosDeglitchCounter, sSpeedSensorNegDeglitchCounter;

	static int32_t sCCPrevPulseInterval;

	int16_t spRawSample, driverTempSample, motorVoltageSample, motorCurrentSample, inVoltageSample, inCurrentSample;
	int32_t inPower, motorPower;
	uint8_t selCCPin = PORTC.IN & PIN2_bm, selCC2Pin = PORTC.IN & PIN5_bm, selFPPin = PORTC.IN & PIN4_bm, pwmEn = !(PORTC.IN & PIN1_bm), pwm = PORTC.IN & PIN6_bm;
	uint8_t curPWMCycles = TCC1.CNTL;
	
	if(I_AM_EVA_L || I_AM_EVA_R)
		ISRReadADC_EVA();
	else
		ISRReadADC_H2A();
	
	while(!(ADCA.CH2.INTFLAGS & 0x01)) ;
	ADCA.CH2.INTFLAGS = 0x01;
	spRawSample = ADCA.CH2RES;
	if(spRawSample < SPEEDSENSOR_MIDWAY_VAL) {
		sSpeedSensorPosDeglitchCounter = 0;
		if(sSpeedSensorPreviousState && ++sSpeedSensorNegDeglitchCounter >= SPEEDSENSOR_DEGLITCH)
			sSpeedSensorPreviousState = 0;
	}
	else {
		sSpeedSensorNegDeglitchCounter = 0;
		if(!sSpeedSensorPreviousState && ++sSpeedSensorPosDeglitchCounter >= SPEEDSENSOR_DEGLITCH) {
			sSpeedSensorPreviousState = 1;
			sSensorData.speedSensorPositivePulsesSeen++;
			if(sSessionCycleCount - sSensorData.speedSensorPreviousValidEdgeTimestamp < SPEEDSENSOR_MAX_INTERVAL)
				sSensorData.speedSensorLastValidInterval = sSessionCycleCount - sSensorData.speedSensorPreviousValidEdgeTimestamp;
			else
				sSensorData.speedSensorLastValidInterval = SPEEDSENSOR_MAX_INTERVAL;
			sSensorData.speedSensorPreviousValidEdgeTimestamp = sSessionCycleCount;
		}
	}
	
	if(sSessionCycleCount - sSensorData.speedSensorPreviousValidEdgeTimestamp > (uint32_t) SPEEDSENSOR_MAX_INTERVAL)
		sSensorData.speedSensorLastValidInterval = SPEEDSENSOR_MAX_INTERVAL;
	else if(sSessionCycleCount - sSensorData.speedSensorPreviousValidEdgeTimestamp > (uint32_t) sSensorData.speedSensorLastValidInterval)
		sSensorData.speedSensorLastValidInterval = sSessionCycleCount - sSensorData.speedSensorPreviousValidEdgeTimestamp;
		
	FILTER32(sSensorData.speedSensorLastValidInterval, sSensorData.speedSensorPulseInterval);
	//sSensorData.speedSensorPulseInterval = (int32_t) ((I_AM_H2A ? H2A_WHEEL_METER_PER_PULSE : EVA_WHEEL_METER_PER_PULSE) * WHEEL_MS_TO_KMH * CYCLES_PER_SECOND) / (GetSpeedfromMotorVoltage() / 65536.0f);	// Alleen voor in de testopstelling
	
	while(!(ADCB.CH2.INTFLAGS & 0x01)) ;
	ADCB.CH2.INTFLAGS = 0x01;
	driverTempSample = ADCB.CH2RES;
	FILTER32(driverTempSample, sSensorData.driverTempFiltered);
	
	while(!(ADCA.CH3.INTFLAGS & 0x01)) ;
	ADCA.CH3.INTFLAGS = 0x01;
	while(!(ADCB.CH3.INTFLAGS & 0x01)) ;
	ADCB.CH3.INTFLAGS = 0x01;
	
	
	if(sSamplingVin) {
		inVoltageSample = ADCA.CH3RES - sCal.inVoltageOffset;
		// Hold min/max value for MINMAX_RESET_PERIOD
		if(inVoltageSample < sSensorData.inVoltageMin){
			sSensorData.inVoltageMin = inVoltageSample;
			sSensorData.inVoltageMinTimestamp = sSessionCycleCount;
		} else if(sSessionCycleCount - sSensorData.inVoltageMinTimestamp >= MINMAX_RESET_PERIOD){
			sSensorData.inVoltageMin = inVoltageSample;
			sSensorData.inVoltageMinTimestamp = sSessionCycleCount;
		}	
			
		if(inVoltageSample > sSensorData.inVoltageMax){
			sSensorData.inVoltageMax = inVoltageSample;
			sSensorData.inVoltageMaxTimestamp = sSessionCycleCount;
		} else if(sSessionCycleCount - sSensorData.inVoltageMaxTimestamp >= MINMAX_RESET_PERIOD){
			sSensorData.inVoltageMax = inVoltageSample;
			sSensorData.inVoltageMaxTimestamp = sSessionCycleCount;
		}
		FILTER32(inVoltageSample, sSensorData.inVoltageFiltered);
		
		inCurrentSample = ADCB.CH3RES - sCal.inCurrentOffset;
		// Hold min/max value for MINMAX_RESET_PERIOD
		if(inCurrentSample < sSensorData.inCurrentMin){
			sSensorData.inCurrentMin = inCurrentSample;
			sSensorData.inCurrentMinTimestamp = sSessionCycleCount;
			} else if(sSessionCycleCount - sSensorData.inCurrentMinTimestamp >= MINMAX_RESET_PERIOD){
			sSensorData.inCurrentMin = inCurrentSample;
			sSensorData.inCurrentMinTimestamp = sSessionCycleCount;
		}
		if(inCurrentSample > sSensorData.inCurrentMax){
			sSensorData.inCurrentMax = inCurrentSample;
			sSensorData.inCurrentMaxTimestamp = sSessionCycleCount;
			} else if(sSessionCycleCount - sSensorData.inCurrentMaxTimestamp >= MINMAX_RESET_PERIOD){
			sSensorData.inCurrentMax = inCurrentSample;
			sSensorData.inCurrentMaxTimestamp = sSessionCycleCount;
		}
		FILTER32(inCurrentSample, sSensorData.inCurrentFiltered);
		
		inPower = ((int32_t) inVoltageSample) * ((int32_t) inCurrentSample);
		FILTER32PWR(inPower, sSensorData.inPowerFiltered);
		sSensorData.inEnergy +=  2 * inPower;
		ADCA.CH3.MUXCTRL = ADC_CH_MUXPOS_PIN12_gc | ADC_CH_MUXNEG_PIN1_gc; /* Next cycle, measure ADC_VMOTOR on PB4 vs V33/2 on PA1 */
		ADCB.CH3.MUXCTRL = ADC_CH_MUXPOS_PIN5_gc | ADC_CH_MUXNEG_PIN1_gc; /* Next cycle, measure ADC_IMOTOR on PB5 vs V33/2 on PB1 */
	}
	else {
		motorVoltageSample = ADCA.CH3RES - sCal.motorVoltageOffset;
		FILTER32(motorVoltageSample, sSensorData.motorVoltageFiltered);	
		motorCurrentSample = ADCB.CH3RES - sCal.motorCurrentOffset;
		FILTER32(motorCurrentSample, sSensorData.motorCurrentFiltered);
		motorPower = ((int32_t) motorVoltageSample) * ((int32_t) motorCurrentSample);
		FILTER32PWR(motorPower, sSensorData.motorPowerFiltered);
		sSensorData.motorEnergy += 2 * motorPower;
		ADCA.CH3.MUXCTRL = ADC_CH_MUXPOS_PIN11_gc | ADC_CH_MUXNEG_PIN1_gc; /* Next cycle, measure ADC_Vin on PB3 vs V33/2 on PA1 */
		ADCB.CH3.MUXCTRL = ADC_CH_MUXPOS_PIN7_gc | ADC_CH_MUXNEG_PIN1_gc; /* Next cycle, measure Iin on PB7 vs V33/2 on PB1 */
	}
	sSamplingVin = !sSamplingVin;

	/* PWM frequency and duty cycle */
	if(pwmEn) {
		sSensorData.pwmDutyCycle -= sSensorData.pwmDutyCycle >> 8;
		if(pwm)
			sSensorData.pwmDutyCycle += PWM_DC_FS;
		sSensorData.pwmFrequency -= sSensorData.pwmFrequency >> 8;
		sSensorData.pwmFrequency += (curPWMCycles - sPrevPWMCycles) << PWM_FREQ_FILTER_SHIFT;
	}
	else {
		sSensorData.pwmDutyCycle = 0;
		sSensorData.pwmFrequency = 0;
	}
	sPrevPWMCycles = curPWMCycles;
	
	/* Pin states */
	if(sSensorData.selFPState != selFPPin) {
		sSensorData.selFPState = selFPPin;
		sSensorData.selFPTimestamp = sSessionCycleCount;
	}
	
	if(sSensorData.selCCState != selCCPin) {
		sSensorData.selCCState = selCCPin;
		sSensorData.selCCTimestamp = sSessionCycleCount;
		/* Did CC just get enabled? */
		if(!selCCPin &&  (sSensorData.speedSensorPulseInterval < (((int32_t) (I_AM_H2A ? H2A_CC_MAX_INTERVAL : EVA_CC_MAX_INTERVAL)) << 16) ) ) {		// Only activate when faster than minimum.
			sCCIsOn = 1;
			sSensorData.ccPower = CC_DEFAULT_POWER;
			
			sCCPrevPulseInterval = sSensorData.ccTargetSpeed = sSensorData.speedSensorPulseInterval;		// Set target speed to current speed
						
			
			sCCRunTimer = CC_REG_CYCLES;
			SET_CC_DRIVE(sSensorData.ccPower);
		}
		else
			sCCIsOn = 0;
	}
	
	if(sSensorData.selCC2State != selCC2Pin) {
		sSensorData.selCC2State = selCC2Pin;
		sSensorData.selCC2Timestamp = sSessionCycleCount;
		//Did RegBraking just get enabled?
		if(!selCC2Pin && I_AM_EVA) SET_CC_DRIVE(REGBRAKE_LEVEL);	
		
		//Did CC2 (boost) just get enabled? Only H2A
		if(!selCC2Pin && I_AM_H2A) {
			sCCIsOn = 1;
			sCCPrevPulseInterval = sSensorData.speedSensorPulseInterval;
			if(sSensorData.speedSensorPulseInterval > sCC2MinSpeed) { // Driving below minimum initial CC2 speed limit
				sSensorData.ccTargetSpeed = sCC2MinSpeed;
				sSensorData.ccPower = CC_MAX_POWER;
			}
			else {
				sSensorData.ccTargetSpeed = sSensorData.speedSensorPulseInterval;
				sSensorData.ccPower = CC_DEFAULT_POWER;
			}
			sCCRunTimer = CC_REG_CYCLES;
			SET_CC_DRIVE(sSensorData.ccPower);
		}
		else
			sCCIsOn = 0;
	}

	if(sCCIsOn && !--sCCRunTimer) {
		
		if(!selCC2Pin && sCC2TargetSpeedUpdate) {
			sSensorData.ccTargetSpeed = sCC2TargetSpeed;
			// Possibly adjust power levels here too, for faster convergence?
			sCC2TargetSpeedUpdate = 0;
		}
		if((sSensorData.speedSensorPulseInterval > sSensorData.ccTargetSpeed)
			&& (sSensorData.speedSensorPulseInterval > sCCPrevPulseInterval) 
			&& (sSensorData.ccPower < (!selCC2Pin ? CC_TURBO_BOOST : CC_MAX_POWER)))
				sSensorData.ccPower++;
		else if((sSensorData.speedSensorPulseInterval < sSensorData.ccTargetSpeed)
			&& (sSensorData.speedSensorPulseInterval < sCCPrevPulseInterval)
			&& (sSensorData.ccPower > 0))
				sSensorData.ccPower--;
		SET_CC_DRIVE(sSensorData.ccPower);
		sCCRunTimer = CC_REG_CYCLES;
		sCCPrevPulseInterval = sSensorData.speedSensorPulseInterval;
	}

	
	if(sTakeSnapshot485) {
		sSensorDataSnapshot485 = sSensorData;
		sSessionCycleCountSnapshot485 = sSessionCycleCount;
		
		sTakeSnapshot485 = 0;
	}
	else if(sTakeSnapshot) { 
		sSensorDataSnapshot = sSensorData;
		sSessionCycleCountSnapshot = sSessionCycleCount;
		
		sTakeSnapshot = 0;
	}

	sSessionCycleCount++;
		
} /* ISR(ADCA_CH0_vect) */

