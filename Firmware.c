/*
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * <joerg@FreeBSD.ORG> wrote this file.  As long as you retain this notice you
 * can do whatever you want with this stuff. If we meet some day, and you think
 * this stuff is worth it, you can buy me a beer in return.        Joerg Wunsch
 * ----------------------------------------------------------------------------
 * $Id: largedemo.c,v 1.1.2.3 2007/01/19 22:20:27 joerg_wunsch Exp $
 */

/* Target manipulation at SPIN U70 setup 
 *      M. Ukhanov 2010.03.04
 */
 
#include <stdint.h>
#include <stdlib.h>

#define F_CPU 16000000UL	/* CPU clock in Hertz */

#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <util/delay.h>

/* Part 1: Macro definitions */

#if !defined(__AVR_ATmega16__)
#  error "Unsupported MCU type"
#endif

#if !defined(HAVE_ADC)
#  define HAVE_ADC 1
#endif

//#define DEBUG 1
#undef DEBUG

#include "Firmware.h"


#define CONTROL_PORT PORTD
#define CONTROL_DDR  DDRD
#define CONTROL_PIN  PD7

#define BEEPER_PORT PORTC
#define BEEPER_DDR  DDRC
#define BEEPER_PIN  PC5

#define TOLERANCE 4   // +- 4 ADC counts for the target positioning

/*
 * Things related to time and timers handling
 */

#define REFRESH_TIMER(TIMER) {							\
	if(TIMER && (systime > TIMER)) TIMER = TIMER_FLAG;	\
}

#define SET_TIMER(TIMER,INTERVAL_MS) {				\
	if(TIMER == 0) TIMER = systime + INTERVAL_MS;	\
}

#define GET_CURRENT_FILTER (IS_RELAY_ON(FilterLimitOut) + 2*IS_RELAY_ON(FilterLimitIn)) 
/*
 * Timer1 overflow interrupt will be called with F_CPU / 2048
 * frequency.  This interrupt routine further divides that value by SYS_CLOCK,
 * resulting in an internal update interval of approx. 10 ms.
 */

#define SYS_CLOCK 	70UL	// Each timer1 interrupt occurs every 1/(16 Mhz / 0x03FF = 15640) sec
#define ONE_SECOND 	(SYS_CLOCK*120) 		// 1 sec
#define ONE_MINUTE	(ONE_SECOND*60)	// 1 min

#define SYSTIME_T uint32_t
#define TIMER_FLAG 0xFFFFFFFF

/* Part 2: Variable definitions */

uint8_t Target1Select,Target2Select,TargetRelayUp,TargetRelayDown,TargetLimitUp,TargetLimitDown;
uint8_t FilterLimitIn,FilterLimitOut,FilterRelayOut,FilterRelayIn,FilterCurrent,FilterRequested;
uint8_t TargetCurrent,TargetRequested,TargetActual,ManualAuto;
SYSTIME_T TimerBeep;
uint16_t Urequested;
uint8_t IsFlashing;
uint8_t ItWasManual;
/*
 * Bits that are set inside interrupt routines, and watched outside in
 * the program's main loop.
 */
volatile SYSTIME_T systime;       	// Number of software clocks (see also ISR(TIMER1_OVF_vect))
volatile uint16_t Uadc;					// Last ADC voltage measurement 
volatile char rxbuff;                   /* Last character read from the UART. */
volatile struct
{
  uint8_t tmr_int: 1;
  uint8_t adc_int: 1;
  uint8_t  rx_int: 1;
}
intflags;

uint8_t calc_crc(char* str,uint8_t len);
uint8_t h2uchar(char *buf);
char* uchar2h(uint8_t i);
void putchr(uint8_t c);
void write_binary(const char *s, uint8_t len);
void printstr(const char *s);
void printstr_p(const char *s);
uint8_t IS_RELAY_ON(uint8_t Relay);

//---------------------------------------------------------------------------
void RELAY_ON(uint8_t Relay)
{
	if(((Relay)&0x10) && ((Relay)&0x20)) 	PORTD |= _BV(GET_PIN(Relay));
	else 	if((Relay)&0x20) 				PORTC |= _BV(GET_PIN(Relay));
	else 	if((Relay)&0x10) 				PORTB |= _BV(GET_PIN(Relay));
	else 									PORTA |= _BV(GET_PIN(Relay));
}
//---------------------------------------------------------------------------

void RELAY_OFF(uint8_t Relay)
{
	if(((Relay)&0x10) && ((Relay)&0x20))	PORTD &= ~(_BV(GET_PIN(Relay)));
	else 	if((Relay)&0x20) 				PORTC &= ~(_BV(GET_PIN(Relay)));
	else 	if((Relay)&0x10) 				PORTB &= ~(_BV(GET_PIN(Relay)));
	else 									PORTA &= ~(_BV(GET_PIN(Relay)));
}
//---------------------------------------------------------------------------


uint8_t IS_RELAY_ON(uint8_t Relay)
{
uint8_t r;

	if(((Relay)&0x10) && ((Relay)&0x20))	r = PIND;
	else 	if((Relay)&0x20) 				r = PINC;
	else 	if((Relay)&0x10) 				r = PINB;
	else 									r = PINA;
	return (r & _BV(GET_PIN(Relay))) == 0 ? 1 : 0;
}
//---------------------------------------------------------------------------

#define SELECT_TARGET1 { 		\
	RELAY_OFF(Target2Select);	\
	RELAY_ON(Target1Select); 	\
	_delay_ms(500.); 			\
	RELAY_OFF(Target1Select); 	\
	wdt_reset();				\
}

#define SELECT_TARGET2 { 		\
	RELAY_OFF(Target1Select);	\
	RELAY_ON(Target2Select); 	\
	_delay_ms(500.); 			\
	RELAY_OFF(Target2Select); 	\
	wdt_reset();				\
}

void MoveTargetOut()
{// if target is in the beam move it out.

	RELAY_OFF(TargetRelayUp);
	if(IS_RELAY_ON(TargetLimitDown)) return;
	RELAY_ON(TargetRelayDown);
	while(!IS_RELAY_ON(TargetLimitDown))
	{
		SET_TIMER(TimerBeep, (ONE_SECOND/2));
		_delay_ms(500.);   // wait for half a second
		wdt_reset();		
	}
	intflags.adc_int = 0;
	intflags.tmr_int = 0;
	intflags.rx_int = 0;  // alas! the device will loose a message if any.
	RELAY_OFF(TargetRelayDown);
}
//---------------------------------------------------------------------------

void
TargetCare()
{
// Take care of the targets: they must not be both in the beam.
	// Look at the first target
	SELECT_TARGET1;
	
	if(!IS_RELAY_ON(TargetLimitDown))
	{	// The first target is in the beam. Look at the second one.
		SELECT_TARGET2;
		MoveTargetOut();
	}
}
//---------------------------------------------------------------------------

void 
Algorithm()
{
/*	This function is called every ~7 ms and realizes the relay manipulation algorithm.
 *  The algorithm does:
 *
 *	1. Checks the status of AUTO/MANUAL switch
 *
 *	2. Manual mode
 *		2.1 Read the status of: 
 *			Target1/Target2 indicator on TargetActual(PC0) and remember it in TargetCurrent
 *			Target position
 *			Filter 
 *		2.2	Set requested values to the current ones.
 *			
 *      2.3 return
 *
 *	3. Automatic mode
 *		3.1 Read the status of: 
 *			Target1/Target2 indicator on TargetActual(PC0) and remember it in TargetCurrent
 *			Target position
 *			Filter 
 *		3.2 If TargetRequested is not the TargetCurrent 
 *			then switch the target and wait for ~500ms
 *		3.3 If TargetPosition is less then the Urequested - TOLERANCE 
 *			and TargetUpLimit is not set then move the target up.
 *		3.4 If TargetPosition is greater then the Urequested + TOLERANCE
 *			and TargetDownLimit is not set then move the target down.
 *		3.5 If FilterRequested is not the FilterCurrent
 *			and corresponding FilterLimit switch is not set then turn on the filter motor.
 *
 *  ToDo: Adapt the algorithm to the transition from Manual to Auto mode.
 */

uint16_t i;
	i = 0;

	TargetCurrent = IS_RELAY_ON(TargetActual);
	if(TimerBeep == TIMER_FLAG) TimerBeep  = 0;  // Reset timer
	
	if(IS_RELAY_ON(ManualAuto)) { // we are in manual mode
		TargetRequested = TargetCurrent;
		Urequested = Uadc;
		FilterCurrent = GET_CURRENT_FILTER;
		if(FilterCurrent < 3 )
		{
			FilterRequested = FilterCurrent-1;
		}
		ItWasManual = 1;
		RELAY_OFF(TargetRelayUp);
		RELAY_OFF(TargetRelayDown);
		RELAY_OFF(FilterRelayIn);
		RELAY_OFF(FilterRelayOut);
		return; // Do nothing 
	}
	
	if(ItWasManual)
	{
		TargetCare();
		ItWasManual = 0;
		TargetCurrent = IS_RELAY_ON(TargetActual);
		FilterCurrent = GET_CURRENT_FILTER;
		if(FilterCurrent < 3 )
		{
			FilterRequested = FilterCurrent-1;
		}
	}
		
	if(TargetCurrent != TargetRequested) {
		
		// Move current target out of the beam
		MoveTargetOut();
		
		// Change target
		if(TargetRequested == 0) {SELECT_TARGET1;}
		else {SELECT_TARGET2;}
		
		i = 0;
		while(1) 
		{
			TargetCurrent = IS_RELAY_ON(TargetActual);
			if(TargetCurrent == TargetRequested) break;
			if(i++ == 3) {
				SET_TIMER(TimerBeep, ONE_SECOND);
				return;
			}
			_delay_ms(500.);   // wait for 500 miliseconds to allow target switching
		} 
		for(i=0; i<3; i++) {  		// Get coordinate (and do it three times for sure)
			ADCSRA |= _BV(ADIE);  	// allow ADC interrupt (if it is not allowed for some reason)
			ADCSRA |= _BV(ADSC);  	// start ADC conversion cycle 
			while(!intflags.adc_int) sleep_mode();
			intflags.adc_int = 0;
			intflags.tmr_int = 0;
			intflags.rx_int = 0;  // alas! the device will loose a message if any.
		}
	}  // end of target switching
	
	if(IsFlashing)
	{
		if(Urequested < TOLERANCE) 
		{
			if(	!IS_RELAY_ON(TargetLimitDown) // Check limit switch state
				&& (Uadc > TOLERANCE) ) 
			{
				RELAY_OFF(TargetRelayUp); 
				RELAY_ON(TargetRelayDown);
			}
		} 
		else
		{
			if(Uadc < (Urequested - TOLERANCE) 
				&& (!IS_RELAY_ON(TargetLimitUp)) ) // Check limit switch state
			{ 
				RELAY_OFF(TargetRelayDown); 
				RELAY_ON(TargetRelayUp);
			}
			else
			{
				RELAY_OFF(TargetRelayUp); 
			}
		}
	
		if(Uadc > (Urequested + TOLERANCE)) 
		{
			if(	!IS_RELAY_ON(TargetLimitDown)) // Check limit switch state
			{
				RELAY_OFF(TargetRelayUp); 
				RELAY_ON(TargetRelayDown);
			}
			else
			{
				RELAY_OFF(TargetRelayDown); 
			}
		}

// Check if we are at the requested position
		if(Uadc == Urequested)
		{
			RELAY_OFF(TargetRelayUp); 
			RELAY_OFF(TargetRelayDown);
			IsFlashing = 0;
		}
	}
	
// Filter ON / OFF handling
	FilterCurrent = GET_CURRENT_FILTER;
	if(FilterCurrent > 2 )
	{
		SET_TIMER(TimerBeep, ONE_SECOND); // Beep the error
		return;
	}
	
	if(FilterCurrent != (FilterRequested+1)) {
		if(FilterRequested == 0) {
			RELAY_OFF(FilterRelayIn);
			if(IS_RELAY_ON(FilterLimitOut)) {
				RELAY_OFF(FilterRelayOut);
			} else {
				RELAY_ON(FilterRelayOut);
			}
		}
		if(FilterRequested == 1) {
			RELAY_OFF(FilterRelayOut);
			if(IS_RELAY_ON(FilterLimitIn)) {
				RELAY_OFF(FilterRelayIn);
			} else {
				RELAY_ON(FilterRelayIn);
			}
		}
	}
	else
	{
				RELAY_OFF(FilterRelayIn);
				RELAY_OFF(FilterRelayOut);
	}
}
//---------------------------------------------------------------------------

char* uchar2h(uint8_t i)
{
   static char buf[3];
   const char code[16]={'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
   buf[1] = code[i&0xf];
   buf[0] = code[i>>4];
   buf[2] = 0;
   return buf;
}
//---------------------------------------------------------------------------

uint8_t h2uchar(char *buf)
{
   uint8_t r1,r2;
   r1 = (buf[0]>='a'? buf[0]-('a'-'A'): buf[0]) -'0'; // to upper case
   r1 = r1>9 ? r1-('A'-'0')+10 : r1;
   r2 = (buf[1]>='a'? buf[1]-('a'-'A'): buf[1]) -'0'; // to upper case
   r2 = r2>9 ? r2-('A'-'0')+10 : r2;
   return (r2 | (r1<<4)) -'0';
}
//---------------------------------------------------------------------------

/*
 * Where to store the configuration settings in EEPROM.  
 * This is used in order to remember the value across a RESET or power cycle.
 */
//uint8_t ee_tab[sizeof(RelayConfig)] __attribute__((section(".eeprom"))); // = {110,100,210,200,310,300,410,400};
//uint8_t ee_CONVERTER __attribute__((section(".eeprom"))); // = {110,100,210,200,310,300,410,400};


/*
 * Mirror of the MCUCSR register, taken early during startup.
 */
uint8_t mcucsr __attribute__((section(".noinit")));

//---------------------------------------------------------------------------

/* Part 3: Interrupt service routines */

ISR(TIMER0_OVF_vect)
{
static uint16_t scaler = 1;

	if (--scaler == 0)
	{
		scaler = 10;
		if(TimerBeep && TimerBeep != TIMER_FLAG)  BEEPER_PORT ^= _BV(BEEPER_PIN);
	}
}
//---------------------------------------------------------------------------

ISR(TIMER1_OVF_vect)
{
static uint16_t scaler = 1;

//	if(TimerBeep && TimerBeep != TIMER_FLAG)  BEEPER_PORT ^= _BV(BEEPER_PIN);
	if (--scaler == 0)
	{
		scaler = SYS_CLOCK;
		intflags.tmr_int = 1;
	}
	systime++;
}
//---------------------------------------------------------------------------

#if HAVE_ADC
/*
 * ADC conversion complete.  Fetch the 10-bit value, and feed the
 * PWM with it.
 */
ISR(ADC_vect)
{
  Uadc = ADCW;
  ADCSRA &= ~_BV(ADIE);		/* disable ADC interrupt */
  intflags.adc_int = 1;
}
#endif /* HAVE_ADC */
//---------------------------------------------------------------------------

/*
 * UART receive interrupt.  Fetch the character received and buffer
 * it, unless there was a framing error.  Note that the main loop
 * checks the received character only once per 10 ms.
 */
ISR(USART_RXC_vect)
{
  uint8_t c;

  c = UDR;
  if (bit_is_clear(UCSRA, FE))
    {
      rxbuff = c;
      intflags.rx_int = 1;
    }
}
//---------------------------------------------------------------------------

/* Part 4: Auxiliary functions */

/*
 * Read out and reset MCUCSR early during startup.
 */
void handle_mcucsr(void)
  __attribute__((section(".init3")))
  __attribute__((naked));
void handle_mcucsr(void)
{
  mcucsr = MCUCSR;
  MCUCSR = 0;
}

//---------------------------------------------------------------------------

/*
 * Some simple UART IO functions.
 */

/*
 * Send character c down the UART Tx, wait until tx holding register
 * is empty.
 */
void putchr(uint8_t c)
{
	loop_until_bit_is_set(UCSRA, UDRE);
	UDR = c;
}
//---------------------------------------------------------------------------

/*
 * Send the fixed length buffer down the UART Tx.
 */
void write_binary(const char *s, uint8_t len)
{
  while (len--)  putchr(*s++);
}
//---------------------------------------------------------------------------

/*
 * Send a C (NUL-terminated) string down the UART Tx.
 */

void printstr(const char *s)
{
  while (*s)
    {
      if (*s == '\n')
		putchr('\r');
      putchr(*s++);
    }
}
//---------------------------------------------------------------------------

/*
 * Same as above, but the string is located in program memory,
 * so "lpm" instructions are needed to fetch it.
 */
void printstr_p(const char *s)
{
  char c;

  for (c = pgm_read_byte(s); c; ++s, c = pgm_read_byte(s))
    {
      if (c == '\n')
			putchr('\r');
      putchr(c);
    }
}
//---------------------------------------------------------------------------

static void
uart_set_baud(void)
{
   #define BAUD 115200
   #define BAUD_TOL 2
   #include <util/setbaud.h>
   UBRRH = UBRRH_VALUE;
   UBRRL = UBRRL_VALUE;
   #if USE_2X
   UCSRA |= (1 << U2X);
   #else
   UCSRA &= ~(1 << U2X);
   #endif
}

/*
 * Do all the startup-time peripheral initializations.
 */
void ioinit(void)
{
//  Reset all global variables here
	TimerBeep = 0;
	Urequested = 0;
	FilterRequested = 0;
	IsFlashing = 0;
	ItWasManual = 1;
	systime = 0;
	rxbuff = 0;

/*  
 *	Disable external interrupts from port pins
 */
	GICR &= 0x1F; // disable INT0, Int1 and INT2

  /*
   * Set up the 16-bit timer 1.
   *
   * Timer 1 will be set up as a 16-bit in the normal mode (e.g. runs from 0 to 0xFFFF). 
   * OC1A and OC1B pins are disconnected.
   * The timer will run on MCU clock.
   */
  //TCCR1A = 3;
  TCCR1A = _BV(WGM10) | _BV(WGM11) ;
  TCCR1B = 1; //1-no prescaling; 2- clck/8; 3-clk/64; 4-clk/256; 5-clk/1024

  //OCR1A = 0x0fff;			/* set TOP value */

	TCCR0 = 2; //1-no prescaling; 2- clck/8; 3-clk/64; 4-clk/256; 5-clk/1024
  /*
   * Enable Port D outputs: PD6 for the clock output, PD7 for the LED
   * flasher.  PD1 is UART TxD but not DDRD setting is provided for
   * that, as enabling the UART transmitter will automatically turn
   * this pin into an output.
   */
  CONTROL_DDR = _BV(CONTROL_PIN);
  BEEPER_DDR = _BV(BEEPER_PIN);

 
 // UCSRA = _BV(U2X);		/* improves baud rate error @ F_CPU = 1 MHz */
	UCSRB = _BV(TXEN)|_BV(RXEN)|_BV(RXCIE); /* tx/rx enable, rx complete intr */
//	UBRRL = (F_CPU / (16 * 9600UL)) - 1;  /* 9600 Bd */

	uart_set_baud();
	
#if HAVE_ADC
	/* 
	 *Measured voltage is supplied on (PA3) ADC3 
	 */
	ADMUX = _BV(MUX0) | _BV(MUX1);
	/*
	 * Select automatic conversion on Timer1 Compare match B
	 */
	SFIOR |= _BV(ADTS2) | _BV(ADTS0);
	
  /*
   * enable ADC, select ADC clock = F_CPU / 128 (i.e. 125 kHz)
   */
	ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);
#endif

// Pin configuration
//  SET_RELAY_CONFIG(Relay,         Port, Pin, ENABLE) 
	SET_RELAY_CONFIG(TargetRelayUp   ,'B' ,2   ,1  );
	SET_RELAY_CONFIG(TargetRelayDown ,'B' ,3   ,1  );
	SET_RELAY_CONFIG(FilterRelayIn   ,'B' ,0   ,1  );
	SET_RELAY_CONFIG(FilterRelayOut  ,'B' ,1   ,1  );
	SET_RELAY_CONFIG(FilterLimitIn   ,'D' ,2   ,0  );
	SET_RELAY_CONFIG(FilterLimitOut  ,'A' ,0   ,0  );
	SET_RELAY_CONFIG(TargetLimitUp   ,'A' ,1   ,0  );
	SET_RELAY_CONFIG(TargetLimitDown ,'A' ,2   ,0  );
	SET_RELAY_CONFIG(Target1Select   ,'C' ,6   ,1  );
	SET_RELAY_CONFIG(Target2Select   ,'C' ,7   ,1  );
	SET_RELAY_CONFIG(TargetActual    ,'C' ,0   ,0  );
	SET_RELAY_CONFIG(ManualAuto      ,'C' ,1   ,0  );

//Set initial values
	RELAY_OFF(TargetRelayUp);
	RELAY_OFF(TargetRelayDown);
	RELAY_OFF(FilterRelayIn);
	RELAY_OFF(FilterRelayOut);
	RELAY_OFF(Target1Select);
	RELAY_OFF(Target2Select);


	TIMSK = _BV(TOIE1);			// Enable Timer1 Overflow Interrupt Enable bit
	TIMSK |= _BV(TOIE0);		// Enable Timer0 Overflow Interrupt Enable bit

	
	if(!IS_RELAY_ON(ManualAuto))   // We are in the Automatic mode.
	{  	
		ItWasManual = 0;
		TargetCurrent = IS_RELAY_ON(TargetActual);
		TargetRequested = TargetCurrent==0? 1 : 0;
		Urequested = 0;
		FilterRequested = GET_CURRENT_FILTER - 1;
	} 
	else 
	{
		ItWasManual = 1;
	}

	intflags.adc_int = 0;
	intflags.tmr_int = 0;
	intflags.rx_int  = 0;

	sei();			/* enable interrupts */

  /*
   * Enable the watchdog with the largest prescaler.  Will cause a
   * watchdog reset after approximately 2 s @ Vcc = 5 V
   */
  wdt_enable(WDTO_2S);
#if 0
	/*
	 *   Read initial configuration of target and filter.
	 *	and make it the required one to prevent unintentional movement.
	 */
	/* 
	 *  Start ADC conversion and remember the result
	 *  as the initial target position
	 */
	ADCSRA |= _BV(ADIE);
	ADCSRA |= _BV(ADSC); // start the conversion cycle
	Urequested = 150;  // Wait max for 1.5 sec for the ADC result (use 'Urequested' to save memory)
	while(--Urequested) 	{	
		wdt_reset();
		sleep_mode();
		if(!intflags.adc_int) continue;   // ignore all other interrupts
		intflags.adc_int = 0;
		Urequested = Uadc;
		break;
	}
	intflags.adc_int = 0;
	intflags.tmr_int = 0;
	intflags.rx_int = 0;
#endif
}
//---------------------------------------------------------------------------

uint16_t read_uint16(uint16_t *store, uint16_t *HowMany)
{
/*
 *	Read unsigned int16 values from uart. 
 *  Each non digit character is considered as a separator between the values.
 *  The '\n' character will terminate the reading.
 * 
 *  Be carefull! It is not clean reading!
 *  It will exit prematurely if number of stray interrupts will be too high (1000).
 *  All non UART interrupts is ignored. So you should clean manualy the 'intflags' 
 *  structure after call this function.
 *  Parameters: 
 *		store - where you want the read values to be stored
 *		HowMany - how many values you want to be read 
 *  Return value - zero    if too many stray interrupts occured (means: the number of
 *                         values put in 'store' is less then wanted 'HowMany')
 *               HowMany - actual number of values put in the 'store';
 *  						(Note once again: the last value could be corrupted 
 *								if return value is zero).
 */

int16_t ind = -1;    // trick to skip leading blanks
uint16_t	cycle = 0;
uint8_t	changed = 0;
uint8_t rxc;

#define MAXWAIT 1000	

	while(++cycle < MAXWAIT)
	{	
		wdt_reset();
		sleep_mode();
		if(!intflags.rx_int) continue;   // ignore other interrupts
		intflags.rx_int = 0;
		rxc = rxbuff;						
		if(rxc == '\n') break;
		if((rxc >= '0') && (rxc <= '9'))
		{
			if(ind == -1) ind = 0; // trick to skip leading blanks
			changed = 0;
			store[ind] = store[ind]*10 + (rxc-'0');
		} else if(!changed) 
		{
			ind++;
			changed = 1;
			if(ind == *HowMany) break;
		}
	}
	
	*HowMany = (ind < 0) ? 0 : ind;
	return (MAXWAIT-cycle);

}
//---------------------------------------------------------------------------

int8_t scan_uint16(uint16_t *store, uint8_t HowMany, char *buf, uint8_t buflen)
{
/*
 *	Scan the buffer and decode unsigned int16 values. 
 *  Each non digit character is considered as a separator between the values.
 *  The EOT and ETX characters will terminate the scan.
 * 
 *  Parameters: 
 *		store 	- where you want the read values to be stored
 *		HowMany - Capacity of the store
 *		buf		- string of chars
 *		len		- length of the string
 *  Return value:
 *				- actual number of values put in the 'store';
 */

int8_t	ind = -1;    // trick to skip leading blanks
int8_t	cycle = -1;
uint8_t	changed = 1;
uint8_t	rxc;

	store[0] = 0;
	while(++cycle < buflen)
	{	
		rxc = buf[cycle];						
		if(rxc == ETX || rxc == EOT) break;
		if((rxc >= '0') && (rxc <= '9'))
		{
			if(ind == -1) ind = 0; // trick to skip leading blanks
			changed = 0;
			store[ind] = store[ind]*10 + (rxc-'0');
		} else if(!changed) 
		{
			ind++;
			changed = 1;
			if(ind == HowMany) break;
			store[ind] = 0;
		}
	}
	
//	ind = (ind < 0) ? 0 : ind;
	return ind+1;

}
//---------------------------------------------------------------------------

uint8_t calc_crc(char* str,uint8_t len)
{
//
//	Compute CRC code for a string of char
  uint8_t bcc  = 0;
  while (len) {
     bcc ^= str[--len];
//     bcc = bcc & 0x7F;  // It is only for 7 bit transmission protocol
  }
	return bcc;
}
//---------------------------------------------------------------------------

int8_t read_binary(char *store, uint8_t *HowMany)
{
// Read binary data from UART
// The protocol is as follows:
// 		-	First byte in a stream should be zero 
//			(this byte is not seen in this function since 
//				it triggers the binary transfer mode)
// 		-	Second byte is N - length of the record (it is the first byte read in this function)
//		-   then follows N bytes of the record body
//		-	the last (N+1) byte is the CRC of the record body
//
//	The function sends ACK to UART in response of successful reading.
//	Otherwise it sends NAK.
//
//	Return value 
//      >0 			if success; (number of cycles left after reading)
//		 0			if too many stray interrupts occured;
//		-1 			if read error detected;
//		
//				 	it returns in variable 'HowMany' actual number of byte 
//					read from UART and placed in the buffer.
#define MAXWAIT 1000	

uint8_t len,ind;
uint16_t cycle=MAXWAIT;

// Read length of the record
	len = 0;
	while(--cycle)
	{	
		wdt_reset();
		sleep_mode();
		if(!intflags.rx_int) continue;   // ignore all other interrupts
		intflags.rx_int = 0;
		len = rxbuff;
		break;
	}
	if(!len) return -1;
	
// Read record body
	ind = 0;
	while(--cycle)
	{	
		wdt_reset();
		sleep_mode();
		if(!intflags.rx_int) continue;   // ignore all other interrupts
		intflags.rx_int = 0;
		store[ind++] = rxbuff;
		if(ind == len) break;
		if(ind == *HowMany)  {
			putchr(NAK);
			while(--cycle)  // Skip the rest of message
			{	
				wdt_reset();
				sleep_mode();
				if(!intflags.rx_int) continue;   // ignore other interrupts
				intflags.rx_int = 0;
				if(++ind == len) break;
			}
			return -2;          // Insufficient buffer error
		}
	}
	
	*HowMany = ind;
	if(cycle == 0) {
		putchr(NAK);
		return 0;
	}

 // Read the CRC byte
	while(--cycle) 	{	
		wdt_reset();
		sleep_mode();
		if(!intflags.rx_int) continue;   // ignore other interrupts
		intflags.rx_int = 0;
		len = rxbuff;
		break;
	}

	if(cycle == 0) {
		putchr(NAK);
		return 0;
	}
	
// Check the CRC
	if(len == calc_crc(store,ind)) {
		putchr(ACK);
	} else {
		putchr(NAK);
		return -3;      // Transmission error
	}

	return (cycle);

}
//---------------------------------------------------------------------------

int16_t read_message(char *store, uint8_t *HowMany)
{
/*
 *	Read message from uart in accord to IEC 61107 standart. 
 *  Every byte in the message body should be printable ASCII 
 *  symbol except the control bytes: SOH,STX,ETX,EOT and the last CRC sum.
 *  Format of a message :
 *      - begins with SOH (0x01) symbol
 *			-- optionaly followed by a printable ASCII sequence 
 *			   (a command name)
 *      - Then optional information block(s) may follow 
 *		  (again, is should be made of the printable characters).
 *			-- STX (0x02) symbol indicates beginning of the information block 
 *             which is made of the printable characters  
 *			-- terminated by ETX (0x03) symbol
 *       .... another info blocks may follow
 *	    - The message body must be closed by EOT (0x04) symbol.
 *      - The last symbol in the message is the CRC byte 
 *        which spans over the whole message body excluding 
 *        the first SOH symbol (but including the last EOT symbol)
 *
 *  NOTES: 
 *
 *  		- 	This function sends to the UART ACK (successs) or NAK (error) symbols in 
 *					response to the message read.
 *       	-	Buffer overflow: 
 *					it fills the buffer and does not skip the rest of message if anything left. 
 *					If you want to read a very long message you should call the function again 
 *					and again and then calculate the CRC sum manualy. 
 *					Please note that in this case the NAK symbol will be sent to UART 
 *					automatically after each call to the function. 
 * 			-	Be carefull! It is not clean reading! It will exit prematurely if number 
 *					of stray interrupts will be too high (MAXWAIT=1000).
 *  		-	All non UART interrupts are ignored. So you should clean manualy 
 *					the 'intflags' structure after call this function.
 *
 *  Parameters: 
 *		store   - where you want the read message to be stored
 *		HowMany - capacity of the storage is (HowMany-1) since I need one byte to put Null there.
 * 
 *  Return value - zero		if too many stray interrupts occured (means: the message is not read 
 *							to the end)
 *					-1		transmission error detected
 *					-2		message is too long to fit in the buffer
 *  Output parameter:
 *				  HowMany - actual number of values put in the 'store';
 *  						(Note once again: message could be corrupted if return value is zero).
 */

uint16_t cycle = 0;
uint8_t	 ind = 0;
uint8_t  rxc = 0;

#define MAXWAIT 1000	

	while(++cycle < MAXWAIT)
	{	
		wdt_reset();
		sleep_mode();
		if(!intflags.rx_int) continue;   // ignore all other interrupts
		intflags.rx_int = 0;
		if(rxc == EOT) {
			rxc = rxbuff;						
			if(rxc == calc_crc(store,ind)) {
				putchr(ACK);
				*HowMany = ind;
				break;
			} else {
				SET_TIMER(TimerBeep, (ONE_SECOND/4));
				putchr(NAK);
				*HowMany = ind;
				return -1;      // Transmission error
			}
		}
		rxc = rxbuff;						
		store[ind++] = rxc;
		if(ind == *HowMany)  {
			SET_TIMER(TimerBeep, (ONE_SECOND/4));
			putchr(NAK);
			while(++cycle < MAXWAIT)  // Skip the rest of message
			{	
				wdt_reset();
				sleep_mode();
				if(!intflags.rx_int) continue;   // ignore other interrupts
				intflags.rx_int = 0;
				if(rxbuff == EOT) break;
			}
			return -2;          // Insufficient buffer error
		}
	}
	
	*HowMany = ind;
	if(MAXWAIT == cycle) {
		SET_TIMER(TimerBeep, (ONE_SECOND/4));
		putchr(NAK);
	}
	return (MAXWAIT-cycle);

}
//---------------------------------------------------------------------------

#define PUTCHR_AND_CRC(CHR) { 	\
		bcc ^=CHR;				\
		putchr(CHR);			\
}

#define PUTSTR_AND_CRC(STR) { 	\
		while(*STR) {			\
			bcc ^=*STR;			\
			putchr(*STR);		\
			STR++;				\
		}						\
}

void write_message(char* Header, uint8_t *Body, uint8_t BodySize)
{// write a message to uart 
// for the message format details consult the 'read_message'


	uint8_t bcc = 0; // clear the crc byte
	putchr(SOH);
	if(Header) 
		while(*Header) {
			PUTCHR_AND_CRC((*Header));
			Header++;
		}
	PUTCHR_AND_CRC(STX);
	if(Body) {
		uint8_t i = 0;
		char *byteadr;
		for(i=0; i < BodySize; i++) {
			byteadr = uchar2h(*(Body+i));
			PUTSTR_AND_CRC(byteadr);
		}
		PUTCHR_AND_CRC(ETX);
	}
	PUTCHR_AND_CRC(EOT);
	putchr(bcc);
}
//---------------------------------------------------------------------------


void do_cmd(char *cmd, uint8_t lencmd)
{// parse a message and do something usefull
uint8_t buf[6];
uint16_t *rqst;

	switch (cmd[0])
	{
	case EOT:	break;
	case '?':		// report current relay status and target position
		cmd[0] = IS_RELAY_ON(FilterLimitOut) + 2*IS_RELAY_ON(FilterLimitIn); // 0 - unknown, 1 - out, 2 - in, 3 - crazy
		cmd[1] = IS_RELAY_ON(TargetActual); // Target numbers are 0 or 1
		uint16_t Tpos = Uadc; // Save current target position in local variable.
		cmd[2] = *(((char *)&Tpos)+1); // MSB goes first
		cmd[3] = *((char *)&Tpos);      // LSB goes last
		cmd[4] = 3 - (IS_RELAY_ON(TargetRelayDown) + 2*IS_RELAY_ON(TargetRelayUp)); // 0 - OK, 1 - down, 2 - up, 3 - crazy
		cmd[5] = 3 - (IS_RELAY_ON(FilterRelayIn) + 2*IS_RELAY_ON(FilterRelayOut)); // 0 - OK, 1 - 1, 2 - 2, 3 - crazy
		cmd[6] = IS_RELAY_ON(TargetLimitDown) + 2*IS_RELAY_ON(TargetLimitUp); // 0 - OK, 1 - out, 2 - in, 3 - crazy
		cmd[7] = 3 - (IS_RELAY_ON(Target1Select) + 2*IS_RELAY_ON(Target2Select)); // 0 - OK, 1 - 1, 2 - 2, 3 - crazy
		cmd[8] = IS_RELAY_ON(ManualAuto);
		write_message("STATUS",cmd,9);
	break;
	case 'Q':		// 	response to automatic device identification query
					// 	PLEASE note that it is a plain text and not the standart 
					//	message format
		printstr_p(PSTR("SPINU70"));
	break;
	case 's':
	case 'S':      // Set new values for target number, target position and filter 
		rqst = (uint16_t *) buf;
		if(!scan_uint16(rqst,3,cmd+2,lencmd-3)) {
			printstr_p("Failed to read new values\n");
		}
		FilterRequested = (uint8_t) rqst[0];
		TargetRequested = (uint8_t) rqst[1];
		Urequested = rqst[2];
// Do some checks of the requested values
		FilterCurrent = IS_RELAY_ON(FilterLimitOut) + 2*IS_RELAY_ON(FilterLimitIn); // 0 - unknown, 1 - out, 2 - in, 3 - crazy
		TargetCurrent = IS_RELAY_ON(TargetActual);
		if(FilterCurrent < 3 && FilterRequested > 1)
		{
			SET_TIMER(TimerBeep, (ONE_SECOND/4));
			FilterRequested = FilterCurrent;
		}
		if(TargetRequested > 1)
		{
			SET_TIMER(TimerBeep, (ONE_SECOND/4));
			TargetRequested = TargetCurrent;
		}
		if(Urequested > 1023)
		{
			SET_TIMER(TimerBeep, (ONE_SECOND/4));
			Urequested = Uadc;
		}
		IsFlashing = 1;
	break;
	default: break;
	}

}
//---------------------------------------------------------------------------

/* Part 5: main() */

#define CBUFSIZE	20

int
main(void)
{
  char cbuf[CBUFSIZE];
  int8_t i;
  uint8_t uc;
  
  ioinit();

  if ((mcucsr & _BV(WDRF)) == _BV(WDRF)) {
#ifdef DEBUG
    printstr_p(PSTR("\nOoops, the watchdog bit me!"));
#endif
	}
	
#ifdef DEBUG
  printstr_p(PSTR("\nHello, this is SPIN U70 target and filter manipulator "
		  "running on an "
#if defined(__AVR_ATmega16__)
		  "ATmega16"
#elif defined(__AVR_ATmega8__)
		  "ATmega8"
#elif defined(__AVR_ATmega48__)
		  "ATmega48"
#elif defined(__AVR_ATmega88__)
		  "ATmega88"
#elif defined(__AVR_ATmega168__)
		  "ATmega168"
#elif defined(__AVR_ATtiny2313__)
		  "ATtiny2313"
#else
		  "unknown AVR"
#endif
		  "\n"));
#endif

  for (;;)
	{
      wdt_reset();

      if (intflags.tmr_int)
		{
			/*
			* Our periodic 10 ms interrupt happened.  See what we can
			* do about it.
			*/
			intflags.tmr_int = 0;
			REFRESH_TIMER(TimerBeep);

			if(!IsFlashing) {
				// Switch off LED on PD7
				CONTROL_PORT &= ~_BV(CONTROL_PIN);
			} else {
			// Toggle flash LED on PD7
				CONTROL_PORT ^= _BV(CONTROL_PIN);
			}
			Algorithm();	//  It is the main job of the whole project.
#if HAVE_ADC
		  /*
		   * Enable next conversion.
		   */
			ADCSRA |= _BV(ADIE);
// start the conversion cycle
			ADCSRA |= _BV(ADSC);   
#endif
			//continue;  // Other interrupts may occure 
		}
#if HAVE_ADC
      if (intflags.adc_int)
		{
			intflags.adc_int = 0;
			//continue;  // Other interrupts may occure 
		}
#endif /* HAVE_ADC */

      if (intflags.rx_int)
		{
			intflags.rx_int = 0;
			uc = rxbuff;
			if(uc != SOH) {
				SET_TIMER(TimerBeep, (ONE_SECOND/4));
				putchr(NAK);  // Skip until SOH 
				continue;
			}
			uc = sizeof(cbuf);
			i = read_message(cbuf,&uc);
			intflags.rx_int  = 0;
			intflags.adc_int = 0;
			intflags.tmr_int = 0;
			switch (i) {
			case 0: 
				printstr_p(PSTR("\nToo many stray interrupts. Message is not read\n"));
				SET_TIMER(TimerBeep, ONE_SECOND);
			break;
			case -1:
				printstr_p(PSTR("\nTransmission error\n"));
				SET_TIMER(TimerBeep, ONE_SECOND);
			break;
			case -2:
				printstr_p(PSTR("\nMessage is too long\n"));
				SET_TIMER(TimerBeep, ONE_SECOND);
			break;
			default: 
				//Parse the message and do something usefull here
				do_cmd(cbuf,uc);
			break;
			}
				// While we are in communication with the user 
				// we do not care of the normal flow of the algorithm.
				// This means that timing is violated.
				// And should be somehow recovered.
				// The recovery is (should be) the inherent property of 
				// the algorithm itself. :) 
			continue;	// Let us see what we have in reality ;-)
		}
	
      sleep_mode();
    }
}
//---------------------------------------------------------------------------
