//--------  Define communication attributes of IEC 61107 ( standart ASCII codes ) --------
#ifndef RelayFirmwareH
#define RelayFirmwareH

#define SOH	0x01
#define STX	0x02
#define ETX	0x03
#define EOT	0x04
#define ACK	0x06
#define NAK	0x15
/*
const char *sSOH = "<SOH>";  // Symbolic names of the above constants
const char *sSTX = "<STX>";
const char *sETX = "<ETX>";
const char *sEOT = "<EOT>";
const char *sACK = "<ACK>";
const char *sNAK = "<NAK>"; 
*/
//typedef unsigned short uint16_t;
//typedef signed short int16_t;
//typedef unsigned char uint8_t;
//typedef signed char int8_t;
//typedef uint16_t 	RELAYONOFF_T;
//typedef uint8_t		RELAYCONFIG_T;

//  Macros to deal with packed relay configuration.
#define GET_PIN(Relay)		((Relay)&7)
#define GET_ENABLE(Relay)    ((Relay)&0x8)
#define SET_PORT(Port,Relay) { 			    												\
	(Relay) &= 0xCF;																		\
	if((Port) == 'A') {                 if(GET_ENABLE(Relay)) DDRA |= _BV(GET_PIN(Relay));} \
	if((Port) == 'B') {(Relay) |= 0x10; if(GET_ENABLE(Relay)) DDRB |= _BV(GET_PIN(Relay));}	\
	if((Port) == 'C') {(Relay) |= 0x20; if(GET_ENABLE(Relay)) DDRC |= _BV(GET_PIN(Relay));} \
	if((Port) == 'D') {(Relay) |= 0x30; if(GET_ENABLE(Relay)) DDRD |= _BV(GET_PIN(Relay));} \
}
#define SET_PIN(Pin,Relay) { 			\
	(Relay) &= 0xf8; 					\
	(Relay) |= Pin&7; 					\
}
#define SET_ENABLE(Flag,Relay) {		\
	(Relay) &= 0xf7;					\
	if(Flag) (Relay) |= 0x8;			\
}
 // Packed: Relay pin number ( 0->2 bits), port (A,B,C,D bits 4->5)
 // Bit 3 - Enable/Disable
#define SET_RELAY_CONFIG(Relay,Port,Pin,ENABLE) { 	\
    Relay = 0;                                      \
	SET_PIN(Pin,Relay);								\
	SET_ENABLE(ENABLE,Relay);			            \
	SET_PORT(Port,Relay);							\
}

#endif

