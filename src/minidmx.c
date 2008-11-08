/**
 * minidmx - RS232-to-DMX-converter
 *
 * Copyright 2001,2002,2003 Mathias Dzionsko <madz@gmx.de>
 *
 * Compiler:
 *   WinAVR <http://sourceforge.net/projects/winavr/>
 *   AVR Studio <http://www.atmel.com/avrstudio>
 * MCU: ATTiny2313 (originally AT90S2313)
 *
 * Changelog:
 *  2006-12-17 Ulrich Radig <http://ulrichradig.de/>
 *    * Ported to ATTiny2313
 *    * Adapted to current WinAVR
 *    * Adapted to my PCB, see
 *      <http://www.ulrichradig.de/home/index.php/avr/rs232-dmx>
 *  2003-09-09 Mathias Dzionsko <http://www.dzionsko.de/>
 *    * Fixed generation of DMX-signal
 *  2003-03-21 Mathias Dzionsko <http://www.dzionsko.de/>
 *    * Reimplemented the in C instead of assembler (v2.0)
 *  2003-02-04 Mathias Dzionsko <http://www.dzionsko.de/>
 *    * LED1 indicates successfully sent DMX data
 *    * LED2 indicates an error with the RS232-communication
 *    * LED3 indicates the end of the startup procedure (running and
 *    * data can be sent)
 *  2001-02-18 Mathias Dzionsko <http://www.dzionsko.de/>
 *    * First public release (v1.0)
 */

#include <inttypes.h>
#include <avr\io.h>
#include <avr\interrupt.h>
//#include <avr\signal.h>

#define F_CPU           9216000
#include <util/delay.h>
//---------------------------------------------------------------------------
// Macros

#define BITS2BYTE(b7,b6,b5,b4,b3,b2,b1,b0) \
  (b7<<7|b6<<6|b5<<5|b4<<4|b3<<3|b2<<2|b1<<1|b0)

// Delay by t µs.
#define Delay16(l)      _delay_loop_2(l)
#define DELAY16VAL(t)   ((XTAL*(long)(t))/4000)
//---------------------------------------------------------------------------
// Constants

// Clock of the MCU in kHz (for Delay16)
#define XTAL            9216

#if defined (__AVR_ATtiny2313__)
    #define TCCR0 TCCR0A
	#define USR UCSRA
	#define UCR UCSRB
	#define UBRR UBRRL
    #define UART_TX_vect USART_TX_vect
#endif

// DMX
#define DMX_PORT        PORTB
#define DMX_PIN         PB0

// LED1 indicates a successfully sent DMX packet
// LED2 indicates an error receiveing data via RS232
// LED3 indicates end of startup procedure
#define LED_PORT        PORTB
#define LED1_PIN        PB2
#define LED2_PIN        PB3
#define LED3_PIN        PB4

// LED indication time: 100ms (in 1/100s)
#define LED1_ONTIME     10
#define LED2_ONTIME     10

// Port B Pins
// PB0 - DMX output
// PB1 - Switch
// PB2 - LED1
// PB3 - LED2
// PB4 - LED3
#define PORTB_INIT      BITS2BYTE(0,0,0,1,1,1,1,1)
#define DDRB_INIT       BITS2BYTE(0,0,0,1,1,1,0,1)

// Port D Pins
// PD0 - RXD
// PD1 - TXD
// PD6 - Testing trigger for an oscillosscope
#define PORTD_INIT      BITS2BYTE(0,0,1,1,0,0,1,0)
#define DDRD_INIT       BITS2BYTE(0,0,1,1,0,0,1,0)

// UART
#define UBRR_INIT       4                       // 115200 Baud
#define UCR_INIT        (_BV(RXCIE)+_BV(RXEN)+_BV(TXEN))
#define RX_BUFSIZE      64
#define RX_TIMEOUT      10 // 100ms (in 1/100s)

// Timer
#define TIMSK_INIT      _BV(TOIE0)
#define TCCR0_INIT      (_BV(CS02)+_BV(CS00))   // CK/1024 => 9000 Hz
#define TIMER0_RELOAD   (256-90)                // => 100 Hz

// RS232-protocol
#define P_BLKSTART      0x5A
#define P_BLKEND        0xA5

#define P_DMXOUT96      0xA0
#define P_DMXOUT256     0xA1
#define P_DMXOUT512     0xA2

#define P_ERROR         0xC0
#define P_ACK           0xC1

// Constants for the Error var
#define E_BUFFEROVERFLOW        1
#define E_UARTTIMEOUT           2
#define E_PROTOCOLERROR         4
//---------------------------------------------------------------------------
// Global vars

volatile uint8_t        TimerLED1, TimerLED2;   // LED Timer
volatile uint8_t        UARTTimer;
volatile uint8_t        Error;

// RS232 receive buffer
uint8_t                 RxBuffer[RX_BUFSIZE];
uint8_t                 RxBufferOut;
volatile uint8_t        RxBufferIn;
//---------------------------------------------------------------------------
// OVERFLOW0 Interrupt

SIGNAL(SIG_OVERFLOW0)
{
  // Reload timer
  TCNT0=TIMER0_RELOAD;

  // UARTTimer
  if (UARTTimer) UARTTimer--;
  
  // Trigger LED1
  if (TimerLED1) {
    LED_PORT &= ~(1<<LED1_PIN);
    TimerLED1--;
  } else LED_PORT |= (1<<LED1_PIN);

  // Trigger LED2
  if (TimerLED2) {
    LED_PORT &= ~(1<<LED2_PIN);
    TimerLED2--;
  } else LED_PORT |= (1<<LED2_PIN);
}
//---------------------------------------------------------------------------
// SIG_UART_RECV Interrupt

ISR (UART_TX_vect)
{
  uint8_t nextpos;
  unsigned char tmp ;

  // Calculate next buffer position
  nextpos=RxBufferIn+1;
  if (nextpos==RX_BUFSIZE) nextpos=0;

  // Buffer full?
  if (nextpos==RxBufferOut) {
    // Indicate error.
    Error|=E_BUFFEROVERFLOW;
    tmp = UDR; // Clear UART Data Register
  } else {
    // Store received data in buffer.
    RxBuffer[RxBufferIn]=UDR;
    RxBufferIn=nextpos;
  }
}
//---------------------------------------------------------------------------
// UARTRecvByte

uint8_t UARTRecvByte(void)
{
  uint8_t data;

  // Wait until a byte arrives in the buffer or a timeout occurs
  UARTTimer=RX_TIMEOUT;
  while (RxBufferIn==RxBufferOut && UARTTimer) ;

  // No data received?
  if (RxBufferIn==RxBufferOut) {
    Error|=E_UARTTIMEOUT;
    return 0;
  }

  // Read byte.
  data=RxBuffer[RxBufferOut++];
  if (RxBufferOut==RX_BUFSIZE) RxBufferOut=0;

  return data;
}
//---------------------------------------------------------------------------
// UARTRSendByte

void UARTSendByte(uint8_t data)
{
  // Wait until the previous byte is completely transmitted.
  loop_until_bit_is_set(USR, UDRE);

  // Send new data.
  UDR=data;
}
//---------------------------------------------------------------------------
// DMXSendByte

// Sends a DNX data byte.
// The spec wants 4 µs per DMX bit.  With a clock frequency of
//   9.216 MHz
// we have to use up 37 cycles so we get 4.0147 µs per Bit which is ok.
// Interrupts are blocked while this code runs.
// We transmit 11 Bit, so we block the interrupts for about 44.3 µs.


void DMXSendByte(uint8_t value)
{
  asm volatile (
"                cli                    \n" // Block interrupts
"                clc                    \n" // 1      - Start Bit
"                rcall DMXSendBit       \n" // +3+6+27=37 cycles
"                                       \n"
"                ror %2                 \n" // 1      - Bit 0
"                rcall DMXSendBit       \n" // +3+6+27=37 cycles
"                ror %2                 \n" //        - Bit 1
"                rcall DMXSendBit       \n"
"                ror %2                 \n" //        - Bit 2
"                rcall DMXSendBit       \n"
"                ror %2                 \n" //        - Bit 3
"                rcall DMXSendBit       \n"
"                ror %2                 \n" //        - Bit 4
"                rcall DMXSendBit       \n"
"                ror %2                 \n" //        - Bit 5
"                rcall DMXSendBit       \n"
"                ror %2                 \n" //        - Bit 6
"                rcall DMXSendBit       \n"
"                ror %2                 \n" //        - Bit 7
"                rcall DMXSendBit       \n"
"                                       \n"
"                sec                    \n"  //        - Stop Bit 1
"                rcall DMXSendBit       \n"
"                sec                    \n"  //        - Stop Bit 2
"                rcall DMXSendBit       \n"
"                sei                    \n"  // Unblock interrupts
"                rjmp DMXSendByteExit   \n"
"                                       \n"
"DMXSendBit:     brcs DMXSendBit1       \n"
"                nop                    \n" // 1+1
"                cbi %0, %1             \n" // +2
"                rjmp DMXSendBit2       \n" // +2=6 cycles
"DMXSendBit1:    sbi %0, %1             \n" // 2+2
"                rjmp DMXSendBit2       \n" // +2=6 cycles
"                                       \n"
"DMXSendBit2:    ldi r31,7              \n" // 1
"DMXSendBit3:    dec r31                \n" // +1
"                brne DMXSendBit3       \n" // +1/+2
"                rjmp DMXSendBit4       \n" // +2
"DMXSendBit4:    ret                    \n" // +4=9+6*3=27 cycles
"DMXSendByteExit:                       \n"
//                          %0               %1            %2
    : : "I" (_SFR_IO_ADDR(DMX_PORT)), "I" (DMX_PIN), "r" (value) : "r31"
  );
}

//---------------------------------------------------------------------------
// DMXSendReset

// Sends a DMX RESET followed by a start byte.

void DMXSendReset(void)
{
  DMX_PORT &=~(1<<DMX_PIN);       // Send RESET
  Delay16(DELAY16VAL(100));     // Wait 100µs (min. 88µs)

  DMX_PORT |= (1<<DMX_PIN);       // Send MARK
  Delay16(DELAY16VAL(10));      // Wait 10µs (min. 8µs)

  DMXSendByte(0);               // Send START byte (0)
}
//---------------------------------------------------------------------------
// DMXSendChannels

void DMXSendChannels(uint16_t cnt)
{
  uint8_t data;

  DMXSendReset();

  while (cnt--) {
    data=UARTRecvByte();
    if (Error) return;
    DMXSendByte(data);
  }
}
//---------------------------------------------------------------------------
// Main

int main(void)
{
  uint8_t data;

  // Initialize Port B
  PORTB=PORTB_INIT;
  DDRB=DDRB_INIT;

  // Initialize Port D
  PORTD=PORTD_INIT;
  DDRD=DDRD_INIT;

  // Initiualize UART
  UBRR=UBRR_INIT;
  UCR=UCR_INIT;

  // Initialize LED-Timer
  TCNT0=TIMER0_RELOAD;
  TCCR0=TCCR0_INIT;
  TIMSK=TIMSK_INIT;

  // Enable interrupts
  sei();

  // Switch on LED3
  LED_PORT |=(1 << LED3_PIN);

  // Main loop
  while (1) {
    Error=0;
    data=UARTRecvByte();
    if (!Error) {
      // Command block received?
      if (data==P_BLKSTART) {
        data=UARTRecvByte();
        if (!Error) {
          // Execute command.
          switch (data) {
            case P_DMXOUT96:  DMXSendChannels(96);  break;
            case P_DMXOUT256: DMXSendChannels(256); break;
            case P_DMXOUT512: DMXSendChannels(512); break;
            default: Error|=E_PROTOCOLERROR;
          }
          if (!Error) {
            // Block ok?
            data=UARTRecvByte();
            if (data!=P_BLKEND) Error|=E_PROTOCOLERROR;
          }
        }
      } else Error|=E_PROTOCOLERROR;
    }
    if (Error) {
      // Indicate error.
      TimerLED2=LED2_ONTIME;
      // Send error reply to master.
      UARTSendByte(P_BLKSTART);
      UARTSendByte(P_ERROR);
      UARTSendByte(P_BLKEND);
    } else {
      // Indicate success.
      TimerLED1=LED1_ONTIME;
      // Send succes reply to master.
      UARTSendByte(P_BLKSTART);
      UARTSendByte(P_ACK);
      UARTSendByte(P_BLKEND);
    }
  }
  return 0;
}
//---------------------------------------------------------------------------
