//===========================================================================
// Dateiname : minidmx.c
// Funktion  : DMX-Ausgang für den PC
// Compiler  : WinAVR (http://sourceforge.net/projects/winavr/)
// MCU       : AT90S2313 ATTiny2313
// Quarz     : 9,216 MHz
// Änderungen: 18.02.2001 - V1.0
//             - Erste öffentliche Version
//             05.02.2003 - V1.1
//             - LED1 zeigt nun ein erfolgreiches Senden der DMX Daten an
//             - LED2 zeigt Fehler beim Datenempfang vom PC an
//             - LED3 zeigt das Ende das Startvorgangs an (Daten können nun
//               gesendet werden)
//             21.03.2003 - V2.0 Testversion
//             - Der Quelltext der Firmware wurde in der Programmiersprache
//               C neu geschrieben
//             09.09.2003 - V2.0
//             - Fehler in der Generierung des DMX-Signals behoben
//             17.12.2006 - V2.0 (Ulrich Radig mail@ulrichradig.de)
//             - Anpassung an neuere WinAVR Version
//             - Anpassung an meine Platinen Version
//
// Copyright (C) 2001,2002,2003 Mathias Dzionsko (madz@gmx.de)
//===========================================================================
#include <inttypes.h>
#include <avr\io.h>
#include <avr\interrupt.h>
//#include <avr\signal.h>

#define F_CPU           9216000
#include <util/delay.h>
//---------------------------------------------------------------------------
// Makros

#define BITS2BYTE(b7,b6,b5,b4,b3,b2,b1,b0) \
  (b7<<7|b6<<6|b5<<5|b4<<4|b3<<3|b2<<2|b1<<1|b0)

// Verzögert den Programmablauf um t µs.
#define Delay16(l)      _delay_loop_2(l)
#define DELAY16VAL(t)   ((XTAL*(long)(t))/4000)
//---------------------------------------------------------------------------
// Konstanten

// Taktfrequenz der MCU in kHz (für Delay16)
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

// LED1 zeigt ein erfolgreiches Senden der DMX Daten an
// LED2 zeigt Fehler beim Datenempfang vom PC an
// LED3 zeigt das Ende das Startvorgangs an (Daten können nun
// gesendet werden)
#define LED_PORT        PORTB
#define LED1_PIN        PB2
#define LED2_PIN        PB3
#define LED3_PIN        PB4

// Leuchtdauer der LEDs: 100ms (in 1/100s)
#define LED1_ONTIME     10
#define LED2_ONTIME     10

// Port B Pins
// PB0 - DMX-Ausgang
// PB1 - Switch
// PB2 - LED1
// PB3 - LED2
// PB4 - LED3
#define PORTB_INIT      BITS2BYTE(0,0,0,1,1,1,1,1)
#define DDRB_INIT       BITS2BYTE(0,0,0,1,1,1,0,1)

// Port D Pins
// PD0 - RXD
// PD1 - TXD
// PD6 - Triggersignal für Oszilloskop (zu Testzwecken)
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

// Kommunikationsprotokoll
#define P_BLKSTART      0x5A
#define P_BLKEND        0xA5

#define P_DMXOUT96      0xA0
#define P_DMXOUT256     0xA1
#define P_DMXOUT512     0xA2

#define P_ERROR         0xC0
#define P_ACK           0xC1

// Konstanten für die Variable Error
#define E_BUFFEROVERFLOW        1
#define E_UARTTIMEOUT           2
#define E_PROTOCOLERROR         4
//---------------------------------------------------------------------------
// Globale Variablen

volatile uint8_t        TimerLED1, TimerLED2;   // LED Timer
volatile uint8_t        UARTTimer;
volatile uint8_t        Error;

// Empgangspuffer für die serielle Schnittstelle
uint8_t                 RxBuffer[RX_BUFSIZE];
uint8_t                 RxBufferOut;
volatile uint8_t        RxBufferIn;
//---------------------------------------------------------------------------
// OVERFLOW0 Interrupt

SIGNAL(SIG_OVERFLOW0)
{
  // Timer nachladen
  TCNT0=TIMER0_RELOAD;

  // UARTTimer
  if (UARTTimer) UARTTimer--;
  
  // LED1 ein/ausschalten
  if (TimerLED1) {
    LED_PORT &= ~(1<<LED1_PIN);
    TimerLED1--;
  } else LED_PORT |= (1<<LED1_PIN);

  // LED2 ein/ausschalten
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

  // Nächste Bufferposition ausrechnen
  nextpos=RxBufferIn+1;
  if (nextpos==RX_BUFSIZE) nextpos=0;

  // Ist der Buffer voll?
  if (nextpos==RxBufferOut) {
    // Buffer ist übergelaufen
    Error|=E_BUFFEROVERFLOW;
    tmp = UDR; // UART Data Register leeren
  } else {
    // Empfangenes Byte in den Buffer schreiben
    RxBuffer[RxBufferIn]=UDR;
    RxBufferIn=nextpos;
  }
}
//---------------------------------------------------------------------------
// UARTRecvByte

uint8_t UARTRecvByte(void)
{
  uint8_t data;

  // Warten bis ein Byte im Buffer oder die Zeit abgelaufen ist
  UARTTimer=RX_TIMEOUT;
  while (RxBufferIn==RxBufferOut && UARTTimer) ;

  // Kein Byte angekommen?
  if (RxBufferIn==RxBufferOut) {
    Error|=E_UARTTIMEOUT;
    return 0;
  }
  
  // Byte auslesen
  data=RxBuffer[RxBufferOut++];
  if (RxBufferOut==RX_BUFSIZE) RxBufferOut=0;

  return data;
}
//---------------------------------------------------------------------------
// UARTRSendByte

void UARTSendByte(uint8_t data)
{
  // Warten bis das letzte Byte gesendet wurde
  loop_until_bit_is_set(USR, UDRE);

  // Byte senden
  UDR=data;
}
//---------------------------------------------------------------------------
// DMXSendByte

// Sendet ein Datenbyte über die DMX-Leitung.
// Die benötigten Takte für ein DMX-Bit sind 37. Die Taktfrequenz beträgt
// 9,216 MHz. Das ergibt 4,0147 µs pro Bit. Die Abweichung beträgt dann 0,37
// Prozent, welche innerhalb der zulässigen Toleranz liegt.
// Interrupts werden für 44,3 µs nicht zugelassen.


void DMXSendByte(uint8_t value)
{
  asm volatile (
"                cli                    \n" // keine Ints zulassen
"                clc                    \n" // 1      - Start Bit
"                rcall DMXSendBit       \n" // +3+33=37 Takte
"                                       \n"
"                ror %2                 \n" // 1      - Bit 0
"                rcall DMXSendBit       \n" // +3+33=37 Takte
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
"                sei                    \n"  // Ints wieder zulassen
"                rjmp DMXSendByteExit   \n"
"                                       \n"
"DMXSendBit:     brcs DMXSendBit1       \n"
"                nop                    \n" // 1+1
"                cbi %0, %1             \n" // +2
"                rjmp DMXSendBit2       \n" // +2=6 Takte
"DMXSendBit1:    sbi %0, %1             \n" // 2+2
"                rjmp DMXSendBit2       \n" // +2=6 Takte
"                                       \n"
"DMXSendBit2:    ldi r31,7              \n" // 1
"DMXSendBit3:    dec r31                \n" // +1
"                brne DMXSendBit3       \n" // +1/+2
"                rjmp DMXSendBit4       \n" // +2
"DMXSendBit4:    ret                    \n" // +4=9+6*3=27 Takte
"DMXSendByteExit:                       \n"
//                          %0               %1            %2
    : : "I" (_SFR_IO_ADDR(DMX_PORT)), "I" (DMX_PIN), "r" (value) : "r31"
  );
}

//---------------------------------------------------------------------------
// DMXSendReset

// Sendet ein Reset-Signal mit anschließendem Start-Byte über die
// DMX-Leitung.

void DMXSendReset(void)
{
  DMX_PORT &=~(1<<DMX_PIN);       // RESET-Signal senden
  Delay16(DELAY16VAL(100));     // 100µs (min. 88µs) warten

  DMX_PORT |= (1<<DMX_PIN);       // MARK-Signal senden
  Delay16(DELAY16VAL(10));      // 10µs (min. 8µs) warten

  DMXSendByte(0);               // Startbyte (0) senden
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
// Hauptfunktion

int main(void)
{
  uint8_t data;

  // Port B initialisieren
  PORTB=PORTB_INIT;
  DDRB=DDRB_INIT;

  // Port D initialisieren
  PORTD=PORTD_INIT;
  DDRD=DDRD_INIT;
  
  // UART initialisieren
  UBRR=UBRR_INIT;
  UCR=UCR_INIT;

  // LED-Timer initialisieren
  TCNT0=TIMER0_RELOAD;
  TCCR0=TCCR0_INIT;
  TIMSK=TIMSK_INIT;

  // Interrupts zulassen
  sei();

  // LED3 einschalten
  LED_PORT |=(1 << LED3_PIN);

  // Hauptschleife
  while (1) {
    Error=0;
    data=UARTRecvByte();
    if (!Error) {
      // Blockanfang?
      if (data==P_BLKSTART) {
        data=UARTRecvByte();
        if (!Error) {
          // Befehl vom PC ausführen
          switch (data) {
            case P_DMXOUT96: DMXSendChannels(96); break;
            case P_DMXOUT256: DMXSendChannels(256); break;
            case P_DMXOUT512: DMXSendChannels(512); break;
            default: Error|=E_PROTOCOLERROR;
          }
          if (!Error) {
            // Stimmt das Blockende?
            data=UARTRecvByte();
            if (data!=P_BLKEND) Error|=E_PROTOCOLERROR;
          }
        }
      } else Error|=E_PROTOCOLERROR;
    }
    if (Error) {
      // Fehler anzeigen
      TimerLED2=LED2_ONTIME;
      // Fehler an den PC senden
      UARTSendByte(P_BLKSTART);
      UARTSendByte(P_ERROR);
      UARTSendByte(P_BLKEND);
    } else {
      // Befehl erfolgreich bearbeitet
      TimerLED1=LED1_ONTIME;
      // Bestätigung an den PC senden
      UARTSendByte(P_BLKSTART);
      UARTSendByte(P_ACK);
      UARTSendByte(P_BLKEND);
    }
  }
  return 0;
}
//---------------------------------------------------------------------------
