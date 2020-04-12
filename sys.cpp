/*
   sys.cpp

   Created: 02-10-2018 13:07:52
    Author: JMR_2
*/

#include <avr/io.h>
#include <avr/interrupt.h>

#include "sys.h"

#ifndef LED_PORT
#define LED_PORT B
#endif
#ifndef LED_PIN
#define LED_PIN 5
#endif

void SYS::init(void) {

#ifndef __AVR_ATmega16__

  /* Disable digital input buffers on port C except for ADC6*/
  DIDR0 = 0x3F;
  /* Enable all port D pull-ups */
  PORTD = 0xFF;
  /* Enable all port B pull-ups, except for LED and PULSE */
  PORT(LED_PORT) = 0b11001111;
  /* Enable both LED and PULSE as outputs */
  DDR(LED_PORT) |= 0b00110000;

  /* Disable unused peripherals */
  ACSR = 1 << ACD;		// turn off comparator
  PRR =
    (1 << PRTWI) |		// turn off 2 wire interface
    (1 << PRTIM2) |		// turn off timer 2
    (1 << PRTIM1) |		// turn off timer 1
    ( 1 << PRSPI);		// turn off SPI interface
  //(1 << PRADC);	       turn off the ADC

#else

  /* No interrupts */
  sei();

  /* Enable all port D pull-ups */
  PORT(UPDI_PORT) = 0xFF;
  /* Enable all port B pull-ups, except for LED and PULSE */
  PORT(LED_PORT) = 0b11001111;
  /* Enable both LED and PULSE as outputs */
  DDR(LED_PORT) |= 0b00110000;

  /* Disable unused peripherals */
  SPCR &= ~(1 << SPE);
  // ADC  &= ~(1 << ADEN);  target power overload function needs ADC
  TWCR &= ~(1 << TWEN);

  /* Disable resources after bootloader */
  TIFR   = 0x00;
  TIMSK  = 0x00;
  TCNT1  = 0x0000;
  OCR1A  = 0x0000;
  OCR1B  = 0x0000;
  TCCR1A = 0x0000;
  TCCR1B = 0x0000;

#endif

}
/* LED continuously on for during upload, blinks for OVL indication */
void SYS::setLED(void) {
  PORT(LED_PORT) |= 0b00100000;
}

void SYS::clearLED(void) {
  PORT(LED_PORT) &= 0b11011111;
}

void SYS::setPULSE(void) {
  PORT(LED_PORT) |= 0b00010000;  // 12V pulse enable
}

void SYS::clearPULSE(void) {
  PORT(LED_PORT) &= 0b11101111;  // 12V pulse disable
}

void SYS::setPOWER(void) {
  DDRC |= 0b00111111;         // enable pullups
  PORTC |= 0b00111111;        // set as outputs
}

void SYS::clearPOWER(void) {
  DDRC &= 0b11000000;         // disable pullups
  PORTC &= 0b11000000;        // set as inputs
}
