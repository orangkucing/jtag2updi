/*
   sys.cpp

   Created: 02-10-2018 13:07:52
    Author: JMR_2
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <util/delay.h>

#include "sys.h"

#ifndef LED_PORT
#define LED_PORT B
#endif
#ifndef LED_PIN
#define LED_PIN 5
#endif
#ifndef HV_PORT
#define HV_PORT B
#endif
#ifndef HV_PIN
#define HV_PIN 4
#endif


void SYS::init(void) {

#ifndef __AVR_ATmega16__

  /* Disable digital input buffers on port C except for ADC6*/
  DIDR0 = 0x3F;
  /* Enable all port D pull-ups */
  PORTD = 0xFF;
  /* Enable all port B pull-ups, except for LED and HV */
  PORT(LED_PORT) = 0b11001111;
  /* Enable both LED and HV as outputs */
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
  /* Enable all port B pull-ups, except for LED and HV */
  PORT(LED_PORT) = 0b11001111;
  /* Enable both LED and HV as outputs */
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
  PORT(LED_PORT) |= 1 << LED_PIN;
}

void SYS::clearLED(void) {
  PORT(LED_PORT) &= ~(1 << LED_PIN);
}

void SYS::pulseHV(void) {
  PORT(HV_PORT) |= 1 << HV_PIN;     // set HV
  _delay_us(250);                   // duration (allowable range = 100-1000µs)
  PORT(HV_PORT) &= ~(1 << HV_PIN);  // clear HV
  _delay_us(3);                     // delay (allowable range = 1-10µs)
}

void SYS::double_breakHV(void) {
  _delay_us(3);
  TCCR0A = 0;
  DDRD |= (1 << DDD6);   // tx enable
  for (uint16_t i = 0; i < 2; i++) {  // send double break, 78µs x 2 = 156µs (max
    PORTD &= ~(1 << DDD6);
    _delay_us(6 * 12);     // low 12 cycle
    PORTD |=  (1 << DDD6);
    _delay_us(6);          // high 1 cycle
  }
  DDRD &= ~(1 << DDD6);    // RX enable
  TCCR0A = (1 << COM0A1) | (1 << COM0A0) | (1 << WGM01);  // setup bit high
  _delay_us(800);         // Debugger.txd = z (allowable range = 200-14000µs)
}

void SYS::setPOWER(void) {
  DDRC |= 0b00111111;         // enable pullups
  PORTC |= 0b00111111;        // set as outputs
}

void SYS::clearPOWER(void) {
  DDRC &= 0b11000000;         // disable pullups
  PORTC &= 0b11000000;        // set as inputs
}
