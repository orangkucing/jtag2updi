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
#include "UPDI_lo_lvl.h"

void SYS::init(void) {

#ifndef __AVR_ATmega16__
#  if defined XTINY
  // Set clock speed to maximum (default 20MHz, or 16MHz set by fuse)
  _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, 0);
  /* Disable unused peripherals */
  //ToDo
# else
#   if defined(ARDUINO_AVR_LARDU_328E)
  clock_prescale_set ( (clock_div_t) __builtin_log2(32000000UL / F_CPU));
#   endif
  /* Disable digital input buffers on port C */
  DIDR0 = 0x3F;
  /* Disable unused peripherals */
  ACSR = 1 << ACD;    // turn off comparator
# endif
  /* Enable all UPDI port pull-ups */
  PORT(UPDI_PORT) = 0xFF;
  /* Enable all LED port pull-ups, except for the LED pin and HV pin */
  PORT(LED_PORT) = 0b11001111;
  /* Configure LED and HV pins as outputs */
  DDR(LED_PORT) |= 0b00110000;

#else
  /* No interrupts */
  sei();
  /* Enable all UPDI port pull-ups */
  PORT(UPDI_PORT) = 0xFF;
  /* Enable LED */
  PORT(LED_PORT) |= (1 << LED_PIN);
  /* Enable all LED port pull-ups, except for the LED pin */
  PORT(LED_PORT) = 0xFF - (1 << LED_PIN);


  /* Disable unused peripherals */
  SPCR &= ~(1 << SPE);
  // ADC  &= ~(1<<ADEN);  target power overload function needs ADC
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
/* LED on for any traffic, remains on when avrdude hangs, blinks for OVL indication */
void SYS::setLED(void) {
  PORT(LED_PORT) |= 1 << LED_PIN;
}

void SYS::clearLED(void) {
  PORT(LED_PORT) &= ~(1 << LED_PIN);
}

void SYS::pulseHV(void) {
  PORT(LED_PORT) |= 0b00010000;     // set HV
  _delay_us(250);                   // duration (allowable range = 100-1000µs)
  PORT(LED_PORT) &= 0b11101111;     // clear HV
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
  _delay_us(1000);         // Debugger.txd = z (allowable range = 200-14000µs)
}

void SYS::setPOWER(void) {
  DDRC |= 0b00111111;         // enable pullups
  PORTC |= 0b00111111;        // set as outputs
  _delay_us(10);
}

void SYS::clearPOWER(void) {
  DDRC &= 0b11000000;         // disable pullups
  PORTC &= 0b11000000;        // set as inputs
}

void SYS::cyclePOWER(void) {
  SYS::clearPOWER();
  _delay_us(10000);
  SYS::setPOWER();
  _delay_us(2000);
}

void SYS::updiENABLE(void) {
  SYS::pulseHV();
  _delay_us(200);
  UPDI_io::put(UPDI::SYNCH);
  _delay_us(100);
}

void SYS::checkOVERLOAD(void) {
  // Use A6 to check for overload on A0-A5 (target power)
  ADCSRA =  (1 << ADEN); // turn ADC on
  ADCSRA |= (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2);  // Prescaler of 128
  ADMUX = (1 << REFS0) | (1 << ADLAR) | (6 & 0x07);  // Use AVcc reference, 8-bit result, select A6

  uint16_t sum = 0;
  for (int i = 0 ; i < 250 ; i++) {  // totalize 250 8-bit readings
    ADCSRA  |= (1 << ADSC);          // start a conversion
    while (ADCSRA &  (1 << ADSC));   // wait while busy
    sum += ADCH;                     // totalize ADC result
  }
  if ((sum / 250) <= 230) {          // if voltage on shorted A0-A5 outputs <= 4.5V
    while (1) {                      // OVERLOAD (fix circuit then press Reset)
      SYS::clearPOWER();             // turn off target power then flash LED at 4Hz
      SYS::setLED();
      _delay_us(50000);
      SYS::clearLED();
      _delay_us(200000);
    }
  }
}

uint8_t SYS::checkHVMODE() {
  // check HV Programming Mode Switch on A7
  ADCSRA =  (1 << ADEN); // turn ADC on
  ADCSRA |= (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2);  // Prescaler of 128
  ADMUX = (1 << REFS0) | (1 << ADLAR) | (7 & 0x07);  // Use AVcc reference, 8-bit result, select A7
  ADCSRA  |= (1 << ADSC);          // start a conversion
  while (ADCSRA &  (1 << ADSC));   // wait while busy
  return ADCH;                     // return HV mode jumper setting
}
