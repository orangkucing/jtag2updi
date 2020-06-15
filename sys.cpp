/*
 * sys.cpp
 *
 * Created: 02-10-2018 13:07:52
 *  Author: JMR_2
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <util/delay.h>
#include "sys.h"
#include "UPDI_lo_lvl.h"
#include "dbg.h"
#include <stdio.h>
#include <string.h>


#include <stdio.h>
#include <string.h>



void SYS::init(void) {
  #ifdef DEBUG_ON
    DBG::initDebug();
  #endif

  #ifdef XAVR
    #ifdef __AVR_DA__
      #if (F_CPU == 24000000)
        /* No division on clock */
        _PROTECTED_WRITE(CLKCTRL_OSCHFCTRLA, (CLKCTRL_OSCHFCTRLA & ~CLKCTRL_FREQSEL_gm ) | (0x09<< CLKCTRL_FREQSEL_gp ));

      #elif (F_CPU == 20000000)
        /* No division on clock */
        _PROTECTED_WRITE(CLKCTRL_OSCHFCTRLA, (CLKCTRL_OSCHFCTRLA & ~CLKCTRL_FREQSEL_gm ) | (0x08<< CLKCTRL_FREQSEL_gp ));

      #elif (F_CPU == 16000000)
        /* No division on clock */
        _PROTECTED_WRITE(CLKCTRL_OSCHFCTRLA, (CLKCTRL_OSCHFCTRLA & ~CLKCTRL_FREQSEL_gm ) | (0x07<< CLKCTRL_FREQSEL_gp ));
      #else
        #error "F_CPU defined as an unsupported value"
      #endif
    #else //0-series or 1-series
      _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, 0);
    #endif
  #else
    #if defined(ARDUINO_AVR_LARDU_328E)
	  #include <avr/power.h>
      clock_prescale_set ( (clock_div_t) __builtin_log2(32000000UL / F_CPU));
    #endif
	  PORT(UPDI_PORT) = 1<<UPDI_PIN;
  #endif

  // Dickson charge pump - Bit 4,3,2,1: HVPWR4 Power, HVSD3 Shutdown, HVCP2 Clock, HVCP1 Clock
  DDRB |=   0b00011110;    // configure HVPWR4, HVSD3, HVCP2, HVCP1 as outputs
  PORTB &= ~0b00011110;    // clear HVPWR4, HVSD3, HVCP2, HVCP1
  PORTB |=  0b00001000;    // set HVSD3

  DDR(LED_PORT) |= (1 << LED_PIN);
  #ifdef LED2_PORT
  DDR(LED2_PORT) |= (1 << LED2_PIN);
  #endif
  TIMER_HOST_MAX=HOST_TIMEOUT;
  TIMER_TARGET_MAX=TARGET_TIMEOUT;
  #if defined(DEBUG_ON)
  DBG::debug(0x18,0xC0,0xFF, 0xEE);
  #endif
}

void SYS::setLED(void){
	PORT(LED_PORT) |= 1 << LED_PIN;
}

void SYS::clearLED(void){
	PORT(LED_PORT) &= ~(1 << LED_PIN);
}

void SYS::setVerLED(void){
        #ifdef LED2_PORT
        PORT(LED2_PORT) |= 1 << LED2_PIN;
        #endif
}

void SYS::clearVerLED(void){
        #ifdef LED2_PORT
        PORT(LED2_PORT) &= ~(1 << LED2_PIN);
        #endif
}

void SYS::setHVLED(void){
  PORT(HVLED_PORT) |= 1 << HVLED_PIN;
}

void SYS::clearHVLED(void){
  PORT(HVLED_PORT) &= ~(1 << HVLED_PIN);
}

/*
inline void SYS::startTimer()
inline void SYS::stopTimer()

Timeout mechanisms, 5/2020, Spence Konde
*/

uint8_t SYS::checkTimeouts() {
return TIMEOUT_REG;
}
void SYS::clearTimeouts() {
  TIMEOUT_REG=WAIT_FOR_HOST|WAIT_FOR_TARGET;
}

void SYS::pulseHV(void) {
#if defined (__AVR_ATmega_Mini__)
  PORTB &= ~0b00001000; // clear HVSD3
  PORTB |=  0b00010000; // set HVPWR4
  for (int j = 0; j <= 255; j++) {
    PORTB &= ~0b00000100; // clear HVCP2
    PORTB |=  0b00000010; // set HVCP1
    __asm__ __volatile__("nop");
    __asm__ __volatile__("nop");
    __asm__ __volatile__("nop");
    __asm__ __volatile__("nop");
    __asm__ __volatile__("nop");
    __asm__ __volatile__("nop");
    __asm__ __volatile__("nop");
    __asm__ __volatile__("nop");
    __asm__ __volatile__("nop");
    __asm__ __volatile__("nop");
    __asm__ __volatile__("nop");
    __asm__ __volatile__("nop");
    __asm__ __volatile__("nop");
    __asm__ __volatile__("nop");
    __asm__ __volatile__("nop");
    __asm__ __volatile__("nop");
    __asm__ __volatile__("nop");
    __asm__ __volatile__("nop");
    __asm__ __volatile__("nop");
    __asm__ __volatile__("nop");
    __asm__ __volatile__("nop");
    __asm__ __volatile__("nop");
    __asm__ __volatile__("nop");
    __asm__ __volatile__("nop");
    PORTB &= ~0b00000010; // clear HVCP1
    PORTB |=  0b00000100; // set HVCP2
    __asm__ __volatile__("nop");
    __asm__ __volatile__("nop");
    __asm__ __volatile__("nop");
    __asm__ __volatile__("nop");
    __asm__ __volatile__("nop");
    __asm__ __volatile__("nop");
    __asm__ __volatile__("nop");
    __asm__ __volatile__("nop");
    __asm__ __volatile__("nop");
  }
  PORTB &= ~0b00000100; // clear HVCP2
  PORTB &= ~0b00010000; // clear HVPWR4
  PORTB |=  0b00001000; // set HVSD3
# endif
  _delay_us(8); // delay (allowable range = 1-10µs)
}

void SYS::setPOWER(void) {
#if defined (__AVR_ATmega_Mini__)
  DDRC |= 0b00111111;         // enable pullups
  PORTC |= 0b00111111;        // set as outputs
# endif
  _delay_us(10);
}

void SYS::clearPOWER(void) {
#if defined (__AVR_ATmega_Mini__)
  DDRC &= 0b11000000;         // disable pullups
  PORTC &= 0b11000000;        // set as inputs
# endif
}

void SYS::cyclePOWER(void) {
  SYS::clearPOWER();
  _delay_us(10000);
  SYS::setPOWER();
  _delay_us(2000);
}

void SYS::checkOVERLOAD(void) {
#if defined (__AVR_ATmega_Mini__)
  // Use A6 to check for overload on A0-A5 (target power)
  ADCSRA =  (1 << ADEN); // turn ADC on
  ADCSRA |= (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2);  // Prescaler of 128
  ADMUX = (1 << REFS0) | (1 << ADLAR) | (6 & 0x07);      // Use AVcc reference, 8-bit result, select A6

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
# endif
}

uint8_t SYS::checkHVMODE() {
#if defined (__AVR_ATmega_Mini__)
  // check HV Programming Mode Switch on A7
  ADCSRA =  (1 << ADEN);                                 // turn ADC on
  ADCSRA |= (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2);  // Prescaler of 128
  ADMUX = (1 << REFS0) | (1 << ADLAR) | (7 & 0x07);      // Use AVcc reference, 8-bit result, select A7
  ADCSRA  |= (1 << ADSC);                                // start a conversion
  while (ADCSRA &  (1 << ADSC));                         // wait while busy
  return ADCH;                                           // return HV mode jumper setting
# endif
}
