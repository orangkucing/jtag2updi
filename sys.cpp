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

  DDR(LED_PORT) |= (1 << LED_PIN);
  #ifdef LED2_PORT
  DDR(LED2_PORT) |= (1 << LED2_PIN);
  #endif
  TIMER_HOST_MAX=HOST_TIMEOUT;
  TIMER_TARGET_MAX=TARGET_TIMEOUT;
  #if defined(DEBUG_ON)
  DBG::debug(0x18,0xC0,0xFF, 0xEE);
  #endif


  #if defined (__AVR_ATmega328P__)
  // Dickson charge pump - Bit 4,3,2,1,0: HVPWR4 Power, HVSD3 Shutdown, HVCP2 Clock, HVCP1 Clock, HVLED
  DDRB |=   0b00011111;    // configure HVPWR4, HVSD3, HVCP2, HVCP1, HVLED as outputs
  PORTB &= ~0b00011110;    // clear HVPWR4, HVSD3, HVCP2, HVCP1
  PORTB |=  0b00001000;    // set HVSD3

  #elif defined (__AVR_ATtiny_Zero_One__)
  // Output Pins
  PORTA.DIRSET |= PIN2_bm | cpp | cp1 |cp2 | cps | PIN7_bm; // Power Switch, Charge Pump (4 pins), LED
  PORTB.DIRSET |= PIN1_bm; // HVLED
  // Set Outputs High
  PORTA.OUTSET |= PIN2_bm | cps; // enable power switch and charge pump shutdown
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

void SYS::pulseHV(void) {
#if defined (__AVR_ATmega328P__)
  PORTB &= ~0b00001000; // clear HVSD3
  PORTB |=  0b00010000; // set HVPWR4
#elif defined (__AVR_ATtiny_Zero_One__)
  PORTA.OUTCLR &= cps; // disable shutdown
  PORTA.OUTSET |= cpp; // turn on power
#endif
  for (int j = 0; j <= 255; j++) {
#if defined (__AVR_ATmega328P__)
    PORTB &= ~0b00000100; // clear HVCP2
    PORTB |=  0b00000010; // set HVCP1
#elif defined (__AVR_ATtiny_Zero_One__)
    PORTA.OUTCLR &= cp2;
    PORTA.OUTSET |= cp1;
#endif
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
#if defined (__AVR_ATmega328P__)
    PORTB &= ~0b00000010; // clear HVCP1
    PORTB |=  0b00000100; // set HVCP2
#elif defined (__AVR_ATtiny_Zero_One__)
    PORTA.OUTCLR &= cp1;
    PORTA.OUTSET |= cp2;
#endif
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
#if defined (__AVR_ATmega328P__)
  PORTB &= ~0b00000100; // clear HVCP2
  PORTB &= ~0b00010000; // clear HVPWR4
  PORTB |=  0b00001000; // set HVSD3
#elif defined (__AVR_ATtiny_Zero_One__)
  PORTA.OUTCLR &= cp2;  // default
  PORTA.OUTCLR &= cpp;  // turn off power
  PORTA.OUTSET |= cps;  // enable shutdown
#endif
  _delay_us(8); // delay (allowable range = 1-10µs)
}

void SYS::setPOWER(void) {
#if defined (__AVR_ATmega328P__)
  DDRC |= 0b00111111;   // enable pullups
  PORTC |= 0b00111111;  // set as outputs
#elif defined (__AVR_ATtiny_Zero_One__)
  PORTA.OUTSET |= PIN2_bm;  // PA2 high
#endif
  _delay_us(10);
}

void SYS::clearPOWER(void) {
#if defined (__AVR_ATmega328P__)
  DDRC &= 0b11000000;   // disable pullups
  PORTC &= 0b11000000;  // set as inputs
#elif defined (__AVR_ATtiny_Zero_One__)
  PORTA.OUTCLR |= PIN2_bm;  // PA2 low
#endif
}

void SYS::cyclePOWER(void) {
  SYS::clearPOWER();
  _delay_us(10000);
  SYS::setPOWER();
  _delay_us(2000);
}

void SYS::checkOVERLOAD(void) {                      // Use A6 to sense overload on A0-A5 (target power)
#if defined (__AVR_ATmega328P__)                     // Arduino Nano
  ADCSRA = (1 << ADEN);                              // turn ADC on
  ADCSRA |= (1 << ADPS0) | (1 << ADPS2);             // prescaler of 32
  ADMUX = (1 << REFS0) | (1 << ADLAR) | (6 & 0x07);  // AVcc reference, 8-bit result, select A6
  uint16_t sum = 0;
  for (int i = 0 ; i < 250 ; i++) {                  // totalize 250 8-bit readings
    ADCSRA  |= (1 << ADSC);                          // start a conversion
    while (ADCSRA &  (1 << ADSC));                   // wait while busy
    sum += ADCH;                                     // totalize ADC result
  }
  if ((sum / 250) <= 230) {                          // if voltage on shorted A0-A5 outputs <= 4.5V
    while (1) {                                      // OVERLOAD (fix circuit then press Reset)
      clearPOWER();                                  // turn off target power then flash LED at 4Hz
      setLED();
      _delay_us(50000);
      clearLED();
      _delay_us(200000);
    }
  }
# endif
}

uint8_t SYS::checkHVMODE() {                         // Check HV Programming Mode Switch
#if defined (__AVR_ATmega328P__)                     // Arduino Nano
  ADCSRA =  (1 << ADEN);                             // turn ADC on
  ADCSRA |= (1 << ADPS0) | (1 << ADPS2);             // prescaler of 32
  ADMUX = (1 << REFS0) | (1 << ADLAR) | (7 & 0x07);  // use AVcc reference, 8-bit result, select A7
  ADCSRA  |= (1 << ADSC);                            // start a conversion
  while (ADCSRA &  (1 << ADSC));                     // wait while busy
  return ADCH;                                       // return HV mode jumper setting
#elif defined (__AVR_ATtiny_Zero_One__)
  PORTA_PIN1CTRL = 0x04;                             // disable digital input buffer for PA1
  ADC0_CTRLA = ADC_RESSEL_8BIT_gc;                   // 8-bit resolution
  ADC0_CTRLC = 0x54;                                 // reduced capacitance, Vdd ref, prescaler of 32
  ADC0_MUXPOS = 0x01;                                // select AIN1
  ADC0_CTRLA |= ADC_ENABLE_bm;                       // turn ADC on
  ADC0_COMMAND |= ADC_STCONV_bm;                     // start a conversion
  while (ADC0_COMMAND & ADC_STCONV_bm);              // wait while busy
  return ADC0_RESL;                                  // return HV mode jumper setting
#else
  return 0;
# endif
}
