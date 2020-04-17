/*
   j2updi.cpp

   Created: 11-11-2017 22:29:58
   Author : JMR_2
*/

// Includes
#include <util/delay.h>
#include <avr/io.h>
#include "sys.h"
#include "updi_io.h"
#include "UPDI_lo_lvl.h"
#include "UPDI_hi_lvl.h"
#include "JICE_io.h"
#include "JTAG2.h"

/* Internal stuff */
namespace {
// Prototypes
void setup();
void loop();
}

int main(void)
{
  setup();
  loop();
}

/* Internal stuff */
namespace {
inline void setup() {
  /* Initialize MCU */
  SYS::init();
  /* Initialize serial links */
  JICE_io::init();
  UPDI_io::init();

}

inline void loop() {
  int progmode_count = 0;
  bool programming = false;

  SYS::setPOWER();
  _delay_us(10);

  // -------------------------------check for overload (A0-A5 <= 4.5V)-------------------------------

  /* initialize ADC for target power overload detection on A6*/
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
  // -------------------------------end check for overload---------------------------------------

  /* initialize ADC for target power overload detection on A7*/
  ADCSRA =  (1 << ADEN); // turn ADC on
  ADCSRA |= (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2);  // Prescaler of 128
  ADMUX = (1 << REFS0) | (1 << ADLAR) | (7 & 0x07);  // Use AVcc reference, 8-bit result, select A7
  ADCSRA  |= (1 << ADSC);          // start a conversion
  while (ADCSRA &  (1 << ADSC));   // wait while busy
  uint8_t hvMode = ADCH;           // get HV mode jumper setting

  while (1) {
    // Receive command
    while (!JTAG2::receive());  // set LED within this function
    // Process command
    switch (JTAG2::packet.body[0]) {
      case JTAG2::CMND_GET_SIGN_ON:

        if (hvMode > 200) {   // if UDPI as GPIO, power-cycle target
          SYS::clearPOWER();
          _delay_us(10000);
          SYS::setPOWER();
          _delay_us(2000);    // allow 2ms power switch turn-on time
        }
        // -----------start HV programming sequence (8.8ms max)---------
        if (hvMode > 100) {   // if UDPI as GPIO or RESET
          SYS::pulseHV();
          SYS::double_breakHV();
          UPDI_io::put(UPDI::SYNCH);
          _delay_us(100);
        } // ---------end HV programming sequence-----------------------

        JTAG2::sign_on();
        break;
      case JTAG2::CMND_GET_PARAMETER:
        JTAG2::get_parameter();
        break;
      case JTAG2::CMND_SET_PARAMETER:
        JTAG2::set_parameter();
        break;
      case JTAG2::CMND_RESET:
      case JTAG2::CMND_ENTER_PROGMODE:
        JTAG2::enter_progmode();
        break;
      case JTAG2::CMND_SIGN_OFF:
        // Restore default baud rate before exiting
        JTAG2::PARAM_BAUD_RATE_VAL = JTAG2::baud_19200;
      case JTAG2::CMND_LEAVE_PROGMODE:
        JTAG2::leave_progmode();
        progmode_count++;  // for LED control
        break;
      case JTAG2::CMND_GET_SYNC:
      case JTAG2::CMND_GO:
        JTAG2::set_status(JTAG2::RSP_OK);
        break;
      case JTAG2::CMND_SET_DEVICE_DESCRIPTOR:
        JTAG2::set_device_descriptor();
        break;
      case JTAG2::CMND_READ_MEMORY:
        JTAG2::read_mem();
        break;
      case JTAG2::CMND_WRITE_MEMORY:
        JTAG2::write_mem();
        break;
      case JTAG2::CMND_XMEGA_ERASE:
        JTAG2::erase();
        break;
      default:
        JTAG2::set_status(JTAG2::RSP_FAILED);
        break;
    }
    // send response
    JTAG2::answer();
    // some commands need to be executed after sending the answer
    JTAG2::delay_exec();
    if (progmode_count == 2) {  // clear LED only after the 2nd run of JTAG2::leave_progmode() and after commands completed
      SYS::clearLED();
      progmode_count = 0;
      if (hvMode > 200) {      // if UDPI=GPIO, power-cycle target
        SYS::clearPOWER();
        _delay_us(10000);
        SYS::setPOWER();
      }
    }
  }
}
}
