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

  /* Initialize writeable parameters to default values */
  JTAG2::init();

}

inline void loop() {
  int progmode_count = 0;
  SYS::setPOWER();
  SYS::checkOVERLOAD();

  while (1) {
    // Receive command
    while (!JTAG2::receive());  // set LED within this function
    // Process command
    switch (JTAG2::packet.body[0]) {
      case JTAG2::CMND_GET_SIGN_ON:
        if (SYS::checkHVMODE() > 200) SYS::cyclePOWER();  // if UDPI as GPIO, power-cycle target
        if (SYS::checkHVMODE() > 100) SYS::updiENABLE();  // if UDPI as GPIO or RESET, apply HV pulse and sync
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
        // Change emulator mode to signal that the UPDI unit must be turned off
        JTAG2::PARAM_EMU_MODE_VAL = JTAG2::emul_mode::EMULATOR_MODE_UNKNOWN;
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
      if (SYS::checkHVMODE() > 200) SYS::cyclePOWER(); // if UDPI as GPIO, power-cycle target
    }

  }
}
}
