# HV UPDI Programmers
The Source Code was cloned from https://github.com/ElTangas/jtag2updi, then updated to add new features for HV programming. 

The tinyAVR® 0 series and 1-series are programmed through the Unified Program and Debug Interface (UPDI) that uses the UPDI/reset pin. Since pin is multi-functional and high voltage tolerant, if configured as Reset would require a 12V enable sequence on the pin to place it in UPDI mode and allow programming. If configured as an output, a strict enable sequence is required to protect the pin.

The HV UPDI programmers ([here](https://github.com/Dlloydev/jtag2updi/wiki)) are open source hardware designed to work with tinyAVR® [0-series](https://www.microchip.com/design-centers/8-bit/avr-mcus/device-selection/attiny1607) and [1-series](https://www.microchip.com/design-centers/8-bit/avr-mcus/device-selection/attiny3217) MCUs,  [megaTinyCore](https://github.com/SpenceKonde/megaTinyCore) and the [Arduino IDE](https://www.arduino.cc/en/Main/Software). Low cost and easy to use without interfering with the normal workflow in the Arduino IDE. The Arduino Serial Monitor is used as normal for debugging sketches and more.

![](../jtag2updi.wiki/images/Protocol.png)

#### [Learn More ...](https://github.com/Dlloydev/jtag2updi/wiki)