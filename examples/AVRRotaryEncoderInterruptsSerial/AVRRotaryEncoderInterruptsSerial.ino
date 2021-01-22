/***************************************************************************************************/
/*
   This is an Arduino sketch for RotaryEncoder library using interrupts

   written by : enjoyneering79
   sourse code: https://github.com/enjoyneering/

   This sketch uses interrupts, specials pins are required to interface
   Board:                                    int.0  int.1  int.2  int.3  int.4  int.5            Level
   Uno, Mini, Pro, ATmega168, ATmega328..... 2      3      x       x      x     x                5v
   Mega2560................................. 2      3      21      20     19    18               5v
   Leonardo, Micro, ATmega32U4.............. 3      2      0       1      7     x                5v
   Digistump, Trinket, ATtiny85............. 2/physical pin 7                                    5v
   Due, SAM3X8E............................. all digital pins                                    3v
   Zero, ATSAMD21G18........................ all digital pins, except pin 4                      3v
   Blue Pill, STM32F103xxxx boards.......... all digital pins, maximun 16 pins at the same time  3v
   ESP8266.................................. all digital pins, except gpio6 - gpio11 & gpio16    3v/5v
   ESP32.................................... all digital pins                                    3v

   NOTE:
   - LOW     interrupt trigges whenever the pin is low
   - HIGH    interrupt triggers whenever the pin is high (Arduino Due, Zero, MKR1000 only)
   - CHANGE  interrupt triggers whenever the pin changes value
   - RISING  interrupt triggers when the pin goes from low to high
   - FALLING interrupt triggers when the pin goes from high to low

   Frameworks & Libraries:
   TimerOne AVR          - https://github.com/PaulStoffregen/TimerOne
   ATtiny  Core          - https://github.com/SpenceKonde/ATTinyCore
   ESP32   Core          - https://github.com/espressif/arduino-esp32
   ESP8266 Core          - https://github.com/esp8266/Arduino
   STM32   Core          - https://github.com/stm32duino/Arduino_Core_STM32
                         - https://github.com/rogerclarkmelbourne/Arduino_STM32

   GNU GPL license, all text above must be included in any redistribution,
   see link for details  - https://www.gnu.org/licenses/licenses.html
*/
/***************************************************************************************************/
#pragma GCC optimize ("Os") //code optimization controls, "O2" or "O3" code performance & "Os" code size

#include <RotaryEncoder.h>

#define PIN_A            2     //ky-040 clk pin, interrupt & add 100nF/0.1uF capacitors between pin & ground!!!
#define PIN_B            4     //ky-040 dt  pin,             add 100nF/0.1uF capacitors between pin & ground!!!
#define BUTTON           3     //ky-040 sw  pin, interrupt & add 100nF/0.1uF capacitors between pin & ground!!!

int16_t position = 0;

RotaryEncoder encoder(PIN_A, PIN_B, BUTTON);


/**************************************************************************/
/*
    encoderISR()

    Encoder A & B pin interrupt service routine

    NOTE:
    - use interrupt pin for A pin!!!
    - add 100nF/0.1uF capacitors between A pin & ground!!!
    - add 100nF/0.1uF capacitors between B pin & ground!!!
*/
/**************************************************************************/
void encoderISR()
{
  encoder.readAB();
}

/**************************************************************************/
/*
    encoderButtonISR()

    Encoder button interrupt service routine

    NOTE:
    - use interrupt pin!!!
    - add 100nF/0.1uF capacitors between pin & ground!!!
*/
/**************************************************************************/
void encoderButtonISR()
{
  encoder.readPushButton();
}

/**************************************************************************/
/*
    setup()

    Main setup
*/
/**************************************************************************/
void setup()
{
  encoder.begin();                                                           //set encoders pins as input & enable built-in pullup resistors

  attachInterrupt(digitalPinToInterrupt(PIN_A),  encoderISR,       CHANGE);  //call encoderISR()    every high->low or low->high changes
  attachInterrupt(digitalPinToInterrupt(BUTTON), encoderButtonISR, FALLING); //call encoderButtonISR() every high to low changes

  Serial.begin(115200);
}

/**************************************************************************/
/*
    loop()

    Main loop
*/
/**************************************************************************/
void loop()
{
  if (position != encoder.getPosition())
  {
    position = encoder.getPosition();
    Serial.println(position);
  }
  
  if (encoder.getPushButton() == true) Serial.println(F("PRESSED"));         //(F()) saves string to flash & keeps dynamic memory free
}
