/***************************************************************************************************/
/*
   This is an Arduino library for Quadrature Rotary Encoder 

   written by : enjoyneering79
   sourse code: https://github.com/enjoyneering/RotaryEncoder

   This library uses interrupts, specials pins are required to interface
   Board:                                    int.0  int.1  int.2  int.3  int.4  int.5            Level
   Uno, Mini, Pro, ATmega168, ATmega328..... 2      3      x       x      x     x                5v
   Mega2560................................. 2      3      21      20     19    18               5v
   Leonardo, Micro, ATmega32U4.............. 3      2      0       1      7     x                5v
   Digistump, Trinket, ATtiny85............. 2/physical pin 7                                    5v
   Due, SAM3X8E............................. all digital pins                                    3v
   Zero, ATSAMD21G18........................ all digital pins, except pin 4                      3v
   Blue Pill, STM32F103xxxx boards.......... all digital pins, maximum 16 pins at the same time  3v
   ESP8266.................................. all digital pins, except gpio6 - gpio11 & gpio16    3v/5v
   ESP32.................................... all digital pins                                    3v

   NOTE:
   - Quadrature encoder makes two waveforms that are 90° out of phase:
                           _______         _______         __
                  PinA ___|       |_______|       |_______|   PinA
          CCW <--              _______         _______
                  PinB _______|       |_______|       |______ PinB

                               _______         _______
                  PinA _______|       |_______|       |______ PinA
          CW  -->          _______         _______         __
                  PinB ___|       |_______|       |_______|   PinB


          The half of the pulses from top to bottom create full state array:  

          prev.A+B   cur.A+B   (prev.AB+cur.AB)  Array   Encoder State
          -------   ---------   --------------   -----   -------------
            00         00            0000          0     stop/idle
            00         01            0001          1     CW,  0x01
            00         10            0010         -1     CCW, 0x02
            00         11            0011          0     invalid state
            01         00            0100         -1     CCW, 0x04
            01         01            0101          0     stop/idle
            01         10            0110          0     invalid state
            01         11            0111          1     CW, 0x07
            10         00            1000          1     CW, 0x08
            10         01            1001          0     invalid state
            10         10            1010          0     stop/idle
            10         11            1011         -1     CCW, 0x0B
            11         00            1100          0     invalid state
            11         01            1101         -1     CCW, 0x0D 
            11         10            1110          1     CW,  0x0E
            11         11            1111          0     stop/idle

          - CW  states 0b0001, 0b0111, 0b1000, 0b1110
          - CCW states 0b0010, 0b0100, 0b1011, 0b1101

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

#include "RotaryEncoder.h"


/**************************************************************************/
/*
    Constructor
*/
/**************************************************************************/
RotaryEncoder::RotaryEncoder(uint8_t encoderA, uint8_t encoderB, uint8_t encoderButton)
{
  _encoderA      = encoderA;
  _encoderB      = encoderB;
  _encoderButton = encoderButton;
}

/**************************************************************************/
/*
    begin

    Sets encoders pins as input & enables built-in pullup resistors

    NOTE:
    - high value of pull-up resistor limits the speed
    - typical value of external pull-up resistor is 10kOhm, acceptable
      range 5kOhm..100kOhm
    - for ESP8266 value of internal pull-up resistors is 30kOhm..100kOhm
    - for AVR     value of internal pull-up resistors is 30kOhm..60kOhm
*/
/**************************************************************************/
void RotaryEncoder::begin()
{
  pinMode(_encoderA,      INPUT_PULLUP); //enable internal pull-up resistors 
  pinMode(_encoderB,      INPUT_PULLUP);
  pinMode(_encoderButton, INPUT_PULLUP);
}

/**************************************************************************/
/*
    readAB()

    Reads "A" & "B" pins value

    NOTE:
    - always call this function before getPosition()
    - 100nF/0.1μF capacitors between A & B channel pin & ground is a must!!!
    - for fast MCU like Cortex use Interrupt Service Routine with "CHANGE"
      parameter, ISR called when pin "A" changes from "1" to "0"
      or from "0" to "1"
    - for slow MCU like 8-bit AVR use Timer1 interrupt & TimerOne library
    - the ISR function must take no parameters & return nothing
    - delay() doesn't work during ISR & millis() doesn't increment
    - declare all global variables inside ISR as "volatile", it prevent
      compiler to make any optimization & unnecessary changes in the code
      with the variable
*/
/**************************************************************************/
void RotaryEncoder::readAB()
{
  noInterrupts();                                               //disable interrupts

  #if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) //slow MCU
  _currValueAB  = (uint8_t)_digitalReadFast(_encoderA) << 1;
  _currValueAB |= (uint8_t)_digitalReadFast(_encoderB);
  #else                                                         //fast MCU
  _currValueAB  = digitalRead(_encoderA) << 1;
  _currValueAB |= digitalRead(_encoderB);
  #endif

  switch ((_prevValueAB | _currValueAB))
  {
    #if defined(__AVR__)                                        //slow MCU
    case 0b1110:                                                //CW states, 1 count  per click
  //case 0b0001: case 0b1110:                                   //CW states, 2 counts per click
    #else                                                       //fast MCU
    case 0b0001: case 0b1110:                                   //CW states, 1 count  per click
  //case 0b0001: case 0b1110: case 0b1000: case 0b0111:         //CW states, 2 counts per click
    #endif
      _counter++;
      break;

    #if defined(__AVR__)                                        //slow MCU
    case 0b0100:                                                //CCW states, 1 count  per click
  //case 0b0100: case 0b1011:                                   //CCW states, 2 count  per click
    #else                                                       //fast MCU
    case 0b0100: case 0b1011:                                   //CCW states, 1 count  per click
  //case 0b0100: case 0b1011: case 0b0010: case 0b1101:         //CCW states, 2 counts per click
    #endif
      _counter--;
      break;
  }

  _prevValueAB = _currValueAB << 2;                             //update previouse state

  interrupts();                                                 //enable interrupts
}

/**************************************************************************/
/*
    readPushButton()

    Reads push button value

    NOTE:
    - always call this function before getPushButton()
    - add 100nF/0.1μF capacitors between button pin & ground to
      reduce bounce!!!
    - designed to use with Interrupt Service Routine with "FALLING"
      parameter, ISR called when push button pin changes from "1" to "0"
    - the ISR function must take no parameters & return nothing
    - delay() doesn't work during ISR & millis() doesn't increment
    - declare all global variables inside ISR as "volatile", it prevent
      compiler to make any optimization & unnecessary changes in the code
      with the variable
*/
/**************************************************************************/
void RotaryEncoder::readPushButton()
{
  noInterrupts();                             //disable interrupts

  _buttonState = digitalRead(_encoderButton); //HIGH not pressed & LOW pressed, because internal pull-up resistor is enabled

  interrupts();                               //enable interrupts
}

/**************************************************************************/
/*
    getPosition()

    Return encoder position
*/
/**************************************************************************/
int16_t RotaryEncoder::getPosition()
{
  return _counter;
}

/**************************************************************************/
/*
    getPushButton()

    Return encoder button state

    NOTE:
    - convert "HIGH" to "false" when button is not presses
    - convert "LOW"  to "true"  when button is pressed
*/
/**************************************************************************/
bool RotaryEncoder::getPushButton()
{
  if (_buttonState == true) return false; //button is not pressed

  _buttonState = true;                    //else button is pressed, flip variable to idle value

  return _buttonState;
}

/**************************************************************************/
/*
    setPosition()

    Manualy sets encoder position
*/
/**************************************************************************/
void RotaryEncoder::setPosition(int16_t position)
{
  _counter = position;
}

/**************************************************************************/
/*
    setPushButton()

    Manualy sets encoder push button state

    NOTE:
    - "true"  button is pressed
    - "false" button is not pressed
*/
/**************************************************************************/
void RotaryEncoder::setPushButton(bool state)
{
  _buttonState = !state;
}


/**************************************************************************/
/*
    _fastDigitalRead()

    Replacemet for Arduino AVR "digitalRead()" function

    NOTE:
    - make shure you call "pinMode(pin, OUTPUT)" before using this
      function
    - Arduino AVR "digitalRead()" is so slow that MCU can't read both
      "A" & "B" pins fast enough during ISR to determine position of encoder
    - about 2.86 times faster than "digitalRead()"
    - ATmega8, ATmega168 & ATmega328P have 3 ports for pins:
      - port B, digital pins 8..13 (do not use B6 & B7, mapped to crystal)
      - port C, analog input pins 0..5
      - port D, digital pins 0..7  (do not use PD0 & PD1, mapped to serial)
    - each port is controlled by 3 registers & each bit of these registers
        corresponds to a single pin:
      - DDRx, port x Data Direction Register & determines whether the pin
        is an INPUT or OUTPUT (where x can be B, C or D)
      - PORTx, port x Data Register & controls whether the pin is HIGH or
        LOW
      - PINx, port x Input Pins Register & reads the state of INPUT pins
*/
/**************************************************************************/
#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
bool RotaryEncoder::_digitalReadFast(uint8_t pin)
{
  switch (pin) //write spaces around " ... ", otherwise it may be parsed wrong
  {
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
      return (PIND) &  (1 << (pin % 8)); //pin 2 (PIND) & 0x04; or (PIND) & (1 << PD2);

    case 9:
    case 10:
    case 11:
    case 12:
    case 13:
      return (PINB) &  (1 << (pin % 8)); //pin 3 (PIND) & 0x08; or (PIND) & (1 << PD3);
  }

  return false;
}
#endif
