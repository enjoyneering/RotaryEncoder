/***************************************************************************************************/
/*
   This is an Arduino library for Quadrature Rotary Encoder 

   written by : enjoyneering
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
   - Quadrature encoder makes two waveforms that are 90 degree out of phase:
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
   AVR     Core          - https://github.com/arduino/ArduinoCore-avr
   ATtiny  Core          - https://github.com/SpenceKonde/ATTinyCore
   ESP32   Core          - https://github.com/espressif/arduino-esp32
   ESP8266 Core          - https://github.com/esp8266/Arduino
   STM32   Core          - https://github.com/stm32duino/Arduino_Core_STM32
                         - https://github.com/rogerclarkmelbourne/Arduino_STM32

   GNU GPL license, all text above must be included in any redistribution,
   see link for details  - https://www.gnu.org/licenses/licenses.html
*/
/***************************************************************************************************/

#include "RotaryEncoderAdvanced.h"



/**************************************************************************/
/*
    Constructor
*/
/**************************************************************************/
template <typename T> RotaryEncoderAdvanced<T>::RotaryEncoderAdvanced(uint8_t encoderA, uint8_t encoderB, uint8_t encoderButton, T stepsPerClick, T minValue, T maxValue) : RotaryEncoder(encoderA, encoderB, encoderButton)
{
  setStepsPerClick(stepsPerClick);
  setMinValue(minValue);
  setMaxValue(maxValue);
}

/**************************************************************************/
/*
    getValue()

    Return encoder value
*/
/**************************************************************************/
template <typename T> T RotaryEncoderAdvanced<T>::getValue()
{
  T value = (T)_counter * _stepsPerClick;

  /* make shure value never gets higher or lower thresholds */
  if (value > _maxValue)
  {
    value = _maxValue;
    setValue(value);
  }
  else if (value < _minValue)
  {
    value = _minValue;
    setValue(value);
  }
  
  return value;
}

/**************************************************************************/
/*
    setValue()

    Manualy sets encoder value
*/
/**************************************************************************/
template <typename T> void RotaryEncoderAdvanced<T>::setValue(T value)
{
  _counter = value / _stepsPerClick;
}

/**************************************************************************/
/*
    getStepsPerClick()

    Return steps per click value
*/
/**************************************************************************/
template <typename T> T RotaryEncoderAdvanced<T>::getStepsPerClick()
{
  return _stepsPerClick;
}

/**************************************************************************/
/*
    setStepsPerClick()

    Set steps per click value
*/
/**************************************************************************/
template <typename T> void RotaryEncoderAdvanced<T>::setStepsPerClick(T value)
{
  if (value == 0) return; //division by zero is prohibited

  _stepsPerClick = value;
}

/**************************************************************************/
/*
    getMinValue()

    Return minimum value
*/
/**************************************************************************/
template <typename T> T RotaryEncoderAdvanced<T>::getMinValue()
{
  return _minValue;
}

/**************************************************************************/
/*
    setMinValue()

    Set minimum value
*/
/**************************************************************************/
template <typename T> void RotaryEncoderAdvanced<T>::setMinValue(T value)
{
  _minValue = value;
}

/**************************************************************************/
/*
    getMaxValue()

    Return maximun value
*/
/**************************************************************************/
template <typename T> T RotaryEncoderAdvanced<T>::getMaxValue()
{
  return _maxValue;
}

/**************************************************************************/
/*
    setMaxValue()

    Set maximun value
*/
/**************************************************************************/
template <typename T> void RotaryEncoderAdvanced<T>::setMaxValue(T value)
{
  _maxValue = value;
}

/**************************************************************************/
/*
    setValues()

    Set current value, steps per click, minimum & maximun values
*/
/**************************************************************************/
template <typename T> void RotaryEncoderAdvanced<T>::setValues(T currValue, T stepsPerClick, T minValue, T maxValue)
{
  noInterrupts();                  //disable interrupts

  setStepsPerClick(stepsPerClick);
  setMinValue(minValue);
  setMaxValue(maxValue);
  setValue(currValue);

  interrupts();                    //enable interrupts
}
