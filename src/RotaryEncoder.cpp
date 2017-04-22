/***************************************************************************************************/
/*
  This is an Arduino library for Quadrature Rotary Encoder 

  written by : enjoyneering79
  sourse code: https://github.com/enjoyneering/

  This library can uses interrupts, specials pins are required to interface

  Board:                                  int.0  int.1  int.2  int.3  int.4  int.5
  Uno, Mini, Pro (ATmega168 & ATmega328)  2      3      x       x      x     x
  Mega2560                                2      3      21      20     19    18
  Leonardo, Micro (ATmega32U4)            3      2      0       1      7     x
  Due                                     all digital pins
  Zero                                    all digital pins, except 4
  ESP8266                                 all digital pins, except gpio6 - gpio11 & gpio16

  NOTE: - Quadrature encoder makes two waveforms that are 90 deg. out of phase.

                           _______         _______         __
                  PinA ___|       |_______|       |_______|   PinA
          CCW <--              _______         _______             --> CW
                  PinB _______|       |_______|       |______ PinB

          The half of the pulses from top to bottom create full state array:  

          cur.A+B   prev.A+B   (cur.AB+prev.AB)  Array   Encoder State
          -------   ---------   --------------   -----   -------------
            00         00            0000          0     stop/idle
            00         01            0001         -1     CCW, 0x01
            00         10            0010          1     CW,  0x02
            00         11            0011          0     invalid state
            01         00            0100          1     CW,  0x04
            01         01            0101          0     stop/idle
            01         10            0110          0     invalid state
            01         11            0111         -1     CCW, 0x07
            10         00            1000         -1     CCW, 0x08
            10         01            1001          0     invalid state
            10         10            1010          0     stop/idle
            10         11            1011          1     CW,  0x0B
            11         00            1100          0     invalid state
            11         01            1101          1     CW,  0x0D 
            11         10            1110         -1     CCW, 0x0E
            11         11            1111          0     stop/idle

        - Most rotary encoders make [latch] sounds. This position called "latch state".
          Latch happends after "n" number of {pulses}/ticks and vary from model to model.
          For example: {00}, {01}, {11}, [10], {00}, {01}, {11}, [10] ..., etc.

  BSD license, all text above must be included in any redistribution
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
  _encoderA        = encoderA;
  _encoderB        = encoderB;
  _encoderButton   = encoderButton;
}

/**************************************************************************/
/*
    begin
*/
/**************************************************************************/
void RotaryEncoder::begin()
{
  pinMode(_encoderA,      INPUT_PULLUP);
  pinMode(_encoderB,      INPUT_PULLUP);
  pinMode(_encoderButton, INPUT_PULLUP);
}

/**************************************************************************/
/*
    updateA()

    Reads A values

    NOTE: - always call this function before readPosition()
          - this function must take no parameters and return nothing
            if used with Interrupt
*/
/**************************************************************************/
void RotaryEncoder::updateA()
{
  //noInterrupts();                       //disable interrupts, same as "cli()"
  _currValueA = digitalRead(_encoderA);
  //interrupts();                         //enable interrupts, same as "sei()"
}

/**************************************************************************/
/*
    updateB()

    Reads B values

    NOTE: - always call this function before readPosition()
          - this function must take no parameters and return nothing
            if used with Interrupt
*/
/**************************************************************************/
void RotaryEncoder::updateB()
{
  _currValueB = digitalRead(_encoderB);
}

/**************************************************************************/
/*
    updatePB()

    Reads push button value

    NOTE: - always call this function before readPushButton()
          - this function must take no parameters and return nothing
            if used with Interrupt
*/
/**************************************************************************/
void RotaryEncoder::updatePB()
{
  _buttonState = digitalRead(_encoderButton);
  //_buttonState = !_buttonState;
}

/**************************************************************************/
/*
    readPosition()

    Return encoder position
*/
/**************************************************************************/
int16_t RotaryEncoder::readPosition()
{
  uint8_t direction = 0;

  /* direction = cw << 1 | ccw */
  direction  = ((_prevValueA ^ _currValueB) & ~(_prevValueB ^ _currValueA)) << 1 | (_prevValueB ^ _currValueA) & ~(_prevValueA ^ _currValueB);

  _prevValueA = _currValueA;
  _prevValueB = _currValueB;

  switch (direction)
  {
    case 0x02:
      _tickCounter++;     
      break;
    case 0x01:
      _tickCounter--;
      break;
  }

  switch (_tickCounter)
  {
    case  ROTARY_ENCODER_LATCH:
      _latchCounter += 1;
      _tickCounter   = 0;
      break;
    case ~ROTARY_ENCODER_LATCH:
      _latchCounter -= 1;
      _tickCounter   = 0;
      break;
  }

  return _latchCounter;
}

/**************************************************************************/
/*
    readPushButton()

    Return encoder button state
*/
/**************************************************************************/
bool RotaryEncoder::readPushButton()
{
  return _buttonState;
}

/**************************************************************************/
/*
    setPosition()

    Manualy sets new encoder position
*/
/**************************************************************************/
void RotaryEncoder::setPosition(int16_t position)
{
  _latchCounter = position;
}

/**************************************************************************/
/*
    setPushButton()

    Manualy sets new encoder push button state
*/
/**************************************************************************/
void RotaryEncoder::setPushButton(bool state)
{
  _buttonState = state;
}
