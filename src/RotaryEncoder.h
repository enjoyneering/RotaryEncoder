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
#ifndef RotaryEncoder_h
#define RotaryEncoder_h

#if ARDUINO >= 100
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#if defined (__AVR__)
 #include <avr/pgmspace.h>
#elif defined(ESP8266)
 #include <pgmspace.h>
#endif

#define CW   0x00
#define CCW  0x01
#define STOP 0x02

class RotaryEncoder
{
  public:
    RotaryEncoder(uint8_t encoderA, uint8_t encoderB, uint8_t encoderButton);

    void     begin(void);
    void     readAB();
    void     readPushButton();

    int16_t  getPosition(void);
    bool     getPushButton(void);
  
    void     setPosition(int16_t position);
    void     setPushButton(bool state);

  private:
             uint8_t _encoderA;
             uint8_t _encoderB;
             uint8_t _encoderButton;

    volatile uint8_t _currValueB   = 0;
    volatile bool    _buttonState  = true; //INPUT_PULLUP is ON

             int16_t _counter      = -1;
};

#endif
