/***************************************************************************************************/
/*
  This is an Arduino sketch for RotaryEncoder library using interrupts

  written by : enjoyneering79
  sourse code: https://github.com/enjoyneering/

  This sketch uses interrupts, specials pins are required to interface

  Board:                                  int.0  int.1  int.2  int.3  int.4  int.5
  Uno, Mini, Pro (ATmega168 & ATmega328)  2      3      x       x      x     x
  Mega2560                                2      3      21      20     19    18
  Leonardo, Micro (ATmega32U4)            3      2      0       1      7     x
  Due                                     all digital pins
  Zero                                    all digital pins, except 4
  ESP8266                                 all digital pins, except gpio6 - gpio11 & gpio16

  NOTE: - LOW     interrupt trigges whenever the pin is low
        - HIGH    interrupt triggers whenever the pin is high (Arduino Due, Zero, MKR1000 only)
        - CHANGE  interrupt triggers whenever the pin changes value
        - RISING  interrupt triggers when the pin goes from low to high
        - FALLING interrupt triggers when the pin goes from high to low

  BSD license, all text above must be included in any redistribution
*/
/***************************************************************************************************/
#include <RotaryEncoder.h>

#define PIN_A  5 //ky-040 clk pin
#define PIN_B  6 //ky-040 dt  pin
#define BUTTON 7 //ky-040 sw  pin

int16_t position = 0;

RotaryEncoder encoder(PIN_A, PIN_B, BUTTON);


void encoderISRa()
{
  encoder.updateA();
}

void encoderISRb()
{
  encoder.updateB();
}

void encoderISRpb()
{
  encoder.updatePB();
}

void setup()
{
  encoder.begin();

  attachInterrupt(digitalPinToInterrupt(PIN_A),  encoderISRa,  CHANGE); //call encoderISRa()  when any high/low changes happen on the pin
  attachInterrupt(digitalPinToInterrupt(PIN_B),  encoderISRb,  CHANGE); //call encoderISRb()  when any high/low changes happen on the pin
  attachInterrupt(digitalPinToInterrupt(BUTTON), encoderISRpb, LOW);    //call encoderISRpb() when low changes happen on the pin

  Serial.begin(115200);
}


void loop()
{
  if (position != encoder.readPosition())
  {
    position = encoder.readPosition();
    Serial.println(position);
  }
  
  if (encoder.readPushButton() == LOW)
  {
    Serial.println("PRESSED");
  }
}
