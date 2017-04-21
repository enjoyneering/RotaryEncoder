/******************************************************************************/
/*
  This is an Arduino sketch for RotaryEncoder library

  written by : enjoyneering79
  sourse code: https://github.com/enjoyneering/

  BSD license, all text above must be included in any redistribution
*/
/******************************************************************************/
#include <RotaryEncoder.h>

#define PIN_A  2 //ky-040 clk pin
#define PIN_B  3 //ky-040 dt  pin
#define BUTTON 4 //ky-040 sw  pin

int16_t position = 0;

RotaryEncoder encoder(PIN_A, PIN_B, BUTTON);


void setup()
{
  encoder.begin();

  Serial.begin(115200);
}


void loop()
{
  encoder.updateA();
  encoder.updateB();
  if (position != encoder.readPosition())
  {
    position = encoder.readPosition();
    Serial.println(position);
  }

  encoder.updatePB();
  if (encoder.readPushButton() == LOW)
  {
    Serial.println("PRESSED");
  }
}
