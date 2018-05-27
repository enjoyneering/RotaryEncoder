/***************************************************************************************************/
/*
  This is an Arduino sketch for RotaryEncoder library using interrupts

  written by : enjoyneering79
  sourse code: https://github.com/enjoyneering/

  This sketch uses interrupts, specials pins are required to interface
  Board:                                    int.0  int.1  int.2  int.3  int.4  int.5
  Uno, Mini, Pro (ATmega168 & ATmega328)... 2      3      x       x      x     x
  Mega2560................................. 2      3      21      20     19    18
  Leonardo, Micro (ATmega32U4)............. 3      2      0       1      7     x
  Due...................................... all digital pins
  Zero..................................... all digital pins, except pin 4
  Arduino STM32/STM32F103xxxx boards....... all digital pins, maximun 16 pins at the same time
  ESP8266.................................. all digital pins, except gpio6 - gpio11 & gpio16

  Frameworks & libraries:
  ATtiny Core           - https://github.com/SpenceKonde/ATTinyCore 
  ATtiny I2C Master lib - https://github.com/SpenceKonde/TinyWireM
  ESP8266 Core          - https://github.com/esp8266/Arduino
  Arduino STM32 Core    - https://github.com/rogerclarkmelbourne/Arduino_STM32

  NOTE:
  - LOW     interrupt trigges whenever the pin is low
  - HIGH    interrupt triggers whenever the pin is high (Arduino Due, Zero, MKR1000 only)
  - CHANGE  interrupt triggers whenever the pin changes value
  - RISING  interrupt triggers when the pin goes from low to high
  - FALLING interrupt triggers when the pin goes from high to low

  GNU GPL license, all text above must be included in any redistribution, see link below for details:
  - https://www.gnu.org/licenses/licenses.html
*/
/***************************************************************************************************/
#include <RotaryEncoder.h>
#include <ESP8266WiFi.h>

#define PIN_A      D5 //ky-040 clk pin, interrupt, add 100nF/0.1uF capacitors between pin & ground
#define PIN_B      D6 //ky-040 dt  pin,            add 100nF/0.1uF capacitors between pin & ground
#define BUTTON     D7 //ky-040 sw  pin, interrupt

int16_t position = 0;

RotaryEncoder encoder(PIN_A, PIN_B, BUTTON);


void encoderISR()
{
  encoder.readAB();
}

void encoderButtonISR()
{
  encoder.readPushButton();
}

void setup()
{
  WiFi.persistent(false);                                                    //disable saving wifi config into SDK flash area
  WiFi.forceSleepBegin();                                                    //disable AP & station by calling "WiFi.mode(WIFI_OFF)" & put modem to sleep

  encoder.begin();                                                           //set encoders pins as input & enable built-in pullup resistors

  attachInterrupt(digitalPinToInterrupt(PIN_A),  encoderISR,       CHANGE);  //call encoderISR()       when high->low or high->low changes happened
  attachInterrupt(digitalPinToInterrupt(BUTTON), encoderButtonISR, FALLING); //call encoderButtonISR() when high->low changes happened

  Serial.begin(115200);
}

void loop()
{
  if (position != encoder.getPosition())
  {
    position = encoder.getPosition();
    Serial.println(position);
  }
  
  if (encoder.getPushButton() == true) Serial.println("PRESSED");
}
