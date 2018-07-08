/***************************************************************************************************/
/*
  This is an Arduino sketch for RotaryEncoder library using interrupts

  written by : enjoyneering79
  sourse code: https://github.com/enjoyneering/

  This sketch uses interrupts, specials pins are required to interface
  Board:                                    int.0  int.1  int.2  int.3  int.4  int.5
  Uno, Mini, Pro, ATmega168, ATmega328..... 2      3      x       x      x     x
  Mega2560................................. 2      3      21      20     19    18
  Leonardo, Micro, ATmega32U4.............. 3      2      0       1      7     x
  Digistump, Trinket, ATtiny85............. 2/physical pin no.7
  Due...................................... all digital pins
  Zero..................................... all digital pins, except pin 4
  Blue Pill, STM32F103xxxx boards.......... all digital pins, maximun 16 pins at the same time
  ESP8266.................................. all digital pins, except gpio6 - gpio11 & gpio16

  PCF8574 chip uses I2C bus to communicate, specials pins are required to interface
  Board:                                    SDA                    SCL
  Uno, Mini, Pro, ATmega168, ATmega328..... A4                     A5
  Mega2560, Due............................ 20                     21
  Leonardo, Micro, ATmega32U4.............. 2                      3
  Digistump, Trinket, ATtiny85............. 0/physical pin no.5    2/physical pin no.7
  Blue Pill, STM32F103xxxx boards.......... PB7*                   PB6*
  ESP8266 ESP-01:.......................... GPIO0/D5               GPIO2/D3
  NodeMCU 1.0, WeMos D1 Mini............... GPIO4/D2               GPIO5/D1

                                           *STM32F103xxxx pins PB6/PB7 are 5v tolerant, but
                                            bi-directional logic level converter is recommended

  Frameworks & libraries:
  TimerOne AVR - https://github.com/PaulStoffregen/TimerOne
  ATtiny  Core - https://github.com/SpenceKonde/ATTinyCore 
  ESP8266 Core - https://github.com/esp8266/Arduino
  STM32   Core - https://github.com/rogerclarkmelbourne/Arduino_STM32

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
#include <TimerOne.h>      //https://github.com/PaulStoffregen/TimerOne
#include <RotaryEncoder.h>

#define PIN_A   5 //ky-040 clk pin, add 100nF/0.1uF capacitors between pin & ground
#define PIN_B   4 //ky-040 dt  pin, add 100nF/0.1uF capacitors between pin & ground
#define BUTTON  3 //ky-040 sw  pin, interrupt

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
  Timer1.initialize();                                                       //optionally timer's period can be set here in usec, default 1 sec. this breaks analogWrite() for pins 9 & 10

  encoder.begin();                                                           //set encoders pins as input & enable built-in pullup resistors

  Timer1.attachInterrupt(encoderISR, 10000);                                 //call encoderISR() every 10000 microseconds/0.01 seconds
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
  
  if (encoder.getPushButton() == true) Serial.println(F("PRESSED"));         //(F()) saves string to flash & keeps dynamic memory free
}
