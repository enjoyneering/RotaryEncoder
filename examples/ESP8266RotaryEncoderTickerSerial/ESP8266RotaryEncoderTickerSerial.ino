/***************************************************************************************************/
/*
   This is an Arduino sketch for RotaryEncoder library using build-in Arduino ESP8266 Ticker

   written by : enjoyneering79
   sourse code: https://github.com/enjoyneering/

   Frameworks & Libraries:
   TimerOne AVR          - https://github.com/PaulStoffregen/TimerOne
   ATtiny  Core          - https://github.com/SpenceKonde/ATTinyCore
   ESP32   Core          - https://github.com/espressif/arduino-esp32
   ESP8266 Core          - https://github.com/esp8266/Arduino
   STM32   Core          - https://github.com/rogerclarkmelbourne/Arduino_STM32

   GNU GPL license, all text above must be included in any redistribution,
   see link for details  - https://www.gnu.org/licenses/licenses.html
*/
/***************************************************************************************************/
#include <ESP8266WiFi.h>
#include <RotaryEncoder.h>
#include <Ticker.h>

#define PIN_A   D5 //ky-040 clk pin, add 100nF/0.1uF capacitors between pin & ground!!!
#define PIN_B   D6 //ky-040 dt  pin, add 100nF/0.1uF capacitors between pin & ground!!!
#define BUTTON  D7 //ky-040 sw  pin, add 100nF/0.1uF capacitors between pin & ground!!!

int16_t position = 0;

RotaryEncoder encoder(PIN_A, PIN_B, BUTTON);

Ticker encoderRotary;
Ticker encoderButton;


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
  WiFi.persistent(false);                                            //disable saving wifi config into SDK flash area
  WiFi.forceSleepBegin();                                            //disable AP & station by calling "WiFi.mode(WIFI_OFF)" & put modem to sleep

  encoder.begin();                                                   //set encoders pins as input & enable built-in pullup resistors

  encoderRotary.attach_ms(10, encoderISR);                           //call encoderISR()       every 10 milliseconds/0.010 seconds
  encoderButton.attach_ms(15, encoderButtonISR);                     //call encoderButtonISR() every 15 milliseconds/0.015 seconds

  Serial.begin(115200);
}

void loop()
{
  if (position != encoder.getPosition())
  {
    position = encoder.getPosition();
    Serial.println(position);
  }
  
  if (encoder.getPushButton() == true) Serial.println(F("PRESSED")); //(F()) save string to flash & keep dynamic memory free
}
