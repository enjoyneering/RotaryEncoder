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
  Blue Pill, STM32F103xxxx boards.......... B7*                    B6*
  ESP8266 ESP-01:.......................... GPIO0/D5               GPIO2/D3
  NodeMCU 1.0, WeMos D1 Mini............... GPIO4/D2               GPIO5/D1

                                            *STM32F103xxxx pins B7/B7 are 5v tolerant, but bi-directional
                                             logic level converter is recommended

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
#pragma GCC optimize ("Os")    //code optimisation controls - "O2" & "O3" code performance, "Os" code size

#include <Wire.h>
#include <TimerOne.h>          //https://github.com/PaulStoffregen/TimerOne
#include <LiquidCrystal_I2C.h> //https://github.com/enjoyneering/LiquidCrystal_I2C
#include <RotaryEncoder.h>

#define LCD_ROWS         4     //qnt. of lcd rows
#define LCD_COLUMNS      20    //qnt. of lcd columns

#define LCD_SPACE_SYMBOL 0x20  //space symbol from the LCD ROM, see p.9 of GDM2004D datasheet

#define PIN_A            2     //ky-040 clk pin, interrupt, add 100nF/0.1uF capacitors between pin & ground
#define PIN_B            4     //ky-040 dt  pin,            add 100nF/0.1uF capacitors between pin & ground
#define BUTTON           3     //ky-040 sw  pin, interrupt

uint16_t buttonCounter = 0;

RotaryEncoder     encoder(PIN_A, PIN_B, BUTTON);
LiquidCrystal_I2C lcd(PCF8574_ADDR_A21_A11_A01, 4, 5, 6, 16, 11, 12, 13, 14, POSITIVE);


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
  attachInterrupt(digitalPinToInterrupt(BUTTON), encoderButtonISR, FALLING); //call pushButtonISR() when high->low changes happened

  /* LCD connection check */ 
  while (lcd.begin(LCD_COLUMNS, LCD_ROWS) != true)                           //20x4 display
  {
    for (uint8_t i = 0; i < 5; i++)                                          //5 blinks if PCF8574/LCD is not connected or wrong pins declaration
    {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(500);
      digitalWrite(LED_BUILTIN, LOW);
      delay(500);
    }
  }

  lcd.print(F("PCF8574 is OK"));                                             //(F()) saves string to flash & keeps dynamic memory free
  delay(1500);

  lcd.clear();

  /* prints static text */
  lcd.print(F("POSITION:"));
  lcd.setCursor(0, 1);
  lcd.print(F("BUTTON  :"));
  lcd.setCursor(10, 1);
  lcd.print(buttonCounter);
  lcd.setCursor(0, 2);
  lcd.print(F("UPTIME  :"));
}

void loop()
{
  lcd.setCursor(10, 0);
  lcd.print(encoder.getPosition());
  lcd.write(LCD_SPACE_SYMBOL);

  if (encoder.getPushButton() == true)
  {
    lcd.setCursor(10, 1);
    lcd.print(buttonCounter++);

    if   (buttonCounter % 4 == 0) lcd.noBacklight();                         //every 4-th button click the backlight is off
    else                          lcd.backlight();
  }

  lcd.setCursor(10, 2);
  lcd.print((millis() / 1000));
}
