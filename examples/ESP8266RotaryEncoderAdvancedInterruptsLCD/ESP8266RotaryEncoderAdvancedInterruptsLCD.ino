/***************************************************************************************************/
/*
   This is an Arduino sketch for RotaryEncoder library using interrupts 

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
#pragma GCC optimize ("Os")          //code optimization controls, "O2" or "O3" code performance & "Os" code size

#include <Wire.h>                    //use bug free i2c driver https://github.com/enjoyneering/ESP8266-I2C-Driver
#include <ESP8266WiFi.h>
#include <LiquidCrystal_I2C.h>       //https://github.com/enjoyneering/LiquidCrystal_I2C
#include <RotaryEncoderAdvanced.h>


#define LCD_ROWS         4           //quantity of lcd rows
#define LCD_COLUMNS      20          //quantity of lcd columns

#define LCD_SPACE_SYMBOL 0x20        //space symbol from lcd ROM, see p.9 of GDM2004D datasheet

#define PIN_A            D5          //ky-040 clk pin, interrupt & add 100nF/0.1uF capacitors between pin & ground!!!
#define PIN_B            D6          //ky-040 dt  pin,             add 100nF/0.1uF capacitors between pin & ground!!!
#define BUTTON           D7          //ky-040 sw  pin, interrupt & add 100nF/0.1uF capacitors between pin & ground!!!

uint16_t buttonCounter = 0;

RotaryEncoderAdvanced<float> encoder(PIN_A, PIN_B, BUTTON, 0.1, 0.0, 3.3);                         //0.1 step per click, minimum value 0, maximum value 3.3
LiquidCrystal_I2C            lcd(PCF8574_ADDR_A21_A11_A01, 4, 5, 6, 16, 11, 12, 13, 14, POSITIVE);


void ICACHE_RAM_ATTR encoderISR()                                            //interrupt service routines need to be in ram
{
  encoder.readAB();
}

void ICACHE_RAM_ATTR encoderButtonISR()
{
  encoder.readPushButton();
}

void setup()
{
  WiFi.persistent(false);                                                    //disable saving wifi config into SDK flash area
  WiFi.forceSleepBegin();                                                    //disable AP & station by calling "WiFi.mode(WIFI_OFF)" & put modem to sleep
  
  encoder.begin();                                                           //set encoders pins as input & enable built-in pullup resistors

  attachInterrupt(digitalPinToInterrupt(PIN_A),  encoderISR,       CHANGE);  //call encoderISR()    every high->low or low->high changes
  attachInterrupt(digitalPinToInterrupt(BUTTON), encoderButtonISR, FALLING); //call pushButtonISR() every high->low              changes

  Serial.begin(115200);

  /* LCD connection check */ 
  while (lcd.begin(LCD_COLUMNS, LCD_ROWS, LCD_5x8DOTS, D2, D1) != true)      //colums - 20, rows - 4, pixels - LCD_5x8DOTS, SDA - D2, SCL - D1
  {
    Serial.println(F("PCF8574 is not connected or lcd pins declaration is wrong. Only pins numbers: 4,5,6,16,11,12,13,14 are legal."));
    delay(5000);
  }

  lcd.print(F("PCF8574 is OK"));                                             //(F()) saves string to flash & keeps dynamic memory free
  delay(1500);

  lcd.clear();

  /* prints static text */
  lcd.print(F("VALUE :"));
  lcd.setCursor(0, 1);                                                       //set 1-st column, 2-nd row
  lcd.print(F("BUTTON:"));
  lcd.setCursor(8, 1);
  lcd.print(buttonCounter);
  lcd.setCursor(0, 2);
  lcd.print(F("UPTIME:"));
}

void loop()
{
  lcd.setCursor(8, 0);
  lcd.print(encoder.getValue(), 1);
  lcd.write(LCD_SPACE_SYMBOL);

  if (encoder.getPushButton() == true)
  {
    lcd.setCursor(8, 1);
    lcd.print(buttonCounter++);

    if   (buttonCounter % 4 == 0) lcd.noBacklight();                         //every 4-th button click backlight is off
    else                          lcd.backlight();
  }

  lcd.setCursor(8, 2);
  lcd.print((millis() / 1000));
}
