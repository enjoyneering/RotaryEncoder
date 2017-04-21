/***************************************************************************************************/
/*
  This is an Arduino sketch for RotaryEncoder library

  written by : enjoyneering79
  sourse code: https://github.com/enjoyneering/

  PCF8574 chip uses I2C bus to communicate, specials pins are required to interface

  Connect PCF8574 to pins :  SDA     SCL
  Uno, Mini, Pro:            A4      A5
  Mega2560, Due:             20      21
  Leonardo:                  2       3
  ATtiny85:                  0(5)    2/A1(7) (ATTinyCore  - https://github.com/SpenceKonde/ATTinyCore
                                              & TinyWireM - https://github.com/SpenceKonde/TinyWireM)
  ESP8266 ESP-xx:            ANY     ANY     (ESP8266Core - https://github.com/esp8266/Arduino)
  NodeMCU 1.0:               ANY     ANY     (D2 & D1 by default)

  BSD license, all text above must be included in any redistribution
*/
/***************************************************************************************************/
#include <LiquidCrystal_I2C.h> //https://github.com/enjoyneering/LiquidCrystal_I2C
#include <Wire.h>
#include <RotaryEncoder.h>

#define LCD_ROWS         4     //qnt. of lcd rows
#define LCD_COLUMNS      20    //qnt. of lcd columns

#define LCD_SPACE_SYMBOL 0x20  //space  symbol from the LCD ROM, see p.9 of GDM2004D datasheet

#define PIN_A            2     //ky-040 clk pin
#define PIN_B            3     //ky-040 dt  pin
#define BUTTON           4     //ky-040 sw  pin

uint16_t buttonCounter = 0;

RotaryEncoder encoder(PIN_A, PIN_B, BUTTON);

LiquidCrystal_I2C lcd(PCF8574_ADDR_A21_A11_A01, 4, 5, 6, 16, 11, 12, 13, 14, POSITIVE);


void setup()
{
  encoder.begin();

  /* LCD connection check */ 
  while (lcd.begin(LCD_COLUMNS, LCD_ROWS) != true)                    //20x4 display
  {
    for (uint8_t i = 0; i > 3; i++)                                   //3 blinks if PCF8574/LCD is not connected or wrong pins declaration
    {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(500);
      digitalWrite(LED_BUILTIN, LOW);
      delay(500);
    }
    delay(5000);
  }
  lcd.print("PCF8574 is OK");
  delay(1500);
  lcd.clear();

  /* prints static text */
  lcd.print("POSITION:");
  lcd.setCursor(0, 1);
    lcd.print("BUTTON  :"); 
}


void loop()
{
  encoder.updateA();
  encoder.updateB();
  lcd.setCursor(10, 0);
    lcd.print(encoder.readPosition());
    lcd.write(LCD_SPACE_SYMBOL);

  encoder.updatePB();
  if (encoder.readPushButton() == LOW)
  {
    lcd.setCursor(10, 1);
      lcd.print(buttonCounter++);
  }
}
