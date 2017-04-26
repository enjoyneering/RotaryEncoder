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

  PCF8574 chip uses I2C bus to communicate, specials pins are required to interface

  Connect PCF8574 to pins :  SDA     SCL
  Uno, Mini, Pro:            A4      A5
  Mega2560, Due:             20      21
  Leonardo:                  2       3
  ATtiny85:                  0(5)    2/A1(7) (ATTinyCore  - https://github.com/SpenceKonde/ATTinyCore
                                              & TinyWireM - https://github.com/SpenceKonde/TinyWireM)
  ESP8266 ESP-xx:            ANY     ANY     (ESP8266Core - https://github.com/esp8266/Arduino)
  NodeMCU 1.0:               ANY     ANY     (D2 & D1 by default)

  NOTE: - LOW     interrupt trigges whenever the pin is low
        - HIGH    interrupt triggers whenever the pin is high (Arduino Due, Zero, MKR1000 only)
        - CHANGE  interrupt triggers whenever the pin changes value
        - RISING  interrupt triggers when the pin goes from low to high
        - FALLING interrupt triggers when the pin goes from high to low

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
#define PIN_B            4     //ky-040 dt  pin
#define BUTTON           3     //ky-040 sw  pin

uint16_t buttonCounter = 0;

RotaryEncoder encoder(PIN_A, PIN_B, BUTTON);

LiquidCrystal_I2C lcd(PCF8574_ADDR_A21_A11_A01, 4, 5, 6, 16, 11, 12, 13, 14, POSITIVE);


void encoderISR()
{
  encoder.readAB();
}

void pushButtonISR()
{
  encoder.readPushButton();
}

void setup()
{
  encoder.begin();

  attachInterrupt(digitalPinToInterrupt(PIN_A),  encoderISR,    FALLING); //call encoderISR()    when high->low changes happened
  attachInterrupt(digitalPinToInterrupt(BUTTON), pushButtonISR, FALLING); //call pushButtonISR() when high->low changes happened

  /* LCD connection check */ 
  while (lcd.begin(LCD_COLUMNS, LCD_ROWS) != true)                        //20x4 display
  {
    for (uint8_t i = 0; i > 3; i++)                                       //3 blinks if PCF8574/LCD is not connected or wrong pins declaration
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
  lcd.setCursor(10, 0);
    lcd.print(encoder.getPosition());
    lcd.write(LCD_SPACE_SYMBOL);

  encoder.readPushButton();
  if (encoder.getPushButton() == LOW)
  {
    lcd.setCursor(10, 1);
      lcd.print(buttonCounter++);
  }
}
