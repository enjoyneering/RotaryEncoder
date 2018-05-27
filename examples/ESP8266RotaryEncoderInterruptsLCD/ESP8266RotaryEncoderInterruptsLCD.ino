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

  PCF8574 chip uses I2C bus to communicate, specials pins are required to interface
  Board:                                    SDA                    SCL
  Uno, Mini, Pro........................... A4                     A5
  Mega2560, Due............................ 20                     21
  Leonardo, Micro, ATmega32U4.............. 2                      3
  Digistump, Trinket, ATtiny85............. 0/physical pin no.5    2/physical pin no.7
  Blue Pill, STM32F103xxxx boards.......... B7                     B6 with 5v->3v logich converter
  ESP8266 ESP-01:.......................... GPIO0/D5               GPIO2/D3
  NodeMCU 1.0, WeMos D1 Mini............... GPIO4/D2               GPIO5/D1

  Frameworks & libraries:
  ATtiny Core           - https://github.com/SpenceKonde/ATTinyCore 
  ATtiny I2C Master lib - https://github.com/SpenceKonde/TinyWireM
  ESP8266 Core          - https://github.com/esp8266/Arduino
  Arduino STM32 Core    - https://github.com/rogerclarkmelbourne/Arduino_STM32

  NOTE:
  - for best result add 100nF/0.1uF capacitors between each channel pin & ground
  - LOW     interrupt trigges whenever the pin is low
  - HIGH    interrupt triggers whenever the pin is high (Arduino Due, Zero, MKR1000 only)
  - CHANGE  interrupt triggers whenever the pin changes value
  - RISING  interrupt triggers when the pin goes from low to high
  - FALLING interrupt triggers when the pin goes from high to low

  GNU GPL license, all text above must be included in any redistribution, see link below for details:
  - https://www.gnu.org/licenses/licenses.html
*/
/***************************************************************************************************/
#include <LiquidCrystal_I2C.h> //https://github.com/enjoyneering/LiquidCrystal_I2C
#include <Wire.h>              //https://github.com/enjoyneering/ESP8266-I2C-Driver
#include <ESP8266WiFi.h>
#include <RotaryEncoder.h>

#define LCD_ROWS         4     //qnt. of lcd rows
#define LCD_COLUMNS      20    //qnt. of lcd columns

#define LCD_SPACE_SYMBOL 0x20  //space symbol from the LCD ROM, see p.9 of GDM2004D datasheet

#define PIN_A            D5    //ky-040 clk pin, interrupt, add 100nF/0.1uF capacitors between pin & ground
#define PIN_B            D6    //ky-040 dt  pin,            add 100nF/0.1uF capacitors between pin & ground
#define BUTTON           D7    //ky-040 sw  pin, interrupt

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
  WiFi.persistent(false);                                                    //disable saving wifi config into SDK flash area
  WiFi.forceSleepBegin();                                                    //disable AP & station by calling "WiFi.mode(WIFI_OFF)" & put modem to sleep
  
  encoder.begin();                                                           //set encoders pins as input & enable built-in pullup resistors

  attachInterrupt(digitalPinToInterrupt(PIN_A),  encoderISR,       CHANGE);  //call encoderISR()    when high->low or high->low changes happened
  attachInterrupt(digitalPinToInterrupt(BUTTON), encoderButtonISR, FALLING); //call pushButtonISR() when high->low changes happened

  Serial.begin(115200);

  /* LCD connection check */ 
  while (lcd.begin(LCD_COLUMNS, LCD_ROWS, LCD_5x8DOTS, D2, D1) != true)      //colums - 20, rows - 4, pixels - LCD_5x8DOTS, SDA - D2, SCL - D1
  {
    Serial.println("PCF8574 is not connected or lcd pins declaration is wrong. Only pins numbers: 4,5,6,16,11,12,13,14 are legal.");
    delay(5000);
  }

  lcd.print("PCF8574 is OK");
  delay(1500);

  lcd.clear();

  /* prints static text */
  lcd.print("POSITION:");
  lcd.setCursor(0, 1);
  lcd.print("BUTTON  :");
  lcd.setCursor(10, 1);
  lcd.print(buttonCounter);
  lcd.setCursor(0, 2);
  lcd.print("UPTIME  :");
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

    if   (buttonCounter % 4 == 0) lcd.noBacklight();
    else                          lcd.backlight();
  }

  lcd.setCursor(10, 2);
  lcd.print((millis() / 1000));
}
