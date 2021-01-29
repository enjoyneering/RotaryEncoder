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
#pragma GCC optimize ("Os")           //code optimization controls, "O2" or "O3" code performance & "Os" code size

#include <Wire.h>
#include <LiquidCrystal_I2C.h>        //https://github.com/enjoyneering/LiquidCrystal_I2C
#include <RotaryEncoderAdvanced.h>


#define LCD_ROWS          4           //quantity of lcd rows
#define LCD_COLUMNS       20          //quantity of lcd columns

#define LCD_SPACE_SYMBOL  0x20        //space symbol from lcd ROM, see p.9 of GDM2004D datasheet

#define PIN_A             2           //ky-040 clk pin, interrupt & add 100nF/0.1uF capacitors between pin & ground!!!
#define PIN_B             4           //ky-040 dt  pin,             add 100nF/0.1uF capacitors between pin & ground!!!
#define BUTTON            3           //ky-040 sw  pin, interrupt & add 100nF/0.1uF capacitors between pin & ground!!!

#define V_STEPS_PER_CLICK 0.01        //0.01v per click
#define V_MIN_VALUE       0           //minimum value 0.00v
#define V_MAX_VALUE       15          //maximum value 15.00v

#define A_STEPS_PER_CLICK 0.1         //0.1A per click
#define A_MIN_VALUE       0           //minimum value 0.0A
#define A_MAX_VALUE       3           //maximum value 3.0A

uint8_t buttonCounter      = 0;
float   backupVoltageValue = 0;
float   backupCurrentValue = 0;


RotaryEncoderAdvanced<float> encoder(PIN_A, PIN_B, BUTTON, V_STEPS_PER_CLICK, V_MIN_VALUE, V_MAX_VALUE);
LiquidCrystal_I2C            lcd(PCF8574_ADDR_A21_A11_A01, 4, 5, 6, 16, 11, 12, 13, 14, POSITIVE);


/**************************************************************************/
/*
    setup()

    Main setup
*/
/**************************************************************************/
void setup()
{
  encoder.begin();                                                           //set encoders pins as input & enable built-in pullup resistors

  attachInterrupt(digitalPinToInterrupt(PIN_A),  encoderISR,       CHANGE);  //call encoderISR()    every high->low or low->high changes
  attachInterrupt(digitalPinToInterrupt(BUTTON), encoderButtonISR, FALLING); //call pushButtonISR() every high to low changes

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
  lcd.print(F("VOLTAGE:"));

  lcd.setCursor(0, 1);                                                       //set 1-st column, 2-nd row
  lcd.print(F("CURRENT:"));
}

/**************************************************************************/
/*
    loop()

    Main loop
*/
/**************************************************************************/
void loop()
{
  pushButtonHandler();
}

/**************************************************************************/
/*
    encoderISR()

    Encoder A & B pin interrupt service routine

    NOTE:
    - use interrupt pin for A pin!!!
    - add 100nF/0.1uF capacitors between A pin & ground!!!
    - add 100nF/0.1uF capacitors between B pin & ground!!!
*/
/**************************************************************************/
void encoderISR()
{
  encoder.readAB();
}

/**************************************************************************/
/*
    encoderButtonISR()

    Encoder button interrupt service routine

    NOTE:
    - use interrupt pin!!!
    - add 100nF/0.1uF capacitors between pin & ground!!!
*/
/**************************************************************************/
void encoderButtonISR()
{
  encoder.readPushButton();
}

/**************************************************************************/
/*
    pushButtonHandler()

    Push button handler

    NOTE:
    - press 1 time  to change VOLTAGE
    - press 2 times to change CURRENT
    - press 3 times to exit
*/
/**************************************************************************/
void pushButtonHandler()
{
  if (encoder.getPushButton() == true) buttonCounter++;

  switch (buttonCounter)
  {
    case 1:
      setPrintVoltage();
      break;

    case 2:
      setPrintCurrent();
      break;

    default:
      buttonCounter = 0;
      break;
  }
}

/**************************************************************************/
/*
    setPrintVoltage()

    Set & print VOLTAGE value
*/
/**************************************************************************/
void setPrintVoltage()
{
  encoder.setValues(backupVoltageValue, V_STEPS_PER_CLICK, V_MIN_VALUE, V_MAX_VALUE); //load "VOLTAGE" values to encoder

  while (encoder.getPushButton() != true)
  {
    lcd.setCursor(8, 0);                                                              //set 9-th column, 1-st row

    lcd.print(encoder.getValue());
    lcd.write(LCD_SPACE_SYMBOL);

    lcd.printHorizontalGraph('V', 2, encoder.getValue(), V_MAX_VALUE);                //name of the bar, 3-rd row, value, maximun value
  }

  backupVoltageValue = encoder.getValue();                                            //backup "VOLTAGE" value

  buttonCounter++;                                                                    //jump to next menu
}

/**************************************************************************/
/*
    setPrintCurrent()

    Set & print CURRENT value
*/
/**************************************************************************/
void setPrintCurrent()
{
  encoder.setValues(backupCurrentValue, A_STEPS_PER_CLICK, A_MIN_VALUE, A_MAX_VALUE); //load "CURRENT" values to encoder

  while (encoder.getPushButton() != true)
  {
    lcd.setCursor(8, 1);                                                              //set 9-th column, 2-nd row

    lcd.print(encoder.getValue());
    lcd.write(LCD_SPACE_SYMBOL);

    lcd.printHorizontalGraph('A', 3, encoder.getValue(), A_MAX_VALUE);                //name of the bar, 4-th row, value, maximun value
  }

  backupCurrentValue = encoder.getValue();                                            //backup "CURRENT" value

  buttonCounter++;                                                                    //jump to next menu
}
