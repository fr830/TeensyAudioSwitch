/*
  The circuit:

 | LED| SPI|SPI |REL1|REL2| I2C| I2C | REL3| RELA|PWM | LCD|
 | 13 | 14 | 15 | 16 | 17 | 18 | 19  | 20  | 21  | 22 | 23 |
 -----------------------------------------------------------
 | I2C| I2C| LCD| LCD| LCD |
 | 29 | 30 | 31 | 32 | 33  |
 --------------------------------
 | 28 | 27 | 26 | 25 | 24 | GND | A12 | A11 | A10 |
 | LCD| LCD| LCD| LCD| LCD| LCD | AMBL| AUDR| AUDL|
 --------------------------------------------
 | 0  |  1 |  2 |  3 |  4 |  5  |  6  |  7  |  8  |  9 | 10 | 11 | 12 |
 |UART|UART|ENCB|ENC |ENC | IR  | IR  | ENCB| ENC | ENC| SPI| SPI| SPI|



LCD
---
  
 * LCD Enable1 31
 * LCD Enable2 32
 * LCD D4 pin 24
 * LCD D5 pin 25
 * LCD D6 pin 26
 * LCD D7 pin 27
 * LCD RS pin 28
 * LCD RW pin 33
 * LCD PWM BackLED 23

UART
----

 * RX pin 0
 * TX pin 1

SPI
---

 * CS1  pin 10
 * MOSI pin 11
 * MISO pin 12
 * SCK  pin 14
 * CS2  pin 15
 

I2C
---

 * SCL0 pin 19
 * SDA0 pin 18
 * SCL1 pin 29
 * SDA1 pin 30

Audio
-----

 * L pin A10
 * R pin A11

Ambient Lux
-----------

 * signal pin A12

IRemote
-------

 * RX pin 6
 * TX pin 5

ENC
---

  * ENC A pin 7, 8, 9
  * ENC b pin 2, 3, 4

RELAY
-----

 * A/B pin 16
 * Phono pin 17
 * Tuner pin 20
 * Tape pin 21

Servo
-----

 * signal pin 22

 */





// include the library code:
#include <LiquidCrystalFast.h>
#include <Encoder.h>
#include <Bounce.h>
#include <Audio.h>
//#include <Wire.h>
//#include <SPI.h>
//#include <SD.h>
//#include <SerialFlash.h>

AudioInputAnalog         AINPUT_L(A10);
AudioInputAnalog         AINPUT_R(A11);
//AudioAnalyzeRMS          RMS_R; 
AudioMixer4              MIXER;
AudioAnalyzePeak         PEAK_L;
//AudioAnalyzeRMS          RMS_L;
AudioAnalyzePeak         PEAK_R;
AudioAnalyzePeak         PEAK_SUM;
AudioAnalyzeFFT256       FFT256;
//AudioAnalyzeRMS          RMS_SUM;
AudioConnection          patchCord1(AINPUT_L, PEAK_L);
AudioConnection          patchCord2(AINPUT_L, 0, MIXER, 0);
AudioConnection          patchCord3(AINPUT_R, PEAK_R);
AudioConnection          patchCord4(AINPUT_R, 0, MIXER, 1);
AudioConnection          patchCord5(MIXER, FFT256);
AudioConnection          patchCord6(MIXER, PEAK_SUM);

//AudioConnection          patchCord7(AINPUT_L, RMS_L);
//AudioConnection          patchCord8(AINPUT_R, RMS_R);
//AudioConnection          patchCord9(MIXER, RMS_SUM);
// 0 UART
// 1 UART
const int PIN_BTN_B   = 2; // OK
const int PIN_ENCB_1  = 3; // OK
const int PIN_ENCB_2  = 4; // OK
const int PIN_IRDA_TX = 5; 
const int PIN_IRDA_RX = 6; 
const int PIN_BTN_A   = 7; // OK
const int PIN_ENCA_1  = 8; // OK
const int PIN_ENCA_2  = 9; // OK
// 10 SPI 
// 11 SPI
// 12 SPI
//const int PIN_LED  = 13;
// 14 SPI
// 15 SPI
const int PIN_REL_AB   = 16;
const int PIN_REL_PHON = 17;
// 18 I2C
// 19 I2C
const int PIN_REL_TUNE = 20;
const int PIN_REL_TAPE = 21;
const int PIN_SERVO    = 22;
const int PIN_LCD_BLED = 23;
const int PIN_LCD_D4   = 24; // OK
const int PIN_LCD_D5   = 25; // OK
const int PIN_LCD_D6   = 26; // OK
const int PIN_LCD_D7   = 27; // OK
const int PIN_LCD_RS   = 28; // OK
// 29 I2C
// 30 I2C
const int PIN_LCD_E1   = 31; // OK
const int PIN_LCD_E2   = 32; // OK
const int PIN_LCD_RW   = 33; // OK


// initialize the library with the numbers of the interface pins
LiquidCrystalFast lcd(PIN_LCD_RS, PIN_LCD_RW, PIN_LCD_E1, PIN_LCD_E2,  PIN_LCD_D4, PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7);


Encoder ENC_A(PIN_ENCA_1, PIN_ENCA_2);
long POS_ENC_A = -999;
Bounce BTN_ENC_A = Bounce( PIN_BTN_A,5 );

Encoder ENC_B(PIN_ENCB_1, PIN_ENCB_2);
long POS_ENC_B = -999;
Bounce BTN_ENC_B = Bounce( PIN_BTN_B,5 );


byte BAR_1[8] = { 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b11111};
byte BAR_2[8] = { 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b11111, 0b11111};
byte BAR_3[8] = { 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b11111, 0b11111, 0b11111};
byte BAR_4[8] = { 0b00000, 0b00000, 0b00000, 0b00000, 0b11111, 0b11111, 0b11111, 0b11111};
byte BAR_5[8] = { 0b00000, 0b00000, 0b00000, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111};
byte BAR_6[8] = { 0b00000, 0b00000, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111};
byte BAR_7[8] = { 0b00000, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111};
byte BAR_8[8] = { 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111};

void setup() {
  AudioMemory(10);
  MIXER.gain(0, 0.5);
  MIXER.gain(1, 0.5);

  // create bars
  lcd.createChar(0, BAR_1);
  lcd.createChar(1, BAR_2);
  lcd.createChar(2, BAR_3);
  lcd.createChar(3, BAR_4);
  lcd.createChar(4, BAR_5);
  lcd.createChar(5, BAR_6);
  lcd.createChar(6, BAR_7);
  lcd.createChar(7, BAR_8);
  
  // set up the LCD's number of rows and columns
  lcd.begin(40, 4);
  pinMode(PIN_BTN_A,INPUT_PULLUP); // button of ENC A
  pinMode(PIN_BTN_B,INPUT_PULLUP); // button of ENC B
}

void loop() {
  long newLeft, newRight;
  // read all inputs
  newLeft  = ENC_A.read();
  newRight = ENC_B.read();
  BTN_ENC_A.update();
  BTN_ENC_B.update();
  
  // set the cursor to column 0, line 1
  // (note: line 1 is the second row, since counting begins with 0):
  lcd.setCursor(0, 0);
  // print the number of seconds since reset:
  lcd.print(millis()/10);
  lcd.setCursor(0, 1);
  lcd.print(POS_ENC_A);
  lcd.setCursor(0, 2);
  lcd.print(POS_ENC_B);
  //Serial.print(" cpu:");
  //Serial.println(AudioProcessorUsageMax());
}

