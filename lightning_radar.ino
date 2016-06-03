//Lightning Radar by Arduino Mega & Playing with Fusion Modul AS3935
/***************************************************************************
  File Name: as3935_lightning_i2c_nocal.ino
  Processor/Platform: Arduino Uno R3 (tested)
  Development Environment: Arduino 1.6.1

  Designed for use with with Playing With Fusion AS3935 Lightning Sensor
  Breakout: SEN-39001-R01. Demo shows how this lightning sensor can be brought
  into an Arduino project without a bunch of calibration needed. This is
  because each board is tested calibrated prior to being shipped, and the
  cal value is written on the packaging.

    SEN-39001-R01 (universal applications)
    ---> http://www.playingwithfusion.com/productview.php?pdid=22

  Copyright © 2015 Playing With Fusion, Inc.
  SOFTWARE LICENSE AGREEMENT: This code is released under the MIT License.

  Permission is hereby granted, free of charge, to any person obtaining a
  copy of this software and associated documentation files (the "Software"),
  to deal in the Software without restriction, including without limitation
  the rights to use, copy, modify, merge, publish, distribute, sublicense,
  and/or sell copies of the Software, and to permit persons to whom the
  Software is furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
  DEALINGS IN THE SOFTWARE.
* **************************************************************************
  REVISION HISTORY:
  Author    Date    Comments
  J. Steinlage    2015Jul20       I2C release based on SPI example

  Playing With Fusion, Inc. invests time and resources developing open-source
  code. Please support Playing With Fusion and continued open-source
  development by buying products from Playing With Fusion!

* **************************************************************************
  APPLICATION SPECIFIC NOTES (READ THIS!!!):
  - This file configures then runs a program on an Arduino to interface with
    an AS3935 Franklin Lightning Sensor manufactured by AMS.
     - Configure Arduino
     - Perform setup for AS3935 chip
       --- capacitance registers for tuning (based on cal value provided)
       --- configurations for your application specifics (indoor/outdoor, etc)
     - read status/info from sensor
     - Write formatted information to serial port
  - Set configs for your specific needs using the #defines for wiring, and
    review the setup() function for other settings (indoor/outdoor, for example)
  - I2C specific note: This example uses the I2C interface via the I2C lib, not
    the 'Wire' lib included with the Arduino IDE.

  Circuit:
     Arduino Uno   Arduino Mega  -->  SEN-39001: AS3935 Breakout
     SDA:    SDA        SDA      -->  MOSI/SDA   (SDA is labeled on the bottom of the Arduino)
     SCLK:   SCL        SCL      -->  SCK/SCL    (SCL is labeled on the bottom of the Arduino)
     SI:     pin  9     pin 9    -->  SI (select interface; GND=SPI, VDD=I2C
     IRQ:    pin  2     pin 2    -->  IRQ
     GND:    GND        ''       -->  CS (pull CS to ground even though it's not used)
     GND:    GND        ''       -->  GND
     5V:     5V         ''       -->  Arduino I/O is at 5V, so power board from 5V. Can use 3.3V with Due, etc
**************************************************************************/
// This program is a demo of how to use most of the functions
// of the library with a supported display modules.
//
// This demo was made for modules with a screen resolution
// of 320x240 pixels.
//
// This program requires the UTFT library.
// web: http://www.henningkarlsen.com/electronics
#include <UTFT.h>

// This program requires both the UTFT and UTouch libraries
// in addition to the UTFT_Buttons add-on library.
#include <UTFT.h>
#include <ITDB02_Touch.h>
#include <UTFT_Buttons_ITDB.h>

// Declare Colors
#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF
#define ORANGE  0xFBE0
#define GRAY    0x7BEF

// Declare which fonts we will be using
extern uint8_t SmallFont[];
extern uint8_t BigFont[];
extern uint8_t SevenSegNumFont[];

// Set the pins to the correct ones for your development shield
// ------------------------------------------------------------
// Arduino Mega:
// Standard Arduino Mega/Due shield            : <display model>,38,39,40,41
// CTE TFT LCD/SD Shield for Arduino Mega      : <display model>,38,39,40,41
//
// Remember to change the model parameter to suit your display module!
UTFT tft(SSD1289, 38, 39, 40, 41);

// Set up UTouch...
// Set the pins to the correct ones for your development board
// -----------------------------------------------------------
// Standard Arduino 2009/Uno/Leonardo shield   : 15,10,14,9,8
// Standard Arduino Mega/Due shield            : 6,5,4,3,2
ITDB02_Touch        myTouch(6, 5, 4, 3, 2);

// Finally we set up UTFT_Buttons :)
UTFT_Buttons  myButtons(&tft, &myTouch);
int but1, but2, but3, but4 , but5, but6, pressed_button;
boolean menue_on = false;

//Pin
//#define LED A5// Pin of LED

#include <TimerOne.h>
//Lightning_Strikes
int copy_strikes_count = 0;
//activ,strenght,distance,age,x_pos,y_pos
//activ:0/1
//strenght:1-100
//distance=0-63km
//age=0-15min
//x:0-139 //position onn screen
//y:0-239 //position onn screen
uint16_t lightning_strike[50][6] = {
  {0, 0, 0, 0, 0, 0}, // Properties
  {0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0},
};
int lightning_timer = 0;
int time_factor = 1;
int last_xpos;
int last_ypos;
boolean noise = false;
boolean disturb = false;
boolean simulate_on = false;
boolean profile_indoor = false;
boolean tick = false;

//----------------------------------------------------------------
// The AS3935 communicates via SPI or I2C.
// This example uses the I2C interface via the I2C lib, not Wire lib
#include "I2C.h"
// include Playing With Fusion AXS3935 libraries
#include "PWFusion_AS3935_I2C.h"

// interrupt trigger global var
volatile int8_t AS3935_ISR_Trig = 0;

// defines for hardware config
#define SI_PIN               9
#define IRQ_PIN              2        // digital pins 2 and 3 are available for interrupt capability
#define AS3935_ADD           0x03     // x03 - standard PWF SEN-39001-R01 config
#define AS3935_CAPACITANCE   72       // <-- SET THIS VALUE TO THE NUMBER LISTED ON YOUR BOARD 

// defines for general chip settings
#define AS3935_INDOORS       1
#define AS3935_OUTDOORS      1
#define AS3935_DIST_DIS      0
#define AS3935_DIST_EN       1

// prototypes
void AS3935_ISR();

PWF_AS3935_I2C  lightning0((uint8_t)IRQ_PIN, (uint8_t)SI_PIN, (uint8_t)AS3935_ADD);
boolean calibrated = false;

//--------------------------------------------------------------
//--------------------------------------------------------------
//--------------------------------------------------------------
void setup()
{
  // Setup the LCD
  tft.InitLCD();//(oriantation:90)
  tft.clrScr();
  tft.setBackColor(BLACK);

  //pinMode(LED, OUTPUT);
  ScreenText(WHITE, 0, 10 , 2, "V0.3-Beta", 0);
  //------------------------------------------------------------------------------
  //Serial.begin(9600);
  //Serial.println("Playing With Fusion: AS3935 Lightning Sensor, SEN-39001-R01");
  //Serial.println("beginning boot procedure....");

  // setup for the the I2C library: (enable pullups, set speed to 400kHz)
  I2c.begin();
  I2c.pullup(true);
  I2c.setSpeed(1);
  delay(2);

  lightning0.AS3935_DefInit();   // set registers to default
  // now update sensor cal for your application and power up chip
  lightning0.AS3935_ManualCal(AS3935_CAPACITANCE, AS3935_OUTDOORS, AS3935_DIST_EN);
  //calibrated = true;
  // AS3935_ManualCal Parameters:
  //   --> capacitance, in pF (marked on package)
  //   --> indoors/outdoors (AS3935_INDOORS:1 / AS3935_OUTDOORS:1)
  //   --> disturbers (AS3935_DIST_EN:1 / AS3935_DIST_DIS:2)
  // function also powers up the chip

  // enable interrupt (hook IRQ pin to Arduino Uno/Mega interrupt input: 0 -> pin 2, 1 -> pin 3 )
  attachInterrupt(0, AS3935_ISR, RISING);
  lightning0.AS3935_PrintAllRegs();
  AS3935_ISR_Trig = 0;           // clear trigger
  //-------------------------------------------------------------------------------------------
  delay(2000);
  tft.clrScr();
  tft.setBackColor(BLACK);

  myTouch.InitTouch(1);
  myTouch.setPrecision(PREC_MEDIUM);

  myButtons.setTextFont(SmallFont);
  myButtons.setButtonColors(VGA_WHITE, VGA_WHITE, VGA_WHITE, VGA_BLACK, VGA_BLACK);
  but1 = myButtons.addButton( 280,  200, 30,  30, "");
  but2 = myButtons.addButton( 10,  50, 80,  30, "Indoor");
  myButtons.disableButton(but2, true);
  but3 = myButtons.addButton( 10,  90, 80,  30, "Outdoor");
  myButtons.disableButton(but3, true);
  but4 = myButtons.addButton( 10,  10, 80,  30, "Simulate");
  myButtons.disableButton(but4, true);
  but5 = myButtons.addButton( 10,  130, 80,  30, "Life x");
  myButtons.disableButton(but5, true);
  but6 = myButtons.addButton( 10,  170, 80,  30, "Calibrate");
  myButtons.disableButton(but6, true);

  tft.clrScr();
  tft.setBackColor(BLACK);
  myButtons.drawButton(but1);

  Timer1.initialize(1000000);
  Timer1.attachInterrupt(refresh_display);
}
//--------------------------------------------------------------
//--------------------------------------------------------------
//--------------------------------------------------------------
void loop() {

  // This program only handles an AS3935 lightning sensor. It does nothing until
  // an interrupt is detected on the IRQ pin.
  while (0 == AS3935_ISR_Trig) {}
  delay(5);

  // reset interrupt flag
  AS3935_ISR_Trig = 0;

  // now get interrupt source
  uint8_t int_src = lightning0.AS3935_GetInterruptSrc();
  if (0 == int_src)
  {
    //Serial.println("interrupt source result not expected");
    noise = false;
    disturb = false;
  }
  else if (1 == int_src)
  {
    uint8_t lightning_dist_km = lightning0.AS3935_GetLightningDistKm();
    //Serial.print("Lightning detected! Distance to strike: ");
    //Serial.print(lightning_dist_km);
    //Serial.println(" kilometers");
    if (lightning_dist_km < 64 && lightning_dist_km >= 0) {
      //activ:0/1
      //strenght:1-100
      //distance=0-63km
      //age=0-15min
      //x:0-139 //position onn screen
      //y:0-239 //position onn screen
      for (int i = 0; i < 50 ; i++) {
        if ( lightning_strike[i][0] == 0) {
          lightning_strike[i][0] = 1;
          lightning_strike[i][1] = 50;
          lightning_strike[i][2] = lightning_dist_km;
          lightning_strike[i][3] = lightning_timer;
          long randNumber;
          randNumber = random(160 - lightning_dist_km, 160 + lightning_dist_km);
          lightning_strike[i][4] = int(randNumber);
          lightning_strike[i][5] = int(230 - (lightning_dist_km * 3.0));
          SetCircle(BLACK , last_xpos,  last_ypos, 8);
          last_xpos = lightning_strike[i][4];
          last_ypos = lightning_strike[i][5];
          SetCircle(MAGENTA , last_xpos,  last_ypos, 8);
          break;
        }
      }
    }

  }
  else if (2 == int_src)
  {
    //Serial.println("Disturber detected");
    disturb = true;
  }
  else if (3 == int_src)
  {
    //Serial.println("Noise level too high");
    noise = true;
  }
  //lightning0.AS3935_PrintAllRegs(); // for debug...

  //------------------------------------------------
  //addButton
  //drawButtons
  //drawButton
  //enableButton
  //disableButton
  //relabelButton
  //buttonEnabled
  //deleteButton
  //deleteAllButtons
  //checkButtons
  //setTextFont
  //setSymbolFont
  //setButtonColors

  while (1)
  {
    if (myTouch.dataAvailable() == true)
    {
      pressed_button = myButtons.checkButtons();

      if (pressed_button == but1) {
        if (myButtons.buttonEnabled(but1)) {
          menue_on = true;
          myButtons.disableButton(but1, true);
          tft.clrScr();
          tft.setBackColor(BLACK);
          myButtons.drawButton(but2);
          myButtons.drawButton(but3);
          myButtons.drawButton(but4);
          myButtons.drawButton(but5);
          myButtons.drawButton(but6);
          myButtons.enableButton(but2, true);
          myButtons.enableButton(but3, true);
          myButtons.enableButton(but4, true);
          myButtons.enableButton(but5, true);
          myButtons.enableButton(but6, true);
        }
      }
      if (pressed_button == but2) {
        if (myButtons.buttonEnabled(but2)) {
          myButtons.disableButton(but2, true);
          myButtons.disableButton(but3, true);
          myButtons.disableButton(but4, true);
          myButtons.disableButton(but5, true);
          myButtons.disableButton(but6, true);
          tft.clrScr();
          tft.setBackColor(BLACK);
          myButtons.drawButton(but1);
          myButtons.enableButton(but1, true);
          lightning0.AS3935_ManualCal(AS3935_CAPACITANCE, AS3935_INDOORS, AS3935_DIST_EN);
          profile_indoor = true;
          menue_on = false;
        }
      }
      if (pressed_button == but3) {
        if (myButtons.buttonEnabled(but3)) {
          myButtons.disableButton(but2, true);
          myButtons.disableButton(but3, true);
          myButtons.disableButton(but4, true);
          myButtons.disableButton(but5, true);
          myButtons.disableButton(but6, true);
          tft.clrScr();
          tft.setBackColor(BLACK);
          myButtons.drawButton(but1);
          myButtons.enableButton(but1, true);
          lightning0.AS3935_ManualCal(AS3935_CAPACITANCE, AS3935_OUTDOORS, AS3935_DIST_EN);
          profile_indoor = false;
          menue_on = false;
        }
      }
      if (pressed_button == but4) {
        if (myButtons.buttonEnabled(but4)) {
          myButtons.disableButton(but2, true);
          myButtons.disableButton(but3, true);
          myButtons.disableButton(but4, true);
          myButtons.disableButton(but5, true);
          myButtons.disableButton(but6, true);
          tft.clrScr();
          tft.setBackColor(BLACK);
          myButtons.drawButton(but1);
          myButtons.enableButton(but1, true);
          if (simulate_on == false) {
            simulate_on = true;
            lightning_timer = 0;
          }
          else {
            simulate_on = false;
          }
          menue_on = false;
        }
      }
      if (pressed_button == but5) {
        if (myButtons.buttonEnabled(but5)) {
          myButtons.disableButton(but2, true);
          myButtons.disableButton(but3, true);
          myButtons.disableButton(but4, true);
          myButtons.disableButton(but5, true);
          myButtons.disableButton(but6, true);
          tft.clrScr();
          tft.setBackColor(BLACK);
          myButtons.drawButton(but1);
          myButtons.enableButton(but1, true);
          time_factor++;
          if (time_factor > 4) {
            time_factor = 1;
          }
          menue_on = false;
        }
      }
      if (pressed_button == but6) {
        if (myButtons.buttonEnabled(but6)) {
          myButtons.disableButton(but2, true);
          myButtons.disableButton(but3, true);
          myButtons.disableButton(but4, true);
          myButtons.disableButton(but5, true);
          myButtons.disableButton(but6, true);
          tft.clrScr();
          tft.setBackColor(BLACK);
          myButtons.drawButton(but1);
          myButtons.enableButton(but1, true);
          lightning0.AS3935_DefInit();   // set registers to default
          menue_on = false;
        }
      }
    }
  }
}
//----------------------------------------------
//--------------GRAFIK-ROUTINEN-----------------
//----------------------------------------------
unsigned long ScreenText(uint16_t color, int xtpos, int ytpos, int text_size , String text, int rotation) {
  tft.setColor(color);
  if (text_size == 1) {
    tft.setFont(SmallFont);
  }
  if (text_size == 2) {
    tft.setFont(BigFont);
  }
  tft.print(text, xtpos, ytpos, rotation);
}

unsigned long SetLines(uint16_t color , int xl1pos, int yl1pos, int xl2pos, int yl2pos) {
  tft.setColor(color);
  tft.drawLine(xl1pos, yl1pos, xl2pos, yl2pos);
}

unsigned long SetPoint(uint16_t color, int xppos, int yppos) {
  tft.setColor(color);
  tft.drawPixel(xppos, yppos);
}

unsigned long SetRect(uint16_t color , int xr1pos, int yr1pos, int xr2pos, int yr2pos) {
  tft.setColor(color);
  tft.drawRect(xr1pos, yr1pos, xr2pos, yr2pos);
}

unsigned long SetFilledRect(uint16_t color , int xr1pos, int yr1pos, int xr2pos, int yr2pos) {
  tft.setColor(color);
  tft.fillRect(xr1pos, yr1pos, xr2pos, yr2pos);
}

unsigned long SetCircle(uint16_t color , int xcpos, int ycpos, int radius) {
  tft.setColor(color);
  tft.drawCircle(xcpos, ycpos, radius);
}

unsigned long SetFilledCircle(uint16_t color , int xcpos, int ycpos, int radius) {
  tft.setColor(color);
  tft.fillCircle(xcpos, ycpos, radius);
}

unsigned long SetTriangle(uint16_t color , int xt1pos, int yt1pos, int xt2pos, int yt2pos , int xt3pos, int yt3pos) {
  tft.setColor(color);
  tft.drawLine(xt1pos,  yt1pos, xt2pos,  yt2pos);
  tft.drawLine(xt2pos,  yt2pos, xt3pos,  yt3pos);
  tft.drawLine(xt3pos,  yt3pos, xt1pos,  yt1pos);
}
//--------------------------------------------------------------
void simulate_strikes() {

  //----create lightning Strike:-----------------------
  ScreenText(WHITE, 120, 30 , 1, "Simulation", 0);

  if (lightning_timer > 700) {
    lightning_timer = 0;
  }
  if (lightning_timer < 10) {
    noise = true;
    disturb = true;
  }
  if (lightning_timer > 10) {
    noise = false;
    disturb = false;
  }
  if (lightning_timer < 250) {
    long randNumber2;
    randNumber2 = random(0, 63);
    for (int i = 0; i < 50 ; i++) {
      if ( lightning_strike[i][0] == 0) {
        lightning_strike[i][0] = 1;
        lightning_strike[i][1] = 50;
        lightning_strike[i][2] = int(randNumber2);//dist
        lightning_strike[i][3] = lightning_timer;
        long randNumber;
        randNumber = random(160 - lightning_strike[i][2], 160 + lightning_strike[i][2]);
        lightning_strike[i][4] = int(randNumber);
        lightning_strike[i][5] = int(230 - (lightning_strike[i][2] * 3.0));//235
        SetCircle(BLACK , last_xpos,  last_ypos, 8);
        last_xpos = lightning_strike[i][4];
        last_ypos = lightning_strike[i][5];
        SetCircle(MAGENTA , last_xpos,  last_ypos, 8);
        break;
        //activ,strenght,distance,age,x_pos,y_pos
        //activ:0/1
        //strenght:1-100
        //distance=0-63km
        //time_stamp=0-32767seconds
        //x:0-139 //position onn screen
        //y:0-239 //position onn screen
      }
    }
  }
}
//--------------------------------------------------------------
void refresh_display() {

  if (menue_on == false) {

    if (tick == true) {
      SetFilledCircle(WHITE , 10,  15, 2);
      tick = false;
    }
    else {
      SetFilledCircle(BLACK , 10,  15, 2);
      tick = true;
    }

    ScreenText(WHITE, 230, 160 , 1, "Life x " + String(time_factor), 0);

    if (profile_indoor == true) {
      ScreenText(WHITE, 230, 140 , 1, "Indoor", 0);
    }
    else {
      ScreenText(WHITE, 230, 140 , 1, "Outdoor", 0);
    }

    ScreenText(WHITE, 40, 59 , 1, "60km", 0);
    ScreenText(WHITE, 67, 119 , 1, "40km", 0);
    ScreenText(WHITE, 93, 179 , 1, "20km", 0);

    if (last_xpos > 0 && last_ypos > 0) {
      SetCircle(BLACK , last_xpos,  last_ypos, 8); //display flackert wenn koordinaten ausserhalb vom display
    }

    if (simulate_on == true) {
      simulate_strikes();
    }

    //--------increment counter---------------------------------
    lightning_timer++;
    if (lightning_timer < 0) {
      lightning_timer = 0;
    }
    //------Scale-----------------------------------------------
    int lightning_age;
    int strikes_count = 0;
    //-----------------------------------------------------------
    if (calibrated == false) {
      ScreenText(WHITE, 20, 180 , 1, "Cal !", 0);
      SetFilledCircle(RED , 10,  185, 2);
    }
    else {
      ScreenText(BLACK, 20, 180 , 1, "Cal !", 0);
      SetFilledCircle(BLACK , 10,  185, 2);
    }

    if (noise == true ) {
      ScreenText(WHITE, 20, 200 , 1, "Noise !", 0);
      SetFilledCircle(RED , 10,  205, 2);
      noise = false;
    }
    else {
      ScreenText(BLACK, 20, 200 , 1, "Noise !", 0);
      SetFilledCircle(BLACK , 10,  205, 2);
    }

    if (disturb == true ) {
      ScreenText(WHITE, 20, 220 , 1, "Disturber !", 0);
      SetFilledCircle(RED , 10,  225, 2);
      disturb = false;
    }
    else {
      ScreenText(BLACK, 20, 220 , 1, "Disturber !", 0);
      SetFilledCircle(BLACK , 10,  225, 2);
    }
    //------------------------------------------------------------
    //ScreenText(WHITE, 140, 10 , 1, String(lightning_timer), 0);
    SetLines(GREEN , 70, 40, 160 , 239);
    SetLines(GREEN , 250, 40, 160 , 239);
    //  SetCircle(GREEN , 120,  319, 92);//20km
    //  SetCircle(GREEN , 120,  319, 181);//40km
    //  SetCircle(GREEN , 120,  319, 270);//60km

    //activ,strenght,distance,age,x_pos,y_pos
    //activ:0/1
    //strenght:1-100
    //distance=0-63km
    //time_stamp=0-32767seconds
    //x:0-139 //position onn screen
    //y:0-239 //position onn screen
    for (int i = 0; i < 50 ; i++) {
      if (lightning_strike[i][0] == 1) {//activ
        strikes_count++;
        lightning_age = lightning_timer - lightning_strike[i][3] ; //time_stamp
        if (lightning_age < 0) {
          lightning_age = lightning_age + 32768;
        }
        //ScreenText(WHITE, 0, 300 , 2, String(lightning_age), 0);
        if ( lightning_age < (60 * time_factor)) {
          SetFilledCircle(WHITE, lightning_strike[i][4], lightning_strike[i][5] , 2);
        }
        if ( lightning_age >= (60 * time_factor) && lightning_age < (120 * time_factor)) {
          SetFilledCircle(YELLOW, lightning_strike[i][4], lightning_strike[i][5] , 2);
        }
        if ( lightning_age >= (120 * time_factor) && lightning_age < (180 * time_factor)) {
          SetFilledCircle(RED, lightning_strike[i][4], lightning_strike[i][5] , 2);
        }
        if ( lightning_age >= (180 * time_factor) && lightning_age < (240 * time_factor)) {
          SetFilledCircle(BLUE, lightning_strike[i][4], lightning_strike[i][5] , 2);
        }
        if ( lightning_age >= (240 * time_factor)) {
          SetFilledCircle(BLACK, lightning_strike[i][4], lightning_strike[i][5] , 2);
          lightning_strike[i][0] = 0; //deactiv
        }
      }
    }
    //-------------------------------------------------------------------------------
    //count strikes
    ScreenText(WHITE, 80, 10 , 1,  "Lightning Strikes:", 0);
    if (copy_strikes_count != strikes_count) {
      SetFilledRect(BLACK, 230, 10, 250, 20);
      ScreenText(WHITE, 230, 10 , 1,  String(strikes_count), 0);
      copy_strikes_count = strikes_count;
    }
    //-------------------------------------------------------------------------------
    //clear the oldest lightning when array full
    if ( lightning_strike[49][0] == 1) {
      int lightning_oldest = 0;
      int array_position = 0;
      for (int x = 0; x < 50 ; x++) {
        lightning_age = lightning_timer - lightning_strike[x][3] ; //time_stamp
        if (lightning_age < 0) {
          lightning_age = lightning_age + 32768;
        }
        if (lightning_age > lightning_oldest) {
          lightning_oldest = lightning_age;
          array_position = x;
        }
      }
      SetFilledCircle(BLACK, lightning_strike[array_position][4], lightning_strike[array_position][5] , 2);
      lightning_strike[array_position][0] = 0;
    }
    lightning_direction();
  }
}
//--------------------------------------------------------------
void lightning_direction() {
  //search the oldest and newest:
  int lightning_oldest = 0;
  int lightning_newest = 32767;
  int array_position_oldest = 0;
  int array_position_newest = 0;
  int lightning_age_1;

  for (int t = 0; t < 50 ; t++) {
    if ( lightning_strike[t][0] == 1) {
      lightning_age_1 = lightning_timer - lightning_strike[t][3] ; //time_stamp
      if (lightning_age_1 < 0) {
        lightning_age_1 = lightning_age_1 + 32768;
      }

      if (lightning_age_1 > lightning_oldest) {
        lightning_oldest = lightning_age_1;
        array_position_oldest = t;
      }
      if (lightning_age_1 < lightning_newest) {
        lightning_newest = lightning_age_1;
        array_position_newest = t;
      }
    }
  }
  if (lightning_strike[array_position_oldest][5] < lightning_strike[array_position_newest][5]) {
    SetTriangle(BLACK , 200, 210, 210, 210, 205, 200);
    SetTriangle(WHITE , 200, 200, 210, 200, 205, 210);

  }
  if (lightning_strike[array_position_oldest][5] > lightning_strike[array_position_newest][5]) {
    SetTriangle(BLACK , 200, 200, 210, 200, 205, 210);
    SetTriangle(WHITE , 200, 210, 210, 210, 205, 200);
  }
  if (lightning_strike[array_position_oldest][5] == lightning_strike[array_position_newest][5]) {
    SetTriangle(BLACK , 200, 200, 210, 200, 205, 210);
    SetTriangle(BLACK , 200, 210, 210, 210, 205, 200);
  }
}
//--------------------------------------------------------------
//--------------------------------------------------------------
// this is irq handler for AS3935 interrupts, has to return void and take no arguments
// always make code in interrupt handlers fast and short
void AS3935_ISR()
{
  AS3935_ISR_Trig = 1;
}
