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
int but1, but2, but3, but4, but5, but6, but7, but8, but9, but10, pressed_button;
int but20, but21, but22, but23, but24;
boolean menue_on = false;

//Pin
#define Beep A5// Pin of Piezo
#define test 13// Pin of test

#include <TimerOne.h>
//Lightning_Strikes
int copy_strikes_count = 0;
int strikes_count = 0;
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
int last_stroke_index;//position on table
int last_xpos;
int last_ypos;
boolean noise = false;//flag
boolean disturb = false;//flag
boolean simulate_on = false;
byte profile_outdoor;
boolean sound_on;
boolean stats;
byte disturber_on;
byte SetMinStrikes;
byte SetNoiseFloorLvl;
byte SetWatchdogThreshold;
byte SetSpikeRejection;
boolean tick = false;
int total_strikes = 0;
int culmulation_strikes = 0;
byte culmulation_strikes_low = 0;
byte culmulation_strikes_high = 0;

//--record data-----
uint16_t time_slot_strike[80][1] = {
  {0},
  {0},
  {0},
  {0},
  {0},
  {0},
  {0},
  {0},
  {0},
  {0},
  {0},
  {0},
  {0},
  {0},
  {0},
  {0},
  {0},
  {0},
  {0},
  {0},
  {0},
  {0},
  {0},
  {0},
  {0},
  {0},
  {0},
  {0},
  {0},
  {0},
  {0},
  {0},
  {0},
  {0},
  {0},
  {0},
  {0},
  {0},
  {0},
  {0},
  {0},
  {0},
  {0},
  {0},
  {0},
  {0},
  {0},
  {0},
  {0},
  {0},
  {0},
  {0},
  {0},
  {0},
  {0},
  {0},
  {0},
  {0},
  {0},
  {0},
};
int record_strikes = 0;
int time_index = 0;
byte set_value = 2;

#include <EEPROM.h>
//----------------------------------------------------------------
// The AS3935 communicates via SPI or I2C.
// This example uses the I2C interface via the I2C lib, not Wire lib
#include "I2C.h"
// include Playing With Fusion AXS3935 libraries
#include "PWFusion_AS3935_I2C.h"

//  PWF_AS3935_I2C(uint8_t IRQx, uint8_t SIx, uint8_t DEVADDx);
//  AS3935_ManualCal(uint8_t capacitance, uint8_t location, uint8_t disturber);
//  AS3935_DefInit(void);
//  AS3935_PowerUp(void);
//  AS3935_PowerDown(void);
//  AS3935_DisturberEn(void);
//  AS3935_DisturberDis(void);
//  AS3935_SetIRQ_Output_Source(uint8_t irq_select);
//  AS3935_SetTuningCaps(uint8_t cap_val);
//  AS3935_GetInterruptSrc(void);
//  AS3935_GetLightningDistKm(void);
//  AS3935_GetStrikeEnergyRaw(void);
//  AS3935_SetMinStrikes(uint8_t min_strk);
//  AS3935_ClearStatistics(void);
//  AS3935_SetIndoors(void);
//  AS3935_SetOutdoors(void);
//  AS3935_GetNoiseFloorLvl(void);
//  AS3935_SetNoiseFloorLvl(uint8_t nf_sel);
//  AS3935_GetWatchdogThreshold(void);
//  AS3935_SetWatchdogThreshold(uint8_t wdth);
//  AS3935_GetSpikeRejection(void);
//  AS3935_SetSpikeRejection(uint8_t srej);
//  AS3935_SetLCO_FDIV(uint8_t fdiv);
//  AS3935_PrintAllRegs(void);

// interrupt trigger global var
volatile int8_t AS3935_ISR_Trig = 0;

// defines for hardware config
#define SI_PIN               18
#define IRQ_PIN              19        // digital pins 2,3,21,20,19,18 are available for interrupt capability
#define AS3935_ADD           0x03     // x03 - standard PWF SEN-39001-R01 config
#define AS3935_CAPACITANCE   88       // <-- SET THIS VALUE TO THE NUMBER LISTED ON YOUR BOARD 
//Mega Board: sda:20, scl:21, irq:19, si:18

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

  myTouch.InitTouch(1);
  myTouch.setPrecision(PREC_MEDIUM);
  delay(100);

  //pinMode(IRQ_PIN, INPUT);
  pinMode(Beep, OUTPUT);
  pinMode(test, OUTPUT);
  ScreenText(WHITE, 10, 10 , 2, "V1.3-Beta", 0);
  ScreenText(WHITE, 10, 50 , 1, "Touch Available:" + String(myTouch.dataAvailable()), 0);
  //------------------------------------------------------------------------------
  Serial.begin(9600);
  //Serial.println("Playing With Fusion: AS3935 Lightning Sensor, SEN-39001-R01");
  //Serial.println("beginning boot procedure....");

  // setup for the the I2C library: (enable pullups, set speed to 400kHz)
  I2c.begin();
  I2c.pullup(true);
  I2c.setSpeed(1);
  delay(2);
  //ScreenText(WHITE, 30, 140 , 1, "I2C result only at Serial Port" , 0);
  //ScreenText(WHITE, 30, 170 , 1, "Wait 20 sec !" , 0);
  //I2c.scan();
  ScreenText(WHITE, 10, 70 , 1, "Init Serial & I2C", 0);

  load_values();//load value from eeprom
  lightning0.AS3935_DefInit();   // set registers to default
  delay(100);
  // now update sensor cal for your application and power up chip
  //lightning0.AS3935_ManualCal(AS3935_CAPACITANCE, 1, 1);//outdoor = 1 / disten = 1
  lightning0.AS3935_ManualCal(AS3935_CAPACITANCE, profile_outdoor, disturber_on);//outdoor = 1 / disten = 1
  calibrated = true;
  delay(100);
  lightning0.AS3935_SetNoiseFloorLvl(SetNoiseFloorLvl);
  delay(100);
  lightning0.AS3935_SetWatchdogThreshold(SetWatchdogThreshold);
  delay(100);
  lightning0.AS3935_SetSpikeRejection(SetSpikeRejection);
  delay(100);

  //I got the best results when activating the IC's outdoor setting. No matter if it actually was physically outdoors or indoors.
  //Though i eventually packed the Arduino and the chip into a ziplock bag and stuck it out the window to get rained on.
  //noisefl | spike rej | watchdog | indoor/outdoor | tuneAntenna
  //0,0,0,outdoors > 37km, 20km, 20km
  //0,0,0,outdoors,5 > 40km, 37km, 34km, 34km, 27km
  //1,0,0,outdoors,5 > 27km, 24km,

  // AS3935_ManualCal Parameters:
  //   --> capacitance, in pF (marked on package)
  //   --> indoors/outdoors (INDOORS:0 / OUTDOORS:1)
  //   --> disturbers (DIST_EN:1 / DIST_DIS:0)
  // function also powers up the chip

  ScreenText(WHITE, 10, 90 , 1, "Load Data from EEPROM", 0);

  // enable interrupt (hook IRQ pin to Arduino Uno/Mega interrupt input: 0 -> pin 2, 1 -> pin 3 / 2 -> pin 21, 3 -> pin 20, 4 -> pin 19, 5 -> pin 18)
  attachInterrupt(4, AS3935_ISR, RISING);

  AS3935_ISR_Trig = 0;           // clear trigger

  ScreenText(WHITE, 10, 110 , 1, "Init AS3935", 0);
  ScreenText(WHITE, 10, 130 , 1, "Debuging on Serial Interface", 0);

  //-------------------------------------------------------------------------------------------
  delay(5000);
  tft.clrScr();
  tft.setBackColor(BLACK);

  myButtons.setTextFont(SmallFont);
  myButtons.setButtonColors(VGA_WHITE, VGA_WHITE, VGA_WHITE, VGA_BLACK, VGA_BLACK);
  but1 = myButtons.addButton( 280,  200, 30,  30, "");
  //----------------------------------------------------------
  but4 = myButtons.addButton( 10,  5, 90,  30, "Simulate");
  myButtons.disableButton(but4, true);
  but2 = myButtons.addButton( 10,  45, 90,  30, "In/Outdoor");
  myButtons.disableButton(but2, true);
  but3 = myButtons.addButton( 10,  85, 90,  30, "Disturber");
  myButtons.disableButton(but3, true);
  //----------------------------------------------------------
  but5 = myButtons.addButton( 115,  5, 90,  30, "Life x");
  myButtons.disableButton(but5, true);
  but6 = myButtons.addButton( 115,  45, 90,  30, "Stats");
  myButtons.disableButton(but6, true);
  but7 = myButtons.addButton( 115,  85, 90,  30, "Sound");
  myButtons.disableButton(but7, true);
  //----------------------------------------------------------
  but8 = myButtons.addButton( 220, 5, 90,  30, "Calibrate");
  myButtons.disableButton(but8, true);
  but9 = myButtons.addButton( 220, 45, 90,  30, "Get Data");
  myButtons.disableButton(but9, true);
  but10 = myButtons.addButton( 220, 85, 90,  30, "Set Data");
  myButtons.disableButton(but10, true);
  //----------------------------------------------------------
  but20 = myButtons.addButton( 21, 140, 130,  20, "Set Noise Lvl");
  myButtons.disableButton(but20, true);
  but21 = myButtons.addButton( 21, 170, 130,  20, "Set Watchdog");
  myButtons.disableButton(but21, true);
  but22 = myButtons.addButton( 21, 200, 130,  20, "Set Spike Lvl");
  myButtons.disableButton(but22, true);

  but23 = myButtons.addButton( 250, 155, 20,  20, "+");
  myButtons.disableButton(but23, true);
  but24 = myButtons.addButton( 250, 185, 20,  20, "-");
  myButtons.disableButton(but24, true);
  //----------------------------------------------------------

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
  //while (0 == AS3935_ISR_Trig) {}
  //delay(5);

  //indication of irq

  if (AS3935_ISR_Trig == 1) {
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
      uint32_t lightning_energy = lightning0.AS3935_GetStrikeEnergyRaw();
      uint8_t lightning_dist_km = lightning0.AS3935_GetLightningDistKm();
      //Serial.print("Lightning detected! Distance to strike: ");
      //Serial.print(lightning_dist_km);
      //Serial.println(" kilometers");
      //Serial.print("Lightning detected! Strike energy: ");
      //Serial.print(lightning_energy);
      //Serial.println(" eV");
      if (lightning_dist_km < 50 && lightning_dist_km >= 0) {
        total_strikes++;
        record_strikes++;
        increment_culmulation_strikes();
        //activ:0/1
        //strenght:1-100
        //distance=0-63km
        //age=0-15min
        //x:0-139 //position onn screen
        //y:0-239 //position onn screen
        for (int i = 0; i < 50 ; i++) {
          if ( lightning_strike[i][0] == 0) {
            last_stroke_index = i;
            lightning_strike[i][0] = 1;
            lightning_strike[i][1] = uint16_t(lightning_energy / 20972); //max:2097151eV = 21Bits,  2097151eV / 20972 = 100%
            lightning_strike[i][2] = lightning_dist_km;
            lightning_strike[i][3] = lightning_timer;
            long randNumber;
            randNumber = random(160 - lightning_dist_km, 160 + lightning_dist_km);
            lightning_strike[i][4] = int(randNumber);
            lightning_strike[i][5] = int(230 - (lightning_dist_km * 4.0));
            SetCircle(BLACK , last_xpos,  last_ypos, 8);
            last_xpos = lightning_strike[i][4];
            last_ypos = lightning_strike[i][5];
            SetCircle(MAGENTA , last_xpos,  last_ypos, 8);
            if (sound_on == true) {
              //do something with piezo
              tone(Beep, 1000, 50);
            }
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
  }
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

  if (myTouch.dataAvailable() == true)
  {
    pressed_button = myButtons.checkButtons();

    if (pressed_button == but1) {//Menue
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
        myButtons.drawButton(but7);
        myButtons.drawButton(but8);
        myButtons.drawButton(but9);
        myButtons.drawButton(but10);
        myButtons.enableButton(but2, true);
        myButtons.enableButton(but3, true);
        myButtons.enableButton(but4, true);
        myButtons.enableButton(but5, true);
        myButtons.enableButton(but6, true);
        myButtons.enableButton(but7, true);
        myButtons.enableButton(but8, true);
        myButtons.enableButton(but9, true);
        myButtons.enableButton(but10, true);
        SetRect(WHITE , 10, 125, 309, 234);//Frame
      }
    }
    if (pressed_button == but2) {//Indoor/Outdoor
      if (myButtons.buttonEnabled(but2)) {
        myButtons.disableButton(but2, true);
        myButtons.disableButton(but3, true);
        myButtons.disableButton(but4, true);
        myButtons.disableButton(but5, true);
        myButtons.disableButton(but6, true);
        myButtons.disableButton(but7, true);
        myButtons.disableButton(but8, true);
        myButtons.disableButton(but9, true);
        myButtons.disableButton(but10, true);
        myButtons.disableButton(but20, true);
        myButtons.disableButton(but21, true);
        myButtons.disableButton(but22, true);
        myButtons.disableButton(but23, true);
        myButtons.disableButton(but24, true);
        tft.clrScr();
        tft.setBackColor(BLACK);
        myButtons.drawButton(but1);
        myButtons.enableButton(but1, true);
        if (profile_outdoor == 1) {
          set_chip_value("SetIndoors", 0);
        }
        else {
          set_chip_value("SetOutdoors", 0);
        }
        menue_on = false;
      }
    }
    if (pressed_button == but3) {//Disturber
      if (myButtons.buttonEnabled(but3)) {
        myButtons.disableButton(but2, true);
        myButtons.disableButton(but3, true);
        myButtons.disableButton(but4, true);
        myButtons.disableButton(but5, true);
        myButtons.disableButton(but6, true);
        myButtons.disableButton(but7, true);
        myButtons.disableButton(but8, true);
        myButtons.disableButton(but9, true);
        myButtons.disableButton(but10, true);
        myButtons.disableButton(but20, true);
        myButtons.disableButton(but21, true);
        myButtons.disableButton(but22, true);
        myButtons.disableButton(but23, true);
        myButtons.disableButton(but24, true);
        tft.clrScr();
        tft.setBackColor(BLACK);
        myButtons.drawButton(but1);
        myButtons.enableButton(but1, true);
        if (disturber_on == 1) {
          set_chip_value("DisturberDis", 0);
        }
        else {
          set_chip_value("DisturberEn", 0);
        }
        menue_on = false;
      }
    }
    if (pressed_button == but4) {//Simulate
      if (myButtons.buttonEnabled(but4)) {
        myButtons.disableButton(but2, true);
        myButtons.disableButton(but3, true);
        myButtons.disableButton(but4, true);
        myButtons.disableButton(but5, true);
        myButtons.disableButton(but6, true);
        myButtons.disableButton(but7, true);
        myButtons.disableButton(but8, true);
        myButtons.disableButton(but9, true);
        myButtons.disableButton(but10, true);
        myButtons.disableButton(but20, true);
        myButtons.disableButton(but21, true);
        myButtons.disableButton(but22, true);
        myButtons.disableButton(but23, true);
        myButtons.disableButton(but24, true);
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
    if (pressed_button == but5) {//Life Factor
      if (myButtons.buttonEnabled(but5)) {
        SetFilledRect(BLACK , 11, 126, 308, 233);
        time_factor++;
        if (time_factor > 16) {
          time_factor = 1;
        }
        EEPROM.update(2, time_factor);
        ScreenText(WHITE, 30, 140 , 1, "Strike life factor: " + String(time_factor), 0);
        ScreenText(WHITE, 30, 170 , 1, "Lifetime: " + String(time_factor * 240 / 60) + " min", 0);

        load_values();//load value from eeprom
      }
    }
    if (pressed_button == but6) {//Statistik
      if (myButtons.buttonEnabled(but6)) {
        myButtons.disableButton(but2, true);
        myButtons.disableButton(but3, true);
        myButtons.disableButton(but4, true);
        myButtons.disableButton(but5, true);
        myButtons.disableButton(but6, true);
        myButtons.disableButton(but7, true);
        myButtons.disableButton(but8, true);
        myButtons.disableButton(but9, true);
        myButtons.disableButton(but10, true);
        myButtons.disableButton(but20, true);
        myButtons.disableButton(but21, true);
        myButtons.disableButton(but22, true);
        myButtons.disableButton(but23, true);
        myButtons.disableButton(but24, true);
        tft.clrScr();
        tft.setBackColor(BLACK);
        myButtons.drawButton(but1);
        myButtons.enableButton(but1, true);
        if (stats == false) {
          EEPROM.update(3, 1);
        }
        else {
          //stats  = false;
          EEPROM.update(3, 0);
        }
        load_values();//load value from eeprom
        menue_on = false;
      }
    }
    if (pressed_button == but7) {//Sound
      if (myButtons.buttonEnabled(but7)) {
        myButtons.disableButton(but2, true);
        myButtons.disableButton(but3, true);
        myButtons.disableButton(but4, true);
        myButtons.disableButton(but5, true);
        myButtons.disableButton(but6, true);
        myButtons.disableButton(but7, true);
        myButtons.disableButton(but8, true);
        myButtons.disableButton(but9, true);
        myButtons.disableButton(but10, true);
        myButtons.disableButton(but20, true);
        myButtons.disableButton(but21, true);
        myButtons.disableButton(but22, true);
        myButtons.disableButton(but23, true);
        myButtons.disableButton(but24, true);
        tft.clrScr();
        tft.setBackColor(BLACK);
        myButtons.drawButton(but1);
        myButtons.enableButton(but1, true);
        if (sound_on == false) {
          EEPROM.update(4, 1);
        }
        else {
          EEPROM.update(4, 0);
        }
        load_values();//load value from eeprom
        menue_on = false;
      }
    }
    if (pressed_button == but8) {//Calibrate
      if (myButtons.buttonEnabled(but8)) {
        myButtons.disableButton(but2, true);
        myButtons.disableButton(but3, true);
        myButtons.disableButton(but4, true);
        myButtons.disableButton(but5, true);
        myButtons.disableButton(but6, true);
        myButtons.disableButton(but7, true);
        myButtons.disableButton(but8, true);
        myButtons.disableButton(but9, true);
        myButtons.disableButton(but10, true);
        myButtons.disableButton(but20, true);
        myButtons.disableButton(but21, true);
        myButtons.disableButton(but22, true);
        myButtons.disableButton(but23, true);
        myButtons.disableButton(but24, true);
        tft.clrScr();
        tft.setBackColor(BLACK);
        myButtons.drawButton(but1);
        myButtons.enableButton(but1, true);
        lightning0.AS3935_ManualCal(AS3935_CAPACITANCE, profile_outdoor, disturber_on);
        calibrated = true;
        menue_on = false;
      }
    }
    if (pressed_button == but9) {//Get Chip Data
      if (myButtons.buttonEnabled(but9)) {
        SetFilledRect(BLACK , 11, 126, 308, 233);
        chip_data();
      }
    }
    if (pressed_button == but10) {//Set chip data
      if (myButtons.buttonEnabled(but10)) {
        SetFilledRect(BLACK , 11, 126, 308, 233);
        myButtons.drawButton(but20);
        myButtons.drawButton(but21);
        myButtons.drawButton(but22);
        myButtons.drawButton(but23);
        myButtons.drawButton(but24);
        myButtons.enableButton(but20, true);
        myButtons.enableButton(but21, true);
        myButtons.enableButton(but22, true);
        myButtons.enableButton(but23, true);
        myButtons.enableButton(but24, true);
        ScreenText(WHITE, 200, 170 , 1, String(set_value) + " ", 0);

      }
    }
    if (pressed_button == but20) {//set Noise Lvl
      if (myButtons.buttonEnabled(but20)) {
        set_chip_value("SetNoiseFloorLvl", set_value);
        calibrated = false;
      }
    }
    if (pressed_button == but21) {//set Watchdog threshhold
      if (myButtons.buttonEnabled(but21)) {
        set_chip_value("SetWatchdogThreshold", set_value);
        calibrated = false;
      }
    }
    if (pressed_button == but22) {//set Spike Lvl
      if (myButtons.buttonEnabled(but22)) {
        set_chip_value("SetSpikeRejection", set_value);
        calibrated = false;
      }
    }

    if (pressed_button == but23) {//incr. Value
      if (myButtons.buttonEnabled(but23)) {
        set_value++;
        if (set_value > 7) set_value = 7;
        ScreenText(WHITE, 200, 170 , 1,  String(set_value)  + " ", 0);
      }
    }
    if (pressed_button == but24) {//decr.. Value
      if (myButtons.buttonEnabled(but24)) {
        set_value--;
        if (set_value > 7) set_value = 7;
        ScreenText(WHITE, 200, 170 , 1,  String(set_value) + " " , 0);
      }
    }
  }
}
//----------------------------------------------
void load_values () {

  int value;
  profile_outdoor = EEPROM.read(0);//indoor/outdoor

  value = EEPROM.read(1);//simulate
  if (value == 0) {
    //simulate_on = false;
  }
  if (value == 1) {
    //simulate_on = true;
  }

  time_factor = EEPROM.read(2);//time factor

  value = EEPROM.read(3);//stats
  if (value == 0) {
    stats = false;
  }
  if (value == 1) {
    stats = true;
  }

  value = EEPROM.read(4);//sound
  if (value == 0) {
    sound_on = false;
  }
  if (value == 1) {
    sound_on = true;
  }

  culmulation_strikes_low = EEPROM.read(5);//count all strikes
  culmulation_strikes_high = EEPROM.read(6);//count all strikes
  culmulation_strikes = ((culmulation_strikes_low << 0) & 0xFF) + ((culmulation_strikes_high << 8) & 0xFF00);
  if (culmulation_strikes < 0 || culmulation_strikes == 32767 ) {
    culmulation_strikes = 0;
    culmulation_strikes_low = ((culmulation_strikes >> 0) & 0xFF);
    culmulation_strikes_high = ((culmulation_strikes >> 8) & 0xFF);
    EEPROM.write(5, culmulation_strikes_low);
    EEPROM.write(6, culmulation_strikes_high);
  }

  disturber_on = EEPROM.read(7);//disturber
  SetMinStrikes = EEPROM.read(8);//SetMinStrikes
  SetNoiseFloorLvl = EEPROM.read(9);//SetNoiseFloorLvl
  SetWatchdogThreshold = EEPROM.read(10);//SetWatchdogThreshold
  SetSpikeRejection = EEPROM.read(11);//SetSpikeRejection
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
    long randNumber2 = random(0, 45);
    long randNumber3 = random(20, 100);
    total_strikes++;
    record_strikes++;
    //increment_culmulation_strikes();
    for (int i = 0; i < 50 ; i++) {
      if ( lightning_strike[i][0] == 0) {
        last_stroke_index = i;
        lightning_strike[i][0] = 1;
        lightning_strike[i][1] = randNumber3;//from 0 to 100%
        lightning_strike[i][2] = int(randNumber2);//dist
        lightning_strike[i][3] = lightning_timer;
        long randNumber;
        randNumber = random(160 - lightning_strike[i][2], 160 + lightning_strike[i][2]);
        lightning_strike[i][4] = int(randNumber);
        lightning_strike[i][5] = int(230 - (lightning_strike[i][2] * 4.0));//235
        SetCircle(BLACK , last_xpos,  last_ypos, 8);
        last_xpos = lightning_strike[i][4];
        last_ypos = lightning_strike[i][5];
        SetCircle(MAGENTA , last_xpos,  last_ypos, 8);
        if (sound_on == true) {
          //do something with piezo
          tone(Beep, 1000, 50);//pin,hertz,duration
        }
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

  //--------increment counter---------------------------------
  lightning_timer++;
  if (lightning_timer < 0) {
    lightning_timer = 0;
  }
  //----------------------------------------------------------
  if (menue_on == false) {

    if (tick == true) {
      SetFilledCircle(WHITE , 12,  15, 2);
      tick = false;
    }
    else {
      SetFilledCircle(BLACK , 12,  15, 2);
      tick = true;
    }

    if (sound_on == true) {
      ScreenText(WHITE, 210, 165 , 1, "Sound On", 0);
    }

    if (profile_outdoor == 1) {
      ScreenText(WHITE, 210, 185 , 1, "Outdoor", 0);
    }
    else {
      ScreenText(WHITE, 210, 185 , 1, "Indoor", 0);
    }

    ScreenText(WHITE, 40, 50 , 1, "45km", 0);
    ScreenText(WHITE, 67, 110 , 1, "30km", 0);
    ScreenText(WHITE, 93, 170 , 1, "15km", 0);

    if (last_xpos > 0 && last_ypos > 0) {
      SetCircle(BLACK , last_xpos,  last_ypos, 8); //display flackert wenn koordinaten ausserhalb vom display
    }

    if (simulate_on == true) {
      simulate_strikes();
    }

    //------Scale-----------------------------------------------
    int lightning_age;
    strikes_count = 0;
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

    if (disturber_on == 0 ) {
      SetFilledCircle(GRAY , 10,  175, 2);
      ScreenText(GRAY, 20, 170 , 1, "Dist of", 0);
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
          lightning_strike[i][1] = 0; //energy
        }
      }
    }
    //-------------------------------------------------------------------------------
    //count strikes
    ScreenText(WHITE, 80, 10 , 1,  "Lightning Strikes:", 0);
    if (strikes_count < 10) {
      ScreenText(WHITE, 230, 10 , 1,  String(strikes_count) + " ", 0);
    }
    else {
      ScreenText(WHITE, 230, 10 , 1,  String(strikes_count), 0);
    }

    if (copy_strikes_count == 0 && strikes_count > 0) {//reset when new thunder storm starts
      total_strikes = strikes_count;
    }
    ScreenText(WHITE, 180, 225 , 1,  "[" + String(total_strikes) + "]    ", 0);
    copy_strikes_count = strikes_count;
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
    lightning_energy();
    lightning_direction();
    time_record_strikes();
  }
}
//--------------------------------------------------------------
void lightning_energy() {

  uint16_t stroke_energy = lightning_strike[last_stroke_index][1]; //from 0 to 100%
  if (stroke_energy > 100) {
    stroke_energy = 100;
  }
  if (stroke_energy < 0) {
    stroke_energy = 0;
  }
  ScreenText(WHITE, 5, 148 , 1, "eV", 0);
  SetRect(WHITE , 10, 40, 15, 142);//Frame
  SetFilledRect(BLACK , 11, 41, 14, 141);//clear content
  if (stroke_energy > 0) {
    SetFilledRect(RED , 11, 141 - stroke_energy, 14, 141); //set bargraph
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
  if (lightning_strike[array_position_oldest][5] < lightning_strike[array_position_newest][5] && total_strikes > 9) {
    SetTriangle(BLACK , 130, 235, 140, 235, 135, 225);
    SetTriangle(WHITE , 130, 225, 140, 225, 135, 235);

  }
  if (lightning_strike[array_position_oldest][5] > lightning_strike[array_position_newest][5] && total_strikes > 9) {
    SetTriangle(BLACK , 130, 225, 140, 225, 135, 235);
    SetTriangle(WHITE , 130, 235, 140, 235, 135, 225);
  }
  if (lightning_strike[array_position_oldest][5] == lightning_strike[array_position_newest][5] && total_strikes < 9) {
    SetTriangle(BLACK , 130, 235, 140, 235, 135, 225);
    SetTriangle(BLACK , 130, 225, 140, 225, 135, 235);
  }

  if (stats == true) {
    int value = lightning_oldest - lightning_newest; //age of ligtnings:Statistics
    if (value >= 0 && value < 60) {
      ScreenText(WHITE, 250, 10 , 1, "/" + String(value) + "sec ", 0);
    }
    if (value >= 60 && value  < 3600) {
      value = value / 60;//min
      ScreenText(WHITE, 250, 10 , 1, "/" + String(value) + "min ", 0);
    }
  }
}
void chip_data() {

  ScreenText(WHITE, 30, 140 , 1, "All Strikes: " + String(culmulation_strikes), 0);
  Serial.print("Culmulation Strikes: ");
  Serial.println(culmulation_strikes);

  int noiseFloor = lightning0.AS3935_GetNoiseFloorLvl();
  ScreenText(WHITE, 30, 160 , 1, "Noise floor Level: " + String(noiseFloor), 0);
  Serial.print("Noise floor: ");
  Serial.println(noiseFloor);

  int watchdogThreshold = lightning0.AS3935_GetWatchdogThreshold();
  ScreenText(WHITE, 30, 180 , 1, "Watchdog threshold: " + String(watchdogThreshold), 0);
  Serial.print("Watchdog threshold: ");
  Serial.println(watchdogThreshold);

  int spikeRejection = lightning0.AS3935_GetSpikeRejection();
  ScreenText(WHITE, 30, 200 , 1, "Spike rejection: " + String(spikeRejection), 0);
  Serial.print("Spike rejection: ");
  Serial.println(spikeRejection);

  lightning0.AS3935_PrintAllRegs();
}
//--------------------------------------------------------------
void time_record_strikes() {

  int slot_age = lightning_timer - time_index ; //time_stamp
  if (slot_age < 0) {
    slot_age = slot_age + 32768;
  }
  if (record_strikes > 50) {
    record_strikes = 50;
  }
  time_slot_strike[0][0] = record_strikes;
  //SetLines(GRAY , 318 - 40, 140 - 51, 318 , 140 - 51); //Top Line
  SetLines(GRAY , 319, 141, 319 , 141 - 51); //y line max. 50
  SetLines(RED , 318 , 140, 318 , 140 - time_slot_strike[0][0]);
  ScreenText(RED, 318 - 105, 135 , 1, "12h", 0);//Scale
  SetLines(RED , 318 - 78, 141, 318 , 141);//Base Line


  if (slot_age > 540) {//9min * 80 = 12h
    time_index = lightning_timer;
    for (int s = 78; s >= 0; s--) {
      int copy_value = time_slot_strike[s][0];
      time_slot_strike[s + 1][0] = copy_value;
      SetLines(BLACK , 318 - s, 140, 318 - s , 140 - 50);
      SetLines(RED , 318 - s, 140, 318 - s , 140 - time_slot_strike[s][0]);
    }
    record_strikes = 0;
  }
}
//--------------------------------------------------------------
void increment_culmulation_strikes() {

  culmulation_strikes_low = EEPROM.read(5);//count all strikes
  culmulation_strikes_high = EEPROM.read(6);//count all strikes
  culmulation_strikes = ((culmulation_strikes_low << 0) & 0xFF) + ((culmulation_strikes_high << 8) & 0xFF00);
  culmulation_strikes++;
  culmulation_strikes_low = ((culmulation_strikes >> 0) & 0xFF);
  culmulation_strikes_high = ((culmulation_strikes >> 8) & 0xFF);
  EEPROM.write(5, culmulation_strikes_low);
  EEPROM.write(6, culmulation_strikes_high);
}
//--------------------------------------------------------------
void set_chip_value(String function, int value) {

  if (function == "SetOutdoors") {
    lightning0.AS3935_SetOutdoors();
    EEPROM.update(0, 1);
    Serial.println("SetOutdoors OK");
  }
  if (function == "SetIndoors") {
    lightning0.AS3935_SetIndoors();
    EEPROM.update(0, 0);
    Serial.println("SetIndoors OK");
  }
  if (function == "DisturberEn") {
    lightning0.AS3935_DisturberEn();
    EEPROM.update(7, 1);
    Serial.println("SetDisturberEn OK");
  }
  if (function == "DisturberDis") {
    lightning0.AS3935_DisturberDis();
    EEPROM.update(7, 0);
    Serial.println("SetDisturberDis OK");
  }
  if (function == "SetMinStrikes") {
    lightning0.AS3935_SetMinStrikes(uint8_t(value));
    EEPROM.update(8, value);
    Serial.print("SetMinStrikes:");
    Serial.println(value);
  }
  if (function == "SetNoiseFloorLvl") {
    lightning0.AS3935_SetNoiseFloorLvl(uint8_t(value));
    EEPROM.update(9, value);
    Serial.print("SetNoiseFloorLvl: ");
    Serial.println(value);
  }
  if (function == "SetWatchdogThreshold") {
    lightning0.AS3935_SetWatchdogThreshold(uint8_t(value));
    EEPROM.update(10, value);
    Serial.print("SetWatchdogThreshold: ");
    Serial.println(value);
  }
  if (function == "SetSpikeRejection") {
    lightning0.AS3935_SetSpikeRejection(uint8_t(value));
    EEPROM.update(11, value);
    Serial.print("SetSpikeRejection: ");
    Serial.println(value);
  }
  load_values();//load value from eeprom
}
//--------------------------------------------------------------
//--------------------------------------------------------------
// this is irq handler for AS3935 interrupts, has to return void and take no arguments
// always make code in interrupt handlers fast and short
void AS3935_ISR()
{
  AS3935_ISR_Trig = 1;
}
