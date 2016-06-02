//Lightning Radar by Arduino Mega & AS3935 Modul
/*
  LightningDetector.pde - AS3935 Franklin Lightning Sensor™ IC by AMS library demo code
  Copyright (c) 2012 Raivis Rengelis (raivis [at] rrkb.lv). All rights reserved.
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; eitherre
  version 3 of the License, or (at your option) any later version.
  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
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
int but1, but2, but3, but4 , pressed_button;
boolean menue_on = false;

//Pin
//#define LED A5// Pin of LED

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
boolean profile_indoor = true;
boolean tick = false;
//----------------------------------------------------------------
#include <TimerOne.h>
#include <SPI.h>
#include <AS3935.h>

//void printAS3935Registers();

// Function prototype that provides SPI transfer and is passed to
// AS3935 to be used from within library, it is defined later in main sketch.
// That is up to user to deal with specific implementation of SPI
// Note that AS3935 library requires this function to have exactly this signature
// and it can not be member function of any C++ class, which happens
// to be almost any Arduino library
// Please make sure your implementation of choice does not deal with CS pin,
// library takes care about it on it's own
byte SPItransfer(byte sendByte);

// Iterrupt handler for AS3935 irqs
// and flag variable that indicates interrupt has been triggered
// Variables that get changed in interrupt routines need to be declared volatile
// otherwise compiler can optimize them away, assuming they never get changed
void AS3935Irq();
volatile int AS3935IrqTriggered;

// First parameter - SPI transfer function, second - Arduino pin used for CS
// and finally third argument - Arduino pin used for IRQ
// It is good idea to chose pin that has interrupts attached, that way one can use
// attachInterrupt in sketch to detect interrupt
// Library internally polls this pin when doing calibration, so being an interrupt pin
// is not a requirement

//AS3935 AS3935(SPItransfer, SS, 2);//old default
int PIN_IRQ = 20;//IRQ PIN of Chip
int PIN_CS = 21;//Sensor CS PIN
AS3935 AS3935(SPItransfer, PIN_CS, PIN_IRQ);

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
  ScreenText(WHITE, 0, 10 , 2, "V0.2-Beta", 0);
  //Serial.begin(9600);
  //------------------------------------------------------------------------------
  // first begin, then set parameters
  SPI.begin();
  // NB! chip uses SPI MODE1
  SPI.setDataMode(SPI_MODE1);
  // NB! max SPI clock speed that chip supports is 2MHz,
  // but never use 500kHz, because that will cause interference
  // to lightning detection circuit
  SPI.setClockDivider(SPI_CLOCK_DIV16);
  // and chip is MSB first
  SPI.setBitOrder(MSBFIRST);
  // reset all internal register values to defaults
  AS3935.reset();
  // and run calibration
  // if lightning detector can not tune tank circuit to required tolerance,
  // calibration function will return false
  if (!AS3935.calibrate())
    //Serial.println("Tuning out of range, check your wiring, your sensor and make sure physics laws have not changed!");
    ScreenText(WHITE, 0, 50 , 1, "Tuning out of range !", 0);
  // since this is demo code, we just go on minding our own business and ignore the fact that someone divided by zero
  // first let's turn on disturber indication and print some register values from AS3935
  // tell AS3935 we are indoors, for outdoors use setOutdoors() function
  AS3935.setOutdoors();
  //  AS3935.calibrate
  //  AS3935.powerDown
  //  AS3935.powerUp
  //  AS3935.interruptSource
  //  AS3935.disableDisturbers
  //  AS3935.enableDisturbers
  //  AS3935.minimumLightnings
  //  AS3935.lightningDistanceKm
  //  AS3935.setIndoors
  //  AS3935.setOutdoors
  //  AS3935.getNoiseFloor
  //  AS3935.setNoiseFloor
  //  AS3935.getSpikeRejection
  //  AS3935.setSpikeRejection
  //  AS3935.getWatchdogThreshold
  //  AS3935.setWatchdogThreshold
  //  AS3935.clearStats
  // turn on indication of distrubers, once you have AS3935 all tuned, you can turn those off with disableDisturbers()
  AS3935.enableDisturbers();
  printAS3935Registers();
  AS3935IrqTriggered = 0;
  // Using interrupts means you do not have to check for pin being set continiously, chip does that for you and
  // notifies your code
  // demo is written and tested on ChipKit MAX32, irq pin is connected to max32 pin 2, that corresponds to interrupt 1
  // look up what pins can be used as interrupts on your specific board and how pins map to int numbers

  // ChipKit Max32 - irq connected to pin 2
  //attachInterrupt(1, AS3935Irq, RISING);
  int myIRQ = 3;//Mega Board
  attachInterrupt(myIRQ, AS3935Irq, RISING);
  // uncomment line below and comment out line above for Arduino Mega 2560, irq still connected to pin 2
  // attachInterrupt(0,AS3935Irq,RISING);
  //-------------------------------------------------------------------------------------------
  delay(5000);
  tft.clrScr();
  tft.setBackColor(BLACK);

  myTouch.InitTouch(1);
  myTouch.setPrecision(PREC_MEDIUM);

  myButtons.setTextFont(SmallFont);
  myButtons.setButtonColors(VGA_WHITE, VGA_WHITE, VGA_WHITE, VGA_BLACK, VGA_BLACK);
  but1 = myButtons.addButton( 280,  200, 30,  30, "");
  but2 = myButtons.addButton( 10,  50, 75,  30, "Indoor");
  myButtons.disableButton(but2, true);
  but3 = myButtons.addButton( 10,  90, 75,  30, "Outdoor");
  myButtons.disableButton(but3, true);
  but4 = myButtons.addButton( 10,  10, 75,  30, "Simulate");
  myButtons.disableButton(but4, true);


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

  // here we go into loop checking if interrupt has been triggered, which kind of defeats
  // the whole purpose of interrupts, but in real life you could put your chip to sleep
  // and lower power consumption or do other nifty things
  if (AS3935IrqTriggered)  {

    // reset the flag
    AS3935IrqTriggered = 0;
    // first step is to find out what caused interrupt
    // as soon as we read interrupt cause register, irq pin goes low
    int irqSource = AS3935.interruptSource();
    // returned value is bitmap field, bit 0 - noise level too high, bit 2 - disturber detected, and finally bit 3 - lightning!
    if (irqSource & 0b0001)
      noise = true;
    //Serial.println("Noise level too high, try adjusting noise floor");

    if (irqSource & 0b0100)
      disturb = true;
    //Serial.println("Disturber detected");

    if (irqSource & 0b1000)  {
      noise = false;
      disturb = false;
      // need to find how far that lightning stroke, function returns approximate distance in kilometers,
      // where value 1 represents storm in detector's near victinity, and 63 - very distant, out of range stroke
      // everything in between is just distance in kilometers
      int strokeDistance = AS3935.lightningDistanceKm();

      if (strokeDistance == 1) {
        //Serial.println("Storm overhead, watch out!");
      }

      if (strokeDistance == 63) {
        //Serial.println("Out of range lightning detected.");
      }

      if (strokeDistance < 63 && strokeDistance > 1) {
        //Serial.print("Lightning detected ");
        //Serial.print(strokeDistance, DEC);
        //Serial.println(" kilometers away.");
      }
      if (strokeDistance < 64 && strokeDistance >= 0) {
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
            lightning_strike[i][2] = strokeDistance;
            lightning_strike[i][3] = lightning_timer;
            long randNumber;
            randNumber = random(160 - strokeDistance, 160 + strokeDistance);
            lightning_strike[i][4] = int(randNumber);
            lightning_strike[i][5] = int(230 - (strokeDistance * 3.0));
            SetCircle(BLACK , last_xpos,  last_ypos, 8);
            last_xpos = lightning_strike[i][4];
            last_ypos = lightning_strike[i][5];
            SetCircle(MAGENTA , last_xpos,  last_ypos, 8);
            break;
          }
        }
      }
    }
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
          myButtons.enableButton(but2, true);
          myButtons.enableButton(but3, true);
          myButtons.enableButton(but4, true);
        }
      }
      if (pressed_button == but2) {
        if (myButtons.buttonEnabled(but2)) {
          myButtons.disableButton(but2, true);
          myButtons.disableButton(but3, true);
          myButtons.disableButton(but4, true);
          tft.clrScr();
          tft.setBackColor(BLACK);
          myButtons.drawButton(but1);
          myButtons.enableButton(but1, true);
          AS3935.setIndoors();
          profile_indoor = true;
          menue_on = false;
        }
      }
      if (pressed_button == but3) {
        if (myButtons.buttonEnabled(but3)) {
          myButtons.disableButton(but2, true);
          myButtons.disableButton(but3, true);
          myButtons.disableButton(but4, true);
          tft.clrScr();
          tft.setBackColor(BLACK);
          myButtons.drawButton(but1);
          myButtons.enableButton(but1, true);
          AS3935.setOutdoors();
          profile_indoor = false;
          menue_on = false;
        }
      }
      if (pressed_button == but4) {
        if (myButtons.buttonEnabled(but4)) {
          myButtons.disableButton(but2, true);
          myButtons.disableButton(but3, true);
          myButtons.disableButton(but4, true);
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

    if (profile_indoor == true) {
      ScreenText(WHITE, 230, 160 , 1, "Indoor", 0);
    }
    else {
      ScreenText(WHITE, 230, 160 , 1, "Outdoor", 0);
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
    ScreenText(WHITE, 80, 10 , 1,  "Lightning Strikes: ", 0);
    if (copy_strikes_count != strikes_count) {
      SetFilledRect(BLACK, 230, 10, 280, 20);
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
void printAS3935Registers()
{
  int noiseFloor = AS3935.getNoiseFloor();
  int spikeRejection = AS3935.getSpikeRejection();
  int watchdogThreshold = AS3935.getWatchdogThreshold();
  ScreenText(WHITE, 0, 70 , 1, "Read Data from AS3935:", 0);
  //Serial.print("Noise floor is: ");
  //Serial.println(noiseFloor, DEC);
  ScreenText(WHITE, 0, 90 , 1, "Noise floor is: " + noiseFloor, 0);
  //Serial.print("Spike rejection is: ");
  //Serial.println(spikeRejection, DEC);
  ScreenText(WHITE, 0, 110 , 1, "Spike rejection is: " + spikeRejection, 0);
  //Serial.print("Watchdog threshold is: ");
  //Serial.println(watchdogThreshold, DEC);
  ScreenText(WHITE, 0, 130 , 1, "Watchdog threshold is: " + watchdogThreshold, 0);
}

// this is implementation of SPI transfer that gets passed to AS3935
// you can (hopefully) wrap any SPI implementation in this
byte SPItransfer(byte sendByte)
{
  return SPI.transfer(sendByte);
}

// this is irq handler for AS3935 interrupts, has to return void and take no arguments
// always make code in interrupt handlers fast and short
void AS3935Irq()
{
  AS3935IrqTriggered = 1;
}
