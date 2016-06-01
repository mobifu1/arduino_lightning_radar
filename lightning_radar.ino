//Lightning Radar by Arduino Mega & AS3935 Modul
//
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
// -------------------
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
int but1, but2, pressed_button;
void setup()
{
  // Setup the LCD
  tft.InitLCD();
  tft.clrScr();
  tft.setBackColor(BLACK);

  myTouch.InitTouch(1);
  myTouch.setPrecision(PREC_MEDIUM);
  myButtons.setTextFont(SmallFont);

  but1 = myButtons.addButton( 10,  20, 30,  40, "Menue");
  but2 = myButtons.addButton( 40,  50, 60,  70, "Back", BUTTON_DISABLED);

  ScreenText(WHITE, 0, 20, 1 , "Start");
}

void loop() {

  ScreenText(WHITE, 0, 50, 1 , "Test");
 
 //------------------------------------------------
  myButtons.drawButtons();

  while (1)
  {
    if (myTouch.dataAvailable() == true)
    {
      pressed_button = myButtons.checkButtons();

      if (pressed_button == but1) {
        if (myButtons.buttonEnabled(but1)) {
          myButtons.disableButton(but1, true);
          myButtons.enableButton(but2, true);
          ScreenText(WHITE, 0, 220, 1 , "Button 1");
        }
      }
      if (pressed_button == but2) {
        if (myButtons.buttonEnabled(but2)) {
          myButtons.disableButton(but2, true);
          myButtons.enableButton(but1, true);
          ScreenText(RED, 0, 240, 1 , "Button 2");
        }
      }
    }
  }
}
//----------------------------------------------
//--------------GRAFIK-ROUTINEN-----------------
//----------------------------------------------
unsigned long ScreenText(uint16_t color, int xtpos, int ytpos, int text_size , String text) {
  tft.setColor(color);
  if (text_size == 1) {
    tft.setFont(SmallFont);
  }
  if (text_size == 2) {
    tft.setFont(BigFont);
  }
  tft.print(text, xtpos, ytpos);
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

