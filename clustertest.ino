/***************************************************
  This is our GFX example for the Adafruit ILI9341 Breakout and Shield
  ----> http://www.adafruit.com/products/1651

  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional)
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/


#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"

// For the Adafruit shield, these are the default.
#define TFT_DC 9
#define TFT_CS 10
#define TFT_RST 8

// Use hardware SPI (on Uno, #13, #12, #11) and the above for CS/DC
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);
// If using the breakout, change pins as desired
//Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_MOSI, TFT_CLK, TFT_RST, TFT_MISO);

const uint16_t SCREEN_CENTRE_X = 160;
const uint16_t SCREEN_CENTRE_Y = 120;
const uint16_t GAUGE_RADIUS = 65;
const uint16_t SPEEDO_CENTRE_X = SCREEN_CENTRE_X - GAUGE_RADIUS - 20;
const uint16_t SPEEDO_CENTRE_Y = SCREEN_CENTRE_Y - 20;
const uint16_t TACHO_CENTRE_X = SCREEN_CENTRE_X + GAUGE_RADIUS + 20;
const uint16_t TACHO_CENTRE_Y = SCREEN_CENTRE_Y - 20;
const uint16_t MAX_SPEED = 240;  // Maximum speed scale
const uint16_t MAX_RPM = 8000;   // Maximum RPM scale
const uint16_t MAJOR_TICK_LENGTH = 10;
const uint16_t MINOR_TICK_LENGTH = 5;
const uint16_t TICK_LABEL_OFFSET = 10;
const uint16_t MAJOR_TICK_COLOUR = ILI9341_WHITE;
const uint16_t MINOR_TICK_COLOUR = ILI9341_RED;
const uint16_t TICK_TEXT_SIZE = 1;

void setup() {
  Serial.begin(9600);
  Serial.println("ILI9341 Test!"); 
 
  // deselect all SPI devices
  pinMode(TFT_CS, OUTPUT);
  digitalWrite(TFT_CS, HIGH);
  tft.begin();
  tft.setRotation(1);
  tft.fillScreen(ILI9341_BLACK);
  drawGauges();
}


void loop(void) {

}

void drawGauges(void) {
  tft.print("Hello");
  tft.drawPixel(160, 120, ILI9341_BLUE);
  // Speedometer
  tft.drawCircle(SPEEDO_CENTRE_X, SPEEDO_CENTRE_Y, GAUGE_RADIUS, ILI9341_WHITE);
  tft.fillCircle(SPEEDO_CENTRE_X, SPEEDO_CENTRE_Y, 5, ILI9341_WHITE);
  for (int i = 0; i <= 25; i++) {
    int tickSpeed = i * 10;
    int angle = 120 + i * 12;
    bool isMajorTick = ((i % 2) == 0);
    int length = isMajorTick ? MAJOR_TICK_LENGTH : MINOR_TICK_LENGTH;
    int colour = isMajorTick ? MAJOR_TICK_COLOUR : MINOR_TICK_COLOUR;

    drawTick(SPEEDO_CENTRE_X, SPEEDO_CENTRE_Y, GAUGE_RADIUS, angle, length, ILI9341_WHITE, isMajorTick, tickSpeed);
  }

  tft.drawCircle(TACHO_CENTRE_X, TACHO_CENTRE_Y, GAUGE_RADIUS, ILI9341_WHITE);
}

void drawTick(int centreX, int centreY, int radius, int angle, int length, int colour, bool showLabel, int labelValue) {
  float rads = angle * PI / 180;
  int outerX = centreX + radius * cos(rads);
  int outerY = centreY + radius * sin(rads);
  int innerX = centreX + (radius - length) * cos(rads);
  int innerY = centreY + (radius - length) * sin(rads);
  
  tft.drawLine(innerX, innerY, outerX, outerY, colour);

  if (showLabel) {
    tft.setTextSize(TICK_TEXT_SIZE);
    int labelX = innerX - TICK_LABEL_OFFSET * cos(rads);
    int labelY = innerY - TICK_LABEL_OFFSET * sin(rads);
    if (labelValue >= 0 && labelValue < 10) {
      tft.setCursor(labelX - 5, labelY - 2);
    } else if (labelValue < 100) {
      tft.setCursor(labelX - 6, labelY - 3);
    } else if (labelValue < 1000) {
      tft.setCursor(labelX - 7, labelY - 4);
    }

    tft.print(labelValue);
  }
}
