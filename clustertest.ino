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
const uint16_t MAX_SPEED = 250;  // Maximum speed scale
const uint16_t MAX_RPM = 9000;   // Maximum RPM scale
const uint16_t MAJOR_TICK_LENGTH = 10;
const uint16_t MINOR_TICK_LENGTH = 5;
const uint16_t NEEDLE_BASE_WIDTH = 6;
const uint16_t TICK_LABEL_OFFSET = 10;
const uint16_t MAJOR_TICK_COLOUR = ILI9341_WHITE;
const uint16_t MINOR_TICK_COLOUR = ILI9341_RED;
const uint16_t TICK_TEXT_SIZE = 1;
const uint16_t GAUGE_BOTTOM_OFFSET_ANGLE = 30; // degrees
const uint16_t GAUGE_COLOUR = ILI9341_WHITE;
const uint16_t BACKGROUND_COLOUR = ILI9341_BLACK;

void setup() {
  Serial.begin(9600);
  Serial.println("ILI9341 Test!"); 
 
  // deselect all SPI devices
  pinMode(TFT_CS, OUTPUT);
  digitalWrite(TFT_CS, HIGH);
  tft.begin();
  tft.setRotation(1);
  tft.fillScreen(ILI9341_BLACK);

  // Draw Speedometer & Tachometer
  float speed = 80.5;
  float rpm = 8000.0;

  drawGauge(SPEEDO_CENTRE_X, SPEEDO_CENTRE_Y, GAUGE_RADIUS, NEEDLE_BASE_WIDTH, MAX_SPEED, 20, speed); // dummy speed value
  drawGauge(TACHO_CENTRE_X, TACHO_CENTRE_Y, GAUGE_RADIUS, NEEDLE_BASE_WIDTH, MAX_RPM / 1000, 1, rpm / 1000); // dummy tach value
}


void loop(void) {

}

void drawGauge(int gaugeX, int gaugeY, int gaugeRadius, int needleWidth, int maxValue, int majorTickIncrement, float currentValue) {
  // Draw gauge (no needle)
  tft.fillCircle(gaugeX, gaugeY, gaugeRadius, BACKGROUND_COLOUR);
  tft.drawCircle(gaugeX, gaugeY, gaugeRadius, GAUGE_COLOUR);
  
  int numTicks = 2 * (maxValue / majorTickIncrement);
  numTicks += (maxValue % majorTickIncrement == 0) ? 1 : 2; // Inclusive zero, and add an extra if there's a last minor tick

  for (int i = 0; i < numTicks; i++) {
    int tickVal = (i / 2) * majorTickIncrement;
    bool isMajorTick = ((i % 2) == 0);
    int angle = map(i, 0, numTicks - 1, 90 + GAUGE_BOTTOM_OFFSET_ANGLE, 450 - GAUGE_BOTTOM_OFFSET_ANGLE);
    int length = isMajorTick ? MAJOR_TICK_LENGTH : MINOR_TICK_LENGTH;
    int colour = isMajorTick ? MAJOR_TICK_COLOUR : MINOR_TICK_COLOUR;

    drawTick(gaugeX, gaugeY, gaugeRadius, angle, length, GAUGE_COLOUR, isMajorTick, tickVal);
  }

  // Draw needle with currentValue
  int currentAngle = map(currentValue, 0, maxValue, 90 + GAUGE_BOTTOM_OFFSET_ANGLE, 450 - GAUGE_BOTTOM_OFFSET_ANGLE);
  drawNeedle(gaugeX, gaugeY, gaugeRadius, needleWidth, currentAngle);
  tft.fillCircle(gaugeX, gaugeY, needleWidth + 1, GAUGE_COLOUR);
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
    // Messy, but work-around for off-spaced gauges
    if (labelValue >= 0 && labelValue < 10) {
      tft.setCursor(labelX - 2, labelY - 2);
    } else if (labelValue >= 10 && labelValue < 100) {
      tft.setCursor(labelX - 6, labelY - 3);
    } else {
      tft.setCursor(labelX - 8, labelY - 4);
    }

    tft.print(labelValue);
  }
}

void drawNeedle(int centreX, int centreY, int length, int width, int angle) {
  float rads = angle * PI / 180;

  int outerX = centreX + length * cos(rads);
  int outerY = centreY + length * sin(rads);

  int innerX1 = centreX + (width / 2) * cos(rads + PI / 2);
  int innerY1 = centreY + (width / 2) * sin(rads + PI / 2);
  int innerX2 = centreX + (width / 2) * cos(rads - PI / 2);
  int innerY2 = centreY + (width / 2) * sin(rads - PI / 2);

  // Draw needle as filled triangle
  tft.fillTriangle(outerX, outerY, innerX1, innerY1, innerX2, innerY2, ILI9341_RED);
}
