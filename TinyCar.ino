#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"

#define HEADLIGHTS 3
#define BRAKELIGHTS 4
#define LEFT_SIGNAL_LIGHT 5
#define RIGHT_SIGNAL_LIGHT 6
#define HAZARDS_BUTTON 2
#define GAS_PEDAL_DIGITAL A0 // Testing as digital, will be switched to analog eventually
#define BRAKE_PEDAL_DIGITAL A1 // Testing as digital, will be switched to analog eventually
#define HEADLIGHTS_LOW_SWITCH A4
#define HEADLIGHTS_HIGH_SWITCH A5
#define LEFT_SIGNAL_SWITCH A2
#define RIGHT_SIGNAL_SWITCH A3
#define TFT_DC 9
#define TFT_CS 10
#define TFT_RST 8

#define SIGNAL_TIMING_MS 750
#define PEDAL_TIMING_MS 100

#define PI 3.14159
#define CM_IN_KM 100000

// Car-specific stats - potential FIXME
const float MAX_SPEED = 250.0;
const float IDLE_RPM = 800.0;
const float MAX_RPM = 9000.0;
const float GAS_INCREASE_RPM_RATE = 100.0;
const float BRAKE_DECREASE_RPM_RATE = 150.0;
const float IDLE_DECREASE_RPM_RATE = 50.0;
const float FINAL_DRIVE_RATIO = 3.58;
const float TRANS_RATIOS[] = {3.49, 1.99, 1.45, 1.00, 0.71, 3.99}; // 1, 2, 3, 4, 5, R
const float TIRE_DIAMETER_CM = 60.1;

// Display constants
const uint16_t SCREEN_CENTRE_X = 160;
const uint16_t SCREEN_CENTRE_Y = 120;
const uint16_t GAUGE_RADIUS = 65;
const uint16_t INNER_GAUGE_RADIUS = 30;
const uint16_t SPEEDO_CENTRE_X = SCREEN_CENTRE_X - GAUGE_RADIUS - 20;
const uint16_t SPEEDO_CENTRE_Y = SCREEN_CENTRE_Y - 20;
const uint16_t TACHO_CENTRE_X = SCREEN_CENTRE_X + GAUGE_RADIUS + 20;
const uint16_t TACHO_CENTRE_Y = SCREEN_CENTRE_Y - 20;
const uint16_t MAJOR_TICK_LENGTH = 10;
const uint16_t MINOR_TICK_LENGTH = 5;
const uint16_t NEEDLE_BASE_WIDTH = 6;
const uint16_t TICK_LABEL_OFFSET = 10;
const uint16_t MAJOR_TICK_COLOUR = ILI9341_WHITE;
const uint16_t MINOR_TICK_COLOUR = ILI9341_YELLOW;
const uint16_t TICK_TEXT_SIZE = 1;
const uint16_t GAUGE_BOTTOM_OFFSET_ANGLE = 30; // degrees
const uint16_t GAUGE_COLOUR = ILI9341_WHITE;
const uint16_t BACKGROUND_COLOUR = ILI9341_BLACK;
const uint16_t NEEDLE_COLOUR = ILI9341_RED;

bool hazardsToggledOn = false;
bool leftSignalToggledOn = false;
bool rightSignalToggledOn = false;
bool signalLightsOn = false;
bool hazardButtonState = HIGH;
unsigned long signalTime = 0;
unsigned long pedalTime = 0;
float currentSpeed = 0.0;
float currentRPM = 0.0;
int currentGear = 0; // 0 = 1, 1 = 2, 3, 4, 5, R, N

float oldSpeed = currentSpeed;
float oldRPM = currentRPM;
float oldGear = currentGear - 1;

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);

void setup() {
  // put your setup code here, to run once:
  pinMode(HEADLIGHTS, OUTPUT);
  pinMode(BRAKELIGHTS, OUTPUT);
  pinMode(LEFT_SIGNAL_LIGHT, OUTPUT);
  pinMode(RIGHT_SIGNAL_LIGHT, OUTPUT);
  pinMode(HAZARDS_BUTTON, INPUT_PULLUP);
  pinMode(GAS_PEDAL_DIGITAL, INPUT_PULLUP);
  pinMode(BRAKE_PEDAL_DIGITAL, INPUT_PULLUP);
  pinMode(HEADLIGHTS_LOW_SWITCH, INPUT_PULLUP);
  pinMode(HEADLIGHTS_HIGH_SWITCH, INPUT_PULLUP);
  pinMode(LEFT_SIGNAL_SWITCH, INPUT_PULLUP);
  pinMode(RIGHT_SIGNAL_SWITCH, INPUT_PULLUP);

  Serial.begin(9600);

  // Setup screen
  setupScreen();
}

void loop() {
  handleTurnSignals();
  handleHeadlights();
  if (millis() - pedalTime >= PEDAL_TIMING_MS) {
    handlePedals();
    updateScreen();

    pedalTime = millis();
  }
}

void setupScreen(void) {
  // deselect all SPI devices
  pinMode(TFT_CS, OUTPUT);
  digitalWrite(TFT_CS, HIGH);
  tft.begin();

  tft.setRotation(1);
  tft.fillScreen(BACKGROUND_COLOUR);

  drawGauge(SPEEDO_CENTRE_X, SPEEDO_CENTRE_Y, GAUGE_RADIUS, INNER_GAUGE_RADIUS, NEEDLE_BASE_WIDTH, MAX_SPEED, 20, currentSpeed);
  drawGauge(TACHO_CENTRE_X, TACHO_CENTRE_Y, GAUGE_RADIUS, INNER_GAUGE_RADIUS, NEEDLE_BASE_WIDTH, MAX_RPM / 1000, 1, currentRPM / 1000);

  // Tachometer RPM text
  tft.setTextSize(1);
  tft.setTextColor(ILI9341_ORANGE, BACKGROUND_COLOUR);
  tft.setCursor(TACHO_CENTRE_X - 8, TACHO_CENTRE_Y + 40);
  tft.print("RPM");
  tft.setCursor(TACHO_CENTRE_X - 16, TACHO_CENTRE_Y + 50);
  tft.print("x1000");

  updateScreen();
}

void updateScreen(void) {
  // Update speed and RPM
  if (abs(oldSpeed - currentSpeed) >= 0.2) {
    updateSpeedometer();
    
  }

  if (abs(oldRPM - currentRPM) >= 10 || currentGear != oldGear) {
    updateTachometer();
  }

  // TODO Update fuel, turn signals, hazards, etc.

}

void updateSpeedometer(void) {
  updateGauge(SPEEDO_CENTRE_X, SPEEDO_CENTRE_Y, GAUGE_RADIUS, INNER_GAUGE_RADIUS, NEEDLE_BASE_WIDTH, MAX_SPEED, 20, currentSpeed, oldSpeed);
  
  // Update inner circle
  tft.drawCircle(SPEEDO_CENTRE_X, SPEEDO_CENTRE_Y, INNER_GAUGE_RADIUS, GAUGE_COLOUR);

  char speedStr[4];
  sprintf(speedStr, "%03d", (int)currentSpeed);

  // Centre text
  int16_t x1, y1;
  uint16_t w, h;
  tft.getTextBounds(speedStr, 0, 0, &x1, &y1, &w, &h);

  int textX = SPEEDO_CENTRE_X - 16;
  int textY = SPEEDO_CENTRE_Y - 12;

  tft.fillRect(textX, textY, w, h, BACKGROUND_COLOUR);

  // Draw leading zeroes as greyed out
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_RED, BACKGROUND_COLOUR);
  tft.setCursor(textX, textY);
  
  int numZeroes = 0;
  for (int i = 0; i < 2; i++) {
    if (speedStr[i] == '0') {
      tft.print('0');
      numZeroes++;
    } else {
      break;
    }
  }

  // Draw the actual number in the normal color
  tft.setTextColor(GAUGE_COLOUR, BACKGROUND_COLOUR);
  tft.print(&speedStr[numZeroes]);

  tft.setCursor(textX + 5, textY + 18);
  tft.setTextSize(1);
  tft.print("KM/H");

  oldSpeed = currentSpeed;
}

void updateTachometer(void) {
  updateGauge(TACHO_CENTRE_X, TACHO_CENTRE_Y, GAUGE_RADIUS, INNER_GAUGE_RADIUS, NEEDLE_BASE_WIDTH, MAX_RPM / 1000, 1, currentRPM / 1000, oldRPM / 1000);
  
  // Update inner circle
  tft.drawCircle(TACHO_CENTRE_X, TACHO_CENTRE_Y, INNER_GAUGE_RADIUS, GAUGE_COLOUR);
  
  if (currentGear != oldGear) {
    // 0 = 1, 1 = 2, 3, 4, 5, R, N
    char gearChar;
    if (currentGear >= 0 && currentGear < 5) {
      gearChar = '1' + currentGear;
    } else if (currentGear == 5) {
      gearChar = 'R';
    } else if (currentGear == 6) {
      gearChar = 'N';
    } else {
      gearChar = '?';
    }

    int textX = TACHO_CENTRE_X - 16;
    int textY = TACHO_CENTRE_Y - 12;

    tft.setTextSize(2);
    tft.setTextColor(GAUGE_COLOUR, BACKGROUND_COLOUR);
    tft.setCursor(textX, textY);
    tft.print(' ');
    tft.print(gearChar);

    tft.setCursor(textX + 4, textY + 18);
    tft.setTextSize(1);
    tft.print("GEAR");

    oldGear = currentGear;
  }

  oldRPM = currentRPM;
}

void handleTurnSignals(void) {
  if (!hazardsToggledOn && !leftSignalToggledOn && !rightSignalToggledOn) {
    // Ensures timing resets if a switch/button is toggled on
    signalTime = millis() - SIGNAL_TIMING_MS;
  }

  readSignalInputs();
  analogWrite(LEFT_SIGNAL_LIGHT, (signalLightsOn && (hazardsToggledOn || leftSignalToggledOn)) ? 255 : 0);
  analogWrite(RIGHT_SIGNAL_LIGHT, (signalLightsOn && (hazardsToggledOn || rightSignalToggledOn)) ? 255 : 0);

  if (hazardsToggledOn || leftSignalToggledOn || rightSignalToggledOn) {
    if (millis() - signalTime >= SIGNAL_TIMING_MS) {
      signalLightsOn = !signalLightsOn;
      signalTime = millis();
    }
  }
}

void readSignalInputs(void) {
  // Individual switches
  leftSignalToggledOn = digitalRead(LEFT_SIGNAL_SWITCH) == LOW;
  rightSignalToggledOn = digitalRead(RIGHT_SIGNAL_SWITCH) == LOW;

  // Hazard light button reading (press toggles it)
  bool hazardButtonReading = digitalRead(HAZARDS_BUTTON);

  if (hazardButtonState != hazardButtonReading) {
    hazardButtonState = hazardButtonReading;

    if (hazardButtonState == LOW) {
      hazardsToggledOn = !hazardsToggledOn;
      signalLightsOn = signalLightsOn;
    }
  }
}

void handleHeadlights(void) {
  if (digitalRead(HEADLIGHTS_LOW_SWITCH) == LOW) {
    analogWrite(HEADLIGHTS, 255);
  } else if (digitalRead(HEADLIGHTS_HIGH_SWITCH) == LOW) {
    analogWrite(HEADLIGHTS, 0);
  } else {
    analogWrite(HEADLIGHTS, 128);
  }
}

// TODO: handle more realistically with analog control + transmission
void handlePedals(void) {
  bool gasPressed = digitalRead(GAS_PEDAL_DIGITAL) == LOW;
  bool brakePressed = digitalRead(BRAKE_PEDAL_DIGITAL) == LOW;
  float currentTransRatio = TRANS_RATIOS[currentGear]; // FIXME when transmission is added

  // Update RPM based off pedals
  if (brakePressed) {
    analogWrite(BRAKELIGHTS, 255);
    currentRPM = max(IDLE_RPM, currentRPM - BRAKE_DECREASE_RPM_RATE);
  } else {
    analogWrite(BRAKELIGHTS, 0);

    if (gasPressed) {
      currentRPM = min(MAX_RPM, currentRPM + GAS_INCREASE_RPM_RATE);
    } else { // Speed more gradually decreases
      currentRPM = max(IDLE_RPM, currentRPM - IDLE_DECREASE_RPM_RATE);
    }
  }

  // Speed calculation based off RPM
  float wheelRPM = currentRPM / (currentTransRatio * FINAL_DRIVE_RATIO);
  currentSpeed = (wheelRPM * TIRE_DIAMETER_CM * 60 * PI) / CM_IN_KM;
}

// Display Functions
// Initial drawing of gauge
void drawGauge(int gaugeX, int gaugeY, int outerRadius, int innerRadius, int needleWidth, int maxValue, int majorTickIncrement, float currentValue) {
  // Draw gauge (no needle)
  tft.fillCircle(gaugeX, gaugeY, outerRadius, BACKGROUND_COLOUR);
  tft.drawCircle(gaugeX, gaugeY, outerRadius, GAUGE_COLOUR);
  
  int numTicks = 2 * (maxValue / majorTickIncrement);
  numTicks += (maxValue % majorTickIncrement == 0) ? 1 : 2; // Inclusive zero, and add an extra if there's a last minor tick

  for (int i = 0; i < numTicks; i++) {
    int tickVal = (i / 2) * majorTickIncrement;
    bool isMajorTick = ((i % 2) == 0);
    int angle = map(i, 0, numTicks - 1, 90 + GAUGE_BOTTOM_OFFSET_ANGLE, 450 - GAUGE_BOTTOM_OFFSET_ANGLE);
    int length = isMajorTick ? MAJOR_TICK_LENGTH : MINOR_TICK_LENGTH;
    int colour = isMajorTick ? MAJOR_TICK_COLOUR : MINOR_TICK_COLOUR;

    drawTick(gaugeX, gaugeY, outerRadius, angle, length, colour, isMajorTick, tickVal);
  }

  // Draw needle with currentValue
  int currentAngle = mapFloat(currentValue, 0, maxValue, 90 + GAUGE_BOTTOM_OFFSET_ANGLE, 450 - GAUGE_BOTTOM_OFFSET_ANGLE);
  drawNeedle(gaugeX, gaugeY, outerRadius - 2, innerRadius + 2, needleWidth, NEEDLE_COLOUR, currentAngle);
  

  // Draw inner circle
  tft.drawCircle(gaugeX, gaugeY, innerRadius, GAUGE_COLOUR);
}

void updateGauge(int gaugeX, int gaugeY, int outerRadius, int innerRadius, int needleWidth, int maxValue, int majorTickIncrement, float currentValue, float oldValue) {
  // Clear old needle
  float oldAngle = mapFloat(oldValue, 0, maxValue, 90 + GAUGE_BOTTOM_OFFSET_ANGLE, 450 - GAUGE_BOTTOM_OFFSET_ANGLE);
  drawNeedle(gaugeX, gaugeY, outerRadius - 2, innerRadius + 2, needleWidth, BACKGROUND_COLOUR, oldAngle);

  // Redraw covered part of gauge
  // Round up to greater major tick, minor tick in the middle, and smaller major tick
  int bigMajorTickVal = ceil(currentValue / majorTickIncrement) * majorTickIncrement;
  int smallMajorTickVal = floor(currentValue / majorTickIncrement) * majorTickIncrement;
  float minorTickVal = (bigMajorTickVal + smallMajorTickVal) / 2.0;

  // Ensure the values are within valid range
  if (smallMajorTickVal < 0) {
    smallMajorTickVal = 0;
  }

  float bigMajorTickAngle = mapFloat(bigMajorTickVal, 0, maxValue, 90 + GAUGE_BOTTOM_OFFSET_ANGLE, 450 - GAUGE_BOTTOM_OFFSET_ANGLE);
  float minorTickAngle = mapFloat(minorTickVal, 0, maxValue, 90 + GAUGE_BOTTOM_OFFSET_ANGLE, 450 - GAUGE_BOTTOM_OFFSET_ANGLE);
  float smallMajorTickAngle = mapFloat(smallMajorTickVal, 0, maxValue, 90 + GAUGE_BOTTOM_OFFSET_ANGLE, 450 - GAUGE_BOTTOM_OFFSET_ANGLE);

  // Check biggest major tick val exists (might end on a minor)
  if (bigMajorTickVal <= maxValue) { // Redraw it
    drawTick(gaugeX, gaugeY, outerRadius, bigMajorTickAngle, MAJOR_TICK_LENGTH, MAJOR_TICK_COLOUR, true, bigMajorTickVal);
  }

  drawTick(gaugeX, gaugeY, outerRadius, minorTickAngle, MINOR_TICK_LENGTH, MINOR_TICK_COLOUR, false, minorTickVal);
  drawTick(gaugeX, gaugeY, outerRadius, smallMajorTickAngle, MAJOR_TICK_LENGTH, MAJOR_TICK_COLOUR, true, smallMajorTickVal);

  // Draw needle with currentValue
  float currentAngle = mapFloat(currentValue, 0, maxValue, 90 + GAUGE_BOTTOM_OFFSET_ANGLE, 450 - GAUGE_BOTTOM_OFFSET_ANGLE);
  drawNeedle(gaugeX, gaugeY, outerRadius - 2, innerRadius + 2, needleWidth, NEEDLE_COLOUR, currentAngle);
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

void drawNeedle(int centreX, int centreY, int outerRadius, int innerRadius, int width, int colour, float angle) {
  float rads = angle * PI / 180;

  int innerX = centreX + innerRadius * cos(rads);
  int innerY = centreY + innerRadius * sin(rads);
  int outerX = centreX + outerRadius * cos(rads);
  int outerY = centreY + outerRadius * sin(rads);

  tft.drawLine(innerX, innerY, outerX, outerY, colour);
}

// Old function for drawing fancier needle, but arduino struggles with it
void drawTriangleNeedle(int centreX, int centreY, int length, int width, int colour, float angle) {
  float rads = angle * PI / 180;

  int outerX = centreX + length * cos(rads);
  int outerY = centreY + length * sin(rads);

  int innerX1 = centreX + (width / 2) * cos(rads + PI / 2);
  int innerY1 = centreY + (width / 2) * sin(rads + PI / 2);
  int innerX2 = centreX + (width / 2) * cos(rads - PI / 2);
  int innerY2 = centreY + (width / 2) * sin(rads - PI / 2);

  tft.fillTriangle(outerX, outerY, innerX1, innerY1, innerX2, innerY2, colour);
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
