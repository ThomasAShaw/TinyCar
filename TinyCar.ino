#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"

// Pin Definitions
#define HEADLIGHTS 3
#define BRAKELIGHTS 4
#define LEFT_SIGNAL_LIGHT 5
#define RIGHT_SIGNAL_LIGHT 6
#define HAZARDS_BUTTON 2
#define GAS_PEDAL_DIGITAL A0 // Testing as digital, will be switched to analog eventually
#define BRAKE_PEDAL_DIGITAL A1 // Testing as digital, will be switched to analog eventually
#define HEADLIGHTS_OFF_SWITCH A4
#define HEADLIGHTS_HIGH_SWITCH A5
#define LEFT_SIGNAL_SWITCH A2
#define RIGHT_SIGNAL_SWITCH A3
#define TFT_DC 9
#define TFT_CS 10
#define TFT_RST 8

// Timing Constants
#define SIGNAL_TIMING_MS 750
#define PEDAL_TIMING_MS 100
#define FUEL_TIMING_MS 1000
#define ODOMETER_TIMING_MS 1000
#define DEBOUNCE_DELAY_MS 50

// Other Useful Constants
#define PI 3.14159
#define CM_IN_KM 100000

// Car Characteristics/Constants (change me to change how the vehicle behaves)
const float MAX_SPEED = 250.0;
const float IDLE_RPM = 800.0;
const float MAX_RPM = 9000.0;
const float GAS_INCREASE_RPM_RATE = 100.0;
const float BRAKE_DECREASE_RPM_RATE = 150.0;
const float IDLE_DECREASE_RPM_RATE = 50.0;
const float FINAL_DRIVE_RATIO = 3.58;
const float TRANS_RATIOS[] = {3.49, 1.99, 1.45, 1.00, 0.71, 3.99, 1.0}; // 1, 2, 3, 4, 5, R, N
const float TIRE_DIAMETER_CM = 60.1;
const float RPM_FUEL_FACTOR = 100.0 / (60.0 * 30.0 * 1000.0 * 1000.0); // at 1000RPM, takes 30 mins to deplete fuel.
const float SPEED_DISTANCE_MULTIPLER = 10.0; // Odometer goes up 10x faster than realistic to show more visuals
const float RPM_UPSHIFT[] = {2500, 3000, 3500, 4000, 4500, MAX_RPM * 2, IDLE_RPM + 1}; // RPM thresholds to upshift
const float RPM_DOWNSHIFT[] = {1200, 1500, 2000, 2500, 3000}; // RPM thresholds to downshift

// Display Size Constants
const uint16_t GAUGE_OUTER_RADIUS = 65;
const uint16_t GAUGE_CENTRE_RADIUS = 28;
const uint16_t GAUGE_BOTTOM_OFFSET_ANGLE = 30; // degrees
const uint16_t SIGNAL_SIZE = 10;  // Size of the arrow signals
const uint16_t FUEL_BAR_SIZE = 6;
const uint16_t TICK_TEXT_SIZE = 1;
const uint16_t TICK_LENGTH = 3;
const uint16_t TICK_LABEL_OFFSET = 10;

// Display Position Constants
const uint16_t SCREEN_CENTRE_X = 160;
const uint16_t SCREEN_CENTRE_Y = 120;
const uint16_t SPEEDO_CENTRE_X = SCREEN_CENTRE_X - GAUGE_OUTER_RADIUS - 20;
const uint16_t SPEEDO_CENTRE_Y = SCREEN_CENTRE_Y - 20;
const uint16_t TACHO_CENTRE_X = SCREEN_CENTRE_X + GAUGE_OUTER_RADIUS + 20;
const uint16_t TACHO_CENTRE_Y = SCREEN_CENTRE_Y - 20;
const uint16_t HAZARD_SIGNAL_X = SCREEN_CENTRE_X;
const uint16_t HAZARD_SIGNAL_Y = SCREEN_CENTRE_Y - 73;
const uint16_t LEFT_SIGNAL_X = SCREEN_CENTRE_X - 2 * (SIGNAL_SIZE + 6);
const uint16_t LEFT_SIGNAL_Y = HAZARD_SIGNAL_Y;
const uint16_t RIGHT_SIGNAL_X = SCREEN_CENTRE_X + 2 * (SIGNAL_SIZE + 6);
const uint16_t RIGHT_SIGNAL_Y = HAZARD_SIGNAL_Y;
const uint16_t FUEL_BAR_X = SCREEN_CENTRE_X - 2 * (FUEL_BAR_SIZE + 2) - FUEL_BAR_SIZE / 2;
const uint16_t FUEL_BAR_Y = SCREEN_CENTRE_Y - 54;
const uint16_t FUEL_ICON_X = SCREEN_CENTRE_X - 7;
const uint16_t FUEL_ICON_Y = FUEL_BAR_Y + 10;
const uint16_t LIGHT_ICON_X = SCREEN_CENTRE_X;
const uint16_t LIGHT_ICON_Y = SCREEN_CENTRE_Y - 8;
const uint16_t ODOMETER_X = SCREEN_CENTRE_X - 16;
const uint16_t ODOMETER_Y = SCREEN_CENTRE_Y + 14;

// Display Colour Constants
const uint16_t GAUGE_PRIMARY_COLOUR = 0xd6da;
const uint16_t GAUGE_SECONDARY_COLOUR = 0x071f;
const uint16_t GAUGE_MIDDLE_COLOUR = 0x00a7;
const uint16_t BACKGROUND_COLOUR = 0x0003;
const uint16_t NEEDLE_COLOUR = 0xe008;
const uint16_t MIDDLE_BACKGROUND_COLOUR = 0x00a7;
const uint16_t TURN_SIGNAL_OFF_COLOUR = MIDDLE_BACKGROUND_COLOUR;
const uint16_t TURN_SIGNAL_ON_COLOUR = 0x16c6;
const uint16_t HAZARD_SIGNAL_OFF_COLOUR = MIDDLE_BACKGROUND_COLOUR;
const uint16_t HAZARD_SIGNAL_ON_COLOUR = 0xe008;
const uint16_t RPM_TEXT_COLOUR = 0xfc60;
const uint16_t GREYED_OUT_COLOUR = 0x016f;
const uint16_t HEADLIGHTS_ON_COLOUR = 0x07e0;
const uint16_t HIGHBEAMS_ON_COLOUR = 0x071f;

enum HeadlightState {
  HEADLIGHTS_OFF,
  HEADLIGHTS_LOW,
  HEADLIGHTS_HIGH
};

// Global Variables
HeadlightState currentHeadlightState = HEADLIGHTS_OFF;
bool hazardsToggledOn = false;
bool signalLightsOn = false;
bool hazardButtonState = HIGH;
unsigned long hazardDebounceTime = 0;
unsigned long signalTime = 0;
unsigned long pedalTime = 0;
unsigned long fuelTime = 0;
unsigned long odometerTime = 0;

float currentSpeed = 0.0;
float currentRPM = 0.0;
int currentGear = 6; // 0 = 1, 1 = 2, 3, 4, 5, 5 = R, 6 = N
float currentFuelLevel = 100.0;
float currentOdometerKM = 0.0;

float oldSpeed = currentSpeed;
float oldRPM = currentRPM;
float oldGear = currentGear - 1;

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);

void setup() {
  setupPins();
  Serial.begin(9600);
  // Setup screen
  setupDisplay();
  drawInitialDisplay();
}

void loop() {
  handleControls();
}

/**
 * Sets up the pins for the simulation.
 */
void setupPins(void) {
  pinMode(HEADLIGHTS, OUTPUT);
  pinMode(BRAKELIGHTS, OUTPUT);
  pinMode(LEFT_SIGNAL_LIGHT, OUTPUT);
  pinMode(RIGHT_SIGNAL_LIGHT, OUTPUT);
  pinMode(HAZARDS_BUTTON, INPUT_PULLUP);
  pinMode(GAS_PEDAL_DIGITAL, INPUT_PULLUP);
  pinMode(BRAKE_PEDAL_DIGITAL, INPUT_PULLUP);
  pinMode(HEADLIGHTS_OFF_SWITCH, INPUT_PULLUP);
  pinMode(HEADLIGHTS_HIGH_SWITCH, INPUT_PULLUP);
  pinMode(LEFT_SIGNAL_SWITCH, INPUT_PULLUP);
  pinMode(RIGHT_SIGNAL_SWITCH, INPUT_PULLUP);
}

/**
 * Initializes the display setup.
 */
void setupDisplay(void) {
  // deselect all SPI devices
  pinMode(TFT_CS, OUTPUT);
  digitalWrite(TFT_CS, HIGH);
  tft.begin();
}

/**
 * Draws the initial state of the display, setting up orientations and background,
 * and drawing initial static elements such as gauges and icons.
 */
void drawInitialDisplay(void) {
  tft.setRotation(1);
  tft.fillScreen(BACKGROUND_COLOUR);

  // Centre background for icons.
  tft.fillRect(FUEL_BAR_X - 16, FUEL_BAR_Y - 4, 72, 34, MIDDLE_BACKGROUND_COLOUR);
  tft.fillRect(ODOMETER_X - 42, ODOMETER_Y - 5, 116, 26, MIDDLE_BACKGROUND_COLOUR);
  tft.fillRect(LIGHT_ICON_X - 24, LIGHT_ICON_Y - 13, 60, 27, MIDDLE_BACKGROUND_COLOUR);

  drawGauge(SPEEDO_CENTRE_X, SPEEDO_CENTRE_Y, GAUGE_OUTER_RADIUS, GAUGE_CENTRE_RADIUS, MAX_SPEED, 20, currentSpeed);
  drawGauge(TACHO_CENTRE_X, TACHO_CENTRE_Y, GAUGE_OUTER_RADIUS, GAUGE_CENTRE_RADIUS, MAX_RPM / 1000, 1, currentRPM / 1000);
  updateSpeedometerDisplay(currentSpeed, currentSpeed); // Force update
  updateTachometerDisplay(currentRPM, currentRPM, currentGear, currentGear); // Force update

  // Tachometer RPM text
  tft.setTextSize(1);
  tft.setTextColor(RPM_TEXT_COLOUR, BACKGROUND_COLOUR);
  tft.setCursor(TACHO_CENTRE_X - 8, TACHO_CENTRE_Y + 35);
  tft.print("RPM");
  tft.setCursor(TACHO_CENTRE_X - 16, TACHO_CENTRE_Y + 45);
  tft.print("x1000");

  updateHazardsDisplay(false);
  updateTurnSignalDisplay(false, false);

  // Fuel Icon + E & F
  tft.setTextSize(1);
  tft.setTextColor(GAUGE_PRIMARY_COLOUR, MIDDLE_BACKGROUND_COLOUR);
  tft.setCursor(FUEL_BAR_X, FUEL_BAR_Y + FUEL_BAR_SIZE + 4);
  tft.print("E");
  tft.setCursor(FUEL_BAR_X + 4 * (FUEL_BAR_SIZE + 2), FUEL_BAR_Y + FUEL_BAR_SIZE + 4);
  tft.print("F");
  drawFuelPump(FUEL_ICON_X, FUEL_ICON_Y, GAUGE_PRIMARY_COLOUR, MIDDLE_BACKGROUND_COLOUR);

  // Odometer KM
  tft.setTextSize(1);
  tft.setTextColor(GAUGE_PRIMARY_COLOUR, MIDDLE_BACKGROUND_COLOUR);
  tft.setCursor(ODOMETER_X + 12, ODOMETER_Y + 10);
  tft.print("KM");
  updateOdometerDisplay(0);
}

/**
 * Handles the periodic update of control inputs such as pedals, turn signals, and headlights.
 * It checks for state changes and updates the display accordingly.
 */
void handleControls(void) {
  handleTurnSignals();
  handleHeadlights();
  if (millis() - pedalTime >= PEDAL_TIMING_MS) {
    handlePedals();
    if (abs(oldSpeed - currentSpeed) >= 0.2) {
      updateSpeedometerDisplay(currentSpeed, oldSpeed);
      oldSpeed = currentSpeed;
    }

    if (abs(oldRPM - currentRPM) >= 10 || currentGear != oldGear) {
      updateTachometerDisplay(currentRPM, oldRPM, currentGear, oldGear);
      oldRPM = currentRPM;
      oldGear = currentGear;
    }

    pedalTime = millis();
  }

  if (millis() - fuelTime >= FUEL_TIMING_MS) {
    updateFuelLevel();
  }

  if (millis() - odometerTime >= ODOMETER_TIMING_MS) {
    updateOdometer();
  }
}

/**
 * Handles the updating of the fuel level based on elapsed time and RPM, 
 * adjusting the visual fuel bar display as necessary.
 */
void updateFuelLevel(void) {
    unsigned long currentTime = millis();
    float elapsedTime = (currentTime - fuelTime);

    // Calculate fuel consumption based on current RPM
    float fuelConsumed = currentRPM * RPM_FUEL_FACTOR * elapsedTime;
    float newFuelLevel = currentFuelLevel - fuelConsumed;

    if (newFuelLevel < 0) {
    newFuelLevel = 0;
    }

    if (newFuelLevel > 100) {
      newFuelLevel = 100;
    }

    if ((int)(newFuelLevel / 20) != (int)(currentFuelLevel / 20)) {
      updateFuelDisplay(int(currentFuelLevel / 20));
    }
    
    currentFuelLevel = newFuelLevel;
    fuelTime = currentTime;
}

/**
 * Updates the odometer based on the speed and elapsed time, 
 * calculating the new distance traveled and updating the display as needed.
 */
void updateOdometer(void) {
    unsigned long currentTime = millis();
    float elapsedTime = (currentTime - odometerTime);

    // Calculate distance travelled based on current speed
    float distanceTravelled = currentSpeed * SPEED_DISTANCE_MULTIPLER * elapsedTime / (60.0 * 60.0 * 1000.0);
    float newOdometerKM = currentOdometerKM + distanceTravelled;

    if (newOdometerKM < 0) {
    newOdometerKM = 0;
    }

    if (newOdometerKM > 100) {
      newOdometerKM = 100;
    }

    if ((unsigned long)(newOdometerKM) != (unsigned long)(currentOdometerKM)) {
      updateOdometerDisplay((unsigned long)(newOdometerKM));
    }
    
    currentOdometerKM = newOdometerKM;
    odometerTime = currentTime;
}

/**
 * Processes inputs from the turn signal switches and the hazard light button, 
 * updating the display and lights accordingly.
 */
void handleTurnSignals(void) {
  // Individual switches
  bool leftSignalSwitchOn = digitalRead(LEFT_SIGNAL_SWITCH) == LOW;
  bool rightSignalSwitchOn = digitalRead(RIGHT_SIGNAL_SWITCH) == LOW;

  // Hazard light button reading (press toggles it)
  bool hazardButtonReading = digitalRead(HAZARDS_BUTTON);

  // Debouncing hazard button and using it as a toggle
  if (hazardButtonState != hazardButtonReading && (millis() - hazardDebounceTime > DEBOUNCE_DELAY_MS)) {
    hazardDebounceTime = millis();
    hazardButtonState = hazardButtonReading;

    if (hazardButtonState == LOW) {
      hazardsToggledOn = !hazardsToggledOn;
      updateHazardsDisplay(hazardsToggledOn);
    }
  }

  bool shouldToggleLights = (hazardsToggledOn || leftSignalSwitchOn || rightSignalSwitchOn);

  if (shouldToggleLights) {
    if (millis() - signalTime >= SIGNAL_TIMING_MS) {
    signalTime = millis();
    signalLightsOn = !signalLightsOn;

    analogWrite(LEFT_SIGNAL_LIGHT, (signalLightsOn && (hazardsToggledOn || leftSignalSwitchOn)) ? 255 : 0);
    analogWrite(RIGHT_SIGNAL_LIGHT, (signalLightsOn && (hazardsToggledOn || rightSignalSwitchOn)) ? 255 : 0);
    updateTurnSignalDisplay(signalLightsOn && (hazardsToggledOn || leftSignalSwitchOn), signalLightsOn && (hazardsToggledOn || rightSignalSwitchOn));
    }
  } else if (signalLightsOn) { // Should turn lights off (as no switches are active now)
    // Ensures timing resets if a switch/button is toggled on at some point in future
    signalLightsOn = false;
    signalTime = millis() - SIGNAL_TIMING_MS;

    analogWrite(LEFT_SIGNAL_LIGHT, 0);
    analogWrite(RIGHT_SIGNAL_LIGHT, 0);
    updateTurnSignalDisplay(false, false);
  }
}

/**
 * Handles the headlights based on input from the headlight switches,
 * updating the display and lights accordingly.
 */
void handleHeadlights(void) {
  HeadlightState newHeadlightState;

  if (digitalRead(HEADLIGHTS_HIGH_SWITCH) == LOW) {
    newHeadlightState = HEADLIGHTS_HIGH;
  } else if (digitalRead(HEADLIGHTS_OFF_SWITCH) == LOW) {
    newHeadlightState = HEADLIGHTS_OFF;
  } else {
    newHeadlightState = HEADLIGHTS_LOW;
  }

  if (newHeadlightState != currentHeadlightState) {
    switch(newHeadlightState) {
      case(HEADLIGHTS_OFF):
        drawHeadlights(LIGHT_ICON_X, LIGHT_ICON_Y, GREYED_OUT_COLOUR, MIDDLE_BACKGROUND_COLOUR);
        analogWrite(HEADLIGHTS, 0);
        break;
      case(HEADLIGHTS_LOW):
        drawHeadlights(LIGHT_ICON_X, LIGHT_ICON_Y, HEADLIGHTS_ON_COLOUR, MIDDLE_BACKGROUND_COLOUR);
        analogWrite(HEADLIGHTS, 128);
        break;
      case(HEADLIGHTS_HIGH):
        drawHighBeams(LIGHT_ICON_X, LIGHT_ICON_Y, HIGHBEAMS_ON_COLOUR, MIDDLE_BACKGROUND_COLOUR);
        analogWrite(HEADLIGHTS, 255);
        break;
    }

    currentHeadlightState = newHeadlightState;
  }
}

/*
* Handles the gas and brake pedals, 
* responding accordingly by updating RPM and speed.
*/
void handlePedals(void) {
  updateGear();

  bool gasPressed = digitalRead(GAS_PEDAL_DIGITAL) == LOW;
  bool brakePressed = digitalRead(BRAKE_PEDAL_DIGITAL) == LOW;
  float currentTransRatio = TRANS_RATIOS[currentGear];

  // Update RPM based off pedals
  if (brakePressed) {
    analogWrite(BRAKELIGHTS, 255);
    currentRPM = max(IDLE_RPM, currentRPM - BRAKE_DECREASE_RPM_RATE);

    if (currentRPM == IDLE_RPM) {
      currentGear = 6;
      currentSpeed -= 0.85;
      return;
    }
  } else {
    analogWrite(BRAKELIGHTS, 0);

    if (gasPressed) {
      currentRPM = min(MAX_RPM, currentRPM + GAS_INCREASE_RPM_RATE);
    } else { // Speed more gradually decreases
      currentRPM = max(IDLE_RPM, currentRPM - IDLE_DECREASE_RPM_RATE);
    }
  }

  // Speed calculation based off RPM, ensuring car not in neutral
  if (currentGear != 6) {
    float wheelRPM = currentRPM / (currentTransRatio * FINAL_DRIVE_RATIO);
    float calculatedSpeed = (wheelRPM * TIRE_DIAMETER_CM * 60 * PI) / CM_IN_KM;

    if (2.0 * currentSpeed < calculatedSpeed) {
      currentSpeed = 0.55 * calculatedSpeed;
    } else {
      currentSpeed = calculatedSpeed;
    }
  } else {
    // Speed decay in idle
    currentSpeed -= 0.3;
  }
  
  if(currentSpeed >= MAX_SPEED) currentSpeed = MAX_SPEED;
  if (currentSpeed < 0) currentSpeed = 0;
}

/**
 * Simulates a simple automatic transmission by shifting gears based on the engine RPM.
 */
void updateGear() {
  // Increase gear
  if (currentGear < 4 && currentRPM > RPM_UPSHIFT[currentGear]) {
    float rpmAdjustmentFactor = TRANS_RATIOS[currentGear] / TRANS_RATIOS[currentGear + 1];
    currentRPM /= rpmAdjustmentFactor; // Adjust RPM based on the change in gear ratio
    currentGear++;
  }
  
  // Decrease gear
  if (currentGear < 5 && currentRPM < RPM_DOWNSHIFT[currentGear] && currentGear > 0) {
    float rpmAdjustmentFactor = TRANS_RATIOS[currentGear] / TRANS_RATIOS[currentGear - 1];
    currentRPM /= rpmAdjustmentFactor; // Adjust RPM based on the change in gear ratio
    currentGear--;
  }

  // Special case for upshifting from neutral
  if (currentGear == 6 && currentRPM > RPM_UPSHIFT[6]) {
    currentGear = 0;
  }
}

// ==================== Display Functions Specific for Car ====================

/**
 * Updates the tachometer on the display.
 * @param currentRPM The current RPM of the vehicle
 * @param oldRPM The RPM of the vehicle the last time this function was called.
 * @param currentGear The current gear of the vehicle
 * @param oldGear The gear of the vehicle the last time this function was called.
 */
void updateTachometerDisplay(float currentRPM, float oldRPM, float currentGear, float oldGear) {
  updateGauge(TACHO_CENTRE_X, TACHO_CENTRE_Y, GAUGE_OUTER_RADIUS, GAUGE_CENTRE_RADIUS, MAX_RPM / 1000, 1, currentRPM / 1000, oldRPM / 1000);
  
  // Update inner circle
  tft.drawCircle(TACHO_CENTRE_X, TACHO_CENTRE_Y, GAUGE_CENTRE_RADIUS, GAUGE_PRIMARY_COLOUR);
  
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
    tft.setTextColor(GAUGE_PRIMARY_COLOUR, GAUGE_MIDDLE_COLOUR);
    tft.setCursor(textX, textY);
    tft.print(' ');
    tft.print(gearChar);

    tft.setCursor(textX + 4, textY + 18);
    tft.setTextSize(1);
    tft.print("GEAR");
  }
}

/**
 * Updates the speedometer on the display.
 * @param currentSpeed The current speed of the vehicle
 * @param oldSpeed The speed of the vehicle the last time this function was called.
 */
void updateSpeedometerDisplay(float currentSpeed, float oldSpeed) {
  updateGauge(SPEEDO_CENTRE_X, SPEEDO_CENTRE_Y, GAUGE_OUTER_RADIUS, GAUGE_CENTRE_RADIUS, MAX_SPEED, 20, currentSpeed, oldSpeed);
  
  // Update inner circle
  tft.drawCircle(SPEEDO_CENTRE_X, SPEEDO_CENTRE_Y, GAUGE_CENTRE_RADIUS, GAUGE_PRIMARY_COLOUR);

  char speedStr[4];
  sprintf(speedStr, "%03d", (int)currentSpeed);

  // Centre text
  int16_t x1, y1;
  uint16_t w, h;
  tft.getTextBounds(speedStr, 0, 0, &x1, &y1, &w, &h);

  int textX = SPEEDO_CENTRE_X - 16;
  int textY = SPEEDO_CENTRE_Y - 12;

  tft.fillRect(textX, textY, w, h, GAUGE_MIDDLE_COLOUR);

  // Draw leading zeroes as greyed out
  tft.setTextSize(2);
  tft.setTextColor(GREYED_OUT_COLOUR, GAUGE_MIDDLE_COLOUR);
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

  // Draw the actual number in the normal colour
  tft.setTextColor(GAUGE_PRIMARY_COLOUR, GAUGE_MIDDLE_COLOUR);
  tft.print(&speedStr[numZeroes]);

  tft.setCursor(textX + 5, textY + 18);
  tft.setTextSize(1);
  tft.print("KM/H");
}

/**
 * Updates the fuel bar on the display.
 * @param fuelLevel The current level of fuel, scaled from 0 (empty) to 5 (full).
 */
void updateFuelDisplay(int fuelLevel) {
  if (fuelLevel > 5) fuelLevel = 5;
  if (fuelLevel < 0) fuelLevel = 0;

  for (int i = 0; i < 5; i++) {
    if (fuelLevel > i) {
      tft.fillRect(FUEL_BAR_X + i * (FUEL_BAR_SIZE + 2), FUEL_BAR_Y, FUEL_BAR_SIZE, FUEL_BAR_SIZE, GAUGE_PRIMARY_COLOUR);
    } else {
      tft.fillRect(FUEL_BAR_X + i * (FUEL_BAR_SIZE + 2), FUEL_BAR_Y, FUEL_BAR_SIZE, FUEL_BAR_SIZE, MIDDLE_BACKGROUND_COLOUR);
      tft.drawRect(FUEL_BAR_X + i * (FUEL_BAR_SIZE + 2), FUEL_BAR_Y, FUEL_BAR_SIZE, FUEL_BAR_SIZE, GAUGE_PRIMARY_COLOUR);
    }
  }
}

/**
 * Updates the hazards icon on the display.
 * @param toggledOn Boolean indicating if the hazards are currently toggled on.
 */
void updateHazardsDisplay(bool toggledOn) {
  tft.drawTriangle(HAZARD_SIGNAL_X, HAZARD_SIGNAL_Y - 4, HAZARD_SIGNAL_X - 4, HAZARD_SIGNAL_Y + 4, HAZARD_SIGNAL_X + 4, HAZARD_SIGNAL_Y + 4, toggledOn ? HAZARD_SIGNAL_ON_COLOUR : HAZARD_SIGNAL_OFF_COLOUR);
  tft.drawTriangle(HAZARD_SIGNAL_X, HAZARD_SIGNAL_Y - 8, HAZARD_SIGNAL_X - 7, HAZARD_SIGNAL_Y + 6, HAZARD_SIGNAL_X + 7, HAZARD_SIGNAL_Y + 6, toggledOn ? HAZARD_SIGNAL_ON_COLOUR : HAZARD_SIGNAL_OFF_COLOUR);
}

/**
 * Updates the odometer on the display.
 * @param odometerReading The number the odometer will read on the display.
 */
void updateOdometerDisplay(unsigned long odometerReading) {
  // Clamp to displayable range
  if (odometerReading > 999999) {
    odometerReading = 999999;
  }

  char kmStr[7];
  sprintf(kmStr, "%06lu", odometerReading);

  // Draw leading zeroes as greyed out
  tft.setTextSize(1);
  tft.setTextColor(GREYED_OUT_COLOUR, MIDDLE_BACKGROUND_COLOUR);
  tft.setCursor(ODOMETER_X, ODOMETER_Y);
  
  int numZeroes = 0;
  for (int i = 0; i < 6; i++) {
    if (kmStr[i] == '0') {
      tft.print('0');
      numZeroes++;
    } else {
      break;
    }
  }

  // Draw the actual number in the normal colour
  tft.setTextColor(GAUGE_PRIMARY_COLOUR, MIDDLE_BACKGROUND_COLOUR);
  tft.print(&kmStr[numZeroes]);
}

/**
 * Updates the turn signal icons on the display.
 * @param leftSignalOn Boolean indicating if the left turn signal is toggled on.
 * @param rightSignalOn Boolean indicating if the right turn signal is toggled on.
 */
void updateTurnSignalDisplay(bool leftSignalOn, bool rightSignalOn) {
  drawArrow(LEFT_SIGNAL_X, LEFT_SIGNAL_Y, SIGNAL_SIZE, leftSignalOn ? TURN_SIGNAL_ON_COLOUR : TURN_SIGNAL_OFF_COLOUR, true);
  drawArrow(RIGHT_SIGNAL_X, RIGHT_SIGNAL_Y, SIGNAL_SIZE, rightSignalOn ? TURN_SIGNAL_ON_COLOUR : TURN_SIGNAL_OFF_COLOUR, false);
}

// ==================== Display Helper Functions ====================

/**
 * Draws a new gauge with ticks and a needle indicating the current value.
 * @param gaugeX The x-coordinate of the gauge centre.
 * @param gaugeY The y-coordinate of the gauge centre.
 * @param outerRadius The outer radius of the entire gauge.
 * @param centreRadius The inner radius of the central circle.
 * @param maxValue The maximum value the gauge can display.
 * @param majorTickIncrement How often a major tick is placed.
 * @param currentValue The current value to be shown by the needle.
 */
void drawGauge(int gaugeX, int gaugeY, int outerRadius, int centreRadius, int maxValue, int majorTickIncrement, float currentValue) {
  // Draw gauge (no needle)
  tft.fillCircle(gaugeX, gaugeY, outerRadius, GAUGE_SECONDARY_COLOUR);
  tft.fillCircle(gaugeX, gaugeY, outerRadius - 2, BACKGROUND_COLOUR);
  tft.fillCircle(gaugeX, gaugeY, outerRadius - 4, GAUGE_PRIMARY_COLOUR);
  tft.fillCircle(gaugeX, gaugeY, outerRadius - 6, BACKGROUND_COLOUR);
  tft.fillCircle(gaugeX, gaugeY, centreRadius, GAUGE_PRIMARY_COLOUR);
  tft.fillCircle(gaugeX, gaugeY, centreRadius - 2, GAUGE_MIDDLE_COLOUR);
  tft.setTextColor(GAUGE_PRIMARY_COLOUR, BACKGROUND_COLOUR);
  int numTicks = 2 * (maxValue / majorTickIncrement);
  numTicks += (maxValue % majorTickIncrement == 0) ? 1 : 2; // Inclusive zero, and add an extra if there's a last minor tick

  int innerRadius = outerRadius - 7; // Radius of circle for ticks

  for (int i = 0; i < numTicks; i++) {
    int tickVal = (i / 2) * majorTickIncrement;
    bool isMajorTick = ((i % 2) == 0);
    float angle = mapFloat(i, 0, numTicks - 1, 90 + GAUGE_BOTTOM_OFFSET_ANGLE, 450 - GAUGE_BOTTOM_OFFSET_ANGLE);
    int colour = isMajorTick ? GAUGE_SECONDARY_COLOUR : GAUGE_PRIMARY_COLOUR;

    drawTick(gaugeX, gaugeY, innerRadius, angle, TICK_LENGTH, colour, isMajorTick, TICK_LABEL_OFFSET, tickVal);
  }

  // Draw needle with currentValue
  int currentAngle = mapFloat(currentValue, 0, maxValue, 90 + GAUGE_BOTTOM_OFFSET_ANGLE, 450 - GAUGE_BOTTOM_OFFSET_ANGLE);
  drawNeedle(gaugeX, gaugeY, innerRadius - 4, centreRadius + 2, NEEDLE_COLOUR, currentAngle);
}

/**
 * Updates the display of a gauge, used when the value changes and partial redrawing is needed.
 * @param gaugeX The x-coordinate of the gauge centre.
 * @param gaugeY The y-coordinate of the gauge centre.
 * @param outerRadius The outer radius of the entire gauge.
 * @param centreRadius The inner radius of the central circle.
 * @param maxValue The maximum value of the gauge.
 * @param majorTickIncrement The value increment between major ticks.
 * @param currentValue The new value to display.
 * @param oldValue The previous value, used to minimize redraw areas.
 */
void updateGauge(int gaugeX, int gaugeY, int outerRadius, int centreRadius, int maxValue, int majorTickIncrement, float currentValue, float oldValue) {
  // Clear old needle
  int innerRadius = outerRadius - 7; // Radius of circle for ticks
  float oldAngle = mapFloat(oldValue, 0, maxValue, 90 + GAUGE_BOTTOM_OFFSET_ANGLE, 450 - GAUGE_BOTTOM_OFFSET_ANGLE);
  drawNeedle(gaugeX, gaugeY, innerRadius - 4, centreRadius + 2, BACKGROUND_COLOUR, oldAngle);
  tft.setTextColor(GAUGE_PRIMARY_COLOUR, BACKGROUND_COLOUR);

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
    drawTick(gaugeX, gaugeY, innerRadius, bigMajorTickAngle, TICK_LENGTH, GAUGE_SECONDARY_COLOUR, true, TICK_LABEL_OFFSET, bigMajorTickVal);
  }

  drawTick(gaugeX, gaugeY, innerRadius, minorTickAngle, TICK_LENGTH, GAUGE_PRIMARY_COLOUR, false, TICK_LABEL_OFFSET, minorTickVal);
  drawTick(gaugeX, gaugeY, innerRadius, smallMajorTickAngle, TICK_LENGTH, GAUGE_SECONDARY_COLOUR, true, TICK_LABEL_OFFSET, smallMajorTickVal);

  // Draw needle with currentValue
  float currentAngle = mapFloat(currentValue, 0, maxValue, 90 + GAUGE_BOTTOM_OFFSET_ANGLE, 450 - GAUGE_BOTTOM_OFFSET_ANGLE);
  drawNeedle(gaugeX, gaugeY, innerRadius - 4, centreRadius + 2, NEEDLE_COLOUR, currentAngle);
}

/**
 * Draws a single tick mark on a gauge.
 * @param centreX The x-coordinate of the gauge centre.
 * @param centreY The y-coordinate of the gauge centre.
 * @param radius The radius where the tick mark is placed.
 * @param angle The angle in degrees for the tick mark.
 * @param length The length of the tick mark.
 * @param colour The colour of the tick mark.
 * @param showLabel Boolean indicating if a label should be shown next to the tick.
 * @param labelValue The value to display in the label, if shown.
 */
void drawTick(int centreX, int centreY, int radius, float angle, int length, int colour, bool showLabel, int labelOffset, int labelValue) {
  float rads = angle * PI / 180;
  int outerX = centreX + radius * cos(rads);
  int outerY = centreY + radius * sin(rads);
  int innerX = centreX + (radius - length) * cos(rads);
  int innerY = centreY + (radius - length) * sin(rads);
  
  tft.drawLine(innerX, innerY, outerX, outerY, colour);

  if (showLabel) {
    tft.setTextSize(TICK_TEXT_SIZE);
    int labelX = innerX - labelOffset * cos(rads);
    int labelY = innerY - labelOffset * sin(rads);
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

/**
 * Draws the needle for a gauge.
 * @param centreX The x-coordinate of the gauge centre.
 * @param centreY The y-coordinate of the gauge centre.
 * @param outerRadius The radius at which the needle tip will point.
 * @param innerRadius The radius from which the needle starts.
 * @param width The width of the needle base.
 * @param colour The colour of the needle.
 * @param angle The angle in degrees at which to draw the needle.
 */
void drawNeedle(int centreX, int centreY, int outerRadius, int innerRadius, int colour, float angle) {
  int innerCentreX = centreX + innerRadius * cos(angle * PI / 180);
  int innerCentreY = centreY + innerRadius * sin(angle * PI / 180);
  int innerX1 = centreX + innerRadius * cos((angle + 3) * PI / 180);
  int innerY1 = centreY + innerRadius * sin((angle + 3) * PI / 180);
  int innerX2 = centreX + innerRadius * cos((angle - 3) * PI / 180);
  int innerY2 = centreY + innerRadius * sin((angle - 3) * PI / 180);
  int outerX = centreX + outerRadius * cos(angle * PI / 180);
  int outerY = centreY + outerRadius * sin(angle * PI / 180);
  

  tft.drawLine(innerCentreX, innerCentreY, outerX, outerY, colour);
  tft.fillTriangle(innerX1, innerY1, innerX2, innerY2, outerX, outerY, colour);
}

/**
 * Draws a fuel pump icon on the display.
 * @param x The top left x-coordinate of the fuel pump icon.
 * @param y The top left y-coordinate of the fuel pump icon.
 * @param iconColour The colour of the icon.
 * @param backgroundColour The background colour.
 */
void drawFuelPump(int x, int y, uint16_t iconColour, uint16_t backgroundColour) {
  tft.fillRoundRect(x, y, 12, 16, 1, iconColour);
  tft.fillRoundRect(x - 3, y + 14, 18, 2, 1, iconColour);
  tft.fillRect(x + 2, y + 3, 8, 4, backgroundColour);
  tft.drawPixel(x + 12, y + 6, iconColour);
  tft.drawLine(x + 13, y + 7, x + 13, y + 11, iconColour);
  tft.drawPixel(x + 14, y + 12, iconColour);
  tft.drawLine(x + 15, y + 11, x + 15, y + 5, iconColour);
  tft.drawLine(x + 15, y + 5, x + 13, y + 2, iconColour);
}

/**
 * Draws a headlight icon on the display.
 * @param x The centre x-coordinate of the headlight icon.
 * @param y The centre y-coordinate of the headlight icon.
 * @param iconColour The colour of the icon.
 * @param backgroundColour The background colour.
 */
void drawHeadlights(int x, int y, uint16_t iconColour, uint16_t backgroundColour) {
  tft.fillCircle(x, y, 10, iconColour);
  tft.fillRect(x - 10, y - 11, 10, 22, MIDDLE_BACKGROUND_COLOUR);
  tft.fillCircle(x, y, 8, MIDDLE_BACKGROUND_COLOUR);
  tft.drawLine(x, y - 10, x, y + 10, iconColour);
  tft.drawLine(x - 1, y - 10, x - 1, y + 10, iconColour);

  // Diagonal Lines
  tft.drawLine(x - 4, y - 9, x - 10, y - 8, iconColour);
  tft.drawLine(x - 4, y - 8, x - 10, y - 7, iconColour);
  tft.drawLine(x - 4, y - 5, x - 10, y - 4, iconColour);
  tft.drawLine(x - 4, y - 4, x - 10, y - 3, iconColour);
  
  tft.drawLine(x - 4, y - 1, x - 10, y, iconColour);
  tft.drawLine(x - 4, y, x - 10, y + 1, iconColour);

  tft.drawLine(x - 4, y + 3, x - 10, y + 4, iconColour);
  tft.drawLine(x - 4, y + 4, x - 10, y + 5, iconColour);
  tft.drawLine(x - 4, y + 7, x - 10, y + 8, iconColour);
  tft.drawLine(x - 4, y + 8, x - 10, y + 9, iconColour);
}

/**
 * Draws a highbeam icon on the display.
 * @param x The centre x-coordinate of the highbeam icon.
 * @param y The centre y-coordinate of the highbeam icon.
 * @param iconColour The colour of the icon.
 * @param backgroundColour The background colour.
 */
void drawHighBeams(int x, int y, uint16_t iconColour, uint16_t backgroundColour) {
  tft.fillCircle(x, y, 10, iconColour);
  tft.fillRect(x - 10, y - 11, 10, 22, backgroundColour);
  tft.fillCircle(x, y, 8, backgroundColour);
  tft.drawLine(x, y - 10, x, y + 10, iconColour);
  tft.drawLine(x - 1, y - 10, x - 1, y + 10, iconColour);

  // Horizontal Lines
  tft.drawLine(x - 4, y - 9, x - 10, y - 9, iconColour);
  tft.drawLine(x - 4, y - 8, x - 10, y - 8, iconColour);
  tft.drawLine(x - 4, y - 5, x - 10, y - 5, iconColour);
  tft.drawLine(x - 4, y - 4, x - 10, y - 4, iconColour);
  tft.drawLine(x - 4, y - 1, x - 10, y - 1, iconColour);
  tft.drawLine(x - 4, y, x - 10, y, iconColour);
  tft.drawLine(x - 4, y + 3, x - 10, y + 3, iconColour);
  tft.drawLine(x - 4, y + 4, x - 10, y + 4, iconColour);
  tft.drawLine(x - 4, y + 7, x - 10, y + 7, iconColour);
  tft.drawLine(x - 4, y + 8, x - 10, y + 8, iconColour);
}

/**
 * Draws an arrow on the display.
 * @param x The x-coordinate of the tip of the arrow.
 * @param y The y-coordinate of the tip of the arrow.
 * @param size Size of the arrow. 
 * @param colour The colour of the arrow icon.
 * @param pointingLeft Boolean indicating if the arrow should point left.
 */
void drawArrow(int x, int y, int size, int colour, bool pointingLeft) {
  if (pointingLeft) {
    tft.fillTriangle(x, y, x + size, y + size / 2, x + size, y - size / 2, colour);
    tft.fillRect(x + size, y - (size / 4), size, size / 2, colour);
  } else {
    tft.fillTriangle(x, y, x - size, y + size / 2, x - size, y - size / 2, colour);
    tft.fillRect(x - (2 * size), y - (size / 4), size, size / 2, colour);
  }
}

/**
 * Maps a float value from one range to another.
 * @param x The input value to map.
 * @param in_min The minimum of the input range.
 * @param in_max The maximum of the input range.
 * @param out_min The minimum of the output range.
 * @param out_max The maximum of the output range.
 * @return The mapped float value.
 */
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
