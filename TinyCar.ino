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

#define SIGNAL_TIMING_MS 750

bool hazardsToggledOn = false;
bool leftSignalToggledOn = false;
bool rightSignalToggledOn = false;
bool signalLightsOn = false;
bool hazardButtonState = HIGH;
unsigned long signalTime = 0;

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
}

void loop() {
  handleTurnSignals();
  handleHeadlights();
  handleGasPedal();
  handleBrakePedal();
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

// TODO
void handleGasPedal(void) {
  if (digitalRead(GAS_PEDAL_DIGITAL) == LOW) {
    // do nothing...
  }
}

// TODO
void handleBrakePedal(void) {
  if (digitalRead(BRAKE_PEDAL_DIGITAL) == LOW) {
    analogWrite(BRAKELIGHTS, 255);
  } else {
    analogWrite(BRAKELIGHTS, 0);
  }
}
