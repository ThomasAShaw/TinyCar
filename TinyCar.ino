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
#define PEDAL_TIMING_MS 100

#define PI 3.14159
#define CM_IN_KM 100000

// Car-specific stats - potential FIXME
const float MAX_SPEED = 200.0;
const float IDLE_RPM = 800.0;
const float MAX_RPM = 8000.0;
const float GAS_INCREASE_RPM_RATE = 100.0;
const float BRAKE_DECREASE_RPM_RATE = 150.0;
const float IDLE_DECREASE_RPM_RATE = 50.0;
const float FINAL_DRIVE_RATIO = 3.58;
const float TRANS_RATIOS[] = {3.49, 1.99, 1.45, 1.00, 0.71, 3.99}; // 1, 2, 3, 4, 5, R
const float TIRE_DIAMETER_CM = 60.1;

bool hazardsToggledOn = false;
bool leftSignalToggledOn = false;
bool rightSignalToggledOn = false;
bool signalLightsOn = false;
bool hazardButtonState = HIGH;
unsigned long signalTime = 0;
unsigned long pedalTime = 0;
unsigned int pedalsCheckedCount = 0;
float currentSpeed = 0.0;
float currentRPM = 0.0;

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
}

void loop() {
  handleTurnSignals();
  handleHeadlights();
  if (millis() - pedalTime >= PEDAL_TIMING_MS) {
    handlePedals();

    pedalTime = millis();
    pedalsCheckedCount++;

    if (pedalsCheckedCount >= 10) {
      Serial.println("Current Speed:");
      Serial.println(currentSpeed);
      Serial.println("Current RPM:");
      Serial.println(currentRPM);

      pedalsCheckedCount = 0;
    }
  }
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
  float currentTransRatio = TRANS_RATIOS[0]; // FIXME when transmission is added

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
