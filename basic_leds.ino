#define HEADLIGHTS 3
#define BRAKELIGHTS 9
#define LEFT_SIGNAL 5
#define RIGHT_SIGNAL 6
#define HAZARDS_BUTTON 2
#define GAS_PEDAL_DIGITAL 12 // Testing as digital, will be switched to analog eventually
#define BRAKE_PEDAL_DIGITAL 13 // Testing as digital, will be switched to analog eventually

bool hazardsToggledOn = false;
bool hazardsLightsOn = false;
bool hazardButtonState = HIGH;
unsigned long recordedTime = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(HEADLIGHTS, OUTPUT);
  pinMode(BRAKELIGHTS, OUTPUT);
  pinMode(LEFT_SIGNAL, OUTPUT);
  pinMode(RIGHT_SIGNAL, OUTPUT);
  pinMode(HAZARDS_BUTTON, INPUT_PULLUP);
  pinMode(GAS_PEDAL_DIGITAL, INPUT_PULLUP);
  pinMode(BRAKE_PEDAL_DIGITAL, INPUT_PULLUP);
  
  analogWrite(HEADLIGHTS, 128);
}

void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(LEFT_SIGNAL, 0);
  analogWrite(RIGHT_SIGNAL, 0);

  bool hazardButtonReading = digitalRead(HAZARDS_BUTTON);

 if (hazardButtonState != hazardButtonReading) {
    hazardButtonState = hazardButtonReading;

    if (hazardButtonState == LOW) {
      hazardsToggledOn = !hazardsToggledOn;
      hazardsLightsOn = hazardsToggledOn;
      recordedTime = millis();
    }
  }

  setHazards(hazardsLightsOn);
  if (hazardsToggledOn) {
    if (millis() - recordedTime >= 750) {
      hazardsLightsOn = !hazardsLightsOn;
      recordedTime = millis();
    }
  }

  // Gas and brakes
  if (digitalRead(GAS_PEDAL_DIGITAL) == HIGH) {
    // do nothing...
  }

  if (digitalRead(BRAKE_PEDAL_DIGITAL) == LOW) {
    analogWrite(BRAKELIGHTS, 255);
  } else {
    analogWrite(BRAKELIGHTS, 128);
  }
}

void setHazards(bool hazardsOn) {
  if (hazardsOn) {
    analogWrite(LEFT_SIGNAL, 255);
    analogWrite(RIGHT_SIGNAL, 255);
  } else {
    analogWrite(LEFT_SIGNAL, 0);
    analogWrite(RIGHT_SIGNAL, 0);
  }
}
