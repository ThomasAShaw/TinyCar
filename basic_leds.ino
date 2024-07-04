#define HEADLIGHTS 2
#define BRAKELIGHTS 4
#define LEFT_SIGNAL 7
#define RIGHT_SIGNAL 8
#define HAZARDS_BUTTON 3

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
  
  digitalWrite(HEADLIGHTS, HIGH);
  digitalWrite(BRAKELIGHTS, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(LEFT_SIGNAL, LOW);
  digitalWrite(RIGHT_SIGNAL, LOW);

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
}

void setHazards(bool hazardsOn) {
  if (hazardsOn) {
    digitalWrite(LEFT_SIGNAL, HIGH);
    digitalWrite(RIGHT_SIGNAL, HIGH);
  } else {
    digitalWrite(LEFT_SIGNAL, LOW);
    digitalWrite(RIGHT_SIGNAL, LOW);
  }
}
