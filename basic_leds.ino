#define HEADLIGHTS 2
#define BRAKELIGHTS 4
#define LEFT_SIGNAL 7
#define RIGHT_SIGNAL 8

#define HAZARDS_BUTTON 3

bool hazardsOn = false;
int recordedTime = millis();
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

  if (digitalRead(HAZARDS_BUTTON) == LOW) {
    hazardsOn = true;
    recordedTime = millis();

    while (digitalRead(HAZARDS_BUTTON) == LOW) {
      setHazards(hazardsOn);
      if (millis() - recordedTime >= 750) {
        hazardsOn = !hazardsOn;
        recordedTime = millis();
      }
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
