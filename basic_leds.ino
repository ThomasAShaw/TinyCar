#define HEADLIGHTS 2
#define BRAKELIGHTS 4
#define LEFT_SIGNAL 7
#define RIGHT_SIGNAL 8

void setup() {
  // put your setup code here, to run once:
  pinMode(HEADLIGHTS, OUTPUT);
  pinMode(BRAKELIGHTS, OUTPUT);
  pinMode(LEFT_SIGNAL, OUTPUT);
  pinMode(RIGHT_SIGNAL, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(HEADLIGHTS, HIGH);
  digitalWrite(BRAKELIGHTS, LOW);
  digitalWrite(LEFT_SIGNAL, HIGH);
  digitalWrite(RIGHT_SIGNAL, LOW);
  delay(1000);
  digitalWrite(HEADLIGHTS, LOW);
  digitalWrite(BRAKELIGHTS, HIGH);
  digitalWrite(LEFT_SIGNAL, LOW);
  digitalWrite(RIGHT_SIGNAL, HIGH);
  delay(1000);
}
