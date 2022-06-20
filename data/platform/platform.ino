#include <Stepper.h>

#define STEPS 200
#define SPEED 1.5

// Give the motor control pins names:
#define pwmA 3
#define pwmB 11
#define brakeA 9
#define brakeB 8

Stepper m(STEPS, 12, 13);
int steps = 0;
bool RUNNING = false;

void setup() {
  // Set the PWM and brake pins so that the direction pins can be used to control the motor:
  pinMode(pwmA, OUTPUT);
  pinMode(pwmB, OUTPUT);
  pinMode(brakeA, OUTPUT);
  pinMode(brakeB, OUTPUT);

  lock();

  m.setSpeed(SPEED);
  Serial.begin(9600);
}

void loop() {
  if (RUNNING){
    m.step(1);
    steps++;
    steps %= STEPS;
  }

  switch (Serial.read()) {
    case '0':
      Serial.println("Resetting...");
      RUNNING = false;
      m.setSpeed(60);
      m.step((STEPS - steps) % STEPS);
      m.setSpeed(SPEED);
      steps = 0;
      lock();
      break;
    case '1':
      RUNNING = !RUNNING;
      if (RUNNING)
        Serial.println("Running...");
      else
        Serial.println("Stopping...");
        lock();
      break;
    case -1:
      return;
    default:
      return;
  }
}


void lock(){
  digitalWrite(pwmA, HIGH);
  digitalWrite(pwmB, HIGH);
  digitalWrite(brakeA, LOW);
  digitalWrite(brakeB, LOW);
}
