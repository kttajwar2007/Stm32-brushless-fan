#include <Servo.h>

// Pin Definitions
#define ENCODER_CLK PA0
#define ENCODER_DT  PA1
#define ENCODER_SW  PA2
#define ESC_PIN     PA8

Servo esc;

// Speed Control Limits (Microsecond Servo Pulses)
const int MIN_THROTTLE = 1000;
const int START_SPEED  = 1060;
const int MAX_THROTTLE = 2000;
const int SPEED_STEP   = 25;

// Shared Variables (Used in Interrupts)
volatile int targetSpeed = MIN_THROTTLE;
volatile bool isFanOn    = false;
volatile unsigned long lastEncoderTime = 0;

// Button Tracking
bool lastRawReading  = HIGH;
bool buttonState      = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

// ESC Arming State
bool escArmed = false;
unsigned long armingStartTime = 0;

int lastSentSpeed = -1;

void ISR_EncoderChange() {
  unsigned long currentTime = millis();
  if (currentTime - lastEncoderTime > 5) {
    if (isFanOn) {
      if (digitalRead(ENCODER_DT) == LOW) {
        targetSpeed += SPEED_STEP;
      } else {
        targetSpeed -= SPEED_STEP;
      }
      targetSpeed = constrain(targetSpeed, START_SPEED, MAX_THROTTLE);
    }
    lastEncoderTime = currentTime;
  }
}

void setup() {
  pinMode(ENCODER_CLK, INPUT_PULLUP);
  pinMode(ENCODER_DT, INPUT_PULLUP);
  pinMode(ENCODER_SW, INPUT_PULLUP);

  esc.attach(ESC_PIN, MIN_THROTTLE, MAX_THROTTLE);
  esc.writeMicroseconds(MIN_THROTTLE);
  armingStartTime = millis();

  attachInterrupt(digitalPinToInterrupt(ENCODER_CLK), ISR_EncoderChange, FALLING);
}

void loop() {
  if (!escArmed) {
    if (millis() - armingStartTime >= 3000) {
      escArmed = true;
    }
    return;
  }

  bool rawReading = digitalRead(ENCODER_SW);

  if (rawReading != lastRawReading) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (rawReading != buttonState) {
      buttonState = rawReading;
      if (buttonState == LOW) {
        isFanOn = !isFanOn;
        targetSpeed = isFanOn ? START_SPEED : MIN_THROTTLE;
      }
    }
  }
  lastRawReading = rawReading;

  int speedSnapshot;
  noInterrupts();
  speedSnapshot = targetSpeed;
  interrupts();

  if (speedSnapshot != lastSentSpeed) {
    esc.writeMicroseconds(speedSnapshot);
    lastSentSpeed = speedSnapshot;
  }
}
