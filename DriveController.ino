/*
  DRIVE CONTROLLER (Node A - "UDRIVE_CONTROLLER")
  --------------------------------------------------
  - A0: 10k pot           -> speed setpoint
  - A1: pot+10k divider   -> load reading
  - D2: pushbutton        -> start/stop toggle (INPUT_PULLUP)
  - D9: PWM (TIP120)      -> drives motor
  - A4/A5: I2C Slave     -> Address 0x08
*/

#include <Wire.h>

// Hardware Pin Definitions
const uint8_t PIN_SPEED_POT = A0;
const uint8_t PIN_LOAD_FSR  = A1;
const uint8_t PIN_BUTTON    = 2;
const uint8_t PIN_PWM_OUT   = 9;

// Network Configurations
const uint8_t I2C_ADDR = 0x08;

// Telemetry Registers
volatile uint16_t telemetrySpeed = 0;
volatile uint16_t telemetryLoad  = 0;

// Internal State
uint16_t speedValue  = 0;
uint16_t loadValue   = 0;
bool motorRunning    = false;

// Non-blocking Debounce
unsigned long lastDebounceTime  = 0;
const unsigned long DEBOUNCE_DELAY = 50;
bool lastRawButtonState  = HIGH;
bool stableButtonState   = HIGH;

// Non-blocking Logging
unsigned long lastSerialPrint = 0;
const unsigned long SERIAL_INTERVAL = 200; // Throttled to prevent buffer backup

void setup() {
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_PWM_OUT, OUTPUT);
  analogWrite(PIN_PWM_OUT, 0);

  Wire.begin(I2C_ADDR);
  Wire.onRequest(sendTelemetry);

  Serial.begin(115200);
}

void loop() {
  readInputs();
  processButtonDebounce();
  updateMotorDrive();
  syncTelemetryBuffer();
  handleSerialLogging();
}

void readInputs() {
  speedValue = analogRead(PIN_SPEED_POT);
  loadValue  = analogRead(PIN_LOAD_FSR);
}

void processButtonDebounce() {
  bool currentRawState = digitalRead(PIN_BUTTON);

  if (currentRawState != lastRawButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
    if (currentRawState != stableButtonState) {
      stableButtonState = currentRawState;

      if (stableButtonState == LOW) {
        motorRunning = !motorRunning;
      }
    }
  }
  lastRawButtonState = currentRawState;
}

void updateMotorDrive() {
  uint8_t pwmDutyCycle = 0;
  if (motorRunning) {
    pwmDutyCycle = speedValue >> 2; // Fast 10-bit (0-1023) to 8-bit (0-255) map
  }
  analogWrite(PIN_PWM_OUT, pwmDutyCycle);
}

void syncTelemetryBuffer() {
  // Simple atomic assignment without disabling global interrupts
  // to prevent delaying incoming Master clock pulses.
  telemetrySpeed = speedValue;
  telemetryLoad  = loadValue;
}

// Executed automatically inside I2C ISR when Master requests bytes
void sendTelemetry() {
  uint8_t buf[4];
  buf[0] = highByte(telemetrySpeed);
  buf[1] = lowByte(telemetrySpeed);
  buf[2] = highByte(telemetryLoad);
  buf[3] = lowByte(telemetryLoad);
  Wire.write(buf, sizeof(buf));
}

void handleSerialLogging() {
  if (millis() - lastSerialPrint >= SERIAL_INTERVAL) {
    lastSerialPrint = millis();
    Serial.print("Speed:");     Serial.print(speedValue);
    Serial.print("  Load:");    Serial.print(loadValue);
    Serial.print("  Running:"); Serial.println(motorRunning ? "YES" : "NO");
  }
}
