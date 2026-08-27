/*
  SAFETY WATCHDOG  (Node B - "USAFETY_WATCHDOG")
  --------------------------------------------------
  - A4/A5: I2C Master  -> Polls Drive Controller (0x08)
  - D7: Relay control  -> HIGH = Power ON, LOW = CUT POWER (Fault)
  - D8: Piezo buzzer   -> Audible alarm
  - D2,D3,D4,D5,D11,D12: 16x2 LCD (RS, EN, D4, D5, D6, D7)
*/

#include <Wire.h>
#include <LiquidCrystal.h>

// I2C & Hardware Pin Configurations
const uint8_t DRIVE_I2C_ADDR = 0x08;
const uint8_t PIN_RELAY      = 7;
const uint8_t PIN_BUZZER     = 8;

// Display Initialization
LiquidCrystal lcd(2, 3, 4, 5, 11, 12);

// Thresholds & Safety Limits (With Hysteresis to prevent chatter)
const uint16_t LOAD_TRIP_THRESHOLD  = 700; // Trips jam above this
const uint16_t LOAD_CLEAR_THRESHOLD = 650; // Auto-clears when load drops below this
const uint32_t COMM_TIMEOUT_MS      = 2000;
const uint32_t POLL_INTERVAL_MS     = 200;
const uint32_t SERIAL_INTERVAL      = 500;

// System States
enum SystemState { STATE_OK, STATE_JAM_FAULT, STATE_COMM_FAULT };
SystemState currentState  = STATE_OK;
SystemState previousState = STATE_OK;

// Telemetry & Timing Trackers
uint16_t lastSpeed = 0;
uint16_t lastLoad  = 0;
uint32_t lastGoodComm = 0;
uint32_t lastPoll     = 0;
uint32_t lastSerial   = 0;

void forceI2CReset() {
  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, OUTPUT);
  for (byte i = 0; i < 9; i++) {
    digitalWrite(SCL, LOW);
    delayMicroseconds(10);
    digitalWrite(SCL, HIGH);
    delayMicroseconds(10);
  }
}

void setup() {
  pinMode(PIN_RELAY, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  
  digitalWrite(PIN_RELAY, HIGH);
  noTone(PIN_BUZZER);

  forceI2CReset();

  Wire.begin();
  Wire.setWireTimeout(25000, true);

  lcd.begin(16, 2);
  lcd.print("System Init...");

  Serial.begin(115200);
  lastGoodComm = millis();
}

void loop() {
  uint32_t currentMillis = millis();

  // 1. Periodic I2C Polling
  if (currentMillis - lastPoll >= POLL_INTERVAL_MS) {
    lastPoll = currentMillis;
    
    uint16_t speedTemp = 0;
    uint16_t loadTemp = 0;

    if (pollDriveController(speedTemp, loadTemp)) {
      lastSpeed = speedTemp;
      lastLoad = loadTemp;
      lastGoodComm = currentMillis;
      
      // AUTO-CLEARING STATE LOGIC WITH HYSTERESIS:
      // - Trips to JAM if load goes above 700
      // - Auto-clears back to OK only when load drops below 650
      if (currentState == STATE_OK && lastLoad >= LOAD_TRIP_THRESHOLD) {
        currentState = STATE_JAM_FAULT;
      } 
      else if (currentState == STATE_JAM_FAULT && lastLoad <= LOAD_CLEAR_THRESHOLD) {
        currentState = STATE_OK;
      }
    }
  }

  // 2. Timeout Check (Overrides to COMM_FAULT if master loses connection)
  if (currentMillis - lastGoodComm > COMM_TIMEOUT_MS) {
    currentState = STATE_COMM_FAULT;
  }

  // 3. Update Hardware Outputs, Display, and Logs
  updateSafetyOutputs();
  updateDisplay();
  handleSerialLogging();
}

bool pollDriveController(uint16_t &speedOut, uint16_t &loadOut) {
  uint8_t bytesReceived = Wire.requestFrom(DRIVE_I2C_ADDR, (uint8_t)4);
  
  if (bytesReceived == 4) {
    uint8_t b0 = Wire.read();
    uint8_t b1 = Wire.read();
    uint8_t b2 = Wire.read();
    uint8_t b3 = Wire.read();

    speedOut = ((uint16_t)b0 << 8) | b1;
    loadOut  = ((uint16_t)b2 << 8) | b3;
    return true;
  }
  return false;
}

void updateSafetyOutputs() {
  if (currentState != previousState) {
    switch (currentState) {
      case STATE_OK:
        digitalWrite(PIN_RELAY, HIGH);
        noTone(PIN_BUZZER);
        break;

      case STATE_JAM_FAULT:
        digitalWrite(PIN_RELAY, LOW);
        tone(PIN_BUZZER, 1000);
        break;

      case STATE_COMM_FAULT:
        digitalWrite(PIN_RELAY, LOW);
        tone(PIN_BUZZER, 2500);
        break;
    }
    previousState = currentState;
  }
}

void updateDisplay() {
  static uint16_t prevDisplaySpeed = 9999;
  static uint16_t prevDisplayLoad  = 9999;
  static SystemState prevDispState = (SystemState)-1;

  // Ignore minor ADC jitter (changes <= 2 counts) to prevent screen flicker
  bool loadChangedSignificantly = abs((int)lastLoad - (int)prevDisplayLoad) > 2;

  if (currentState != prevDispState || lastSpeed != prevDisplaySpeed || loadChangedSignificantly) {
    char lineBuf[17];

    switch (currentState) {
      case STATE_OK: {
        uint8_t speedPct = map(constrain(lastSpeed, 0, 1023), 0, 1023, 0, 100);
        snprintf(lineBuf, sizeof(lineBuf), "Sys OK  Spd:%3d%%", speedPct);
        lcd.setCursor(0, 0);
        lcd.print(lineBuf);

        snprintf(lineBuf, sizeof(lineBuf), "Load: %-10d", lastLoad);
        lcd.setCursor(0, 1);
        lcd.print(lineBuf);
        break;
      }

      case STATE_JAM_FAULT:
        lcd.setCursor(0, 0);
        lcd.print("** JAM FAULT ** ");
        snprintf(lineBuf, sizeof(lineBuf), "Load: %-10d", lastLoad);
        lcd.setCursor(0, 1);
        lcd.print(lineBuf);
        break;

      case STATE_COMM_FAULT:
        lcd.setCursor(0, 0);
        lcd.print("** COMM FAULT **");
        lcd.setCursor(0, 1);
        lcd.print("No I2C Response ");
        break;
    }

    prevDisplaySpeed = lastSpeed;
    prevDisplayLoad  = lastLoad;
    prevDispState    = currentState;
  }
}

void handleSerialLogging() {
  if (millis() - lastSerial >= SERIAL_INTERVAL) {
    lastSerial = millis();
    Serial.print("State: ");
    Serial.print(currentState == STATE_OK ? "OK" : (currentState == STATE_JAM_FAULT ? "JAM" : "COMM_FAULT"));
    Serial.print(" | Speed: "); Serial.print(lastSpeed);
    Serial.print(" | Load: ");  Serial.println(lastLoad);
  }
}
