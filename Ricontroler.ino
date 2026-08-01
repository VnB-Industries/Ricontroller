// Arduino flow-controlled dispenser
// Receives commands over USB serial from Raspberry Pi (or serial console for debug)
// Command format: DISPENSE:<pump1_volume_cL>:<pump2_volume_cL> e.g. "DISPENSE:2:20"
// Also supports:
//   STOP                           (aborts an in-progress dispense/calibration)
//   CAL:<pump_index>:<quantity_cL> (start calibration mode for pump 1 or 2)
//   SETFLOW:<p1_pulses_per_cL>:<p2_cL_per_s> (set runtime constants)
//
// RFID: Grove NFC shield (PN532, SPI default wiring). Install "Adafruit PN532"
//       from Arduino Library Manager. CS/SS on D10 by default.
//
// Sensor: YF-S401, calibration factor 5880 pulses/liter
// Pump1: driven through relay/MOSFET on pin 7 (flow sensor backed)
// Pump2: driven through relay/MOSFET on pin 6 (time estimation only)

#include <SPI.h>
#include <Adafruit_PN532.h>

#if defined(__AVR__)
#include <avr/io.h>
#include <avr/wdt.h>
#endif

// PN532 in hardware SPI mode. Grove shield defaults CS/SS to D10.
#define PN532_SS_PIN  10
Adafruit_PN532 nfc(PN532_SS_PIN);

const int hallsensor = 2;      // Flow sensor signal pin (interrupt-capable: 2 or 3 on Uno)
const int pump1Pin = 7;        // Relay/MOSFET control pin for pump1
const int pump2Pin = 6;        // Relay/MOSFET control pin for pump2
const int irSensorPin = 4;     // IR object detection sensor digital output

const bool irDetectActiveLow = true; // true for common LM393 modules: LOW means object detected

volatile unsigned long pulseCount = 0;

const float defaultPulsesPerCl = 540.42;
const float defaultPump2FlowClPerSec = 2.8;

float pulsesPerCl = defaultPulsesPerCl;
float pump2FlowClPerSec = defaultPump2FlowClPerSec;

enum RunMode
{
  RUN_MODE_IDLE = 0,
  RUN_MODE_DISPENSE,
  RUN_MODE_CALIBRATION
};

RunMode runMode = RUN_MODE_IDLE;

bool dispensing = false;
bool pump1Running = false;
bool pump2Running = false;
bool pump1Done = false;
bool pump2Done = false;

float requestedPump1Cl = 0.0;
float requestedPump2Cl = 0.0;

unsigned long targetPulses = 0;
unsigned long dispenseStartTime = 0;
unsigned long maxDispenseTime = 0;      // Safety timeout, calculated per dispense
unsigned long pump2StartTime = 0;
unsigned long pump2TargetDurationMs = 0;
unsigned long lastProgressReport = 0;

int calibrationPumpIndex = 0;            // 1 for pump1, 2 for pump2
float calibrationTargetCl = 0.0;
unsigned long calibrationStartTime = 0;

const float expectedMinFlow_Lmin = 0.2; // Used only to size the safety timeout;
                                         // set below your pump's slowest expected flow

String inputBuffer = "";

// RFID debounce state
static char          s_activeUidHex[32]    = "";
static unsigned long s_activeUidLastSeenMs = 0;
const unsigned long  rfidRemovalTimeoutMs  = 1500UL;

static bool nfcReady = false;
static unsigned long s_lastNfcInitAttemptMs = 0;

#if defined(__AVR__)
static uint8_t s_resetFlags = 0;

// Capture reset flags as early as possible, before runtime init may alter MCUSR.
void captureResetFlags(void) __attribute__((naked)) __attribute__((section(".init3")));
void captureResetFlags(void)
{
  s_resetFlags = MCUSR;
  MCUSR = 0;
  wdt_disable();
}
#endif

void readSerialCommands();
void handleCommand(String cmd);
void startDispense(float pump1Centiliters, float pump2Centiliters);
void stopDispense(const char* resultMsg);
void startCalibration(int pumpIndex, float targetCentiliters);
void stopCalibration(const char* resultMsg, bool applyCalibration);
void processCalibrationTick();
void processDispenseTick();
void pollRfid();
bool isObjectDetected();
void printResetCause();
void tryInitNfc(bool force);

void rpm()
{
  pulseCount++;
}

bool isObjectDetected()
{
  int raw = digitalRead(irSensorPin);
  return irDetectActiveLow ? (raw == LOW) : (raw == HIGH);
}

void printResetCause()
{
#if defined(__AVR__)
  Serial.print("RESET_CAUSE:");
  bool any = false;

  if (s_resetFlags & _BV(PORF))
  {
    Serial.print("POR");
    any = true;
  }
  if (s_resetFlags & _BV(EXTRF))
  {
    if (any) Serial.print("|");
    Serial.print("EXT");
    any = true;
  }
  if (s_resetFlags & _BV(BORF))
  {
    if (any) Serial.print("|");
    Serial.print("BOR");
    any = true;
  }
  if (s_resetFlags & _BV(WDRF))
  {
    if (any) Serial.print("|");
    Serial.print("WDT");
    any = true;
  }

  if (!any)
  {
    Serial.print("UNKNOWN");
  }

  Serial.print(":0x");
  Serial.println(s_resetFlags, HEX);
#else
  Serial.println("RESET_CAUSE:UNSUPPORTED");
#endif
}

void tryInitNfc(bool force)
{
  unsigned long nowMs = millis();
  if (!force && (nowMs - s_lastNfcInitAttemptMs < 2000UL))
  {
    return;
  }

  s_lastNfcInitAttemptMs = nowMs;

  if (nfc.getFirmwareVersion())
  {
    nfc.SAMConfig();
    if (!nfcReady)
    {
      Serial.println("NFC:READY");
    }
    nfcReady = true;
  }
  else
  {
    if (force || nfcReady)
    {
      Serial.println("NFC:NO_SHIELD");
    }
    nfcReady = false;
  }
}

bool parseNonNegativeFloat(const String& text, float* outValue)
{
  String token = text;
  token.trim();
  if (token.length() == 0)
  {
    return false;
  }

  bool dotSeen = false;
  bool digitSeen = false;

  for (unsigned int i = 0; i < token.length(); i++)
  {
    char c = token.charAt(i);
    if (c >= '0' && c <= '9')
    {
      digitSeen = true;
      continue;
    }
    if (c == '.')
    {
      if (dotSeen)
      {
        return false;
      }
      dotSeen = true;
      continue;
    }

    return false;
  }

  if (!digitSeen)
  {
    return false;
  }

  float value = token.toFloat();
  if (value < 0)
  {
    return false;
  }

  *outValue = value;
  return true;
}

float getPump1MeasuredCl(unsigned long currentPulses)
{
  if (pulsesPerCl <= 0.0001)
  {
    return 0.0;
  }

  return currentPulses / pulsesPerCl;
}

float getPump2EstimatedCl(unsigned long nowMs)
{
  if (runMode == RUN_MODE_CALIBRATION && calibrationPumpIndex == 2)
  {
    unsigned long elapsedCal = nowMs - calibrationStartTime;
    return (elapsedCal / 1000.0) * pump2FlowClPerSec;
  }

  if (requestedPump2Cl <= 0)
  {
    return 0.0;
  }

  unsigned long elapsed = nowMs - pump2StartTime;
  if (elapsed > pump2TargetDurationMs)
  {
    elapsed = pump2TargetDurationMs;
  }

  float estimated = (elapsed / 1000.0) * pump2FlowClPerSec;
  if (estimated > requestedPump2Cl)
  {
    estimated = requestedPump2Cl;
  }

  return estimated;
}

void printProgress(unsigned long currentPulses, unsigned long nowMs)
{
  Serial.print("PROGRESS:");
  Serial.print(getPump1MeasuredCl(currentPulses), 3);
  Serial.print(":");
  Serial.println(getPump2EstimatedCl(nowMs), 3);
}

void setup()
{
  pinMode(hallsensor, INPUT_PULLUP);
  pinMode(pump1Pin, OUTPUT);
  pinMode(pump2Pin, OUTPUT);
  pinMode(irSensorPin, INPUT_PULLUP);
  digitalWrite(pump1Pin, LOW);   // Pump OFF at startup (assumes active-HIGH relay; invert if yours is active-LOW)
  digitalWrite(pump2Pin, LOW);

  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(hallsensor), rpm, RISING);

  nfc.begin();
  tryInitNfc(true);

  printResetCause();
  Serial.println("READY");
}

void loop()
{
  readSerialCommands();

  if (dispensing)
  {
    if (runMode == RUN_MODE_CALIBRATION)
    {
      processCalibrationTick();
      return;
    }

    processDispenseTick();
    return;
  }
  else
  {
    // Not dispensing: poll for RFID tags
    pollRfid();
  }
}

void readSerialCommands()
{
  while (Serial.available())
  {
    char c = Serial.read();
    if (c == '\n' || c == '\r')
    {
      if (inputBuffer.length() > 0)
      {
        handleCommand(inputBuffer);
        inputBuffer = "";
      }
    }
    else
    {
      inputBuffer += c;
    }
  }
}

void handleCommand(String cmd)
{
  cmd.trim();

  if (cmd.startsWith("DISPENSE:"))
  {
    if (dispensing)
    {
      Serial.println("ERROR:BUSY");
      return;
    }

    if (!isObjectDetected())
    {
      Serial.println("ERROR:NO_OBJECT");
      return;
    }

    String payload = cmd.substring(9);
    int sep = payload.indexOf(':');
    if (sep < 0 || payload.indexOf(':', sep + 1) >= 0)
    {
      Serial.println("ERROR:INVALID_VOLUME");
      return;
    }

    String pump1Token = payload.substring(0, sep);
    String pump2Token = payload.substring(sep + 1);

    float pump1Cl = 0;
    float pump2Cl = 0;
    if (!parseNonNegativeFloat(pump1Token, &pump1Cl) || !parseNonNegativeFloat(pump2Token, &pump2Cl))
    {
      Serial.println("ERROR:INVALID_VOLUME");
      return;
    }

    if (pump1Cl <= 0 && pump2Cl <= 0)
    {
      Serial.println("ERROR:INVALID_VOLUME");
      return;
    }

    startDispense(pump1Cl, pump2Cl);
  }
  else if (cmd.startsWith("CAL:"))
  {
    if (dispensing)
    {
      Serial.println("ERROR:BUSY");
      return;
    }

    if (!isObjectDetected())
    {
      Serial.println("ERROR:NO_OBJECT");
      return;
    }

    String payload = cmd.substring(4);
    int sep = payload.indexOf(':');
    if (sep < 0 || payload.indexOf(':', sep + 1) >= 0)
    {
      Serial.println("ERROR:INVALID_CALIBRATION");
      return;
    }

    String pumpToken = payload.substring(0, sep);
    String quantityToken = payload.substring(sep + 1);
    pumpToken.trim();

    int pumpIndex = pumpToken.toInt();
    float quantityCl = 0.0;
    if ((pumpIndex != 1 && pumpIndex != 2) || !parseNonNegativeFloat(quantityToken, &quantityCl) || quantityCl <= 0.0)
    {
      Serial.println("ERROR:INVALID_CALIBRATION");
      return;
    }

    startCalibration(pumpIndex, quantityCl);
  }
  else if (cmd.startsWith("SETFLOW:"))
  {
    String payload = cmd.substring(8);
    int sep = payload.indexOf(':');
    if (sep < 0 || payload.indexOf(':', sep + 1) >= 0)
    {
      Serial.println("ERROR:INVALID_CONSTANTS");
      return;
    }

    String p1Token = payload.substring(0, sep);
    String p2Token = payload.substring(sep + 1);
    float newP1 = 0.0;
    float newP2 = 0.0;
    if (!parseNonNegativeFloat(p1Token, &newP1) || !parseNonNegativeFloat(p2Token, &newP2)
        || newP1 <= 0.0 || newP2 <= 0.0)
    {
      Serial.println("ERROR:INVALID_CONSTANTS");
      return;
    }

    pulsesPerCl = newP1;
    pump2FlowClPerSec = newP2;

    Serial.print("SETFLOW:OK:");
    Serial.print(pulsesPerCl, 3);
    Serial.print(":");
    Serial.println(pump2FlowClPerSec, 3);
  }
  else if (cmd == "STOP")
  {
    if (dispensing)
    {
      if (runMode == RUN_MODE_CALIBRATION)
      {
        stopCalibration("CAL:STOPPED", false);
      }
      else
      {
        stopDispense("STOPPED");
      }
    }
    else
    {
      Serial.println("ERROR:NOT_DISPENSING");
    }
  }
  else
  {
    Serial.println("ERROR:UNKNOWN_COMMAND");
  }
}

void startDispense(float pump1Centiliters, float pump2Centiliters)
{
  runMode = RUN_MODE_DISPENSE;
  requestedPump1Cl = pump1Centiliters;
  requestedPump2Cl = pump2Centiliters;

  noInterrupts();
  pulseCount = 0;
  interrupts();

  targetPulses = (unsigned long)(pump1Centiliters * pulsesPerCl);
  dispenseStartTime = millis();
  pump2StartTime = dispenseStartTime;
  pump2TargetDurationMs = (unsigned long)((pump2Centiliters / pump2FlowClPerSec) * 1000.0);
  lastProgressReport = dispenseStartTime;

  if (pump1Centiliters > 0)
  {
    float liters = pump1Centiliters / 100.0;
    maxDispenseTime = (unsigned long)((liters / expectedMinFlow_Lmin) * 60000.0 * 1.5);
    digitalWrite(pump1Pin, HIGH);
    pump1Running = true;
    pump1Done = false;
  }
  else
  {
    maxDispenseTime = 0;
    digitalWrite(pump1Pin, LOW);
    pump1Running = false;
    pump1Done = true;
  }

  if (pump2Centiliters > 0)
  {
    digitalWrite(pump2Pin, HIGH);
    pump2Running = true;
    pump2Done = false;
  }
  else
  {
    digitalWrite(pump2Pin, LOW);
    pump2Running = false;
    pump2Done = true;
  }

  dispensing = true;

  Serial.print("STARTED:");
  Serial.print(pump1Centiliters, 3);
  Serial.print(":");
  Serial.println(pump2Centiliters, 3);
}

void startCalibration(int pumpIndex, float targetCentiliters)
{
  runMode = RUN_MODE_CALIBRATION;
  dispensing = true;
  calibrationPumpIndex = pumpIndex;
  calibrationTargetCl = targetCentiliters;
  calibrationStartTime = millis();
  lastProgressReport = calibrationStartTime;

  requestedPump1Cl = 0.0;
  requestedPump2Cl = 0.0;

  noInterrupts();
  pulseCount = 0;
  interrupts();

  digitalWrite(pump1Pin, LOW);
  digitalWrite(pump2Pin, LOW);
  pump1Running = false;
  pump2Running = false;
  pump1Done = false;
  pump2Done = false;

  if (pumpIndex == 1)
  {
    digitalWrite(pump1Pin, HIGH);
    pump1Running = true;
  }
  else
  {
    digitalWrite(pump2Pin, HIGH);
    pump2Running = true;
    pump2StartTime = calibrationStartTime;
  }

  Serial.print("CAL:STARTED:");
  Serial.print(pumpIndex);
  Serial.print(":");
  Serial.println(targetCentiliters, 3);
}

void stopDispense(const char* resultMsg)
{
  unsigned long nowMs = millis();
  digitalWrite(pump1Pin, LOW);
  digitalWrite(pump2Pin, LOW);
  dispensing = false;
  runMode = RUN_MODE_IDLE;
  pump1Running = false;
  pump2Running = false;
  pump1Done = true;
  pump2Done = true;

  noInterrupts();
  unsigned long currentPulses = pulseCount;
  interrupts();

  float actualPump1Cl = getPump1MeasuredCl(currentPulses);
  float estimatedPump2Cl = getPump2EstimatedCl(nowMs);

  Serial.print(resultMsg);
  Serial.print(":");
  Serial.print(actualPump1Cl, 3);
  Serial.print(":");
  Serial.println(estimatedPump2Cl, 3);
}

void stopCalibration(const char* resultMsg, bool applyCalibration)
{
  unsigned long nowMs = millis();
  digitalWrite(pump1Pin, LOW);
  digitalWrite(pump2Pin, LOW);

  dispensing = false;
  runMode = RUN_MODE_IDLE;
  pump1Running = false;
  pump2Running = false;
  pump1Done = true;
  pump2Done = true;

  noInterrupts();
  unsigned long currentPulses = pulseCount;
  interrupts();

  unsigned long elapsedMs = nowMs - calibrationStartTime;
  float elapsedSec = elapsedMs / 1000.0;

  float newPump1 = pulsesPerCl;
  float newPump2 = pump2FlowClPerSec;

  if (applyCalibration)
  {
    if (calibrationPumpIndex == 1)
    {
      if (calibrationTargetCl > 0.0 && currentPulses > 0)
      {
        newPump1 = currentPulses / calibrationTargetCl;
      }
    }
    else if (calibrationPumpIndex == 2)
    {
      if (calibrationTargetCl > 0.0 && elapsedSec > 0.05)
      {
        newPump2 = calibrationTargetCl / elapsedSec;
      }
    }

    if (newPump1 > 0.0)
    {
      pulsesPerCl = newPump1;
    }
    if (newPump2 > 0.0)
    {
      pump2FlowClPerSec = newPump2;
    }
  }

  Serial.print(resultMsg);
  Serial.print(":");
  Serial.print(pulsesPerCl, 3);
  Serial.print(":");
  Serial.println(pump2FlowClPerSec, 3);

  calibrationPumpIndex = 0;
  calibrationTargetCl = 0.0;
  calibrationStartTime = 0;
}

void processDispenseTick()
{
  if (!isObjectDetected())
  {
    stopDispense("ERROR:NO_OBJECT");
    return;
  }

  unsigned long nowMs = millis();

  noInterrupts();
  unsigned long currentPulses = pulseCount;
  interrupts();

  if (pump1Running)
  {
    if (currentPulses >= targetPulses)
    {
      digitalWrite(pump1Pin, LOW);
      pump1Running = false;
      pump1Done = true;
    }
    else if (nowMs - dispenseStartTime > maxDispenseTime)
    {
      stopDispense("ERROR:TIMEOUT");
      return;
    }
  }

  if (pump2Running && (nowMs - pump2StartTime >= pump2TargetDurationMs))
  {
    digitalWrite(pump2Pin, LOW);
    pump2Running = false;
    pump2Done = true;
  }

  if (pump1Done && pump2Done)
  {
    stopDispense("OK");
  }
  else if (nowMs - lastProgressReport >= 250)
  {
    lastProgressReport = nowMs;
    printProgress(currentPulses, nowMs);
  }
}

void processCalibrationTick()
{
  unsigned long nowMs = millis();

  noInterrupts();
  unsigned long currentPulses = pulseCount;
  interrupts();

  if (!isObjectDetected())
  {
    stopCalibration("CAL:STOPPED_BY_SENSOR", true);
    return;
  }

  if (nowMs - lastProgressReport >= 250)
  {
    lastProgressReport = nowMs;
    printProgress(currentPulses, nowMs);
  }
}

// Poll for an RFID tag and send "RFID:<hex_uid>" when one is found.
// Called in loop() only when not dispensing, so the 50 ms I2C timeout
// does not interfere with pump control.
void pollRfid()
{
  unsigned long nowMs = millis();

  if (!nfcReady)
  {
    // Keep trying to recover PN532 after transient power/reset issues.
    tryInitNfc(false);
    return;
  }

  uint8_t uid[7];
  uint8_t uidLen = 0;

  if (!nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLen, 50))
  {
    // Consider the tag removed only after a grace period to tolerate brief read misses.
    if (s_activeUidHex[0] != '\0' && (nowMs - s_activeUidLastSeenMs) > rfidRemovalTimeoutMs)
    {
      s_activeUidHex[0] = '\0';
      s_activeUidLastSeenMs = 0;
    }
    return;
  }
  if (uidLen == 0) return;

  // Format UID bytes as uppercase hex
  char uidHex[32] = "";
  for (uint8_t i = 0; i < uidLen && i < 7; i++) {
    char tmp[3];
    snprintf(tmp, sizeof(tmp), "%02X", uid[i]);
    strcat(uidHex, tmp);
  }

  // Same tag is still present: refresh presence timestamp and do not re-emit.
  if (strcmp(uidHex, s_activeUidHex) == 0)
  {
    s_activeUidLastSeenMs = nowMs;
    return;
  }

  // New tag (or previous one removed): emit once and mark as active.
  strncpy(s_activeUidHex, uidHex, sizeof(s_activeUidHex) - 1);
  s_activeUidHex[sizeof(s_activeUidHex) - 1] = '\0';
  s_activeUidLastSeenMs = nowMs;

  Serial.print("RFID:");
  Serial.println(uidHex);
}
