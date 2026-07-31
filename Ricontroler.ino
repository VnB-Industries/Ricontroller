// Arduino flow-controlled dispenser
// Receives commands over USB serial from Raspberry Pi (or serial console for debug)
// Command format: DISPENSE:<pump1_cL>:<pump2_cL>[:<pump3_cL>...] e.g. "DISPENSE:2:20"
//   Pump 1 (index 0) has a flow meter; pumps 2..N are time-based.
// Also supports:
//   STOP                           (aborts an in-progress dispense/calibration)
//   CAL:<pump_index>:<quantity_cL> (calibrate pump 1..N; pump 1 uses pulses, others time)
//   SETFLOW:<p1_pulses_per_cL>:<p2_cL_per_s>[:<p3_cL_per_s>...] (set runtime constants)
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
const int irSensorPin = 4;     // IR object detection sensor digital output

// Pump 1 (index 0) is the flow-meter pump; pumps 2..N are time-based.
// Pins avoid 2 (flow interrupt), 4 (IR) and 10-13 (SPI/PN532).
#define MAX_PUMPS 6
const int pumpPins[MAX_PUMPS] = { 7, 6, 5, 8, 9, 3 };

const bool irDetectActiveLow = true; // true for common LM393 modules: LOW means object detected

volatile unsigned long pulseCount = 0;

const float defaultPulsesPerCl = 540.42;
const float defaultPump2FlowClPerSec = 2.8;

float pulsesPerCl = defaultPulsesPerCl;            // pump 1 flow-meter constant (pulses/cL)
float timeFlowClPerSec[MAX_PUMPS];                 // time-based pumps (cL/s); index 0 unused

enum RunMode
{
  RUN_MODE_IDLE = 0,
  RUN_MODE_DISPENSE,
  RUN_MODE_CALIBRATION
};

RunMode runMode = RUN_MODE_IDLE;

bool dispensing = false;
int  activePumpCount = 0;

bool  pumpRunning[MAX_PUMPS];
bool  pumpDone[MAX_PUMPS];
float requestedCl[MAX_PUMPS];
unsigned long pumpStartTime[MAX_PUMPS];
unsigned long pumpTargetDurationMs[MAX_PUMPS];

unsigned long targetPulses = 0;         // pump 1 flow-meter target
unsigned long dispenseStartTime = 0;
unsigned long maxDispenseTime = 0;      // Safety timeout, calculated per dispense
unsigned long lastProgressReport = 0;

int calibrationPumpIndex = 0;           // 1-based pump number under calibration
float calibrationTargetCl = 0.0;
unsigned long calibrationStartTime = 0;

const float expectedMinFlow_Lmin = 0.2; // Used only to size the safety timeout;
                                         // set below your pump's slowest expected flow

String inputBuffer = "";

// RFID debounce state
static char          s_activeUidHex[32]    = "";
static unsigned long s_activeUidLastSeenMs = 0;
const unsigned long  rfidRemovalTimeoutMs  = 5000UL;

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
void startDispense(float* volumesCentiliters, int count);
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

// Split a colon-separated list of non-negative floats into out[]. Returns the
// number parsed, or -1 on any invalid/empty token or if there are more than maxOut.
int splitFloats(const String& payload, float* out, int maxOut, bool requirePositive)
{
  int start = 0;
  int count = 0;

  while (true)
  {
    int sep = payload.indexOf(':', start);
    String token = (sep < 0) ? payload.substring(start) : payload.substring(start, sep);

    float value = 0.0;
    if (!parseNonNegativeFloat(token, &value))
    {
      return -1;
    }
    if (requirePositive && value <= 0.0)
    {
      return -1;
    }
    if (count >= maxOut)
    {
      return -1;
    }

    out[count++] = value;

    if (sep < 0)
    {
      break;
    }
    start = sep + 1;
  }

  return count;
}

float getPump1MeasuredCl(unsigned long currentPulses)
{
  if (pulsesPerCl <= 0.0001)
  {
    return 0.0;
  }

  return currentPulses / pulsesPerCl;
}

// Time-based estimate for a non-flow-meter pump (0-based index >= 1).
float getPumpTimeEstimateCl(int idx, unsigned long nowMs)
{
  if (idx < 1 || idx >= MAX_PUMPS)
  {
    return 0.0;
  }

  float flow = timeFlowClPerSec[idx];
  if (requestedCl[idx] <= 0 || flow <= 0.0001)
  {
    return 0.0;
  }

  unsigned long elapsed = nowMs - pumpStartTime[idx];
  if (elapsed > pumpTargetDurationMs[idx])
  {
    elapsed = pumpTargetDurationMs[idx];
  }

  float estimated = (elapsed / 1000.0) * flow;
  if (estimated > requestedCl[idx])
  {
    estimated = requestedCl[idx];
  }

  return estimated;
}

void printProgress(unsigned long currentPulses, unsigned long nowMs)
{
  Serial.print("PROGRESS:");
  for (int i = 0; i < activePumpCount; i++)
  {
    if (i > 0)
    {
      Serial.print(":");
    }
    float value = (i == 0) ? getPump1MeasuredCl(currentPulses)
                           : getPumpTimeEstimateCl(i, nowMs);
    Serial.print(value, 3);
  }
  Serial.println();
}

void setup()
{
  pinMode(hallsensor, INPUT_PULLUP);
  pinMode(irSensorPin, INPUT_PULLUP);

  for (int i = 0; i < MAX_PUMPS; i++)
  {
    pinMode(pumpPins[i], OUTPUT);
    digitalWrite(pumpPins[i], LOW);   // Pumps OFF at startup (active-HIGH relay; invert if active-LOW)
    timeFlowClPerSec[i] = defaultPump2FlowClPerSec;
    pumpRunning[i] = false;
    pumpDone[i] = false;
    requestedCl[i] = 0.0;
    pumpStartTime[i] = 0;
    pumpTargetDurationMs[i] = 0;
  }

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
    float vols[MAX_PUMPS];
    int count = splitFloats(payload, vols, MAX_PUMPS, false);
    if (count < 1)
    {
      Serial.println("ERROR:INVALID_VOLUME");
      return;
    }

    bool anyPositive = false;
    for (int i = 0; i < count; i++)
    {
      if (vols[i] > 0)
      {
        anyPositive = true;
      }
    }
    if (!anyPositive)
    {
      Serial.println("ERROR:INVALID_VOLUME");
      return;
    }

    startDispense(vols, count);
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
    if (pumpIndex < 1 || pumpIndex > MAX_PUMPS
        || !parseNonNegativeFloat(quantityToken, &quantityCl) || quantityCl <= 0.0)
    {
      Serial.println("ERROR:INVALID_CALIBRATION");
      return;
    }

    startCalibration(pumpIndex, quantityCl);
  }
  else if (cmd.startsWith("SETFLOW:"))
  {
    String payload = cmd.substring(8);
    float vals[MAX_PUMPS];
    int count = splitFloats(payload, vals, MAX_PUMPS, true);
    if (count < 1)
    {
      Serial.println("ERROR:INVALID_CONSTANTS");
      return;
    }

    pulsesPerCl = vals[0];
    for (int i = 1; i < count; i++)
    {
      timeFlowClPerSec[i] = vals[i];
    }

    Serial.print("SETFLOW:OK");
    for (int i = 0; i < count; i++)
    {
      Serial.print(":");
      Serial.print(vals[i], 3);
    }
    Serial.println();
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

void startDispense(float* volumesCentiliters, int count)
{
  runMode = RUN_MODE_DISPENSE;
  activePumpCount = count;

  noInterrupts();
  pulseCount = 0;
  interrupts();

  dispenseStartTime = millis();
  lastProgressReport = dispenseStartTime;
  maxDispenseTime = 0;

  for (int i = 0; i < count; i++)
  {
    requestedCl[i] = volumesCentiliters[i];
    pumpStartTime[i] = dispenseStartTime;
    pumpTargetDurationMs[i] = 0;

    if (i == 0)
    {
      // Pump 1: flow-meter controlled.
      targetPulses = (unsigned long)(volumesCentiliters[0] * pulsesPerCl);
      if (volumesCentiliters[0] > 0)
      {
        float liters = volumesCentiliters[0] / 100.0;
        maxDispenseTime = (unsigned long)((liters / expectedMinFlow_Lmin) * 60000.0 * 1.5);
        digitalWrite(pumpPins[0], HIGH);
        pumpRunning[0] = true;
        pumpDone[0] = false;
      }
      else
      {
        digitalWrite(pumpPins[0], LOW);
        pumpRunning[0] = false;
        pumpDone[0] = true;
      }
    }
    else
    {
      // Pumps 2..N: time-based.
      float flow = timeFlowClPerSec[i];
      if (volumesCentiliters[i] > 0 && flow > 0.0001)
      {
        pumpTargetDurationMs[i] = (unsigned long)((volumesCentiliters[i] / flow) * 1000.0);
        digitalWrite(pumpPins[i], HIGH);
        pumpRunning[i] = true;
        pumpDone[i] = false;
      }
      else
      {
        digitalWrite(pumpPins[i], LOW);
        pumpRunning[i] = false;
        pumpDone[i] = true;
      }
    }
  }

  dispensing = true;

  Serial.print("STARTED");
  for (int i = 0; i < count; i++)
  {
    Serial.print(":");
    Serial.print(volumesCentiliters[i], 3);
  }
  Serial.println();
}

void startCalibration(int pumpIndex, float targetCentiliters)
{
  runMode = RUN_MODE_CALIBRATION;
  dispensing = true;
  calibrationPumpIndex = pumpIndex;
  calibrationTargetCl = targetCentiliters;
  calibrationStartTime = millis();
  lastProgressReport = calibrationStartTime;
  activePumpCount = 0;

  noInterrupts();
  pulseCount = 0;
  interrupts();

  for (int i = 0; i < MAX_PUMPS; i++)
  {
    digitalWrite(pumpPins[i], LOW);
    pumpRunning[i] = false;
    pumpDone[i] = false;
    requestedCl[i] = 0.0;
  }

  int idx = pumpIndex - 1;
  digitalWrite(pumpPins[idx], HIGH);
  pumpRunning[idx] = true;
  pumpStartTime[idx] = calibrationStartTime;

  Serial.print("CAL:STARTED:");
  Serial.print(pumpIndex);
  Serial.print(":");
  Serial.println(targetCentiliters, 3);
}

void stopDispense(const char* resultMsg)
{
  unsigned long nowMs = millis();
  for (int i = 0; i < MAX_PUMPS; i++)
  {
    digitalWrite(pumpPins[i], LOW);
    pumpRunning[i] = false;
    pumpDone[i] = true;
  }
  dispensing = false;
  runMode = RUN_MODE_IDLE;

  noInterrupts();
  unsigned long currentPulses = pulseCount;
  interrupts();

  int n = activePumpCount > 0 ? activePumpCount : 1;

  Serial.print(resultMsg);
  for (int i = 0; i < n; i++)
  {
    Serial.print(":");
    float value = (i == 0) ? getPump1MeasuredCl(currentPulses)
                           : getPumpTimeEstimateCl(i, nowMs);
    Serial.print(value, 3);
  }
  Serial.println();

  activePumpCount = 0;
}

void stopCalibration(const char* resultMsg, bool applyCalibration)
{
  unsigned long nowMs = millis();
  for (int i = 0; i < MAX_PUMPS; i++)
  {
    digitalWrite(pumpPins[i], LOW);
    pumpRunning[i] = false;
    pumpDone[i] = true;
  }

  dispensing = false;
  runMode = RUN_MODE_IDLE;

  noInterrupts();
  unsigned long currentPulses = pulseCount;
  interrupts();

  int idx = calibrationPumpIndex - 1;
  float elapsedSec = (nowMs - calibrationStartTime) / 1000.0;
  float newConstant;

  if (calibrationPumpIndex == 1)
  {
    newConstant = pulsesPerCl;
    if (applyCalibration && calibrationTargetCl > 0.0 && currentPulses > 0)
    {
      newConstant = currentPulses / calibrationTargetCl;
      pulsesPerCl = newConstant;
    }
  }
  else
  {
    newConstant = timeFlowClPerSec[idx];
    if (applyCalibration && calibrationTargetCl > 0.0 && elapsedSec > 0.05)
    {
      newConstant = calibrationTargetCl / elapsedSec;
      timeFlowClPerSec[idx] = newConstant;
    }
  }

  Serial.print(resultMsg);
  Serial.print(":");
  Serial.print(calibrationPumpIndex);
  Serial.print(":");
  Serial.println(newConstant, 3);

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

  if (pumpRunning[0])
  {
    if (currentPulses >= targetPulses)
    {
      digitalWrite(pumpPins[0], LOW);
      pumpRunning[0] = false;
      pumpDone[0] = true;
    }
    else if (nowMs - dispenseStartTime > maxDispenseTime)
    {
      stopDispense("ERROR:TIMEOUT");
      return;
    }
  }

  for (int i = 1; i < activePumpCount; i++)
  {
    if (pumpRunning[i] && (nowMs - pumpStartTime[i] >= pumpTargetDurationMs[i]))
    {
      digitalWrite(pumpPins[i], LOW);
      pumpRunning[i] = false;
      pumpDone[i] = true;
    }
  }

  bool allDone = true;
  for (int i = 0; i < activePumpCount; i++)
  {
    if (!pumpDone[i])
    {
      allDone = false;
    }
  }

  if (allDone)
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
    float value;
    if (calibrationPumpIndex == 1)
    {
      value = getPump1MeasuredCl(currentPulses);
    }
    else
    {
      value = ((nowMs - calibrationStartTime) / 1000.0) * timeFlowClPerSec[calibrationPumpIndex - 1];
    }
    Serial.print("PROGRESS:");
    Serial.println(value, 3);
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

  uint8_t uid[7]= {0};
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
