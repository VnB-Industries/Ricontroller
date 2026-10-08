// Arduino flow-controlled dispenser
// Receives commands over USB serial from Raspberry Pi (or serial console for debug)
//
// Every command is sequenced and every reply that answers one echoes that
// sequence, so the host can tell an ack from unsolicited telemetry and can
// discard a reply to a command it has already abandoned:
//
//   @<seq>:DISPENSE:<p1_cL>:<p1_pulses_per_cL>:<p2_cL>:<p2_cL_per_s>[...]
//       Fields alternate volume/flow-constant per pump. Controller computes targets.
//       Pump 1 (index 0) has a flow meter; pumps 2..N are time-based.
//   @<seq>:CAL:<pump_index>:<quantity_cL>
//       Calibrate pump 1..N; pump 1 uses pulses, others time.
//   @<seq>:PURGE:<pump_index|0=all>:<duration_ms>
//       Run pumps for a bounded time. Duration is clamped to PURGE_MAX_MS here
//       as well, so the pumps stop even if the host dies mid-purge.
//   @0:STOP
//       Aborts whatever is running. Deliberately unsequenced: it must be
//       obeyed whatever either side thinks is in flight.
//
// Replies:  @<seq>:STARTED:<cL>...        @<seq>:OK:<measured>...
//           @<seq>:STOPPED:<measured>...  @<seq>:ERROR:<CODE>[:detail]
//           @<seq>:ABORTED:<CODE>:<measured>...  run cut short after starting
//           @<seq>:CAL_STARTED:<pump>:<target_cL>
//           @<seq>:CAL_DONE:<pump>:<constant>:<by_sensor 0|1>
// Telemetry (never sequenced): PROGRESS:<cL>...  RFID:<uid>  READY
//                              NFC:<state>  RESET_CAUSE:<cause>:0x<raw>
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
  RUN_MODE_CALIBRATION,
  RUN_MODE_PURGE
};

RunMode runMode = RUN_MODE_IDLE;

bool dispensing = false;
int  activePumpCount = 0;

// Sequence of the command that started the run in progress, so its terminal
// reply can be matched to it by the host.
unsigned int activeSeq = 0;

bool  pumpRunning[MAX_PUMPS];
bool  pumpDone[MAX_PUMPS];
float requestedCl[MAX_PUMPS];
unsigned long pumpStartTime[MAX_PUMPS];
unsigned long pumpTargetDurationMs[MAX_PUMPS];
// Staggered start: a pump waits until its slot instead of the loop blocking in
// delay(). Blocking meant serial was not read for up to 600 ms while pumps
// were already running, so a STOP could not get through.
bool          pumpPendingStart[MAX_PUMPS];
unsigned long pumpStartAtMs[MAX_PUMPS];

unsigned long targetPulses = 0;         // pump 1 flow-meter target
unsigned long dispenseStartTime = 0;
unsigned long maxDispenseTime = 0;      // Safety timeout, calculated per dispense
unsigned long lastProgressReport = 0;

int calibrationPumpIndex = 0;           // 1-based pump number under calibration
float calibrationTargetCl = 0.0;
unsigned long calibrationStartTime = 0;
// Calibration used to end only when the IR sensor stopped seeing the glass.
// A stuck or unplugged sensor therefore ran the pump forever; this bounds it.
unsigned long calibrationMaxMs = 0;
const unsigned long calibrationCeilingMs = 120000UL;
const unsigned long calibrationFloorMs   = 15000UL;

unsigned long purgeDurationMs = 0;
const unsigned long PURGE_MAX_MS = 15000UL;

const float expectedMinFlow_Lmin = 0.2; // Used only to size the safety timeout;
                                         // set below your pump's slowest expected flow

const unsigned long pumpStartStaggerMs = 120; // Gap between relay activations to avoid a combined inrush-current brownout

// Fixed-size command buffer. A String grew without bound here, so a noise
// burst with no newline could exhaust the 2 kB of SRAM mid-dispense.
#define CMD_BUF_LEN 160
char cmdBuf[CMD_BUF_LEN];
unsigned int cmdLen = 0;
bool cmdOverflow = false;

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
void handleCommand(char* cmd);
void startDispense(unsigned int seq, float* volumes, float* flows, int count);
void stopDispense(const char* resultMsg, const char* errorCode);
void startCalibration(unsigned int seq, int pumpIndex, float targetCentiliters);
void stopCalibration(bool stoppedBySensor, bool applyCalibration, const char* errorCode);
void startPurge(unsigned int seq, int pumpIndex, unsigned long durationMs);
void processCalibrationTick();
void processDispenseTick();
void pollRfid();
bool isObjectDetected();
void printResetCause();
void tryInitNfc(bool force);
void replyHeader(unsigned int seq);
void replyError(unsigned int seq, const char* code);
void runWatchdogEnable();
void runWatchdogDisable();
void serviceStaggeredStarts(unsigned long nowMs);

void rpm()
{
  pulseCount++;
}

bool isObjectDetected()
{
  int raw = digitalRead(irSensorPin);
  return irDetectActiveLow ? (raw == LOW) : (raw == HIGH);
}

// ── reply helpers ──────────────────────────────────────────────────────────

void replyHeader(unsigned int seq)
{
  Serial.print('@');
  Serial.print(seq);
  Serial.print(':');
}

void replyError(unsigned int seq, const char* code)
{
  replyHeader(seq);
  Serial.print("ERROR:");
  Serial.println(code);
}

// ── watchdog ───────────────────────────────────────────────────────────────
//
// Armed only while pumps may be energised. An AVR reset leaves the pump pins
// as inputs, which releases an active-HIGH relay, so a firmware hang becomes
// pumps-off instead of pumps-on-forever. captureResetFlags() disables the
// watchdog in .init3, which is what stops the board from reset-looping.

void runWatchdogEnable()
{
#if defined(__AVR__)
  wdt_enable(WDTO_2S);
#endif
}

void runWatchdogDisable()
{
#if defined(__AVR__)
  wdt_disable();
#endif
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

// ── parsing ────────────────────────────────────────────────────────────────

bool parseNonNegativeFloat(const char* text, float* outValue)
{
  if (text == NULL) return false;

  // Skip leading spaces.
  while (*text == ' ' || *text == '\t') text++;
  if (*text == '\0') return false;

  bool dotSeen = false;
  bool digitSeen = false;

  for (const char* p = text; *p != '\0'; p++)
  {
    char c = *p;
    if (c >= '0' && c <= '9')
    {
      digitSeen = true;
      continue;
    }
    if (c == '.')
    {
      if (dotSeen) return false;
      dotSeen = true;
      continue;
    }
    if (c == ' ' || c == '\t')
    {
      // Trailing space only.
      for (const char* q = p; *q != '\0'; q++)
        if (*q != ' ' && *q != '\t') return false;
      break;
    }
    return false;
  }

  if (!digitSeen) return false;

  float value = atof(text);
  if (value < 0) return false;

  *outValue = value;
  return true;
}

// Split a colon-separated list of non-negative floats into out[]. Returns the
// number parsed, or -1 on any invalid/empty token or if there are more than maxOut.
// @p payload is modified in place.
int splitFloats(char* payload, float* out, int maxOut, bool requirePositive)
{
  int count = 0;
  char* cursor = payload;

  while (true)
  {
    char* sep = strchr(cursor, ':');
    if (sep != NULL) *sep = '\0';

    float value = 0.0;
    if (!parseNonNegativeFloat(cursor, &value)) return -1;
    if (requirePositive && value <= 0.0) return -1;
    if (count >= maxOut) return -1;

    out[count++] = value;

    if (sep == NULL) break;
    cursor = sep + 1;
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

// Volume a purge pump has pushed: purge has no target volume, so this is just
// elapsed time against the pump's flow constant.
float getPurgeEstimateCl(int idx, unsigned long nowMs)
{
  if (idx < 0 || idx >= MAX_PUMPS) return 0.0;
  if (!pumpRunning[idx] && pumpStartTime[idx] == 0) return 0.0;

  unsigned long elapsed = nowMs - pumpStartTime[idx];
  if (elapsed > pumpTargetDurationMs[idx]) elapsed = pumpTargetDurationMs[idx];

  float flow = (idx == 0) ? 0.0 : timeFlowClPerSec[idx];
  if (idx == 0)
  {
    noInterrupts();
    unsigned long pulses = pulseCount;
    interrupts();
    return getPump1MeasuredCl(pulses);
  }
  return (elapsed / 1000.0) * flow;
}

float measuredForPump(int idx, unsigned long currentPulses, unsigned long nowMs)
{
  if (runMode == RUN_MODE_PURGE) return getPurgeEstimateCl(idx, nowMs);
  return (idx == 0) ? getPump1MeasuredCl(currentPulses)
                    : getPumpTimeEstimateCl(idx, nowMs);
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
    Serial.print(measuredForPump(i, currentPulses, nowMs), 3);
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
    pumpPendingStart[i] = false;
    pumpStartAtMs[i] = 0;
  }

  Serial.begin(115200);
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
#if defined(__AVR__)
    wdt_reset();
#endif

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
      if (cmdOverflow)
      {
        // The line was truncated, so acting on it would be guesswork.
        replyError(0, "CMD_TOO_LONG");
        cmdOverflow = false;
        cmdLen = 0;
        continue;
      }
      if (cmdLen > 0)
      {
        cmdBuf[cmdLen] = '\0';
        handleCommand(cmdBuf);
        cmdLen = 0;
      }
    }
    else if (cmdLen + 1 < CMD_BUF_LEN)
    {
      cmdBuf[cmdLen++] = c;
    }
    else
    {
      cmdOverflow = true;
    }
  }
}

void handleCommand(char* cmd)
{
  // Strip surrounding whitespace.
  while (*cmd == ' ' || *cmd == '\t') cmd++;
  for (int i = (int)strlen(cmd) - 1; i >= 0 && (cmd[i] == ' ' || cmd[i] == '\t'); i--)
    cmd[i] = '\0';

  // Every command is "@<seq>:<verb>...".
  if (*cmd != '@')
  {
    replyError(0, "NO_SEQUENCE");
    return;
  }

  char* seqEnd = NULL;
  unsigned long seqVal = strtoul(cmd + 1, &seqEnd, 10);
  if (seqEnd == NULL || *seqEnd != ':' || seqVal > 65535UL)
  {
    replyError(0, "NO_SEQUENCE");
    return;
  }

  unsigned int seq = (unsigned int)seqVal;
  char* body = seqEnd + 1;

  if (strcmp(body, "STOP") == 0)
  {
    if (!dispensing)
    {
      replyError(seq, "NOT_DISPENSING");
      return;
    }

    if (runMode == RUN_MODE_CALIBRATION)
    {
      // Aborted by hand: do not fold this run into the stored constant.
      stopCalibration(false, false, NULL);
    }
    else
    {
      stopDispense("STOPPED", NULL);
    }
    return;
  }

  if (dispensing)
  {
    replyError(seq, "BUSY");
    return;
  }

  if (strncmp(body, "DISPENSE:", 9) == 0)
  {
    if (!isObjectDetected())
    {
      replyError(seq, "NO_OBJECT");
      return;
    }

    char* payload = body + 9;
    // Format: <cl>:<flow_const>:<cl>:<flow_const>... (2 fields per pump)
    float all_params[MAX_PUMPS * 2];
    int field_count = splitFloats(payload, all_params, MAX_PUMPS * 2, false);
    if (field_count < 2 || (field_count % 2) != 0)
    {
      replyError(seq, "INVALID_VOLUME:FIELDS");
      return;
    }

    int pump_count = field_count / 2;
    float vols[MAX_PUMPS];
    float flows[MAX_PUMPS];
    bool anyPositive = false;

    for (int i = 0; i < pump_count; i++)
    {
      vols[i]  = all_params[2 * i];
      flows[i] = all_params[2 * i + 1];
      if (vols[i] > 0)
      {
        if (flows[i] <= 0)
        {
          replyError(seq, "INVALID_VOLUME:FLOW");
          return;
        }
        anyPositive = true;
      }
    }
    if (!anyPositive)
    {
      replyError(seq, "INVALID_VOLUME:ALLZERO");
      return;
    }

    startDispense(seq, vols, flows, pump_count);
    return;
  }

  if (strncmp(body, "CAL:", 4) == 0)
  {
    if (!isObjectDetected())
    {
      replyError(seq, "NO_OBJECT");
      return;
    }

    char* payload = body + 4;
    char* sep = strchr(payload, ':');
    if (sep == NULL || strchr(sep + 1, ':') != NULL)
    {
      replyError(seq, "INVALID_CALIBRATION");
      return;
    }

    *sep = '\0';
    int pumpIndex = atoi(payload);
    float quantityCl = 0.0;
    if (pumpIndex < 1 || pumpIndex > MAX_PUMPS
        || !parseNonNegativeFloat(sep + 1, &quantityCl) || quantityCl <= 0.0)
    {
      replyError(seq, "INVALID_CALIBRATION");
      return;
    }

    startCalibration(seq, pumpIndex, quantityCl);
    return;
  }

  if (strncmp(body, "PURGE:", 6) == 0)
  {
    if (!isObjectDetected())
    {
      replyError(seq, "NO_OBJECT");
      return;
    }

    char* payload = body + 6;
    char* sep = strchr(payload, ':');
    if (sep == NULL || strchr(sep + 1, ':') != NULL)
    {
      replyError(seq, "INVALID_PURGE");
      return;
    }

    *sep = '\0';
    int pumpIndex = atoi(payload);
    long durationMs = atol(sep + 1);
    if (pumpIndex < 0 || pumpIndex > MAX_PUMPS || durationMs <= 0)
    {
      replyError(seq, "INVALID_PURGE");
      return;
    }

    startPurge(seq, pumpIndex, (unsigned long)durationMs);
    return;
  }

  replyError(seq, "UNKNOWN_COMMAND");
}

// Turn on any pump whose staggered start slot has arrived.
void serviceStaggeredStarts(unsigned long nowMs)
{
  for (int i = 0; i < MAX_PUMPS; i++)
  {
    if (!pumpPendingStart[i]) continue;
    if ((long)(nowMs - pumpStartAtMs[i]) < 0) continue;

    pumpPendingStart[i] = false;
    pumpStartTime[i] = nowMs;
    digitalWrite(pumpPins[i], HIGH);
    pumpRunning[i] = true;
    pumpDone[i] = false;
  }
}

void startDispense(unsigned int seq, float* volumes, float* flows, int count)
{
  // Refresh stored constants so calibration progress uses up-to-date values.
  pulsesPerCl = flows[0];
  for (int i = 1; i < count; i++)
    timeFlowClPerSec[i] = flows[i];

  runMode = RUN_MODE_DISPENSE;
  activePumpCount = count;
  activeSeq = seq;

  noInterrupts();
  pulseCount = 0;
  interrupts();

  unsigned long nowMs = millis();
  dispenseStartTime = nowMs;
  lastProgressReport = nowMs;
  maxDispenseTime = 0;

  int staggerSlot = 0;

  for (int i = 0; i < MAX_PUMPS; i++)
  {
    pumpPendingStart[i] = false;
    pumpStartAtMs[i] = 0;
    if (i >= count)
    {
      requestedCl[i] = 0.0;
      pumpTargetDurationMs[i] = 0;
      pumpRunning[i] = false;
      pumpDone[i] = true;
      continue;
    }

    requestedCl[i] = volumes[i];
    pumpStartTime[i] = nowMs;
    pumpTargetDurationMs[i] = 0;

    if (i == 0)
    {
      // Pump 1: flow-meter controlled.
      targetPulses = (unsigned long)(volumes[0] * pulsesPerCl);
      if (volumes[0] > 0)
      {
        float liters = volumes[0] / 100.0;
        maxDispenseTime = (unsigned long)((liters / expectedMinFlow_Lmin) * 60000.0 * 1.5);
        // First pump starts immediately; the rest are spread out below.
        pumpStartTime[0] = nowMs;
        digitalWrite(pumpPins[0], HIGH);
        pumpRunning[0] = true;
        pumpDone[0] = false;
        staggerSlot = 1;
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
      if (volumes[i] > 0 && flow > 0.0001)
      {
        pumpTargetDurationMs[i] = (unsigned long)((volumes[i] / flow) * 1000.0);
        // Deferred start, serviced from the tick: no delay() here, so STOP
        // stays reachable while the earlier pumps are already running.
        pumpPendingStart[i] = true;
        pumpStartAtMs[i] = nowMs + (unsigned long)staggerSlot * pumpStartStaggerMs;
        pumpStartTime[i] = pumpStartAtMs[i];
        pumpRunning[i] = false;
        pumpDone[i] = false;
        staggerSlot++;
      }
      else
      {
        digitalWrite(pumpPins[i], LOW);
        pumpRunning[i] = false;
        pumpDone[i] = true;
      }
    }
  }

  // The flow-meter timeout is measured from the first start; the staggered
  // pumps finish on their own durations, so give the window the stagger back.
  if (maxDispenseTime > 0)
  {
    maxDispenseTime += (unsigned long)staggerSlot * pumpStartStaggerMs;
  }

  dispensing = true;
  runWatchdogEnable();

  replyHeader(seq);
  Serial.print("STARTED");
  for (int i = 0; i < count; i++)
  {
    Serial.print(":");
    Serial.print(volumes[i], 3);
  }
  Serial.println();
}

void startPurge(unsigned int seq, int pumpIndex, unsigned long durationMs)
{
  // Clamped here as well as on the host: this is the value that actually
  // governs the relays, and it must be safe on its own.
  if (durationMs > PURGE_MAX_MS) durationMs = PURGE_MAX_MS;

  runMode = RUN_MODE_PURGE;
  activeSeq = seq;
  purgeDurationMs = durationMs;

  noInterrupts();
  pulseCount = 0;
  interrupts();

  unsigned long nowMs = millis();
  dispenseStartTime = nowMs;
  lastProgressReport = nowMs;
  maxDispenseTime = durationMs + 2000UL;   // hard ceiling for the whole run
  targetPulses = 0;                        // purge is time-driven, not metered

  bool all = (pumpIndex == 0);
  int staggerSlot = 0;
  activePumpCount = all ? MAX_PUMPS : pumpIndex;

  for (int i = 0; i < MAX_PUMPS; i++)
  {
    bool selected = all || (i == pumpIndex - 1);

    pumpPendingStart[i] = false;
    pumpStartAtMs[i] = 0;
    requestedCl[i] = 0.0;
    pumpStartTime[i] = nowMs;

    if (!selected)
    {
      digitalWrite(pumpPins[i], LOW);
      pumpTargetDurationMs[i] = 0;
      pumpRunning[i] = false;
      pumpDone[i] = true;
      continue;
    }

    pumpTargetDurationMs[i] = durationMs;
    pumpDone[i] = false;

    if (staggerSlot == 0)
    {
      digitalWrite(pumpPins[i], HIGH);
      pumpRunning[i] = true;
    }
    else
    {
      pumpPendingStart[i] = true;
      pumpStartAtMs[i] = nowMs + (unsigned long)staggerSlot * pumpStartStaggerMs;
      pumpStartTime[i] = pumpStartAtMs[i];
      pumpRunning[i] = false;
    }
    staggerSlot++;
  }

  dispensing = true;
  runWatchdogEnable();

  replyHeader(seq);
  Serial.print("STARTED:");
  Serial.println(durationMs);
}

// @p errorCode non-NULL turns the terminal reply into an ERROR instead of a
// normal completion, so the host learns why a pour was cut short.
void stopDispense(const char* resultMsg, const char* errorCode)
{
  unsigned long nowMs = millis();
  for (int i = 0; i < MAX_PUMPS; i++)
  {
    digitalWrite(pumpPins[i], LOW);
    pumpRunning[i] = false;
    pumpDone[i] = true;
    pumpPendingStart[i] = false;
  }
  dispensing = false;
  runWatchdogDisable();

  noInterrupts();
  unsigned long currentPulses = pulseCount;
  interrupts();

  int n = activePumpCount > 0 ? activePumpCount : 1;
  unsigned int seq = activeSeq;

  // A run that had already started reports ABORTED, not ERROR: the pumps did
  // move liquid, and the host has to be able to account for it.
  replyHeader(seq);
  if (errorCode != NULL)
  {
    Serial.print("ABORTED:");
    Serial.print(errorCode);
  }
  else
  {
    Serial.print(resultMsg);
  }
  for (int i = 0; i < n; i++)
  {
    Serial.print(":");
    Serial.print(measuredForPump(i, currentPulses, nowMs), 3);
  }
  Serial.println();

  runMode = RUN_MODE_IDLE;
  activePumpCount = 0;
  activeSeq = 0;
  purgeDurationMs = 0;
}

void startCalibration(unsigned int seq, int pumpIndex, float targetCentiliters)
{
  runMode = RUN_MODE_CALIBRATION;
  dispensing = true;
  activeSeq = seq;
  calibrationPumpIndex = pumpIndex;
  calibrationTargetCl = targetCentiliters;
  calibrationStartTime = millis();
  lastProgressReport = calibrationStartTime;
  activePumpCount = 0;

  // Bound the run from the expected duration at the current constant, so a
  // sensor that never reports the glass leaving cannot pump indefinitely.
  int idx = pumpIndex - 1;
  float expectedSec;
  if (pumpIndex == 1)
  {
    // No flow-rate constant for the metered pump; assume the slowest flow the
    // safety timeout is sized for.
    expectedSec = (targetCentiliters / 100.0) / expectedMinFlow_Lmin * 60.0;
  }
  else
  {
    float flow = timeFlowClPerSec[idx];
    expectedSec = (flow > 0.0001) ? (targetCentiliters / flow) : 0.0;
  }

  unsigned long bound = (unsigned long)(expectedSec * 3000.0);  // 3x, in ms
  if (bound < calibrationFloorMs)   bound = calibrationFloorMs;
  if (bound > calibrationCeilingMs) bound = calibrationCeilingMs;
  calibrationMaxMs = bound;

  noInterrupts();
  pulseCount = 0;
  interrupts();

  for (int i = 0; i < MAX_PUMPS; i++)
  {
    digitalWrite(pumpPins[i], LOW);
    pumpRunning[i] = false;
    pumpDone[i] = false;
    requestedCl[i] = 0.0;
    pumpPendingStart[i] = false;
  }

  digitalWrite(pumpPins[idx], HIGH);
  pumpRunning[idx] = true;
  pumpStartTime[idx] = calibrationStartTime;

  runWatchdogEnable();

  replyHeader(seq);
  Serial.print("CAL_STARTED:");
  Serial.print(pumpIndex);
  Serial.print(":");
  Serial.println(targetCentiliters, 3);
}

// @p stoppedBySensor records whether the glass leaving ended the run, which is
// the only case the measurement is trusted for.
// @p errorCode non-NULL reports a failure instead of a constant.
void stopCalibration(bool stoppedBySensor, bool applyCalibration, const char* errorCode)
{
  unsigned long nowMs = millis();
  for (int i = 0; i < MAX_PUMPS; i++)
  {
    digitalWrite(pumpPins[i], LOW);
    pumpRunning[i] = false;
    pumpDone[i] = true;
    pumpPendingStart[i] = false;
  }

  dispensing = false;
  runWatchdogDisable();

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
    newConstant = (idx >= 0 && idx < MAX_PUMPS) ? timeFlowClPerSec[idx]
                                                : defaultPump2FlowClPerSec;
    if (applyCalibration && calibrationTargetCl > 0.0 && elapsedSec > 0.05)
    {
      newConstant = calibrationTargetCl / elapsedSec;
      if (idx >= 0 && idx < MAX_PUMPS) timeFlowClPerSec[idx] = newConstant;
    }
  }

  unsigned int seq = activeSeq;
  replyHeader(seq);
  if (errorCode != NULL)
  {
    Serial.print("ERROR:");
    Serial.println(errorCode);
  }
  else
  {
    Serial.print("CAL_DONE:");
    Serial.print(calibrationPumpIndex);
    Serial.print(":");
    Serial.print(newConstant, 3);
    Serial.print(":");
    Serial.println(stoppedBySensor ? 1 : 0);
  }

  runMode = RUN_MODE_IDLE;
  calibrationPumpIndex = 0;
  calibrationTargetCl = 0.0;
  calibrationStartTime = 0;
  calibrationMaxMs = 0;
  activeSeq = 0;
}

void processDispenseTick()
{
  unsigned long nowMs = millis();

  if (!isObjectDetected())
  {
    stopDispense(NULL, "NO_OBJECT");
    return;
  }

  serviceStaggeredStarts(nowMs);

  noInterrupts();
  unsigned long currentPulses = pulseCount;
  interrupts();

  if (runMode == RUN_MODE_PURGE)
  {
    // Purge is purely duration-driven.
    for (int i = 0; i < MAX_PUMPS; i++)
    {
      if (pumpRunning[i] && (nowMs - pumpStartTime[i] >= pumpTargetDurationMs[i]))
      {
        digitalWrite(pumpPins[i], LOW);
        pumpRunning[i] = false;
        pumpDone[i] = true;
      }
    }

    if (nowMs - dispenseStartTime > maxDispenseTime)
    {
      stopDispense("OK", NULL);
      return;
    }

    bool allDone = true;
    for (int i = 0; i < MAX_PUMPS; i++)
      if (pumpPendingStart[i] || pumpRunning[i]) allDone = false;

    if (allDone)
    {
      stopDispense("OK", NULL);
    }
    else if (nowMs - lastProgressReport >= 250)
    {
      lastProgressReport = nowMs;
      printProgress(currentPulses, nowMs);
    }
    return;
  }

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
      stopDispense(NULL, "TIMEOUT");
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
    if (!pumpDone[i] || pumpPendingStart[i])
    {
      allDone = false;
    }
  }

  if (allDone)
  {
    stopDispense("OK", NULL);
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
    // The glass leaving is the intended end of a calibration run.
    stopCalibration(true, true, NULL);
    return;
  }

  if (nowMs - calibrationStartTime > calibrationMaxMs)
  {
    // Ran far past the expected duration: the IR sensor is stuck, unplugged or
    // permanently blocked. Shut the pump down and keep the old constant.
    stopCalibration(false, false, "CAL_TIMEOUT");
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
// Called in loop() only when not dispensing, so the 50 ms read timeout
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
