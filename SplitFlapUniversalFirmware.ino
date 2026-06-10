// ==============================================================================
// Split-Flap Display Module Firmware — v12
// ==============================================================================
// Each module controls one character cell driven by a 28BYJ-48 stepper motor.
// A Hall effect sensor detects a magnet on the reel to find the home position.
// Modules communicate over a shared RS-485 half-duplex serial bus.
//
// ==============================================================================
// MESSAGE FORMAT  (backward-compatible with v6 and all later versions)
// ==============================================================================
//
// Every message begins with 'm', followed by the module address, then a command
// letter, then optional payload, then '\n' (or a 50 ms idle timeout).
//
//   m  <ADDR>  <CMD>  [data]  \n
//
// ADDR formats accepted:
//   Two-digit zero-padded decimal  "m38"  "m05"   — v6-style (primary)
//   Variable-length decimal        "m5"   "m138"  — v7+-style (also accepted)
//   Two stars                      "m**"           — v6 broadcast
//   One star                       "m*"            — v7+ broadcast (also accepted)
//   Literal 'X'                    "mX"            — provisioning address (v8+)
//
// Address parsing works as a simple accumulator: digits accumulate into idBuffer,
// '*' sets the wildcard flag, 'X' sets the provisioning flag.  The first character
// that is not a digit / '*' / 'X' is treated as the command letter.  This means
// all of the following address formats work without any special cases:
//   m**h   m*h   m38h   m038h   m5h   m138h   mXH...
//
// ==============================================================================
// COMMANDS  (all v6 commands preserved exactly)
// ==============================================================================
//
//  -<char>   Display the character <char> (single byte, from FLAP_CHARS)
//  +<n>      Display flap at index <n>  (v7+ only; not sent by v6 code)
//  h         Home the reel
//  c         Calibrate revolution length and report result
//  o<n>      Set home offset (steps past Hall trigger to flap 0)
//  t<n>      Set total steps per revolution
//  s<n>      Nudge forward n steps and add to home offset
//  g<n>      Go to raw step position n
//  w<i>:<p>  Write calibrated position p for flap index i
//  i<n>      Set module ID to n
//  a<n>      Set auto-home flag (1=home on boot, 0=restore saved position)
//  d         Dump EEPROM config
//  e         Erase calibrated flap position map
//  R         Reset provisioning (erase ID, resume advertising)  [v9+]
//  v         Report firmware version and serial number  → replies m<id>v:<version>:<moduleId>:<serialNumber>\n
//  F         Factory-reset EEPROM defaults (preserves module ID and magic byte)
//
// Provisioning commands (address mX, all modules respond):
//  mXH<sn>         Home by serial number
//  mXI<sn>:<id>    Assign ID by serial number  → replies mXack:<sn>:<id>
//
// Unprovisioned modules (ID==255) advertise every ~10-15 s:
//   mXadv:<serialNumber>
//
// ==============================================================================
// EEPROM LAYOUT  (identical to v6 — no structural changes ever made)
// ==============================================================================
//
//  Addr  Size  Contents
//  0x00   1    Magic / version byte
//  0x01   2    Home offset (steps from Hall trigger to flap 0)
//  0x03   2    Total steps per revolution
//  0x05   1    Module bus ID (0-254; 255 = unprovisioned)
//  0x06   1    Auto-home flag (1 = home on boot)
//  0x07   2    Saved step position (used when auto-home is off)
//  0x09   1    Saved flap index    (used when auto-home is off)
//  0x0A   2    (reserved / padding)
//  0x0C  128   Calibrated step positions: 64 × uint16_t, 0xFFFF = uncalibrated
//
// Magic byte history — all recognised and migrated to 0x5D:
//   0x5D  v6 original                  → load all fields as-is (already current)
//   0x5E  v8/v9 (erroneous bump)       → same field layout; load fields, write 0x5D
//   other blank chip or unknown        → write defaults, leave ID as 255, write 0x5D
//
// ==============================================================================
// CHANGE LOG
// ==============================================================================
//   v6  — Original release.  Hardcoded ID per flash.  2-char fixed address parser.
//   v7  — Variable-length address parser; '+' index command; moveToIndex().
//   v8  — Dynamic IDs via SIGROW serial number; advertisement; H/I provisioning.
//          BUG: INIT_VALUE bumped to 0x5E, wiping v6 EEPROM on upgrade.
//   v9  — 'R' de-provisioning command.
//   v10 — Attempted EEPROM fix (incomplete: missed 0x5E case and introduced 0x5C).
//          CSMA advertisement with per-interval random jitter.
//   v11 — Complete EEPROM fix: 0x5D (v6) and 0x5E (v8/v9) both migrate cleanly.
//          Removed erroneous 0x5C magic.  Address parser made fully backward-
//          compatible with v6 two-char zero-padded format.  All v6 commands
//          preserved with identical behaviour.
//   v12 — New 'v' command: report firmware version string.
//          New 'F' command: factory-reset all EEPROM calibration values to
//          defaults while preserving the module ID and the magic byte.
// ==============================================================================

#include <SoftwareSerial.h>
#include <EEPROM.h>

// ==========================================
//            CONFIGURATION
// ==========================================
const String FLAP_CHARS =
  " ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789!@#$&()-+=;q:%'.,/?*roygbpw";

// Advertisement timing
const unsigned long ADVERTISE_BASE_MS   = 10000UL; // base interval
const unsigned long ADVERTISE_JITTER_MS =  5000UL; // max extra jitter

// Carrier-sense window before transmitting (ms)
const unsigned long CSMA_LISTEN_MS = 20UL;

// ---- EEPROM addresses (unchanged from v6) ----
const int ADDR_INIT        = 0;
const int ADDR_HOME_OFFSET = 1;
const int ADDR_TOTAL_STEPS = 3;
const int ADDR_MODULE_ID   = 5;
const int ADDR_AUTO_HOME   = 6;
const int ADDR_SAVED_POS   = 7;
const int ADDR_SAVED_INDEX = 9;
const int ADDR_MAP_START   = 12;

// The one valid magic value for the current layout (same as v6).
// 0x5E (written by v8/v9) is treated as equivalent during migration.
const uint8_t EEPROM_MAGIC = 0x5D;
const uint8_t EEPROM_MAGIC_V8V9 = 0x5E; // recognise but migrate away from

// Firmware version string returned by the 'v' command.
const char FIRMWARE_VERSION[] = "12";

// ── Deferred reply type — must be declared before any function that uses it ──
// The Arduino IDE auto-generates forward declarations for all functions; if the
// enum is defined inside a function body or after a function that references it
// in its signature the auto-generated prototype will fail to compile.
enum PendingReply { REPLY_NONE, REPLY_VERSION, REPLY_DUMP };

const unsigned long REPLY_DIRECT_MS = 10UL;  // settling delay for direct-addressed replies
const unsigned long REPLY_SLOT_MS   = 80UL;  // per-ID slot width for broadcast replies
                                              // must be > worst-case TX frame time (~45 ms at 9600 baud)

// ==========================================
//        ATtiny1616 SERIAL NUMBER
// ==========================================
#define SIGROW_BASE   0x1100
#define SERIAL_OFFSET 3
#define SERIAL_LEN    10

char serialStr[SERIAL_LEN * 2 + 1];

void readSerialNumber() {
  for (uint8_t i = 0; i < SERIAL_LEN; i++) {
    uint8_t b  = _MMIO_BYTE(SIGROW_BASE + SERIAL_OFFSET + i);
    uint8_t hi = (b >> 4) & 0x0F;
    uint8_t lo =  b       & 0x0F;
    serialStr[i * 2]     = hi < 10 ? ('0' + hi) : ('A' + hi - 10);
    serialStr[i * 2 + 1] = lo < 10 ? ('0' + lo) : ('A' + lo - 10);
  }
  serialStr[SERIAL_LEN * 2] = '\0';
}

bool serialMatches(const String &candidate) {
  if (candidate.length() != (unsigned)(SERIAL_LEN * 2)) return false;
  for (uint8_t i = 0; i < SERIAL_LEN * 2; i++) {
    char a = serialStr[i], b = candidate[i];
    if (a >= 'a') a -= 32;
    if (b >= 'a') b -= 32;
    if (a != b) return false;
  }
  return true;
}

// ==========================================
//       PSEUDO-RANDOM (LCG, seeded from SN)
// ==========================================
static uint32_t _lcgState = 0;

void seedRandom() {
  uint32_t seed = 0;
  for (uint8_t i = 0; i < SERIAL_LEN; i++) {
    seed ^= (uint32_t)_MMIO_BYTE(SIGROW_BASE + SERIAL_OFFSET + i) << ((i & 3) * 8);
    seed  = (seed << 5) | (seed >> 27);
  }
  _lcgState = seed ? seed : 0xDEADBEEFUL;
}

uint32_t lcgRand(uint32_t range) {
  _lcgState = _lcgState * 1664525UL + 1013904223UL;
  return _lcgState % range;
}

// ==========================================
//           OPERATING STATE
// ==========================================
uint8_t moduleId        = 255;
bool    autoHomeEnabled = true;
int     stepsFromHallToZero = 2832;
int     totalStepsPerRev    = 4096;

// idChars: two-character zero-padded decimal ID used in outbound responses.
// Kept for response format compatibility with v6 tooling (IDs 0-99).
// IDs 100-254 use printModuleId() which outputs the full decimal string.
char idChars[2] = {'*', '*'};

void updateIdChars() {
  if (moduleId < 100) {
    idChars[0] = (moduleId / 10) + '0';
    idChars[1] = (moduleId % 10) + '0';
  }
  // IDs >= 100: idChars unused; responses use printModuleId() directly.
}

// ==========================================
//          PIN DEFINITIONS
// ==========================================
const int RS485_RX = 3;
const int RS485_TX = 1;
const int RS485_DE = 2;

#define IN1 9
#define IN2 8
#define IN3 7
#define IN4 6
#define HALL_PIN 4

// ==========================================
//               GLOBALS
// ==========================================
SoftwareSerial rs485(RS485_RX, RS485_TX);

long currentStepPos   = 0;
int  currentPhase     = 0;
int  parseState       = 0;
int  currentFlapIndex = -1;
int  tempIndex        = -1;

const int stepDelay = 1;

String buffer      = "";
String idBuffer    = "";
bool   idWildcard  = false;
bool   idProvision = false;

// ── Non-blocking deferred reply ───────────────────────────────────────────────
// When a command requires a response (v, d) the parser does NOT transmit
// immediately.  Instead it records what to send and when, then returns so
// the main loop keeps draining the RX buffer.  The actual transmission
// happens in loop() once pendingReplyTime has elapsed.
//
// For direct-addressed commands the delay is a short fixed settling time
// (REPLY_DIRECT_MS) so the Pi has time to switch its transceiver to RX.
// For broadcast commands each module uses its ID as a time-slot index
// (moduleId × REPLY_SLOT_MS) so replies arrive sequentially with no
// overlap.  While waiting, the module keeps reading serial normally.

PendingReply  pendingReply     = REPLY_NONE;
unsigned long pendingReplyTime = 0;

// Schedule a deferred reply.  isBroadcast=true uses slot-based timing.
void schedulePendingReply(PendingReply type, bool isBroadcast) {
  unsigned long delay_ms;
  if (isBroadcast) {
    delay_ms = (moduleId != 255)
      ? (unsigned long)moduleId * REPLY_SLOT_MS
      : (unsigned long)_MMIO_BYTE(SIGROW_BASE + SERIAL_OFFSET + SERIAL_LEN - 1) * REPLY_SLOT_MS;
  } else {
    delay_ms = REPLY_DIRECT_MS;
  }
  pendingReply     = type;
  pendingReplyTime = millis() + delay_ms;
}

unsigned long lastSerialTime    = 0;
unsigned long nextAdvertiseTime = 0;

String snBuffer    = "";
bool   snColonSeen = false;

const uint8_t halfStepSequence[8][4] = {
  {1,0,0,0}, {1,1,0,0}, {0,1,0,0}, {0,1,1,0},
  {0,0,1,0}, {0,0,1,1}, {0,0,0,1}, {1,0,0,1}
};

// ==========================================
//          EEPROM HELPERS
// ==========================================
void saveHomeOffset() { EEPROM.put(ADDR_HOME_OFFSET, stepsFromHallToZero); }
void saveTotalSteps()  { EEPROM.put(ADDR_TOTAL_STEPS, totalStepsPerRev); }

void saveState() {
  if (!autoHomeEnabled) {
    EEPROM.put(ADDR_SAVED_POS,   currentStepPos);
    EEPROM.put(ADDR_SAVED_INDEX, currentFlapIndex);
  }
}

// ==========================================
//          RESPONSE HELPERS
// ==========================================
// Print module ID in outbound messages.
// IDs 0-99: always two chars with leading zero, matching v6 format.
// IDs 100-254: three chars (v7+ controllers expect this).
void printModuleId() {
  if (moduleId < 10)       rs485.print("0");
  // (for IDs 10-99 no padding needed; for 100+ no padding needed either)
  rs485.print(moduleId);
}

void dumpEeprom() {
  digitalWrite(RS485_DE, HIGH);
  delay(2);

  rs485.print("m");
  printModuleId();
  rs485.print("d:");
  rs485.print(stepsFromHallToZero);
  rs485.print(":");
  rs485.print(totalStepsPerRev);
  rs485.print(":");

  bool first = true;
  for (int i = 0; i < 64; i++) {
    uint16_t pos = 0xFFFF;
    EEPROM.get(ADDR_MAP_START + (i * 2), pos);
    if (pos != 0xFFFF) {
      if (!first) rs485.print(",");
      rs485.print(i); rs485.print("="); rs485.print(pos);
      first = false;
    }
  }

  rs485.print("\n");
  delay(45);   // wait for UART to finish clocking out at 9600 baud
  digitalWrite(RS485_DE, LOW);
  // Flush echo bytes received while DE was HIGH.
  while (rs485.available()) rs485.read();
}

// ==========================================
//       ADVERTISEMENT  (unprovisioned)
// ==========================================
void scheduleNextAdvertisement() {
  nextAdvertiseTime = millis() + ADVERTISE_BASE_MS + lcgRand(ADVERTISE_JITTER_MS);
}

// Carrier-sense then transmit.  Returns true if sent, false if bus was busy.
bool sendAdvertisement() {
  while (rs485.available()) rs485.read();   // drain stale bytes

  unsigned long listenEnd = millis() + CSMA_LISTEN_MS;
  while (millis() < listenEnd) {
    if (rs485.available()) return false;    // bus busy — defer
  }

  digitalWrite(RS485_DE, HIGH);
  delay(2);

  rs485.print("mXadv:");
  rs485.print(serialStr);
  rs485.print("\n");

  delay(45);   // wait for UART to finish clocking out at 9600 baud
  digitalWrite(RS485_DE, LOW);
  // Flush echo bytes received while DE was HIGH.
  while (rs485.available()) rs485.read();
  return true;
}

// ==========================================
//         PROVISIONING RESET
// ==========================================
void resetProvisioning() {
  moduleId = 255;
  EEPROM.write(ADDR_MODULE_ID, 255);
  // Schedule first advertisement soon
  nextAdvertiseTime = millis() + lcgRand(ADVERTISE_JITTER_MS);
}

// ==========================================
//         VERSION RESPONSE
// ==========================================
// Reply format:  m<id>v:<version>:<moduleId>:<serialNumber>\n
// Example:       m38v:12:38:A3F24C0018E7D29B3F01\n
// Unprovisioned: m255v:12:255:A3F24C0018E7D29B3F01\n
//
// Called by loop() after the deferred reply timer fires — never called
// directly from the parser so there are no blocking delays in the RX path.
void sendVersionResponse() {
  digitalWrite(RS485_DE, HIGH);
  delay(2);
  rs485.print("m");
  printModuleId();
  rs485.print("v:");
  rs485.print(FIRMWARE_VERSION);
  rs485.print(":");
  rs485.print(moduleId);
  rs485.print(":");
  rs485.print(serialStr);
  rs485.print("\n");
  delay(45);   // wait for UART to finish clocking out at 9600 baud
  digitalWrite(RS485_DE, LOW);
  // Flush any bytes that were echoed into the RX buffer while DE was HIGH.
  // Without this the parser would attempt to parse our own transmission.
  while (rs485.available()) rs485.read();
}

// ==========================================
//       EEPROM FACTORY RESET
// ==========================================
// Resets all calibration and configuration values to their defaults while
// preserving the module ID (so the module stays on the bus) and the magic
// byte (so the next boot recognises the EEPROM as initialised).
//
// Values reset:
//   home offset      → 2832
//   total steps      → 4096
//   auto-home flag   → 1 (enabled)
//   saved step pos   → 0
//   saved flap index → 0
//   flap position map → all 0xFFFF (uncalibrated)
//
// Values preserved:
//   EEPROM magic byte (ADDR_INIT)
//   module ID (ADDR_MODULE_ID)
void resetEepromDefaults() {
  stepsFromHallToZero = 2832;
  totalStepsPerRev    = 4096;
  autoHomeEnabled     = true;

  saveHomeOffset();
  saveTotalSteps();
  EEPROM.write(ADDR_AUTO_HOME, 1);

  // Clear saved position
  long zeroPosL = 0;
  int8_t zeroIdx = 0;
  EEPROM.put(ADDR_SAVED_POS,   zeroPosL);
  EEPROM.put(ADDR_SAVED_INDEX, zeroIdx);

  // Erase the entire calibrated flap map
  for (int i = 0; i < 64; i++) {
    uint16_t empty = 0xFFFF;
    EEPROM.put(ADDR_MAP_START + (i * 2), empty);
  }
}

// ==========================================
//           MOTOR FUNCTIONS
// ==========================================
void applyStep(const uint8_t *step) {
  digitalWrite(IN1, step[0]); digitalWrite(IN2, step[1]);
  digitalWrite(IN3, step[2]); digitalWrite(IN4, step[3]);
}

void stepBackward(int steps) {
  static bool lastHallState = false;
  for (int k = 0; k < steps; k++) {
    bool hallNow = hallActive();
    if (hallNow && !lastHallState)
      currentStepPos = totalStepsPerRev - stepsFromHallToZero;
    lastHallState = hallNow;

    if (--currentPhase < 0) currentPhase = 7;
    applyStep(halfStepSequence[currentPhase]);
    delay(stepDelay);

    if (++currentStepPos >= totalStepsPerRev) currentStepPos = 0;
  }
}

void releaseMotor() {
  digitalWrite(IN1,0); digitalWrite(IN2,0);
  digitalWrite(IN3,0); digitalWrite(IN4,0);
}

bool hallActive() { return digitalRead(HALL_PIN) == LOW; }

// ==========================================
//           LOGIC FUNCTIONS
// ==========================================
void homeModule() {
  long safety = 0;
  while (!hallActive() && safety < (totalStepsPerRev + 500)) {
    stepBackward(1); safety++;
  }
  stepBackward(stepsFromHallToZero);
  currentStepPos = 0; currentFlapIndex = 0;
  releaseMotor();
}

void calibrateModule() {
  long safety = 0;
  while (hallActive()  && safety < 4000) { stepBackward(1); safety++; delay(5); }
  safety = 0;
  while (!hallActive() && safety < 5000) { stepBackward(1); safety++; }
  while ( hallActive())                  { stepBackward(1); }

  int measuredSteps = 0;
  while (!hallActive() && measuredSteps < 5000) { stepBackward(1); measuredSteps++; }
  while ( hallActive())                         { stepBackward(1); measuredSteps++; }

  delay(50);
  digitalWrite(RS485_DE, HIGH);
  delay(10);

  // Response format matches v6: m<2-char-id>:<steps>\n for IDs 0-99
  rs485.print("m");
  printModuleId();
  rs485.print(":");
  rs485.print(measuredSteps);
  rs485.print("\n");

  delay(45);
  digitalWrite(RS485_DE, LOW);
  while (rs485.available()) rs485.read();

  totalStepsPerRev = measuredSteps;
  saveTotalSteps();
  homeModule();
  saveState();
}

void moveToIndex(int targetIndex) {
  if (targetIndex < 0 || targetIndex >= (int)FLAP_CHARS.length()) return;
  if (currentFlapIndex == targetIndex) return;
  if (currentFlapIndex == -1) homeModule();

  uint16_t mappedPos = 0xFFFF;
  EEPROM.get(ADDR_MAP_START + (targetIndex * 2), mappedPos);

  long targetStepPos = (mappedPos != 0xFFFF)
    ? (long)mappedPos
    : ((long)targetIndex * (long)totalStepsPerRev) / 64;

  long stepsToMove = targetStepPos - currentStepPos;
  if (stepsToMove < 0) stepsToMove += totalStepsPerRev;
  while (stepsToMove-- > 0) stepBackward(1);

  releaseMotor();
  currentFlapIndex = targetIndex;
  saveState();
}

void moveToChar(char targetChar) {
  int idx = FLAP_CHARS.indexOf(targetChar);
  if (idx != -1) moveToIndex(idx);
}

// ==========================================
//               SETUP
// ==========================================
void setup() {
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(HALL_PIN, INPUT_PULLUP);
  pinMode(RS485_DE, OUTPUT);
  digitalWrite(RS485_DE, LOW);

  readSerialNumber();
  seedRandom();
  rs485.begin(9600);

  // ── EEPROM load / migrate / init ──────────────────────────────────────────
  // The physical EEPROM field layout has been identical since v6.
  // Only the magic byte in ADDR_INIT has varied across versions:
  //   0x5D  written by v6 (and now again by v11)  → load all fields
  //   0x5E  written by v8/v9 (erroneous bump)     → same layout; load, then fix magic
  //   other (0xFF on blank chip, or unknown)       → write fresh defaults
  //
  // In all three cases the module ends up with 0x5D in ADDR_INIT after setup().

  uint8_t magic = EEPROM.read(ADDR_INIT);

  if (magic == EEPROM_MAGIC || magic == EEPROM_MAGIC_V8V9) {
    // Valid data exists — load everything.
    EEPROM.get(ADDR_HOME_OFFSET, stepsFromHallToZero);
    EEPROM.get(ADDR_TOTAL_STEPS, totalStepsPerRev);
    moduleId        = EEPROM.read(ADDR_MODULE_ID);
    autoHomeEnabled = (EEPROM.read(ADDR_AUTO_HOME) == 1);

    // If this was a v8/v9 magic, normalise it now so future boots are clean.
    if (magic == EEPROM_MAGIC_V8V9) {
      EEPROM.write(ADDR_INIT, EEPROM_MAGIC);
    }
  } else {
    // Blank chip or unrecognised magic — write defaults.
    // Module ID intentionally left as 255 (unprovisioned).
    EEPROM.write(ADDR_INIT, EEPROM_MAGIC);
    saveHomeOffset();
    saveTotalSteps();
    EEPROM.write(ADDR_MODULE_ID, 255);
    EEPROM.write(ADDR_AUTO_HOME, 1);
    autoHomeEnabled = true;
    moduleId        = 255;

    for (int i = 0; i < 64; i++) {
      uint16_t empty = 0xFFFF;
      EEPROM.put(ADDR_MAP_START + (i * 2), empty);
    }
  }

  updateIdChars();

  // ── Staggered startup ─────────────────────────────────────────────────────
  // Provisioned: deterministic stagger by ID so motors spread evenly.
  // Unprovisioned: random stagger from serial number so a batch of fresh
  //   modules doesn't hammer current simultaneously.
  if (moduleId != 255) {
    delay((unsigned long)moduleId * 150UL);
  } else {
    delay(lcgRand(10000UL));
  }

  // ── Home or restore ───────────────────────────────────────────────────────
  if (autoHomeEnabled) {
    homeModule();
    saveState();
  } else {
    EEPROM.get(ADDR_SAVED_POS, currentStepPos);
    if (currentStepPos >= (long)totalStepsPerRev) currentStepPos = 0;
    currentFlapIndex = (int8_t)EEPROM.read(ADDR_SAVED_INDEX);
  }

  // Schedule first advertisement at a random offset within the first interval.
  scheduleNextAdvertisement();
}

// ==========================================
//               MAIN LOOP
// ==========================================
// Address parser state machine
// ─────────────────────────────
// parseState 0  : idle, waiting for 'm'
// parseState 1  : reading address field (digits / '*' / 'X'); dispatches on CMD
// parseState 4  : '-' cmd  — reading one display character
// parseState 5  : 'o' cmd  — home offset digits
// parseState 6  : 't' cmd  — total steps digits
// parseState 7  : 's' cmd  — nudge steps digits
// parseState 8  : 'g' cmd  — raw step position digits
// parseState 9  : 'w' cmd  — index ':' position digits
// parseState 10 : 'i' cmd  — new module ID digits
// parseState 11 : 'a' cmd  — auto-home flag digit
// parseState 12 : '+' cmd  — flap index digits  (v7+)
// parseState 13 : 'H' prov — serial number hex digits
// parseState 14 : 'I' prov — serial number ':' new ID digits

void loop() {

  // ── Deferred reply (v, d commands) ───────────────────────────────────────
  // The parser schedules replies rather than sending them inline, so the RX
  // buffer keeps draining while we wait for our time slot.
  if (pendingReply != REPLY_NONE && (long)(millis() - pendingReplyTime) >= 0) {
    switch (pendingReply) {
      case REPLY_VERSION: sendVersionResponse(); break;
      case REPLY_DUMP:    dumpEeprom();          break;
      default: break;
    }
    pendingReply = REPLY_NONE;
  }

  // ── Advertisement heartbeat (unprovisioned only) ──────────────────────────
  if (moduleId == 255 && (long)(millis() - nextAdvertiseTime) >= 0) {
    sendAdvertisement();       // CSMA inside; silently drops if bus busy
    scheduleNextAdvertisement();
  }

  // ── Incoming serial ───────────────────────────────────────────────────────
  while (rs485.available()) {
    char c = rs485.read();
    lastSerialTime = millis();

    switch (parseState) {

      // ── State 0: wait for 'm' ──────────────────────────────────────────
      case 0:
        if (c == 'm') {
          idBuffer = ""; idWildcard = false; idProvision = false;
          parseState = 1;
        }
        break;

      // ── State 1: accumulate address, then dispatch ─────────────────────
      // Accepts any mix of digits, '*', and 'X'.
      // Leading zeros in IDs (v6 format: "m05") are handled naturally because
      // "05".toInt() == 5, which equals moduleId when moduleId == 5.
      case 1:
        if      (c == '*')      { idWildcard  = true; }
        else if (c == 'X')      { idProvision = true; }
        else if (isDigit(c))    { if (!idWildcard && !idProvision) idBuffer += c; }
        else {
          // c is the command letter.
          bool match = idWildcard ||
                       (idBuffer.length() > 0 &&
                        (uint8_t)idBuffer.toInt() == moduleId);

          if (idProvision) {
            // Provisioning commands: every module listens, SN check is inside.
            if      (c == 'H') { snBuffer = "";
                                  parseState = 13; }
            else if (c == 'I') { snBuffer = ""; snColonSeen = false; buffer = "";
                                  parseState = 14; }
            else                 parseState = 0;

          } else if (match) {
            // Normal addressed / broadcast commands
            switch (c) {
              case '-': parseState = 4; break;
              case '+': buffer = ""; parseState = 12; break;
              case 'h': homeModule(); saveState(); parseState = 0; break;
              case 'c': calibrateModule();          parseState = 0; break;
              case 'o': buffer = ""; parseState = 5; break;
              case 't': buffer = ""; parseState = 6; break;
              case 's': buffer = ""; parseState = 7; break;
              case 'g': buffer = ""; parseState = 8; break;
              case 'w': buffer = ""; tempIndex = -1; parseState = 9; break;
              case 'i': buffer = ""; parseState = 10; break;
              case 'a': buffer = ""; parseState = 11; break;
              case 'd': schedulePendingReply(REPLY_DUMP,    idWildcard); parseState = 0; break;
              case 'R': resetProvisioning();                              parseState = 0; break;
              case 'v': schedulePendingReply(REPLY_VERSION, idWildcard); parseState = 0; break;
              case 'F': resetEepromDefaults();                            parseState = 0; break;
              case 'e':
                for (int i = 0; i < 64; i++) {
                  uint16_t empty = 0xFFFF;
                  EEPROM.put(ADDR_MAP_START + (i * 2), empty);
                }
                parseState = 0;
                break;
              default: parseState = 0; break;
            }
          } else {
            parseState = 0;
          }

          idBuffer = ""; idWildcard = false; idProvision = false;
        }
        break;

      // ── State 4: '-' display character ────────────────────────────────
      case 4:
        moveToChar(c);
        parseState = 0;
        break;

      // ── States 5-8: single-value numeric commands ──────────────────────
      case 5:
        if (isDigit(c)) { buffer += c; break; }
        if (buffer.length() > 0) { stepsFromHallToZero = buffer.toInt(); saveHomeOffset(); }
        parseState = 0; break;

      case 6:
        if (isDigit(c)) { buffer += c; break; }
        if (buffer.length() > 0) { totalStepsPerRev = buffer.toInt(); saveTotalSteps(); }
        parseState = 0; break;

      case 7:
        if (isDigit(c)) { buffer += c; break; }
        if (buffer.length() > 0) {
          int n = buffer.toInt();
          stepBackward(n); releaseMotor();
          stepsFromHallToZero += n; saveHomeOffset();
        }
        parseState = 0; break;

      case 8:
        if (isDigit(c)) { buffer += c; break; }
        if (buffer.length() > 0) {
          long target = buffer.toInt();
          long move   = target - currentStepPos;
          if (move < 0) move += totalStepsPerRev;
          while (move-- > 0) stepBackward(1);
          releaseMotor(); currentFlapIndex = -2; saveState();
        }
        parseState = 0; break;

      // ── State 9: 'w' write flap map entry ─────────────────────────────
      case 9:
        if (c == ':')       { tempIndex = buffer.toInt(); buffer = ""; }
        else if (isDigit(c)) { buffer += c; }
        else {
          if (buffer.length() > 0 && tempIndex != -1) {
            uint16_t pos = buffer.toInt();
            EEPROM.put(ADDR_MAP_START + (tempIndex * 2), pos);
          }
          parseState = 0;
        }
        break;

      // ── State 10: 'i' set ID ───────────────────────────────────────────
      case 10:
        if (isDigit(c)) { buffer += c; break; }
        if (buffer.length() > 0) {
          moduleId = (uint8_t)buffer.toInt();
          EEPROM.write(ADDR_MODULE_ID, moduleId);
          updateIdChars();
        }
        parseState = 0; break;

      // ── State 11: 'a' auto-home flag ──────────────────────────────────
      case 11:
        if (isDigit(c)) { buffer += c; break; }
        if (buffer.length() > 0) {
          autoHomeEnabled = (buffer.toInt() == 1);
          EEPROM.write(ADDR_AUTO_HOME, autoHomeEnabled ? 1 : 0);
          saveState();
        }
        parseState = 0; break;

      // ── State 12: '+' flap by index (v7+) ─────────────────────────────
      case 12:
        if (isDigit(c)) { buffer += c; break; }
        moveToIndex(buffer.toInt());
        parseState = 0; break;

      // ── State 13: 'H' home by serial number ───────────────────────────
      case 13:
        if (c != '\n' && c != '\r') { snBuffer += c; break; }
        if (serialMatches(snBuffer)) { homeModule(); saveState(); }
        snBuffer = ""; parseState = 0; break;

      // ── State 14: 'I' assign ID by serial number ──────────────────────
      case 14:
        if (c == ':' && !snColonSeen) {
          snColonSeen = true; buffer = "";
        } else if (c != '\n' && c != '\r') {
          if (!snColonSeen) snBuffer += c;
          else if (isDigit(c)) buffer += c;
        } else {
          if (snColonSeen && buffer.length() > 0 && serialMatches(snBuffer)) {
            moduleId = (uint8_t)buffer.toInt();
            EEPROM.write(ADDR_MODULE_ID, moduleId);
            updateIdChars();

            delay(20);
            digitalWrite(RS485_DE, HIGH);
            delay(5);
            rs485.print("mXack:");
            rs485.print(serialStr);
            rs485.print(":");
            rs485.print(moduleId);
            rs485.print("\n");
            delay(45);
            digitalWrite(RS485_DE, LOW);
            while (rs485.available()) rs485.read();
          }
          snBuffer = ""; snColonSeen = false; buffer = ""; parseState = 0;
        }
        break;
    }
  }

  // ── Timeout: flush incomplete numeric commands (50 ms idle) ───────────────
  if (parseState >= 5 && parseState <= 12 && (millis() - lastSerialTime > 50)) {
    if (buffer.length() > 0) {
      switch (parseState) {
        case 5:  stepsFromHallToZero = buffer.toInt(); saveHomeOffset(); break;
        case 6:  totalStepsPerRev = buffer.toInt(); saveTotalSteps(); break;
        case 7:  { int n = buffer.toInt(); stepBackward(n); releaseMotor();
                   stepsFromHallToZero += n; saveHomeOffset(); } break;
        case 8:  { long t = buffer.toInt(), mv = t - currentStepPos;
                   if (mv < 0) mv += totalStepsPerRev;
                   while (mv-- > 0) stepBackward(1);
                   releaseMotor(); currentFlapIndex = -2; saveState(); } break;
        case 9:  if (tempIndex != -1) {
                   uint16_t pos = buffer.toInt();
                   EEPROM.put(ADDR_MAP_START + (tempIndex * 2), pos);
                 } break;
        case 10: moduleId = (uint8_t)buffer.toInt();
                 EEPROM.write(ADDR_MODULE_ID, moduleId); updateIdChars(); break;
        case 11: autoHomeEnabled = (buffer.toInt() == 1);
                 EEPROM.write(ADDR_AUTO_HOME, autoHomeEnabled ? 1 : 0);
                 saveState(); break;
        case 12: moveToIndex(buffer.toInt()); break;
      }
    }
    parseState = 0;
  }

  // ── Timeout: flush incomplete provisioning frames (200 ms idle) ───────────
  if ((parseState == 13 || parseState == 14) && (millis() - lastSerialTime > 200)) {
    snBuffer = ""; snColonSeen = false; buffer = ""; parseState = 0;
  }
}
