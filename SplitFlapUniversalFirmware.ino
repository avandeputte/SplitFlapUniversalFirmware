// ==============================================================================
// Split-Flap Universal Firmware — v23
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
//  d         Dump EEPROM config as a single line (DIRECT-ADDRESSED ONLY):
//              m<id>d:<homeOffset>:<totalSteps>:<idx>=<pos>,<idx>=<pos>,...\n
//            Broadcast m*d is ignored (a full dump can be ~620 ms, too long to
//            stagger across a broadcast slot).
//  e         Erase calibrated flap position map
//  R         Reset provisioning (erase ID, resume advertising)  [v9+]
//  v         Report firmware version and serial number  → replies m<id>v:<version>:<moduleId>:<serialNumber>\n
//            Broadcast form may carry an optional ID range to poll the bus in
//            batches (recommended for large installations):
//              m*v\n        → all provisioned modules answer (slot = id × 45 ms)
//              m*v0-49\n    → only IDs 0–49 answer (slot = (id-lo) × 45 ms)
//              m*v50-99\n   → only IDs 50–99 answer
//            The Pi polls one batch at a time and re-issues any batch that
//            returns incomplete, which is far more reliable at 200+ modules
//            than a single full-bus sweep.
//  F         Factory-reset EEPROM defaults (preserves module ID and magic byte)
//
// Provisioning commands (address mX, all modules respond):
//  mXH<sn>         Home by serial number
//  mXI<sn>:<id>    Assign ID by serial number  → replies mXack:<sn>:<id>
//  mXD<sn>         Dump EEPROM config by serial number  → single-line reply,
//                  same format as 'd'
//  mXF<sn>         Factory-reset EEPROM by serial number (preserves module ID)
//  mXW<sn>:<homeOffset>:<totalSteps>:<idx>=<pos>,...\n
//                  Restore a previously dumped EEPROM onto the matching module.
//                  Module ID is always preserved; all other calibration fields
//                  and the flap position map are overwritten with the supplied
//                  values.  Map entries not present in the payload are cleared
//                  to 0xFFFF (uncalibrated).
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
//   v23 — Current release.  Consolidates all changes since the original v6.
//         Functional additions since v6:
//           - Dynamic module IDs via the ATtiny SIGROW serial number, with
//             advertisement of unprovisioned modules and serial-number-based
//             provisioning commands (mXH/mXI/mXD/mXF/mXW).
//           - Variable-length address parser (v6 two-char zero-padded format
//             still accepted); '+' index command; 'v' version report; 'F'
//             factory-reset; 'R' de-provision; 'd' single-line EEPROM dump.
//           - Scalable broadcast version query (m*v with optional <lo>-<hi>
//             range) for polling large buses in retryable batches.
//         EEPROM compatibility:
//           - Layout unchanged since v6.  Magic 0x5D (v6) and 0x5E (v8/v9) both
//             migrate cleanly; blank chips initialise to unprovisioned (ID 255).
//         Reliability & resilience hardening:
//           - 2 s watchdog with resets through all long blocking sections.
//           - All parser accumulators are fixed char[] buffers (no heap / no
//             String); FLAP_CHARS in PROGMEM.
//           - EEPROM writes are read-compare-write to minimise wear.
//           - Bounded all motor/Hall wait loops; calibration is sanity-checked.
//           - homeModule() reports failure instead of faking a known position.
//           - Capped startup stagger; advertisements paused during m* sweeps.
//           - Broadcast reply slotting (100 ms slots) and DE timing tuned for
//             reliable multi-module replies; direct v/d reply synchronously.
// ==============================================================================

#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>

// ── Deferred reply type ───────────────────────────────────────────────────────
// MUST be declared before anything else: the Arduino IDE auto-generates forward
// prototypes for every function and inserts them at the top of the sketch.  If a
// function that takes a PendingReply parameter (schedulePendingReply) is
// prototyped before this enum exists, compilation fails with
// "PendingReply was not declared in this scope".  Keeping the enum here, ahead of
// all other declarations, guarantees the generated prototypes can see it.
enum PendingReply { REPLY_NONE, REPLY_VERSION };

// ==========================================
//            CONFIGURATION
// ==========================================
// Number of physical flaps on the reel.  Single source of truth — used for the
// bounds check in moveToIndex and every loop over the EEPROM flap map.
#define NUM_FLAPS 64

// Ordered set of characters on the reel (index = physical flap position).
// Stored in flash (PROGMEM) rather than as an Arduino String to avoid using
// SRAM/heap on the 2 KB-SRAM ATtiny1616.
const char FLAP_CHARS[] PROGMEM =
  " ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789!@#$&()-+=;q:%'.,/?*roygbpw";

// Return the flap index for a character, or -1 if not present.
// Reads FLAP_CHARS from PROGMEM one byte at a time.
int flapIndexOf(char target) {
  for (int i = 0; i < NUM_FLAPS; i++) {
    char ch = (char)pgm_read_byte(&FLAP_CHARS[i]);
    if (ch == '\0') break;       // end of string (defensive)
    if (ch == target) return i;
  }
  return -1;
}

// Watchdog timeout. Must exceed the longest single blocking section between
// wdt_reset() calls.  The long motor moves call wdt_reset() internally, so this
// only needs to cover a normal loop iteration with margin.
#define WDT_TIMEOUT WDTO_2S

// Advertisement timing
const unsigned long ADVERTISE_BASE_MS   = 10000UL; // base interval
const unsigned long ADVERTISE_JITTER_MS =  5000UL; // max extra jitter

// ── Broadcast reply timing ────────────────────────────────────────────────────
// When many modules answer one broadcast (m*v) each transmits in its own time
// slot indexed by module ID:  replyTime = commandTime + (id - lo) × REPLY_SLOT_MS.
//
// SLOT WIDTH — this is the critical reliability parameter.
//   A version frame "m200v:16:200:<20 hex>\n" is up to ~38 characters.
//   At 9600 baud (10 bits/char) that is 38 × 10 / 9600 ≈ 40 ms ON THE WIRE.
//   The ATtiny1616 runs on its internal oscillator with ±2-3% tolerance, so
//   two modules' 45 ms timers could differ by 2-3 ms, and the MAX485 DE line
//   has its own enable/disable propagation.  A slot only a few ms larger than
//   the frame therefore collapses under drift and frames collide — which is
//   why even 7 modules were unreliable.
//
//   REPLY_SLOT_MS is set to comfortably MORE THAN TWICE the worst-case frame
//   time so that even with maximum drift, DE turnaround, and a late loop tick,
//   each module's frame finishes well inside its own slot with the next slot
//   still empty.  100 ms → ~60 ms of guard after a 40 ms frame.
const unsigned long REPLY_DIRECT_MS = 30UL;   // settling delay before a direct-addressed reply.
                                              // Must be long enough for the controller's RS-485
                                              // transceiver to finish transmitting the command and
                                              // switch to receive before the module answers.  10 ms
                                              // was too tight for some USB-RS485 adapters, which made
                                              // fast/low-ID replies (e.g. m0v) get clipped.
const unsigned long REPLY_SLOT_MS   = 100UL;  // per-ID slot width for broadcast replies
                                              // (must be ≫ worst-case frame time of ~40 ms)
const unsigned long REPLY_LEADIN_MS = 30UL;   // fixed offset before the FIRST broadcast slot, so the
                                              // lowest ID (slot 0, e.g. module 0 on m*v) waits for the
                                              // controller's transceiver turnaround before replying.

// ── Ranged broadcast version query ────────────────────────────────────────────
// Command form:  m*v<lo>-<hi>\n   (e.g. "m*v0-49\n")
// Only modules whose ID is within [lo, hi] respond.  Their slot is computed
// relative to lo:  replyTime = commandTime + (moduleId - lo) × REPLY_SLOT_MS,
// so each batch starts answering immediately and a 50-wide batch completes in
// ~2.25 s regardless of where in the ID space it sits.  The Pi polls the bus
// one batch at a time and re-issues any batch that comes back short.
//
// Plain "m*v\n" with no range still works and is equivalent to m*v0-254.
int  replyRangeLo = 0;    // inclusive low bound for the current ranged query
int  replyRangeHi = 254;  // inclusive high bound for the current ranged query

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
const char FIRMWARE_VERSION[] = "23";

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

bool serialMatches(const char *candidate) {
  // Must be exactly the serial-number length
  int len = 0;
  while (candidate[len] != '\0') len++;
  if (len != SERIAL_LEN * 2) return false;
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

// ── Fixed-size parser accumulators (no Arduino String / no heap) ──────────────
// Maximum field sizes are bounded by the protocol:
//   idBuffer       : up to 3 ID digits
//   buffer         : numeric payloads (step counts up to ~5 digits) + a little slack
//   snBuffer       : exactly 20 serial-number hex chars
//   restorePayload : <homeOffset>:<totalSteps>:<map> — bounded by NUM_FLAPS entries
// Each buffer carries its own length and a helper to append a char safely; once
// full, further chars are dropped (the frame will be rejected/timed out cleanly).
#define IDBUF_MAX   4
#define NUMBUF_MAX  8
#define SNBUF_MAX   20
#define RESTORE_MAX 600

char idBuf[IDBUF_MAX + 1];        int idLen = 0;
char numBuf[NUMBUF_MAX + 1];      int numLen = 0;
char snBuf[SNBUF_MAX + 1];        int snLen = 0;
char restoreBuf[RESTORE_MAX + 1]; int restoreLen = 0;

bool   idWildcard  = false;
bool   idProvision = false;

// Safe append: returns false (and drops the char) if the buffer is full.
inline bool bufAppend(char *buf, int &len, int maxLen, char c) {
  if (len >= maxLen) return false;
  buf[len++] = c;
  buf[len] = '\0';
  return true;
}
inline void bufClear(char *buf, int &len) { len = 0; buf[0] = '\0'; }

// Parse a decimal char buffer to long (empty → 0).
inline long bufToLong(const char *buf, int len) {
  return len > 0 ? atol(buf) : 0;
}

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
//
// For broadcast replies the slot is computed RELATIVE to replyRangeLo so each
// batch starts answering promptly:
//     replyTime = now + (moduleId - replyRangeLo + 1) × REPLY_SLOT_MS
// The +1 ensures the lowest ID in the range — including module ID 0 — still
// gets one full settling slot rather than replying at t=0.
//
// A module whose ID falls outside [replyRangeLo, replyRangeHi] does NOT reply
// at all — this is what makes ranged polling work.  Unprovisioned modules
// (ID 255) never match a numeric range and so never answer a broadcast v query
// (they are discovered via the advertisement mechanism instead).
void schedulePendingReply(PendingReply type, bool isBroadcast) {
  if (isBroadcast) {
    // Out-of-range (including unprovisioned ID 255) → stay silent.
    if (moduleId < replyRangeLo || moduleId > replyRangeHi) {
      pendingReply = REPLY_NONE;
      return;
    }
    // Broadcast slot timing:
    //   replyTime = now + REPLY_LEADIN_MS + (id - lo) × REPLY_SLOT_MS
    //
    // REPLY_LEADIN_MS is a fixed offset applied to EVERY module so that even
    // the lowest ID in the range (slot 0, e.g. module 0 on a full m*v) does not
    // begin transmitting until the controller has finished sending the command
    // and switched its transceiver to receive.  Previously the lowest ID used a
    // +1 slot for this purpose, but a dedicated lead-in is clearer and keeps the
    // slot index aligned with the ID offset (id 0 → slot 0, id 1 → slot 1, ...).
    unsigned long slot = REPLY_LEADIN_MS +
                         (unsigned long)(moduleId - replyRangeLo) * REPLY_SLOT_MS;
    pendingReplyTime = millis() + slot;
  } else {
    pendingReplyTime = millis() + REPLY_DIRECT_MS;
  }
  pendingReply = type;
}

unsigned long lastSerialTime    = 0;
unsigned long nextAdvertiseTime = 0;
unsigned long suppressAdvUntil  = 0;   // advertisements paused until this time (after seeing m*)

bool   snColonSeen = false;

const uint8_t halfStepSequence[8][4] = {
  {1,0,0,0}, {1,1,0,0}, {0,1,0,0}, {0,1,1,0},
  {0,0,1,0}, {0,0,1,1}, {0,0,0,1}, {1,0,0,1}
};

// ==========================================
//          EEPROM HELPERS
// ==========================================
// Write a byte only if it differs from what's already stored, to avoid wearing
// out EEPROM cells (rated ~100k writes) on values that haven't changed.
void eepromUpdateByte(int addr, uint8_t value) {
  if (EEPROM.read(addr) != value) EEPROM.write(addr, value);
}

// Update an arbitrary object (read-compare-write per byte).  Mirrors EEPROM.put
// but skips bytes that are already correct.
template <typename T>
void eepromUpdate(int addr, const T &value) {
  const uint8_t *p = (const uint8_t *)&value;
  for (unsigned i = 0; i < sizeof(T); i++) eepromUpdateByte(addr + i, p[i]);
}

void saveHomeOffset() { eepromUpdate(ADDR_HOME_OFFSET, stepsFromHallToZero); }
void saveTotalSteps()  { eepromUpdate(ADDR_TOTAL_STEPS, totalStepsPerRev); }

void saveState() {
  if (!autoHomeEnabled) {
    eepromUpdate(ADDR_SAVED_POS,   currentStepPos);
    eepromUpdateByte(ADDR_SAVED_INDEX, (uint8_t)(int8_t)currentFlapIndex);
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

// Dump EEPROM configuration over RS-485 as a SINGLE line:
//   m<id>d:<homeOffset>:<totalSteps>:<idx>=<pos>,<idx>=<pos>,...\n
//
// Reliability notes:
//  - The entire line is assembled into one fixed char[] buffer FIRST, doing all
//    64 EEPROM reads before the driver is enabled.  Nothing slow (EEPROM access,
//    integer formatting) happens while DE is HIGH, so the transmit is one tight,
//    predictable burst with a minimal driver-enable window.
//  - A fixed char[] is used instead of Arduino String, so there is no heap
//    allocation.  The previous String/streaming approaches risked exhausting the
//    ATtiny1616's 2 KB SRAM (or fragmenting the heap) when many flap entries were
//    populated, which made the dump silently produce nothing.
//
// Worst case (all 64 entries, 3-digit ID): ~595 bytes.  The buffer is sized to
// 640 to leave headroom and a guaranteed null terminator.
#define DUMP_BUF_SIZE 640

void dumpEeprom() {
  static char buf[DUMP_BUF_SIZE];   // static: not on the stack, not the heap
  int n = 0;                        // current write index into buf

  // ── Header: m<id>d:<homeOffset>:<totalSteps>: ────────────────────────
  buf[n++] = 'm';
  if (moduleId < 10) buf[n++] = '0';
  n += snprintf(&buf[n], DUMP_BUF_SIZE - n, "%u", (unsigned)moduleId);
  n += snprintf(&buf[n], DUMP_BUF_SIZE - n, "d:%d:%d:",
                stepsFromHallToZero, totalStepsPerRev);

  // ── Map entries: <idx>=<pos>,<idx>=<pos>,... ─────────────────────────
  // All EEPROM reads happen here, BEFORE the driver is enabled.
  bool first = true;
  for (int i = 0; i < NUM_FLAPS; i++) {
    uint16_t pos = 0xFFFF;
    EEPROM.get(ADDR_MAP_START + (i * 2), pos);
    if (pos == 0xFFFF) continue;

    // Guard against overflow (cannot happen at 64×9+header < 640, but safe).
    if (n > DUMP_BUF_SIZE - 12) break;

    if (!first) buf[n++] = ',';
    n += snprintf(&buf[n], DUMP_BUF_SIZE - n, "%d=%u", i, (unsigned)pos);
    first = false;
  }

  buf[n] = '\0';

  // ── Transmit the assembled line in one burst ─────────────────────────
  digitalWrite(RS485_DE, HIGH);
  delayMicroseconds(200);           // driver-enable settle
  rs485.print(buf);
  rs485.print("\n");
  delay(2);                         // hold until final stop bit is on the wire
  digitalWrite(RS485_DE, LOW);
  while (rs485.available()) rs485.read();   // discard self-echo
  parseState = 0;
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
  delayMicroseconds(200);

  rs485.print("mXadv:");
  rs485.print(serialStr);
  rs485.print("\n");

  delay(2);   // hold DE until final stop bit is on the wire
  digitalWrite(RS485_DE, LOW);
  while (rs485.available()) rs485.read();
  return true;
}

// ==========================================
//         PROVISIONING RESET
// ==========================================
void resetProvisioning() {
  moduleId = 255;
  eepromUpdateByte(ADDR_MODULE_ID, 255);
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
  delayMicroseconds(200);  // let DE/driver enable settle before the start bit
  rs485.print("m");
  printModuleId();
  rs485.print("v:");
  rs485.print(FIRMWARE_VERSION);
  rs485.print(":");
  rs485.print(moduleId);
  rs485.print(":");
  rs485.print(serialStr);
  rs485.print("\n");
  // Hold DE HIGH for ~1.5 character times after the last print() so the final
  // stop bit is fully clocked out on the wire before the driver is disabled.
  // At 9600 baud one character ≈ 1.04 ms; 2 ms covers it with margin.
  delay(2);
  digitalWrite(RS485_DE, LOW);
  // Flush echo bytes that accumulated in the RX buffer while DE was HIGH,
  // and reset the parser so we start clean for the next command.
  while (rs485.available()) rs485.read();
  parseState = 0;
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
  eepromUpdateByte(ADDR_AUTO_HOME, 1);

  // Clear saved position
  long   zeroPosL = 0;
  eepromUpdate(ADDR_SAVED_POS, zeroPosL);
  eepromUpdateByte(ADDR_SAVED_INDEX, 0);

  // Erase the entire calibrated flap map
  for (int i = 0; i < NUM_FLAPS; i++) {
    uint16_t empty = 0xFFFF;
    eepromUpdate(ADDR_MAP_START + (i * 2), empty);
  }
}

// ==========================================
//       EEPROM RESTORE FROM PAYLOAD
// ==========================================
// Parses the payload portion of an mXW restore command and writes all
// calibration fields to EEPROM.  Module ID and magic byte are preserved.
//
// Payload format (the part after mXW<sn>:):
//   <homeOffset>:<totalSteps>:<idx>=<pos>,<idx>=<pos>,...
//
// Examples:
//   2832:4096:                         (no calibrated flap positions)
//   2832:4096:0=0,7=342,12=683
//
// All NUM_FLAPS map entries are first cleared to 0xFFFF, then only the entries
// present in the payload are written.  Parses a plain C string (no Arduino
// String / no heap).
void restoreEepromFromPayload(const char *payload) {
  const char *p = payload;
  char *end;

  // ── Parse homeOffset ─────────────────────────────────────────────────
  long ho = strtol(p, &end, 10);
  if (*end != ':') return;          // malformed
  p = end + 1;

  // ── Parse totalSteps ─────────────────────────────────────────────────
  long ts = strtol(p, &end, 10);
  if (*end != ':') return;          // malformed
  p = end + 1;

  stepsFromHallToZero = (int)ho;
  totalStepsPerRev    = (int)ts;
  saveHomeOffset();
  saveTotalSteps();

  // ── Clear the entire flap map first ──────────────────────────────────
  for (int i = 0; i < NUM_FLAPS; i++) {
    uint16_t empty = 0xFFFF;
    eepromUpdate(ADDR_MAP_START + (i * 2), empty);
  }

  // ── Parse and write flap map entries: idx=pos,idx=pos,... ────────────
  while (*p) {
    long flapIdx = strtol(p, &end, 10);
    if (end == p || *end != '=') break;   // no number, or missing '='
    p = end + 1;
    long flapPos = strtol(p, &end, 10);
    if (end == p) break;                   // no number after '='
    p = end;

    if (flapIdx >= 0 && flapIdx < NUM_FLAPS) {
      uint16_t v = (uint16_t)flapPos;
      eepromUpdate(ADDR_MAP_START + (flapIdx * 2), v);
    }
    if (*p == ',') p++;                     // step over separator
    else break;                            // end of list (or malformed)
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
    wdt_reset();   // keep the watchdog happy during long moves
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

// Nudge the home position by a signed number of half-steps and fold the change
// into the stored home offset.  The reel is mechanically one-way (flaps only
// fall in one direction), so a NEGATIVE nudge cannot literally step the motor
// backward.  Instead we move forward to the equivalent position on the reel:
//   positive n  → step forward n
//   negative n  → step forward (totalStepsPerRev + n) so the reel lands at the
//                 same physical spot it would reach by going back |n| steps.
// Either way the stored offset is adjusted by the SIGNED amount so future moves
// reference the new home, and the value is wrapped to stay within one rev.
void nudge(int n) {
  if (totalStepsPerRev <= 0) return;

  // Physical move: convert any signed nudge into an equivalent forward count.
  long fwd = n % totalStepsPerRev;
  if (fwd < 0) fwd += totalStepsPerRev;
  for (long k = 0; k < fwd; k++) stepBackward(1);
  releaseMotor();

  // Calibration: adjust the stored home offset by the signed amount, wrapped
  // into [0, totalStepsPerRev).
  long newOffset = (long)stepsFromHallToZero + n;
  newOffset %= totalStepsPerRev;
  if (newOffset < 0) newOffset += totalStepsPerRev;
  stepsFromHallToZero = (int)newOffset;
  saveHomeOffset();
}

// ==========================================
//           LOGIC FUNCTIONS
// ==========================================
// Drive backward until the Hall sensor edge is found, then advance the home
// offset to land on flap 0.  If the Hall edge is NEVER found within the safety
// window (stuck/failed sensor, missing magnet), do NOT claim a known position:
// leave currentFlapIndex = -1 so the next move re-attempts homing instead of
// silently displaying the wrong flap.  Returns true on success.
bool homeModule() {
  long safety = 0;
  bool foundHome = false;
  while (safety < (totalStepsPerRev + 500)) {
    if (hallActive()) { foundHome = true; break; }
    stepBackward(1); safety++;
  }

  if (!foundHome) {
    releaseMotor();
    currentFlapIndex = -1;     // unknown — force a re-home on next move
    return false;
  }

  stepBackward(stepsFromHallToZero);
  currentStepPos = 0; currentFlapIndex = 0;
  releaseMotor();
  return true;
}

void calibrateModule() {
  long safety = 0;
  // Move off the magnet if currently on it (bounded).
  while (hallActive() && safety < 4000) { stepBackward(1); safety++; delay(5); }

  // Find the first magnet edge (bounded).
  safety = 0;
  while (!hallActive() && safety < (totalStepsPerRev + 1000)) { stepBackward(1); safety++; }

  // Step through the magnet region (bounded — previously unbounded → could hang).
  safety = 0;
  while (hallActive() && safety < 2000) { stepBackward(1); safety++; }

  // Measure one full revolution: count steps until we return through the magnet.
  int measuredSteps = 0;
  while (!hallActive() && measuredSteps < (totalStepsPerRev * 2 + 1000)) {
    stepBackward(1); measuredSteps++;
  }
  safety = 0;
  while (hallActive() && safety < 2000) { stepBackward(1); measuredSteps++; safety++; }

  // Sanity-check the measurement before committing it.  A wildly wrong value
  // (stuck sensor, no magnet) is rejected so we don't poison the calibration.
  bool valid = (measuredSteps > 500 && measuredSteps < 8000);

  delay(50);
  digitalWrite(RS485_DE, HIGH);
  delayMicroseconds(200);

  // Response format matches v6: m<2-char-id>:<steps>\n for IDs 0-99.
  // On a rejected measurement we still report the raw count so the controller
  // can see something went wrong.
  rs485.print("m");
  printModuleId();
  rs485.print(":");
  rs485.print(measuredSteps);
  rs485.print("\n");

  delay(2);
  digitalWrite(RS485_DE, LOW);
  while (rs485.available()) rs485.read();

  if (valid) {
    totalStepsPerRev = measuredSteps;
    saveTotalSteps();
  }
  homeModule();
  saveState();
}

void moveToIndex(int targetIndex) {
  if (targetIndex < 0 || targetIndex >= NUM_FLAPS) return;
  if (currentFlapIndex == targetIndex) return;
  if (currentFlapIndex == -1) {
    // Position unknown — must home first.  If homing fails, abort the move
    // rather than stepping blindly to a wrong absolute position.
    if (!homeModule()) return;
    if (currentFlapIndex == targetIndex) { releaseMotor(); return; }
  }

  uint16_t mappedPos = 0xFFFF;
  EEPROM.get(ADDR_MAP_START + (targetIndex * 2), mappedPos);

  long targetStepPos = (mappedPos != 0xFFFF)
    ? (long)mappedPos
    : ((long)targetIndex * (long)totalStepsPerRev) / NUM_FLAPS;

  long stepsToMove = targetStepPos - currentStepPos;
  if (stepsToMove < 0) stepsToMove += totalStepsPerRev;
  while (stepsToMove-- > 0) stepBackward(1);

  releaseMotor();
  currentFlapIndex = targetIndex;
  saveState();
}

void moveToChar(char targetChar) {
  int idx = flapIndexOf(targetChar);
  if (idx != -1) moveToIndex(idx);
}

// ==========================================
//               SETUP
// ==========================================
void setup() {
  // Disable the watchdog early in case we got here via a WDT reset (the flag
  // can leave the WDT running on some AVRs), then configure it fresh below.
  wdt_disable();

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
      eepromUpdateByte(ADDR_INIT, EEPROM_MAGIC);
    }
  } else {
    // Blank chip or unrecognised magic — write defaults.
    // Module ID intentionally left as 255 (unprovisioned).
    eepromUpdateByte(ADDR_INIT, EEPROM_MAGIC);
    saveHomeOffset();
    saveTotalSteps();
    eepromUpdateByte(ADDR_MODULE_ID, 255);
    eepromUpdateByte(ADDR_AUTO_HOME, 1);
    autoHomeEnabled = true;
    moduleId        = 255;

    for (int i = 0; i < NUM_FLAPS; i++) {
      uint16_t empty = 0xFFFF;
      eepromUpdate(ADDR_MAP_START + (i * 2), empty);
    }
  }

  updateIdChars();

  // Sanity-clamp calibration values loaded from EEPROM so a corrupted cell can't
  // produce absurd step counts / divide-by-something-silly later.
  if (totalStepsPerRev < 500 || totalStepsPerRev > 8000) totalStepsPerRev = 4096;
  if (stepsFromHallToZero < 0 || stepsFromHallToZero > totalStepsPerRev)
    stepsFromHallToZero = 2832;

  // ── Staggered startup ─────────────────────────────────────────────────────
  // Spread motor inrush so a whole panel powering on together doesn't brown out.
  // The stagger is CAPPED so high IDs aren't unresponsive for tens of seconds:
  // we spread across a fixed window using (id mod N) rather than raw id.
  // Watchdog is not yet running here, so a plain delay is safe.
  unsigned long staggerMs;
  if (moduleId != 255) {
    staggerMs = (unsigned long)(moduleId % 32) * 120UL;  // 0–3.7 s window
  } else {
    staggerMs = lcgRand(4000UL);                          // 0–4 s random
  }
  delay(staggerMs);

  // ── Start the watchdog now that the slow init/stagger is done ──────────────
  wdt_enable(WDT_TIMEOUT);

  // ── Home or restore ───────────────────────────────────────────────────────
  if (autoHomeEnabled) {
    homeModule();
    saveState();
  } else {
    EEPROM.get(ADDR_SAVED_POS, currentStepPos);
    if (currentStepPos < 0 || currentStepPos >= (long)totalStepsPerRev) currentStepPos = 0;
    currentFlapIndex = (int8_t)EEPROM.read(ADDR_SAVED_INDEX);
    if (currentFlapIndex < -1 || currentFlapIndex >= NUM_FLAPS) currentFlapIndex = -1;
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
// parseState 15 : 'D' prov — serial number hex digits (dump by SN)
// parseState 16 : 'F' prov — serial number hex digits (factory reset by SN)
// parseState 17 : 'W' prov — serial number ':' full calibration payload (restore by SN)
// parseState 18 : broadcast 'v' — optional "<lo>-<hi>" ID range for batched query

void loop() {
  wdt_reset();   // service the watchdog every iteration

  // ── Deferred reply (broadcast m*v only) ──────────────────────────────────
  // Only broadcast version replies are deferred (to stagger across modules).
  // Direct v/d and mXD reply synchronously in the parser.
  if (pendingReply != REPLY_NONE && (long)(millis() - pendingReplyTime) >= 0) {
    if (pendingReply == REPLY_VERSION) sendVersionResponse();
    pendingReply = REPLY_NONE;
  }

  // ── Advertisement heartbeat (unprovisioned only) ──────────────────────────
  // Suppressed for a short window after any m* broadcast so an advertisement
  // can't collide with an in-progress m*v reply sweep.
  if (moduleId == 255 &&
      (long)(millis() - nextAdvertiseTime) >= 0 &&
      (long)(millis() - suppressAdvUntil) >= 0) {
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
          bufClear(idBuf, idLen); idWildcard = false; idProvision = false;
          parseState = 1;
        }
        break;

      // ── State 1: accumulate address, then dispatch ─────────────────────
      // Accepts any mix of digits, '*', and 'X'.  Leading zeros are fine
      // ("05" → 5).  The digit accumulator is length-capped (IDBUF_MAX) so a
      // flood of digits can't grow a buffer without bound.
      case 1:
        if      (c == '*')      { idWildcard  = true; }
        else if (c == 'X')      { idProvision = true; }
        else if (isDigit(c))    { if (!idWildcard && !idProvision) bufAppend(idBuf, idLen, IDBUF_MAX, c); }
        else {
          // c is the command letter.
          bool match = idWildcard ||
                       (idLen > 0 && (uint8_t)bufToLong(idBuf, idLen) == moduleId);

          // Pause advertisements briefly on ANY broadcast so an unprovisioned
          // module doesn't talk over an in-progress m*v reply sweep.
          if (idWildcard) suppressAdvUntil = millis() + 30000UL;

          if (idProvision) {
            // Provisioning commands: every module listens, SN check is inside.
            if      (c == 'H') { bufClear(snBuf, snLen);
                                  parseState = 13; }
            else if (c == 'I') { bufClear(snBuf, snLen); snColonSeen = false; bufClear(numBuf, numLen);
                                  parseState = 14; }
            else if (c == 'D') { bufClear(snBuf, snLen);
                                  parseState = 15; }
            else if (c == 'F') { bufClear(snBuf, snLen);
                                  parseState = 16; }
            else if (c == 'W') { bufClear(snBuf, snLen); snColonSeen = false; restoreLen = 0; restoreBuf[0] = '\0';
                                  parseState = 17; }
            else                 parseState = 0;

          } else if (match) {
            // Normal addressed / broadcast commands
            switch (c) {
              case '-': parseState = 4; break;
              case '+': bufClear(numBuf, numLen); parseState = 12; break;
              case 'h': homeModule(); saveState(); parseState = 0; break;
              case 'c': calibrateModule();          parseState = 0; break;
              case 'o': bufClear(numBuf, numLen); parseState = 5; break;
              case 't': bufClear(numBuf, numLen); parseState = 6; break;
              case 's': bufClear(numBuf, numLen); parseState = 7; break;
              case 'g': bufClear(numBuf, numLen); parseState = 8; break;
              case 'w': bufClear(numBuf, numLen); tempIndex = -1; parseState = 9; break;
              case 'i': bufClear(numBuf, numLen); parseState = 10; break;
              case 'a': bufClear(numBuf, numLen); parseState = 11; break;
              case 'd':
                // Dumps answered ONLY when addressed directly, SYNCHRONOUSLY,
                // exactly like mXD.  Broadcast dumps are ignored (too long to
                // stagger across a reply slot).
                if (!idWildcard) {
                  dumpEeprom();
                }
                parseState = 0;
                break;
              case 'R': resetProvisioning();                              parseState = 0; break;
              case 'v':
                if (idWildcard) {
                  // Broadcast version query — may carry an optional "<lo>-<hi>"
                  // range.  Collect it in state 18; broadcast replies are
                  // staggered via the deferred path.
                  bufClear(numBuf, numLen);
                  replyRangeLo = 0;
                  replyRangeHi = 254;
                  parseState = 18;
                } else {
                  // Direct query — reply SYNCHRONOUSLY and immediately, like mXD.
                  sendVersionResponse();
                  parseState = 0;
                }
                break;
              case 'F': resetEepromDefaults();                            parseState = 0; break;
              case 'e':
                for (int i = 0; i < NUM_FLAPS; i++) {
                  uint16_t empty = 0xFFFF;
                  eepromUpdate(ADDR_MAP_START + (i * 2), empty);
                }
                parseState = 0;
                break;
              default: parseState = 0; break;
            }
          } else {
            parseState = 0;
          }

          bufClear(idBuf, idLen); idWildcard = false; idProvision = false;
        }
        break;

      // ── State 4: '-' display character ────────────────────────────────
      case 4:
        moveToChar(c);
        parseState = 0;
        break;

      // ── States 5-8: single-value numeric commands ──────────────────────
      case 5:
        if (isDigit(c)) { bufAppend(numBuf, numLen, NUMBUF_MAX, c); break; }
        if (numLen > 0) { stepsFromHallToZero = (int)bufToLong(numBuf, numLen); saveHomeOffset(); }
        parseState = 0; break;

      case 6:
        if (isDigit(c)) { bufAppend(numBuf, numLen, NUMBUF_MAX, c); break; }
        if (numLen > 0) { totalStepsPerRev = (int)bufToLong(numBuf, numLen); saveTotalSteps(); }
        parseState = 0; break;

      case 7:
        // Nudge: accept an optional leading '-' so negative offsets work
        // (e.g. "m38s-16").  '-' is only valid as the first character.
        if (c == '-' && numLen == 0) { bufAppend(numBuf, numLen, NUMBUF_MAX, c); break; }
        if (isDigit(c)) { bufAppend(numBuf, numLen, NUMBUF_MAX, c); break; }
        if (numLen > 0) {
          int n = (int)bufToLong(numBuf, numLen);
          nudge(n);
        }
        parseState = 0; break;

      case 8:
        if (isDigit(c)) { bufAppend(numBuf, numLen, NUMBUF_MAX, c); break; }
        if (numLen > 0) {
          long target = bufToLong(numBuf, numLen);
          long move   = target - currentStepPos;
          if (move < 0) move += totalStepsPerRev;
          while (move-- > 0) stepBackward(1);
          releaseMotor(); currentFlapIndex = -2; saveState();
        }
        parseState = 0; break;

      // ── State 9: 'w' write flap map entry ─────────────────────────────
      case 9:
        if (c == ':')        { tempIndex = (int)bufToLong(numBuf, numLen); bufClear(numBuf, numLen); }
        else if (isDigit(c)) { bufAppend(numBuf, numLen, NUMBUF_MAX, c); }
        else {
          if (numLen > 0 && tempIndex >= 0 && tempIndex < NUM_FLAPS) {
            uint16_t pos = (uint16_t)bufToLong(numBuf, numLen);
            eepromUpdate(ADDR_MAP_START + (tempIndex * 2), pos);
          }
          parseState = 0;
        }
        break;

      // ── State 10: 'i' set ID ───────────────────────────────────────────
      case 10:
        if (isDigit(c)) { bufAppend(numBuf, numLen, NUMBUF_MAX, c); break; }
        if (numLen > 0) {
          moduleId = (uint8_t)bufToLong(numBuf, numLen);
          eepromUpdateByte(ADDR_MODULE_ID, moduleId);
          updateIdChars();
        }
        parseState = 0; break;

      // ── State 11: 'a' auto-home flag ──────────────────────────────────
      case 11:
        if (isDigit(c)) { bufAppend(numBuf, numLen, NUMBUF_MAX, c); break; }
        if (numLen > 0) {
          autoHomeEnabled = (bufToLong(numBuf, numLen) == 1);
          eepromUpdateByte(ADDR_AUTO_HOME, autoHomeEnabled ? 1 : 0);
          saveState();
        }
        parseState = 0; break;

      // ── State 12: '+' flap by index (v7+) ─────────────────────────────
      case 12:
        if (isDigit(c)) { bufAppend(numBuf, numLen, NUMBUF_MAX, c); break; }
        moveToIndex((int)bufToLong(numBuf, numLen));
        parseState = 0; break;

      // ── State 13: 'H' home by serial number ───────────────────────────
      case 13:
        if (c != '\n' && c != '\r') { bufAppend(snBuf, snLen, SNBUF_MAX, c); break; }
        if (serialMatches(snBuf)) { homeModule(); saveState(); }
        bufClear(snBuf, snLen); parseState = 0; break;

      // ── State 14: 'I' assign ID by serial number ──────────────────────
      case 14:
        if (c == ':' && !snColonSeen) {
          snColonSeen = true; bufClear(numBuf, numLen);
        } else if (c != '\n' && c != '\r') {
          if (!snColonSeen) bufAppend(snBuf, snLen, SNBUF_MAX, c);
          else if (isDigit(c)) bufAppend(numBuf, numLen, NUMBUF_MAX, c);
        } else {
          if (snColonSeen && numLen > 0 && serialMatches(snBuf)) {
            moduleId = (uint8_t)bufToLong(numBuf, numLen);
            eepromUpdateByte(ADDR_MODULE_ID, moduleId);
            updateIdChars();

            delay(20);
            digitalWrite(RS485_DE, HIGH);
            delayMicroseconds(200);
            rs485.print("mXack:");
            rs485.print(serialStr);
            rs485.print(":");
            rs485.print(moduleId);
            rs485.print("\n");
            delay(2);
            digitalWrite(RS485_DE, LOW);
            while (rs485.available()) rs485.read();
          }
          bufClear(snBuf, snLen); snColonSeen = false; bufClear(numBuf, numLen); parseState = 0;
        }
        break;

      // ── State 15: 'D' dump EEPROM by serial number ────────────────────
      // Format:  mXD<serialNumber>\n
      // Only the matching module replies — direct call, no collision risk.
      case 15:
        if (c != '\n' && c != '\r') { bufAppend(snBuf, snLen, SNBUF_MAX, c); break; }
        if (serialMatches(snBuf)) dumpEeprom();
        bufClear(snBuf, snLen); parseState = 0;
        break;

      // ── State 16: 'F' factory-reset EEPROM by serial number ───────────
      // Format:  mXF<serialNumber>\n
      // Resets all calibration values to defaults; preserves module ID.
      case 16:
        if (c != '\n' && c != '\r') { bufAppend(snBuf, snLen, SNBUF_MAX, c); break; }
        if (serialMatches(snBuf)) resetEepromDefaults();
        bufClear(snBuf, snLen); parseState = 0;
        break;

      // ── State 17: 'W' restore EEPROM by serial number ─────────────────
      // Format:  mXW<serialNumber>:<homeOffset>:<totalSteps>:<idx>=<pos>,...\n
      //
      //   snBuf       accumulates the 20-char serial number
      //   restoreBuf  accumulates everything after the first ':' (length-capped)
      //
      // Module ID and magic byte are always preserved; all other calibration
      // fields and the flap map are overwritten with the supplied values.
      case 17:
        if (c == ':' && !snColonSeen) {
          // First colon — switch from accumulating SN to accumulating payload
          snColonSeen = true;
        } else if (c != '\n' && c != '\r') {
          if (!snColonSeen) bufAppend(snBuf, snLen, SNBUF_MAX, c);
          else              bufAppend(restoreBuf, restoreLen, RESTORE_MAX, c);
        } else {
          // Terminator — act if this module matches
          if (snColonSeen && serialMatches(snBuf)) {
            restoreEepromFromPayload(restoreBuf);
          }
          bufClear(snBuf, snLen); snColonSeen = false;
          restoreLen = 0; restoreBuf[0] = '\0'; parseState = 0;
        }
        break;

      // ── State 18: optional range for broadcast version query ───────────
      // Reached after "m*v".  Accepts an optional "<lo>-<hi>" range:
      //   m*v\n         → whole bus   (replyRangeLo=0,  replyRangeHi=254)
      //   m*v0-49\n     → IDs 0–49 only
      //   m*v50-99\n    → IDs 50–99 only
      //
      // numBuf accumulates the low number until '-', then the high number.
      // snColonSeen is reused here as a "seen the dash" flag.
      case 18:
        if (isDigit(c)) {
          bufAppend(numBuf, numLen, NUMBUF_MAX, c);
        } else if (c == '-') {
          // Low bound complete; start collecting high bound.
          replyRangeLo = (int)bufToLong(numBuf, numLen);
          bufClear(numBuf, numLen);
          snColonSeen = true;  // reused: marks that we've passed the dash
        } else {
          // Terminator (\n, \r, or anything else) — finalise and schedule.
          if (snColonSeen) {
            if (numLen > 0) replyRangeHi = (int)bufToLong(numBuf, numLen);
          } else {
            if (numLen > 0) {
              replyRangeLo = (int)bufToLong(numBuf, numLen);
              replyRangeHi = replyRangeLo;
            }
          }
          schedulePendingReply(REPLY_VERSION, true);
          bufClear(numBuf, numLen); snColonSeen = false; parseState = 0;
        }
        break;

    }  // end switch(parseState)
  }  // end while(rs485.available())

  // ── Timeout: flush incomplete numeric commands (50 ms idle) ───────────────
  if (parseState >= 5 && parseState <= 12 && (millis() - lastSerialTime > 50)) {
    if (numLen > 0) {
      switch (parseState) {
        case 5:  stepsFromHallToZero = (int)bufToLong(numBuf, numLen); saveHomeOffset(); break;
        case 6:  totalStepsPerRev = (int)bufToLong(numBuf, numLen); saveTotalSteps(); break;
        case 7:  { int n = (int)bufToLong(numBuf, numLen); nudge(n); } break;
        case 8:  { long t = bufToLong(numBuf, numLen), mv = t - currentStepPos;
                   if (mv < 0) mv += totalStepsPerRev;
                   while (mv-- > 0) stepBackward(1);
                   releaseMotor(); currentFlapIndex = -2; saveState(); } break;
        case 9:  if (tempIndex >= 0 && tempIndex < NUM_FLAPS) {
                   uint16_t pos = (uint16_t)bufToLong(numBuf, numLen);
                   eepromUpdate(ADDR_MAP_START + (tempIndex * 2), pos);
                 } break;
        case 10: moduleId = (uint8_t)bufToLong(numBuf, numLen);
                 eepromUpdateByte(ADDR_MODULE_ID, moduleId); updateIdChars(); break;
        case 11: autoHomeEnabled = (bufToLong(numBuf, numLen) == 1);
                 eepromUpdateByte(ADDR_AUTO_HOME, autoHomeEnabled ? 1 : 0);
                 saveState(); break;
        case 12: moveToIndex((int)bufToLong(numBuf, numLen)); break;
      }
    }
    bufClear(numBuf, numLen);
    parseState = 0;
  }

  // ── Timeout: flush incomplete provisioning frames (200 ms idle) ───────────
  if ((parseState == 13 || parseState == 14 || parseState == 15 ||
       parseState == 16 || parseState == 17)
      && (millis() - lastSerialTime > 200)) {
    bufClear(snBuf, snLen); snColonSeen = false;
    restoreLen = 0; restoreBuf[0] = '\0'; bufClear(numBuf, numLen); parseState = 0;
  }

  // ── Timeout: finalise a broadcast version-range query (50 ms idle) ────────
  // If the line had no explicit terminator, schedule the reply with whatever
  // range was parsed so far (defaults already set when entering state 18).
  if (parseState == 18 && (millis() - lastSerialTime > 50)) {
    if (snColonSeen) {
      if (numLen > 0) replyRangeHi = (int)bufToLong(numBuf, numLen);
    } else if (numLen > 0) {
      replyRangeLo = (int)bufToLong(numBuf, numLen);
      replyRangeHi = replyRangeLo;
    }
    schedulePendingReply(REPLY_VERSION, true);
    bufClear(numBuf, numLen); snColonSeen = false; parseState = 0;
  }
}  // end loop()
