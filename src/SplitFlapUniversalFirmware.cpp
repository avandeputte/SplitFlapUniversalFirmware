// ==============================================================================
// Split-Flap Universal Firmware — v31
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
//  -<char>   Display the character <char> (single byte, from the configured
//            flap set — see 'N').  A character that is not in the set homes the
//            reel instead.
//  +<n>      Display flap at index <n>  (v7+ only; not sent by v6 code)
//  h         Home the reel
//  c         Calibrate revolution length and report result
//  T         Hall sensor self-test (DIRECT-ADDRESSED ONLY).  Steps one
//            revolution and reports sensor health:
//              m<id>T:<code>:<edges>:<activeSamples>\n
//            code 0=OK, 1=stuck active (shorted/jammed), 2=stuck inactive
//            (dead/disconnected), 3=multiple active regions (noise/stray
//            magnet), 4=inverted polarity (sensor reads active everywhere with
//            one brief dip at the magnet — wired backwards).
//            <edges> = home pulses seen per rev (healthy = 1); <activeSamples>
//            = how many sampled steps read active (advisory detail).
//  P         Re-detect Hall sensor polarity (DIRECT-ADDRESSED ONLY, drives one
//            revolution), then re-home.  Reply: m<id>P:<code>:<level>\n where
//            code 0=detected/1=no magnet seen (kept previous), level 0=active
//            LOW / 1=active HIGH (the level now in use).  Polarity is also
//            auto-detected on first boot and at every calibrate, so this is only
//            needed after re-mounting a magnet without a full recalibration.
//  Q         Diagnostics snapshot (DIRECT-ADDRESSED ONLY, no motor movement):
//              m<id>Q:<resetCause>:<bootCount>:<vcc_mV>:<eepromOk>:<curIndex>\n
//            resetCause = raw RSTFR bits from last reset (0x01 power-on,
//            0x02 brown-out, 0x04 external, 0x08 watchdog, 0x10 software);
//            bootCount = boots since counter reset (wraps 255); vcc_mV =
//            supply millivolts; eepromOk = 1 if EEPROM verify passed.
//  M[<n>]    Mechanical self-test (DIRECT-ADDRESSED ONLY, drives the motor):
//              m<id>M:<code>:<min>:<max>:<spreadTenthsPct>:<gateActive>:<gateSpan>:<avgMagnetWidth>:<r1>,<r2>,...,<rN>\n
//            Samples several revolutions and compares them.  Bare 'm<id>M' uses
//            the default count (MECH_TEST_REVS); 'm<id>M<n>' requests n rotations,
//            clamped to [MECH_TEST_REVS_MIN, MECH_TEST_REVS_MAX] (more rotations
//            catch rarer intermittent faults but take longer, ~4 s each).
//            code 0=OK (all revs consistent), 1=inconsistent (spread >5%, i.e.
//            intermittent missed steps — drag, weak supply, failing driver),
//            2=no motion (motor not turning — open coil/dead driver/jam, OR a
//            dead Hall sensor: run 'T' first to disambiguate).
//            min/max = smallest/largest steps-per-rev measured (0 if no motion);
//            spreadTenthsPct = (max-min)/avg in tenths of a percent (e.g. 23 =
//            2.3%).  gateActive/gateSpan always report what the motion-detect
//            gate observed: gateActive = Hall active-samples seen while driving
//            ~1.1 revolutions (gateSpan).  On a code-2, gateActive ≈ one magnet
//            width means the reel under-rotated (motor slip); ≈0 means the
//            sensor never fired; ≈gateSpan means it was parked on the magnet.
//            avgMagnetWidth = average magnet width across the revs; r1..rN = the
//            raw steps-per-rev for each rotation (the trend distinguishes an
//            intermittent glitch from a progressive drift).
//  o<n>      Set home offset (steps past Hall trigger to flap 0)
//  t<n>      Set total steps per revolution
//  s<n>      Nudge forward n steps and add to home offset
//  g<n>      Go to raw step position n
//  w<i>:<p>  Write calibrated position p for flap index i
//  i<n>      Set module ID to n
//  a<n>      Set auto-home flag (1=home on boot, 0=restore saved position)
//  N<count>:<chars>
//            Configure the flap set.  BOTH parts are optional and INDEPENDENT:
//              m<id>N<count>      → set physical flap count only (1..64)
//              m<id>N:<chars>     → set the reel's character set only
//              m<id>N<count>:<chars> → set both
//            <count> drives the valid flap-index range and the position math;
//            <chars> is the ordered character at each flap index (used by the
//            '-'<char> command), up to 64 bytes and MAY itself contain ':'.
//            Both default to 64 / the built-in set and persist in EEPROM.
//            Works direct-addressed OR broadcast (m*N... sets the whole panel
//            at once); there is no reply.  Read the current values back via 'A'.
//            A serial-number form (mXN<sn>:<count>:<chars>) is listed below.
//  d         Dump EEPROM config as a single line (DIRECT-ADDRESSED ONLY):
//              m<id>d:<homeOffset>:<totalSteps>:<idx>=<pos>,<idx>=<pos>,...\n
//            Broadcast m*d is ignored (a full dump can be ~620 ms, too long to
//            stagger across a broadcast slot).
//  A         Combined "all fields" dump — everything from 'v' AND 'd' in ONE
//            message, so a client can fetch a module's complete state in a
//            single command:
//              m<id>A:<version>:<moduleId>:<serialNumber>:<homeOffset>:<totalSteps>:<autoHome>:<curIndex>:<idx>=<pos>,...:<flapCount>:<flapChars>\n
//            <flapCount> and <flapChars> (the configurable flap set; see 'N')
//            were APPENDED in v31 — fields up to and including the map list are
//            byte-identical to earlier firmware, so older controllers ignore the
//            tail.  <flapChars> is the FINAL field and may itself contain ':',
//            ',' or '=', so read it verbatim to end-of-line (do not split it).
//            The 'v' and 'd' commands are unchanged and remain for backward
//            compatibility.
//            Broadcast form is supported and staggered like 'v', with an
//            optional ID range (each 'A' frame is long, so a full sweep is slow
//            — prefer ranged batches on large buses):
//              m*A\n        → all provisioned modules answer (wide slots)
//              m*A0-49\n    → only IDs 0–49 answer
//              m*A50-99\n   → only IDs 50–99 answer
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
//  mXA<sn>         Combined all-fields dump by serial number → single-line
//                  reply, same format as 'A'
//  mXT<sn>         Hall sensor self-test by serial number → same reply as 'T'
//  mXQ<sn>         Diagnostics snapshot by serial number → same reply as 'Q'
//  mXM<sn>         Mechanical self-test by serial number → same reply as 'M'
//  mXF<sn>         Factory-reset EEPROM by serial number (preserves module ID)
//  mXN<sn>:<count>:<chars>
//                  Configure the flap set by serial number (same as 'N', but
//                  targeted by serial instead of bus ID).  <count> and <chars>
//                  are both optional and independent; no reply.
//  mXW<sn>:<homeOffset>:<totalSteps>:<idx>=<pos>,...[:<flapCount>:<flapChars>]\n
//                  Restore a previously dumped EEPROM onto the matching module.
//                  Module ID is always preserved; all other calibration fields
//                  and the flap position map are overwritten with the supplied
//                  values.  Map entries not present in the payload are cleared
//                  to 0xFFFF (uncalibrated).  The optional trailing
//                  :<flapCount>:<flapChars> (as emitted by the 'A' dump) restores
//                  the configurable flap set; if omitted, the flap set is left
//                  unchanged.  <flapChars> is the final field and may contain ':'.
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
//  0x0A   1    Boot counter (diagnostics; was reserved/padding pre-v26)
//  0x0B   1    EEPROM health-check scratch byte (diagnostics; was reserved)
//  0x0C  128   Calibrated step positions: 64 × uint16_t, 0xFFFF = uncalibrated
//  0x8C    1   Hall sensor active level (0=LOW,1=HIGH,0xFF=undetected; auto)
//  0x8D    1   Flap count (1..64; 0xFF/out-of-range = default 64)  [v31]
//  0x8E   64   Flap char set (0xFF in first byte = default set)    [v31]
//
// v31 added the two flap-set fields in previously-unused EEPROM (0x8D onward);
// no v6-era field moved, so the layout stays backward-compatible.
//
// Magic byte history — all recognised and migrated to 0x5D:
//   0x5D  v6 original                  → load all fields as-is (already current)
//   0x5E  v8/v9 (erroneous bump)       → same field layout; load fields, write 0x5D
//   other blank chip or unknown        → write defaults, leave ID as 255, write 0x5D
//
// ==============================================================================
// CHANGE LOG
// ==============================================================================
//   v31 — The flap count and character set are now configurable at runtime via
//         the new 'N' command (m<id>N<count>:<chars>), independently of each
//         other and persisted in EEPROM; both default to the previous 64 /
//         built-in FLAP_CHARS.  NUM_FLAPS became MAX_FLAPS (the compile-time
//         capacity = 64); the ACTIVE count is the runtime `flapCount`, which now
//         drives index bounds and the position math, while the configured char
//         set is read from EEPROM (or the PROGMEM default) by flapCharAt().  'N'
//         works direct, as a broadcast (m*N...) to set a whole panel at once, and
//         by serial number (mXN<sn>:<count>:<chars>).  The combined 'A' dump now
//         reports the configured count + chars (appended at the end, so older
//         controllers are unaffected), and the mXW restore accepts the same
//         :<flapCount>:<flapChars> tail so an 'A' dump round-trips.  A '-'<char>
//         request for a character NOT in the configured set now homes the reel
//         instead of doing nothing.  New EEPROM
//         fields live in the previously-unused tail (0x8D+), so the layout stays
//         backward-compatible and modules upgraded from pre-v31 fall back to the
//         defaults until set.  To fit the 16 KB part, the RS-485 transmit framing
//         (txBegin/txEnd/sendLine/txReplyHeader/txNumField), the dump assembly
//         (appendMsgId/appendFlapMap), the flap-map erase (clearFlapMap), the 'g'
//         move (gotoRawStep) and the parameterless serial-number provisioning
//         commands (one shared parse state) were all de-duplicated.
//   v30 — Freshly-flashed modules now auto-derive the values the firmware CAN
//         measure, so an unknown module isn't left guessing:
//           - Hall sensor polarity is AUTO-DETECTED (works with either sensor
//             wiring or magnet orientation).  hallActive() compares the pin
//             against a detected active level (EEPROM addr 140) instead of a
//             hard-coded LOW.  Detection picks the MINORITY level over one rev
//             (the magnet covers only a small fraction of a rev) as active.
//           - Steps-per-revolution is AUTO-MEASURED on the fresh-EEPROM path
//             rather than assumed: a 28BYJ-48 is 4096 in half-step but 2048 in
//             full-step (clones vary too), so assuming 4096 would break a 2048
//             module.  The default home offset is scaled to the measured rev if
//             the old default would exceed it.  Only a VALID measurement is
//             committed; a failed one keeps the defaults.
//         Both run only when EEPROM polarity is undetected (0xFF) — i.e. a first
//         flash or a reflash where EEPROM was NOT preserved.  A normal reflash
//         with preserved EEPROM never triggers them, so a good calibration is
//         never silently overwritten.  The new 'P' command re-detects polarity
//         on demand.  The home OFFSET still requires the operator's dial-in (it
//         is the magnet-to-flap-0 distance, which the firmware cannot measure).
//         NOTE: a module upgraded from pre-v30 firmware reads 0xFF at addr 140
//         and so auto-measures once on its first v30 boot (one extra revolution);
//         the measurement only re-confirms a correct existing step count.
//   v29 — Mechanical self-test accepts an optional revolution count: 'm<id>M<n>'
//         samples n rotations (clamped to [MECH_TEST_REVS_MIN, MECH_TEST_REVS_MAX]
//         = 5..20) for better statistics on rare intermittent faults; bare
//         'm<id>M' still uses the default.  Direct ID form only; the mXM<sn>
//         serial form always uses the default count.  Backward compatible.
//   v28 — Richer self-test telemetry (all APPEND-ONLY; existing fields and their
//         positions are unchanged, so older parsers keep working):
//           'M' now appends the average magnet width and the raw steps-per-rev
//               for every sampled rotation:
//                 ...:<gateSpan>:<avgMagnetWidth>:<r1>,<r2>,...,<rN>\n
//               The per-rotation trend distinguishes an intermittent glitch from
//               a progressive drift (which min/max/spread report identically),
//               and a wandering magnet width flags a changing sensor-magnet gap.
//           'T' now appends fallingEdges (active→inactive transitions); a clean
//               sensor shows rising == falling == 1.
//   v27 — Mechanical self-test ('M') now always reports the motion-detect gate
//         observations as two additional fixed fields:
//           m<id>M:<code>:<min>:<max>:<spreadTenthsPct>:<gateActive>:<gateSpan>\n
//         Every field has the same meaning regardless of result code (no more
//         code-dependent field reuse), so the controller parser never has to
//         branch.  gateActive/gateSpan make a code-2 NO_MOTION result directly
//         diagnosable: ~one magnet width = reel under-rotated (motor slip), ~0 =
//         sensor never fired, ~gateSpan = parked on the magnet.
//   v26 — Module hardware self-diagnostics (no protocol changes to existing
//         commands).  All new commands have a direct m<id>X form and an
//         mX<sn> serial-number form.
//           'T' Hall sensor self-test — steps one revolution and reports a
//               status code distinguishing healthy from stuck-active,
//               stuck-inactive/disconnected, inverted-polarity, and
//               multiple-region (noise) faults:
//                 m<id>T:<code>:<edges>:<activeSamples>\n
//               (0 OK, 1 stuck active, 2 dead/disconnected, 3 multiple,
//                4 inverted polarity).  Counts over exactly one revolution
//               from an off-magnet start so a healthy sensor reads clean.
//           'Q' diagnostics snapshot (no motor movement) — reset cause
//               (RSTFR), boot counter, supply voltage (mV via internal ADC
//               reference), EEPROM write-verify, and current flap index:
//                 m<id>Q:<resetCause>:<bootCount>:<vcc_mV>:<eepromOk>:<curIndex>\n
//           'M' mechanical self-test — drives the motor and measures steps per
//               revolution across several rotations (MECH_TEST_REVS) to detect
//               intermittent missed steps via the spread between revolutions,
//               and a non-moving motor (over a full revolution the Hall sensor
//               must see the magnet enter and leave; if it stays in one state
//               the reel isn't turning → open coil/dead driver/jam):
//                 m<id>M:<code>:<min>:<max>:<spreadTenthsPct>\n
//         Reset cause is captured and cleared at boot; the boot counter and an
//         EEPROM-health scratch byte use addresses 10/11 (previously reserved
//         padding — no change to any existing field or the flap map).
//   v25 — Reclaimed ~600 bytes of SRAM by sharing one work buffer between the
//         incoming mXW restore payload and the outgoing d/A dump assembly (they
//         never overlap in time).  Cleared the IDE's low-memory warning.
//   v24 — Consolidates all changes since the original v6.
//         New in v24 specifically:
//           - 'A' combined all-fields dump: version + EEPROM state in ONE
//             message (m<id>A, broadcast-staggered m*A with optional ID range,
//             and mXA<sn> by serial number).  'v' and 'd' remain unchanged.
//           - Negative nudge ('s' command) now works on the one-way reel.
//           - All user-editable settings gathered into one USER CONFIGURATION
//             block at the top (firmware version, pins, flap count, motor speed,
//             advertisement and broadcast-reply timing, watchdog timeout).
//         Functional additions since v6:
//           - Dynamic module IDs via the ATtiny SIGROW serial number, with
//             advertisement of unprovisioned modules and serial-number-based
//             provisioning commands (mXH/mXI/mXD/mXF/mXW/mXA).
//           - Variable-length address parser (v6 two-char zero-padded format
//             still accepted); '+' index command; 'v' version report; 'F'
//             factory-reset; 'R' de-provision; 'd' single-line EEPROM dump.
//           - Scalable broadcast queries (m*v / m*A with optional <lo>-<hi>
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
//           - Broadcast reply slotting and DE timing tuned for reliable
//             multi-module replies; direct v/d/A reply synchronously.
// ==============================================================================

#include <Arduino.h>
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
enum PendingReply { REPLY_NONE, REPLY_VERSION, REPLY_ALL };

// ── Forward declarations ─────────────────────────────────────────────────────
// In a .cpp the Arduino IDE's automatic prototype generation does not run, so
// every function used before its definition is declared here.  (Building as
// .cpp also avoids the .ino preprocessor mis-parsing apostrophes in comments.)
int flapIndexOf(char target);
void readSerialNumber();
bool serialMatches(const char *candidate);
uint16_t readVccMillivolts();
bool eepromHealthOk();
void seedRandom();
void updateIdChars();
void schedulePendingReply(PendingReply type, bool isBroadcast);
void eepromUpdateByte(int addr, uint8_t value);
void saveHomeOffset();
void saveTotalSteps();
void saveHallActive();
void saveState();
char flapCharAt(int i);
void loadFlapConfig();
void setFlapChars(const char *src, int len);
void applyFlapConfig();
void clearFlapMap();
void txBegin();
void txEnd();
void sendLine(const char *s);
void printModuleId();
void dumpEeprom();
void dumpAll();
void scheduleNextAdvertisement();
bool sendAdvertisement();
void resetProvisioning();
void sendVersionResponse();
void resetEepromDefaults();
void restoreEepromFromPayload(const char *payload);
void applyStep(const uint8_t *step);
void stepBackward(int steps);
void releaseMotor();
bool hallActive();
bool detectHallPolarity();
void nudge(int n);
bool homeModule();
int measureStepCount();
void calibrateModule();
void hallSelfTest();
void reportDiagnostics();
void mechanicalTest(int requestedRevs);
void moveToIndex(int targetIndex);
void moveToChar(char targetChar);
void gotoRawStep(long target);


// ==============================================================================
// ████  USER CONFIGURATION  —  edit these to match your hardware / preferences ██
// ==============================================================================
// Everything a typical user might want to change by hand is gathered here.
// Lower-level implementation constants (EEPROM addresses, buffer sizes, the
// EEPROM magic byte) live further down and normally should NOT be touched.

// ── Firmware version ──────────────────────────────────────────────────────────
// Returned by the 'v', 'A', and mXA commands.  Bump when you change behaviour.
const char FIRMWARE_VERSION[] = "31";

// ── Pin assignments ───────────────────────────────────────────────────────────
// RS-485 transceiver
const int RS485_RX = 3;     // microcontroller RX  (from transceiver RO)
const int RS485_TX = 1;     // microcontroller TX  (to   transceiver DI)
const int RS485_DE = 2;     // driver enable       (HIGH = transmit)
// Stepper coils (via ULN2003 or similar)
#define IN1 9
#define IN2 8
#define IN3 7
#define IN4 6
// Hall-effect home sensor.  The firmware AUTO-DETECTS which logic level means
// "magnet present" (see hallActiveLevel / detectHallPolarity), so it adapts to
// either sensor wiring polarity automatically.  HALL_DEFAULT_ACTIVE is only the
// fallback used before the first detection (an A3144 pulls the line LOW at the
// magnet, so LOW is the sensible default).
#define HALL_PIN 4
#define HALL_DEFAULT_ACTIVE LOW

// ── Mechanical ────────────────────────────────────────────────────────────────
#define MAX_FLAPS 64        // MAXIMUM physical flaps = capacity of the EEPROM flap
                            // map and char buffers (compile-time upper bound).
                            // The ACTIVE count is the runtime `flapCount` below,
                            // configurable via the 'N' command (defaults to this).
const int stepDelay = 1;    // ms between half-steps (lower = faster, less torque)

// ── Advertisement timing (unprovisioned modules) ─────────────────────────────
const unsigned long ADVERTISE_BASE_MS   = 10000UL; // base interval between adverts
const unsigned long ADVERTISE_JITTER_MS =  5000UL; // max extra random jitter

// ── Broadcast reply slot timing ──────────────────────────────────────────────
// When many modules answer one broadcast (m*v / m*A) each transmits in its own
// time slot indexed by module ID.  A slot MUST be comfortably wider than the
// worst-case frame time at 9600 baud or adjacent frames collide:
//   * a version  ('v') frame is ~40 ms  → REPLY_SLOT_MS     = 100 ms
//   * a combined ('A') frame is ~570 ms → REPLY_ALL_SLOT_MS = 700 ms
// REPLY_DIRECT_MS is the settling delay before a direct (non-broadcast) reply.
// REPLY_LEADIN_MS is a fixed offset before the first broadcast slot so the
// lowest ID waits for the controller's transceiver turnaround.
const unsigned long REPLY_DIRECT_MS   =  30UL;
const unsigned long REPLY_SLOT_MS     = 100UL;
const unsigned long REPLY_ALL_SLOT_MS = 700UL;
const unsigned long REPLY_LEADIN_MS   =  30UL;

// ── Carrier-sense window before transmitting an advertisement (ms) ───────────
const unsigned long CSMA_LISTEN_MS = 20UL;

// ── Watchdog timeout ──────────────────────────────────────────────────────────
// Must exceed the longest blocking section between wdt_reset() calls.  Long
// motor moves call wdt_reset() internally, so this only needs to cover a normal
// loop iteration with margin.
#define WDT_TIMEOUT WDTO_2S

// ── Mechanical self-test ('M') ────────────────────────────────────────────────
// Number of revolutions sampled to assess steps-per-rev consistency.  More revs
// give a better estimate of intermittent missed steps but take longer (each rev
// is ~4 s at the default step delay).  MECH_TEST_REVS is the default used by a
// bare 'm<id>M'; a request may override it with 'm<id>M<n>', clamped to
// [MECH_TEST_REVS_MIN, MECH_TEST_REVS_MAX].  The MAX also sizes the per-rotation
// result buffer, so raising it costs a little SRAM (one long per rotation).
#define MECH_TEST_REVS     5
#define MECH_TEST_REVS_MIN 5
#define MECH_TEST_REVS_MAX 20
// ==============================================================================
// ████  END USER CONFIGURATION  ████████████████████████████████████████████████
// ==============================================================================


// ==========================================
//            FLAP CHARACTER SET
// ==========================================
// DEFAULT ordered set of characters on the reel (index = physical flap
// position).  Stored in flash (PROGMEM) rather than as an Arduino String to
// avoid using SRAM/heap on the 2 KB-SRAM ATtiny1616.  This is only the
// compile-time FALLBACK: the live set can be overridden per module via the 'N'
// command (then persisted in EEPROM and read back through flapCharAt()).  If you
// change this, keep its length <= MAX_FLAPS (set in USER CONFIGURATION above).
const char FLAP_CHARS[] PROGMEM =
  " ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789!@#$&()-+=;q:%'.,/?*roygbpw";
// flapIndexOf() / flapCharAt() are defined after the EEPROM addresses below
// (they reference ADDR_FLAP_CHARS).

// ── Ranged broadcast query state (runtime, not user-config) ───────────────────
// Command form:  m*v<lo>-<hi>\n  or  m*A<lo>-<hi>\n  (e.g. "m*v0-49\n")
// Only modules whose ID is within [lo, hi] respond, each in a slot relative to
// lo, so the controller can poll the bus one batch at a time and re-issue any
// batch that comes back short.  Plain "m*v\n" / "m*A\n" = the full 0-254 range.
int  replyRangeLo = 0;    // inclusive low bound for the current ranged query
int  replyRangeHi = 254;  // inclusive high bound for the current ranged query

// Which reply type a broadcast range query (state 18) should schedule once the
// optional "<lo>-<hi>" range has been parsed.  Set when entering state 18 from
// either 'v' (REPLY_VERSION) or 'A' (REPLY_ALL).
PendingReply broadcastReplyType = REPLY_VERSION;

// ---- EEPROM addresses (unchanged from v6) ----
const int ADDR_INIT        = 0;
const int ADDR_HOME_OFFSET = 1;
const int ADDR_TOTAL_STEPS = 3;
const int ADDR_MODULE_ID   = 5;
const int ADDR_AUTO_HOME   = 6;
const int ADDR_SAVED_POS   = 7;
const int ADDR_SAVED_INDEX = 9;
// Addresses 10 and 11 were "reserved / padding" in the v6 layout (between the
// saved index at 9 and the flap map at 12).  v26 puts them to use for
// diagnostics WITHOUT disturbing any existing field or the map start:
const int ADDR_BOOT_COUNT  = 10;  // 1 byte, wraps at 255 — boots since last reset to 0
const int ADDR_EE_SCRATCH  = 11;  // 1 byte, used by the EEPROM write-read-verify health check
const int ADDR_MAP_START   = 12;  // 128 bytes: 64 × uint16_t flap positions (runs to addr 139)
// Hall sensor active level, stored AFTER the flap map (addr 140) so it disturbs
// no v6-era field.  1 byte: 0 = active LOW, 1 = active HIGH, 0xFF = not yet
// detected (fall back to HALL_DEFAULT_ACTIVE).  Auto-detected at calibration.
const int ADDR_HALL_ACTIVE = 140;
// Runtime-configurable flap set (the 'N' command), stored in the previously
// unused tail of the EEPROM so no v6-era field moves:
//   141        flap count (1..MAX_FLAPS; 0xFF/out-of-range = use default 64)
//   142..205   flap char set (MAX_FLAPS bytes; 0xFF in byte 0 = use default set)
const int ADDR_FLAP_COUNT  = 141;        // 1 byte
const int ADDR_FLAP_CHARS  = 142;        // MAX_FLAPS bytes (142..142+MAX_FLAPS-1)

// ── Runtime-configurable flap set (the 'N' command) ───────────────────────────
// flapCount        ACTIVE number of physical flaps.  Drives the valid index
//                  range and the position math (targetIndex * totalStepsPerRev /
//                  flapCount), so it must match the real reel.  Default MAX_FLAPS.
// flapCharsCustom  false → the char set is the compile-time FLAP_CHARS default;
//                  true  → it has been configured via 'N' and lives in EEPROM
//                  (ADDR_FLAP_CHARS).  Read one byte at a time via flapCharAt()
//                  so no extra RAM buffer is needed on the 2 KB chip.
// flapCount and the configured char-set LENGTH are set INDEPENDENTLY by 'N', so a
// mismatch is tolerated: chars past flapCount are simply unreachable, and
// indices with no character just can't be addressed by '-'<char>.
uint8_t flapCount       = MAX_FLAPS;
bool    flapCharsCustom = false;

// Character at flap index i (0..MAX_FLAPS-1): from EEPROM when configured, else
// from the compile-time PROGMEM default.  Returns '\0' past the set's end.
char flapCharAt(int i) {
  if (flapCharsCustom) {
    uint8_t b = EEPROM.read(ADDR_FLAP_CHARS + i);
    return (b == 0xFF) ? '\0' : (char)b;   // 0xFF = unused tail
  }
  return (char)pgm_read_byte(&FLAP_CHARS[i]);
}

// Return the flap index for a character, or -1 if not present.
int flapIndexOf(char target) {
  for (int i = 0; i < MAX_FLAPS; i++) {
    char ch = flapCharAt(i);
    if (ch == '\0') break;       // end of the configured set
    if (ch == target) return i;
  }
  return -1;
}

// The one valid magic value for the current layout (same as v6).
// 0x5E (written by v8/v9) is treated as equivalent during migration.
const uint8_t EEPROM_MAGIC = 0x5D;
const uint8_t EEPROM_MAGIC_V8V9 = 0x5E; // recognise but migrate away from

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
  serialStr[SERIAL_LEN * 2] = '\0';}

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
//        DIAGNOSTICS  (registers & state)
// ==========================================
// ATtiny1616 reset-flag register: RSTCTRL.RSTFR (provided by the core headers).
// Each bit latches the cause of the most recent reset until cleared; we read
// and clear it once at boot so the 'Q' diagnostics command can report why the
// module last reset.
//   bit0 PORF  power-on    bit1 BORF  brown-out   bit2 EXTRF external
//   bit3 WDRF  watchdog    bit4 SWRF  software     bit5 UPDIRF UPDI

uint8_t resetCause = 0;          // captured at boot, raw RSTFR bits
uint8_t bootCount  = 0;          // ADDR_BOOT_COUNT, incremented each boot (wraps)

// Read the ATtiny1616 supply voltage (VCC) in millivolts using the ADC with the
// internal 1.1 V reference measured against VDD.  Returns 0 if unsupported.
// On the megaTinyCore the simplest portable route is analogReadEnh / the core's
// internal-reference helpers, but to avoid depending on a specific core version
// we use the documented ADC0 path: measure the internal reference relative to
// VDD, then VCC = 1024 * 1100mV / adc_reading (10-bit).
uint16_t readVccMillivolts() {
  // Select VDD as reference, internal reference (1.1V) as the input channel.
  // Register details per ATtiny1616 datasheet (ADC0).
  VREF.CTRLA = (VREF.CTRLA & ~VREF_ADC0REFSEL_gm) | VREF_ADC0REFSEL_1V1_gc;
  ADC0.CTRLC = ADC_PRESC_DIV16_gc | ADC_REFSEL_VDDREF_gc | ADC_SAMPCAP_bm;
  ADC0.MUXPOS = ADC_MUXPOS_INTREF_gc;     // measure the internal reference
  ADC0.CTRLA  = ADC_ENABLE_bm;            // 10-bit, single-ended

  // Discard one conversion after switching reference (settling), then average a few.
  uint32_t acc = 0;
  for (uint8_t i = 0; i < 5; i++) {
    ADC0.COMMAND = ADC_STCONV_bm;
    while (ADC0.COMMAND & ADC_STCONV_bm) { /* wait */ }
    uint16_t r = ADC0.RES;
    if (i > 0) acc += r;                   // skip the first (settling) sample
  }
  uint16_t adc = acc / 4;
  if (adc == 0) return 0;
  // VCC = (1024 * 1100) / adc   (1.1V internal ref, 10-bit full scale = 1024)
  return (uint16_t)((1024UL * 1100UL) / adc);
}

// EEPROM write-read-verify health check on a dedicated scratch byte.
// Writes a known pattern, reads it back, restores the previous value.
// Returns true if the cell read back correctly.
bool eepromHealthOk() {
  uint8_t saved = EEPROM.read(ADDR_EE_SCRATCH);
  const uint8_t pattern = 0xA5;
  EEPROM.write(ADDR_EE_SCRATCH, pattern);
  bool ok1 = (EEPROM.read(ADDR_EE_SCRATCH) == pattern);
  EEPROM.write(ADDR_EE_SCRATCH, (uint8_t)~pattern);
  bool ok2 = (EEPROM.read(ADDR_EE_SCRATCH) == (uint8_t)~pattern);
  EEPROM.write(ADDR_EE_SCRATCH, saved);    // restore
  return ok1 && ok2;
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
// Logic level that means "magnet present", auto-detected per module so either
// sensor/magnet orientation works.  Defaults to HALL_DEFAULT_ACTIVE until the
// first detection runs.
uint8_t hallActiveLevel = HALL_DEFAULT_ACTIVE;

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
//               GLOBALS
// ==========================================
SoftwareSerial rs485(RS485_RX, RS485_TX);

long currentStepPos   = 0;
int  currentPhase     = 0;
int  parseState       = 0;
int  currentFlapIndex = -1;
int  tempIndex        = -1;

// ── Fixed-size parser accumulators (no Arduino String / no heap) ──────────────
// Maximum field sizes are bounded by the protocol:
//   idBuffer       : up to 3 ID digits
//   buffer         : numeric payloads (step counts up to ~5 digits) + a little slack
//   snBuffer       : exactly 20 serial-number hex chars
//   restorePayload : <homeOffset>:<totalSteps>:<map> — bounded by MAX_FLAPS entries
// Each buffer carries its own length and a helper to append a char safely; once
// full, further chars are dropped (the frame will be rejected/timed out cleanly).
#define IDBUF_MAX   4
#define NUMBUF_MAX  8
#define SNBUF_MAX   20
#define RESTORE_MAX 600

// ── Shared large work buffer ──────────────────────────────────────────────────
// One buffer serves BOTH the incoming mXW restore payload AND the outgoing d/A
// dump assembly.  These never overlap in time: a restore payload is consumed
// the instant its terminator arrives (parse state 17), and the dump commands
// run in unrelated parser states — a module is never receiving a restore while
// transmitting a dump.  Sharing one buffer instead of two saves ~600 bytes of
// the ATtiny1616's 2 KB SRAM.  Sized to the larger of the two needs.
#define DUMP_BUF_SIZE 720   // also holds the +flapCount/flapChars 'A' tail (v31)
#if (RESTORE_MAX + 1) > DUMP_BUF_SIZE
  #error "WORK_BUF too small for restore payload"
#endif
static char workBuf[DUMP_BUF_SIZE];

// restoreBuf is an alias into the shared buffer (used while accumulating mXW).
char *const restoreBuf = workBuf;  int restoreLen = 0;

char idBuf[IDBUF_MAX + 1];        int idLen = 0;
char numBuf[NUMBUF_MAX + 1];      int numLen = 0;
char snBuf[SNBUF_MAX + 1];        int snLen = 0;

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
    //   replyTime = now + REPLY_LEADIN_MS + (id - lo) × slotWidth
    //
    // REPLY_LEADIN_MS is a fixed offset applied to EVERY module so that even
    // the lowest ID in the range (slot 0, e.g. module 0 on a full m*v) does not
    // begin transmitting until the controller has finished sending the command
    // and switched its transceiver to receive.  Previously the lowest ID used a
    // +1 slot for this purpose, but a dedicated lead-in is clearer and keeps the
    // slot index aligned with the ID offset (id 0 → slot 0, id 1 → slot 1, ...).
    //
    // The slot WIDTH depends on the reply type: the combined 'A' dump frame is
    // an order of magnitude longer than a version frame, so it needs a much
    // wider slot to avoid adjacent frames overlapping.
    unsigned long slotWidth = (type == REPLY_ALL) ? REPLY_ALL_SLOT_MS : REPLY_SLOT_MS;
    unsigned long slot = REPLY_LEADIN_MS +
                         (unsigned long)(moduleId - replyRangeLo) * slotWidth;
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
// The command letter of a serial-number-addressed provisioning command whose
// action takes no parameters (H/D/A/T/Q/M/F).  All seven share one parse state
// (collect SN → run action) to save flash; provCmd selects the action.
char   provCmd     = 0;

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
void saveHallActive()  { eepromUpdateByte(ADDR_HALL_ACTIVE, hallActiveLevel == HIGH ? 1 : 0); }

// ── Flap-set configuration ('N' command) helpers ──────────────────────────────
// Persist the configured char set to EEPROM: the first `len` bytes (clamped to
// MAX_FLAPS), with the rest of the field marked unused (0xFF), and flag the set
// as custom so flapCharAt() reads EEPROM instead of the PROGMEM default.
void setFlapChars(const char *src, int len) {
  if (len > MAX_FLAPS) len = MAX_FLAPS;
  for (int i = 0; i < MAX_FLAPS; i++)
    eepromUpdateByte(ADDR_FLAP_CHARS + i, (i < len) ? (uint8_t)src[i] : 0xFF);
  flapCharsCustom = true;
}

// Load the flap count and the "char set is custom" flag from EEPROM, falling
// back to defaults when a module has never been configured (0xFF/0x00 on a fresh
// chip or a module upgraded from pre-v31 firmware).
void loadFlapConfig() {
  uint8_t cnt = EEPROM.read(ADDR_FLAP_COUNT);
  flapCount = (cnt >= 1 && cnt <= MAX_FLAPS) ? cnt : MAX_FLAPS;

  uint8_t first = EEPROM.read(ADDR_FLAP_CHARS);
  flapCharsCustom = (first != 0xFF && first != 0x00);
}

// Apply a parsed 'N' command (shared by the direct state 24 and the mXN serial
// form, state 25).  Both parts are optional and independent:
//   numBuf      the count digits     → flapCount (1..MAX_FLAPS)
//   restoreBuf  the chars after ':'  → char set (chars only accumulate after a
//               ':' is seen, so restoreLen>0 already implies "chars supplied")
// An out-of-range count or an empty side is left unchanged.
void applyFlapConfig() {
  if (numLen > 0) {
    long cnt = bufToLong(numBuf, numLen);
    if (cnt >= 1 && cnt <= MAX_FLAPS) {
      flapCount = (uint8_t)cnt;
      eepromUpdateByte(ADDR_FLAP_COUNT, flapCount);
    }
  }
  if (restoreLen > 0) setFlapChars(restoreBuf, restoreLen);
}

// Erase the entire calibrated flap position map (all MAX_FLAPS entries → 0xFFFF
// = uncalibrated).  Shared by factory reset, restore, fresh-EEPROM init and 'e'.
void clearFlapMap() {
  for (int i = 0; i < MAX_FLAPS; i++) {
    uint16_t empty = 0xFFFF;
    eepromUpdate(ADDR_MAP_START + (i * 2), empty);
  }
}

void saveState() {
  if (!autoHomeEnabled) {
    eepromUpdate(ADDR_SAVED_POS,   currentStepPos);
    eepromUpdateByte(ADDR_SAVED_INDEX, (uint8_t)(int8_t)currentFlapIndex);
  }
}

// ==========================================
//          RESPONSE HELPERS
// ==========================================
// RS-485 transmit framing, factored out of every reply path so the driver-enable
// timing lives in one place (and to keep flash use down on the 16 KB part):
//   txBegin  assert DE and let the driver settle before the start bit
//   txEnd    hold DE for the final stop bit, release it, then drain the self-echo
//   sendLine txBegin + one line + '\n' + txEnd (for the pre-assembled dumps)
void txBegin() {
  digitalWrite(RS485_DE, HIGH);
  delayMicroseconds(200);          // driver-enable settle before the start bit
}
void txEnd() {
  delay(2);                        // hold DE until the final stop bit is on the wire
  digitalWrite(RS485_DE, LOW);
  while (rs485.available()) rs485.read();   // discard self-echo
}
void sendLine(const char *s) {
  txBegin();
  rs485.print(s);
  rs485.print("\n");
  txEnd();
}
// Common opening of a streamed reply: enable the driver and emit "m<id><tag>".
void txReplyHeader(const char *tag);    // defined after printModuleId below
void txNumField(long v);

// Print module ID in outbound messages.
// IDs 0-99: always two chars with leading zero, matching v6 format.
// IDs 100-254: three chars (v7+ controllers expect this).
void printModuleId() {
  if (moduleId < 10)       rs485.print("0");
  // (for IDs 10-99 no padding needed; for 100+ no padding needed either)
  rs485.print(moduleId);
}

void txReplyHeader(const char *tag) {
  txBegin();
  rs485.print("m");
  printModuleId();
  rs485.print(tag);
}
// Emit one ":<value>" field of a colon-delimited reply (the dominant pattern in
// the v/T/Q/M/P replies); factored to keep flash down.
void txNumField(long v) {
  rs485.print(":");
  rs485.print(v);
}

// Append the calibrated flap map ("<idx>=<pos>,..." for the ACTIVE flaps with a
// stored position) to buf at offset n; returns the new offset.  Shared by the
// 'd' and 'A' dumps.  All EEPROM reads happen here, before the driver is on.
int appendFlapMap(char *buf, int n) {
  bool first = true;
  for (int i = 0; i < flapCount; i++) {
    uint16_t pos = 0xFFFF;
    EEPROM.get(ADDR_MAP_START + (i * 2), pos);
    if (pos == 0xFFFF) continue;
    if (n > DUMP_BUF_SIZE - 12) break;     // overflow guard
    if (!first) buf[n++] = ',';
    n += snprintf(&buf[n], DUMP_BUF_SIZE - n, "%d=%u", i, (unsigned)pos);
    first = false;
  }
  return n;
}

// Write the "m<id>" prefix (zero-padded to 2 digits for IDs 0-9, like the v6
// wire format) into buf at offset n; returns the new offset.  Shared by the
// 'd' and 'A' dump assembly.
int appendMsgId(char *buf, int n) {
  buf[n++] = 'm';
  if (moduleId < 10) buf[n++] = '0';
  n += snprintf(&buf[n], DUMP_BUF_SIZE - n, "%u", (unsigned)moduleId);
  return n;
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
// Worst case for the plain 'd' dump (all 64 entries, 3-digit ID): ~595 bytes.
// The combined 'A' message adds version + serial + autohome + curindex (~31
// bytes more) and still fits the shared workBuf (DUMP_BUF_SIZE, declared with
// the parser buffers above) with headroom and a guaranteed null terminator.

void dumpEeprom() {
  char *buf = workBuf;              // shared work buffer (see declaration above)
  int n = 0;                        // current write index into buf

  // ── Header: m<id>d:<homeOffset>:<totalSteps>: ────────────────────────
  n = appendMsgId(buf, n);
  n += snprintf(&buf[n], DUMP_BUF_SIZE - n, "d:%d:%d:",
                stepsFromHallToZero, totalStepsPerRev);

  // ── Map entries: <idx>=<pos>,<idx>=<pos>,... ─────────────────────────
  n = appendFlapMap(buf, n);

  buf[n] = '\0';

  // ── Transmit the assembled line in one burst ─────────────────────────
  sendLine(buf);
  parseState = 0;
}

// ==========================================
//      COMBINED "ALL FIELDS" DUMP  ('A')
// ==========================================
// Single message containing EVERYTHING a client could need about a module:
// all fields from the 'v' (version) response AND all fields from the 'd'
// (EEPROM dump) response, so a client can fetch a module's complete state in
// one command instead of two.  The 'v' and 'd' commands are unchanged and
// remain for backward compatibility.
//
// Reply format (single line):
//   m<id>A:<version>:<moduleId>:<serialNumber>:<homeOffset>:<totalSteps>:<autoHome>:<curIndex>:<idx>=<pos>,<idx>=<pos>,...\n
//
// Field-by-field:
//   <version>       firmware version string            (from 'v')
//   <moduleId>      bus ID, 255 = unprovisioned        (from 'v')
//   <serialNumber>  20-hex ATtiny serial number        (from 'v')
//   <homeOffset>    steps from Hall trigger to flap 0  (from 'd')
//   <totalSteps>    steps per revolution               (from 'd')
//   <autoHome>      1 = home on boot, 0 = restore      (new — was implicit)
//   <curIndex>      current flap index, -1 if unknown  (new — live state)
//   <idx>=<pos>,... calibrated flap map (only populated entries; from 'd')
//
// Example (full map omitted for brevity):
//   m38A:30:38:A3F24C0018E7D29B3F01:2832:4096:1:0:0=0,7=342\n
//
// Like dumpEeprom(), the whole line is assembled into a fixed buffer with all
// EEPROM reads done BEFORE the driver is enabled, then sent as one tight burst.
void dumpAll() {
  char *buf = workBuf;              // shared work buffer (see declaration above)
  int n = 0;

  // ── Header + scalar fields (all reads done before DE is asserted) ────
  n = appendMsgId(buf, n);
  n += snprintf(&buf[n], DUMP_BUF_SIZE - n, "A:%s:%u:%s:%d:%d:%d:%d:",
                FIRMWARE_VERSION,
                (unsigned)moduleId,
                serialStr,
                stepsFromHallToZero,
                totalStepsPerRev,
                autoHomeEnabled ? 1 : 0,
                currentFlapIndex);

  // ── Flap map (only populated, ACTIVE entries) ────────────────────────
  n = appendFlapMap(buf, n);

  // ── Configurable flap set ────────────────────────────────────────────
  // Appended AFTER the map so fields up to and including the map list stay
  // byte-identical to pre-v31 'A' replies (old controllers ignore the tail).
  //   :<flapCount>:<flapChars>
  // flapChars is the FINAL field and may itself contain ':'/','/'=' — read it
  // verbatim to end-of-line (do not split it further).
  n += snprintf(&buf[n], DUMP_BUF_SIZE - n, ":%u:", (unsigned)flapCount);
  for (int i = 0; i < MAX_FLAPS; i++) {
    char ch = flapCharAt(i);
    if (ch == '\0') break;                  // end of the configured set
    if (n >= DUMP_BUF_SIZE - 2) break;      // leave room for the null
    buf[n++] = ch;
  }

  buf[n] = '\0';

  // ── Transmit in one burst ────────────────────────────────────────────
  sendLine(buf);
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

  txBegin();
  rs485.print("mXadv:");
  rs485.print(serialStr);
  rs485.print("\n");
  txEnd();
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
  txReplyHeader("v:");
  rs485.print(FIRMWARE_VERSION);
  txNumField(moduleId);
  rs485.print(":");
  rs485.print(serialStr);
  rs485.print("\n");
  txEnd();
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
  clearFlapMap();
  // Mark Hall polarity as undetected so it is re-detected on the next boot or
  // calibration rather than assuming a possibly-wrong level.
  eepromUpdateByte(ADDR_HALL_ACTIVE, 0xFF);
  hallActiveLevel = HALL_DEFAULT_ACTIVE;

  // Restore the flap set to the compile-time defaults: mark EEPROM unset (0xFF)
  // so flapCharAt() falls back to the PROGMEM default set immediately.
  eepromUpdateByte(ADDR_FLAP_COUNT, 0xFF);
  eepromUpdateByte(ADDR_FLAP_CHARS, 0xFF);
  flapCount       = MAX_FLAPS;
  flapCharsCustom = false;
}

// ==========================================
//       EEPROM RESTORE FROM PAYLOAD
// ==========================================
// Parses the payload portion of an mXW restore command and writes all
// calibration fields to EEPROM.  Module ID and magic byte are preserved.
//
// Payload format (the part after mXW<sn>:):
//   <homeOffset>:<totalSteps>:<idx>=<pos>,<idx>=<pos>,...[:<flapCount>:<flapChars>]
//
// Examples:
//   2832:4096:                         (no calibrated flap positions)
//   2832:4096:0=0,7=342,12=683
//   2832:4096:0=0,7=342:64: ABC...     (with the optional flap-set tail)
//
// All MAX_FLAPS map entries are first cleared to 0xFFFF, then only the entries
// present in the payload are written.  The optional trailing
// ":<flapCount>:<flapChars>" (as emitted by the 'A' dump) restores the
// configurable flap set; when absent, the flap set is left unchanged.  Parses a
// plain C string (no Arduino String / no heap).
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
  clearFlapMap();

  // ── Parse and write flap map entries: idx=pos,idx=pos,... ────────────
  while (*p) {
    long flapIdx = strtol(p, &end, 10);
    if (end == p || *end != '=') break;   // no number, or missing '='
    p = end + 1;
    long flapPos = strtol(p, &end, 10);
    if (end == p) break;                   // no number after '='
    p = end;

    if (flapIdx >= 0 && flapIdx < MAX_FLAPS) {
      uint16_t v = (uint16_t)flapPos;
      eepromUpdate(ADDR_MAP_START + (flapIdx * 2), v);
    }
    if (*p == ',') p++;                     // step over separator
    else break;                            // end of list (or malformed)
  }

  // ── Optional flap-set tail: :<flapCount>:<flapChars> ──────────────────
  // Mirrors the tail of the 'A' dump, so an 'A' reply can be restored verbatim.
  // The map loop above stops at the ':' that precedes <flapCount>.  Absent tail
  // (older/hand-written payloads) → flap config left unchanged.
  if (*p == ':') {
    p++;
    long cnt = strtol(p, &end, 10);
    if (end != p && cnt >= 1 && cnt <= MAX_FLAPS) {
      flapCount = (uint8_t)cnt;
      eepromUpdateByte(ADDR_FLAP_COUNT, flapCount);
    }
    p = end;
    if (*p == ':' && p[1] != '\0')
      setFlapChars(p + 1, strlen(p + 1));   // rest of line = chars (verbatim)
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

bool hallActive() { return digitalRead(HALL_PIN) == hallActiveLevel; }

// Auto-detect which logic level means "magnet present" by observing the raw pin
// over one full revolution.  The home magnet covers only a small fraction of a
// revolution (~150-180 of ~4096 steps), so whichever raw level is the MINORITY
// is the active (magnet) level; the majority level is the background.  This
// adapts the firmware to either sensor wiring polarity automatically.
//
// Returns true and updates hallActiveLevel (without saving) if a clear single
// minority region was found.  Returns false WITHOUT changing hallActiveLevel if
// the pin never changes across the revolution — that is not a polarity question
// but a "no magnet seen" condition (magnet flipped to the wrong pole, or a dead
// or disconnected sensor), which the caller should surface rather than guess at.
//
// Drives the motor one revolution; leaves it released.
bool detectHallPolarity() {
  long lowSamples  = 0;
  long highSamples = 0;
  // Sample the RAW pin (not hallActive(), which depends on the very thing we are
  // trying to determine) once per step across a full revolution.
  for (long i = 0; i < (long)totalStepsPerRev; i++) {
    if (digitalRead(HALL_PIN) == LOW) lowSamples++;
    else                              highSamples++;
    stepBackward(1);
  }
  releaseMotor();

  // If essentially all samples are the same level, the sensor never saw the
  // magnet change state → no usable home signal.  Do not change polarity.
  long total   = lowSamples + highSamples;
  long minority = (lowSamples < highSamples) ? lowSamples : highSamples;
  if (total == 0) return false;
  // Require the minority region to be a small but non-zero fraction of the rev
  // (a real magnet), and not so large that the signal is ambiguous.  "Never
  // changes" shows up as minority == 0.
  if (minority == 0) return false;
  if (minority > (totalStepsPerRev / 4)) return false;  // too wide to be a magnet

  // The minority level is the active (magnet) level.
  hallActiveLevel = (lowSamples < highSamples) ? LOW : HIGH;
  return true;
}

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

// Measures steps-per-revolution once and returns it, or 0 if the measurement is
// invalid (no magnet seen / stuck sensor).  Self-contained and side-effect-free
// apart from moving the motor — it does NOT save anything.  Used by the
// first-boot auto-measure path; calibrateModule has its own inline measurement
// plus the RS-485 reply.  Assumes hall polarity is already correct.
int measureStepCount() {
  long safety = 0;
  while (hallActive() && safety < 4000) { stepBackward(1); safety++; }
  safety = 0;
  while (!hallActive() && safety < (totalStepsPerRev + 1000)) { stepBackward(1); safety++; }
  safety = 0;
  while (hallActive() && safety < 2000) { stepBackward(1); safety++; }
  int measured = 0;
  while (!hallActive() && measured < (totalStepsPerRev * 2 + 1000)) { stepBackward(1); measured++; }
  safety = 0;
  while (hallActive() && safety < 2000) { stepBackward(1); measured++; safety++; }
  releaseMotor();
  return (measured > 500 && measured < 8000) ? measured : 0;
}

void calibrateModule() {
  // First, auto-detect the Hall sensor's active level so this module works with
  // either magnet/sensor orientation.  If detection fails (the sensor never
  // changes state across a revolution — flipped magnet or dead/disconnected
  // sensor), keep the previous level and let the revolution measurement below
  // fail its sanity check, which reports the problem.
  if (detectHallPolarity()) {
    saveHallActive();
  }

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
  // Response format matches v6: m<2-char-id>:<steps>\n for IDs 0-99.
  // On a rejected measurement we still report the raw count so the controller
  // can see something went wrong.
  txReplyHeader(":");
  rs485.print(measuredSteps);
  rs485.print("\n");
  txEnd();

  if (valid) {
    totalStepsPerRev = measuredSteps;
    saveTotalSteps();
  }
  homeModule();
  saveState();
}

// ==========================================
//          HALL SENSOR SELF-TEST  ('T')
// ==========================================
// Actively probes the Hall home sensor and reports a status code so a
// controller can detect a malfunctioning, disconnected, or mis-wired sensor
// without having to infer it from calibration numbers.
//
// Method: step one full revolution (plus a margin) and watch the sensor.
//   - count how many SAMPLES read active           (activeSamples)
//   - count how many inactive→active TRANSITIONS    (edges = home pulses/rev)
// A healthy sensor produces exactly ONE active region per revolution: a modest
// run of active samples bracketed by inactive samples, i.e. edges == 1.
//
// Status codes (reported as m<id>T:<code>:<edges>:<activeSamples>\n):
//   0  OK            edges == 1 and the active region is a sane width
//   1  STUCK_ACTIVE  sensor active for (nearly) the whole revolution
//                    → shorted, magnet jammed at sensor, or mis-wiring
//   2  STUCK_INACTIVE never went active across a full revolution
//                    → disconnected, dead, missing magnet, or inverted polarity
//   3  MULTIPLE       more than one active region per revolution
//                    → stray magnet, electrical noise, or a chattering sensor
//
// The extra <edges> and <activeSamples> fields are advisory detail; a client
// that only cares about pass/fail can look at <code> alone.  The test leaves
// the reel re-homed (or flagged unknown) afterward, like calibrate does.
void hallSelfTest() {
  // Probe over EXACTLY one revolution.  Counting over more than one rev would
  // cross the single home region twice and make a healthy sensor look like it
  // has multiple active regions.
  //
  // Before counting, step to a known INACTIVE start so the home region isn't
  // split across the count's start/end boundary (which would miscount edges).
  // Bounded so a stuck-active sensor can't spin here forever.
  long runIn = 0;
  long runInMax = (long)totalStepsPerRev + (totalStepsPerRev / 10);
  while (hallActive() && runIn < runInMax) { stepBackward(1); runIn++; }
  // (If the sensor is stuck active, runIn saturates and we start anyway; the
  //  classification below still flags it correctly as STUCK_ACTIVE.)

  long activeSamples  = 0;
  int  risingEdges    = 0;   // inactive → active transitions
  int  fallingEdges   = 0;   // active → inactive transitions
  bool prevActive     = hallActive();
  if (prevActive) activeSamples++;

  for (long i = 0; i < (long)totalStepsPerRev; i++) {
    stepBackward(1);
    bool now = hallActive();
    if (now) activeSamples++;
    if ( now && !prevActive) risingEdges++;   // inactive → active
    if (!now &&  prevActive) fallingEdges++;   // active  → inactive
    prevActive = now;
  }
  releaseMotor();

  // Thresholds (10% of a revolution).  A healthy home magnet covers only a
  // small fraction of the revolution, so "most of the rev" vs "a brief region"
  // are far apart and unambiguous.
  long mostlyActive = (long)totalStepsPerRev - (totalStepsPerRev / 10);  // ≥90% active
  long briefRegion  = totalStepsPerRev / 10;                              // ≤10% active

  // Classify:
  //   0 OK             one short active region          (active LOW at magnet)
  //   1 STUCK_ACTIVE   active the whole rev, NO dips     (shorted / jammed)
  //   2 STUCK_INACTIVE never active                      (dead / disconnected)
  //   3 MULTIPLE       more than one active region       (noise / stray magnet)
  //   4 INVERTED       active most of the rev with ONE   (sensor wired with
  //                    brief inactive dip at the magnet   reversed polarity)
  //
  // The inverted case is the mirror image of OK: instead of one brief ACTIVE
  // region in an inactive background, it shows one brief INACTIVE region in an
  // active background.  We detect that as "mostly active, but it did fall
  // inactive exactly once" — distinct from stuck-active, which never falls.
  uint8_t code;
  if (activeSamples == 0) {
    code = 2;  // STUCK_INACTIVE — never asserted at all
  } else if (activeSamples >= mostlyActive) {
    // Active for ~the whole revolution.  Did it ever dip inactive?
    if (fallingEdges >= 1 &&
        ((long)totalStepsPerRev - activeSamples) <= briefRegion) {
      code = 4;  // INVERTED_POLARITY — one brief inactive dip in an active field
    } else {
      code = 1;  // STUCK_ACTIVE — never released
    }
  } else if (risingEdges > 1) {
    code = 3;  // MULTIPLE active regions
  } else {
    code = 0;  // OK — exactly one active region of sane width
  }

  // ── Transmit: m<id>T:<code>:<risingEdges>:<activeSamples>:<fallingEdges>\n ──
  // fallingEdges (appended in v28) is opt-in detail: a clean sensor shows
  // risingEdges == fallingEdges == 1.  A mismatch (e.g. 1 rising, 0 falling)
  // indicates the active region straddles the revolution boundary or the sensor
  // releases raggedly.  Old parsers ignore the trailing field.
  txReplyHeader("T:");
  rs485.print(code);
  txNumField(risingEdges);
  txNumField(activeSamples);
  txNumField(fallingEdges);
  rs485.print("\n");
  txEnd();

  // Leave the module in a known state.
  homeModule();
  saveState();
  parseState = 0;
}

// ==========================================
//        DIAGNOSTICS REPORT  ('Q')
// ==========================================
// Instantaneous health snapshot — no motor movement.  Reports:
//   m<id>Q:<resetCause>:<bootCount>:<vcc_mV>:<eepromOk>:<curIndex>\n
//     resetCause  raw RSTFR bits from the last reset (see RSTCTRL.RSTFR notes;
//                 e.g. 0x08 = watchdog, 0x02 = brown-out, 0x01 = power-on)
//     bootCount   boots since the counter was last reset (wraps at 255)
//     vcc_mV      measured supply voltage in millivolts (0 if unavailable)
//     eepromOk    1 = EEPROM scratch cell write-read-verify passed, else 0
//     curIndex    current flap index (-1 = unknown / needs homing)
// A controller can watch for: a non-zero watchdog/brown-out reset bit, a VCC
// sagging below ~4.5 V under load, eepromOk == 0, or a climbing bootCount that
// indicates the module is silently resetting in the field.
void reportDiagnostics() {
  uint16_t vcc = readVccMillivolts();
  uint8_t  eeOk = eepromHealthOk() ? 1 : 0;

  txReplyHeader("Q:");
  rs485.print(resetCause);
  txNumField(bootCount);
  txNumField(vcc);
  txNumField(eeOk);
  txNumField(currentFlapIndex);
  rs485.print("\n");
  txEnd();
  parseState = 0;
}

// ==========================================
//        MECHANICAL SELF-TEST  ('M')
// ==========================================
// Active test that drives the motor to detect missed steps (mechanical drag,
// slipping coupler, weak supply, failing driver) and a non-moving motor (open
// coil / dead driver channel).  Reports:
//   m<id>M:<code>:<min>:<max>:<spreadTenthsPct>\n
//     code 0 = OK            all revolutions agree within tolerance
//          1 = INCONSISTENT  spread between revolutions exceeds tolerance →
//                            intermittent missed steps (drag, weak supply,
//                            failing driver)
//          2 = NO_MOTION     the Hall sensor never transitioned over a full
//                            revolution → motor isn't turning (open coil, dead
//                            driver, jam) OR the Hall sensor is dead (run 'T')
//     min, max          = smallest / largest steps-per-revolution measured
//                         across MECH_TEST_REVS revolutions
//     spreadTenthsPct   = (max-min)/average as a percentage × 10 (one decimal),
//                         e.g. 23 means 2.3% spread.  This is the consistency
//                         metric: small = repeatable, large = dropping steps.
//
// Method: home, then measure steps-per-revolution MECH_TEST_REVS times in a row
// and compare.  A healthy module returns nearly-identical counts every rev (a
// small spread); intermittent missed steps show up as a larger spread that two
// samples alone might miss.
//
// Returns the module re-homed afterward.

// Measures one full revolution (return value = steps per rev).  Also reports the
// width of the magnet's active region via the out-parameter magnetWidth, which
// the mechanical test averages — a per-revolution sensor-gap signal that no
// single-shot measurement can reveal (a wobbling shaft or loosening magnet makes
// this vary across revolutions).  Pass nullptr if the width isn't needed.
static long measureOneRev(long *magnetWidth = nullptr) {
  // From somewhere off the magnet, step until the magnet edge, counting steps
  // for one full revolution back to the next edge.  Bounded for safety.
  long safety = 0;
  // ensure we start off-magnet
  while (hallActive() && safety < (totalStepsPerRev + 500)) { stepBackward(1); safety++; }
  // advance to the first edge
  safety = 0;
  while (!hallActive() && safety < (totalStepsPerRev + 500)) { stepBackward(1); safety++; }
  // step through the magnet region — this loop count IS the magnet's active width
  long width = 0;
  while (hallActive() && width < 2000) { stepBackward(1); width++; }
  if (magnetWidth) *magnetWidth = width;
  // count a full revolution until we return through the magnet
  long count = 0;
  while (!hallActive() && count < (totalStepsPerRev * 2 + 1000)) { stepBackward(1); count++; }
  long s2 = 0;
  while (hallActive() && s2 < 2000) { stepBackward(1); count++; s2++; }
  return count;
}

void mechanicalTest(int requestedRevs) {
  // Clamp the requested rotation count to the configured bounds.  A bare
  // 'm<id>M' passes MECH_TEST_REVS (the default); 'm<id>M<n>' passes n.
  int nReq = requestedRevs;
  if (nReq < MECH_TEST_REVS_MIN) nReq = MECH_TEST_REVS_MIN;
  if (nReq > MECH_TEST_REVS_MAX) nReq = MECH_TEST_REVS_MAX;

  // Detect "no motion" by driving a FULL revolution and checking that the Hall
  // sensor sees the magnet enter and leave.  A quarter-rev check is unreliable:
  // the home magnet is active for only a narrow window, so if the reel starts
  // with the magnet far from the sensor, the motor can turn correctly yet the
  // sensor never changes within a quarter turn — a false NO_MOTION.
  //
  // NOTE: this test observes motion THROUGH the Hall sensor, so a working motor
  // with a DEAD sensor also reports NO_MOTION.  Run the 'T' Hall self-test first
  // to confirm the sensor is healthy, then interpret an M code 2 accordingly.
  // Count active samples across the gate sweep too, so a NO_MOTION result can
  // report WHAT it observed rather than just 0:0.  This distinguishes "sensor
  // never went active" (under-rotation / motor slip / wrong magnet) from
  // "sensor never went inactive" (parked on magnet, stuck active).
  long gateActive = 0;
  bool startState  = hallActive();
  bool sawActive   = startState;
  bool sawInactive = !startState;
  if (startState) gateActive++;
  long gateSpan = (long)totalStepsPerRev + (totalStepsPerRev / 10);
  for (long i = 0; i < gateSpan; i++) {
    stepBackward(1);
    if (hallActive()) { sawActive = true; gateActive++; }
    else                sawInactive = true;
  }
  bool moved = sawActive && sawInactive;

  uint8_t code;
  long mn = 0, mx = 0;              // min/max steps-per-rev (0 when no motion)
  long spreadTenthsPct = 0;
  long revs[MECH_TEST_REVS_MAX];   // per-rotation step counts (sized to the max)
  int  nRevs = 0;                  // how many rotation values are valid
  long avgMagnetWidth = 0;         // average active-region width across the revs

  for (int r = 0; r < nReq; r++) revs[r] = 0;

  if (!moved) {
    code = 2;  // NO_MOTION — gate never saw both Hall states over a full rev
    // mn/mx/spread/revs stay 0; the gate fields below carry the diagnostic detail.
  } else {
    // Sample the requested number of revolutions: track min, max, sum (for the
    // average step count), each raw rotation value, and the magnet width per rev.
    long sum = 0;
    long widthSum = 0;
    for (int r = 0; r < nReq; r++) {
      long width = 0;
      long m = measureOneRev(&width);
      revs[r] = m;
      widthSum += width;
      if (r == 0) { mn = mx = m; }
      else {
        if (m < mn) mn = m;
        if (m > mx) mx = m;
      }
      sum += m;
    }
    nRevs = nReq;
    long avg = sum / nReq;
    avgMagnetWidth = widthSum / nReq;

    // Spread as tenths of a percent of the average: (max-min)/avg × 1000.
    // Integer math, avg is always well above zero for a turning motor.
    spreadTenthsPct = (avg > 0) ? ((mx - mn) * 1000L) / avg : 0;

    // Tolerance: 5.0% spread (= 50 tenths) is the pass/fail line.
    code = (spreadTenthsPct <= 50) ? 0 : 1;   // 0 OK, 1 INCONSISTENT
  }
  releaseMotor();

  // Reply format (all original fields unchanged; new fields APPENDED so existing
  // parsers keep working — they simply ignore anything past gateSpan):
  //   m<id>M:<code>:<min>:<max>:<spread>:<gateActive>:<gateSpan>:<avgMagnetWidth>:<r1>,<r2>,...,<rN>\n
  //     code            0 OK, 1 inconsistent, 2 no motion
  //     min,max         smallest/largest steps-per-rev measured (0 if no motion)
  //     spread          (max-min)/avg in tenths of a percent (0 if no motion)
  //     gateActive      Hall active-sample count during the one-rev motion gate
  //     gateSpan        total steps driven during the gate (~1.1 revolutions)
  //   --- appended in v28 (opt-in; old parsers ignore) -------------------------
  //     avgMagnetWidth  average magnet active-region width across the revs.  A
  //                     value that drifts from the 'T' test's width, or that the
  //                     per-rev raw list shows varying, points to a changing
  //                     sensor-magnet gap (shaft wobble, loosening magnet).
  //     r1..rN          the raw steps-per-rev for each of MECH_TEST_REVS rotations,
  //                     comma-separated.  The TREND across these distinguishes an
  //                     intermittent single glitch from a progressive drift that
  //                     min/max/spread alone cannot.  Empty on a no-motion result.
  //                     Parsers should split on ',' rather than assume a count.
  txReplyHeader("M:");
  rs485.print(code);
  txNumField(mn);
  txNumField(mx);
  txNumField(spreadTenthsPct);
  txNumField(gateActive);
  txNumField(gateSpan);
  txNumField(avgMagnetWidth);
  rs485.print(":");
  for (int r = 0; r < nRevs; r++) {
    if (r > 0) rs485.print(",");
    rs485.print(revs[r]);
  }
  rs485.print("\n");
  txEnd();

  homeModule();
  saveState();
  parseState = 0;
}

// 'g' command: step forward to absolute raw step position `target`, then mark
// the flap index unknown.  Shared by the parser and its idle-timeout path.
void gotoRawStep(long target) {
  long move = target - currentStepPos;
  if (move < 0) move += totalStepsPerRev;
  while (move-- > 0) stepBackward(1);
  releaseMotor(); currentFlapIndex = -2; saveState();
}

void moveToIndex(int targetIndex) {
  if (targetIndex < 0 || targetIndex >= flapCount) return;
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
    : ((long)targetIndex * (long)totalStepsPerRev) / flapCount;

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
  else { homeModule(); saveState(); }   // char not in the flap set → just home
}

// ==========================================
//               SETUP
// ==========================================
void setup() {
  // Capture WHY we just reset (RSTFR latches the cause) before anything clears
  // it, then clear it by writing the bits back so the next reset's cause is
  // recorded cleanly.  Reported later by the 'Q' diagnostics command.
  resetCause = RSTCTRL.RSTFR;
  RSTCTRL.RSTFR = resetCause;   // writing 1s clears the latched flags

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

    // Hall active level (auto-polarity).  0 = LOW, 1 = HIGH, 0xFF = never
    // detected → keep the compile-time default until the first calibration.
    uint8_t hp = EEPROM.read(ADDR_HALL_ACTIVE);
    if (hp == 0)      hallActiveLevel = LOW;
    else if (hp == 1) hallActiveLevel = HIGH;
    // else (0xFF): leave hallActiveLevel at HALL_DEFAULT_ACTIVE

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

    clearFlapMap();
    // Leave Hall polarity undetected (0xFF); first boot below will detect it.
    eepromUpdateByte(ADDR_HALL_ACTIVE, 0xFF);
    // Leave the flap config unset (0xFF) too; loadFlapConfig() falls back to
    // the compile-time defaults until the operator configures it via 'N'.
    eepromUpdateByte(ADDR_FLAP_COUNT, 0xFF);
    eepromUpdateByte(ADDR_FLAP_CHARS, 0xFF);
  }

  // Load the configurable flap count + char set (defaults when never set).
  loadFlapConfig();

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

  // ── First-boot auto-setup (polarity + step count) ──────────────────────────
  // A module with no stored polarity (EEPROM held 0xFF) is freshly flashed or was
  // reflashed without EEPROM preserved.  On that path only, derive the two values
  // the firmware CAN measure for itself, so an unknown module isn't left guessing:
  //   1. Hall polarity — so either sensor/magnet orientation works.
  //   2. Steps per revolution — must NOT be assumed: a 28BYJ-48 is 4096 in
  //      half-step but 2048 in full-step (and clones vary), so a wrong default
  //      would break all motion.  Measured here and the default home offset is
  //      scaled to the measured revolution (the offset itself still needs the
  //      operator's dial-in, but starts from a sane fraction of a real rev rather
  //      than a fixed value that may exceed a shorter revolution).
  // The home offset is NOT derivable and is left for the operator.
  // This never runs on a module with valid preserved EEPROM, so a normal reflash
  // can't trigger a silent re-measure.
  bool freshModule = (EEPROM.read(ADDR_HALL_ACTIVE) == 0xFF);
  if (freshModule) {
    // Polarity first — the step-count measurement depends on hallActive().
    if (detectHallPolarity()) saveHallActive();

    // Then step count, but only commit a VALID measurement; on failure keep the
    // default so a bad reading can't poison totalStepsPerRev.
    int steps = measureStepCount();
    if (steps > 0) {
      totalStepsPerRev = steps;
      saveTotalSteps();
      // Scale the default offset to the measured revolution if the current
      // default no longer fits (e.g. 2832 on a 2048-step reel is past one rev).
      if (stepsFromHallToZero >= totalStepsPerRev) {
        stepsFromHallToZero = totalStepsPerRev / 2;   // sane mid-rev starting point
        saveHomeOffset();
      }
    }
  }

  // ── Home or restore ───────────────────────────────────────────────────────
  if (autoHomeEnabled) {
    homeModule();
    saveState();
  } else {
    EEPROM.get(ADDR_SAVED_POS, currentStepPos);
    if (currentStepPos < 0 || currentStepPos >= (long)totalStepsPerRev) currentStepPos = 0;
    currentFlapIndex = (int8_t)EEPROM.read(ADDR_SAVED_INDEX);
    if (currentFlapIndex < -1 || currentFlapIndex >= flapCount) currentFlapIndex = -1;
  }

  // ── Boot counter (diagnostics) ─────────────────────────────────────────────
  // Increment a wrapping count of boots so the 'Q' command can reveal a module
  // that is silently resetting in the field.  Uses the wear-saving updater.
  bootCount = EEPROM.read(ADDR_BOOT_COUNT);
  if (bootCount == 0xFF) bootCount = 0;   // treat a blank/erased cell as 0
  bootCount++;
  eepromUpdateByte(ADDR_BOOT_COUNT, bootCount);

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
// parseState 13 : prov by SN, parameterless — collect serial number, then run
//                 the action selected by provCmd (H/D/A/T/Q/M/F)
// parseState 14 : 'I' prov — serial number ':' new ID digits
// parseState 17 : 'W' prov — serial number ':' full calibration payload (restore by SN)
// parseState 18 : broadcast 'v'/'A' — optional "<lo>-<hi>" ID range for batched query
// parseState 23 : direct 'M' — optional revolution-count digits (m<id>M<n>)
// parseState 24 : direct 'N' — flap config: <count> ':' <chars>
// parseState 25 : 'N' prov — serial number ':' <count> ':' <chars> (config by SN)

void loop() {
  wdt_reset();   // service the watchdog every iteration

  // ── Deferred reply (broadcast m*v only) ──────────────────────────────────
  // Only broadcast version replies are deferred (to stagger across modules).
  // Direct v/d and mXD reply synchronously in the parser.
  if (pendingReply != REPLY_NONE && (long)(millis() - pendingReplyTime) >= 0) {
    if      (pendingReply == REPLY_VERSION) sendVersionResponse();
    else if (pendingReply == REPLY_ALL)     dumpAll();
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
            // H/D/A/T/Q/M/F take no parameter and share state 13 (collect SN,
            // then run the action selected by provCmd).  I/W/N carry extra
            // fields and have their own states.
            if (c=='H'||c=='D'||c=='A'||c=='T'||c=='Q'||c=='M'||c=='F') {
                                  bufClear(snBuf, snLen); provCmd = c;
                                  parseState = 13; }
            else if (c == 'I') { bufClear(snBuf, snLen); snColonSeen = false; bufClear(numBuf, numLen);
                                  parseState = 14; }
            else if (c == 'W') { bufClear(snBuf, snLen); snColonSeen = false; restoreLen = 0; restoreBuf[0] = '\0';
                                  parseState = 17; }
            else if (c == 'N') { bufClear(snBuf, snLen); bufClear(numBuf, numLen);
                                  restoreLen = 0; restoreBuf[0] = '\0'; tempIndex = 0;
                                  parseState = 25; }
            else                 parseState = 0;

          } else if (match) {
            // Normal addressed / broadcast commands
            switch (c) {
              case '-': parseState = 4; break;
              case '+': bufClear(numBuf, numLen); parseState = 12; break;
              case 'h': homeModule(); saveState(); parseState = 0; break;
              case 'c': calibrateModule();          parseState = 0; break;
              case 'T':
                // Hall sensor self-test.  Long, motor-driven operation, so it's
                // direct-addressed only and synchronous (like 'd'/'A'/'c');
                // broadcast m*T is ignored.
                if (!idWildcard) {
                  hallSelfTest();
                }
                parseState = 0;
                break;
              case 'P':
                // Re-detect Hall sensor polarity (drives one revolution) and
                // report it.  Direct-addressed only.  Reply:
                //   m<id>P:<code>:<level>\n
                //     code  0 = detected OK, 1 = no magnet seen (kept old level)
                //     level 0 = active LOW, 1 = active HIGH (the level now in use)
                if (!idWildcard) {
                  uint8_t pcode = 0;
                  if (detectHallPolarity()) { saveHallActive(); pcode = 0; }
                  else                      { pcode = 1; }
                  txReplyHeader("P:");
                  rs485.print(pcode);
                  txNumField(hallActiveLevel == HIGH ? 1 : 0);
                  rs485.print("\n");
                  txEnd();
                  homeModule();   // re-home now that polarity may have changed
                }
                parseState = 0;
                break;
              case 'Q':
                // Diagnostics snapshot (reset cause, boot count, VCC, EEPROM
                // health, current index).  Instantaneous read, no movement.
                // Direct-addressed only; broadcast m*Q is ignored.
                if (!idWildcard) {
                  reportDiagnostics();
                }
                parseState = 0;
                break;
              case 'M':
                // Mechanical self-test (missed-step / no-motion detection).
                // Long, motor-driven; direct-addressed only and synchronous.
                // Accepts an OPTIONAL revolution count: m<id>M<n> (n clamped to
                // [MECH_TEST_REVS_MIN, MECH_TEST_REVS_MAX]); bare m<id>M uses the
                // default.  Collect any digits in state 23, then run on terminator.
                if (!idWildcard) {
                  bufClear(numBuf, numLen);
                  parseState = 23;
                } else {
                  parseState = 0;
                }
                break;
              case 'o': bufClear(numBuf, numLen); parseState = 5; break;
              case 't': bufClear(numBuf, numLen); parseState = 6; break;
              case 's': bufClear(numBuf, numLen); parseState = 7; break;
              case 'g': bufClear(numBuf, numLen); parseState = 8; break;
              case 'w': bufClear(numBuf, numLen); tempIndex = -1; parseState = 9; break;
              case 'i': bufClear(numBuf, numLen); parseState = 10; break;
              case 'a': bufClear(numBuf, numLen); parseState = 11; break;
              case 'N':
                // Configure the flap set: m<id>N<count>:<chars>
                // Both parts optional (count before ':', chars after).  Applies
                // to a direct ID or a broadcast m*N (whole panel at once); no
                // reply.  Parsed in state 24.  numBuf collects the count digits;
                // the chars accumulate into the shared work buffer (never in use
                // for a restore at the same time); snColonSeen marks the ':'.
                bufClear(numBuf, numLen);
                restoreLen = 0; restoreBuf[0] = '\0';
                snColonSeen = false;
                parseState = 24;
                break;
              case 'd':
                // Dumps answered ONLY when addressed directly, SYNCHRONOUSLY,
                // exactly like mXD.  Broadcast dumps are ignored (too long to
                // stagger across a reply slot).
                if (!idWildcard) {
                  dumpEeprom();
                }
                parseState = 0;
                break;
              case 'A':
                // Combined "all fields" dump (version + EEPROM in one message).
                if (idWildcard) {
                  // Broadcast all-fields query — may carry an optional
                  // "<lo>-<hi>" range.  Collect it in state 18; replies are
                  // staggered via the deferred path using the wider 'A' slot.
                  // Because each 'A' frame is long (~570 ms), a full sweep is
                  // slow — prefer ranged batches (m*A<lo>-<hi>) for big buses.
                  bufClear(numBuf, numLen);
                  replyRangeLo = 0;
                  replyRangeHi = 254;
                  broadcastReplyType = REPLY_ALL;
                  parseState = 18;
                } else {
                  // Direct query — reply SYNCHRONOUSLY and immediately, like mXD.
                  dumpAll();
                  parseState = 0;
                }
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
                  broadcastReplyType = REPLY_VERSION;
                  parseState = 18;
                } else {
                  // Direct query — reply SYNCHRONOUSLY and immediately, like mXD.
                  sendVersionResponse();
                  parseState = 0;
                }
                break;
              case 'F': resetEepromDefaults();                            parseState = 0; break;
              case 'e':
                clearFlapMap();
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
        if (numLen > 0) gotoRawStep(bufToLong(numBuf, numLen));
        parseState = 0; break;

      // ── State 9: 'w' write flap map entry ─────────────────────────────
      case 9:
        if (c == ':')        { tempIndex = (int)bufToLong(numBuf, numLen); bufClear(numBuf, numLen); }
        else if (isDigit(c)) { bufAppend(numBuf, numLen, NUMBUF_MAX, c); }
        else {
          if (numLen > 0 && tempIndex >= 0 && tempIndex < flapCount) {
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

      // ── State 13: parameterless provisioning-by-SN (H/D/A/T/Q/M/F) ────
      // Collect the serial number, then — if it matches THIS module — run the
      // action selected by provCmd.  Replies (D/A/T/Q/M) are sent synchronously
      // from their handlers; only the matching module ever answers.
      case 13:
        if (c != '\n' && c != '\r') { bufAppend(snBuf, snLen, SNBUF_MAX, c); break; }
        if (serialMatches(snBuf)) {
          switch (provCmd) {
            case 'H': homeModule(); saveState();        break;
            case 'D': dumpEeprom();                     break;
            case 'A': dumpAll();                        break;
            case 'T': hallSelfTest();                   break;
            case 'Q': reportDiagnostics();              break;
            case 'M': mechanicalTest(MECH_TEST_REVS);   break;
            case 'F': resetEepromDefaults();            break;
          }
        }
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
            txBegin();
            rs485.print("mXack:");
            rs485.print(serialStr);
            txNumField(moduleId);
            rs485.print("\n");
            txEnd();
          }
          bufClear(snBuf, snLen); snColonSeen = false; bufClear(numBuf, numLen); parseState = 0;
        }
        break;

      // ── State 23: optional revolution count for direct 'M' ────────────
      // Reached after a direct (non-broadcast) 'm<id>M'.  Collects optional
      // digits: bare 'm<id>M' → default count; 'm<id>M<n>' → n (clamped inside
      // mechanicalTest to [MECH_TEST_REVS_MIN, MECH_TEST_REVS_MAX]).
      case 23:
        if (isDigit(c)) { bufAppend(numBuf, numLen, NUMBUF_MAX, c); break; }
        // Terminator (\n, \r, or anything else): run with the parsed count, or
        // the default if no digits were supplied.
        mechanicalTest(numLen > 0 ? (int)bufToLong(numBuf, numLen) : MECH_TEST_REVS);
        bufClear(numBuf, numLen);
        parseState = 0;
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

      // ── State 18: optional range for a broadcast version/all query ─────
      // Reached after "m*v" or "m*A".  Accepts an optional "<lo>-<hi>" range:
      //   m*v\n / m*A\n   → whole bus   (replyRangeLo=0,  replyRangeHi=254)
      //   m*v0-49\n       → IDs 0–49 only
      //   m*A50-99\n      → IDs 50–99 only
      //
      // broadcastReplyType (set on entry) selects whether matching modules
      // reply with a version frame or a combined all-fields dump.
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
          schedulePendingReply(broadcastReplyType, true);
          bufClear(numBuf, numLen); snColonSeen = false; parseState = 0;
        }
        break;

      // ── State 24: 'N' configure flap count + char set ─────────────────
      // Form: m<id>N<count>:<chars>   (count and chars both optional)
      //   - before the FIRST ':' → digits accumulate as the flap count
      //   - after  the FIRST ':' → every byte (incl. further ':') is a flap
      //     char, taken verbatim until the terminator
      // Applied on '\n'/'\r' (or the 50 ms idle timeout below).  No reply.
      case 24:
        if (!snColonSeen) {
          if (c == ':')             { snColonSeen = true; }          // → chars
          else if (isDigit(c))      { bufAppend(numBuf, numLen, NUMBUF_MAX, c); }
          else if (c == '\n' || c == '\r') {
            applyFlapConfig();
            bufClear(numBuf, numLen); restoreLen = 0; restoreBuf[0] = '\0';
            snColonSeen = false; parseState = 0;
          }
          // any other char before ':' is ignored (malformed count)
        } else {
          if (c == '\n' || c == '\r') {
            applyFlapConfig();
            bufClear(numBuf, numLen); restoreLen = 0; restoreBuf[0] = '\0';
            snColonSeen = false; parseState = 0;
          } else if (restoreLen < MAX_FLAPS) {
            restoreBuf[restoreLen++] = c;     // accumulate the char set verbatim
            restoreBuf[restoreLen]   = '\0';
          }
          // chars beyond MAX_FLAPS are dropped (buffer full)
        }
        break;

      // ── State 25: 'N' configure flap set by serial number ─────────────
      // Form: mXN<serialNumber>:<count>:<chars>   (count and chars optional)
      // tempIndex tracks the field: 0 = serial number, 1 = count, 2 = chars.
      // Only the module whose serial matches applies the change.  Like the
      // direct 'N', the chars field is taken verbatim and may contain ':'.
      case 25:
        if (tempIndex == 0) {                       // serial number
          if (c == ':')                        tempIndex = 1;
          else if (c != '\n' && c != '\r')     bufAppend(snBuf, snLen, SNBUF_MAX, c);
          else { bufClear(snBuf, snLen); parseState = 0; }   // no colon → abort
        } else if (tempIndex == 1) {                // count
          if (c == ':')                        tempIndex = 2;
          else if (isDigit(c))                 bufAppend(numBuf, numLen, NUMBUF_MAX, c);
          else if (c == '\n' || c == '\r') {
            if (serialMatches(snBuf)) applyFlapConfig();
            bufClear(snBuf, snLen); bufClear(numBuf, numLen); parseState = 0;
          }
        } else {                                    // chars (verbatim)
          if (c == '\n' || c == '\r') {
            if (serialMatches(snBuf)) applyFlapConfig();
            bufClear(snBuf, snLen); bufClear(numBuf, numLen);
            restoreLen = 0; restoreBuf[0] = '\0'; parseState = 0;
          } else if (restoreLen < MAX_FLAPS) {
            restoreBuf[restoreLen++] = c;
            restoreBuf[restoreLen]   = '\0';
          }
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
        case 8:  gotoRawStep(bufToLong(numBuf, numLen)); break;
        case 9:  if (tempIndex >= 0 && tempIndex < flapCount) {
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

  // ── Timeout: run a direct 'M' that arrived without a terminator (50 ms) ───
  // State 23 collects an optional revolution count after 'm<id>M'.  If the line
  // idles with no terminator, run the test now with the parsed count (or the
  // default if none was given).
  if (parseState == 23 && (millis() - lastSerialTime > 50)) {
    mechanicalTest(numLen > 0 ? (int)bufToLong(numBuf, numLen) : MECH_TEST_REVS);
    bufClear(numBuf, numLen);
    parseState = 0;
  }

  // ── Timeout: apply an 'N' flap-config that arrived without a terminator ───
  // State 24 collects m<id>N<count>:<chars>.  Within a frame, bytes arrive ~1 ms
  // apart, so a 50 ms idle means the frame truly ended; apply whatever parsed.
  if (parseState == 24 && (millis() - lastSerialTime > 50)) {
    applyFlapConfig();
    bufClear(numBuf, numLen); restoreLen = 0; restoreBuf[0] = '\0';
    snColonSeen = false; parseState = 0;
  }

  // ── Timeout: flush incomplete provisioning frames (200 ms idle) ───────────
  // States 13 (H/D/A/T/Q/M/F), 14 (I), 17 (W) and 25 (N) by serial number.
  if ((parseState == 13 || parseState == 14 || parseState == 17 ||
       parseState == 25)
      && (millis() - lastSerialTime > 200)) {
    bufClear(snBuf, snLen); snColonSeen = false;
    restoreLen = 0; restoreBuf[0] = '\0'; bufClear(numBuf, numLen); parseState = 0;
  }

  // ── Timeout: finalise a broadcast version/all range query (50 ms idle) ────
  // If the line had no explicit terminator, schedule the reply with whatever
  // range was parsed so far (defaults already set when entering state 18).
  if (parseState == 18 && (millis() - lastSerialTime > 50)) {
    if (snColonSeen) {
      if (numLen > 0) replyRangeHi = (int)bufToLong(numBuf, numLen);
    } else if (numLen > 0) {
      replyRangeLo = (int)bufToLong(numBuf, numLen);
      replyRangeHi = replyRangeLo;
    }
    schedulePendingReply(broadcastReplyType, true);
    bufClear(numBuf, numLen); snColonSeen = false; parseState = 0;
  }
}  // end loop()
