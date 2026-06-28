# Split-Flap Display Firmware

Firmware for a multi-module split-flap display where each character cell is an independent ATtiny1616 microcontroller connected to a shared RS-485 bus. A Raspberry Pi (or any serial host) sends commands over the bus to drive individual cells or the whole display at once.

---

## Table of Contents

- [Repository Contents](#repository-contents)
- [Flashing the Firmware](#flashing-the-firmware)
- [How It Works](#how-it-works)
  - [Module Identity and Provisioning](#module-identity-and-provisioning)
  - [Boot Sequence](#boot-sequence)
  - [EEPROM Layout](#eeprom-layout)
- [RS-485 Message Protocol](#rs-485-message-protocol)
  - [Message Format](#message-format)
  - [Addressing](#addressing)
- [Command Reference](#command-reference)
  - [Display Commands](#display-commands)
  - [Calibration Commands](#calibration-commands)
  - [Configuration Commands](#configuration-commands)
  - [Diagnostic Commands](#diagnostic-commands)
  - [Provisioning Commands](#provisioning-commands)
  - [De-provisioning Command](#de-provisioning-command)
- [Flap Character Set](#flap-character-set)
- [Provisioning Script (`provision.py`)](#provisioning-script-provisionpy)
  - [Requirements](#requirements)
  - [Running the Script](#running-the-script)
  - [Provisioning a New Module](#provisioning-a-new-module)
  - [De-provisioning Modules](#de-provisioning-modules)
- [Calibration Workflow](#calibration-workflow)
- [Diagnostics Workflow](#diagnostics-workflow)
- [Upgrading Firmware](#upgrading-firmware)

---

## Repository Contents

```
src/SplitFlapUniversalFirmware.cpp  — ATtiny1616 firmware (built with PlatformIO)
platformio.ini                      — PlatformIO build/upload configuration
.vscode/                            — VSCode + PlatformIO workspace settings
provision.py                        — Raspberry Pi provisioning and management tool
README.md                           — This file (command reference + protocol)
SETUP.md                            — Step-by-step VSCode + PlatformIO install/build/upload guide
ARCHITECTURE.md                     — How the firmware is structured and why
RELEASE_NOTES.md                    — Per-version change history
```

> **This is now a VSCode / PlatformIO project — there is no longer an Arduino `.ino` sketch.** The firmware lives in `src/SplitFlapUniversalFirmware.cpp` and is built/uploaded with PlatformIO. (PlatformIO's `.ino`→`.cpp` preprocessor mis-parses the single quotes in this firmware's comments, so the project uses a plain `.cpp` with an explicit `#include <Arduino.h>` and forward declarations.) The legacy Arduino IDE workflow is no longer supported.

---

## Flashing the Firmware

The firmware targets the ATtiny1616 via the [megaTinyCore](https://github.com/SpenceKonde/megaTinyCore) platform, built and uploaded with **[PlatformIO](https://platformio.org/)** inside **VSCode** (or via the `pio` CLI). Programming is over **SerialUPDI** (a cheap USB-serial adapter), not a dedicated programmer.

**New here? Follow the full walkthrough in [SETUP.md](SETUP.md)** — it covers installing VSCode, installing PlatformIO, getting the code, and building/uploading from scratch.

Quick version, once your toolchain is set up:

1. Open this project folder in VSCode (PlatformIO auto-installs the ATtiny toolchain on first build).
2. Wire the SerialUPDI adapter to the module's UPDI pin and **power the board separately** (the adapter does not power it). Set `upload_port` in [`platformio.ini`](platformio.ini) to your adapter.
3. **Once per chip**, write the fuses so EEPROM is retained across reflashes:
   ```bash
   pio run -t fuses
   ```
   This sets the `EESAVE` fuse (and clock/BOD/UPDI fuses) — the PlatformIO equivalent of the Arduino IDE's *Burn Bootloader*. Without it, every flash wipes the module's ID and calibration (see [Upgrading Firmware](#upgrading-firmware)).
4. Build and upload:
   ```bash
   pio run -t upload
   ```
   In VSCode you can instead click the PlatformIO **Build** (✓) and **Upload** (→) buttons in the status bar.

Every module runs the same firmware binary. IDs and the flap set are assigned at runtime via the bus — there is nothing to change before flashing.

---

## How It Works

### Module Identity and Provisioning

Each ATtiny1616 contains a factory-programmed 10-byte unique serial number in its signature row (SIGROW, bytes `0x1103–0x110C`). The firmware reads this at boot and formats it as a 20-character uppercase hex string (e.g. `A3F24C0018E7D29B3F01`).

On first boot, or after a de-provisioning reset, a module has no assigned bus ID (`moduleId = 255`). In this state it:

1. Does **not** respond to numeric ID commands.
2. Broadcasts its serial number on the RS-485 bus approximately every **10–15 seconds** so the provisioning tool can discover it. The interval is randomised per-module to avoid collisions when many unprovisioned modules are powered on simultaneously.

Once a numeric bus ID (0–254) is assigned via the provisioning tool, the module stores it in EEPROM and begins responding to that ID on all future commands. It stops advertising.

### Boot Sequence

```
Power on
  │
  ├─ Capture reset cause (RSTFR) and increment the boot counter
  ├─ Read ATtiny serial number from SIGROW
  ├─ Seed random number generator from serial number
  ├─ Load config from EEPROM
  │     Magic = 0x5D (v6/current) → load all fields
  │     Magic = 0x5E (v8/v9 era)  → load all fields, rewrite magic to 0x5D
  │     Anything else (blank chip) → write defaults, module ID = 255
  │
  ├─ Staggered startup delay (capped so high IDs are not unresponsive for long)
  │     Provisioned   → delay = (moduleId mod 32) × 120 ms  (≤ ~3.7 s)
  │     Unprovisioned → delay = random 0–4 s                (spread from serial number)
  │     (prevents inrush current when many modules power on together)
  │
  ├─ Enable the 2 s watchdog timer
  └─ Home or restore position
        autoHome = 1 → spin to Hall sensor, advance to flap 0
        autoHome = 0 → restore last saved step position from EEPROM
```

A 2-second hardware watchdog runs continuously after boot; if any operation hangs (e.g. a stuck Hall sensor mid-move), the module resets and re-homes itself rather than staying dead until power-cycled.

### EEPROM Layout

The EEPROM field layout has been compatible since v6. Addresses `0x0A` and `0x0B` were reserved padding in the original layout and were put to use for diagnostics in v26 without disturbing any existing field or the flap map. The Hall-polarity byte (`0x8C`, v30) and the configurable flap-set fields (`0x8D`–`0xCD`, v31) live in the previously-unused tail, so **no v6-era field has ever moved**. Only the magic byte has varied across firmware versions; all known variants are recognised and migrated automatically on upgrade, and any field a module has never written reads back as `0xFF`/`0xFFFF` and falls back to its default.

| Address | Size | Contents |
|---|---|---|
| 0x00 | 1 byte | Magic byte (`0x5D`) — confirms EEPROM has been initialised |
| 0x01 | 2 bytes | Home offset — steps from Hall trigger edge to flap 0 |
| 0x03 | 2 bytes | Total steps per revolution |
| 0x05 | 1 byte | Module bus ID (255 = unprovisioned) |
| 0x06 | 1 byte | Auto-home flag (1 = home on boot, 0 = restore saved position) |
| 0x07 | 2 bytes | Saved step position (used when auto-home is off) |
| 0x09 | 1 byte | Saved flap index (used when auto-home is off) |
| 0x0A | 1 byte | Boot counter (diagnostics; wraps at 255) |
| 0x0B | 1 byte | EEPROM health-check scratch byte (diagnostics) |
| 0x0C | 128 bytes | Calibrated step positions: 64 × `uint16_t`, `0xFFFF` = uncalibrated |
| 0x8C | 1 byte | Hall sensor active level (v30): `0` = active-low, `1` = active-high, `0xFF` = not yet auto-detected |
| 0x8D | 1 byte | **Flap count** (v31): active number of flaps, `1`–`64`; `0xFF`/out-of-range → default `64` |
| 0x8E | 64 bytes | **Flap character set** (v31): one byte per flap; `0xFF` in the first byte → use the built-in default set |

**Magic byte history** — all values are recognised and migrated to `0x5D` on first boot after upgrade:

| Value | Written by | Action on load |
|---|---|---|
| `0x5D` | v6 and later (current) | Load all fields — already current |
| `0x5E` | v8, v9 (erroneous bump) | Load all fields, rewrite magic to `0x5D` |
| Other | Blank chip | Write all defaults, leave ID as 255 |

---

## RS-485 Message Protocol

### Message Format

```
m  <ADDR>  <CMD>  [data]  \n
│     │      │      │
│     │      │      └── Command payload (digits, one character, or hex string)
│     │      └───────── Command letter (see Command Reference)
│     └──────────────── Module address (see Addressing below)
└────────────────────── Literal 'm' — start-of-message marker
```

Every message begins with a lowercase `m` and ends with a newline (`\n`). Messages without an explicit terminator are flushed after a 50 ms idle timeout.

### Addressing

The firmware accepts all address formats that have ever been used across all versions:

| Format | Example | Meaning |
|---|---|---|
| Two-digit zero-padded decimal | `m38`, `m05` | Single module — v6 style |
| Variable-length decimal | `m5`, `m138` | Single module — v7+ style |
| One or two stars | `m*`, `m**` | Broadcast to all provisioned modules |
| Star with ID range | `m*v0-49` | Broadcast to a sub-range of IDs (see `v` / `A` below) |
| Literal `X` | `mX` | Provisioning address — all modules respond regardless of ID |

Broadcast replies are **staggered**: each responding module waits for a time slot indexed by its ID before transmitting, so replies arrive as a clean sequence rather than colliding. The slot width depends on the reply size (see the `v` and `A` commands). Long, motor-driven commands and multi-line replies (`d`, `A`, `c`, `T`, `Q`, `M`) are **direct-addressed only** — they cannot be staggered across a broadcast and ignore the `*` wildcard.

---

## Command Reference

In all examples `<ID>` is the numeric module ID (e.g. `38`), `*` broadcasts to all provisioned modules, and `X` addresses all modules regardless of provisioning state.

### Display Commands

#### `-` — Show character

Move the reel to display a specific character from the flap set.

```
m<ID>-<char>\n
```

| Part | Description |
|---|---|
| `<char>` | Exactly one byte matching a character in the [flap character set](#flap-character-set) |

If the requested character is **not** in the module's flap set, the module simply **homes** the reel (since v31) rather than ignoring the command — a predictable, visible response to an unknown character.

**Examples:**
```
m38-B\n       → module 38 shows 'B'
m5-7\n        → module 5 shows the character '7'
m*- \n        → all modules show blank (space = flap 0)
m38-Z\n       → if 'Z' is not on this reel, module 38 homes instead
```

---

#### `+` — Show flap by index

Move the reel to a specific flap by its zero-based numeric index (`0` to flap-count − 1, i.e. 0–63 by default).

```
m<ID>+<index>\n
```

| Part | Description |
|---|---|
| `<index>` | Integer from `0` to the configured flap count − 1 (0–63 by default). An index ≥ the flap count is ignored. |

**Examples:**
```
m38+0\n       → module 38 shows flap 0 (blank)
m38+7\n       → module 38 shows flap 7 ('F')
m38+12\n      → module 38 shows flap 12 ('K')
```

---

### Calibration Commands

#### `h` — Home

Spin the reel until the Hall sensor fires, then advance the calibrated offset to land on flap 0 (blank). Resets the internal step counter to zero.

```
m<ID>h\n
m*h\n         (broadcast)
```

---

#### `c` — Calibrate revolution

Measures the exact number of half-steps in one full revolution by timing two passes of the Hall sensor. Saves the result to EEPROM, reports it over RS-485, then homes automatically.

**Response format:**
```
m<ID>:<measuredSteps>\n
```

**Example:**
```
m38c\n        → module measures revolution, replies m38:4096\n
```

---

#### `o` — Set home offset

Sets the number of half-steps to travel past the Hall sensor trigger before the reel reaches flap 0. Saved to EEPROM immediately.

```
m<ID>o<steps>\n
```

**Example:**
```
m38o2832\n
```

---

#### `s` — Nudge

Advances the reel by `n` additional half-steps from its current position and adds that distance to the stored home offset. Use this for fine-tuning the home position without re-running a full calibration. The value may be **negative** (e.g. `m38s-16`); since the reel is mechanically one-way, a negative nudge spins forward by nearly a full revolution to reach the equivalent position, and the stored offset is adjusted (and wrapped) accordingly.

```
m<ID>s<steps>\n
```

**Example:**
```
m38s16\n      → nudge 16 steps forward, update home offset
```

---

#### `t` — Set total steps

Overrides the total half-steps per revolution. Normally set automatically by `c`, but can be set manually. Saved to EEPROM immediately.

```
m<ID>t<steps>\n
```

**Example:**
```
m38t4096\n
```

---

#### `g` — Go to raw step position

Moves the reel to an absolute step position (0 to `totalStepsPerRev − 1`). Sets the current flap index to unknown. Useful for diagnostics and manual fine-calibration.

```
m<ID>g<position>\n
```

**Example:**
```
m38g512\n
```

---

#### `w` — Write calibrated flap position

Stores a fine-calibrated step position for a specific flap index in the EEPROM map. When an entry is present, `moveToIndex` uses it instead of the evenly-spaced estimate.

```
m<ID>w<index>:<stepPosition>\n
```

**Example:**
```
m38w7:342\n   → flap index 7 is at step 342
```

---

#### `e` — Erase flap position map

Clears all 64 fine-calibrated flap positions from EEPROM (resets to `0xFFFF`). The module falls back to evenly-spaced estimates until re-calibrated.

```
m<ID>e\n
m*e\n         (broadcast)
```

---

### Configuration Commands

#### `i` — Set module ID

Assigns a new numeric bus ID and saves it to EEPROM. Takes effect immediately.

```
m<ID>i<newId>\n
```

**Example:**
```
m38i42\n      → reassign module 38 to ID 42
```

> For first-time assignment on an unprovisioned module, use the `mXI` provisioning command instead.

---

#### `a` — Set auto-home mode

Controls whether the module homes the reel on every boot.

```
m<ID>a1\n     → home on boot (default)
m<ID>a0\n     → restore last saved position on boot
```

When auto-home is disabled the current step position and flap index are saved to EEPROM whenever the reel moves, allowing the module to resume without homing after a power cycle.

---

#### `N` — Configure the flap set

Sets the **number of physical flaps** and/or the **character set** on the reel, and persists them to EEPROM. Both parts are **optional and independent**, so you can set either one alone or both together. Both default to `64` and the built-in [flap character set](#flap-character-set).

```
m<ID>N<count>:<chars>\n
```

| Part | Description |
|---|---|
| `<count>` | Physical flap count, `1`–`64`. Drives the valid flap-index range and the step-position maths (`index × totalSteps / count`), so it must match the real reel. Out-of-range values are ignored. |
| `<chars>` | The ordered character at each flap index (used by the `-` command). Up to 64 bytes, taken **verbatim** to end-of-line — it **may itself contain `:`**, so only the *first* colon separates `<count>` from `<chars>`. |

Either side may be omitted:

```
m38N48\n                          → set flap count to 48 only (chars unchanged)
m38N: ABCDEFGHIJ\n                → set the character set only (count unchanged)
m38N10: 0123456789\n             → set both: 10 flaps showing digits 0–9
m*N64: ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789!@#$&()-+=;q:%'.,/?*roygbpw\n
                                  → reset every module on the bus to the default 64-flap set at once
```

There is **no reply**. The command works **direct-addressed**, as a **broadcast** (`m*N…`, to configure a whole panel in one shot), and **by serial number** (`mXN<sn>:<count>:<chars>` — see [Provisioning Commands](#provisioning-commands)). Read the current values back with the [`A` dump](#a--combined-all-fields-dump), which now reports them.

> **Independence note:** count and character-set length are stored separately and a mismatch is tolerated. If the count exceeds the number of characters, the extra indices simply have no character to display via `-`; if the character set is longer than the count, the surplus characters are unreachable. For a normal reel, set them to the same value.

> A factory reset (`F`) returns both the flap count and the character set to their defaults.

---

### Diagnostic Commands

#### `d` — Dump EEPROM

Returns the module's full calibration configuration in one RS-485 message. **Direct-addressed only** (broadcast `m*d` is ignored — a full dump can be ~600 bytes / ~0.6 s, too long to stagger).

**Response format:**
```
m<ID>d:<homeOffset>:<totalSteps>:<idx>=<pos>,<idx>=<pos>,...\n
```

Only calibrated flap map entries (those not equal to `0xFFFF`) are included.

**Example response:**
```
m38d:2832:4096:0=0,7=342,12=683\n
```

---

#### `A` — Combined "all fields" dump

Returns **everything from `v` and `d` in a single message**, so a client can fetch a module's complete state in one command instead of two. The `v` and `d` commands are unchanged and remain available.

**Response format:**
```
m<ID>A:<version>:<moduleId>:<serialNumber>:<homeOffset>:<totalSteps>:<autoHome>:<curIndex>:<idx>=<pos>,...:<flapCount>:<flapChars>\n
```

**Example response:**
```
m38A:31:38:A3F24C0018E7D29B3F01:2832:4096:1:0:0=0,7=342:64: ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789!@#$&()-+=;q:%'.,/?*roygbpw\n
```

> **`<flapCount>` and `<flapChars>` were appended in v31.** They come *after* the variable-length flap map, so every field up to and including the map is byte-for-byte identical to the pre-v31 format and **older controllers keep working unchanged** (they just ignore the tail). `<flapChars>` is the **final** field and may itself contain `:`, `,` or `=`, so read it verbatim to end-of-line — do not split it further. This is exactly the tail that the [`mXW` restore](#serial-number-variants-of-other-commands) accepts, so an `A` dump round-trips back into a module.

Direct-addressed `m<ID>A` and serial-number `mXA<sn>` forms reply synchronously. The broadcast form `m*A` **is** supported and staggered, with an optional ID range — but because each `A` frame is long (~570 ms), a full sweep is slow; prefer ranged batches on large buses:

```
m*A\n         → all provisioned modules answer (wide ~700 ms slots)
m*A0-49\n     → only IDs 0–49 answer
m*A50-99\n    → only IDs 50–99 answer
```

---

#### `v` — Report firmware version

Returns the firmware version, module ID, and ATtiny1616 serial number. Useful for verifying which version is running after an upgrade and for cross-referencing a provisioned ID back to a physical serial number.

**Response format:**
```
m<ID>v:<version>:<moduleId>:<serialNumber>\n
```

**Examples:**
```
m38v\n        → m38v:31:38:A3F24C0018E7D29B3F01\n
m*v\n         → every provisioned module replies in sequence, each in its own slot
m*v0-49\n     → only IDs 0–49 reply (poll the bus in retryable batches)
```

> On a broadcast `m*v`, each module delays its reply by a fixed lead-in plus `(moduleId − rangeLo) × 100 ms` before asserting the bus, so replies arrive as a clean, collision-free sequence. The optional `<lo>-<hi>` range lets a controller poll a large bus in batches and re-issue only the ranges that come back incomplete — far more reliable at scale than one all-or-nothing sweep. A plain `m*v` is equivalent to `m*v0-254`.

---

#### `F` — Factory reset EEPROM

Resets all calibration and configuration values to their firmware defaults while **preserving the module ID** (so the module stays on the bus without re-provisioning) and the EEPROM magic byte.

```
m<ID>F\n
m*F\n         (broadcast)
```

**Values reset to defaults:**

| Field | Default |
|---|---|
| Home offset | 2832 steps |
| Total steps per revolution | 4096 steps |
| Auto-home flag | 1 (enabled) |
| Saved step position | 0 |
| Saved flap index | 0 |
| Flap position map | All `0xFFFF` (uncalibrated) |

**Values preserved:**

| Field | Reason |
|---|---|
| Module ID | Module stays addressable on the bus |
| EEPROM magic byte | Next boot loads correctly without re-initialising |

> After an `F` reset the module will need to be re-calibrated. Run `c` (calibrate revolution) followed by `s` (nudge) to re-establish the home position.

---

#### `T` — Hall sensor self-test

Steps the reel through one revolution and reports the health of the Hall home sensor. **Direct-addressed only.** Useful for diagnosing a sensor that is disconnected, wired with reversed polarity, or picking up noise.

**Response format:**
```
m<ID>T:<code>:<edges>:<activeSamples>:<fallingEdges>\n
```

| `code` | Meaning | Likely cause |
|---|---|---|
| 0 | OK | One clean home pulse per revolution |
| 1 | Stuck active | Shorted sensor, magnet jammed at the sensor, or mis-wiring |
| 2 | Dead / disconnected | No magnet detection at all — dead or unplugged sensor |
| 3 | Multiple regions | Stray magnet, electrical noise, or a chattering sensor |
| 4 | Inverted polarity | Sensor reads active everywhere with one brief dip — wired backwards |

`<edges>` (rising edges) is the number of home pulses seen per revolution (a healthy sensor shows 1); `<activeSamples>` is how many sampled steps read active (advisory detail). `<fallingEdges>` (appended in v28) counts active→inactive transitions; a clean sensor shows `edges == fallingEdges == 1`. A mismatch indicates the active region straddles the revolution boundary or the sensor releases raggedly. An older parser ignores this trailing field.

---

#### `Q` — Diagnostics snapshot

Returns an instantaneous module health snapshot. **No motor movement**, so it is fast and safe to poll. **Direct-addressed only.**

**Response format:**
```
m<ID>Q:<resetCause>:<bootCount>:<vcc_mV>:<eepromOk>:<curIndex>\n
```

| Field | Meaning |
|---|---|
| `resetCause` | Raw reset-flag bits from the last reset: `0x01` power-on, `0x02` brown-out, `0x04` external, `0x08` watchdog, `0x10` software |
| `bootCount` | Boots since the counter was last cleared (wraps at 255) — a climbing value indicates a module silently resetting in the field |
| `vcc_mV` | Measured supply voltage in millivolts |
| `eepromOk` | `1` if the EEPROM write-read-verify check passed, else `0` |
| `curIndex` | Current flap index (`-1` = position unknown / needs homing) |

**Example response:**
```
m38Q:0:2:4980:1:0\n
```

> Watch for a non-zero watchdog (`0x08`) or brown-out (`0x02`) reset bit, a supply voltage sagging below your design target under load, `eepromOk = 0`, or a `bootCount` that keeps climbing between polls.

---

#### `M` — Mechanical self-test

Drives the motor and measures steps-per-revolution across several rotations to detect missed steps and a non-moving motor. **Direct-addressed only.** Because it physically spins the reel for several revolutions, it takes ~20 seconds at the default count.

Accepts an **optional revolution count**: bare `m<id>M` samples the default number of rotations, while `m<id>M<n>` requests `n` rotations for better statistics on rare intermittent faults. `n` is clamped to a safe range (5–20 by default); values below the minimum or above the maximum are pulled into range. More rotations catch rarer faults but take proportionally longer (~4 s each). The serial-number form (`mXM<sn>`) does not take a count and always uses the default.

```
m38M\n        → mechanical test, default rotation count
m38M20\n      → mechanical test, 20 rotations (more thorough)
```

**Response format:**
```
m<ID>M:<code>:<min>:<max>:<spreadTenthsPct>:<gateActive>:<gateSpan>:<avgMagnetWidth>:<r1>,<r2>,...,<rN>\n
```

| `code` | Meaning | Likely cause |
|---|---|---|
| 0 | OK | All revolutions consistent |
| 1 | Inconsistent | Spread > 5% — intermittent missed steps (drag, weak supply, failing driver) |
| 2 | No motion | Motor not turning (open coil, dead driver, jam) **or** a dead Hall sensor |

Every field has the same meaning regardless of result code (the parser never has to branch on `code`):

| Field | Meaning |
|---|---|
| `min` / `max` | Smallest / largest steps-per-revolution measured (both `0` on a no-motion result) |
| `spreadTenthsPct` | `(max − min) / average` in tenths of a percent (e.g. `23` = 2.3%); `0` on a no-motion result. A healthy module shows a near-zero spread. |
| `gateActive` | Hall active-samples seen while driving the ~1.1-revolution motion-detect gate |
| `gateSpan` | Total steps driven during that gate |
| `avgMagnetWidth` | Average width (in steps) of the magnet's active region across the measured revolutions |
| `r1,…,rN` | Raw steps-per-revolution for each of `MECH_TEST_REVS` rotations, comma-separated (empty on a no-motion result) |

> **Compatibility:** `avgMagnetWidth` and the raw rotation list were appended in v28. They come *after* `gateSpan`, so a parser written for the earlier format keeps working unchanged — it simply ignores the trailing fields. A parser that wants the new data should split the rotation list on commas rather than assume a fixed count (it follows `MECH_TEST_REVS`).

**Why the raw rotation list matters:** `min`/`max`/`spread` summarise *how much* the revolutions varied but not *how*. The raw sequence reveals the shape:

- `4096, 4095, 4097, 4096, 3700` — one outlier among healthy revs → an **intermittent** glitch (a single snag), motor basically fine.
- `4096, 4030, 3960, 3890, 3820` — a steady decline → a **progressive** fault (heating/torque loss, loosening coupler), a failure in progress.

Both produce a similar `spread`, but they are very different diagnoses — only the per-rotation trend tells them apart. `avgMagnetWidth` adds an independent signal: if it drifts from the `T` test's `activeSamples`, or the reel passes on step count while the width wanders, the **sensor-to-magnet gap is changing** as the reel turns (shaft wobble, a loosening magnet).

The `gateActive` / `gateSpan` fields make a **code 2** result directly diagnosable without further probing:

- `gateActive` ≈ one magnet width (e.g. ~168) → the sensor worked but the reel **did not complete a sensed revolution** — motor slip / under-rotation.
- `gateActive` ≈ 0 → the sensor **never went active** — dead/disconnected sensor, or the magnet never reached it.
- `gateActive` ≈ `gateSpan` → the sensor **stayed active the whole time** — parked on the magnet or stuck active.

> Because `M` observes motion *through* the Hall sensor, a working motor with a dead sensor also reports code 2. **Run `T` first** to confirm the sensor is healthy, then interpret an `M` result accordingly.

---

### Provisioning Commands

These commands use the `X` address field and are processed by **every module on the bus** regardless of provisioning state. Each module performs its own serial-number check internally.

---

#### `mXH` — Home by serial number

Homes a specific unprovisioned module identified by its SIGROW serial number. Use this to physically locate a module before assigning it an ID.

```
mXH<serialNumber>\n
```

| Part | Description |
|---|---|
| `<serialNumber>` | 20-character uppercase hex string from the ATtiny SIGROW |

**Example:**
```
mXHA3F24C0018E7D29B3F01\n
```

Only the matching module moves. All others silently discard the command.

---

#### `mXI` — Assign ID by serial number

Assigns a permanent bus ID to a module identified by its serial number. The module writes the new ID to EEPROM, stops advertising, and sends an acknowledgement.

**Command format:**
```
mXI<serialNumber>:<newId>\n
```

**Acknowledgement format:**
```
mXack:<serialNumber>:<assignedId>\n
```

**Example:**
```
mXIA3F24C0018E7D29B3F01:38\n
→  mXackA3F24C0018E7D29B3F01:38\n
```

---

#### Serial-number variants of other commands

Several commands have an `mX<letter><serialNumber>` form that targets a single module by its serial number rather than its bus ID. Only the module whose serial matches responds; all others discard the command. These are useful before a module has been assigned an ID, or to address a specific module when its ID is unknown.

| Command | Action | Reply |
|---|---|---|
| `mXD<sn>` | Dump EEPROM config by serial number | Same as `d` |
| `mXA<sn>` | Combined all-fields dump by serial number | Same as `A` |
| `mXF<sn>` | Factory-reset EEPROM by serial number (preserves module ID) | None |
| `mXN<sn>:<count>:<chars>` | Configure the flap set by serial number (both parts optional; same semantics as the `N` command) | None |
| `mXW<sn>:<homeOffset>:<totalSteps>:<idx>=<pos>,...[:<flapCount>:<flapChars>]` | Restore a previously dumped EEPROM image (preserves module ID; map entries absent from the payload are cleared; the optional flap-set tail is restored if present) | None |
| `mXT<sn>` | Hall sensor self-test by serial number | Same as `T` |
| `mXQ<sn>` | Diagnostics snapshot by serial number | Same as `Q` |
| `mXM<sn>` | Mechanical self-test by serial number | Same as `M` |

**Backup-and-restore example** — capture a module's complete state and push it back (e.g. after replacing a board). Using `A` (rather than `d`) captures the flap set as well, and the `mXW` restore accepts that same tail:

```
mXAA3F24C0018E7D29B3F01\n
→  m38A:31:38:A3F24C0018E7D29B3F01:2832:4096:1:0:0=0,7=342:64: ABC…w\n

mXWA3F24C0018E7D29B3F01:2832:4096:0=0,7=342:64: ABC…w\n
```

The `mXW` payload mirrors the calibration portion of the `d`/`A` dumps (home offset, total steps, flap map), optionally followed by the `:<flapCount>:<flapChars>` tail that `A` emits. If the tail is omitted, the flap set is left unchanged — so a `d`/`mXD` dump (which has no tail) still restores correctly.

---

### De-provisioning Command

#### `R` — Reset provisioning

Erases the stored bus ID from EEPROM and returns the module to the unprovisioned state. All calibration data is preserved. The module immediately begins advertising its serial number again.

```
m<ID>R\n      → de-provision one module by its current ID
m*R\n         → broadcast: de-provision all modules on the bus
```

**Examples:**
```
m38R\n        → de-provision module 38
m*R\n         → de-provision every module simultaneously
```

> After a broadcast reset each module will re-advertise within 10–15 seconds. The provisioning tool automatically opens a provisioning dialog for each one as it appears.

---

## Flap Character Set

By default the reel holds 64 physical flaps. Position 0 is always blank (the home position). The index in the string below corresponds to a physical flap on the reel.

```
" ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789!@#$&()-+=;q:%'.,/?*roygbpw"
```

| Index range | Characters |
|---|---|
| 0 | ` ` (blank / home) |
| 1–26 | `A` – `Z` |
| 27–36 | `0` – `9` |
| 37–46 | `! @ # $ & ( ) - + =` |
| 47–56 | `; q : % ' . , / ? *` |
| 57–63 | `r o y g b p w` (colour flaps) |

The lowercase letters at the end (`r`, `o`, `y`, `g`, `b`, `p`, `w`) represent physical colour flaps. Address them by character with the `-` command — e.g. `m38-r\n` shows the red flap.

> **This set is the compile-time default only.** Since v31 the flap count and character set are **configurable per module at runtime** via the [`N` command](#n--configure-the-flap-set) (and persisted in EEPROM), so a reel with a different number of flaps or a different character ordering is fully supported without recompiling. The string above is the fallback used by any module that has never been configured. The maximum is 64 flaps. The current set of any module can be read back from the [`A` dump](#a--combined-all-fields-dump).

---

## Provisioning Script (`provision.py`)

`provision.py` is a Raspberry Pi terminal tool that automates the discovery and management of modules on the RS-485 bus.

### Requirements

- Python 3.9 or later
- [pyserial](https://pypi.org/project/pyserial/)

```bash
pip install pyserial
```

### Running the Script

```bash
python3 provision.py [--port PORT] [--baud BAUD]
```

| Argument | Default | Description |
|---|---|---|
| `--port` | `/dev/ttyUSB0` | Serial port for the USB-to-RS-485 adapter |
| `--baud` | `9600` | Baud rate — must match the firmware |

**Example:**
```bash
python3 provision.py --port /dev/ttyACM0
```

Once running the script listens continuously for module advertisements. While idle it accepts two hotkeys:

| Key | Action |
|---|---|
| `d` | Open the de-provision menu |
| `q` | Quit |

No Enter key is required — the terminal is in raw mode while idle.

---

### Provisioning a New Module

When a module advertises for the first time the script opens an interactive dialog:

```
────────────────────────────────────────────────────────────
  New module detected
  Serial : A3F24C0018E7D29B3F01
────────────────────────────────────────────────────────────
  Home this module now to identify it? [y/N]
```

**Step 1 — Identify (optional)**

Entering `y` sends `mXH<serialNumber>` to the bus. That module's reel spins to the blank/home position so you can physically see which tile it is. All other modules ignore the command.

**Step 2 — Assign an ID**

```
  Enter bus ID to assign (0–254), or 's' to skip:
```

- Enter a number between 0 and 254 to assign that ID.
- Enter `s` to skip — the module will keep advertising and the dialog will reopen on the next advertisement.
- The script warns if you enter an ID already used in the current session.

**Step 3 — Confirm and acknowledge**

After confirmation the script sends `mXI<serialNumber>:<id>` and waits up to 2 seconds for the module's `mXack` response. A success or timeout warning is displayed, then the script returns to listening mode.

---

### De-provisioning Modules

Press `d` at any time to open the de-provision menu:

```
────────────────────────────────────────────────────────────
  De-provision modules
────────────────────────────────────────────────────────────
  IDs provisioned this session:
     38  →  A3F24C0018E7D29B3F01
     42  →  B10055FFA3C2918D7E44

  Options:
    Enter a module ID (0–254) to de-provision that module
    Enter 'all' to de-provision every module on the bus
    Enter 's' to go back
```

**De-provision one module**

Enter a numeric ID. If that ID was provisioned in the current session its serial number is shown for confirmation. IDs from previous sessions not in the list can also be entered — the `R` command is sent regardless.

After confirmation the script sends `m<id>R\n`, removes the module from the session registry, and clears its serial from the seen list so its provisioning dialog will reopen when it next advertises.

**De-provision all modules**

Enter `all`. A warning prompt appears before the script broadcasts `m*R\n`. The entire session registry is cleared. Every module will erase its ID, begin advertising, and get a fresh provisioning dialog as each advertisement arrives.

---

## Calibration Workflow

Follow these steps when setting up a module for the first time or after a mechanical change.

1. **Flash** the firmware and power up the module.

2. **Provision** the module using `provision.py` to assign it a bus ID (e.g. `38`).

3. **Calibrate the revolution** — measures the exact steps per revolution and saves it to EEPROM:
   ```
   m38c\n
   ```
   The module reports the measured step count and homes automatically.

4. **Check the home position** — if flap 0 (blank) is not perfectly centred in the window after homing, nudge it:
   ```
   m38s8\n     → advance 8 more steps and update the home offset
   ```
   Repeat with small values until the blank flap is centred.

5. **(Optional) Configure the flap set** — only if this reel differs from the default 64-flap layout. Set the count and/or character set (see the [`N` command](#n--configure-the-flap-set)):
   ```
   m38N10: 0123456789\n     → a 10-flap reel showing the digits 0–9
   ```
   Skip this step for standard 64-flap reels.

6. **Test character display:**
   ```
   m38-A\n
   m38-0\n
   m38- \n     → back to blank
   ```

7. **(Optional) Fine-calibrate individual flaps** — if specific characters land slightly off, drive to the exact step position manually and write it to the map:
   ```
   m38g340\n         → manually drive to step 340
   m38w7:340\n       → save step 340 as the calibrated position for flap index 7
   ```

8. **Verify the full map:**
   ```
   m38d\n            → dump EEPROM config including all saved flap positions
   ```

9. **Confirm firmware version and check hardware health:**
   ```
   m38v\n            → m38v:31:38:A3F24C0018E7D29B3F01\n
   m38T\n            → Hall sensor self-test (expect code 0)
   m38M\n            → mechanical self-test (expect code 0, near-zero spread)
   m38Q\n            → diagnostics snapshot (check supply voltage, reset cause)
   ```

   Alternatively, `m38A\n` returns the version, ID, serial number, full calibration, **and the configured flap set** in a single message.

---

## Diagnostics Workflow

When a module misbehaves, work through the self-tests in this order. Each isolates a different subsystem, and the order matters because later tests depend on earlier ones being healthy.

1. **`m<id>Q`** — instantaneous snapshot, no movement. Check first:
   - Is the **supply voltage** at your design target? A sagging rail (e.g. 3.3 V on a 5 V system) under-powers the stepper and causes missed steps or stalls — fix this before chasing motor/sensor faults.
   - Is the **reset cause** a watchdog (`0x08`) or brown-out (`0x02`)? Is **bootCount** climbing? Either indicates the module is resetting unexpectedly.
   - Did the **EEPROM check** pass (`eepromOk = 1`)?

2. **`m<id>T`** — Hall sensor self-test. The sensor must be healthy before the mechanical test can be trusted, because `M` observes motion through it.
   - Code 0 → sensor good, proceed.
   - Code 2 → dead/disconnected; code 4 → wired with reversed polarity; code 3 → noise or a stray magnet.

3. **`m<id>M`** — mechanical self-test, only meaningful once `T` reports code 0.
   - Code 0 with near-zero spread → motor healthy.
   - Code 1 → intermittent missed steps (check supply under load, mechanical drag, the driver).
   - Code 2 (with a healthy `T`) → motor not turning: open coil, dead ULN2003 channel, or a physical jam.

> A wild `M` result (a huge spread, or a "revolution" measured as far fewer steps than expected) usually means the **Hall sensor is reporting false detections** rather than the motor failing — confirm with `T` before replacing a motor or driver. On a **code 2** specifically, read the `gateActive` field: a value near one magnet width (~150–200) means the reel under-rotated (motor slip) despite a working sensor; near 0 means the sensor never fired; near `gateSpan` means it was parked on the magnet.

---

## Upgrading Firmware

All firmware versions from v6 onward use the same EEPROM field layout. Flashing a newer firmware version onto an existing module will not erase calibration data or the module ID — **provided the `EESAVE` fuse is set** so that the chip erase performed before each UPDI flash preserves EEPROM.

With PlatformIO: run `pio run -t fuses` **once per chip** (the equivalent of the Arduino IDE's *Burn Bootloader*); this sets `EESAVE` via `board_hardware.eesave = yes` in [`platformio.ini`](platformio.ini). After that, every `pio run -t upload` preserves the module's ID, calibration, flap map, auto-detected Hall polarity, and configured flap set. If this fuse is *not* set, each flash wipes EEPROM and the module reverts to unprovisioned with default calibration. See [SETUP.md](SETUP.md) for the full toolchain walkthrough.

| Previous version | Magic byte on chip | Behaviour on first boot of new firmware |
|---|---|---|
| v6 or later | `0x5D` | Loaded as-is — no migration needed |
| v8 or v9 | `0x5E` | Fields loaded, magic rewritten to `0x5D` |
| Blank chip | `0xFF` | Full default init, ID set to 255 |

> **Upgrading to v31 from an earlier version:** the new flap-count and flap-character-set EEPROM fields (`0x8D`/`0x8E`) read back as `0xFF` on a module that predates them, so the module transparently falls back to the default 64-flap set until you configure it with the [`N` command](#n--configure-the-flap-set). No action is required for reels that use the standard 64-flap character set.

> If a module ends up in an unexpected state after flashing, send `m<id>v\n` to confirm the firmware version and `m<id>d\n` (or `m<id>A\n` for everything at once, including the flap set) to inspect its EEPROM. Use `m<id>F\n` to reset calibration to defaults while keeping the ID, or `m<id>R\n` to fully de-provision the module. As a recovery path, keep periodic `mXA` dumps of each module so calibration **and the flap set** can be restored with `mXW` if a chip is ever wiped or replaced.
