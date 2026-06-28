# Release Notes

Version history for the Split-Flap Universal Firmware. The firmware version is reported by the `v` and `A` commands (e.g. `m38v` → `m38v:31:38:<serial>`).

All versions from v6 onward share the same EEPROM field layout, and every change to a reply has been **append-only** — new fields are added at the end so older controllers keep working. Upgrading preserves a module's ID and calibration **as long as the `EESAVE` fuse is set** (see [SETUP.md](SETUP.md) § "Write the fuses").

---

## v31 — Configurable flap set + project moved to PlatformIO

**Project / tooling**
- **This is now a VSCode / PlatformIO project — the Arduino `.ino` sketch is gone.** The firmware is `src/SplitFlapUniversalFirmware.cpp`, built and uploaded with PlatformIO over SerialUPDI. See the new [SETUP.md](SETUP.md) for a from-scratch install/build/upload walkthrough. (`.cpp` rather than `.ino` because PlatformIO's `.ino` preprocessor mis-parses the apostrophes in the firmware's comments.)
- New documentation: [SETUP.md](SETUP.md) (toolchain guide) and this `RELEASE_NOTES.md`; `README.md` and `ARCHITECTURE.md` updated throughout.

**New feature — runtime-configurable flap count and character set**
- New **`N` command** sets the number of physical flaps and/or the reel's character set, persisted to EEPROM:
  - `m<ID>N<count>:<chars>` — both parts optional and independent.
  - Works **direct**, as a **broadcast** (`m*N…`, to set a whole panel at once), and **by serial number** (`mXN<sn>:<count>:<chars>`).
  - Both default to the previous **64 flaps** and the built-in character set, so existing reels are unaffected.
- The **`A` dump now reports the configured count and characters** (`…:<flapCount>:<flapChars>`), appended after the flap map so older controllers ignore the addition.
- The **`mXW` restore accepts the same `:<flapCount>:<flapChars>` tail**, so an `A` dump round-trips back into a module (capture full state with `mXA`, restore with `mXW`).
- A `-<char>` request for a character **not** on the configured reel now **homes** the module instead of doing nothing — a visible, predictable response.

**Behaviour / compatibility**
- New EEPROM fields live in the previously-unused tail (`0x8D` flap count, `0x8E…` characters); no existing field moved. A module upgraded from pre-v31 reads `0xFF` there and falls back to the default 64-flap set until configured — **no action needed** for standard reels.
- Flap count drives the valid index range and the step-position maths; count and character-set length are stored independently and a mismatch is tolerated.

**Internal (no functional change)**
- The firmware fills ~99% of the ATtiny1616's 16 KB flash, so this feature was paid for by de-duplicating code: the RS-485 transmit framing (`txBegin`/`txEnd`/`sendLine`/`txReplyHeader`/`txNumField`), dump assembly (`appendMsgId`/`appendFlapMap`), flap-map erase (`clearFlapMap`), the `g` move (`gotoRawStep`), and the seven parameterless serial-number provisioning commands (`H`/`D`/`A`/`T`/`Q`/`M`/`F`, now one shared parse state).


- **Hall sensor polarity is auto-detected** so a module works with either sensor wiring or magnet orientation (no more hard-coded active-low assumption). Stored in EEPROM (`0x8C`).
- **Steps-per-revolution is auto-measured** on a fresh chip rather than assumed (a 28BYJ-48 is 4096 half-steps, but full-step and clone variants differ); only a sane measurement is committed.
- Both run **only** when EEPROM polarity is undetected (`0xFF`) — i.e. a first flash or a flash where EEPROM wasn't preserved — so a good calibration is never silently overwritten. A module upgraded from pre-v30 auto-measures once on its first v30 boot.
- New **`P` command** re-detects Hall polarity on demand. The home *offset* still requires the operator's dial-in.

---

## v29 — Mechanical self-test rotation count

- `m<ID>M<n>` runs the mechanical self-test for `n` rotations (clamped to 5–20) for better statistics on rare intermittent faults; bare `m<ID>M` still uses the default. The `mXM<sn>` serial form always uses the default. Backward compatible.

---

## v28 — Richer self-test telemetry (append-only)

- `M` appends the average magnet width and the raw steps-per-revolution for every sampled rotation. The per-rotation trend distinguishes a one-off glitch from a progressive drift; a wandering magnet width flags a changing sensor-to-magnet gap.
- `T` appends `fallingEdges` (active→inactive transitions); a clean sensor shows rising == falling == 1.

---

## v27 — Mechanical self-test gate fields

- `M` always reports the motion-detect gate observations as two fixed fields (`gateActive`, `gateSpan`), with the same meaning regardless of result code (the controller never has to branch). This makes a "no motion" result directly diagnosable: ~one magnet width = reel under-rotated (motor slip), ~0 = sensor never fired, ~`gateSpan` = parked on the magnet.

---

## v26 — Hardware self-diagnostics

- New **`T`** (Hall sensor self-test), **`Q`** (diagnostics snapshot: reset cause, boot count, supply voltage, EEPROM health, current index), and **`M`** (mechanical self-test) commands, each with a direct `m<ID>X` form and an `mX<sn>` serial-number form.
- Reset cause and a boot counter are captured at boot using the formerly-reserved EEPROM padding bytes (`0x0A`/`0x0B`) — no existing field moved.

---

## v25 — SRAM reclaim

- The incoming `mXW` restore payload and the outgoing `d`/`A` dump assembly share one work buffer (they never overlap in time), reclaiming ~600 bytes of SRAM.

---

## v24 — Consolidated baseline

- Combined all changes since the original v6:
  - Dynamic module IDs via the ATtiny SIGROW serial number, with unprovisioned-module advertisement and serial-number provisioning commands (`mXH`/`mXI`/`mXD`/`mXF`/`mXW`/`mXA`).
  - Variable-length address parser (v6 two-char zero-padded format still accepted); `+` index command; `v` version report; `F` factory reset; `R` de-provision; `d` single-line EEPROM dump; `A` combined all-fields dump.
  - Scalable broadcast queries (`m*v` / `m*A` with optional `<lo>-<hi>` range) for polling large buses in retryable batches.
  - Negative nudge (`s`) on the one-way reel.
  - 2-second watchdog through all long blocking sections; fixed `char[]` parser buffers (no heap/`String`); read-compare-write EEPROM updates to minimise wear.
- EEPROM compatibility: layout unchanged since v6; magic `0x5D` (v6) and `0x5E` (v8/v9) both migrate cleanly; blank chips initialise to unprovisioned (ID 255).

---

## v6 and earlier

The original protocol baseline: fixed two-character zero-padded addressing, the core display/calibration commands, and the EEPROM field layout that every later version preserves.
