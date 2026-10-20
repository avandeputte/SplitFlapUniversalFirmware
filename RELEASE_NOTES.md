# Release Notes

Version history for the Split-Flap Universal Firmware. The firmware version is reported by the `v` and `A` commands (e.g. `m38v` → `m38v:35:38:<serial>`).

All versions from v6 onward share the same EEPROM field layout, and every change to a reply has been **append-only** — new fields are added at the end so older controllers keep working. Upgrading preserves a module's ID and calibration **as long as the `EESAVE` fuse is set** (see [SETUP.md](SETUP.md) § "Write the fuses").

---

## v35 — Firmware updates over the bus

v35 adds a bootloader, so from now on new firmware can reach a whole wall over the RS-485 bus instead of module by module over UPDI. Getting there takes **one last flash over UPDI per module**, which also sets a new fuse. The EEPROM layout is unchanged, and so is every reply except `Q`, which gains one field at the end and now reports the real reset cause.

**Action required, once per module**
- **Flash v35 over UPDI.** Its image now starts with the bootloader, which needs the fuse `BOOTEND = 0x04` to reserve its 1 KB:
  - The [web flasher](https://avandeputte.github.io/SplitFlapUniversalFirmware/) sets the fuse from the image by itself.
  - With PlatformIO, run `pio run -t fuses` once, then upload.
  - With avrdude, [FLASHING.md](FLASHING.md) has the commands (`fuse8` is now `0x04`).
- **The fuse must match the image.** A v35 image with `BOOTEND = 0x00`, or an older one with `0x04`, does not run. The web flasher picks the right value for whichever image it writes, so going back to an older release is safe with it too.

**Updates over the bus**
- **The bootloader** ([bootloader/](bootloader/)) takes the first 1 KB of flash. At every start it checks the firmware against its CRC (about 150 ms) and runs it.
- **A new command, `U`**, restarts a module into update mode, and `m*U` does the whole wall. It is accepted only in a checksummed frame, so line noise can never trigger it.
- **The procedure.** The gateway broadcasts the image page by page, about 40 s for any number of modules. It then asks each module what it is missing, resends just that, and tells the modules to go.
- **[BUS-UPDATE.md](BUS-UPDATE.md)** has the protocol and the procedure. [`tools/busflash.py`](tools/busflash.py) is a reference sender, for a computer with a USB–RS485 adapter.
- **Every release now includes `firmware-update.bin`**, the part of the image sent over the bus.
- **Safety:**
  - A module never runs a partial image. An update that stops part way, power cut included, leaves it waiting in update mode for the image.
  - Every page is read back after it is written.
  - The final check needs every page plus a CRC match.
  - The bootloader never writes EEPROM, so IDs and calibration are safe.
  - It never touches its own section.
  - It answers only when asked by its own ID.
- **Not yet tried on a real bus.** On modules, the bootloader's start-up check works, and so does its fall-back to update mode when the image is broken. The update itself has so far run only on a host simulation of the chip, so try the first one on a single module. A module that does not finish an update is either back on its old firmware or waiting in update mode, and a UPDI flash always recovers it.

**Diagnostics**
- **`Q` appends `:<blVersion>`**, the bootloader's version, or 0 if the module has none. Older controllers ignore it.
- **`Q` reports the real reset cause.** It has read `0` since it was added: megaTinyCore's startup clears the reset flags before the firmware's `setup()` runs. The firmware now reads the copy the core leaves for it.

**Web flasher**
- **It now gets into a module whose firmware keeps restarting the chip.** It sends the programming key while it holds the chip in reset. Before, a firmware that restarted the chip every few milliseconds could use the key up before the flasher confirmed it, and the flash failed.

**Build**
- **Layout.** The firmware is linked at `0x0400`, behind the bootloader, and may use up to 15,358 bytes.
  - `firmware.hex` is the whole image: bootloader, firmware, and the image CRC in the last two bytes.
  - A post-build step (`bootloader/merge.py`) puts it together, so `pio run -t upload` and avrdude write all of it.
- **`strtol` and `atol` are gone.** A 14-line parser gives the same results on all of 2 million test strings, edge cases included, and saves about 650 bytes. Flash use is 14,671 of 15,358 bytes (95.5%).
- **Checked against v33** on a host simulation of the firmware: 8,000 randomised traffic runs, plain and checksummed, give identical replies with identical timing, apart from `Q`'s new field.
- **The bootloader's own tests** cover every boot decision, faulty and hostile bus traffic, and power lost at every flash write of an update.

---

## v34 — The watchdog really is 2 seconds

A one-line fix, and very likely the end of modules that never answer a dump. No fuse change and no EEPROM-layout change, and every reply is unchanged: just upload.

**The bug**
- **The firmware asked for a 2 s watchdog with avr-libc's `WDTO_2S`.** On this chip, `wdt_enable()` puts that constant straight into the watchdog's period field, where 7 means 512 cycles. The watchdog has really been running at **0.512 s**.
- **One thing took longer than that without resetting it: a reply.** A reply is sent in one go after a single reset, at about 1 ms a byte. **Any reply longer than about 490 bytes reset the module part way through.**
- **In practice, the dumps of a well-calibrated module.** The `A` and `d` dumps of a module with many calibrated flap positions run past that: a fully calibrated module's `A` dump is about 630 bytes. Such a module:
  - answers every short command, but never a dump;
  - counts one more boot (`bootCount` in `Q`) for every attempt;
  - "recovers" when its calibration is erased, because that makes the dump short again.

  That is the gateway's soak-test report: two modules on a 64-module wall that never answered a dump.

**The fix**
- **The chip's own constant.** The watchdog now uses `WDT_PERIOD_2KCLK_gc`: 2.048 s, as the code and the docs always said. The longest possible reply, about 0.73 s, is well inside it.
- **Checked on a module:** the watchdog register now reads `0x09` (2K cycles), where v33 left `0x07` (512 cycles).
- **A hang is still caught**, in 2 s instead of 0.5 s.

---

## v33 — Bus robustness from gateway soak tests, optional checksums, and no more printf

No fuse change and no EEPROM-layout change: just upload. Every reply to a plain message is byte-for-byte unchanged except `Q`, which gains four fields at the end, and messages may now carry an optional checksum. These changes answer the requests the gateway project filed after soak-testing a 64-module wall.

**Checksums (optional)**
- **Any message can carry a checksum:** `~HHHH<message>\n`. `HHHH` is the CRC-16/CCITT-FALSE of the message from its `m` to its terminator, as 4 hex digits; in Python, `binascii.crc_hqx(message, 0xFFFF)`.
- **A module checks it before acting on the message.** A frame that fails is dropped and counted in `Q` (`crcErrors`).
- **A frame that passes runs exactly as the plain message would**, and every reply it causes carries a checksum in the same form, staggered broadcast replies included.
- **Messages without a checksum are handled and answered exactly as before.**
- **Older firmware skips the checksum** and runs the message unchecked, so checksums work on mixed walls, broadcasts included.
- See README → *Checksums* for the details, including why a checksummed message should always be terminated.
- **All replies are now assembled before transmitting.** The bytes are unchanged; this is what lets the checksum go in front.

**Receive path**
- **A clean frame after an overflow is no longer eaten.** When a busy module's receive ring overflows, the firmware still abandons the frame that was cut and skips the spliced tail. The skip now ends after 5 ms of bus silence instead of 200 ms. In v32 a command sent within 200 ms of the module finishing its backlog was discarded, and its own terminator was what cleared the discard; it is now answered whenever it starts 5 ms or more after the junk. (An idle module never overflows: it reads bytes as fast as the bus delivers them, however long a neighbour's reply.)
- **Commands without a terminator behave exactly like terminated ones.** The 50 ms idle timeout now feeds the parser a `\n` instead of running a separate copy of each command's handling. As a side effect, commands that arrived while such a command was running are no longer lost to the reply of an earlier broadcast.
- **`+` with no index is ignored**, like every other numeric command (it used to move to flap 0).

**Broadcast replies**
- **Carrier sense before a staggered reply.** When its slot comes round, a module answering `m*v` or `m*A` waits until the bus has been quiet for 20 ms, so a slot that would land on another frame defers instead of destroying both. On a healthy bus every slot already sees that much quiet, so timing is unchanged. The unprovisioned-module advertisement uses the same check and no longer busy-waits 20 ms.
- **Timing documented.** The slot clock starts at the broadcast's terminator. While the bus is busy each module's `millis()` runs about 15% slow, so later slots land later than nominal: they are spread apart, never pushed together, and a controller should wait somewhat longer for the last ones. See README → `v`.

**Diagnostics**
- **`Q` appends `:<rxOverflows>:<bcastSent>:<bcastLate>:<crcErrors>`**, all counted since boot:
  - receive-ring overflows;
  - staggered broadcast replies transmitted;
  - how many of those had to wait for the bus;
  - checksummed frames for this module that failed their check.

  Older controllers ignore the extra fields.

**Dumps and calibration data**
- **Out-of-range flap positions are refused** by `w` and by the `mXW` restore (which leaves such entries uncalibrated). A negative position in a restore payload used to be stored as e.g. `65368`. The module never used such a value for a move, so nothing on the display changes.
- **A corrupt flap-character cell holding a line terminator is sent as `?`** in the `A` dump instead of ending the frame early.
- **A dump can never be truncated**, and the build now proves it: the longest possible `A` line is ~700 bytes against a 720-byte buffer, checked at compile time.
- **Every transmission starts with a watchdog reset.** The longest reply, a full `A` dump, takes ~0.75 s, well inside the 2 s watchdog. (It wasn't: the watchdog really ran at 0.5 s until v34 fixed it.)

**Build**
- **printf is gone.** The dumps were the only formatted output; two small helpers now produce them byte for byte as `snprintf` did, so the ~1 KB printf library and the `-Wl,-u,vfprintf -lprintf_min` flags are out of `platformio.ini`. The build no longer depends on which printf variant a toolchain would pick.
- Flash use is down from 99.1% to 92.7% (16,244 → 15,181 bytes), checksums included; RAM +11 bytes.

---

## v32 — Brown-out protection and receive-path hardening

No wire-format or EEPROM-layout change. This release addresses modules that "forgot" their settings after a power cycle, and several ways a busy module could commit a truncated frame to EEPROM.

**Fuse change — action required on every module**
- The **brown-out detector is now enabled at 2.6 V** (`board_hardware.bod = 2.6v`, fuse `0x54`). It was previously disabled, which at 10 MHz (datasheet minimum 2.7 V) let the CPU execute during the part of every power-up and power-down ramp where flash and EEPROM reads are unreliable — the textbook cause of AVR EEPROM corruption. **Run `pio run -t fuses` once on each module** (before or after the upload; it does not erase EEPROM). A brown-out reset now shows as `0x02` in the `Q` reset cause. Without PlatformIO, [FLASHING.md](FLASHING.md) has the avrdude commands for the fuses and the attached `firmware.hex`.

**Receive path**
- **RX overflow guard.** SoftwareSerial drops bytes once its ring is full while the firmware is inside a long move or EEPROM burst. The parser now detects that, finishes the intact frames already buffered, abandons the one frame that was cut, and skips its spliced tail — instead of writing a truncated character set or flap map to EEPROM, or acting on a display frame assembled from two different frames.
- **Receive buffer raised to 128 bytes** (`-D_SS_MAX_RX_BUFF=128`), so every `N` flap-set frame (up to 91 bytes) fits even on a busy module.
- **Catch-all 200 ms idle timeout** for every parser state, so a lone `m` from bus noise or a lost terminator can never leave the parser waiting to swallow the next frame.
- **Deferred broadcast replies and advertisements wait for an idle parser**, and the advertisement no longer discards unread receive bytes — an unprovisioned replacement board can no longer eat the middle of its own `mXW` restore.
- **`RESTORE_MAX` 600 → 640** (a full 64-entry map plus 64-char set is ~579 bytes; the old limit left 21 bytes of headroom and truncated silently).

**EEPROM**
- **Every EEPROM write pulses the watchdog.** A full restore or factory reset on a calibrated module is ~300 back-to-back writes, close enough to the 2 s watchdog that a reset mid-restore (map cleared, half rewritten) was a real risk.
- **Saved step position fixed.** It was written as a 4-byte `long` into the 2-byte field at `0x07`, spilling over the saved index and boot counter, and read back with those bytes included — so restore-on-boot (auto-home off) always clamped to 0.
- **Magic byte written last** when initialising a blank chip, so a power loss midway leaves "not initialised" rather than a valid magic over `0xFF` fields.
- **Calibration sanity clamp shared by boot and `mXW` restore**, so a garbled payload can never persist a zero-length revolution; a mapped flap position outside the revolution falls back to even spacing instead of driving up to 16 turns.
- **`Q` health check is now two write cycles, run on the first `Q` after boot and then at most hourly** with the result cached — a controller polling `Q` every minute would previously have worn the scratch cell out in weeks.

**Internal**
- The motion primitive is `advance()` (formerly `stepBackward`; the name was historical — the reel only ever moves forward) and the three revolution-measurement paths (first boot, `c`, `M`) share one `measureOneRev()`. These v31-era refactors had not been pushed to GitHub before; no behaviour change.

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
