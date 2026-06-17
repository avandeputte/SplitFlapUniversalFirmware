# Architecture

This document describes how the Split-Flap Universal Firmware is structured and why it is built the way it is. It is aimed at someone modifying the firmware or writing controller-side software against it. For the command reference and wiring, see `README.md`.

## System context

A split-flap display is an array of character cells. Each cell is one self-contained module:

- an **ATtiny1616** microcontroller running this firmware (3.3 V logic rail),
- a **28BYJ-48** stepper motor turning the flap reel, driven through a **ULN2003** driver from a **separate 12 V motor supply** (the motor is *not* powered from the processor rail),
- a **Hall-effect sensor** that detects a magnet on the reel to establish the home position,
- a half-duplex **RS-485** transceiver, with the driver-enable (DE) line on a GPIO.

All modules share one RS-485 bus. A controller (e.g. a Raspberry Pi or ESP) is the bus master; it sends commands and the addressed module(s) reply. Modules never initiate traffic except for unprovisioned-module advertisements (see *Provisioning*).

The firmware's job is to accept addressed commands over the bus, move the reel to the requested flap, maintain its calibration in EEPROM, and report hardware health on request.

## Design principles

The firmware is shaped by the constraints of the target and the deployment:

- **Tiny RAM.** The ATtiny1616 has ~2 KB of SRAM. The firmware avoids the Arduino `String` class and all heap allocation; every buffer is fixed-size and statically allocated. Where two large buffers are never used at the same time, they share one backing store (see *Memory strategy*).
- **No reverse motion.** The reel is mechanically one-way. Any "move backward" is actually a forward move of nearly a full revolution to reach the equivalent position. This shapes homing, nudging, and the calibration math.
- **Shared half-duplex bus.** Only one device may drive the bus at a time. Replies to broadcasts must be staggered in time so they do not collide (see *Bus and reply timing*).
- **Backward compatibility is mandatory.** The message format and EEPROM layout have been stable since v6. New features are added as new command letters or as append-only extensions to existing replies, never by changing the position or meaning of existing fields.
- **Fail safe, not silent.** A hardware watchdog resets a hung module so it re-homes itself rather than staying dead. Calibration and measurement loops are all bounded so a stuck sensor can never spin the motor forever.

## Code layout

The firmware is a single Arduino sketch. Reading top to bottom, it is organised as:

1. **Header documentation** — message format, full command reference, EEPROM map, and the version changelog, all in comments.
2. **`enum PendingReply`** — declared immediately after the includes. Its position matters: Arduino auto-generates function prototypes, and a function taking this enum as a parameter must see the type first. (This has been an easy thing to break.)
3. **User configuration block** — pin assignments, mechanical constants, timing, and the watchdog timeout, fenced by clear `BEGIN`/`END` banners. This is the only region intended to be edited per-installation.
4. **Globals and fixed-size buffers** — calibration values, current position, the parser accumulators, and the deferred-reply state.
5. **Helper functions** — serial-number reading, supply-voltage measurement, EEPROM read/write/verify, ID formatting.
6. **EEPROM persistence** — load on boot, save individual fields, dump and restore the whole configuration.
7. **Provisioning** — advertisement, version response, factory reset, restore-from-payload.
8. **Motor functions** — `applyStep`, `stepBackward`, `releaseMotor`, `hallActive`, `nudge`, `homeModule`, `calibrateModule`.
9. **Diagnostics** — `hallSelfTest` (`T`), `reportDiagnostics` (`Q`), `mechanicalTest` (`M`).
10. **Movement** — `moveToIndex`, `moveToChar`.
11. **`setup()` and `loop()`** — boot sequence and the main parser/dispatch loop.

## Boot sequence

On reset, `setup()`:

1. Captures the reset cause from `RSTCTRL.RSTFR` (so `Q` can later report *why* the module last reset) and increments a boot counter in EEPROM.
2. Reads the ATtiny serial number from `SIGROW` and seeds the RNG from it.
3. Loads configuration from EEPROM. The first byte is a magic value: `0x5D` is current; `0x5E` is a v8/v9-era layout that is loaded and then rewritten to `0x5D`; anything else is treated as a blank chip and initialised to defaults with module ID 255 (unprovisioned). Loaded calibration values are sanity-clamped.
4. Waits a **staggered startup delay** so that a display full of modules powering on together does not all lurch at once (inrush). Provisioned modules use a delay derived from their ID; unprovisioned modules use a random delay seeded from their serial number.
5. Enables the **2-second watchdog**.
6. **Homes** (spin to the Hall sensor, then advance the stored offset to flap 0) or, if auto-home is disabled, restores the last saved step position from EEPROM.

## Main loop and the command parser

`loop()` is a byte-at-a-time state machine. Bytes arrive from the RS-485 port and drive `parseState` through a sequence of states. There is no blocking read; a partial command that stops arriving is flushed after a 50 ms idle timeout.

A message is `m <ADDR> <CMD> [data] \n`. Address parsing is a deliberately simple accumulator: digits accumulate into an ID buffer, `*` sets a broadcast-wildcard flag, `X` sets a provisioning flag, and the first character that is none of those is taken as the command letter. This single mechanism accepts every supported address form (`m38h`, `m5h`, `m**h`, `m*h`, `mXH…`) without special cases.

The command letter selects a handler. Simple commands act immediately; commands that take a numeric or string payload transition into a dedicated collecting state (for example, the optional rotation count on `m<id>M<n>` is gathered in its own state and run on the terminator or idle timeout). Long, motor-driven, or multi-line commands (`c`, `d`, `A`, `T`, `Q`, `M`) are **direct-addressed only** — they ignore the broadcast wildcard because they cannot be safely staggered across many modules.

### Why a hand-rolled state machine

The parser uses fixed accumulator buffers and explicit states rather than reading whole lines into a string. This keeps RAM flat and bounded regardless of input, tolerates partial/garbled frames on a noisy bus (the idle timeout recovers the parser), and lets a command begin acting the instant its terminator arrives.

## Addressing and the message format

Addresses may be two-digit zero-padded (`m05`, v6 style), variable-length decimal (`m138`, v7+), a broadcast wildcard (`m*` / `m**`), a provisioning address (`mX`, all modules regardless of ID), or a ranged broadcast for queries (`m*v0-49`). The `m`-prefix, command-letter vocabulary, and reply shapes are unchanged from v6; everything added since is either a new command letter or an append-only field, so older controllers keep working.

## Bus and reply timing

Because the bus is shared half-duplex, a module asserts DE (drive enable), transmits its reply as one tight burst, then releases DE. For a **direct** command this is immediate. For a **broadcast** query, every addressed module would otherwise answer simultaneously and collide, so replies are **staggered**: each module computes a reply time of

```
now + REPLY_LEADIN_MS + (moduleId − rangeLo) × slotWidth
```

and waits for its slot before transmitting. The slot width depends on the reply size — short version replies (`v`) use a 100 ms slot, large combined dumps (`A`) use a 700 ms slot. The optional ID range on a broadcast query (`m*v0-49`) lets a controller poll a large bus in batches and re-issue only the ranges that came back incomplete, which is far more robust at scale than one all-or-nothing sweep. A runtime variable records which reply type a staggered broadcast should produce, so the shared scheduling state can serve either kind.

The deferred reply is **non-blocking**: the module records *when* it should answer and continues running its loop, rather than busy-waiting, so it stays responsive.

## Persistence and the EEPROM layout

Calibration and identity live in EEPROM so they survive power cycles and (with the correct fuse) reflashing.

| Addr | Size | Field |
|------|------|-------|
| 0x00 | 1 | Magic byte (layout version marker) |
| 0x01 | 2 | Home offset (steps from Hall trigger to flap 0) |
| 0x03 | 2 | Total steps per revolution |
| 0x05 | 1 | Module ID (255 = unprovisioned) |
| 0x06 | 1 | Auto-home flag |
| 0x07 | 2 | Saved step position (used when auto-home is off) |
| 0x09 | 1 | Saved flap index |
| 0x0A | 1 | Boot counter (diagnostics) |
| 0x0B | 1 | EEPROM health-check scratch byte |
| 0x0C | 128 | Flap map: 64 × `uint16_t` calibrated positions, `0xFFFF` = uncalibrated |

This layout has been stable since v6. The boot-counter and scratch bytes at `0x0A`/`0x0B` were reserved padding in the original layout and were put to use without moving any existing field. Only the magic byte has varied between firmware generations, and all known values are migrated automatically on first boot.

EEPROM writes go through an `update`-style helper that reads first and only writes changed bytes, to spare the limited write-endurance of the cells. A small write-read-verify check (using the scratch byte) backs the `eepromOk` field in the `Q` diagnostics.

> **Reflashing:** preserving EEPROM across a UPDI flash requires the `EESAVE` fuse to be set ("EEPROM retained" + Burn Bootloader once per chip). Without it, each flash erases calibration and identity, and the module reverts to unprovisioned with default calibration.

## Memory strategy

SRAM is the scarcest resource. Two deliberate choices keep it under control:

- **No `String`, no heap.** All parsing and reply assembly use fixed `char` buffers sized to their worst case.
- **One shared work buffer.** The incoming restore payload (`mXW`) and the outgoing dump assembly (`d` / `A`) are both large but never needed at the same instant, so they share a single statically-allocated buffer. This reclaimed roughly 600 bytes and cleared the IDE's low-memory warning.

The per-call stack cost of the diagnostics is bounded too — for example, the mechanical test's per-rotation array is sized to the maximum requestable rotation count and lives only on the stack during the test.

## Motion model

All movement is forward-only, in half-steps, through `stepBackward(n)` (the name is historical; it advances the reel). Each step pulses the watchdog, advances the half-step phase, and tracks the absolute step position, wrapping at one revolution. A Hall transition seen mid-move re-synchronises the absolute position to the known home location.

- **Homing** spins until the Hall sensor trips, then advances the stored home offset to land on flap 0. If the sensor never trips within a bounded search, homing fails honestly and marks the position unknown (forcing a re-home on the next move) rather than pretending to be homed.
- **Moving to a flap** computes the forward distance from the current position to the target (consulting the calibrated flap map where present) and steps it, homing first if the current position is unknown.
- **Nudging** fine-tunes the home offset; a negative nudge is realised as a forward move of nearly a full revolution to the equivalent position, with the stored offset wrapped accordingly.
- **Calibration** measures the steps in one revolution by counting between Hall edges, sanity-checks the result, and stores it.

Every wait-for-sensor loop in the motion and calibration code is bounded by a safety counter, so a disconnected or stuck sensor can never cause an unbounded spin — the loop exits and the higher-level operation reports failure.

## Diagnostics subsystem

Three commands let a controller assess module health over the bus without physical inspection. They are designed to be run in order, because each depends on the previous being sound:

1. **`Q` — snapshot (no movement).** Reports reset cause, boot count, supply voltage, EEPROM health, and current index. Fast and safe to poll. Check this first: an unexpected reset cause or a climbing boot count points to a module resetting in the field.
2. **`T` — Hall sensor self-test.** Steps one revolution and classifies the sensor: healthy, stuck active, dead/disconnected, multiple regions (noise), or inverted polarity. The mechanical test observes motion *through* this sensor, so the sensor must be proven healthy before the mechanical result can be trusted.
3. **`M` — mechanical self-test.** Drives the motor across several revolutions and reports per-rotation step counts plus summary statistics (min, max, spread) and gate observations. The spread and the raw per-rotation trend distinguish a one-off glitch from a progressive fault; the gate fields make a "no motion" result diagnosable (sensor never fired vs. reel under-rotated vs. parked on the magnet). The number of rotations is requestable (`m<id>M<n>`) for better statistics on rare intermittent faults.

A central design point: the mechanical test's "did it move?" check drives a **full** revolution and requires the sensor to see the magnet both enter and leave. An earlier quarter-revolution check produced false "no motion" results whenever the reel happened to start with the magnet far from the sensor — the motor turned correctly but the sensor never changed state within the short window. Reasoning about *what the sensor can observe*, not just *what the motor was told to do*, is the key to that part of the code.

## Extending the firmware

When adding features, preserve the two compatibility contracts:

- **Protocol:** add a new command letter, or append new fields to the *end* of an existing reply. Never move or repurpose an existing field's position — controllers parse by index. If a reply's last field is variable-length (like the flap map), think carefully before appending after it, because "the rest of the line" parsers will be affected.
- **EEPROM:** add new fields after the existing layout (past the flap map), and treat an unwritten value (`0xFF` / `0xFFFF`) as "unknown" so modules flashed from older firmware degrade gracefully.

Keep new wait-for-hardware loops bounded, pulse the watchdog inside any long motor move, and remember that the `PendingReply` enum must stay directly after the includes for the Arduino auto-prototyper. Verify changes compile and that braces balance before flashing.
