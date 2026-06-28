# Setup Guide — Build & Flash the Firmware (VSCode + PlatformIO)

This is a step-by-step walkthrough for someone starting from nothing: install VSCode, install PlatformIO, get the code, and build and upload the firmware to an ATtiny1616 module.

> **This project is VSCode / PlatformIO only — there is no Arduino `.ino` sketch.** The firmware is `src/SplitFlapUniversalFirmware.cpp`, built per `platformio.ini`. The old Arduino IDE workflow is no longer supported.

For *what the firmware does* and the command protocol, see [README.md](README.md). For *how it is built internally*, see [ARCHITECTURE.md](ARCHITECTURE.md).

---

## Table of Contents

1. [What you need](#1-what-you-need)
2. [Install Visual Studio Code](#2-install-visual-studio-code)
3. [Install the PlatformIO extension](#3-install-the-platformio-extension)
4. [Get the firmware code](#4-get-the-firmware-code)
5. [Open the project in VSCode](#5-open-the-project-in-vscode)
6. [Wire the programmer](#6-wire-the-programmer)
7. [Tell PlatformIO which serial port to use](#7-tell-platformio-which-serial-port-to-use)
8. [Build (compile) the firmware](#8-build-compile-the-firmware)
9. [Write the fuses — once per chip](#9-write-the-fuses--once-per-chip)
10. [Upload (flash) the firmware](#10-upload-flash-the-firmware)
11. [Verify it's running](#11-verify-its-running)
12. [Everyday workflow](#everyday-workflow)
13. [Command-line (no VSCode) alternative](#command-line-no-vscode-alternative)
14. [Troubleshooting](#troubleshooting)

---

## 1. What you need

**Software**
- A computer running Windows, macOS, or Linux.
- Visual Studio Code (free).
- The PlatformIO IDE extension (free; installs the ATtiny compiler toolchain for you).
- Git (optional — only if you want to `clone` rather than download a ZIP).

**Hardware**
- An ATtiny1616-based split-flap module (or a bare ATtiny1616).
- A **SerialUPDI** programmer — a cheap USB-to-serial (UART) adapter (CP2102, CH340, FT232, etc.) wired for UPDI. A single resistor (typically 470 Ω – 1 kΩ) between the adapter's TX and RX forms the UPDI data line; that line connects to the chip's **UPDI** pin.
- A way to **power the module** during programming. **The SerialUPDI adapter does not power the board** — provide the module's normal supply (and common ground with the adapter).

> If you have never wired a SerialUPDI adapter, search "SerialUPDI megaTinyCore wiring" for the standard TX—(resistor)—RX → UPDI arrangement. megaTinyCore's documentation has a detailed page on it.

---

## 2. Install Visual Studio Code

1. Go to <https://code.visualstudio.com/> and download the installer for your OS.
2. Run the installer and accept the defaults.
   - **Windows:** during install, ticking "Add to PATH" is convenient but optional.
   - **macOS:** drag *Visual Studio Code* into *Applications*.
   - **Linux:** use the `.deb`/`.rpm` package or your distro's store.
3. Launch VSCode once to confirm it opens.

---

## 3. Install the PlatformIO extension

PlatformIO is what actually compiles and uploads the firmware; it also downloads the ATtiny toolchain automatically the first time you build.

1. In VSCode, open the **Extensions** view: click the square icon in the left sidebar, or press `Ctrl+Shift+X` (`Cmd+Shift+X` on macOS).
2. Search for **PlatformIO IDE**.
3. Click **Install** on the extension published by *PlatformIO*.
4. Wait for it to finish (it shows progress in the bottom status bar) and **reload/restart VSCode** if prompted. The first install can take a few minutes — it sets up its own Python environment.
5. When it's ready you'll see a small **ant/alien head icon** in the left sidebar and a row of PlatformIO buttons (a ✓ check, a → arrow, etc.) in the blue status bar along the bottom.

> This project's `.vscode/extensions.json` recommends the PlatformIO extension, so VSCode may also prompt you to install it automatically when you open the folder in the next step.

---

## 4. Get the firmware code

Pick **one** of the following.

**Option A — Download a ZIP (simplest)**
1. Download the project archive (or copy the project folder) onto your computer.
2. Unzip it somewhere memorable, e.g. `Documents/SplitFlapUniversalFirmware/31`.

**Option B — Clone with Git**
```bash
git clone <repository-url>
cd <repository>/31
```

Either way, the folder you want to open in VSCode is the one that **directly contains `platformio.ini`** (the `31` folder). Its contents:

```
platformio.ini                      ← PlatformIO sees this and knows it's a project
src/SplitFlapUniversalFirmware.cpp  ← the firmware
.vscode/                            ← workspace settings
README.md  ARCHITECTURE.md  SETUP.md  RELEASE_NOTES.md
provision.py                        ← Raspberry Pi provisioning tool (not needed to build)
```

---

## 5. Open the project in VSCode

1. **File → Open Folder…** and choose the folder that contains `platformio.ini` (the `31` folder). *Do not* open a parent folder that merely contains it several levels down — PlatformIO looks for `platformio.ini` at the root of the opened folder.
2. The first time you open it, PlatformIO recognises the project and may spend a minute or two installing the **atmelmegaavr** platform and the AVR toolchain. Watch the bottom status bar; let it finish.
3. If VSCode asks "Do you trust the authors of the files in this folder?", choose **Yes** so the build can run.

---

## 6. Wire the programmer

With the module **powered off**:

1. Connect the SerialUPDI adapter's **UPDI data line** (the TX/RX-with-resistor junction) to the module's **UPDI** pin.
2. Connect the adapter's **GND** to the module's **GND** (common ground is required).
3. Apply the module's **normal power supply**. The adapter does **not** power the board.
4. Plug the USB adapter into your computer.

> Double-check power before your first upload — "UPDI link initialization failed" is most often an unpowered target, not a wiring fault.

---

## 7. Tell PlatformIO which serial port to use

PlatformIO needs to know which serial port your USB adapter shows up as. Edit [`platformio.ini`](platformio.ini) and set `upload_port` (and, if you'll use the serial monitor, `monitor_port`):

```ini
upload_port  = /dev/cu.usbserial-XXXX    ; macOS example
; upload_port = COM5                      ; Windows example
; upload_port = /dev/ttyUSB0              ; Linux example
```

**Find your port name:**
- **Windows:** Device Manager → *Ports (COM & LPT)* → e.g. `COM5`.
- **macOS:** run `ls /dev/cu.usbserial-*` (or `ls /dev/cu.*`) in a terminal.
- **Linux:** run `ls /dev/ttyUSB*` or `ls /dev/ttyACM*` (you may need to add your user to the `dialout` group: `sudo usermod -aG dialout $USER`, then log out/in).

> If you omit `upload_port`, PlatformIO will try to auto-detect a single connected adapter, which often works when only one is plugged in.

The rest of `platformio.ini` is already configured for this board (ATtiny1616, 10 MHz internal clock, EEPROM-retain fuse, SerialUPDI at 57600 baud, minimal-`printf` size flags). You normally don't need to change anything else.

---

## 8. Build (compile) the firmware

You don't need hardware connected to *build* — this is a good first check that the toolchain works.

- **In VSCode:** click the **Build** button (the ✓ checkmark) in the bottom status bar, or press `Ctrl+Alt+B`. You can also open the PlatformIO sidebar → *Project Tasks → ATtiny1616 → General → Build*.
- A terminal panel opens and shows the compile. Success ends with something like:
  ```
  RAM:   [=====     ]  48.3% (used 990 bytes from 2048 bytes)
  Flash: [==========]  98.8% (used 16182 bytes from 16384 bytes)
  ========================= [SUCCESS] ... =========================
  ```

> **Heads-up:** the firmware fills ~99% of the chip's 16 KB flash by design. That's expected. If you add code and it overflows ("region `text' overflowed"), see [ARCHITECTURE.md → Memory strategy](ARCHITECTURE.md#memory-strategy).

---

## 9. Write the fuses — once per chip

Before the **first** upload to a given chip, write its fuses. This configures the clock, brown-out, UPDI pin, and — most importantly — the **`EESAVE`** fuse that **preserves each module's ID, calibration, and flap set across future reflashes**. This is the PlatformIO equivalent of the Arduino IDE's *Burn Bootloader*.

- **In VSCode:** PlatformIO sidebar → *Project Tasks → ATtiny1616 → Platform → **Set Fuses*** (it may be labelled *Set Fuses* / *Burn Bootloader* / *fuses* depending on version).
- **Or from the terminal:**
  ```bash
  pio run -t fuses
  ```

You only need to do this **once per physical chip**. After that, every upload keeps EEPROM intact.

> Skipping this means every upload **erases** the module's EEPROM, so it forgets its bus ID, calibration, and configured flap set and reverts to an unprovisioned default state.

---

## 10. Upload (flash) the firmware

With the module powered and the programmer connected:

- **In VSCode:** click the **Upload** button (the → arrow) in the bottom status bar, or press `Ctrl+Alt+U`, or use *Project Tasks → ATtiny1616 → General → Upload*.
- **Or from the terminal:**
  ```bash
  pio run -t upload
  ```

This builds (if needed) and flashes over SerialUPDI. Success ends with an avrdude "X bytes of flash verified" and `[SUCCESS]`.

Every module runs the **same** binary — there is nothing to change per-module before flashing. Bus IDs and the flap set are assigned later over the RS-485 bus (see the README).

---

## 11. Verify it's running

Once flashed and connected to your RS-485 bus, you can confirm the version over the bus (see [README.md](README.md) for the full protocol and the `provision.py` tool):

```
m<ID>v\n      → m<ID>v:31:<ID>:<serialNumber>\n
```

A freshly flashed, never-provisioned module advertises its serial number every ~10–15 seconds and has bus ID 255 until you provision it.

---

## Everyday workflow

After the one-time setup, the normal loop is just two clicks (or two commands):

| Action | VSCode button | CLI |
|---|---|---|
| Compile | Build ✓ | `pio run` |
| Flash | Upload → | `pio run -t upload` |
| Write fuses (once per new chip) | *Set Fuses* task | `pio run -t fuses` |
| Clean build artifacts | *Clean* task | `pio run -t clean` |

You do **not** re-run the fuses step on subsequent uploads to the same chip.

---

## Command-line (no VSCode) alternative

If you prefer the terminal, you can install just the PlatformIO Core CLI (`pio`) without VSCode — see <https://docs.platformio.org/en/latest/core/installation/index.html>. Then, from the folder containing `platformio.ini`:

```bash
pio run                 # build
pio run -t fuses        # write fuses (once per chip)
pio run -t upload       # build + flash
pio device list         # list serial ports (to fill in upload_port)
```

---

## Troubleshooting

**"UPDI link initialization failed" / can't connect**
1. **Is the target powered?** The SerialUPDI adapter does not power the board. This is the most common cause.
2. Check the UPDI wiring (TX—resistor—RX junction → UPDI pin) and that adapter GND is tied to module GND.
3. Confirm `upload_port` matches your adapter (Section 7).
4. Try a slower upload speed: in `platformio.ini` set `upload_speed = 28800` (or lower).

**Wrong/!busy serial port**
- Close any serial monitor or other program holding the port. On Linux, ensure your user is in the `dialout` group.
- Unplug/replug the adapter and re-check the port name.

**Build fails with `'T' does not name a type` (or similar, pointing at a comment)**
- You are trying to build a `.ino` file. This project is `.cpp` on purpose — PlatformIO's `.ino` preprocessor mis-parses the apostrophes in the comments. Build `src/SplitFlapUniversalFirmware.cpp`.

**`region 'text' overflowed by N bytes`**
- The firmware is near the 16 KB flash limit. Don't switch away from `-lprintf_min`/`-Os`/`-flto` in `platformio.ini`. If you added code, factor out duplication (see [ARCHITECTURE.md → Memory strategy](ARCHITECTURE.md#memory-strategy)). As a last resort, the bottom of `platformio.ini` documents `board_upload.maximum_size = 16384` to reclaim any flash a bootloader region might reserve.

**Module forgets its ID / calibration after a reflash**
- The `EESAVE` fuse isn't set. Run the fuses step (Section 9) once per chip. Keep periodic `mXA` dumps so you can restore a module with `mXW` if its EEPROM is ever wiped (see README).

**PlatformIO didn't recognise the project**
- Make sure you opened the folder that directly contains `platformio.ini` (the `31` folder), not a parent folder.
