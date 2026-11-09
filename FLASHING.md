# Flashing a release, no PlatformIO needed

## The easy way: the web flasher

Open the **[web flasher](https://avandeputte.github.io/SplitFlapUniversalFirmware/)** in Chrome or Edge on a desktop computer, choose the flasher, and click **Flash module**. There is nothing to install, and it does everything this page describes:

1. It checks the chip.
2. It saves a backup of the module's settings to a file.
3. It writes the fuses, if the chip needs them.
4. It reads which driver board the module is, then flashes the current release's image for that board and verifies it.
5. It checks the module kept its ID and calibration.

Then attach the next module and click **Flash next module**.

To use it offline, download `flasher.html` from the Assets of the [latest release](https://github.com/avandeputte/SplitFlapUniversalFirmware/releases/latest) and open it in Chrome or Edge. Firefox and Safari can't talk to serial ports; for those, or for scripting, use avrdude as below.

## With avrdude

You do not need VSCode, PlatformIO or a compiler to put the current firmware on a module. Every [release](https://github.com/avandeputte/SplitFlapUniversalFirmware/releases/latest) includes the compiled firmware, one `.hex` file per driver board, and the free command-line tool **avrdude** writes it to the chip through the [SerialUPDI flasher](FLASHER.md). Two commands per module: one writes the fuses, one writes the firmware.

## What you need

- The flasher and its three jumper wires, from [FLASHER.md](FLASHER.md).
- The firmware for your module's board, from the **Assets** section of the [latest release](https://github.com/avandeputte/SplitFlapUniversalFirmware/releases/latest) (see below). Save it somewhere easy to reach, such as your Downloads folder.
- **avrdude 7.0 or newer.** The `serialupdi` programmer was added in 7.0; older versions do not know it.

| System | Install |
|---|---|
| macOS | `brew install avrdude` ([Homebrew](https://brew.sh)) |
| Windows | Download the Windows zip from the [avrdude releases page](https://github.com/avrdudes/avrdude/releases), unzip it, and run `avrdude.exe` from that folder in a Command Prompt. Windows 10 and 11 normally install the CH340 driver by themselves; if no COM port appears, install the WCH `CH341SER` driver. |
| Linux | `sudo apt install avrdude` on Debian or Ubuntu 24.04 and newer. Older distributions ship avrdude 6.x, which lacks `serialupdi`; use the release build from the page above instead. Give yourself access to the port with `sudo usermod -aG dialout $USER`, then log out and back in. |

Check the install: `avrdude -c '?'` lists the supported programmers, and `serialupdi` should be among them.

## Which file: the driver board

Since v36 every release has one image per driver board ([README](README.md#driver-boards-v36)):

| Board | File |
|---|---|
| v4, and earlier boards | `firmware-v4.hex` |
| v5 | `firmware-v5.hex` |

Releases up to v35 have a single `firmware.hex`, which is for the v4.

A module running v36 or later reports its board as the last field of its `m<ID>v` reply, and the gateway's Modules tab shows it. The web flasher reads it from the board itself and needs no choice. avrdude can't tell: it writes whatever it is given.

**A wrong image leaves the module doing nothing.** It also writes the other board's bootloader, which can't reach the transceiver, so the module can't be updated over the bus either. Flash the right image over UPDI to fix it; the settings are kept.
- **On a v5 board, the v4 image is the one to avoid.** Its bootloader drives PA5 and PA6, two pins of the v5's IO header, while it waits.
- **On a v4 board, the v5 image** drives nothing.

## Find the port

Plug the flasher into a USB port, then:

| System | Command | Looks like |
|---|---|---|
| macOS | `ls /dev/cu.usbserial*` | `/dev/cu.usbserial-11430` |
| Linux | `ls /dev/ttyUSB*` | `/dev/ttyUSB0` |
| Windows | Device Manager → Ports (COM & LPT) | `USB-SERIAL CH340 (COM5)` → use `COM5` |

Use that name wherever the commands below say `PORT`.

## Connect the module

Three wires from the flasher to the module's `UPDI Interface` header: `3V3` to `+3V3`, `GND` to `G`, `RXD` to `UPDI`. The flasher powers the module during flashing, so nothing else needs to be connected.

## Step 1: write the fuses

Fuses are the chip's configuration: clock source, brown-out detector, and the setting that keeps EEPROM (the module's ID and calibration) intact when the flash is erased. This has to be done **once per module**, and **before** the firmware on a module that already holds settings, because Step 2 erases the chip and only the `EESAVE` fuse from this step prevents that erase from taking the EEPROM with it. Running it again on a module that already has the right fuses is harmless.

macOS and Linux:

```
avrdude -c serialupdi -p attiny1616 -P PORT -b 57600 \
  -U fuse0:w:0x00:m \
  -U fuse1:w:0x54:m \
  -U fuse2:w:0x02:m \
  -U fuse4:w:0x00:m \
  -U fuse5:w:0xC5:m \
  -U fuse6:w:0x06:m \
  -U fuse7:w:0x00:m \
  -U fuse8:w:0x04:m \
  -U lock:w:0xC5:m
```

Windows, one line:

```
avrdude -c serialupdi -p attiny1616 -P COM5 -b 57600 -U fuse0:w:0x00:m -U fuse1:w:0x54:m -U fuse2:w:0x02:m -U fuse4:w:0x00:m -U fuse5:w:0xC5:m -U fuse6:w:0x06:m -U fuse7:w:0x00:m -U fuse8:w:0x04:m -U lock:w:0xC5:m
```

Each fuse ends with `1 byte written, 1 verified`. What they set:

| Fuse | Value | Meaning |
|---|---|---|
| `fuse0` WDTCFG | `0x00` | Watchdog off at reset; the firmware turns it on itself. |
| `fuse1` BODCFG | `0x54` | Brown-out detector on at 2.6 V. Protects EEPROM during power-up and power-down. |
| `fuse2` OSCCFG | `0x02` | 20 MHz internal oscillator, which the firmware runs at 10 MHz. |
| `fuse4` TCD0CFG | `0x00` | Default. |
| `fuse5` SYSCFG0 | `0xC5` | **EEPROM retained across flashes** (`EESAVE`), UPDI pin left as UPDI. |
| `fuse6` SYSCFG1 | `0x06` | Start-up delay. |
| `fuse7` APPEND | `0x00` | No application data section. |
| `fuse8` BOOTEND | `0x04` | 1 KB boot section, for the bootloader that takes updates over the bus (v35 and later). **To flash v35 or older, use `0x00`**: the fuse must match the image, or the module does not run. |
| `lock` | `0xC5` | Unlocked. |

These are the same values `platformio.ini` writes with `pio run -t fuses`. If a future release changes a fuse, the release notes will say so: v35 changed `fuse8`.

## Step 2: write the firmware

```
avrdude -c serialupdi -p attiny1616 -P PORT -b 57600 -U flash:w:firmware-v4.hex:i
```

On a v5 board, write `firmware-v5.hex` instead. Run it from the folder where you saved the file, or give the full path to it. From v35 the file holds the bootloader as well as the firmware, and this writes both. It takes about 25 seconds and ends with a line like `16244 bytes of flash verified` (the exact number depends on the release). The module restarts on the new firmware immediately.

## Step 3, optional: confirm the settings survived

```
avrdude -c serialupdi -p attiny1616 -P PORT -b 57600 -U eeprom:r:-:h
```

This prints the module's EEPROM as hex. The first value should be `0x5d` (settings present) and the sixth value is the module's bus ID (`0xff` means unprovisioned). If the first value is `0xff` on a module that had settings, the fuses were not written before the flash.

## Doing a whole wall

Repeat for each module: connect the three wires, Step 1, Step 2, disconnect, next. Once a module has the right fuses, later firmware updates need only Step 2. From v35 they do not need the flasher at all: they can go over the bus, to the whole wall at once ([BUS-UPDATE.md](BUS-UPDATE.md)). On the bus afterwards, `m<ID>v` reports the firmware version so you can confirm every module took the update.

## Troubleshooting

- **`UPDI link initialization failed`**: the module is not answering. Check all three wires at both ends, wait a few seconds and try again, confirm the diode band is at `TXD` (see [FLASHER.md](FLASHER.md)), and as a last resort use `-b 28800`.
- **`unknown programmer` or `invalid programmer`**: your avrdude is older than 7.0. Install a current one from the releases page.
- **`can't open device` on Linux**: you are not in the `dialout` group yet. Add yourself and log in again.
- **`can't open device` on Windows**: wrong COM number, or the CH340 driver is missing.
- **Verification error partway through**: the link dropped. Run Step 2 again.
