# Flashing a release with avrdude, no PlatformIO needed

You do not need VSCode, PlatformIO or a compiler to put the current firmware on a module. Every [release](https://github.com/avandeputte/SplitFlapUniversalFirmware/releases/latest) includes the compiled `firmware.hex`, and the free command-line tool **avrdude** writes it to the chip through the [SerialUPDI flasher](FLASHER.md). Two commands per module: one writes the fuses, one writes the firmware.

## What you need

- The flasher and its three jumper wires, from [FLASHER.md](FLASHER.md).
- `firmware.hex` from the **Assets** section of the [latest release](https://github.com/avandeputte/SplitFlapUniversalFirmware/releases/latest). Save it somewhere easy to reach, such as your Downloads folder.
- **avrdude 7.0 or newer.** The `serialupdi` programmer was added in 7.0; older versions do not know it.

| System | Install |
|---|---|
| macOS | `brew install avrdude` ([Homebrew](https://brew.sh)) |
| Windows | Download the Windows zip from the [avrdude releases page](https://github.com/avrdudes/avrdude/releases), unzip it, and run `avrdude.exe` from that folder in a Command Prompt. Windows 10 and 11 normally install the CH340 driver by themselves; if no COM port appears, install the WCH `CH341SER` driver. |
| Linux | `sudo apt install avrdude` on Debian or Ubuntu 24.04 and newer. Older distributions ship avrdude 6.x, which lacks `serialupdi`; use the release build from the page above instead. Give yourself access to the port with `sudo usermod -aG dialout $USER`, then log out and back in. |

Check the install: `avrdude -c '?'` lists the supported programmers, and `serialupdi` should be among them.

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
  -U fuse8:w:0x00:m \
  -U lock:w:0xC5:m
```

Windows, one line:

```
avrdude -c serialupdi -p attiny1616 -P COM5 -b 57600 -U fuse0:w:0x00:m -U fuse1:w:0x54:m -U fuse2:w:0x02:m -U fuse4:w:0x00:m -U fuse5:w:0xC5:m -U fuse6:w:0x06:m -U fuse7:w:0x00:m -U fuse8:w:0x00:m -U lock:w:0xC5:m
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
| `fuse8` BOOTEND | `0x00` | No bootloader section. |
| `lock` | `0xC5` | Unlocked. |

These are the same values `platformio.ini` writes with `pio run -t fuses`. If a future release changes a fuse, the release notes will say so.

## Step 2: write the firmware

```
avrdude -c serialupdi -p attiny1616 -P PORT -b 57600 -U flash:w:firmware.hex:i
```

Run it from the folder where you saved `firmware.hex`, or give the full path to the file. It takes about 25 seconds and ends with a line like `16244 bytes of flash verified` (the exact number depends on the release). The module restarts on the new firmware immediately.

## Step 3, optional: confirm the settings survived

```
avrdude -c serialupdi -p attiny1616 -P PORT -b 57600 -U eeprom:r:-:h
```

This prints the module's EEPROM as hex. The first value should be `0x5d` (settings present) and the sixth value is the module's bus ID (`0xff` means unprovisioned). If the first value is `0xff` on a module that had settings, the fuses were not written before the flash.

## Doing a whole wall

Repeat for each module: connect the three wires, Step 1, Step 2, disconnect, next. Once a module has the right fuses, later firmware updates need only Step 2. On the bus afterwards, `m<ID>v` reports the firmware version so you can confirm every module took the update.

## Troubleshooting

- **`UPDI link initialization failed`**: the module is not answering. Check all three wires at both ends, wait a few seconds and try again, confirm the diode band is at `TXD` (see [FLASHER.md](FLASHER.md)), and as a last resort use `-b 28800`.
- **`unknown programmer` or `invalid programmer`**: your avrdude is older than 7.0. Install a current one from the releases page.
- **`can't open device` on Linux**: you are not in the `dialout` group yet. Add yourself and log in again.
- **`can't open device` on Windows**: wrong COM number, or the CH340 driver is missing.
- **Verification error partway through**: the link dropped. Run Step 2 again.
