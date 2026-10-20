# Bootloader

The 1 KB program at the start of every module's flash, from v34 on. It starts the firmware, and, when asked, replaces it with an image sent over the RS-485 bus. [BUS-UPDATE.md](../BUS-UPDATE.md) describes the protocol and the update procedure. This page is for working on the bootloader itself.

**The bootloader can only be replaced over UPDI.** Every module carries the version it was last flashed with over UPDI, and an update over the bus never touches it. A change here therefore reaches a wall only by visiting every module with the web flasher. Make one only for a good reason, keep the protocol backward compatible, and bump `BL_VERSION`.

## Files

| File | What it is |
|---|---|
| `bootloader.c` | Everything but the bit timing: the boot decision, frames, pages, status, the image check. |
| `uart.S` | The bus at 9600 baud, bit-banged on the firmware's RS-485 pins, in assembly so every bit takes a known number of cycles. |
| `bootloader.hex` | The build that goes onto modules. Committed, so every firmware build merges exactly these bytes. |
| `build.py` | Builds `bootloader.hex` with PlatformIO's avr-gcc; `--test` also runs the host tests. |
| `merge.py` | The firmware build's last step (`extra_scripts` in `platformio.ini`). It puts the bootloader and the image CRC around the firmware in `firmware.hex`, and writes `firmware-update.bin`. |
| `test/` | The host tests: `bootloader.c` on a simulated chip. |

## Flash layout

| Address | What |
|---|---|
| `0x0000`–`0x03FB` | The bootloader. It runs at every reset. |
| `0x03FC`–`0x03FF` | Its signature: `S` `F` `B` and its version. The web flasher sets `BOOTEND` from it, and the firmware reports it as the last field of `Q`. |
| `0x0400`–`0x3FFD` | The firmware, linked to start here (`board_build.text_section_start`). |
| `0x3FFE`–`0x3FFF` | The image CRC, CRC-16/CCITT-FALSE of `0x0400`–`0x3FFD`, big-endian. |

The fuse `BOOTEND = 0x04` makes `0x0000`–`0x03FF` the boot section. The chip then puts the firmware's interrupt vectors at `0x0400`, and refuses writes to the boot section from code running in it.

## At every reset

1. **Speed.** The clock goes to 10 MHz.
2. **An update request.** If the firmware asked for an update, the bootloader stays in update mode. The firmware asks with `U`: it leaves `0xB007` at `0x3F00` in RAM, then does a software reset. The bootloader clears that word in every case. It leaves the reset flags alone, for the firmware to report.
3. **The firmware.** If the stored image CRC is not `0xFFFF` and matches the CRC of `0x0400`–`0x3FFD`, it jumps to `0x0400`. This check takes about 150 ms.
4. **Otherwise**, update mode, until a good image arrives.

In update mode it drives only the bus pins: TX, and DE while it answers a status request. The watchdog runs with an 8-second timeout, and every good frame resets it. RAM use is a 110-byte state block at `0x3800` plus a few bytes of stack.

## Why it is built the way it is

- **1 KB, in C.** An earlier version, all in assembly, fitted 512 bytes, but only after giving up checks. 1 KB keeps the checks, keeps the code readable, and lets the host tests run the real `bootloader.c`. v34 removed `strtol` and `atol` from the firmware, which paid for most of the extra space.
- **Every page is read back after it is written.** A page that reads back wrong does not count, and the gateway sends it again.
- **`G` needs every page, then the CRC.** A CRC-16 check alone would accept a mixed image about once in 65,536 tries; requiring every page to have been read back first rules that out.
- **CRC-16/CCITT-FALSE**, the same CRC as the firmware's checksummed frames, so a gateway needs only one CRC routine.

## Building

```
python3 bootloader/build.py --test
```

The build refuses code that would run into the signature. It also refuses any compiler warning. `-fno-move-loop-invariants` saves about 20 bytes with this compiler (avr-gcc 7.3). Commit `bootloader.hex` together with the source change.

## Testing

`test/test_bootloader.c` compiles `bootloader.c` for the computer, against a simulated chip (`test/host_hal.h`). The simulated chip has flash written only through the page buffer, a protected boot section, reset flags that add up, a watchdog, and a bus as a timeline of bytes, where anything sent while the bootloader is busy is lost. The tests cover:

- every boot decision;
- updates to one module, to all, to an unprovisioned module, and frames for others;
- missing pages, pages that read back wrong, and a change of image part way;
- garbage, bad CRCs, frames sent too close together, and line noise;
- power lost at every flash write of an update, before, during and after the write. Each time, the module comes back with either the old firmware or the new one, never a mix. When it can run neither, it waits for an update, and the next update completes.

The bit timing in `uart.S` is not simulated. Its cycle counts are in its comments: 1,043 cycles a bit to receive and 1,046–1,047 to send, against 1,041.7 at 9600 baud, well inside what a UART tolerates.
