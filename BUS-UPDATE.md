# Updating firmware over the bus

From v35 every module carries a bootloader: 1 KB at the start of its flash that can replace the firmware with an image sent over the RS-485 bus. A gateway, or a computer with a USB–RS485 adapter, can then update a whole wall at once in under a minute, without touching a module.

This page describes the protocol and the procedure for whoever implements the sender. [`tools/busflash.py`](tools/busflash.py) is a reference sender, and [`bootloader/`](bootloader/) holds the bootloader's source, build and tests.

## Getting the bootloader onto a module

Installing it takes one last flash over UPDI per module, with the [web flasher](https://avandeputte.github.io/SplitFlapUniversalFirmware/) or as in [FLASHING.md](FLASHING.md). Any v35 or later image includes the bootloader, and the fuse that reserves its section is `BOOTEND = 0x04`, which the web flasher sets by itself. After that, every update can go over the bus.

A module has the bootloader if its firmware is v35 or later. The last field of the `Q` diagnostics reply gives the bootloader's version, which is 0 if there is none.

## Two boards, one image each

Since v36 there are two driver boards, and each has its own firmware and bootloader. They differ only in how they reach the RS-485 transceiver:

| Board | Bus | Bootloader | Update image |
|---|---|---|---|
| v4 (and earlier) | bit-banged on PA5/PA7, DE on PA6 | version 1 | `firmware-update-v4.bin` |
| v5 | the hardware USART on PA1/PA2, DE on PA4 | version 2 | `firmware-update-v5.bin` |

A v5 board grounds PC0, which floats on a v4. The firmware reads it at every start.

**Send each module the image for its board.** From v36, the last field of the `v` reply is the board: `m<id>v:<version>:<id>:<serial>:<board>`. A module that does not report one runs v35 or older, and is a v4. In update mode the status reply says it too: a version 2 bootloader is on a v5.

**A wrong image does no harm.** A module given the other board's image checks it, starts it, and the firmware, seeing the wrong board, goes straight back to the bootloader without touching the bus pins. It then waits there, in update mode, for the right image, which an update over the bus or a UPDI flash gives it.

A wall with both boards takes two updates, one per image. Gateway 4.3.0 and later leave a module alone if the uploaded image is not for its board.

## The image

Every release from v36 includes `firmware-update-v4.bin` and `firmware-update-v5.bin`, one per board; v35 had a single `firmware-update.bin`, for the v4. Each is 15,360 bytes, holding the flash from `0x0400` to `0x3FFF`:

- **Pages:** the image is 240 pages of 64 bytes, numbered 16 to 255. Page *n* is the 64 bytes at offset (*n* − 16) × 64 in the file.
- **Board:** the byte at flash `0x3FFC`, offset 15,356 in the file: 4 or 5. The next byte is reserved, `FF`. Images before v36 have `FF` there, and are all v4 images.
- **Image CRC:** the file's last two bytes, big-endian. It is the CRC-16 of everything before them, the board included.

The same bytes are in the release's `firmware-v4.hex` or `firmware-v5.hex`, from `0x0400`, for a sender that would rather parse that.

All CRCs on this page are **CRC-16/CCITT-FALSE**: polynomial `0x1021`, initial value `0xFFFF`, no reflection, no final XOR. That is the same CRC as the firmware's checksummed frames; in Python, `binascii.crc_hqx(data, 0xFFFF)`.

## Frames

A bootloader frame is one line: `#`, the frame's bytes in upper-case hex, then `\n`:

```
<to> <command> <payload ...> <CRC hi> <CRC lo>
```

- **`to`:** a module ID, or `FF` for every module.
- **CRC:** covers every byte before it.
- **Rejected frames:** a frame that is not upper-case hex, has an odd number of digits, is too long or fails its CRC is ignored. A `#` anywhere starts a new frame.
- **Modules not in update mode** ignore these frames entirely. Their firmware only acts on lines starting with `m` or `~`, so update traffic can run while other modules on the bus keep their firmware.

| Command | Bytes | What it does | Reply |
|---|---|---|---|
| `P` page | `to` `50` `<image CRC: 2>` `<page>` `<64 bytes>` | Writes one page of the image identified by the image CRC. The module reads the page back, and it counts only if it matches. Pages below 16 (the bootloader) are refused. | none |
| `S` status | `to` `53` | Reports the module's state. Never answered when `to` is `FF`. | yes |
| `G` go | `to` `47` | Starts the image, if every one of its 240 pages has been written and read back and the whole image matches its CRC. Sent before any `P`, it returns to the firmware already there. | none: the module restarts |

The status reply is a frame too:

```
<id> 73 <version> <reason> <flags> <image CRC: 2> <pages: 32> [<board>] <CRC: 2>
```

That is 41 bytes from a version 1 bootloader, and 42 from version 2, which adds the board.

| Field | Meaning |
|---|---|
| `id` | The module's ID. `73` is `'s'`. |
| `version` | The bootloader's version: 1 on a v4 board, 2 on a v5. |
| `reason` | Why the module is in update mode: 1 if it was asked to (`U`), 2 if it has no valid firmware. |
| `flags` | Bit 0: the last `G` did not start an image (some pages missing, or the image did not match its CRC). |
| `image CRC` | The image being received; `FFFF` if none yet. |
| `pages` | 256 bits, bit *n* % 8 of byte *n* / 8: page *n* of that image has been written and read back right. Pages 0–15 are always 0. |
| `board` | Version 2 only: the board, 5. A version 1 reply has no such byte, and comes from a v4. |

Example: the gateway sends `#1B53` followed by the CRC of `1B 53` and `\n`. Module 27 answers with `#1B73010100B52C0000FFFF…FF` plus its CRC.

## Timing

The bus runs at 9600 baud, 8N1, as for the firmware. While a module writes a page, or works out a CRC, it does not listen, and anything sent in that time is lost. So the sender leaves a gap after each frame, counted from the frame's last byte.

| After | Wait at least | Why |
|---|---|---|
| `P` | 20 ms | The module erases, writes and reads back the page: about 5 ms. |
| `S` | until the reply, up to 250 ms | The reply starts about 3 ms after the request and takes about 90 ms. |
| `G` | 500 ms | The module checks the whole image (about 150 ms), stores its CRC, restarts and checks it again. |
| `U` | 200 ms | The module restarts into its bootloader. |

A page frame is 144 characters, so the 240 pages take about 40 seconds with their gaps, for one module or the whole wall alike.

**Watchdog:** a module in update mode restarts after 8 seconds without a good frame on the bus, for any module. Keep frames coming during an update: a status request for another module counts. A module that restarts this way keeps what it has written, but forgets which pages it has written, so it needs them all again. If it had not received any page yet, it goes back to its firmware.

## The procedure

1. **Check the modules.** `m<id>v` gives each module's firmware version, and from v36 its board. Only modules on v35 or later have the bootloader. Update only the modules whose board matches the image's.
2. **Into update mode.** Send `m*U` for every module, or `m<id>U` for one, **as a checksummed frame**: `~HHHHm*U\n`, where `HHHH` is the CRC of `m*U` in upper-case hex. A plain `U` is ignored, so line noise can never take a module off line. Older firmware ignores `U` too. There is no reply. On a wall with both boards, send `m<id>U` to the modules of the image's board only: every module in update mode takes the pages sent to `FF`, and the others then wait in their bootloader for their own image.
3. **Confirm.** `S` to each module. An answer with reason 1 means it is waiting. Check its board here too, if step 1 could not: a module on the wrong board can be sent `G` before any `P`, which returns it to its firmware.
4. **The pages.** Send pages 16 to 255 to `FF`, with the gaps above. Every module in update mode takes them.
5. **Check each module.** `S` to each one. Its image CRC must be the image's, and its page bits must all be set for 16–255. Resend any missing pages, to `FF` or to that module only, then ask again.
6. **Go.** Send `G`, to `FF` or to each module. A module that has every page checks the image and restarts into the new firmware. A module with pages missing stays in update mode, with flag bit 0 set.
7. **Confirm the new firmware.** Once the modules have homed, `m<id>v` should give the new version. A module that still answers `S` is still in its bootloader, and its status says why.

To leave update mode without changing anything, send `G` before any `P`. After the first `P`, the old firmware is no longer valid, so the module waits until a complete image arrives.

## What the bootloader guarantees

- **It never starts a partial image.** It erases the stored image CRC before writing the first page of a new image. It writes the CRC back only after every page has been written and read back, and the whole image matches. A module that loses power, or loses the gateway, part way through just waits in update mode for the image. Simulated power loss at every flash write of an update never produced anything else.
- **It never touches the bootloader section.** The chip refuses writes there, and pages below 16 are refused anyway.
- **It never writes EEPROM**, so the module's ID and calibration survive any update, finished or not.
- **It answers only `S`**, and only when that module is addressed by its own ID.
- **At every start** it checks the whole image against its CRC before running it, about 150 ms.
- **The v5 build stays off the bus on any other board.** It checks the PC0 strap first, and on a v4, where its pins are the IO header, it drives nothing and waits for the watchdog. Only a UPDI flash of the wrong image could put it there.

If anything ever went wrong with the bootloader itself, the [web flasher](https://avandeputte.github.io/SplitFlapUniversalFirmware/) restores a module over UPDI.

## Testing without the gateway

With a USB–RS485 adapter on the bus and the gateway unplugged:

```
pip install pyserial
python3 tools/busflash.py /dev/cu.usbserial-XXXX firmware-update-v4.bin --all --id 3 --id 4 --id 5
```

`--id` names each module to update. `--all` sends the pages once, to every named module at once; without it, each module gets its own copy in turn. It reads the board from the image and from each module's bootloader status, and stops before sending a page if any named module is on the other board.

**Not yet tried on a v5 board.** On the v5 modules flashed so far over UPDI, the firmware starts, and it sends on the bus. Its receiving, and an update over the bus, have not been tried on one yet.
