# PlatformIO post-build step: platformio.ini has extra_scripts = post:bootloader/merge.py
#
# The firmware is linked to start at 0x0400.  This turns the linker's
# firmware.hex into the image the chip needs, in place:
#   0x0000-0x03FF  the bootloader, from bootloader/bootloader.hex
#   0x0400-...     the firmware
#   0x3FFE-0x3FFF  CRC-16/CCITT-FALSE of 0x0400..0x3FFD, big-endian: the
#                  bootloader starts the firmware only if this checks out
# so that `pio run -t upload`, avrdude and the web flasher all write the
# whole thing.  It also writes firmware-update.bin next to it: 0x0400..0x3FFF,
# the part the gateway sends over the bus (BUS-UPDATE.md).
import os

Import("env")  # noqa: F821 (provided by PlatformIO)

FLASH_SIZE = 0x4000
APP_START = 0x0400
CRC_AT = FLASH_SIZE - 2
BOOTLOADER_HEX = os.path.join(env.subst("$PROJECT_DIR"), "bootloader", "bootloader.hex")  # noqa: F821


def read_hex(path):
    """{address: byte} from an Intel HEX file."""
    data, base = {}, 0
    with open(path) as f:
        for n, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            rec = bytes.fromhex(line[1:])
            if line[0] != ":" or len(rec) != rec[0] + 5 or sum(rec) & 0xFF:
                raise ValueError(f"{path}:{n}: not a valid Intel HEX record")
            kind, addr = rec[3], (rec[1] << 8) | rec[2]
            if kind == 0x00:
                for i, b in enumerate(rec[4:-1]):
                    data[base + addr + i] = b
            elif kind == 0x01:
                break
            elif kind == 0x02:
                base = ((rec[4] << 8) | rec[5]) << 4
            elif kind == 0x04:
                base = ((rec[4] << 8) | rec[5]) << 16
    return data


def write_hex(path, image, ranges):
    lines = []
    for lo, hi in ranges:
        for at in range(lo, hi, 16):
            chunk = image[at:min(at + 16, hi)]
            rec = bytes([len(chunk), at >> 8, at & 0xFF, 0]) + chunk
            lines.append(":" + (rec + bytes([-sum(rec) & 0xFF])).hex().upper())
    lines.append(":00000001FF")
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")


def crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def merge(source, target, env):
    hex_path = str(target[0])
    firmware = read_hex(hex_path)
    bootloader = read_hex(BOOTLOADER_HEX)
    if not firmware or min(firmware) < APP_START:
        raise SystemExit(f"{hex_path}: the firmware must start at {APP_START:#06x} (link with .text there)")
    if max(firmware) >= CRC_AT:
        raise SystemExit(f"{hex_path}: the firmware runs into the image CRC at {CRC_AT:#06x}")
    if max(bootloader) >= APP_START:
        raise SystemExit(f"{BOOTLOADER_HEX}: the bootloader runs past {APP_START:#06x}")

    image = bytearray(b"\xff" * FLASH_SIZE)
    for a, b in list(bootloader.items()) + list(firmware.items()):
        image[a] = b
    crc = crc16(image[APP_START:CRC_AT])
    if crc == 0xFFFF:                       # means "no image" to the bootloader
        if CRC_AT - 1 in firmware:
            raise SystemExit("the image CRC is 0xFFFF and there is no spare byte to change it")
        image[CRC_AT - 1] = 0xFE
        crc = crc16(image[APP_START:CRC_AT])
    image[CRC_AT], image[CRC_AT + 1] = crc >> 8, crc & 0xFF

    end = max(firmware) + 1
    write_hex(hex_path, image, [(0, end), (CRC_AT - 1 if image[CRC_AT - 1] != 0xFF else CRC_AT, FLASH_SIZE)])
    with open(os.path.join(os.path.dirname(hex_path), "firmware-update.bin"), "wb") as f:
        f.write(image[APP_START:])
    sig = bytes(image[APP_START - 4:APP_START])
    version = sig[3] if sig[:3] == b"SFB" else "?"
    print(f"Image: bootloader v{version} ({len(bootloader)} bytes), firmware {end - APP_START} bytes "
          f"(of {CRC_AT - APP_START}), CRC {crc:04X}; firmware-update.bin for the bus")


env.Depends("$BUILD_DIR/${PROGNAME}.hex", BOOTLOADER_HEX)  # noqa: F821
env.AddPostAction("$BUILD_DIR/${PROGNAME}.hex", merge)  # noqa: F821
