#!/usr/bin/env python3
"""Update split-flap modules' firmware over the RS-485 bus (BUS-UPDATE.md).

    python3 tools/busflash.py PORT firmware-update.bin --id 27
    python3 tools/busflash.py PORT firmware-update.bin --id 3 --id 4 --id 5
    python3 tools/busflash.py PORT firmware-update.bin --all --id 3 --id 4

PORT is a USB-RS485 adapter on the modules' bus (with nothing else talking;
unplug the gateway).  firmware-update.bin comes with every release from v35.
The modules must already run v35 or later, which installed the bootloader.

--id N   a module to update; give it once per module.  The modules are asked
         for their status one by one, so every module must be named.
--all    send the pages to every module at once (address 0xFF) instead of to
         each in turn: a whole wall in about 45 s instead of 45 s a module.

This is the reference for a gateway's implementation: it does what
BUS-UPDATE.md describes, step by step, with the timing it gives.
Needs pyserial: pip install pyserial
"""
import argparse
import binascii
import sys
import time

try:
    import serial
except ImportError:
    sys.exit('pyserial is needed: pip install pyserial')

PAGE = 64
FIRST_PAGE, LAST_PAGE = 16, 255           # the firmware: 0x0400..0x3FFF
IMAGE_SIZE = (LAST_PAGE + 1 - FIRST_PAGE) * PAGE
TO_ALL = 0xFF
BYTE_TIME = 10 / 9600                     # 8N1 at 9600 baud
# Quiet time after the last byte of a frame (BUS-UPDATE.md, "Timing"):
PAGE_GAP = 0.020                          # after P: the module writes and reads back the page
STATUS_WAIT = 0.25                        # after S: the reply starts ~3 ms later and takes ~90 ms
GO_WAIT = 0.5                             # after G: two image checks and a restart
ENTER_WAIT = 0.2                          # after U: the restart into the bootloader


def crc16(data):
    return binascii.crc_hqx(bytes(data), 0xFFFF)    # CRC-16/CCITT-FALSE


class Bus:
    """Times everything from when a frame's last byte is on the wire, worked out
    from the baud rate: a USB adapter's write() returns before that."""
    def __init__(self, port):
        self.s = serial.Serial(port, 9600, timeout=0.05)
        self.rx = b''
        self.done_at = time.time()

    def send(self, line):
        self.idle(0)
        self.s.reset_input_buffer()       # also drops an adapter's echo of earlier frames
        self.rx = b''
        start = time.time()
        self.s.write(line)
        self.done_at = start + len(line) * BYTE_TIME

    def idle(self, gap):
        """Wait until `gap` seconds after the last byte sent."""
        delay = self.done_at + gap - time.time()
        if delay > 0:
            time.sleep(delay)

    def frame(self, data):
        """A bootloader frame: '#', upper-case hex of data and its CRC, '\\n'."""
        data = bytes(data)
        self.send(b'#' + (data + crc16(data).to_bytes(2, 'big')).hex().upper().encode() + b'\n')

    def firmware_frame(self, text):
        """A firmware command, checksummed: ~HHHH<text>\\n (the firmware's CHECKSUMS)."""
        body = text.encode()
        self.send(b'~%04X' % crc16(body) + body + b'\n')

    def lines(self, wait):
        """Lines received until `wait` seconds after the last byte sent."""
        end = self.done_at + wait
        while time.time() < end:
            self.rx += self.s.read(256)
        *done, self.rx = self.rx.split(b'\n')
        return done


def status(bus, mid, tries=3):
    """The bootloader's status for module mid, or None if it does not answer."""
    for _ in range(tries):
        bus.frame([mid, ord('S')])
        for line in bus.lines(STATUS_WAIT):
            line = line.strip()
            if not line.startswith(b'#'):
                continue                  # an adapter's own echo, or noise
            try:
                d = bytes.fromhex(line[1:].decode())
            except ValueError:
                continue
            if len(d) == 41 and crc16(d) == 0 and d[0] == mid and d[1] == ord('s'):
                return {'version': d[2], 'reason': d[3], 'flags': d[4],
                        'image': int.from_bytes(d[5:7], 'big'), 'pages': d[7:39]}
    return None


def missing(st, image_crc):
    if st['image'] != image_crc:
        return list(range(FIRST_PAGE, LAST_PAGE + 1))
    return [p for p in range(FIRST_PAGE, LAST_PAGE + 1) if not st['pages'][p // 8] & (1 << (p % 8))]


def send_pages(bus, to, image, image_crc, pages):
    for n, p in enumerate(pages, 1):
        data = image[(p - FIRST_PAGE) * PAGE:(p - FIRST_PAGE + 1) * PAGE]
        bus.frame([to, ord('P'), image_crc >> 8, image_crc & 0xFF, p] + list(data))
        bus.idle(PAGE_GAP)
        if n % 20 == 0 or n == len(pages):
            print(f'\r  {n}/{len(pages)} pages', end='', flush=True)
    print()


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('port')
    ap.add_argument('image', help='firmware-update.bin from a release')
    ap.add_argument('--id', type=int, action='append', required=True, help='a module to update')
    ap.add_argument('--all', action='store_true', help='send the pages to all modules at once')
    args = ap.parse_args()

    image = open(args.image, 'rb').read()
    if len(image) != IMAGE_SIZE:
        sys.exit(f'{args.image}: {len(image)} bytes, expected {IMAGE_SIZE} (firmware-update.bin)')
    image_crc = int.from_bytes(image[-2:], 'big')
    if crc16(image[:-2]) != image_crc or image_crc == 0xFFFF:
        sys.exit(f'{args.image}: its CRC does not check out')
    bus = Bus(args.port)
    ids = args.id

    print('Into the bootloader:', ', '.join(map(str, ids)))
    for mid in ([TO_ALL] if args.all else ids):
        bus.firmware_frame('m*U' if mid == TO_ALL else f'm{mid}U')
        bus.idle(ENTER_WAIT)
    for mid in ids:
        st = status(bus, mid)
        if not st:
            sys.exit(f'module {mid} does not answer in its bootloader: is it running v35 or later?')
        print(f'  module {mid}: bootloader {st["version"]}')

    todo = {mid: list(range(FIRST_PAGE, LAST_PAGE + 1)) for mid in ids}
    for attempt in range(1, 6):
        if args.all:
            pages = sorted(set().union(*todo.values()))
            print(f'Pass {attempt}: {len(pages)} pages to all')
            send_pages(bus, TO_ALL, image, image_crc, pages)
        else:
            for mid, pages in todo.items():
                print(f'Pass {attempt}: {len(pages)} pages to module {mid}')
                send_pages(bus, mid, image, image_crc, pages)
        for mid in list(todo):
            st = status(bus, mid)
            todo[mid] = missing(st, image_crc) if st else todo[mid]
            if not todo[mid]:
                del todo[mid]
        if not todo:
            break
        print('  still missing:', {m: len(p) for m, p in todo.items()})
    if todo:
        sys.exit(f'gave up: modules {sorted(todo)} still miss pages')

    print('Go')
    for mid in ([TO_ALL] if args.all else ids):
        bus.frame([mid, ord('G')])
        bus.idle(GO_WAIT)
    failed = [mid for mid in ids if status(bus, mid, tries=1)]
    if failed:
        sys.exit(f'modules {failed} are still in their bootloader: the image did not check out')
    print('Done: every module started the new firmware.  Check with m<id>v once they have homed.')


if __name__ == '__main__':
    main()
