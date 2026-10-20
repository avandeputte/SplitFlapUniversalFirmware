#!/usr/bin/env python3
"""Build the bootloader: bootloader.hex, next to this file.

    python3 bootloader/build.py            # build, and check it fits
    python3 bootloader/build.py --test     # also run the host tests

Uses PlatformIO's avr-gcc (~/.platformio/packages/toolchain-atmelavr), or
the one named by $AVR_GCC, or avr-gcc on the PATH.  bootloader.hex is
committed: the firmware build merges that file, so every module gets the
same bootloader whatever compiler built the firmware.  Rebuild it only when
the bootloader changes, and commit the two together.
"""
import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
SIGNATURE_AT = 0x3FC        # the last 4 bytes of the 1 KB boot section (BOOTEND = 0x04)


def tool(name):
    exe = name + ('.exe' if os.name == 'nt' else '')
    env = os.environ.get('AVR_GCC')
    candidates = [os.path.join(os.path.dirname(env), exe)] if env else []
    candidates.append(os.path.join(os.path.expanduser('~'), '.platformio', 'packages',
                                   'toolchain-atmelavr', 'bin', exe))
    for path in candidates:
        if os.path.isfile(path):
            return path
    found = shutil.which(exe)
    if found:
        return found
    sys.exit(f'{name} not found: install PlatformIO, or set AVR_GCC to the avr-gcc to use')


def main():
    gcc, objcopy, objdump = tool('avr-gcc'), tool('avr-objcopy'), tool('avr-objdump')
    with tempfile.TemporaryDirectory() as tmp:
        elf = os.path.join(tmp, 'bootloader.elf')
        subprocess.run([gcc, '-mmcu=attiny1616', '-Os', '-fno-split-wide-types', '-mrelax',
                        '-fno-move-loop-invariants', '-Wall', '-Wextra', '-Werror',
                        '-nostartfiles', '-nostdlib', '-Wl,--relax',
                        '-Wl,--section-start=.text=0',
                        f'-Wl,--section-start=.blsig={SIGNATURE_AT:#x}',
                        '-o', elf,
                        os.path.join(HERE, 'bootloader.c'), os.path.join(HERE, 'uart.S')],
                       check=True)
        sections = subprocess.run([objdump, '-h', elf], check=True, capture_output=True, text=True).stdout
        size = next(int(line.split()[2], 16) for line in sections.splitlines()
                    if len(line.split()) > 2 and line.split()[1] == '.text')
        if size > SIGNATURE_AT:
            sys.exit(f'the bootloader is {size} bytes: it must end before {SIGNATURE_AT:#x}')
        out = os.path.join(HERE, 'bootloader.hex')
        subprocess.run([objcopy, '-O', 'ihex', '-j', '.text', '-j', '.blsig', elf, out], check=True)
        print(f'{out}: {size} bytes of code, {SIGNATURE_AT - size} to spare')

    if '--test' in sys.argv[1:]:
        cc = shutil.which('cc') or shutil.which('gcc') or shutil.which('clang')
        if not cc:
            sys.exit('no host C compiler (cc, gcc or clang) for the tests')
        with tempfile.TemporaryDirectory() as tmp:
            exe = os.path.join(tmp, 'bltest')
            subprocess.run([cc, '-std=c11', '-O2', '-Wall', '-Wextra', '-DHOST',
                            '-I', os.path.join(HERE, 'test'), '-o', exe,
                            os.path.join(HERE, 'test', 'test_bootloader.c')], check=True)
            sys.exit(subprocess.run([exe]).returncode)


if __name__ == '__main__':
    main()
