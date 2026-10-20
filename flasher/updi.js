// =============================================================================
// Split-Flap module flasher: the UPDI programming core
// =============================================================================
// Programs a module's ATtiny1616 through the SerialUPDI flasher (a USB-serial
// adapter whose TX and RX are joined through a diode, see FLASHER.md) with no
// avrdude, PlatformIO or other software installed.  It speaks the chip's UPDI
// protocol directly (ATtiny1614/1616/1617 datasheet, "UPDI - Unified Program
// and Debug Interface"), following the SerialUPDI conventions of Microchip's
// pymcuprog, which avrdude's serialupdi programmer also implements.
//
// The serial port comes from a transport object, so the same code runs in a
// browser (webserial.js, the Web Serial API) and under Node for testing:
//   open(baudRate, stopBits)   8 data bits, even parity
//   close()
//   write(bytes)               Uint8Array
//   read(count, timeoutMs)     resolves exactly `count` bytes, or rejects
//   discardInput()
//
// flashModule() is the whole routine: identify the chip, back up its EEPROM,
// put the fuses right, write and verify the flash, and check that the EEPROM
// (the module's ID and calibration) came through untouched.
// =============================================================================
(function (root) {
  'use strict';

  // ── UPDI instruction set ──────────────────────────────────────────────────
  const SYNC = 0x55, ACK = 0x40;
  const LDS = 0x00, STS = 0x40, LD = 0x20, ST = 0x60;
  const LDCS = 0x80, STCS = 0xC0, REPEAT = 0xA0, KEY = 0xE0;
  const PTR_INC = 0x04, PTR_ADDRESS = 0x08;
  const ADDRESS_16 = 0x04, DATA_8 = 0x00, DATA_16 = 0x01;
  const KEY_SIB = 0x04, SIB_16_BYTES = 0x01, KEY_64_BITS = 0x00;
  // Control/status registers, reached with LDCS/STCS.
  const CS_STATUSA = 0x00, CS_CTRLA = 0x02, CS_CTRLB = 0x03;
  const ASI_KEY_STATUS = 0x07, ASI_RESET_REQ = 0x08, ASI_SYS_STATUS = 0x0B;
  const CTRLA_IBDLY = 0x80, CTRLA_RSD = 0x08;
  const CTRLB_CCDETDIS = 0x08, CTRLB_UPDIDIS = 0x04;
  const KEY_STATUS_NVMPROG = 0x10;
  const SYS_STATUS_RSTSYS = 0x20, SYS_STATUS_NVMPROG = 0x08, SYS_STATUS_LOCKSTATUS = 0x01;
  const RESET_REQ_VALUE = 0x59;
  const KEY_NVMPROG = 'NVMProg ';

  // ── The chip ──────────────────────────────────────────────────────────────
  const CHIP = {
    name: 'ATtiny1616',
    signature: [0x1E, 0x94, 0x21],
    flashBase: 0x8000, flashSize: 0x4000, flashPage: 64,
    eepromBase: 0x1400, eepromSize: 256, eepromPage: 32,
    fuseBase: 0x1280, lockAddress: 0x128A, unlocked: 0xC5,
    sigrow: 0x1100, serialOffset: 3, serialLength: 10,
    nvmctrl: 0x1000,
  };
  const NVM_CTRLA = 0x00, NVM_STATUS = 0x02, NVM_DATAL = 0x06, NVM_ADDRL = 0x08, NVM_ADDRH = 0x09;
  const NVM_ERASE_WRITE_PAGE = 0x03, NVM_PAGE_BUFFER_CLEAR = 0x04, NVM_WRITE_FUSE = 0x07;
  const NVM_FLASH_BUSY = 0x01, NVM_EEPROM_BUSY = 0x02, NVM_WRITE_ERROR = 0x04;

  // The fuses every module needs: exactly what FLASHING.md and `pio run -t fuses`
  // write.  BODCFG holds the chip in reset below 2.6 V, OSCCFG selects 20 MHz,
  // and SYSCFG0 keeps the UPDI pin a UPDI pin and sets EESAVE, so a chip erase
  // leaves the EEPROM alone: avrdude and PlatformIO start every upload with one.
  // (This flasher erases page by page, which never touches the EEPROM.)  Fuse 3
  // does not exist on this part.  SYSCFG0 is written last, after everything
  // else has gone right.
  //
  // BOOTEND depends on the image: from v35 on, an image starts with the 1 KB
  // bootloader that takes updates over the bus, and needs BOOTEND = 0x04 (the
  // firmware's interrupt vectors are then at 0x0400); older images need 0x00.
  // An image has the bootloader if its signature ('S','F','B',version) is at
  // the end of the boot section.
  const BOOTLOADER_SIGNATURE_AT = 0x3FC;
  function bootloaderVersion(firmware) {
    const im = firmware.image, at = BOOTLOADER_SIGNATURE_AT;
    return im[at] === 0x53 && im[at + 1] === 0x46 && im[at + 2] === 0x42 ? im[at + 3] : 0;
  }
  function fusesFor(firmware) {
    return [
      { number: 0, name: 'WDTCFG',  value: 0x00 },
      { number: 1, name: 'BODCFG',  value: 0x54 },
      { number: 2, name: 'OSCCFG',  value: 0x02 },
      { number: 4, name: 'TCD0CFG', value: 0x00 },
      { number: 6, name: 'SYSCFG1', value: 0x06 },
      { number: 7, name: 'APPEND',  value: 0x00 },
      { number: 8, name: 'BOOTEND', value: bootloaderVersion(firmware) ? 0x04 : 0x00 },
      { number: 5, name: 'SYSCFG0', value: 0xC5 },
    ];
  }
  const FUSE_COUNT = 9;                    // fuse0..fuse8, read as one block

  const BAUD = 57600;                      // the speed avrdude uses with this flasher
  const REPLY_MS = 250;                    // a reply or an echo that has not come by then never will

  class FlasherError extends Error {
    constructor(message, hint) { super(message); this.name = 'FlasherError'; this.hint = hint || ''; }
  }

  const hex2 = (b) => b.toString(16).toUpperCase().padStart(2, '0');
  const sleep = (ms) => new Promise((resolve) => setTimeout(resolve, ms));
  const lockedError = () => new FlasherError('This chip is locked.',
    'Unlocking it erases its EEPROM too, so it is left alone: unlock it with avrdude (chip erase) if that is acceptable.');

  // ── UPDI link ─────────────────────────────────────────────────────────────
  class Updi {
    constructor(transport, log) {
      this.t = transport;
      this.log = log || (() => {});
    }

    // Everything sent comes straight back on RX through the flasher's diode:
    // read the echo and check it, which is also the first sign of bad wiring.
    async send(bytes) {
      const data = Uint8Array.from(bytes);
      await this.t.write(data);
      let echo;
      try {
        echo = await this.t.read(data.length, REPLY_MS);
      } catch (err) {
        throw new FlasherError('The flasher does not echo what it sends.',
          'Check that the flasher is the diode-modified adapter from FLASHER.md and that its USB cable is plugged in.');
      }
      for (let i = 0; i < data.length; i++) {
        if (echo[i] !== data[i]) {
          throw new FlasherError('The flasher echoed something other than what was sent.',
            'Something else is driving the UPDI line: check the three wires to the module.');
        }
      }
    }

    async receive(count) {
      try {
        return await this.t.read(count, REPLY_MS);
      } catch (err) {
        throw new FlasherError('The chip did not answer.',
          'Check that the module is connected (3V3, GND and UPDI) and powered.');
      }
    }

    async expectAck(what) {
      const r = await this.receive(1);
      if (r[0] !== ACK) throw new FlasherError(`The chip refused ${what} (0x${hex2(r[0])}).`);
    }

    async ldcs(register) {
      await this.send([SYNC, LDCS | register]);
      return (await this.receive(1))[0];
    }
    async stcs(register, value) { await this.send([SYNC, STCS | register, value]); }

    async ld(address) {
      await this.send([SYNC, LDS | ADDRESS_16 | DATA_8, address & 0xFF, address >> 8]);
      return (await this.receive(1))[0];
    }
    async st(address, value) {
      await this.send([SYNC, STS | ADDRESS_16 | DATA_8, address & 0xFF, address >> 8]);
      await this.expectAck('an address');
      await this.send([value & 0xFF]);
      await this.expectAck('a write');
    }
    async st16(address, value) {
      await this.send([SYNC, STS | ADDRESS_16 | DATA_16, address & 0xFF, address >> 8]);
      await this.expectAck('an address');
      await this.send([value & 0xFF, (value >> 8) & 0xFF]);
      await this.expectAck('a write');
    }
    async setPointer(address) {
      await this.send([SYNC, ST | PTR_ADDRESS | DATA_16, address & 0xFF, address >> 8]);
      await this.expectAck('the pointer');
    }
    async repeat(count) { await this.send([SYNC, REPEAT, (count - 1) & 0xFF]); }

    // Up to 256 bytes from `address`, through the pointer with post-increment.
    async readBytes(address, count) {
      await this.setPointer(address);
      if (count > 1) await this.repeat(count);
      await this.send([SYNC, LD | PTR_INC | DATA_8]);
      return this.receive(count);
    }
    // Up to 128 words (256 bytes): the fast path for reading flash back.
    async readWords(address, byteCount) {
      await this.setPointer(address);
      if (byteCount > 2) await this.repeat(byteCount >> 1);
      await this.send([SYNC, LD | PTR_INC | DATA_16]);
      return this.receive(byteCount);
    }
    // A run of words with replies switched off (RSD) while they stream in.
    // REPEAT applies to the very next instruction, so it goes after the STCS
    // that switches replies off and directly before the ST it repeats.
    async writeWords(address, data) {
      if (data.length === 2) return this.st16(address, data[0] | (data[1] << 8));
      await this.setPointer(address);
      await this.stcs(CS_CTRLA, CTRLA_IBDLY | CTRLA_RSD);
      await this.repeat(data.length >> 1);
      await this.send([SYNC, ST | PTR_INC | DATA_16]);
      await this.send(data);
      await this.stcs(CS_CTRLA, CTRLA_IBDLY);
    }
    // A run of bytes, each acknowledged (EEPROM pages).
    async writeBytes(address, data) {
      await this.setPointer(address);
      await this.repeat(data.length);
      await this.send([SYNC, ST | PTR_INC | DATA_8, data[0]]);
      await this.expectAck('a write');
      for (let i = 1; i < data.length; i++) {
        await this.send([data[i]]);
        await this.expectAck('a write');
      }
    }

    async initSession() {
      await this.stcs(CS_CTRLB, CTRLB_CCDETDIS);   // no collision detection
      await this.stcs(CS_CTRLA, CTRLA_IBDLY);      // inter-byte delay on the chip's replies
    }
    async linkUp() {
      try {
        await this.initSession();
        return (await this.ldcs(CS_STATUSA)) !== 0;
      } catch (err) {
        return false;
      }
    }

    // A UPDI BREAK resets the chip's UPDI interface into a known state.  It is a
    // line held low for longer than a frame: two zero bytes at 300 baud hold it
    // low for 33 ms each, above the 24.6 ms the datasheet asks for.
    async doubleBreak() {
      await this.t.close();
      await this.t.open(300, 1);
      await this.t.write(Uint8Array.from([0x00, 0x00]));
      try { await this.t.read(2, 500); } catch (err) { /* no echo needed here */ }
      await this.t.close();
      await this.t.open(BAUD, 2);
    }

    async connect() {
      await this.t.open(BAUD, 2);
      this.t.discardInput();
      // A first zero byte wakes the UPDI interface if it is disabled.  Its echo
      // comes from the flasher alone, chip or no chip: without one, the flasher
      // itself is the problem.
      let echoError = null;
      try { await this.send([0x00]); } catch (err) { if (!(err instanceof FlasherError)) throw err; echoError = err; }
      if (await this.linkUp()) return;
      this.log('No answer, sending a UPDI break');
      await this.doubleBreak();
      if (await this.linkUp()) return;
      if (echoError) throw echoError;
      throw new FlasherError('UPDI link initialization failed: the chip did not answer.',
        'Check the module is attached: 3V3, GND and UPDI from the flasher (FLASHING.md). ' +
        'A module that was only just plugged in may need a second try.');
    }

    async systemInfo() {
      await this.send([SYNC, KEY | KEY_SIB | SIB_16_BYTES]);
      const sib = await this.receive(16);
      return String.fromCharCode(...sib).replace(/[^\x20-\x7E]/g, '').trim();
    }

    // After a reset the chip spends its start-up time (SYSCFG1: 32 ms with
    // these fuses) loading its fuses and lock bits, and reads as locked until
    // then.  A key sent in that window is used up when the start-up ends, so
    // KEY_STATUS can read clear for a key that worked: wait for the chip to
    // have started before sending one or trusting its status.  A chip still
    // locked once it has had ample time is really locked.
    async waitStarted() {
      const deadline = Date.now() + 500;
      for (;;) {
        const status = await this.ldcs(ASI_SYS_STATUS);
        if (!(status & (SYS_STATUS_RSTSYS | SYS_STATUS_LOCKSTATUS))) return status;
        if (Date.now() > deadline) {
          if (status & SYS_STATUS_RSTSYS) {
            throw new FlasherError('The chip did not come out of reset.',
              'Check the module\'s 3V3: below 2.6 V the brown-out detector holds the chip in reset.');
          }
          return status;
        }
        await sleep(2);
      }
    }

    // The chip starts again in programming mode if the key went in just
    // before, and leaves programming mode if not.
    async reset() {
      await this.stcs(ASI_RESET_REQ, RESET_REQ_VALUE);
      await this.stcs(ASI_RESET_REQ, 0x00);
      return this.waitStarted();
    }

    async enterProgMode() {
      let status = await this.waitStarted();
      if (status & SYS_STATUS_LOCKSTATUS) throw lockedError();
      if (status & SYS_STATUS_NVMPROG) return;
      const key = KEY_NVMPROG.split('').reverse().map((ch) => ch.charCodeAt(0));
      await this.send([SYNC, KEY | KEY_64_BITS]);
      await this.send(key);
      if (!((await this.ldcs(ASI_KEY_STATUS)) & KEY_STATUS_NVMPROG)) {
        throw new FlasherError('The chip did not accept the programming key.');
      }
      status = await this.reset();
      if (status & SYS_STATUS_LOCKSTATUS) throw lockedError();
      if (!(status & SYS_STATUS_NVMPROG)) throw new FlasherError('The chip did not enter programming mode.');
    }

    // Reset and release UPDI: the chip starts the firmware it now holds.
    async leaveProgMode() {
      await this.stcs(ASI_RESET_REQ, RESET_REQ_VALUE);
      await this.stcs(ASI_RESET_REQ, 0x00);
      await this.stcs(CS_CTRLB, CTRLB_UPDIDIS | CTRLB_CCDETDIS);
    }

    // ── NVM controller ────────────────────────────────────────────────────
    async nvmReady(timeoutMs) {
      const deadline = Date.now() + timeoutMs;
      for (;;) {
        const status = await this.ld(CHIP.nvmctrl + NVM_STATUS);
        if (status & NVM_WRITE_ERROR) throw new FlasherError('The chip reported a write error.');
        if (!(status & (NVM_FLASH_BUSY | NVM_EEPROM_BUSY))) return;
        if (Date.now() > deadline) throw new FlasherError('The chip stayed busy writing.');
      }
    }
    async nvmCommand(command, timeoutMs) {
      await this.nvmReady(100);
      await this.st(CHIP.nvmctrl + NVM_CTRLA, command);
      await this.nvmReady(timeoutMs || 100);
    }

    // Erase and write in one command, rather than a chip erase first: a page
    // erase touches only the flash, so the EEPROM is safe whatever the fuses.
    async writeFlashPage(offset, data) {
      await this.nvmCommand(NVM_PAGE_BUFFER_CLEAR);
      await this.writeWords(CHIP.flashBase + offset, data);
      await this.nvmCommand(NVM_ERASE_WRITE_PAGE);
    }

    async writeEepromPage(offset, data) {
      await this.nvmCommand(NVM_PAGE_BUFFER_CLEAR);
      await this.writeBytes(CHIP.eepromBase + offset, data);
      await this.nvmCommand(NVM_ERASE_WRITE_PAGE);
    }

    async readFuses() { return this.readBytes(CHIP.fuseBase, FUSE_COUNT); }

    async writeFuse(number, value) {
      const address = CHIP.fuseBase + number;
      // SYSCFG0 decides what the UPDI pin is.  A value that made it a GPIO or
      // reset pin would lock this programmer, and avrdude, out of the chip for
      // good, and one without EESAVE would let the next erase wipe the EEPROM.
      // Checked here, on the address actually written, whatever the caller meant.
      if (address === CHIP.fuseBase + 5 && (value & 0x0D) !== 0x05) {
        throw new FlasherError(`Refusing to write SYSCFG0 = 0x${hex2(value)}: it must keep the UPDI pin and EESAVE.`);
      }
      await this.nvmReady(100);
      await this.st(CHIP.nvmctrl + NVM_ADDRL, address & 0xFF);
      await this.st(CHIP.nvmctrl + NVM_ADDRH, address >> 8);
      await this.st(CHIP.nvmctrl + NVM_DATAL, value);
      await this.st(CHIP.nvmctrl + NVM_CTRLA, NVM_WRITE_FUSE);
      await this.nvmReady(100);
    }
  }

  // ── Intel HEX ─────────────────────────────────────────────────────────────
  // The release's firmware.hex, as avr-objcopy writes it, into a flash image.
  function parseIntelHex(text, size) {
    const image = new Uint8Array(size).fill(0xFF);
    let base = 0, end = 0, eof = false, lineNo = 0;
    for (const raw of String(text).split(/\r?\n/)) {
      lineNo++;
      const line = raw.trim();
      if (!line) continue;
      if (eof) throw new FlasherError(`Firmware file: data after the end record (line ${lineNo}).`);
      if (line[0] !== ':' || line.length % 2 === 0 || !/^:[0-9A-Fa-f]+$/.test(line)) {
        throw new FlasherError(`Firmware file: line ${lineNo} is not Intel HEX.`);
      }
      const b = [];
      for (let i = 1; i < line.length; i += 2) b.push(parseInt(line.slice(i, i + 2), 16));
      const count = b[0], offset = (b[1] << 8) | b[2], type = b[3];
      if (b.length !== count + 5) throw new FlasherError(`Firmware file: line ${lineNo} has the wrong length.`);
      if (b.reduce((sum, x) => sum + x, 0) & 0xFF) throw new FlasherError(`Firmware file: line ${lineNo} has a bad checksum.`);
      if (type === 0x00) {
        for (let i = 0; i < count; i++) {
          const a = base + offset + i;
          if (a >= size) throw new FlasherError(`Firmware file: it does not fit the ${size}-byte flash.`);
          image[a] = b[4 + i];
          if (a + 1 > end) end = a + 1;
        }
      } else if (type === 0x01) {
        eof = true;
      } else if (type === 0x02) {
        base = ((b[4] << 8) | b[5]) << 4;
      } else if (type === 0x04) {
        base = ((b[4] << 8) | b[5]) << 16;
      } else if (type !== 0x03 && type !== 0x05) {
        throw new FlasherError(`Firmware file: unknown record type ${type} on line ${lineNo}.`);
      }
    }
    if (!eof) throw new FlasherError('Firmware file: it has no end record, so it may be incomplete.');
    if (!end) throw new FlasherError('Firmware file: it contains no data.');
    return { image, length: end };
  }

  // 256 bytes of EEPROM as an Intel HEX file avrdude can write back:
  //   avrdude -c serialupdi -p attiny1616 -P PORT -b 57600 -U eeprom:w:FILE:i
  function toIntelHex(bytes) {
    const lines = [];
    for (let at = 0; at < bytes.length; at += 16) {
      const chunk = Array.from(bytes.slice(at, at + 16));
      const record = [chunk.length, at >> 8, at & 0xFF, 0x00, ...chunk];
      const sum = (0x100 - (record.reduce((s, x) => s + x, 0) & 0xFF)) & 0xFF;
      lines.push(':' + record.concat([sum]).map(hex2).join(''));
    }
    lines.push(':00000001FF');
    return lines.join('\n') + '\n';
  }

  // ── The module's settings, from its EEPROM (layout in the firmware header) ─
  function describeEeprom(e) {
    const u16 = (i) => e[i] | (e[i + 1] << 8);
    const s16 = (i) => (u16(i) << 16) >> 16;
    let calibrated = 0;
    for (let i = 0; i < 64; i++) if (u16(12 + 2 * i) !== 0xFFFF) calibrated++;
    return {
      initialised: e[0] === 0x5D || e[0] === 0x5E,
      id: e[5],
      homeOffset: s16(1),
      totalSteps: s16(3),
      autoHome: e[6] === 1,
      bootCount: e[10],
      calibrated,
    };
  }

  function sameBytes(a, b) {
    if (a.length !== b.length) return false;
    for (let i = 0; i < a.length; i++) if (a[i] !== b[i]) return false;
    return true;
  }

  // ── The whole routine ─────────────────────────────────────────────────────
  // events: step(id, text), progress(done, total), backup(eeprom, settings,
  // serial), log(text).  Resolves with what happened; rejects with a
  // FlasherError (whose `result` holds what had happened by then).
  async function flashModule(transport, firmware, events) {
    const on = Object.assign({ step() {}, progress() {}, backup() {}, log() {} }, events);
    const updi = new Updi(transport, on.log);
    const result = { fusesWritten: [], eepromRestored: false };
    let inProg = false;
    try {
      on.step('connect', 'Connecting to the chip');
      await updi.connect();
      result.systemInfo = await updi.systemInfo();
      on.log(`System information: ${result.systemInfo}`);
      await updi.enterProgMode();
      inProg = true;

      on.step('identify', 'Identifying the chip');
      const signature = await updi.readBytes(CHIP.sigrow, 3);
      if (!sameBytes(signature, CHIP.signature)) {
        throw new FlasherError(`This is not an ${CHIP.name} (signature ${Array.from(signature).map(hex2).join(' ')}).`);
      }
      const serial = await updi.readBytes(CHIP.sigrow + CHIP.serialOffset, CHIP.serialLength);
      result.serial = Array.from(serial).map(hex2).join('');
      const lock = await updi.ld(CHIP.lockAddress);
      if (lock !== CHIP.unlocked) throw lockedError();

      on.step('backup', 'Backing up the module\'s settings');
      const eeprom = await updi.readBytes(CHIP.eepromBase, CHIP.eepromSize);
      result.eeprom = eeprom;
      result.settings = describeEeprom(eeprom);
      on.backup(eeprom, result.settings, result.serial);

      on.step('fuses', 'Checking the fuses');
      const before = await updi.readFuses();
      result.fusesBefore = Array.from(before);
      const fuses = fusesFor(firmware);
      const wrong = fuses.filter((f) => before[f.number] !== f.value);
      if (wrong.length) {
        let expected = Uint8Array.from(before);
        for (const f of wrong) {
          on.log(`Writing ${f.name} (fuse${f.number}): 0x${hex2(before[f.number])} → 0x${hex2(f.value)}`);
          await updi.writeFuse(f.number, f.value);
          expected[f.number] = f.value;
          // Read them all back: the right fuse must have changed, and no other.
          const now = await updi.readFuses();
          if (!sameBytes(now, expected)) {
            throw new FlasherError(`Writing ${f.name} did not give the expected fuses ` +
              `(${Array.from(now).map(hex2).join(' ')}); stopping before the flash is touched.`);
          }
          result.fusesWritten.push({ name: f.name, number: f.number, from: before[f.number], to: f.value });
        }
        // The new fuses take effect when the chip restarts at the end.  Nothing
        // here waits for them: a reset now would end programming mode (the chip
        // ignores the key while in it) and run the old firmware for a moment.
      }

      // Every page, so that the ones past the firmware end up blank, as after
      // a chip erase.
      on.step('write', 'Writing the firmware');
      const pages = CHIP.flashSize / CHIP.flashPage;
      for (let i = 0; i < pages; i++) {
        const at = i * CHIP.flashPage;
        await updi.writeFlashPage(at, firmware.image.subarray(at, at + CHIP.flashPage));
        on.progress(i + 1, pages);
      }

      on.step('verify', 'Verifying the firmware');
      const chunk = 256;
      for (let at = 0; at < CHIP.flashSize; at += chunk) {
        const back = await updi.readWords(CHIP.flashBase + at, chunk);
        for (let i = 0; i < chunk; i++) {
          if (back[i] !== firmware.image[at + i]) {
            throw new FlasherError(`Verification failed at flash byte 0x${(at + i).toString(16)}.`,
              'Try again; if it keeps failing, check the wiring and the module\'s power.');
          }
        }
        on.progress(at + chunk, CHIP.flashSize);
      }
      result.bytes = firmware.length;

      on.step('settings', 'Checking the module kept its settings');
      let now = await updi.readBytes(CHIP.eepromBase, CHIP.eepromSize);
      if (!sameBytes(now, eeprom)) {
        on.log('The EEPROM changed during flashing: writing the backup back');
        for (let at = 0; at < CHIP.eepromSize; at += CHIP.eepromPage) {
          const want = eeprom.subarray(at, at + CHIP.eepromPage);
          if (!sameBytes(now.subarray(at, at + CHIP.eepromPage), want)) await updi.writeEepromPage(at, want);
        }
        now = await updi.readBytes(CHIP.eepromBase, CHIP.eepromSize);
        if (!sameBytes(now, eeprom)) {
          throw new FlasherError('The module\'s settings could not be put back.',
            'Its backup was saved: write it back with avrdude (see the backup file\'s note).');
        }
        result.eepromRestored = true;
      }

      on.step('start', 'Starting the new firmware');
      await updi.leaveProgMode();
      inProg = false;
      result.ok = true;
      return result;
    } catch (err) {
      if (inProg) { try { await updi.leaveProgMode(); } catch (e) { /* the original error matters more */ } }
      const failure = err instanceof FlasherError ? err : new FlasherError(err && err.message ? err.message : String(err));
      failure.result = result;
      throw failure;
    }
  }

  const api = { CHIP, Updi, FlasherError, parseIntelHex, toIntelHex, describeEeprom, bootloaderVersion, fusesFor, flashModule };
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  else root.SplitFlapUpdi = api;
})(typeof window !== 'undefined' ? window : globalThis);
