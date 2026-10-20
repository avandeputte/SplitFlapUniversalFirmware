// =============================================================================
// Split-Flap module flasher: the browser's serial port (Web Serial API)
// =============================================================================
// The transport updi.js programs through: open/close/write/read/discardInput
// over a port the user picked with navigator.serial.requestPort().  Chrome,
// Edge and Opera on desktop have Web Serial; Firefox and Safari do not.
// =============================================================================
(function (root) {
  'use strict';

  class WebSerialTransport {
    constructor(port) {
      this.port = port;
      this.reader = null;
      this.writer = null;
      this.pump = null;
      this.buffer = [];
      this.waiter = null;
      this.failure = null;
      this.isOpen = false;
    }

    static supported() { return typeof navigator !== 'undefined' && 'serial' in navigator; }

    // Must be called from a click: the browser shows its port picker.
    static async choose() { return new WebSerialTransport(await navigator.serial.requestPort()); }

    describe() {
      const info = this.port.getInfo ? this.port.getInfo() : {};
      if (!info.usbVendorId) return 'serial port';
      const vid = info.usbVendorId.toString(16).padStart(4, '0');
      const pid = (info.usbProductId || 0).toString(16).padStart(4, '0');
      const known = { '1a86': 'CH340', '0403': 'FTDI', '10c4': 'CP210x', '067b': 'PL2303' }[vid];
      return `${known ? known + ' ' : ''}USB serial (${vid}:${pid})`;
    }

    async open(baudRate, stopBits) {
      if (this.isOpen) await this.close();
      await this.port.open({ baudRate, dataBits: 8, parity: 'even', stopBits, flowControl: 'none', bufferSize: 4096 });
      this.isOpen = true;
      this.failure = null;
      this.buffer = [];
      this.writer = this.port.writable.getWriter();
      this.reader = this.port.readable.getReader();
      this.pump = this.readForever(this.reader);
    }

    // Everything the port receives lands in this.buffer; read() takes from it.
    async readForever(reader) {
      try {
        for (;;) {
          const { value, done } = await reader.read();
          if (done) break;
          if (value) { for (const b of value) this.buffer.push(b); this.wake(); }
        }
      } catch (err) {
        this.failure = err;
        this.wake();
      }
    }

    wake() { if (this.waiter) { const w = this.waiter; this.waiter = null; w(); } }

    async read(count, timeoutMs) {
      const deadline = Date.now() + timeoutMs;
      while (this.buffer.length < count) {
        if (this.failure) throw this.failure;
        const left = deadline - Date.now();
        if (left <= 0) throw new Error(`no reply within ${timeoutMs} ms`);
        await new Promise((resolve) => { this.waiter = resolve; setTimeout(resolve, left); });
      }
      return Uint8Array.from(this.buffer.splice(0, count));
    }

    discardInput() { this.buffer = []; }

    async write(bytes) { await this.writer.write(Uint8Array.from(bytes)); }

    async close() {
      if (!this.isOpen) return;
      this.isOpen = false;
      try { await this.reader.cancel(); } catch (err) { /* already gone */ }
      try { await this.pump; } catch (err) { /* already gone */ }
      try { this.reader.releaseLock(); } catch (err) { /* already gone */ }
      try { this.writer.releaseLock(); } catch (err) { /* already gone */ }
      await this.port.close();
    }
  }

  root.WebSerialTransport = WebSerialTransport;
})(typeof window !== 'undefined' ? window : globalThis);
