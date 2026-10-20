// =============================================================================
// Split-Flap module bootloader: firmware updates over the RS-485 bus
// =============================================================================
// Lives in the chip's 1 KB boot section (fuse BOOTEND = 0x04); the module
// firmware starts at 0x0400.  It can only be replaced over UPDI, so it does
// as little as possible and checks everything it does.  BUS-UPDATE.md
// describes the protocol and the gateway's side of an update; README.md in
// this folder, the build and the tests.
//
// At every reset:
//   - The firmware asked for an update (a software reset with REQUEST_MAGIC
//     left at REQUEST_AT in RAM): stay here, in update mode.
//   - Otherwise start the firmware if its image checks out: the CRC-16 of
//     APP_START..0x3FFD must equal the big-endian word at 0x3FFE, and that
//     word must not be 0xFFFF.
//   - Otherwise stay here, in update mode, until a good image arrives.
// The reset flags are left alone: the firmware's startup reads and clears
// them, and reports them.
//
// In update mode the bootloader listens on the bus (9600 8N1, bit-banged on
// the firmware's pins, uart.S) for frames of upper-case hex between '#' and
// '\n':  <to> <command> <payload...> <CRC-16, big-endian>
// The CRC is CRC-16/CCITT-FALSE, as in the firmware's checksummed frames, and
// <to> is the module ID from EEPROM, or 0xFF for every module.
//   P <image CRC:2> <page> <64 bytes>   write one flash page of an image
//   S                                   status, to one module only; reply:
//         <id> 's' <version> <reason> <flags> <image CRC:2> <pages:32>
//   G                                   go: start the image if it checks out,
//                                       or with no image, the old firmware
// The first P of an image (a new image CRC) erases the stored image CRC, so a
// module whose update stops part-way stays here.  Each page is read back
// after it is written, and only a page that reads back right counts.  G
// needs every page of the image counted, and the whole image to match its
// CRC, before it stores that CRC.  Eight seconds without a good frame reset
// the chip (watchdog), which then starts the firmware if it is valid.  The
// EEPROM is never written.
// =============================================================================
#include <stdint.h>

#define BL_VERSION        1
#define APP_START         0x0400u          // BOOTEND = 0x04
#define FLASH_SIZE        0x4000u
#define PAGE_SIZE         64u
#define FIRST_APP_PAGE    (APP_START / PAGE_SIZE)
#define LAST_PAGE         (FLASH_SIZE / PAGE_SIZE - 1)
#define IMAGE_CRC_AT      (FLASH_SIZE - 2)
#define NO_IMAGE          0xFFFFu
#define TO_ALL            0xFF
#define EEPROM_MODULE_ID  5                // ADDR_MODULE_ID in the firmware
#define REQUEST_MAGIC     0xB007u          // the firmware leaves this at REQUEST_AT...
#define REQUEST_AT        0x3F00u          // ...in RAM before its software reset
#define FRAME_MAX         71               // to, P, image CRC, page, 64 bytes, CRC

#define REASON_REQUESTED  1                // status reason: why it is in update mode
#define REASON_NO_APP     2
#define FLAG_GO_FAILED    1                // status flags: the last G did not start an image

#ifdef HOST
#include "host_hal.h"                      // the simulated chip, for the host tests
#define IN_Y(p)
#else
#include <avr/io.h>

#define DE_BIT            6                // PA6, driver enable, high = transmit
#define TX_BIT            5                // PA5, to the transceiver's DI
#define RX_BIT            7                // PA7, from its RO

// The flash appears in the data space; reads come from the flash and writes
// go to the NVM controller's page buffer.
#define FLASH_RD(a)       (*(volatile uint8_t *)(MAPPED_PROGMEM_START + (a)))
#define PAGEBUF_WR(a, v)  (*(volatile uint8_t *)(MAPPED_PROGMEM_START + (a)) = (v))
#define MODULE_ID()       (*(volatile uint8_t *)(MAPPED_EEPROM_START + EEPROM_MODULE_ID))
#define REQUEST           (*(volatile uint16_t *)REQUEST_AT)
#define RESET_FLAGS       RSTCTRL.RSTFR
#define WDR()             __asm__ __volatile__("wdr")
#define START_APP()       __asm__ __volatile__("jmp %0" :: "i"(APP_START))
#define SW_RESET()        do { _PROTECTED_WRITE(RSTCTRL.SWRR, RSTCTRL_SWRE_bm); for (;;) {} } while (0)
#define DRIVE_BUS(on)     do { if (on) VPORTA.OUT |= 1 << DE_BIT; else VPORTA.OUT &= ~(1 << DE_BIT); } while (0)
#define SET_UP_UPDATE_MODE() do {                                                  \
    VPORTA.OUT |= 1 << TX_BIT;                         /* idle high */             \
    VPORTA.DIR |= (1 << TX_BIT) | (1 << DE_BIT);       /* DE low: listening */     \
    PORTA.PIN7CTRL = PORT_PULLUPEN_bm;                 /* RX defined while the receiver is off */ \
    _PROTECTED_WRITE(WDT.CTRLA, WDT_PERIOD_8KCLK_gc);                              \
  } while (0)
// Keep a pointer in Y, so that the fields behind it are reached as Y+offset
// (2 bytes of code each) rather than by address (4 bytes).
#define IN_Y(p)           __asm__("" : "+y"(p))

uint8_t rx(void);                          // uart.S
void tx(uint8_t b);
void half_bit(void);

// Erase the page last written to the page buffer and write the buffer to it.
static void __attribute__((noinline)) flash_page(void) {
  _PROTECTED_WRITE_SPM(NVMCTRL.CTRLA, NVMCTRL_CMD_PAGEERASEWRITE_gc);
  while (NVMCTRL.STATUS & NVMCTRL_FBUSY_bm) {}
}
#endif

// No C runtime initialises this (there is none): update_mode() does.
struct state {
  struct {                                 // the reply to S, as sent
    uint8_t id, s, version, reason, flags;
    uint8_t image[2];                      // CRC of the image being received; FF FF if none
    uint8_t pages[(LAST_PAGE + 1) / 8];    // a bit per page written and read back right
  } status;
  uint8_t frame[FRAME_MAX];                // the frame just received
};
static struct state state;

static uint16_t __attribute__((noinline)) crc16(uint16_t crc, uint8_t b) {   // CRC-16/CCITT-FALSE
  crc ^= (uint16_t)b << 8;
  for (uint8_t i = 8; i; i--) crc = crc & 0x8000 ? (crc << 1) ^ 0x1021 : crc << 1;
  return crc;
}

static uint16_t __attribute__((noinline)) image_crc(void) {
  uint16_t crc = 0xFFFF;
  for (uint16_t a = APP_START; a != IMAGE_CRC_AT; a++) crc = crc16(crc, FLASH_RD(a));
  return crc;
}

// True if crc is an image CRC (NO_IMAGE is not) and the image in the flash has it.
static uint8_t __attribute__((noinline)) image_matches(uint16_t crc) {
  return crc != NO_IMAGE && image_crc() == crc;
}

static uint16_t stored_crc(void) {
  return ((uint16_t)FLASH_RD(IMAGE_CRC_AT) << 8) | FLASH_RD(IMAGE_CRC_AT + 1);
}

// Write n bytes from src to the start of a page, through the page buffer, and
// read them back.  Bytes of the page past n are erased (the page buffer is
// blank after every write).
static uint8_t __attribute__((noinline)) write_page(uint8_t page, const uint8_t *src, uint8_t n) {
  uint16_t a = (uint16_t)page * PAGE_SIZE;
  for (uint8_t i = 0; i < n; i++) PAGEBUF_WR(a + i, src[i]);
  flash_page();
  for (uint8_t i = 0; i < n; i++) if (FLASH_RD(a + i) != src[i]) return 0;
  return 1;
}

// Rewrite the last page with a new stored image CRC, keeping its other 62
// bytes: each is read from the flash and written to the page buffer at the
// same address.  Returns whether the CRC reads back right.
static uint8_t __attribute__((noinline)) store_crc(uint16_t crc) {
  for (uint16_t a = LAST_PAGE * PAGE_SIZE; a != IMAGE_CRC_AT; a++) PAGEBUF_WR(a, FLASH_RD(a));
  PAGEBUF_WR(IMAGE_CRC_AT, crc >> 8);
  PAGEBUF_WR(IMAGE_CRC_AT + 1, crc);
  flash_page();
  return stored_crc() == crc;
}

static void __attribute__((noinline)) tx_hex(uint8_t v) {
  v &= 15;
  tx(v < 10 ? '0' + v : 'A' - 10 + v);
}

static void __attribute__((noinline)) tx_byte(uint8_t b) {
  tx_hex(b >> 4);
  tx_hex(b);
}

// The next good frame into st->frame; returns its length, CRC included.  A
// frame that is not upper-case hex, has half a byte, is too long or fails its
// CRC is dropped; a '#' anywhere starts a new one.
static inline __attribute__((always_inline)) uint8_t receive_frame(struct state *st) {
  uint8_t c;
  do c = rx(); while (c != '#');
  for (;;) {
    uint8_t n = 0, odd = 0, hi = 0;
    uint16_t crc = 0xFFFF;
    for (;;) {
      c = rx();
      if (c == '#') break;
      if (c == '\n') {
        if (!odd && !crc) return n;        // each command checks its own length
        break;
      }
      uint8_t v = c - '0';
      if (v > 9) {
        v = c - 'A';
        if (v > 5) break;
        v += 10;
      }
      if (odd) {
        if (n == FRAME_MAX) break;
        v |= hi;
        st->frame[n++] = v;
        crc = crc16(crc, v);
      } else {
        hi = v << 4;
      }
      odd ^= 1;
    }
    while (c != '#') c = rx();             // a bad frame: wait for the next one
  }
}

static inline __attribute__((always_inline)) void send_status(struct state *st) {
  uint8_t *p = (uint8_t *)&st->status;
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 64; i; i--) half_bit();       // ~3 ms for the gateway to let go of the bus
  DRIVE_BUS(1);
  tx('#');
  for (uint8_t i = 0; i < sizeof st->status; i++) { crc = crc16(crc, p[i]); tx_byte(p[i]); }
  tx_byte(crc >> 8);
  tx_byte(crc);
  tx('\n');
  DRIVE_BUS(0);
}

// The image being received is now crc (hi, lo): none of its pages yet.
static void __attribute__((noinline)) start_image(struct state *st, uint8_t hi, uint8_t lo) {
  st->status.image[0] = hi;
  st->status.image[1] = lo;
  st->status.flags = 0;
  for (uint8_t i = 0; i < sizeof st->status.pages; i++) st->status.pages[i] = 0;
}

static void __attribute__((noreturn)) update_mode(uint8_t reason) {
  struct state *st = &state;
  IN_Y(st);
  SET_UP_UPDATE_MODE();
  st->status.s = 's';
  st->status.version = BL_VERSION;
  st->status.reason = reason;
  start_image(st, 0xFF, 0xFF);
  for (;;) {
    uint8_t n = receive_frame(st);
    WDR();
    uint8_t *f = st->frame, to = f[0], cmd = f[1];
    if (to != MODULE_ID() && to != TO_ALL) continue;

    if (cmd == 'P' && n == FRAME_MAX) {
      uint8_t page = f[4];
      if (page < FIRST_APP_PAGE || (f[2] & f[3]) == 0xFF) continue;   // FF FF is "no image"
      if (f[2] != st->status.image[0] || f[3] != st->status.image[1]) {   // a new image:
        start_image(st, f[2], f[3]);
        store_crc(NO_IMAGE);               // the stored one is no longer valid either
      }
      // The last page leaves its last two bytes blank: only G stores the CRC.
      uint8_t bit = 1 << (page & 7), *mark = &st->status.pages[page >> 3];
      *mark &= ~bit;
      if (write_page(page, f + 5, page == LAST_PAGE ? PAGE_SIZE - 2 : PAGE_SIZE)) *mark |= bit;

    } else if (cmd == 'S' && n == 4 && to != TO_ALL) {
      st->status.id = to;
      send_status(st);

    } else if (cmd == 'G' && n == 4) {
      // With no image received, back to the firmware already there.  With
      // one, every page must have been written and read back, and the whole
      // image must match its CRC, before the CRC is stored.
      uint16_t crc = ((uint16_t)st->status.image[0] << 8) | st->status.image[1];
      uint8_t all = 0xFF;
      for (uint8_t i = FIRST_APP_PAGE / 8; i < sizeof st->status.pages; i++) all &= st->status.pages[i];
      if (crc == NO_IMAGE ? image_matches(stored_crc())
                          : all == 0xFF && image_matches(crc) && store_crc(crc)) SW_RESET();
      st->status.flags |= FLAG_GO_FAILED;
    }
  }
}

// What every reset comes to.
static void __attribute__((noreturn)) boot(void) {
  uint8_t requested = (RESET_FLAGS & RSTCTRL_SWRF_bm) && REQUEST == REQUEST_MAGIC;
  REQUEST = 0;
  if (requested) update_mode(REASON_REQUESTED);
  if (image_matches(stored_crc())) START_APP();
  update_mode(REASON_NO_APP);
}

#ifndef HOST
// Identifies the bootloader in an image, e.g. for the web flasher to set
// BOOTEND, and for the firmware to report: 'S','F','B', version.
const uint8_t bl_signature[4] __attribute__((section(".blsig"), used)) = { 'S', 'F', 'B', BL_VERSION };

int main(void) __attribute__((OS_main, section(".init9")));
int main(void) {
  __asm__ __volatile__("cli\n\tclr __zero_reg__");
  _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, CLKCTRL_PDIV_2X_gc | CLKCTRL_PEN_bm);   // 10 MHz
  boot();
}
#endif
