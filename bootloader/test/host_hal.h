// =============================================================================
// The simulated chip the host tests run bootloader.c on
// =============================================================================
// bootloader.c compiled with -DHOST includes this instead of its AVR
// definitions.  It models what the bootloader relies on:
//   - the flash, written only through the page buffer, a page at a time, and
//     never in the boot section (the hardware refuses; counted in boot_writes);
//   - EEPROM, the RAM word the firmware leaves its request in, and the reset
//     flags, which add up across resets until the firmware clears them;
//   - the bus as a timeline of bytes: a byte that starts while the bootloader
//     is busy (writing a page, working out a CRC) is lost, as on the chip;
//   - the watchdog, eight seconds after the last good frame in update mode;
//   - faults: power lost before, after or in the middle of a chosen flash
//     write, and a write that reads back wrong.
// Time is simulated: the costs below are the chip's, to within a few percent.
// test_bootloader.c includes bootloader.c.
// =============================================================================
#include <setjmp.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define RSTCTRL_SWRF_bm   0x10
#define BIT_TIME          (1.0 / 9600)
#define PAGE_WRITE_TIME   0.004            // erase and write, CPU halted
#define FLASH_READ_TIME   8e-6             // a byte of image_crc: ~80 cycles at 10 MHz
#define WATCHDOG_TIME     8.0

typedef struct { double at; uint8_t b; } bus_byte_t;

typedef struct {
  uint8_t flash[0x4000];
  uint8_t pagebuf[64];
  int pagebuf_page;                        // -1: nothing in the page buffer
  uint8_t eeprom[256];
  uint16_t request;                        // RAM at REQUEST_AT
  uint8_t rstfr;
  bus_byte_t *in;                          // what the gateway sends, in time order
  size_t in_len, in_pos, lost;
  bus_byte_t out[8192];                    // what the module sends
  size_t out_len;
  int de, wdt_on, sent_without_de;
  double now, last_wdr;
  long writes;                             // flash_page calls so far
  long lose_power_at, bad_write_at;        // ...which one fails, 0 for none
  int lose_power_how;                      // 0 before the write, 1 after, 2 halfway: garbage page
  long boot_writes;                        // attempts at the boot section: must stay 0
} chip_t;

extern chip_t chip;
extern jmp_buf chip_stop;
enum { STOP_APP = 1, STOP_RESET, STOP_WATCHDOG, STOP_IDLE, STOP_POWER };

#define FLASH_RD(a)          host_flash_rd(a)
#define PAGEBUF_WR(a, v)     host_pagebuf_wr((a), (v))
#define MODULE_ID()          (chip.eeprom[EEPROM_MODULE_ID])
#define REQUEST              (chip.request)
#define RESET_FLAGS          (chip.rstfr)
#define WDR()                (chip.last_wdr = chip.now)
#define START_APP()          longjmp(chip_stop, STOP_APP)
#define SW_RESET()           longjmp(chip_stop, STOP_RESET)
#define DRIVE_BUS(on)        (chip.de = (on))
#define SET_UP_UPDATE_MODE() (chip.wdt_on = 1, chip.last_wdr = chip.now)

static uint8_t host_flash_rd(uint16_t a) {
  chip.now += FLASH_READ_TIME;
  return chip.flash[a & 0x3FFF];
}

// A page buffer write ANDs into the buffer, so writing a byte twice before a
// page write would show up as a wrong page.
static void host_pagebuf_wr(uint16_t a, uint8_t v) {
  chip.pagebuf[a % 64] &= v;
  chip.pagebuf_page = (a & 0x3FFF) / 64;
}

static void flash_page(void) {
  int page = chip.pagebuf_page, power = ++chip.writes == chip.lose_power_at;
  if (power && chip.lose_power_how == 0) longjmp(chip_stop, STOP_POWER);
  if (page >= 0 && page < 0x400 / 64) {
    chip.boot_writes++;
  } else if (page >= 0) {
    memcpy(&chip.flash[page * 64], chip.pagebuf, 64);
    if (chip.writes == chip.bad_write_at) chip.flash[page * 64 + 17] ^= 0x04;
    if (power && chip.lose_power_how == 2) for (int i = 0; i < 64; i++) chip.flash[page * 64 + i] = rand();
  }
  memset(chip.pagebuf, 0xFF, sizeof chip.pagebuf);
  chip.pagebuf_page = -1;
  chip.now += PAGE_WRITE_TIME;
  if (power) longjmp(chip_stop, STOP_POWER);
}

static void half_bit(void) { chip.now += BIT_TIME / 2; }

static uint8_t rx(void) {
  for (;;) {
    if (chip.in_pos == chip.in_len) {      // nothing more is coming
      if (chip.wdt_on) { chip.now = chip.last_wdr + WATCHDOG_TIME; longjmp(chip_stop, STOP_WATCHDOG); }
      longjmp(chip_stop, STOP_IDLE);
    }
    bus_byte_t *b = &chip.in[chip.in_pos];
    if (chip.wdt_on && b->at > chip.last_wdr + WATCHDOG_TIME) {
      chip.now = chip.last_wdr + WATCHDOG_TIME;
      longjmp(chip_stop, STOP_WATCHDOG);
    }
    chip.in_pos++;
    if (b->at < chip.now) { chip.lost++; continue; }   // it started while the bootloader was busy
    chip.now = b->at + 9.5 * BIT_TIME;     // rx returns once the stop bit has begun
    return b->b;
  }
}

static void tx(uint8_t b) {
  if (!chip.de) chip.sent_without_de++;
  if (chip.out_len < sizeof chip.out / sizeof chip.out[0]) chip.out[chip.out_len++] = (bus_byte_t){ chip.now, b };
  chip.now += 10 * BIT_TIME;
}
