// =============================================================================
// Host tests of the bootloader: bootloader.c on a simulated chip (host_hal.h)
// =============================================================================
//   cc -std=c11 -O2 -Wall -DHOST -Ibootloader/test -o bltest bootloader/test/test_bootloader.c && ./bltest
//
// The gateway's side is a script: frames go onto the bus timeline with the
// gaps BUS-UPDATE.md asks for.  A run starts the chip from a snapshot and
// takes it through every reset until the timeline is used up.  A step that
// depends on an answer (resend the pages a status says are missing) replays
// the run with more frames added: the simulation is deterministic, so the
// chip answers the same way up to that point.
// =============================================================================
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../bootloader.c"

chip_t chip;
jmp_buf chip_stop;

static int failures, checks;
#define CHECK(cond, ...) do {                                                   \
    checks++;                                                                   \
    if (!(cond)) { failures++; printf("  FAIL line %d: ", __LINE__); printf(__VA_ARGS__); printf("\n"); } \
  } while (0)

#define PAGE_GAP          0.015            // after a P: the page write and read-back
#define STATUS_WAIT       0.150            // after an S: the reply
#define GO_WAIT           0.400            // after a G: the image check, and a restart
#define MODULE            27
#define FIRST             ((int)FIRST_APP_PAGE)
#define LAST              ((int)LAST_PAGE)

// ── images ───────────────────────────────────────────────────────────────────
typedef struct { uint8_t bytes[FLASH_SIZE]; uint16_t crc; } image_t;
static image_t old_fw, new_fw, other_fw;

static uint16_t crc_ref(const uint8_t *p, size_t n) {   // CRC-16/CCITT-FALSE, the textbook way
  uint16_t crc = 0xFFFF;
  while (n--) {
    crc ^= (uint16_t)*p++ << 8;
    for (int i = 0; i < 8; i++) crc = crc & 0x8000 ? (crc << 1) ^ 0x1021 : crc << 1;
  }
  return crc;
}

// A firmware image as the build makes it: code from APP_START, blank to the
// end, its CRC in the last two bytes (never 0xFFFF).
static void make_image(image_t *im, size_t len, unsigned seed) {
  memset(im->bytes, 0xFF, sizeof im->bytes);
  srand(seed);
  for (size_t i = 0; i < len; i++) im->bytes[APP_START + i] = rand();
  do {
    im->crc = crc_ref(im->bytes + APP_START, IMAGE_CRC_AT - APP_START);
    if (im->crc == NO_IMAGE) im->bytes[IMAGE_CRC_AT - 1] ^= 1;
  } while (im->crc == NO_IMAGE);
  im->bytes[IMAGE_CRC_AT] = im->crc >> 8;
  im->bytes[IMAGE_CRC_AT + 1] = im->crc;
}

static int flash_holds(const image_t *im) {             // the app section, CRC included
  return !memcmp(chip.flash + APP_START, im->bytes + APP_START, FLASH_SIZE - APP_START);
}

// ── the gateway's script ─────────────────────────────────────────────────────
static bus_byte_t timeline[400000];
static size_t tl_len;
static double gw_t;                        // when the gateway's line is next free
static double noise;                       // chance that a byte gets a bit flipped
static unsigned noise_seed;

static void gw_byte(uint8_t b) {
  if (noise > 0 && (double)rand_r(&noise_seed) / RAND_MAX < noise) b ^= 1 << (rand_r(&noise_seed) % 8);
  timeline[tl_len++] = (bus_byte_t){ gw_t, b };
  gw_t += 10 * BIT_TIME;
}
static void gw_hex(uint8_t b) { gw_byte("0123456789ABCDEF"[b >> 4]); gw_byte("0123456789ABCDEF"[b & 15]); }
static void gw_frame(const uint8_t *d, size_t n) {
  uint16_t crc = crc_ref(d, n);
  gw_byte('#');
  for (size_t i = 0; i < n; i++) gw_hex(d[i]);
  gw_hex(crc >> 8);
  gw_hex(crc);
  gw_byte('\n');
}
static void gw_wait(double t) { gw_t += t; }
static void gw_page(uint8_t to, const image_t *im, int page) {
  uint8_t d[69] = { to, 'P', im->crc >> 8, im->crc & 0xFF, page };
  memcpy(d + 5, im->bytes + page * PAGE_SIZE, PAGE_SIZE);
  gw_frame(d, sizeof d);
  gw_wait(PAGE_GAP);
}
static void gw_image(uint8_t to, const image_t *im) {
  for (int p = FIRST; p <= LAST; p++) gw_page(to, im, p);
}
static void gw_status(uint8_t to) { uint8_t d[2] = { to, 'S' }; gw_frame(d, 2); gw_wait(STATUS_WAIT); }
static void gw_go(uint8_t to) { uint8_t d[2] = { to, 'G' }; gw_frame(d, 2); gw_wait(GO_WAIT); }
static void gw_text(const char *s) { while (*s) gw_byte(*s++); }

// ── running the chip ─────────────────────────────────────────────────────────
static chip_t start;                       // the chip at the start of every run
static struct state start_state;
static int resets, app_starts;

static void new_chip(const image_t *installed, uint8_t id) {
  memset(&start, 0, sizeof start);
  memset(start.flash, 0xFF, sizeof start.flash);
  for (int i = 0; i < 0x400; i++) start.flash[i] = 0x5A;   // the bootloader itself
  if (installed) memcpy(start.flash + APP_START, installed->bytes + APP_START, FLASH_SIZE - APP_START);
  memset(start.eeprom, 0xFF, sizeof start.eeprom);
  start.eeprom[EEPROM_MODULE_ID] = id;
  start.rstfr = 0x01;                      // power-on
  start.request = 0x1234;                  // RAM is garbage at power-on
  memset(&start_state, 0xA5, sizeof start_state);
  tl_len = 0;
  gw_t = 0.2;
  noise = 0;
}

// Through every reset, until the firmware starts, the power goes, or the
// chip sits in update mode with nothing more on the bus (two watchdog resets
// in a row with nothing received).  Returns why it stopped.
static int simulate(void) {
  chip = start;
  state = start_state;
  chip.in = timeline;
  chip.in_len = tl_len;
  memset(chip.pagebuf, 0xFF, sizeof chip.pagebuf);
  chip.pagebuf_page = -1;
  resets = app_starts = 0;
  srand(1);
  int idle = 0;
  for (;;) {
    size_t before = chip.in_pos;
    chip.de = 0;
    chip.wdt_on = 0;
    memset(chip.pagebuf, 0xFF, sizeof chip.pagebuf);   // a reset clears the page buffer
    chip.pagebuf_page = -1;
    int r = setjmp(chip_stop);
    if (r == 0) boot();
    if (r == STOP_APP) { app_starts++; chip.rstfr = 0; return r; }   // the firmware clears the flags
    if (r != STOP_RESET && r != STOP_WATCHDOG) return r;
    chip.rstfr |= r == STOP_RESET ? RSTCTRL_SWRF_bm : 0x08;
    resets++;
    idle = r == STOP_WATCHDOG && chip.in_pos == before && chip.in_pos == chip.in_len ? idle + 1 : 0;
    if (idle == 2) return r;
  }
}

// The firmware asks for an update: magic in RAM, software reset.
static void requested(void) {
  start.request = REQUEST_MAGIC;
  start.rstfr |= RSTCTRL_SWRF_bm;
}

// ── the module's answers ─────────────────────────────────────────────────────
typedef struct { uint8_t id, s, version, reason, flags, image[2], pages[32]; } reply_t;
static double reply_at[64];                // when each reply decoded by replies() began

// The status replies the module sent, decoded and checked; returns how many.
static int replies(reply_t *out, int max) {
  int n = 0;
  size_t i = 0;
  while (i < chip.out_len) {
    if (chip.out[i].b != '#') { i++; continue; }
    uint8_t d[64];
    size_t len = 0, j = i + 1;
    while (j + 1 < chip.out_len && chip.out[j].b != '\n' && len < sizeof d) {
      unsigned v;
      char h[3] = { chip.out[j].b, chip.out[j + 1].b, 0 };
      if (sscanf(h, "%2X", &v) != 1) break;
      d[len++] = v;
      j += 2;
    }
    CHECK(j < chip.out_len && chip.out[j].b == '\n', "a reply is not terminated");
    CHECK(len == sizeof(reply_t) + 2, "a reply is %zu bytes", len);
    CHECK(crc_ref(d, len) == 0, "a reply fails its CRC");
    if (len == sizeof(reply_t) + 2 && n < max && n < 64) { reply_at[n] = chip.out[i].at; memcpy(&out[n++], d, sizeof(reply_t)); }
    i = j + 1;
  }
  return n;
}

static int pages_missing(const reply_t *r, int *list) {
  int n = 0;
  for (int p = FIRST; p <= LAST; p++) if (!(r->pages[p / 8] & (1 << (p % 8)))) list[n++] = p;
  return n;
}

// What every run must respect, whatever happened.
static void invariants(const char *what) {
  CHECK(chip.boot_writes == 0, "%s: %ld writes aimed at the boot section", what, chip.boot_writes);
  CHECK(chip.sent_without_de == 0, "%s: sent with the driver off", what);
}

// ── tests ────────────────────────────────────────────────────────────────────
static void test(const char *name) { printf("%s\n", name); }

static void boot_decisions(void) {
  test("boot: a valid image starts, with no delay beyond the CRC check");
  new_chip(&old_fw, MODULE);
  CHECK(simulate() == STOP_APP, "did not start the firmware");
  CHECK(chip.now < 0.15, "took %.3f s", chip.now);
  CHECK(chip.out_len == 0 && !chip.de, "touched the bus");

  test("boot: no image, a blank stored CRC, a wrong stored CRC, a corrupted byte: stay");
  const char *cases[] = { "blank", "stored CRC erased", "stored CRC wrong", "one byte of code flipped" };
  for (int c = 0; c < 4; c++) {
    new_chip(c ? &old_fw : NULL, MODULE);
    if (c == 1) start.flash[IMAGE_CRC_AT] = start.flash[IMAGE_CRC_AT + 1] = 0xFF;
    if (c == 2) start.flash[IMAGE_CRC_AT + 1] ^= 0x10;
    if (c == 3) start.flash[0x1234] ^= 0x01;
    gw_wait(1);
    gw_status(MODULE);
    int r = simulate();
    reply_t rep[2];
    CHECK(r == STOP_WATCHDOG && app_starts == 0, "%s: the firmware started", cases[c]);
    CHECK(replies(rep, 2) == 1 && rep[0].reason == REASON_NO_APP, "%s: no status, or the wrong reason", cases[c]);
  }

  test("boot: an update request stays, even with a valid image; the magic is cleared");
  new_chip(&old_fw, MODULE);
  requested();
  gw_status(MODULE);
  gw_go(MODULE);                           // nothing received: back to the firmware
  reply_t rep[2];
  CHECK(simulate() == STOP_APP && resets == 1, "G did not take it back to the firmware");
  int none[256];
  CHECK(replies(rep, 2) == 1 && rep[0].reason == REASON_REQUESTED && rep[0].version == BL_VERSION &&
        rep[0].s == 's' && rep[0].id == MODULE && rep[0].image[0] == 0xFF && rep[0].image[1] == 0xFF &&
        rep[0].flags == 0 && pages_missing(&rep[0], none) == LAST - FIRST + 1,
        "wrong status before any page (RAM was garbage)");
  CHECK(chip.request == 0, "the request magic was not cleared");

  test("boot: a software reset without the magic, or the magic without one: the firmware starts");
  new_chip(&old_fw, MODULE);
  start.rstfr = RSTCTRL_SWRF_bm;
  CHECK(simulate() == STOP_APP, "SWRF alone kept it in the bootloader");
  new_chip(&old_fw, MODULE);
  start.request = REQUEST_MAGIC;
  start.rstfr = 0x01;
  CHECK(simulate() == STOP_APP && chip.request == 0, "the magic alone kept it, or was not cleared");

  test("boot: an idle update request returns to the firmware after 8 s (watchdog)");
  new_chip(&old_fw, MODULE);
  requested();
  int r = simulate();
  CHECK(r == STOP_APP && resets == 1 && chip.now > 8 && chip.now < 8.5, "stopped with %d after %.2f s", r, chip.now);
}

// The whole procedure from BUS-UPDATE.md for one module, adapting to its
// answers: all pages, status, resend what is missing, ..., go.  Returns the
// number of passes it took, or 0 if the module never completed.
static int update_one(uint8_t to, const image_t *im) {
  gw_image(to, im);
  for (int pass = 1; pass <= 12; pass++) {
    double asked = gw_t;
    gw_status(to);
    simulate();
    reply_t rep[64];
    int n = replies(rep, 64);
    if (n == 0 || reply_at[n - 1] < asked) continue;   // the request or its answer got lost: ask again
    const reply_t *last = &rep[n - 1];
    if (last->image[0] != (im->crc >> 8) || last->image[1] != (im->crc & 0xFF)) {
      gw_image(to, im);                    // it has not seen this image yet: everything again
      continue;
    }
    int missing[256], m = pages_missing(last, missing);
    if (m == 0) { gw_go(to); return pass; }
    for (int i = 0; i < m; i++) gw_page(to, im, missing[i]);
  }
  return 0;
}

static void updates(void) {
  test("update: every page, a status, go: the new firmware starts");
  new_chip(&old_fw, MODULE);
  requested();
  CHECK(update_one(MODULE, &new_fw) == 1, "did not complete in one pass");
  int r = simulate();
  CHECK(r == STOP_APP && flash_holds(&new_fw) && resets == 1, "stopped with %d, image %s", r,
        flash_holds(&new_fw) ? "new" : "not new");
  CHECK(chip.lost == 0, "%zu bytes lost to busy time with the recommended gaps", chip.lost);
  invariants("update");
  printf("  %d pages in %.1f s of bus time\n", LAST - FIRST + 1, gw_t);

  test("update: addressed to every module (0xFF) works the same; S to 0xFF gets no answer");
  new_chip(&old_fw, MODULE);
  requested();
  gw_image(TO_ALL, &new_fw);
  gw_status(TO_ALL);
  gw_go(TO_ALL);
  r = simulate();
  reply_t rep[4];
  CHECK(r == STOP_APP && flash_holds(&new_fw), "broadcast update failed");
  CHECK(replies(rep, 4) == 0, "answered a broadcast status");

  test("update: an unprovisioned module (ID 0xFF) takes broadcasts, never answers");
  new_chip(&old_fw, 0xFF);
  requested();
  gw_image(TO_ALL, &new_fw);
  gw_status(TO_ALL);
  gw_go(TO_ALL);
  CHECK(simulate() == STOP_APP && flash_holds(&new_fw) && chip.out_len == 0, "failed or answered");

  test("update: frames for another module change nothing");
  new_chip(&old_fw, MODULE);
  requested();
  gw_image(MODULE + 1, &new_fw);
  gw_go(MODULE + 1);
  gw_go(MODULE);                           // nothing for us: back to the old firmware
  CHECK(simulate() == STOP_APP && flash_holds(&old_fw) && chip.writes == 0, "acted on another module's frames");

  test("update: a module with no firmware takes an image without being asked");
  new_chip(NULL, MODULE);
  CHECK(update_one(MODULE, &new_fw) == 1, "did not complete");
  CHECK(simulate() == STOP_APP && flash_holds(&new_fw), "did not start");
}

static void incomplete(void) {
  test("missing pages: G refuses, says so, and the resent pages complete it");
  new_chip(&old_fw, MODULE);
  requested();
  for (int p = FIRST; p <= LAST; p++) if (p != 100 && p != 255) gw_page(MODULE, &new_fw, p);
  gw_go(MODULE);
  gw_status(MODULE);
  simulate();
  reply_t rep[8];
  int missing[256];
  CHECK(replies(rep, 8) == 1 && rep[0].flags == FLAG_GO_FAILED, "G did not fail");
  CHECK(pages_missing(&rep[0], missing) == 2 && missing[0] == 100 && missing[1] == 255, "wrong pages missing");
  gw_page(MODULE, &new_fw, 100);
  gw_page(MODULE, &new_fw, 255);
  gw_go(MODULE);
  CHECK(simulate() == STOP_APP && flash_holds(&new_fw), "did not complete");

  test("missing pages: power lost before the end leaves it waiting, never running a mix");
  new_chip(&old_fw, MODULE);
  requested();
  for (int p = FIRST; p < 200; p++) gw_page(MODULE, &new_fw, p);
  start.lose_power_at = 120;               // power fails during page 120's write
  start.lose_power_how = 1;
  CHECK(simulate() == STOP_POWER, "the power did not fail");
  start = chip;
  start.in_pos = start.in_len = 0;
  start.rstfr = 0x01;
  start.lose_power_at = 0;
  tl_len = 0;
  gw_t = chip.now + 0.5;
  gw_status(MODULE);
  int r = simulate();
  CHECK(r == STOP_WATCHDOG && app_starts == 0 && replies(rep, 8) == 1 && rep[0].reason == REASON_NO_APP,
        "ran something after an interrupted update");

  test("a page that reads back wrong is not counted, and is resent");
  new_chip(&old_fw, MODULE);
  requested();
  start.bad_write_at = 50;
  CHECK(update_one(MODULE, &new_fw) == 2, "took other than two passes");
  CHECK(simulate() == STOP_APP && flash_holds(&new_fw), "did not complete");

  test("a new image part-way: the old pages no longer count, the new one completes");
  new_chip(&old_fw, MODULE);
  requested();
  for (int p = FIRST; p < 120; p++) gw_page(MODULE, &other_fw, p);
  CHECK(update_one(MODULE, &new_fw) == 1, "did not complete");
  CHECK(simulate() == STOP_APP && flash_holds(&new_fw), "did not start the new image");

  test("the same image again after an interruption: pages already written still count");
  new_chip(&old_fw, MODULE);
  requested();
  for (int p = FIRST; p < 120; p++) gw_page(MODULE, &new_fw, p);
  gw_status(MODULE);
  for (int p = 120; p <= LAST; p++) gw_page(MODULE, &new_fw, p);
  gw_go(MODULE);
  CHECK(simulate() == STOP_APP && flash_holds(&new_fw), "did not complete");
}

static void hostile_bus(void) {
  test("garbage between frames, bad hex, a bad CRC, half a byte, too long, a stray '#': all ignored");
  new_chip(&old_fw, MODULE);
  requested();
  gw_text("m27v\nhello\n#\n#1B53\n#1b530000\n#ZZ\n#1B5\n");
  uint8_t d[69] = { MODULE, 'P', new_fw.crc >> 8, new_fw.crc & 0xFF, 20 };
  memcpy(d + 5, new_fw.bytes + 20 * 64, 64);
  uint16_t crc = crc_ref(d, 69);
  gw_byte('#');                            // the right frame with its CRC off by one bit
  for (int i = 0; i < 69; i++) gw_hex(d[i]);
  gw_hex(crc >> 8);
  gw_hex((crc & 0xFF) ^ 1);
  gw_byte('\n');
  for (int i = 0; i < 80; i++) gw_hex(0x11);   // too long
  gw_byte('\n');
  gw_text("#1B");                          // cut short by a new frame
  gw_status(MODULE);
  int r = simulate();
  reply_t rep[4];
  CHECK(replies(rep, 4) == 1 && rep[0].image[0] == 0xFF, "acted on garbage, or did not answer after it");
  CHECK(chip.writes == 0, "wrote %ld pages", chip.writes);
  CHECK(r == STOP_APP && resets == 1, "stopped with %d after %d resets", r, resets);   // the watchdog, then the old firmware

  test("P aimed at the boot section, or carrying image CRC 0xFFFF: ignored");
  new_chip(&old_fw, MODULE);
  requested();
  for (int p = 0; p < FIRST; p++) gw_page(MODULE, &new_fw, p);
  image_t ffff = new_fw;
  ffff.crc = 0xFFFF;
  gw_page(MODULE, &ffff, 40);
  gw_go(MODULE);
  CHECK(simulate() == STOP_APP && flash_holds(&old_fw) && chip.writes == 0, "wrote something");
  invariants("boot section");

  test("garbage alone does not keep it awake: the watchdog still fires");
  new_chip(&old_fw, MODULE);
  requested();
  for (int i = 0; i < 2000; i++) { gw_text("#1B5300\n"); gw_wait(0.01); }
  int r2 = simulate();
  CHECK(r2 == STOP_APP && resets == 1 && chip.now < 9, "stopped with %d after %.1f s", r2, chip.now);
  invariants("garbage");

  test("frames too close together are lost, not misread");
  new_chip(&old_fw, MODULE);
  requested();
  for (int p = FIRST; p <= LAST; p++) { gw_page(MODULE, &new_fw, p); gw_t -= PAGE_GAP; }
  gw_wait(0.05);
  gw_status(MODULE);
  gw_go(MODULE);
  simulate();
  CHECK(chip.lost > 0 && app_starts == 0, "no gap and still complete?");
  invariants("no gaps");

  test("line noise: 1 byte in 3000 damaged (~5% of page frames), the procedure still completes");
  for (unsigned seed = 1; seed <= 20; seed++) {
    new_chip(&old_fw, MODULE);
    requested();
    noise = 1.0 / 3000;
    noise_seed = seed;
    int passes = update_one(MODULE, &new_fw);
    int r3 = simulate();
    CHECK(passes > 0 && r3 == STOP_APP && flash_holds(&new_fw), "seed %u: passes %d, stop %d", seed, passes, r3);
    invariants("noise");
  }
}

// The chip as power comes back after a loss: flash and EEPROM as they were,
// RAM garbage, the power-on flag, and a bus that is quiet until `quiet` s on.
static void power_back(const chip_t *lost, double quiet) {
  start = *lost;
  start.in_pos = start.in_len = 0;
  start.rstfr = 0x01;
  start.request = 0x1234;
  start.lose_power_at = 0;
  memset(&start_state, 0xA5, sizeof start_state);
  tl_len = 0;
  gw_t = lost->now + quiet;
}

static void power_loss(void) {
  test("power lost at every flash write of an update, before, after and half way through it");
  new_chip(&old_fw, MODULE);
  requested();
  update_one(MODULE, &new_fw);             // the script, built once
  chip_t base = start;
  size_t script = tl_len;
  double script_t = gw_t;
  static bus_byte_t saved[sizeof timeline / sizeof timeline[0]];
  memcpy(saved, timeline, script * sizeof *timeline);
  CHECK(simulate() == STOP_APP && flash_holds(&new_fw), "the undisturbed update failed");
  long total = chip.writes;
  int bad = 0, old_ran = 0, new_ran = 0, waited = 0;
  for (long w = 1; w <= total; w++) {
    for (int how = 0; how < 3; how++) {
      start = base;
      memset(&start_state, 0xA5, sizeof start_state);
      memcpy(timeline, saved, script * sizeof *timeline);
      tl_len = script;
      gw_t = script_t;
      start.lose_power_at = w;
      start.lose_power_how = how;
      int r = simulate();
      if (r != STOP_POWER) { bad++; printf("  write %ld/%d: the power did not fail (%d)\n", w, how, r); continue; }
      chip_t lost = chip;
      power_back(&lost, 1.0);              // nobody on the bus: what does it start?
      r = simulate();
      invariants("power back");
      if (r == STOP_APP) {
        if (flash_holds(&new_fw)) new_ran++;
        else if (flash_holds(&old_fw)) old_ran++;
        else { bad++; printf("  write %ld/%d: started an image that is neither\n", w, how); }
        continue;
      }
      waited++;
      power_back(&lost, 1.0);              // ...and a complete update from there works
      if (!update_one(MODULE, &new_fw) || simulate() != STOP_APP || !flash_holds(&new_fw)) {
        bad++;
        printf("  write %ld/%d: no recovery\n", w, how);
      }
      invariants("recovery");
    }
  }
  CHECK(bad == 0, "%d bad outcomes", bad);
  printf("  %ld writes x 3 ways: then the old firmware ran %d times, the new one %d, and %d waited for an update\n",
         total, old_ran, new_ran, waited);
}

int main(void) {
  make_image(&old_fw, 14000, 11);
  make_image(&new_fw, 14500, 22);
  make_image(&other_fw, 9000, 33);
  CHECK(crc_ref((const uint8_t *)"123456789", 9) == 0x29B1, "the reference CRC is not CCITT-FALSE");
  uint16_t c = 0xFFFF;
  for (const char *p = "123456789"; *p; p++) c = crc16(c, *p);
  CHECK(c == 0x29B1, "crc16() gives %04X for the check string", c);
  boot_decisions();
  updates();
  incomplete();
  hostile_bus();
  power_loss();
  printf("\n%d checks, %d failed\n", checks, failures);
  return failures != 0;
}
