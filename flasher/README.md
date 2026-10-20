# Web flasher

A browser page that flashes a module through the [SerialUPDI flasher](../FLASHER.md), with nothing installed. It works in Chrome, Edge and Opera on desktop, which have the Web Serial API; Firefox and Safari don't. Users open the published copy on GitHub Pages, or `flasher.html` from a release; see [FLASHING.md](../FLASHING.md).

## What it does

For each module it does what the avrdude instructions in FLASHING.md do, plus a backup, in this order:

1. Identify the chip: it must be an ATtiny1616, and unlocked.
2. Read the EEPROM, which holds the module's ID and calibration, and offer it as a backup file. That file can be written back with avrdude: `-U eeprom:w:FILE:i`.
3. Compare the fuses with the fixed set from FLASHING.md, and write only the ones that differ. After each write it reads every fuse back, and stops before touching the flash if the result isn't exactly what was expected. SYSCFG0 is written last, and the code refuses any value that would stop the UPDI pin working or clear EESAVE.
4. Write the firmware page by page, erasing each page as it goes, then read the whole flash back to verify it. avrdude starts with a chip erase instead, which takes the EEPROM with it unless EESAVE is in effect. A page erase never touches the EEPROM, so the settings are safe even on a chip whose new EESAVE fuse hasn't taken effect yet.
5. Read the EEPROM again. If it changed anyway, write the backup back.
6. Restart the chip: it starts the new firmware, with the new fuses.

New fuses take effect only at the restart in step 6. A reset any earlier would end programming mode, because the chip ignores the programming key while it is in that mode, and the old firmware would run for a moment.

## Files

| File | What it is |
|---|---|
| `updi.js` | The programming core: the UPDI protocol (as Microchip's pymcuprog and avrdude's `serialupdi` implement it), Intel HEX parsing, and the routine above. It has no browser dependencies, so it also runs under Node. |
| `webserial.js` | The serial port, through the Web Serial API. |
| `index.html`, `app.js` | The page. |
| `build.mjs` | Builds the published page: `node flasher/build.mjs <firmware.hex> <version>` writes `flasher/dist/index.html`, a single file with the scripts and the firmware inlined. It uses plain Node, with no packages. |

## Publishing

`.github/workflows/flasher.yml` runs on every published release. It:
- downloads the release's `firmware.hex`;
- builds the page;
- attaches it to the release as `flasher.html`;
- deploys it to GitHub Pages.

GitHub Pages has to be enabled once, under Settings → Pages → Source: "GitHub Actions". To rebuild from the latest release, use Actions → Web flasher → Run workflow.

## Trying a change locally

Build the page and open it straight from disk in Chrome or Edge:

```
node flasher/build.mjs .pio/build/ATtiny1616/firmware.hex 33
open flasher/dist/index.html
```

Chrome treats a page opened from disk as a secure context, so Web Serial works there. If yours doesn't, serve the folder instead: `python3 -m http.server -d flasher/dist`, then open `http://localhost:8000`.
