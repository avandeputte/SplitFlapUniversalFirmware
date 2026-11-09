// =============================================================================
// Split-Flap module flasher: build the page for publishing
// =============================================================================
// Produces one self-contained HTML file with the firmware embedded, so it works
// from GitHub Pages and also as a downloaded file opened straight from disk.
//
//   node flasher/build.mjs <version> <firmware.hex>... [--out <dir>]
//
// One hex file per driver board: since v36 a release has firmware-v4.hex and
// firmware-v5.hex, and the page writes each module the one for its board.
// Each image names its board itself (0x3FFC), so the order and the file names
// do not matter.  --out defaults to flasher/dist; the page is written there as
// index.html.  Plain Node, no packages.  The firmware is checked with the
// page's own parser before anything is written.
// =============================================================================
import { readFileSync, writeFileSync, mkdirSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';
import { createRequire } from 'node:module';

const here = dirname(fileURLToPath(import.meta.url));
const args = process.argv.slice(2);
let outDir = join(here, 'dist');
const outAt = args.indexOf('--out');
if (outAt >= 0) { outDir = args[outAt + 1]; args.splice(outAt, 2); }
const [version, ...hexPaths] = args;
if (!version || !hexPaths.length || !outDir) {
  console.error('usage: node flasher/build.mjs <version> <firmware.hex>... [--out <dir>]');
  process.exit(2);
}

const { parseIntelHex, imageBoard, CHIP } = createRequire(import.meta.url)('./updi.js');
const hexes = [], boards = [];
for (const path of hexPaths) {
  const hex = readFileSync(path, 'utf8');
  const board = imageBoard(parseIntelHex(hex, CHIP.flashSize));
  if (boards.includes(board)) throw new Error(`${path}: a second image for the v${board} board`);
  hexes.push(hex);
  boards.push(board);
}

const firmwareJs = `window.BUNDLED_FIRMWARE = ${JSON.stringify({ version: String(version).replace(/^v/, ''), hexes })};`;
const sources = {
  'firmware.js': firmwareJs,
  'updi.js': readFileSync(join(here, 'updi.js'), 'utf8'),
  'webserial.js': readFileSync(join(here, 'webserial.js'), 'utf8'),
  'app.js': readFileSync(join(here, 'app.js'), 'utf8'),
};

let html = readFileSync(join(here, 'index.html'), 'utf8');
for (const [name, code] of Object.entries(sources)) {
  const tag = `<script src="${name}"></script>`;
  if (!html.includes(tag)) throw new Error(`index.html has no ${tag}`);
  if (/<\/script/i.test(code)) throw new Error(`${name} contains "</script", which would end the inline script`);
  html = html.replace(tag, () => `<script>\n${code}\n</script>`);
}

mkdirSync(outDir, { recursive: true });
const out = join(outDir, 'index.html');
writeFileSync(out, html);
console.log(`${out}: firmware v${version} for board ${boards.map((b) => `v${b}`).join(' and ')}, page ${html.length} bytes`);
