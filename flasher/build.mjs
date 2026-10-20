// =============================================================================
// Split-Flap module flasher: build the page for publishing
// =============================================================================
// Produces one self-contained HTML file with the firmware embedded, so it works
// from GitHub Pages and also as a downloaded file opened straight from disk.
//
//   node flasher/build.mjs <firmware.hex> <version> [outDir]
//
// outDir defaults to flasher/dist; the page is written there as index.html.
// Plain Node, no packages.  The firmware is checked with the page's own parser
// before anything is written.
// =============================================================================
import { readFileSync, writeFileSync, mkdirSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';
import { createRequire } from 'node:module';

const here = dirname(fileURLToPath(import.meta.url));
const [hexPath, version, outDir = join(here, 'dist')] = process.argv.slice(2);
if (!hexPath || !version) {
  console.error('usage: node flasher/build.mjs <firmware.hex> <version> [outDir]');
  process.exit(2);
}

const { parseIntelHex, CHIP } = createRequire(import.meta.url)('./updi.js');
const hex = readFileSync(hexPath, 'utf8');
const parsed = parseIntelHex(hex, CHIP.flashSize);

const firmwareJs = `window.BUNDLED_FIRMWARE = ${JSON.stringify({ version: String(version).replace(/^v/, ''), hex })};`;
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
console.log(`${out}: firmware v${version}, ${parsed.length} bytes, page ${html.length} bytes`);
