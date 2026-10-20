// =============================================================================
// Split-Flap module flasher: the page
// =============================================================================
// Wires index.html to the programming core (updi.js) and the browser's serial
// port (webserial.js).  The firmware comes bundled (firmware.js, written by
// build.mjs) or from a file the user picks.
// =============================================================================
(function () {
  'use strict';
  const U = window.SplitFlapUpdi;
  const W = window.WebSerialTransport;
  const $ = (id) => document.getElementById(id);

  const STEPS = [
    ['connect', 'Connecting to the chip'],
    ['identify', 'Identifying the chip'],
    ['backup', 'Backing up the module\'s settings'],
    ['fuses', 'Checking the fuses'],
    ['write', 'Writing the firmware'],
    ['verify', 'Verifying the firmware'],
    ['settings', 'Checking the module kept its settings'],
    ['start', 'Starting the new firmware'],
  ];
  const RESTORE_COMMAND = 'avrdude -c serialupdi -p attiny1616 -P PORT -b 57600 -U eeprom:w:FILE:i';

  let firmware = null;      // { label, parsed }
  let transport = null;
  let busy = false;

  function log(text) {
    const el = $('log');
    el.textContent += `[${new Date().toLocaleTimeString()}] ${text}\n`;
    el.scrollTop = el.scrollHeight;
  }

  function el(tag, attrs, children) {
    const node = document.createElement(tag);
    for (const [k, v] of Object.entries(attrs || {})) {
      if (k === 'text') node.textContent = v; else if (k === 'class') node.className = v; else node.setAttribute(k, v);
    }
    for (const c of children || []) node.append(c);
    return node;
  }

  function refresh() {
    $('connect').disabled = busy || !W.supported();
    $('flash').disabled = busy || !transport || !firmware;
  }

  // ── Firmware ──────────────────────────────────────────────────────────────
  function loadFirmware(label, text) {
    const parsed = U.parseIntelHex(text, U.CHIP.flashSize);
    firmware = { label, parsed };
    $('fw-summary').textContent =
      `${label}: ${parsed.length.toLocaleString()} bytes of the ${U.CHIP.name}'s ${U.CHIP.flashSize.toLocaleString()}.`;
    $('fw-summary').className = '';
    log(`Firmware: ${label}, ${parsed.length} bytes`);
    refresh();
  }

  if (window.BUNDLED_FIRMWARE) {
    try { loadFirmware(`Firmware v${window.BUNDLED_FIRMWARE.version} (included with this page)`, window.BUNDLED_FIRMWARE.hex); }
    catch (err) { $('fw-summary').textContent = `The included firmware is damaged: ${err.message}`; }
  } else {
    $('fw-summary').textContent = 'Choose the firmware.hex file from a release.';
  }

  $('fw-file').addEventListener('change', async (ev) => {
    const file = ev.target.files[0];
    if (!file) return;
    try {
      loadFirmware(file.name, await file.text());
    } catch (err) {
      firmware = null;
      $('fw-summary').textContent = `${file.name} can't be used: ${err.message}`;
      $('fw-summary').className = 'status';
      refresh();
    }
  });

  // ── Flasher ───────────────────────────────────────────────────────────────
  $('connect').addEventListener('click', async () => {
    try {
      const chosen = await W.choose();
      if (transport && transport !== chosen) { try { await transport.close(); } catch (err) { /* gone */ } }
      transport = chosen;
      $('port-status').textContent = `Connected: ${chosen.describe()}.`;
      log(`Flasher: ${chosen.describe()}`);
    } catch (err) {
      if (err && err.name === 'NotFoundError') return;          // picker closed without a choice
      $('port-status').textContent = `Couldn't open the flasher: ${err.message}`;
    }
    refresh();
  });

  // ── Backups ───────────────────────────────────────────────────────────────
  function backupFile(eeprom, settings, serial) {
    const d = new Date();
    const stamp = `${d.getFullYear()}${String(d.getMonth() + 1).padStart(2, '0')}${String(d.getDate()).padStart(2, '0')}` +
      `-${String(d.getHours()).padStart(2, '0')}${String(d.getMinutes()).padStart(2, '0')}`;
    const who = !settings.initialised ? 'blank' : settings.id === 255 ? 'unprovisioned' : `module-${settings.id}`;
    const name = `${who}-${serial}-${stamp}.eeprom.hex`;
    const url = URL.createObjectURL(new Blob([U.toIntelHex(eeprom)], { type: 'text/plain' }));
    return { name, url };
  }

  function download(file) {
    const a = el('a', { href: file.url, download: file.name });
    document.body.append(a);
    a.click();
    a.remove();
  }

  function moduleName(settings) {
    if (!settings) return 'Module';
    if (!settings.initialised) return 'Blank chip';
    return settings.id === 255 ? 'Unprovisioned module' : `Module ${settings.id}`;
  }

  // ── Flashing ──────────────────────────────────────────────────────────────
  function showSteps() {
    const list = $('steps');
    list.replaceChildren(...STEPS.map(([id, text]) =>
      el('li', { 'data-step': id }, [el('span', { class: 'icon', text: '○' }), el('span', { text })])));
    list.classList.remove('hidden');
  }
  function markStep(id, state) {
    const li = $('steps').querySelector(`[data-step="${id}"]`);
    if (!li) return;
    li.className = state;
    const icon = li.querySelector('.icon');
    icon.replaceChildren();
    if (state === 'active') icon.append(el('span', { class: 'spinner', 'aria-hidden': 'true' }));
    else icon.textContent = state === 'done' ? '✓' : state === 'failed' ? '✕' : '○';
  }

  function facts(result) {
    const s = result.settings;
    const rows = [];
    if (s && s.initialised) {
      rows.push(['Module ID', s.id === 255 ? 'not provisioned yet' : String(s.id)]);
      rows.push(['Home offset', `${s.homeOffset.toLocaleString()} steps`]);
      rows.push(['Steps per revolution', s.totalSteps.toLocaleString()]);
      rows.push(['Calibrated flaps', String(s.calibrated)]);
    } else if (s) {
      rows.push(['Settings', 'none yet: a blank chip']);
    }
    if (result.serial) rows.push(['Serial number', result.serial]);
    if (result.fusesBefore) {
      rows.push(['Fuses', result.fusesWritten.length
        ? `written: ${result.fusesWritten.map((f) => f.name).join(', ')}`
        : 'already correct']);
    }
    return el('dl', { class: 'facts' }, rows.flatMap(([k, v]) => [el('dt', { text: k }), el('dd', { text: v })]));
  }

  function addSessionRow(result, ok, backup) {
    $('session-card').classList.remove('hidden');
    const backupCell = backup ? el('a', { href: backup.url, download: backup.name, text: 'Download' }) : '—';
    $('session').prepend(el('tr', {}, [
      el('td', { text: moduleName(result.settings) }),
      el('td', { class: 'serial', text: result.serial || '—' }),
      el('td', { text: ok ? '✓ flashed' : '✕ failed' }),
      el('td', {}, [backupCell]),
    ]));
  }

  async function flash() {
    busy = true;
    refresh();
    showSteps();
    const outcome = $('outcome');
    outcome.classList.add('hidden');
    const bar = $('progress');
    bar.classList.add('hidden');
    let current = null;
    let backup = null;
    const started = Date.now();
    log('— Flashing —');
    try {
      const result = await U.flashModule(transport, firmware.parsed, {
        step(id) {
          if (current) markStep(current, 'done');
          current = id;
          markStep(id, 'active');
          bar.classList.toggle('hidden', id !== 'write' && id !== 'verify');
          bar.value = 0;
          log(STEPS.find(([s]) => s === id)[1]);
        },
        progress(done, total) { bar.max = total; bar.value = done; },
        backup(eeprom, settings, serial) {
          backup = backupFile(eeprom, settings, serial);
          log(`${moduleName(settings)}, serial ${serial}: settings backed up as ${backup.name}`);
          if ($('autosave').checked) download(backup);
        },
        log,
      });
      markStep(current, 'done');
      bar.classList.add('hidden');
      const secs = ((Date.now() - started) / 1000).toFixed(0);
      outcome.replaceChildren(
        el('div', { class: 'notice ok' }, [
          el('p', {}, [el('strong', { text: `${moduleName(result.settings)} is flashed and verified` }),
            ` (${result.bytes.toLocaleString()} bytes, ${secs} s)` +
            (result.eepromRestored ? '. Its settings changed during flashing and were written back from the backup.' :
              result.settings && result.settings.initialised ? ', and it kept its settings.' : '.')]),
        ]),
        facts(result),
        el('p', { class: 'small muted', style: 'margin-top:10px' }, [
          'Settings backup: ', el('a', { href: backup.url, download: backup.name, text: backup.name }),
          `. To put it back if ever needed: `, el('code', { text: RESTORE_COMMAND }), '.']),
        el('p', { class: 'status', style: 'margin-top:10px', text: 'Attach the next module, then click Flash next module.' }),
      );
      outcome.classList.remove('hidden');
      $('flash').textContent = 'Flash next module';
      addSessionRow(result, true, backup);
      log(`Done in ${secs} s`);
    } catch (err) {
      if (current) markStep(current, 'failed');
      bar.classList.add('hidden');
      const result = err.result || {};
      const parts = [el('p', {}, [el('strong', { text: err.message })])];
      if (err.hint) parts.push(el('p', { text: err.hint }));
      if (current === 'write' || current === 'verify') {
        parts.push(el('p', { text: 'The module has no working firmware until it is flashed again; its settings are not affected. Click Flash module to retry.' }));
      }
      if (backup) parts.push(el('p', {}, ['Its settings were backed up first: ', el('a', { href: backup.url, download: backup.name, text: backup.name }), '.']));
      outcome.replaceChildren(el('div', { class: 'notice bad' }, parts));
      outcome.classList.remove('hidden');
      if (result.settings || result.serial) addSessionRow(result, false, backup);
      log(`Failed: ${err.message}${err.hint ? ' — ' + err.hint : ''}`);
    } finally {
      busy = false;
      refresh();
    }
  }
  $('flash').addEventListener('click', flash);

  window.addEventListener('beforeunload', (ev) => {
    if (busy) { ev.preventDefault(); ev.returnValue = ''; }
  });

  if (!W.supported()) $('unsupported').classList.remove('hidden');
  refresh();
})();
