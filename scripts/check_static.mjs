#!/usr/bin/env node
// PR-time guard for the browser layer.
//
// Two failures the Julia suite cannot see and the release build only
// discovers at tag time:
//
//   1. A page or module imports `/static/...` and the asset did not travel.
//      `build/build_app.jl` checks this too, but only inside the Windows
//      release job, so a typo merges and then fails when you cut a release.
//   2. A static module does not parse. Browser ES modules are never loaded by
//      any compiler or linter here; `node --check` parses them without running
//      them.
//
// Both checks are intentionally the same rules build_app.jl enforces, so a
// green run here predicts a green release build.
import { readFileSync, readdirSync, existsSync } from 'node:fs';
import { execFileSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';
import { join, basename } from 'node:path';

const scriptsDir = fileURLToPath(new URL('.', import.meta.url)); // .../scripts/
const staticDir = join(scriptsDir, 'static');

// Same whitelist shape as panelapp.jl's `static_asset` and build_app.jl.
const REF = /['"]\/static\/([A-Za-z0-9_-]+\.(?:js|css|geojson))['"]/g;

const html = readdirSync(scriptsDir).filter(f => f.endsWith('.html'));
const modules = readdirSync(staticDir).filter(f => f.endsWith('.js'));
const sources = [
  ...html.map(f => join(scriptsDir, f)),
  ...modules.map(f => join(staticDir, f)),
];

const missing = new Set();
for (const src of sources) {
  const text = readFileSync(src, 'utf8');
  for (const m of text.matchAll(REF))
    if (!existsSync(join(staticDir, m[1])))
      missing.add(`${basename(src)} imports /static/${m[1]}`);
}

const broken = [];
for (const f of modules) {
  try {
    execFileSync(process.execPath, ['--check', join(staticDir, f)], { stdio: 'pipe' });
  } catch (e) {
    const first = String(e.stderr || e.message).trim().split('\n').find(Boolean);
    broken.push(`${f}: ${first}`);
  }
}

if (missing.size || broken.length) {
  for (const m of missing) console.error(`missing asset: ${m}`);
  for (const b of broken) console.error(`syntax error: ${b}`);
  process.exit(1);
}
console.log(`static check ok: ${html.length} pages, ${modules.length} modules`);
