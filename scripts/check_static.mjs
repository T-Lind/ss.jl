#!/usr/bin/env node
// Check shipped Julia-panel and static-browser assets, including inline JS.
import { readFileSync, readdirSync, existsSync } from 'node:fs';
import { execFileSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';
import { join, dirname, relative } from 'node:path';

const root = fileURLToPath(new URL('..', import.meta.url));
const issues = [];
let pages = 0, modules = 0, inline = 0;
const STATIC = /['"]\/static\/([A-Za-z0-9_-]+\.(?:js|css|geojson))['"]/g;
const IMPORT = /\b(?:from\s+|import\s*(?:\(\s*)?)['"]([^'"]+)['"]/g;

function references(source, file, tree) {
  const label = relative(root, file);
  for (const m of source.matchAll(STATIC))
    if (!existsSync(join(tree, 'static', m[1]))) issues.push(`${label}: missing /static/${m[1]}`);
  for (const m of source.matchAll(IMPORT)) {
    const ref = m[1].split(/[?#]/)[0];
    if (!ref.startsWith('.') && !ref.startsWith('/')) continue;
    const target = ref.startsWith('/') ? join(tree, ref) : join(dirname(file), ref);
    if (!existsSync(target)) issues.push(`${label}: missing import ${ref}`);
  }
}

function syntax(source, file, module = true) {
  try {
    execFileSync(process.execPath, [module ? '--input-type=module' : '--input-type=commonjs', '--check'],
      { input: source, stdio: ['pipe', 'pipe', 'pipe'] });
  } catch (err) {
    issues.push(`${relative(root, file)}: ${String(err.stderr || err.message).trim()}`);
  }
}

for (const name of ['scripts', 'web']) {
  const tree = join(root, name);
  const html = readdirSync(tree).filter(f => f.endsWith('.html'));
  for (const page of html) {
    const file = join(tree, page), source = readFileSync(file, 'utf8');
    references(source, file, tree);
    for (const match of source.matchAll(/<script\b([^>]*)>([\s\S]*?)<\/script\s*>/gi)) {
      const attrs = match[1], body = match[2];
      const type = /\btype\s*=\s*['"]([^'"]+)['"]/i.exec(attrs)?.[1];
      if (!body.trim() || (type && !['module', 'text/javascript', 'application/javascript'].includes(type))) continue;
      syntax(body, file, type === 'module'); inline++;
    }
  }
  pages += html.length;
  for (const folder of ['static', ...(name === 'web' ? ['src'] : [])]) {
    const dir = join(tree, folder);
    for (const entry of readdirSync(dir).filter(f => f.endsWith('.js'))) {
      const file = join(dir, entry), source = readFileSync(file, 'utf8');
      references(source, file, tree); syntax(source, file); modules++;
    }
  }
}

if (issues.length) {
  for (const issue of issues) console.error(issue);
  process.exit(1);
}
console.log(`static check ok: ${pages} pages, ${modules} modules, ${inline} inline scripts`);
