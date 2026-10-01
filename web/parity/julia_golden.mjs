// A missing optional runtime may skip parity. A broken reference must fail.
// spawnSync reports missing executables through error.code; nonzero exit,
// timeout, malformed JSON and buffer overflow are reference failures.
import { spawnSync } from 'node:child_process';
import { mkdtempSync, readFileSync, rmSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { join, dirname } from 'node:path';
import { fileURLToPath } from 'node:url';

const root = join(dirname(fileURLToPath(import.meta.url)), '..', '..');

export function juliaGolden(script, {
  julia = process.env.JULIA || 'julia',
  required = process.env.SSJL_REQUIRE_JULIA === '1',
  ...options
} = {}) {
  const dir = mkdtempSync(join(tmpdir(), 'ssjl-parity-'));
  try {
    const out = join(dir, 'golden.json');
    const r = spawnSync(julia, ['--project=' + root, script, out], {
      encoding: 'utf8', maxBuffer: 256 * 1024 * 1024, timeout: 1_200_000, ...options,
    });
    if (r.error?.code === 'ENOENT' && !required) return null;
    if (r.error || r.status !== 0) {
      throw new Error(`Julia reference failed for ${script}: ${r.error?.message || `exit ${r.status}, signal ${r.signal}`}\n${r.stderr || ''}`);
    }
    return JSON.parse(readFileSync(out, 'utf8'));
  } finally {
    rmSync(dir, { recursive: true, force: true });
  }
}
