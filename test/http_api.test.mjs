import { test } from 'node:test';
import assert from 'node:assert/strict';
import { post } from '../scripts/static/api.js';

test('HTTP cancellation during reply parsing retains its cancellation meaning', async () => {
  const original = globalThis.fetch;
  const controller = new AbortController();
  let parsing;
  const started = new Promise(resolve => parsing = resolve);
  globalThis.fetch = async (_, { signal }) => ({
    status: 200,
    json: () => new Promise((_, reject) => {
      signal.addEventListener('abort', () => reject(new DOMException('aborted', 'AbortError')), { once: true });
      parsing();
    }),
  });
  try {
    const request = post('/api/run', '', { signal: controller.signal });
    await started;
    controller.abort();
    await assert.rejects(request, { name: 'AbortError',
      message: 'stopped waiting; computation continues in the background' });
  } finally { globalThis.fetch = original; }
});

test('HTTP parse and transport failures name the endpoint without duplicate prefixes', async () => {
  const original = globalThis.fetch;
  try {
    globalThis.fetch = async () => ({ status: 502, json: async () => { throw new SyntaxError('bad JSON'); } });
    await assert.rejects(post('/api/geometry', ''), { message: '/api/geometry: HTTP 502, unreadable reply' });
    globalThis.fetch = async () => { throw new TypeError('network unavailable'); };
    await assert.rejects(post('/api/geometry', ''), { message: '/api/geometry: network unavailable' });
  } finally { globalThis.fetch = original; }
});

test('HTTP progress continues until the complete reply has been consumed', async () => {
  const original = { fetch: globalThis.fetch, setTimeout: globalThis.setTimeout,
    setInterval: globalThis.setInterval, clearInterval: globalThis.clearInterval };
  let poll, finish, reading, cleared = 0;
  const started = new Promise(resolve => reading = resolve);
  globalThis.setTimeout = () => 1;
  globalThis.setInterval = callback => { poll = callback; return 2; };
  globalThis.clearInterval = () => cleared++;
  globalThis.fetch = async path => ({
    status: 200,
    json: path.startsWith('/api/progress')
      ? async () => ({ ok: true, stage: 'ascent' })
      : () => new Promise(resolve => { finish = resolve; reading(); }),
  });
  try {
    const progress = [];
    const request = post('/api/run', '', { onProgress: p => progress.push(p) });
    await started;
    assert.equal(cleared, 0);
    await poll();
    assert.equal(progress[0].stage, 'ascent');
    finish({ ok: true });
    assert.deepEqual(await request, { ok: true });
    assert.equal(cleared, 1);
  } finally { Object.assign(globalThis, original); }
});
