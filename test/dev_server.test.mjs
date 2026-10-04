import { test } from 'node:test';
import assert from 'node:assert/strict';
import { request } from 'node:http';
import { once } from 'node:events';
import { createDevServer } from '../web/serve.mjs';

test('static server survives malformed URLs and confines files to the web directory', async () => {
  const server = createDevServer().listen(0, '127.0.0.1');
  await once(server, 'listening');
  const port = server.address().port;
  const get = (path, method = 'GET') => new Promise((resolve, reject) => {
    const req = request({ hostname: '127.0.0.1', port, path, method }, res => {
      const chunks = [];
      res.on('data', data => chunks.push(data));
      res.on('end', () => resolve({ status: res.statusCode, body: Buffer.concat(chunks).toString() }));
    });
    req.on('error', reject); req.end();
  });
  try {
    assert.equal((await get('/static/%ZZ')).status, 400);
    assert.equal((await get('/%2e%2e/README.md')).status, 403);
    assert.equal((await get('/%2e%2e%5cREADME.md')).status, 403);
    const page = await get('/');
    assert.equal(page.status, 200);
    assert.match(page.body, /mission control/);
    for (const path of ['/build', '/models', '/analysis', '/launch'])
      assert.equal((await get(path)).status, 200, path);
    assert.equal((await get('/static/ui.js', 'HEAD')).body, '');
    assert.equal((await get('/', 'POST')).status, 405);
  } finally {
    await new Promise(resolve => server.close(resolve));
  }
});
