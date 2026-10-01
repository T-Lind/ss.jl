// Dev-only static file server for web/. ES modules cannot load over file://,
// so this is how you run the port locally until it is hosted somewhere static.
//
//   node web/serve.mjs [port]
import { createServer } from 'node:http';
import { readFile } from 'node:fs/promises';
import { fileURLToPath } from 'node:url';
import { dirname, join, normalize } from 'node:path';

const root = dirname(fileURLToPath(import.meta.url));
const port = Number(process.argv[2]) || 8099;
const TYPES = {
  '.html': 'text/html; charset=utf-8',
  '.js': 'text/javascript; charset=utf-8',
  '.mjs': 'text/javascript; charset=utf-8',
  '.css': 'text/css; charset=utf-8',
  '.json': 'application/json; charset=utf-8',
  '.geojson': 'application/geo+json; charset=utf-8',
};

// The Julia panel served extensionless routes (/launch, /build, /analysis) and
// mission control at /. Reproduce those so the pages' own links keep working.
const ROUTES = {
  '/': '/panel_page.html',
  '/panel': '/panel_page.html',
  '/launch': '/launch_page.html',
  '/build': '/build_page.html',
  '/models': '/models_page.html',
  '/analysis': '/analysis_page.html',
  '/home': '/index.html',
};

createServer(async (req, res) => {
  const url = decodeURIComponent((req.url || '/').split('?')[0]);
  const target = ROUTES[url] || (url === '/' ? '/index.html' : url);
  const rel = normalize(target).replace(/^(\.\.[/\\])+/, '');
  const file = join(root, rel);
  if (!file.startsWith(root)) { res.writeHead(403).end('forbidden'); return; }
  try {
    const body = await readFile(file);
    const ext = file.slice(file.lastIndexOf('.'));
    res.writeHead(200, { 'content-type': TYPES[ext] || 'application/octet-stream' });
    res.end(body);
  } catch {
    res.writeHead(404, { 'content-type': 'text/plain' }).end('not found');
  }
}).listen(port, () => console.log(`web/ on http://127.0.0.1:${port}`));
