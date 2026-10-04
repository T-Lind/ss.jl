// Dev-only static file server for web/. ES modules cannot load over file://,
// so this is how you run the port locally until it is hosted somewhere static.
//
//   node web/serve.mjs [port]
import { createServer } from 'node:http';
import { readFile } from 'node:fs/promises';
import { fileURLToPath } from 'node:url';
import { dirname, relative, resolve, isAbsolute } from 'node:path';

const root = dirname(fileURLToPath(import.meta.url));
const TYPES = {
  '.html': 'text/html; charset=utf-8',
  '.js': 'text/javascript; charset=utf-8',
  '.mjs': 'text/javascript; charset=utf-8',
  '.css': 'text/css; charset=utf-8',
  '.json': 'application/json; charset=utf-8',
  '.jpg': 'image/jpeg',
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

export function createDevServer() {
  return createServer(async (req, res) => {
    if (!['GET', 'HEAD'].includes(req.method)) {
      res.writeHead(405, { allow: 'GET, HEAD' }).end('method not allowed'); return;
    }
    let url;
    try { url = decodeURIComponent((req.url || '/').split('?')[0]); }
    catch { res.writeHead(400).end('invalid URL encoding'); return; }
    const target = ROUTES[url] || (url === '/' ? '/index.html' : url);
    const file = resolve(root, '.' + target.replaceAll('\\', '/'));
    const rel = relative(root, file);
    if (rel === '..' || rel.startsWith('../') || rel.startsWith('..\\') || isAbsolute(rel)) {
      res.writeHead(403).end('forbidden'); return;
    }
    try {
      const body = await readFile(file);
      const ext = file.slice(file.lastIndexOf('.'));
      res.writeHead(200, { 'content-type': TYPES[ext] || 'application/octet-stream' });
      res.end(req.method === 'HEAD' ? undefined : body);
    } catch {
      res.writeHead(404, { 'content-type': 'text/plain' }).end('not found');
    }
  });
}

if (process.argv[1] && resolve(process.argv[1]) === fileURLToPath(import.meta.url)) {
  const port = Number(process.argv[2] ?? 8099);
  createDevServer().listen(port, '127.0.0.1', () => console.log(`web/ on http://127.0.0.1:${port}`));
}
