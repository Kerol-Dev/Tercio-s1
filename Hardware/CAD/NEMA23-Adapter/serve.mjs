// Local viewer for the NEMA 23 adapter:  node serve.mjs  →  http://localhost:8766
import fs from 'node:fs';
import http from 'node:http';
import path from 'node:path';

const root = path.dirname(new URL(import.meta.url).pathname);
const port = Number(process.env.PORT ?? 8766);
const types = { '.html': 'text/html', '.js': 'text/javascript', '.css': 'text/css', '.json': 'application/json', '.glb': 'model/gltf-binary', '.step': 'application/step' };

http.createServer((req, res) => {
  let url = decodeURIComponent(req.url.split('?')[0]);
  if (url === '/') return res.writeHead(302, { location: '/viewer/' }).end();
  if (url.endsWith('/')) url += 'index.html';
  const file = path.join(root, url);
  if (!file.startsWith(root) || file.includes('.venv')) return res.writeHead(403).end();
  fs.readFile(file, (err, data) => {
    if (err) return res.writeHead(404).end('not found');
    res.writeHead(200, { 'content-type': types[path.extname(file)] ?? 'application/octet-stream', 'cache-control': 'no-store' });
    res.end(data);
  });
}).listen(port, '127.0.0.1', () => console.log(`http://localhost:${port}`));
