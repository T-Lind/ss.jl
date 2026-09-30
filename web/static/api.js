// The panel server's HTTP surface, in process.
//
// The pages were written against the Julia panel's endpoints. This module is a
// drop-in replacement that runs the same payload builders — now ES modules in
// web/src/ — directly in the browser, so the whole app works as a static site.
// The signatures and the {ok, error} contract are unchanged; `post` only
// differs in that there is no network underneath it.
import { panelRun, panelGeometry, panelSweep, panelSolve,
         panelCatalogue } from '../src/panel.js';

/** Every run this page has flown, newest first. Kept in memory, and mirrored
 *  into sessionStorage so a run survives the navigation to /launch or
 *  /analysis — the server's store spanned requests, and this is the same
 *  session. A run too big for the quota simply stays in memory. */
const RUNS = new Map();
const RUN_ORDER = [];
const MAX_RUNS = 12;
const STORE_KEY = 'ssjl.runs';
let RUN_SEQ = 0;

function loadStore() {
  try {
    const data = JSON.parse(sessionStorage.getItem(STORE_KEY) || 'null');
    if (!data) return;
    for (const r of data.runs || []) RUNS.set(r.id, r);
    RUN_ORDER.push(...(data.order || []));
    RUN_SEQ = data.seq || RUN_ORDER.length;
  } catch (e) { /* no storage, or a corrupt entry: start empty */ }
}

function saveStore() {
  try {
    const runs = RUN_ORDER.filter(id => RUNS.has(id)).map(id => RUNS.get(id));
    sessionStorage.setItem(STORE_KEY, JSON.stringify({ runs, order: RUN_ORDER, seq: RUN_SEQ }));
  } catch (e) { /* over quota: the in-memory store still works this page */ }
}

loadStore();

function rememberRun(payload, p) {
  RUN_SEQ += 1;
  const id = 'r' + RUN_SEQ;
  payload.id = id;
  payload.at = Date.now() / 1000;
  const params = {};
  for (const [k, v] of Object.entries(p))
    if (!String(k).startsWith('_')) params[String(k)] = String(v);
  payload.params = params;
  RUNS.set(id, payload);
  RUN_ORDER.unshift(id);
  while (RUN_ORDER.length > MAX_RUNS) RUNS.delete(RUN_ORDER.pop());
  saveStore();
  return payload;
}

function runDigest(payload) {
  const p = payload.params || {};
  return {
    id: payload.id, at: payload.at, mode: p.mode ?? 'flyby',
    vname: p.vname ?? 'vehicle', outcome: payload.outcome ?? '?',
    metrics: payload.metrics ?? {}, params: p,
  };
}

/** The history list, newest first. */
export function runsList() {
  return { ok: true, runs: RUN_ORDER.filter(id => RUNS.has(id)).map(id => runDigest(RUNS.get(id))) };
}

/** One stored run, whole, or the same explanatory miss the server gave. */
export function runsGet(id) {
  const payload = RUNS.get(id);
  if (!payload) return { ok: false, error: `no run "${id}" — history keeps the last ` +
    `${MAX_RUNS} runs of this session, and starts empty each time the app opens` };
  return payload;
}

const paramsFrom = body => {
  const p = {};
  for (const [k, v] of new URLSearchParams(body || '')) p[k] = v;
  return p;
};

// A mission is seconds of synchronous simulation. Run on the main thread it
// blocks everything, so the busy overlay never paints and the elapsed clock
// never ticks: the page freezes, then completes, which is exactly the "the run
// pop-up is gone" report. The worker keeps the main thread free, so the
// overlay animates and the stage updates arrive while the flight is computed.
let worker = null, jobSeq = 0;
const pending = new Map();

function ensureWorker() {
  if (worker) return worker;
  if (typeof Worker === 'undefined') return null;
  try {
    worker = new Worker(new URL('./runworker.js', import.meta.url), { type: 'module' });
  } catch (e) { worker = null; return null; }
  worker.onmessage = e => {
    const { id, progress, result, error } = e.data || {};
    const job = pending.get(id);
    if (!job) return;
    if (progress) { if (job.onProgress) job.onProgress(progress); return; }
    pending.delete(id);
    if (error !== undefined) job.reject(new Error(error));
    else job.resolve(result);
  };
  worker.onerror = e => {
    const err = new Error(e.message || 'the simulation worker stopped');
    for (const [, job] of pending) job.reject(err);
    pending.clear();
    worker = null;
  };
  return worker;
}

function inWorker(path, p, onProgress) {
  const w = ensureWorker();
  if (!w) return null;
  jobSeq += 1;
  const id = 'w' + jobSeq;
  return new Promise((resolve, reject) => {
    pending.set(id, { resolve, reject, onProgress });
    w.postMessage({ id, path, params: p });
  });
}

/** Run the matching payload builder and answer {ok, ...}. Throws only for a
 *  route that does not exist; a mission that cannot be flown comes back as
 *  {ok: false, error}, exactly as the server's safe_call did. */
export async function post(path, body, options = {}) {
  const p = paramsFrom(body);
  const onProgress = options && options.onProgress;
  const job = inWorker(path, p, onProgress);
  if (job) {
    let out;
    try { out = await job; }
    catch (err) { return { ok: false, error: String(err && err.message ? err.message : err) }; }
    return path === '/api/run' && out && out.ok === true ? rememberRun(out, p) : out;
  }
  // No worker at all (an engine without module workers, or opened over
  // file://): fall back to the old synchronous, in-process path.
  try {
    if (path === '/api/run') {
      const out = panelRun(p);
      const result = out.ok === true ? rememberRun(out, p) : out;
      if (onProgress)
        onProgress({ ok: true, stage: 'complete', detail: 'trajectory ready',
                     current: 4, total: 4, done: true });
      return result;
    }
    if (path === '/api/geometry') return panelGeometry(p);
    if (path === '/api/sweep') return panelSweep(p, onProgress);
    if (path === '/api/solve') return panelSolve(p, onProgress);
  } catch (err) {
    return { ok: false, error: String(err && err.message ? err.message : err) };
  }
  return { ok: false, error: `no route for POST ${path}` };
}

/** Apply the {ok, error} contract, preferring the builder's own wording. */
export function expect(j, what) {
  if (!j || !j.ok) throw new Error((j && j.error) || `${what} failed`);
  return j;
}

export const run      = async (body, options) => expect(await post('/api/run', body, options), 'run');
export const geometry = async (body, options) => expect(await post('/api/geometry', body, options), 'geometry');
export const sweep    = async (body, options) => expect(await post('/api/sweep', body, options), 'sweep');
export const solve    = async (body, options) => expect(await post('/api/solve', body, options), 'solve');

/** Stage and engine catalogues. Deliberately tolerant: the forms render with
 *  empty option lists rather than not rendering at all. */
export async function catalogue() {
  try {
    return panelCatalogue();
  } catch (e) {
    return {};
  }
}

export async function health() {
  return { ok: true, uptime_s: 0, julia: 'browser', threads: 1 };
}
