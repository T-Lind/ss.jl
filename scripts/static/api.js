// The panel server's HTTP surface, in one place.
//
// Every POST endpoint takes a URLSearchParams body and answers either
// {ok: true, ...} or {ok: false, error: "..."}. That unwrapping was repeated
// at six call sites across three pages, each with its own fallback message
// for the same failure.
//
// Transport and validation are kept separate on purpose. `post` only speaks
// HTTP; `expect` applies the ok-check. Most callers want both and use the
// named wrappers, but the builder has to test whether a newer edit superseded
// this request BETWEEN parsing and validating — folding the check into the
// transport would turn a superseded reply into a spurious error banner.

/** Fetch and parse, nothing more. Throws only on network or parse failure,
 *  with the path in the message — a bare "Failed to fetch" names no endpoint. */
export async function post(path, body, options = {}) {
  const u = new URLSearchParams(body || '');
  const onProgress = options && options.onProgress;
  const job = onProgress
    ? (globalThis.crypto && crypto.randomUUID
        ? crypto.randomUUID() : `j${Date.now()}-${Math.random().toString(36).slice(2)}`)
    : '';
  if (job) u.set('_job', job);
  let poll = null, stopped = false;
  const readProgress = async () => {
    if (stopped) return;
    try {
      const p = await (await fetch('/api/progress?job=' + encodeURIComponent(job))).json();
      if (!stopped && p && p.ok) onProgress(p);
    } catch (_) { /* the original request owns transport failure reporting */ }
  };
  if (job) {
    setTimeout(readProgress, 120);
    poll = setInterval(readProgress, 350);
  }
  let r;
  try {
    r = await fetch(path, { method: 'POST', body: u });
  } catch (e) {
    throw new Error(`${path}: ${e.message || 'network error'}`);
  } finally {
    stopped = true;
    if (poll) clearInterval(poll);
  }
  try {
    return await r.json();
  } catch (e) {
    throw new Error(`${path}: HTTP ${r.status}, unreadable reply`);
  }
}

/** Apply the {ok, error} contract, preferring the server's own wording. */
export function expect(j, what) {
  if (!j || !j.ok) throw new Error((j && j.error) || `${what} failed`);
  return j;
}

export const run      = async (body, options) => expect(await post('/api/run', body, options), 'run');
export const geometry = async (body, options) => expect(await post('/api/geometry', body, options), 'geometry');
export const sweep    = async (body, options) => expect(await post('/api/sweep', body, options), 'sweep');
export const solve    = async (body, options) => expect(await post('/api/solve', body, options), 'solve');

/** Stage and engine catalogues. GET, and deliberately tolerant: the forms
 *  render with empty option lists rather than not rendering at all. */
export async function catalogue() {
  try {
    return await (await fetch('/api/catalogue')).json();
  } catch (e) {
    return {};
  }
}

export async function health() {
  return await (await fetch('/api/health')).json();
}
