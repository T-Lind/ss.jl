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
export async function post(path, body) {
  let r;
  try {
    r = await fetch(path, { method: 'POST', body });
  } catch (e) {
    throw new Error(`${path}: ${e.message || 'network error'}`);
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

export const run      = async body => expect(await post('/api/run', body), 'run');
export const geometry = async body => expect(await post('/api/geometry', body), 'geometry');
export const sweep    = async body => expect(await post('/api/sweep', body), 'sweep');
export const solve    = async body => expect(await post('/api/solve', body), 'solve');

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
