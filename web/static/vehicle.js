// The wire between the builder and mission control.
//
// A vehicle is EDITED on /build and FLOWN from /. localStorage carries it:
// the builder writes the whole vehicle as a query string on every accepted
// edit, and the panel — in any other tab — applies it on the storage event,
// so the mission page's geometry view follows the builder live.
//
// Only the vehicle's own keys travel. Echoing mission fields (mode, target
// altitude, epoch) back through this channel would resurrect stale ones on
// the next load, and the failure is silent: you would fly a mission you had
// already reconfigured. `own` is the single definition of that boundary;
// both pages read it from here rather than each keeping a copy to drift.

export const KEY = 'ssjl.vehicle';

/** True for the keys that describe the vehicle itself, as opposed to the
 *  mission being flown with it. Stage fields are s<n>_*, boosters b_*,
 *  lander l_*. */
export const own = k =>
  k === 'nstages' || k === 'nboost' || k === 'vname' ||
  k === 'diameter' || k === 'fairing' || k === 'fairing_on' ||
  k === 'pod_mass' || k === 'pod_dia' || k === 'crewed' ||
  // the payload is spacecraft + cargo, and all three keys describe the thing
  // being flown rather than the flight — leave any of them out and the
  // builder's payload silently reverts on the next page load
  k === 'payload_kind' || k === 'bus_mass' || k === 'cargo_mass' ||
  k === 'kick_deg' || k === 'opt_kick' ||
  /^l_/.test(k) || /^s\d+_/.test(k) || /^b_/.test(k);

/** Narrow a full form to just the vehicle keys. */
export function pack(params) {
  const v = new URLSearchParams();
  for (const [k, val] of params) if (own(k)) v.set(k, val);
  return v;
}

/** Publish a vehicle. Call only once the server has accepted the
 *  configuration, so a half-typed NaN stays local to the tab that typed it.
 *  Storage can throw (private mode, quota); a vehicle that fails to publish
 *  is not worth losing the edit over. */
export function save(params) {
  try {
    localStorage.setItem(KEY, pack(params).toString());
    return true;
  } catch (e) {
    return false;
  }
}

/** The stored vehicle as a query string, or '' if there is none. */
export function load() {
  try {
    return localStorage.getItem(KEY) || '';
  } catch (e) {
    return '';
  }
}

// A note that the vehicle was edited since the last time mission control looked.
//
// The storage event only fires in OTHER tabs, so editing in the builder and
// then navigating back in the SAME tab applied the new stack silently and the
// page showed "NO RUN YET" as though nothing had happened. The builder drops
// this note on every accepted edit; mission control takes it on arrival (and
// when the cross-tab event fires) and raises the stale banner.
export const DIRTY_KEY = 'ssjl.vehicleDirty';

export function markDirty(why) {
  try { localStorage.setItem(DIRTY_KEY, why || 'the vehicle changed'); } catch (e) {}
}

/** Read and clear the note, returning the reason or null. */
export function takeDirty() {
  try {
    const w = localStorage.getItem(DIRTY_KEY);
    if (w) localStorage.removeItem(DIRTY_KEY);
    return w;
  } catch (e) {
    return null;
  }
}
