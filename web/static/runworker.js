import { panelRun, panelGeometry, panelSweep, panelSolve } from '../src/panel.js';

function progress(id, stage, detail, current, total, done) {
  self.postMessage({ id, progress: { ok: true, stage, detail, current, total, done: !!done } });
}

const SIM_DETAIL = {
  landing: 'optimizing ascent, translunar injection, and powered descent…',
  orbit: 'optimizing ascent and propagating the requested orbit…',
  suborbital: 'solving guidance and propagating the ballistic arc…',
  flyby: 'optimizing ascent and the free-return trajectory…',
};

self.onmessage = e => {
  const { id, path, params, mode } = e.data || {};
  try {
    let result;
    if (path === '/api/run') {
      const m = mode || params.mode || 'flyby';
      progress(id, 'configuration', 'validating the vehicle and mission inputs…', 1, 4);
      progress(id, 'simulation', SIM_DETAIL[m] || SIM_DETAIL.flyby, 2, 4);
      result = panelRun(params, mode);
      progress(id, 'diagnostics', 'building event, telemetry, and ground-track data…', 3, 4);
      progress(id, 'complete', 'trajectory ready', 4, 4, true);
    } else if (path === '/api/geometry') {
      result = panelGeometry(params);
    } else if (path === '/api/sweep') {
      result = panelSweep(params, p => progress(id, p.stage, p.detail, p.current, p.total, p.done));
    } else if (path === '/api/solve') {
      result = panelSolve(params, p => progress(id, p.stage, p.detail, p.current, p.total, p.done));
    } else {
      result = { ok: false, error: `no route for POST ${path}` };
    }
    self.postMessage({ id, result });
  } catch (err) {
    self.postMessage({ id, error: String(err && err.message ? err.message : err) });
  }
};
