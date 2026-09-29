// Port of src/aerodynamics.jl. Tables are {x, y}; interpolation is clamped
// linear, matching interp1's searchsortedlast.
export const table1d = (x, y) => ({ x: Array.from(x), y: Array.from(y) });

export function interp1(t, x) {
  const xs = t.x, ys = t.y;
  const n = xs.length;
  if (x <= xs[0]) return ys[0];
  if (x >= xs[n - 1]) return ys[n - 1];
  // searchsortedlast: last index with xs[i] <= x
  let lo = 0, hi = n - 1;
  while (hi - lo > 1) { const m = (lo + hi) >> 1; if (xs[m] <= x) lo = m; else hi = m; }
  const i = lo;
  const f = (x - xs[i]) / (xs[i + 1] - xs[i]);
  return ys[i] + f * (ys[i + 1] - ys[i]);
}

export const capsuleAero = (cd0, cla, cma, cmq, cl_trim, alpha_trim = 0.0,
                            kd_alpha2 = 0.4) =>
  ({ kind: 'capsule', cd0, cla, cma, cmq, cl_trim, alpha_trim, kd_alpha2 });

export function cd_coeff(a, M, alpha) {
  const da = alpha - a.alpha_trim;
  return interp1(a.cd0, M) * (1 + a.kd_alpha2 * da * da);
}
export function cl_coeff(a, M, alpha) {
  return interp1(a.cla, M) * (alpha - a.alpha_trim) + interp1(a.cl_trim, M);
}
export function cm_coeff(a, M, alpha, qhat) {
  return interp1(a.cma, M) * (alpha - a.alpha_trim) + interp1(a.cmq, M) * qhat;
}

export function default_capsule_aero({ alpha_trim = 0.0, cl_trim_hyp = 0.0 } = {}) {
  const mach = [0.3, 0.7, 0.9, 1.1, 1.5, 2.0, 3.0, 5.0, 8.0, 12.0, 20.0, 30.0];
  const cd0 = [0.78, 0.85, 1.00, 1.25, 1.35, 1.40, 1.42, 1.45, 1.48, 1.50, 1.52, 1.52];
  const cla = [0.25, 0.25, 0.30, 0.35, 0.40, 0.45, 0.50, 0.50, 0.50, 0.50, 0.50, 0.50];
  const cma = [-0.06, -0.05, -0.04, -0.07, -0.10, -0.12, -0.12, -0.11, -0.10, -0.10, -0.10, -0.10];
  const cmq = [-0.20, -0.12, -0.08, -0.15, -0.25, -0.30, -0.30, -0.30, -0.30, -0.30, -0.30, -0.30];
  const cltr = [0, 0, 0, 0.3, 0.6, 0.8, 1, 1, 1, 1, 1, 1].map(v => cl_trim_hyp * v);
  return capsuleAero(table1d(mach, cd0), table1d(mach, cla), table1d(mach, cma),
                     table1d(mach, cmq), table1d(mach, cltr), alpha_trim);
}

export function scaled_aero(a, { cd_mult = 1.0, cma_mult = 1.0, alpha_trim_delta = 0.0 } = {}) {
  return capsuleAero(
    table1d(a.cd0.x, a.cd0.y.map(v => v * cd_mult)),
    a.cla,
    table1d(a.cma.x, a.cma.y.map(v => v * cma_mult)),
    a.cmq, a.cl_trim, a.alpha_trim + alpha_trim_delta, a.kd_alpha2);
}
