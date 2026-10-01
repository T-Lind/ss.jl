// Port of src/solve.jl. Keep evaluated metrics separate from Illinois weights.
export function find_root(f, lo, hi, { target = 0, xtol = 0, ftol = 0, max_iter = 24 } = {}) {
  if (![lo, hi, target, xtol, ftol].every(Number.isFinite) || xtol < 0 || ftol < 0
      || !Number.isInteger(max_iter) || max_iter < 1)
    throw new RangeError('finite bounds, target, nonnegative tolerances and positive integer budget required');
  [lo, hi] = [Math.min(lo, hi), Math.max(lo, hi)];
  if (!Number.isFinite(hi - lo)) throw new RangeError('search interval is too large');
  const history = [], xt = xtol > 0 ? xtol : 1e-4 * Math.max(hi - lo, Number.EPSILON);
  let best_x = NaN, best_v = NaN, best_f = Infinity, ulo = lo, uhi = hi;
  const result = status => ({ x: best_x, value: best_v, target,
    iterations: history.length, status, lo: ulo, hi: uhi, history });
  const evaluate = x => {
    if (history.length >= max_iter) return NaN;
    let v;
    try { const raw = f(x); v = raw == null ? NaN : Number(raw); } catch (e) { v = NaN; }
    const residual = v - target;
    history.push([x, v]);
    if (Number.isFinite(residual) && Math.abs(residual) < best_f) {
      best_x = x; best_v = v; best_f = Math.abs(residual);
    }
    return residual;
  };
  const usable = (x, toward) => {
    let v = evaluate(x);
    for (let k = 0; k < 8; k++) {
      if (Number.isFinite(v) || history.length >= max_iter || x === toward) break;
      x += 0.5 * (toward - x); v = evaluate(x);
    }
    return [x, v];
  };
  let flo, fhi;
  [lo, flo] = usable(lo, hi);
  if (!Number.isFinite(flo)) return result(history.length >= max_iter ? 'max_iter' : 'infeasible');
  if (Math.abs(flo) <= ftol) return result('converged');
  if (history.length >= max_iter) return result('max_iter');
  [hi, fhi] = usable(hi, lo);
  [ulo, uhi] = [lo, hi];
  if (!Number.isFinite(fhi)) return result(history.length >= max_iter ? 'max_iter' : 'infeasible');
  if (Math.abs(fhi) <= ftol) return result('converged');
  if ((flo < 0) === (fhi < 0)) return result('no_bracket');
  let side = 0;
  let vlo = history.findLast(([x]) => x === lo)[1], vhi = history.at(-1)[1];
  const bracketResult = () => {
    [best_x, best_v] = Math.abs(vlo - target) <= Math.abs(vhi - target) ? [lo, vlo] : [hi, vhi];
    return result('converged');
  };
  while (hi - lo > xt) {
    if (history.length >= max_iter) return result('max_iter');
    const scale = Math.max(Math.abs(flo), Math.abs(fhi));
    const a = Math.abs(flo) / scale, b = Math.abs(fhi) / scale;
    let x = lo + (hi - lo) * a / (a + b);
    x = Math.min(hi - 0.01 * (hi - lo), Math.max(lo + 0.01 * (hi - lo), x));
    if (!(lo < x && x < hi)) return bracketResult();
    const fx = evaluate(x);
    if (!Number.isFinite(fx)) return result('infeasible');
    if (Math.abs(fx) <= ftol) return result('converged');
    if ((fx < 0) !== (flo < 0)) {
      hi = x; fhi = fx;
      vhi = history.at(-1)[1];
      if (side === -1) flo *= 0.5;
      side = -1;
    } else {
      lo = x; flo = fx;
      vlo = history.at(-1)[1];
      if (side === 1) fhi *= 0.5;
      side = 1;
    }
  }
  return bracketResult();
}
