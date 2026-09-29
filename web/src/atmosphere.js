// Port of src/atmosphere.jl: USSA76 (layered 0-86 km, tabulated 86-500 km,
// exponential above) and ScaledAtmosphere.
import { G0, R_AIR, GAMMA_AIR } from './constants.js';

// geopotential base altitude [m], base T [K], base p [Pa], lapse rate [K/m]
const USSA76_LAYERS = [
  [0.0, 288.15, 101325.0, -0.0065],
  [11000.0, 216.65, 22632.06, 0.0],
  [20000.0, 216.65, 5474.889, 0.0010],
  [32000.0, 228.65, 868.0187, 0.0028],
  [47000.0, 270.65, 110.9063, 0.0],
  [51000.0, 270.65, 66.93887, -0.0028],
  [71000.0, 214.65, 3.956420, -0.0020],
];
const USSA76_R0 = 6356766.0;

// geometric altitude [m], T [K], rho [kg/m^3]
const USSA76_UPPER = [
  [86.0e3, 186.87, 6.958e-6],
  [90.0e3, 186.87, 3.416e-6],
  [95.0e3, 188.42, 1.393e-6],
  [100.0e3, 195.08, 5.604e-7],
  [110.0e3, 240.00, 9.708e-8],
  [120.0e3, 360.00, 2.222e-8],
  [130.0e3, 469.27, 8.152e-9],
  [140.0e3, 559.63, 3.831e-9],
  [150.0e3, 634.39, 2.076e-9],
  [160.0e3, 696.29, 1.233e-9],
  [180.0e3, 790.07, 5.194e-10],
  [200.0e3, 854.56, 2.541e-10],
  [250.0e3, 941.33, 6.073e-11],
  [300.0e3, 976.01, 1.916e-11],
  [350.0e3, 990.06, 7.014e-12],
  [400.0e3, 995.83, 2.803e-12],
  [450.0e3, 998.22, 1.184e-12],
  [500.0e3, 999.24, 5.215e-13],
];

export const USSA76 = { kind: 'ussa76' };

// -> [rho, T, p, a]
export function atmosphere_state(atm, hIn) {
  if (atm.kind === 'scaled') {
    const [rho, T, p, a] = atmosphere_state(atm.base, hIn);
    return [rho * atm.rho_mult, T, p * atm.rho_mult, a];
  }
  let h = Math.max(hIn, -100.0);
  if (h <= 86.0e3) {
    const hp = USSA76_R0 * h / (USSA76_R0 + h);
    let k = USSA76_LAYERS.length - 1;
    for (let i = 0; i < USSA76_LAYERS.length; i++) {
      if (i === USSA76_LAYERS.length - 1 || USSA76_LAYERS[i + 1][0] > hp) {
        k = i;
        break;
      }
    }
    const [hb, Tb, pb, L] = USSA76_LAYERS[k];
    const dh = hp - hb;
    let T, p;
    if (L === 0.0) {
      T = Tb;
      p = pb * Math.exp(-G0 * dh / (R_AIR * Tb));
    } else {
      T = Tb + L * dh;
      p = pb * Math.pow(Tb / T, G0 / (R_AIR * L));
    }
    const rho = p / (R_AIR * T);
    return [rho, T, p, Math.sqrt(GAMMA_AIR * R_AIR * T)];
  }
  const n = USSA76_UPPER.length;
  if (h >= USSA76_UPPER[n - 1][0]) {
    const [h1, , r1] = USSA76_UPPER[n - 2];
    const [h2, T2, r2] = USSA76_UPPER[n - 1];
    const Hs = (h2 - h1) / Math.log(r1 / r2);
    const rho = r2 * Math.exp(-(h - h2) / Hs);
    return [rho, T2, rho * R_AIR * T2, Math.sqrt(GAMMA_AIR * R_AIR * T2)];
  }
  let k = 0;
  for (let i = 0; i < n - 1; i++) {
    if (USSA76_UPPER[i + 1][0] > h) { k = i; break; }
  }
  const [h1, T1, r1] = USSA76_UPPER[k];
  const [h2, T2, r2] = USSA76_UPPER[k + 1];
  const f = (h - h1) / (h2 - h1);
  const T = T1 + f * (T2 - T1);
  const rho = r1 * Math.exp(f * Math.log(r2 / r1));
  return [rho, T, rho * R_AIR * T, Math.sqrt(GAMMA_AIR * R_AIR * T)];
}

export const scaledAtmosphere = (base, rho_mult) => ({ kind: 'scaled', base, rho_mult });
