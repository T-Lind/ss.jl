// Vehicle assembly: a part-based model of the launcher, and the one place that
// turns a stack of parts into the panel's vehicle parameters (and back).
//
// The simulator flies a LaunchVehicle described by s<k>_* stage fields, a b_*
// strap-on set and a payload. The assembly page lets you build that vehicle
// out of parts instead — tanks, engine clusters, decouplers, a payload, a
// fairing and radial boosters — and this module is the contract between the
// two. Nothing here simulates; it only translates, so the vehicle you see in
// the hangar and the vehicle the mission flies cannot drift apart.
//
// Translation rules, stated once:
//   * the stack is ordered top to bottom; decouplers split it into stages,
//     and the bottom stage is s1 (fires first), exactly as the panel numbers
//     them. The simulator flies 2-5 stages, so the assembly must too.
//   * a stage is its tanks plus ONE engine cluster at its bottom. The engine
//     decides the propellant; the tanks decide how much of it there is
//     (size_by=length: the simulator's own volume rule, with domes).
//   * tanks of mixed diameter in one stage are flown at the stage's largest
//     diameter with the same total volume — reported as a warning, not hidden.
//   * a named (catalogue) engine lets the simulator estimate the stage's dry
//     mass. A generic engine has no catalogue entry, so its dry mass is
//     estimated here with the same relation (`stageMass` mirrors
//     src/engines.js stage_mass) and sent explicitly.
//   * an explicit dry-mass override on the engine part wins over both — that
//     is how real vehicles with stated masses (Saturn V) keep them.
//
// Shared by the static web build and the Julia panel, so it imports nothing
// from ../src: the catalogue (engines, propellant bulk densities) arrives
// from api.catalogue() at runtime, and the handful of propellant constants
// the dry-mass estimate needs are mirrored below.

export const G0 = 9.80665;
export const P0_SEA = 101325.0;

/** Propellant constants, mirrored from src/engines.js PROPELLANTS:
 *  [fuel density, oxidiser density, mixture ratio, cryogenic]. */
export const PROPS = {
  kerolox:    [810.0, 1141.0, 2.56, true],
  hydrolox:   [70.8, 1141.0, 5.50, true],
  methalox:   [422.6, 1141.0, 3.60, true],
  hypergolic: [880.0, 1443.0, 1.65, false],
  hydrazine:  [1004.0, 0.0, 0.0, false],
  solid:      [1750.0, 0.0, 0.0, false],
};

export const PROP_COLORS = {
  kerolox: '#e8a33c', hydrolox: '#6fb7ff', methalox: '#b58cff',
  hypergolic: '#7fd67f', hydrazine: '#9ad', solid: '#d7745a',
};

export function bulkDensity(prop) {
  const [rf, ro, mr] = PROPS[prop] || PROPS.kerolox;
  return mr <= 0 ? rf : (1 + mr) / (1 / rf + mr / ro);
}

/** Propellant a tank volume holds, by the simulator's own rule
 *  (src/panel.js stage_from_params, size_by=length): the length includes
 *  0.9 d of domes and ullage, and 15 % of the barrel is not propellant. */
export function propFromLength(len, d, prop) {
  return Math.max(1.0, (Math.max(len, 0.95 * d) - 0.9 * d) *
                       bulkDensity(prop) * Math.PI * (d / 2) ** 2 / 1.15);
}

/** The inverse: the tank length that holds `mprop` at diameter d. */
export function lengthFromProp(mprop, d, prop) {
  return mprop * 1.15 / (bulkDensity(prop) * Math.PI * (d / 2) ** 2) + 0.9 * d;
}

/** Dry mass of a stage, mirroring src/engines.js stage_mass. A solid motor is
 *  its own tank, and a filament or steel case is far heavier than a liquid
 *  tank's wall per unit volume, so it is charged 9 % of its propellant. */
export function stageMass(eng, n, mprop, d) {
  const [rf, ro, mr, cryo] = PROPS[eng.prop] || PROPS.kerolox;
  const engines = n * eng.mass_kg;
  if (eng.prop === 'solid') return engines + 0.09 * mprop + 0.55 * mprop ** 0.78 * 0.3;
  const mf = mr <= 0 ? mprop : mprop / (1 + mr);
  const vf = mf / rf, vox = mr <= 0 ? 0 : (mprop - mf) / ro;
  const kf = eng.prop === 'hydrolox' ? 9.1 : 12.2;
  const tanks = kf * vf + 12.2 * vox;
  const L = (vf + vox) / (Math.PI * d * d / 4);
  const area = Math.PI * d * L + Math.PI * d * d / 2;
  const insulation = cryo ? 1.123 * area : 0;
  const thrustStructure = 2.55e-4 * n * eng.thrust_kn * 1e3;
  const systems = 0.55 * mprop ** 0.78;
  return tanks + insulation + engines + thrustStructure + systems;
}

// --------------------------------------------------------------- catalogue --

/** Display names for the simulator's catalogue engines. */
const ENGINE_LABELS = {
  merlin_1d: 'Merlin 1D', merlin_1d_vac: 'Merlin 1D Vac', rutherford: 'Rutherford',
  rutherford_vac: 'Rutherford Vac', raptor_2: 'Raptor 2', rs25: 'RS-25',
  rl10b2: 'RL10B-2', vinci: 'Vinci', aj10: 'AJ10', f1: 'F-1', j2: 'J-2',
};

/** Generic engines: not in the simulator's catalogue, so they fly as
 *  'manual' stages with these figures. The Sable family reproduces the
 *  reference vehicle per engine (950 kN / 5 engines, and so on). Mass follows
 *  the simulator's own estimate for an unknown engine, thrust / (g0 * 100). */
export const GENERIC_ENGINES = {
  sable_b190: { label: 'Sable B-190', prop: 'kerolox', thrust_kn: 190, isp_vac: 305, ae: 0.16,
                note: 'sea-level kerolox booster engine — the reference first stage flies five' },
  sable_u95:  { label: 'Sable U-95 Vac', prop: 'kerolox', thrust_kn: 95, isp_vac: 345, ae: 0.0,
                note: 'vacuum-optimised upper-stage engine' },
  sable_k15:  { label: 'Sable K-15', prop: 'hypergolic', thrust_kn: 15, isp_vac: 315, ae: 0.0,
                note: 'storable kick-stage engine — restartable, does TLI' },
  sable_s190: { label: 'Sable S-190', prop: 'kerolox', thrust_kn: 190, isp_vac: 285, ae: 0.16,
                note: 'simplified strap-on engine, gas-generator cycle' },
  granite_srm: { label: 'Granite SRM', prop: 'solid', thrust_kn: 1400, isp_vac: 268, ae: 0.9,
                 note: 'solid rocket motor: its case is the tank, it burns until empty' },
};
for (const e of Object.values(GENERIC_ENGINES)) e.mass_kg = e.thrust_kn * 1e3 / (G0 * 100);

/** Engines the page can offer: the catalogue's plus the generic ones. Each
 *  entry: {id, label, prop, thrust_kn, isp_vac, isp_sl, mass_kg, generic}. */
export function engineList(catalogue) {
  const out = [];
  for (const e of (catalogue?.engines || [])) {
    out.push({ id: e.name, label: ENGINE_LABELS[e.name] || e.name, prop: e.propellant,
               thrust_kn: e.thrust_kn, isp_vac: e.isp_vac,
               isp_sl: e.isp_sl > 0 ? e.isp_sl : null, mass_kg: e.mass_kg, generic: false });
  }
  for (const [id, e] of Object.entries(GENERIC_ENGINES)) {
    const tsl = e.thrust_kn - P0_SEA * e.ae / 1e3;
    out.push({ id, label: e.label, prop: e.prop, thrust_kn: e.thrust_kn, isp_vac: e.isp_vac,
               isp_sl: e.ae > 0 ? e.isp_vac * tsl / e.thrust_kn : null, mass_kg: e.mass_kg,
               generic: true, note: e.note });
  }
  return out;
}

export function engineById(catalogue, id) {
  return engineList(catalogue).find(e => e.id === id) || null;
}

/** Imported manual engines keep their actual performance on the part. */
export function engineForPart(catalogue, part) {
  return part.engine === 'manual' && part.custom ? part.custom : engineById(catalogue, part.engine);
}

/** Standard tank diameters [m], from small launchers to Saturn class. */
export const TANK_DIAMETERS = [1.2, 1.8, 2.4, 3.0, 3.7, 5.0, 6.6, 9.0, 10.1];
export const TANK_CLASS = { 1.2: 'XS', 1.8: 'S', 2.4: 'M', 3.0: 'M+', 3.7: 'L', 5.0: 'XL',
                            6.6: 'XXL', 9.0: 'H', 10.1: 'H+' };

/** Capsules by name: [mass kg, diameter m] — the builder's PODS table. */
export const CAPSULES = [
  ['Minimal 1-crew', 900, 1.60], ['Mercury', 1400, 1.89], ['Soyuz DM', 2950, 2.20],
  ['Gemini', 3850, 2.32], ['Shenzhou DM', 3240, 2.52], ['Dragon 1 cargo', 4200, 3.66],
  ['Apollo CM', 5560, 3.91], ['Crew Dragon', 12500, 4.00], ['Starliner', 13000, 4.56],
  ['Orion CM', 10400, 5.02],
];
export const PROBES = [['Probe core S', 200], ['Probe core M', 600], ['Probe core L', 2000]];
/** Lunar landers: dry, propellant, thrust kN, Isp s, min throttle %, deployed diameter. */
export const LANDERS = [
  ['Lander — light', { dry: 1200, prop: 3400, thrust_kn: 16, isp: 311, throttle_min: 10, diameter: 3.2 }],
  ['Lander — standard', { dry: 3500, prop: 9000, thrust_kn: 45, isp: 311, throttle_min: 10, diameter: 4.2 }],
  ['Lander — heavy', { dry: 6000, prop: 16000, thrust_kn: 75, isp: 311, throttle_min: 10, diameter: 5.2 }],
];

/** The parts palette, by category. `make()` returns a fresh part instance. */
export function palette(catalogue) {
  const cats = [];
  cats.push({ id: 'pods', label: 'Command & payload', parts: [
    ...CAPSULES.map(([n, m, d]) => ({ key: 'cap:' + n, label: n, sub: `${(m/1000).toFixed(1)} t · ⌀${d} m`,
      make: () => ({ type: 'payload', kind: 'capsule', name: n, mass: m, diameter: d, crewed: true, cargo: 0 }) })),
    ...PROBES.map(([n, m]) => ({ key: 'bus:' + n, label: n, sub: `${m} kg uncrewed bus`,
      make: () => ({ type: 'payload', kind: 'bus', name: n, mass: m, diameter: 0, crewed: false, cargo: 0 }) })),
    ...LANDERS.map(([n, l]) => ({ key: 'lnd:' + n, label: n,
      sub: `${((l.dry + l.prop)/1000).toFixed(1)} t · ${l.thrust_kn} kN · lunar landing`,
      make: () => ({ type: 'payload', kind: 'lander', name: n, lander: { ...l } }) })),
  ]});
  cats.push({ id: 'tanks', label: 'Fuel tanks', parts: TANK_DIAMETERS.map(d => ({
    key: 'tank:' + d, label: `Tank ${TANK_CLASS[d]} · ⌀${d} m`, sub: `${(2 * d).toFixed(1)} m long, stretchable`,
    make: () => ({ type: 'tank', diameter: d, length: +(2 * d).toFixed(2) }) })) });
  cats.push({ id: 'engines', label: 'Engines', parts: engineList(catalogue).map(e => ({
    key: 'eng:' + e.id, label: e.label,
    sub: `${e.thrust_kn >= 1000 ? (e.thrust_kn/1000).toFixed(2) + ' MN' : e.thrust_kn.toFixed(0) + ' kN'} · ${e.isp_vac.toFixed(0)} s · ${e.prop}`,
    make: () => ({ type: 'engine', engine: e.id, count: 1, dry: null }) })) });
  cats.push({ id: 'struct', label: 'Structural', parts: [
    { key: 'dec', label: 'Stage decoupler', sub: 'separates two stages',
      make: () => ({ type: 'decoupler' }) },
    { key: 'fair', label: 'Payload fairing', sub: 'aero shroud, jettisoned at 120 km',
      make: () => ({ type: 'fairing', mass: 150 }) },
  ]});
  cats.push({ id: 'radial', label: 'Radial boosters', parts: [
    { key: 'strap', label: 'Liquid strap-on', sub: 'radial set on stage 1, 2-8 in symmetry',
      make: () => ({ type: 'boosters', count: 2, diameter: 1.5, length: 9.0, engine: 'sable_s190',
                     engines: 2, throttle: 100, ign: 0, sep: 0, dry: null }) },
    { key: 'srb', label: 'Solid booster', sub: 'radial solid motors on stage 1',
      make: () => ({ type: 'boosters', count: 2, diameter: 1.6, length: 14.0, engine: 'granite_srm',
                     engines: 1, throttle: 100, ign: 0, sep: 0, dry: null }) },
  ]});
  return cats;
}

// ----------------------------------------------------------------- assembly --

// ------------------------------------------------------------------ paint --
// Paint is cosmetic and changes nothing the simulator flies. A part may carry
// `paint: {c, a, p}` — base colour, accent colour, pattern. The launch view
// paints per SECTION (each stage, the strap-on set, the fairing, the
// payload), so a stage takes the paint of its first painted tank. It travels
// as one `livery` parameter: JSON keyed s1..s5, b, fair, pay.
export const PATTERNS = [
  ['solid', 'Solid'], ['bands', 'Bands'], ['roll', 'Roll pattern'],
  ['checker', 'Checker'], ['stripe', 'Racing stripe'],
];
export const SWATCHES = [
  ['#e9ebee', 'White'], ['#1c1f24', 'Black'], ['#b9c0c8', 'Bare steel'],
  ['#d9772b', 'Foam orange'], ['#c9a24a', 'Gold foil'], ['#2a4b7c', 'Navy'],
  ['#b8322a', 'Red'], ['#3f7d4e', 'Green'], ['#f0c419', 'Yellow'], ['#6b6f76', 'Graphite'],
];
/** Ready-made liveries: a function of the stage index (1 = bottom). */
export const LIVERIES = {
  'Classic white': () => ({ c: '#e9ebee', a: '#1c1f24', p: 'solid' }),
  'Roll pattern': k => ({ c: '#e9ebee', a: '#1c1f24', p: k === 1 ? 'roll' : 'bands' }),
  'Stealth': () => ({ c: '#1c1f24', a: '#6b6f76', p: 'solid' }),
  'Foam & steel': k => k === 1 ? { c: '#d9772b', a: '#b9c0c8', p: 'solid' }
                               : { c: '#b9c0c8', a: '#1c1f24', p: 'bands' },
  'Racer': () => ({ c: '#e9ebee', a: '#b8322a', p: 'stripe' }),
};
const HEX = /^#[0-9a-f]{6}$/i;
const cleanPaint = q => q && HEX.test(q.c || '') ? { c: q.c.toLowerCase(),
  a: HEX.test(q.a || '') ? q.a.toLowerCase() : '#1c1f24',
  p: PATTERNS.some(([k]) => k === q.p) ? q.p : 'solid' } : null;

/** The livery of an assembly, per launch-view section, or null if unpainted. */
export function liveryOf(asm, catalogue) {
  const a = analyse(asm, catalogue), out = {};
  a.stages.forEach(s => {
    const t = s.tanks.find(t => cleanPaint(t.paint));
    if (t) out['s' + s.k] = cleanPaint(t.paint);
  });
  if (cleanPaint(asm.boosters?.paint)) out.b = cleanPaint(asm.boosters.paint);
  if (cleanPaint(a.fairing?.paint)) out.fair = cleanPaint(a.fairing.paint);
  if (cleanPaint(a.payload?.paint)) out.pay = cleanPaint(a.payload.paint);
  return Object.keys(out).length ? out : null;
}
/** Parse a `livery` parameter; null when absent or malformed. */
export function parseLivery(str) {
  if (!str) return null;
  try {
    const o = JSON.parse(str), out = {};
    for (const [k, v] of Object.entries(o || {}))
      if (/^(s[1-5]|b|fair|pay)$/.test(k) && cleanPaint(v)) out[k] = cleanPaint(v);
    return Object.keys(out).length ? out : null;
  } catch (e) { return null; }
}

let SEQ = 0;
const pid = () => 'p' + (++SEQ).toString(36);
/** Give a part instance an id (parts from palette().make() come without). */
export const withId = part => (part.id ? part : { ...part, id: pid() });

/** An empty assembly. `parts` is the stack, top to bottom; `boosters` is the
 *  radial set on stage 1 or null; `kick_deg` the pitch-over kick. */
export function emptyAssembly(name = 'Untitled') {
  return { name, parts: [], boosters: null, kick_deg: 8, opt_kick: true };
}

/** Split the stack into stages, bottom first (index 0 = s1), with the
 *  payload and fairing found above them. Every problem is reported in
 *  `issues` ({level: 'error'|'warn', msg, partId?}); an assembly with no
 *  errors maps onto a vehicle the simulator will accept. */
export function analyse(asm, catalogue) {
  const issues = [];
  const parts = asm.parts;
  const payloadIdx = parts.findIndex(p => p.type === 'payload');
  const payload = payloadIdx >= 0 ? parts[payloadIdx] : null;
  const fairing = parts.find(p => p.type === 'fairing') || null;
  if (!payload) issues.push({ level: 'error', msg: 'no payload — add a capsule, probe core or lander on top' });
  else if (payloadIdx !== 0) issues.push({ level: 'error', msg: 'the payload must sit at the top of the stack', partId: payload.id });
  if (parts.filter(p => p.type === 'payload').length > 1)
    issues.push({ level: 'error', msg: 'only one payload can fly' });
  if (fairing && payload?.kind === 'lander')
    issues.push({ level: 'warn', msg: 'a lander flies exposed in its adapter; the fairing is kept as a shroud' });

  // segments between decouplers, top to bottom
  const segs = [[]];
  for (const p of parts) {
    if (p.type === 'payload' || p.type === 'fairing') continue;
    if (p.type === 'decoupler') { segs.push([]); continue; }
    segs[segs.length - 1].push(p);
  }
  const stagesTop = segs.filter(s => s.length > 0);
  const stages = stagesTop.slice().reverse().map((seg, i) => {
    const k = i + 1;
    const tanks = seg.filter(p => p.type === 'tank');
    const engines = seg.filter(p => p.type === 'engine');
    const engine = engines[engines.length - 1] || null;
    const e = engine ? engineForPart(catalogue, engine) : null;
    if (engine && !e) issues.push({ level: 'error', msg: `stage ${k} has an unknown engine`, partId: engine.id });
    if (!engine) issues.push({ level: 'error', msg: `stage ${k} has no engine`, partId: seg[seg.length - 1].id });
    if (engines.length > 1) issues.push({ level: 'error', msg: `stage ${k} has ${engines.length} engine parts — a stage burns one engine type; set the cluster count instead`, partId: engines[0].id });
    if (engine && seg[seg.length - 1] !== engine) issues.push({ level: 'error', msg: `stage ${k}: the engine must be at the bottom of its stage (add a decoupler below it)`, partId: engine.id });
    if (!tanks.length && e?.prop !== 'solid') issues.push({ level: 'error', msg: `stage ${k} has no fuel tank`, partId: engine?.id });
    const diameter = tanks.length ? Math.max(...tanks.map(t => t.diameter)) : 1.0;
    if (new Set(tanks.map(t => t.diameter)).size > 1)
      issues.push({ level: 'warn', msg: `stage ${k} mixes tank diameters — flown as ⌀${diameter} m with the same volume`, partId: tanks[0].id });
    // equivalent length at the stage diameter, same barrel volume
    const domes = 0.9 * diameter;
    const barrel = tanks.reduce((s, t) => s + Math.max(0, t.length - 0.9 * t.diameter) * (t.diameter / diameter) ** 2, 0);
    const length = tanks.length ? barrel + domes : e?.prop === 'solid' ? 4 * diameter : 0;
    const prop = e ? e.prop : 'kerolox';
    const mprop = length > 0 ? propFromLength(length, diameter, prop) : 0;
    const n = engine ? Math.max(1, Math.min(33, Math.round(engine.count || 1))) : 0;
    let dry = null, dryAuto = false;
    if (engine && engine.dry != null && engine.dry > 0) dry = engine.dry;
    else if (e && !e.generic) dryAuto = true;
    else if (e) dry = stageMass(e, n, mprop, diameter);
    return { k, seg, tanks, engine, e, n, diameter, length, prop, mprop, dry, dryAuto };
  });
  if (stages.length && (stages.length < 2 || stages.length > 5))
    issues.push({ level: 'error', msg: `the simulator flies 2–5 stages; this stack has ${stages.length}` });
  if (!stages.length) issues.push({ level: 'error', msg: 'no stages yet — add tanks and an engine below the payload' });

  let boosters = null;
  if (asm.boosters) {
    const b = asm.boosters, e = engineForPart(catalogue, b);
    if (!e) issues.push({ level: 'error', msg: 'the boosters have no engine' });
    const prop = e ? e.prop : 'kerolox';
    const mprop = propFromLength(b.length, b.diameter, prop);
    const n = Math.max(1, Math.min(9, Math.round(b.engines || 1)));
    let dry = null, dryAuto = false;
    if (b.dry != null && b.dry > 0) dry = b.dry;
    else if (e && !e.generic) dryAuto = true;
    else if (e) dry = stageMass(e, n, mprop, b.diameter);
    boosters = { ...b, e, prop, mprop, n, dry, dryAuto };
    if (b.count < 2 || b.count > 8) issues.push({ level: 'error', msg: 'a booster set is 2–8 in symmetry' });
  }
  return { payload, fairing, stages, boosters, issues,
           ok: !issues.some(i => i.level === 'error') };
}

/** Which missions a payload can fly. A lander is the lunar-landing vehicle;
 *  a capsule or probe flies everything else. */
export function missionsFor(payload) {
  if (!payload) return [];
  return payload.kind === 'lander' ? ['landing'] : ['suborbital', 'orbit', 'flyby'];
}

/** The panel's vehicle parameters for an assembly. Only vehicle keys (see
 *  static/vehicle.js `own`) are produced; mission keys belong to the flight
 *  plan. Throws if the assembly has errors. */
export function toParams(asm, catalogue) {
  const a = analyse(asm, catalogue);
  if (!a.ok) throw new Error(a.issues.find(i => i.level === 'error').msg);
  const u = new URLSearchParams();
  const set = (k, v) => u.set(k, String(v));
  set('vname', asm.name || 'Untitled');
  set('nstages', a.stages.length);
  set('diameter', a.stages[0].diameter);
  set('kick_deg', asm.kick_deg ?? 8);
  set('opt_kick', asm.opt_kick === false ? '0' : '1');
  const stageKeys = (pre, s, eng, n, dia, len, dry, dryAuto) => {
    set(pre + 'size_by', 'length');
    set(pre + 'len', len);
    set(pre + 'diameter', dia);
    set(pre + 'engines', n);
    if (eng.generic) {
      set(pre + 'engine', 'manual');
      set(pre + 'propellant', eng.prop);
      set(pre + 'thrust_kn', eng.thrust_kn * n);
      set(pre + 'isp', eng.isp_vac);
      set(pre + 'dry', dry);
      set(pre + 'dry_auto', '0');
    } else {
      set(pre + 'engine', eng.id);
      set(pre + 'propellant', eng.prop);
      set(pre + 'dry_auto', dryAuto ? '1' : '0');
      if (!dryAuto) set(pre + 'dry', dry);
    }
  };
  a.stages.forEach(s => {
    // a solid stage with no tank part is its own case: one motor-length barrel
    const len = s.tanks.length ? s.length : 4 * s.diameter;
    stageKeys(`s${s.k}_`, s, s.e, s.n, s.diameter, len, s.dry ?? stageMass(s.e, s.n, s.mprop, s.diameter), s.dryAuto);
  });
  if (a.boosters) {
    const b = a.boosters;
    set('nboost', b.count);
    stageKeys('b_', b, b.e, b.n, b.diameter, b.length, b.dry, b.dryAuto);
    set('b_throttle', b.throttle ?? 100);
    set('b_ign_delay', b.ign ?? 0);
    set('b_sep_delay', b.sep ?? 0);
  } else set('nboost', 0);
  const p = a.payload;
  if (a.fairing) { set('fairing_on', '1'); set('fairing', a.fairing.mass ?? 150); }
  else set('fairing_on', '0');
  if (p.kind === 'lander') {
    const l = p.lander;
    set('l_dry', l.dry); set('l_prop', l.prop); set('l_thrust_kn', l.thrust_kn);
    set('l_isp', l.isp); set('l_throttle_min', l.throttle_min); set('l_diameter', l.diameter);
  } else {
    set('payload_kind', p.kind === 'bus' ? 'bus' : 'capsule');
    if (p.kind === 'bus') set('bus_mass', p.mass); else set('pod_mass', p.mass);
    set('cargo_mass', p.cargo || 0);
    set('pod_dia', p.kind === 'capsule' ? p.diameter : 0);
    set('crewed', p.kind === 'capsule' && p.crewed !== false ? '1' : '0');
  }
  const liv = liveryOf(asm, catalogue);
  if (liv) set('livery', JSON.stringify(liv));
  return u;
}

/** Rebuild an assembly from vehicle parameters (a stored vehicle, a preset,
 *  or a query string). Stages come back as one tank and one engine cluster
 *  each — the parameters carry totals, not the parts that made them. */
export function fromParams(input, catalogue) {
  const q = input instanceof URLSearchParams ? input : new URLSearchParams(
    typeof input === 'string' ? input : Object.entries(input).map(([k, v]) => [k, String(v)]));
  const g = (k, d) => { const v = q.get(k); return v === null || v.trim() === '' ? d : v; };
  const f = (k, d) => { const v = Number(g(k, NaN)); return Number.isFinite(v) ? v : d; };
  const b = (k, d) => { const v = q.get(k); return v === null ? d : ['1', 'true', 'on'].includes(v); };
  const asm = emptyAssembly(g('vname', 'Sable'));
  asm.kick_deg = f('kick_deg', 8);
  asm.opt_kick = b('opt_kick', true);
  const coreD = f('diameter', 1.8);
  const nst = Math.min(5, Math.max(2, Math.round(f('nstages', 3))));
  const ROLE = {
    booster: { dry: 3800, prop: 42000, thrust: 950, isp: 305, ptype: 'kerolox', ne: 5 },
    upper: { dry: 900, prop: 9500, thrust: 95, isp: 345, ptype: 'kerolox', ne: 1 },
    kick: { dry: 140, prop: 950, thrust: 15, isp: 315, ptype: 'hypergolic', ne: 1 },
  };
  const engines = engineList(catalogue);
  // The engine part for a stage's fields: the named engine, else the generic
  // engine that matches exactly. Other manual engines retain their figures.
  const enginePart = (pre, def) => {
    const n = Math.max(1, Math.round(f(pre + 'engines', def.ne)));
    const name = g(pre + 'engine', 'manual');
    const dryAuto = b(pre + 'dry_auto', false);
    if (name !== 'manual') {
      const e = engines.find(x => x.id === name);
      return { e, part: { type: 'engine', engine: name, count: n, dry: dryAuto ? null : f(pre + 'dry', null) } };
    }
    const per = f(pre + 'thrust_kn', def.thrust) / n, isp = f(pre + 'isp', def.isp);
    const prop = g(pre + 'propellant', def.ptype);
    const close = (x, y) => Math.abs(x - y) <= 1e-12 * Math.max(Math.abs(y), 1);
    let e = engines.find(x => x.generic && x.prop === prop && close(x.thrust_kn, per) && close(x.isp_vac, isp));
    if (!e) e = { id: 'manual', label: 'Custom engine', prop, thrust_kn: per, isp_vac: isp,
                  mass_kg: per * 1e3 / (G0 * 100), generic: true };
    return { e, part: { type: 'engine', engine: e.id, count: n, dry: f(pre + 'dry', def.dry),
                        ...(e.id === 'manual' ? { custom: { ...e } } : {}) } };
  };
  const tankLen = (pre, def, d, prop) => g(pre + 'size_by', 'prop') === 'length'
    ? Math.max(f(pre + 'len', 10), 0.95 * d)
    : lengthFromProp(f(pre + 'prop', def.prop), d, prop);
  const LIV = parseLivery(q.get('livery'));
  const stack = [];
  for (let k = nst; k >= 1; k--) {
    const role = k === 1 ? 'booster' : k === nst ? 'kick' : 'upper';
    const sc = role === 'upper' ? 0.25 ** (k - 2) : 1;
    const def = { ...ROLE[role], dry: ROLE[role].dry * sc, prop: ROLE[role].prop * sc,
                  thrust: ROLE[role].thrust * sc };
    const pre = `s${k}_`;
    const d = f(pre + 'diameter', coreD);
    const { e, part } = enginePart(pre, def);
    if (k !== nst) stack.push({ type: 'decoupler' });
    stack.push({ type: 'tank', diameter: d, length: tankLen(pre, def, d, e?.prop || def.ptype),
                 ...(LIV?.['s' + k] ? { paint: LIV['s' + k] } : {}) });
    stack.push(part);
  }
  const top = [];
  if (g('mode', '') === 'landing' || (q.has('l_dry') && !q.has('pod_mass') && !q.has('payload_kind'))) {
    top.push({ type: 'payload', kind: 'lander', name: 'Lander', lander: {
      dry: f('l_dry', 3500), prop: f('l_prop', 9000), thrust_kn: f('l_thrust_kn', 45),
      isp: f('l_isp', 311), throttle_min: f('l_throttle_min', 10), diameter: f('l_diameter', 4.2) } });
  } else if (g('payload_kind', 'capsule') === 'bus') {
    top.push({ type: 'payload', kind: 'bus', name: 'Probe core', mass: f('bus_mass', 200), diameter: 0,
               crewed: false, cargo: f('cargo_mass', 0) });
  } else {
    const m = f('pod_mass', 350);
    const named = CAPSULES.find(([, cm]) => Math.abs(cm - m) < 1);
    top.push({ type: 'payload', kind: 'capsule', name: named ? named[0] : 'Capsule', mass: m,
               diameter: f('pod_dia', 0) || (named ? named[2] : 2 * 0.086 * Math.cbrt(m)),
               crewed: b('crewed', true), cargo: f('cargo_mass', 0) });
  }
  if (b('fairing_on', true)) top.push({ type: 'fairing', mass: f('fairing', 150), ...(LIV?.fair ? { paint: LIV.fair } : {}) });
  if (LIV?.pay && top[0]) top[0].paint = LIV.pay;
  asm.parts = [...top, ...stack].map(withId);
  const nb = Math.round(f('nboost', 0));
  if (nb > 0) {
    const BR = { dry: 900, prop: 12000, thrust: 380, isp: 285, ptype: 'kerolox', ne: 2 };
    const { e, part } = enginePart('b_', BR);
    const d = f('b_diameter', coreD * 0.85);
    asm.boosters = { type: 'boosters', count: Math.min(8, Math.max(2, nb)), diameter: d,
      length: tankLen('b_', BR, d, e?.prop || BR.ptype), engine: part.engine, engines: part.count,
      ...(part.custom ? { custom: part.custom } : {}),
      throttle: f('b_throttle', 100), ign: f('b_ign_delay', 0), sep: f('b_sep_delay', 0), dry: part.dry,
      ...(LIV?.b ? { paint: LIV.b } : {}) };
  }
  return asm;
}

/** Ready-made vehicles, as panel parameters (the same figures as mission
 *  control's vehicle presets) plus the payload each one is meant to carry. */
export const PRESETS = {
  'Sable': {
    vname: 'Sable', nstages: 3, nboost: 0, diameter: 1.8, fairing: 150, fairing_on: 1, kick_deg: 8, opt_kick: 1,
    s1_engine: 'manual', s1_prop: 42000, s1_dry: 3800, s1_thrust_kn: 950, s1_isp: 305,
    s1_engines: 5, s1_propellant: 'kerolox', s1_diameter: 1.8,
    s2_engine: 'manual', s2_prop: 9500, s2_dry: 900, s2_thrust_kn: 95, s2_isp: 345,
    s2_engines: 1, s2_propellant: 'kerolox', s2_diameter: 1.8,
    s3_engine: 'manual', s3_prop: 950, s3_dry: 140, s3_thrust_kn: 15, s3_isp: 315,
    s3_engines: 1, s3_propellant: 'hypergolic', s3_diameter: 1.8,
    payload_kind: 'capsule', pod_mass: 350, cargo_mass: 0, crewed: 1,
  },
  'Sable Heavy (4 strap-ons)': {
    vname: 'Sable Heavy', nstages: 3, nboost: 4, diameter: 1.8, fairing: 150, fairing_on: 1, kick_deg: 8, opt_kick: 1,
    s1_engine: 'manual', s1_prop: 42000, s1_dry: 3800, s1_thrust_kn: 950, s1_isp: 305,
    s1_engines: 5, s1_propellant: 'kerolox', s1_diameter: 1.8,
    s2_engine: 'manual', s2_prop: 9500, s2_dry: 900, s2_thrust_kn: 95, s2_isp: 345,
    s2_engines: 1, s2_propellant: 'kerolox', s2_diameter: 1.8,
    s3_engine: 'manual', s3_prop: 950, s3_dry: 140, s3_thrust_kn: 15, s3_isp: 315,
    s3_engines: 1, s3_propellant: 'hypergolic', s3_diameter: 1.8,
    b_engine: 'manual', b_prop: 12000, b_dry: 900, b_thrust_kn: 380, b_isp: 285,
    b_engines: 2, b_propellant: 'kerolox', b_diameter: 1.53, b_throttle: 60,
    payload_kind: 'capsule', pod_mass: 350, cargo_mass: 0, crewed: 1,
  },
  'Falcon-class (Merlin)': {
    vname: 'Falcon-class', nstages: 3, nboost: 0, diameter: 3.7, fairing: 1900, fairing_on: 1, kick_deg: 6, opt_kick: 1,
    s1_engine: 'merlin_1d', s1_engines: 9, s1_prop: 411000, s1_dry_auto: 1, s1_diameter: 3.7,
    s2_engine: 'merlin_1d_vac', s2_engines: 1, s2_prop: 108000, s2_dry_auto: 1, s2_diameter: 3.7,
    s3_engine: 'aj10', s3_engines: 1, s3_prop: 2200, s3_dry_auto: 1, s3_diameter: 2.4,
    payload_kind: 'capsule', pod_mass: 4200, pod_dia: 3.66, cargo_mass: 0, crewed: 1,
  },
  'Saturn V (Apollo)': {
    vname: 'Saturn V', nstages: 3, nboost: 0, diameter: 10.1, fairing_on: 0, kick_deg: 1, opt_kick: 1,
    s1_engine: 'f1', s1_engines: 5, s1_prop: 2077000, s1_dry: 130000, s1_dry_auto: 0, s1_diameter: 10.1,
    s2_engine: 'j2', s2_engines: 5, s2_prop: 451000, s2_dry: 36000, s2_dry_auto: 0, s2_diameter: 10.1,
    s3_engine: 'j2', s3_engines: 1, s3_prop: 106600, s3_dry: 13500, s3_dry_auto: 0, s3_diameter: 6.6,
    l_dry: 3500, l_prop: 9000, l_thrust_kn: 45, l_isp: 311, l_throttle_min: 10, l_diameter: 4.2,
  },
  'Starship (expendable)': {
    vname: 'Starship', nstages: 2, nboost: 0, diameter: 9.0, fairing_on: 0, kick_deg: 5, opt_kick: 1,
    s1_engine: 'raptor_2', s1_engines: 33, s1_prop: 3400000, s1_dry_auto: 1, s1_diameter: 9.0,
    s2_engine: 'raptor_2', s2_engines: 6, s2_prop: 1200000, s2_dry_auto: 1, s2_diameter: 9.0,
    payload_kind: 'capsule', pod_mass: 12500, pod_dia: 4.0, cargo_mass: 0, crewed: 1,
  },
};

/** Mission catalogue for the flight-plan dialog: mission keys only. */
export const MISSIONS = [
  { id: 'hop', mode: 'suborbital', group: 'Suborbital', label: 'Kármán line hop',
    desc: 'Straight up past 100 km and back down under parachutes.', dv: 2.0,
    params: { mode: 'suborbital', sub_profile: 'hop', sub_apogee_km: 100 } },
  { id: 'shot', mode: 'suborbital', group: 'Suborbital', label: 'Downrange 400 km',
    desc: 'A lofted ballistic arc to a splashdown 400 km out.', dv: 2.6,
    params: { mode: 'suborbital', sub_profile: 'downrange', sub_range_km: 400, sub_loft_deg: 40 } },
  { id: 'leo', mode: 'orbit', group: 'Earth orbit', label: 'Low Earth orbit',
    desc: 'Insertion into a 200 km parking orbit, then circularise.', dv: 9.3,
    params: { mode: 'orbit', orbit: 'leo', n_orbits: 2, deorbit: 0 } },
  { id: 'leo_ret', mode: 'orbit', group: 'Earth orbit', label: 'LEO and home',
    desc: 'Three revolutions, a deorbit burn, entry and splashdown.', dv: 9.5,
    params: { mode: 'orbit', orbit: 'leo', n_orbits: 3, deorbit: 1 } },
  { id: 'polar', mode: 'orbit', group: 'Earth orbit', label: 'Polar orbit',
    desc: 'A 90° inclination orbit — launch azimuth due north.', dv: 9.6,
    params: { mode: 'orbit', orbit: 'polar', n_orbits: 2, deorbit: 0 } },
  { id: 'molniya', mode: 'orbit', group: 'Earth orbit', label: 'Molniya',
    desc: 'Highly elliptical, 63.4° — apogee hanging over the north.', dv: 11.7,
    params: { mode: 'orbit', orbit: 'molniya', n_orbits: 2, deorbit: 0 } },
  { id: 'geo', mode: 'orbit', group: 'Earth orbit', label: 'Geostationary',
    desc: 'Transfer, then circularise and remove the inclination at apogee.', dv: 13.6,
    params: { mode: 'orbit', orbit: 'geo', n_orbits: 2, deorbit: 0 } },
  { id: 'flyby', mode: 'flyby', group: 'The Moon', label: 'Circumlunar free return',
    desc: 'Trans-lunar injection, around the far side, and home to a 10.6 km/s entry.', dv: 12.5,
    params: { mode: 'flyby' } },
  { id: 'landing', mode: 'landing', group: 'The Moon', label: 'Lunar landing',
    desc: 'Lunar orbit insertion, descent-orbit burn and a guided powered descent. Needs a lander.', dv: 12.5,
    params: { mode: 'landing' } },
];

/** [id, name, latitude°, longitude°] — the panel's LAUNCH_SITES. */
export const LAUNCH_SITES = [
  ['cape', 'Cape Canaveral', 28.5, -80.6], ['vandenberg', 'Vandenberg', 34.6, -120.6],
  ['wallops', 'Wallops', 37.9, -75.5], ['baikonur', 'Baikonur', 45.9, 63.3],
  ['kourou', 'Kourou', 5.2, -52.8], ['tanegashima', 'Tanegashima', 30.4, 131.0],
  ['sriharikota', 'Sriharikota', 13.7, 80.2], ['wenchang', 'Wenchang', 19.6, 110.9],
  ['mahia', 'Mahia', -39.3, 177.9],
];
/** The simulator's launch_h for a local mean solar time at a site:
 *  launch_h = 0 is Greenwich noon, so local time = 12 + launch_h + lon/15. */
export function launchHourFor(lon, localHour = 9.5) {
  const h = ((localHour - 12 - lon / 15) % 24 + 24) % 24;
  return Math.round(h * 4) / 4;
}
export const LANDING_SITES = [
  ['auto', 'Free-return site (auto)'], ['tranquillity', 'Mare Tranquillitatis · Apollo 11'],
  ['procellarum', 'Oceanus Procellarum · Apollo 12'], ['framauro', 'Fra Mauro · Apollo 14'],
  ['hadley', 'Hadley Rille · Apollo 15'], ['descartes', 'Descartes · Apollo 16'],
  ['taurus', 'Taurus–Littrow · Apollo 17'], ['fecunditatis', 'Mare Fecunditatis · Luna 16'],
  ['procellarum9', 'Oceanus Procellarum · Luna 9'], ['aitken', 'South Pole–Aitken · Chang’e 4'],
];

/** Vertical layout for drawing the stack: every part with its y extent in
 *  metres (y = 0 at the bottom of stage 1, growing up), its diameter, and
 *  the stage it belongs to. Engines are drawn with a bell whose size follows
 *  thrust; decouplers are thin rings. Purely visual. */
export function layout(asm, catalogue) {
  const a = analyse(asm, catalogue);
  const rows = [];
  const stageOf = new Map();
  a.stages.forEach(s => s.seg.forEach(p => stageOf.set(p.id, s.k)));
  // walk bottom-up
  let y = 0;
  const parts = asm.parts.slice().reverse();
  let below = null;
  for (const p of parts) {
    let h, d;
    if (p.type === 'tank') { h = p.length; d = p.diameter; }
    else if (p.type === 'engine') {
      const e = engineForPart(catalogue, p);
      const st = a.stages.find(s => s.engine === p);
      d = st ? st.diameter : 1.8;
      const per = e ? e.thrust_kn : 100;
      h = Math.max(0.6, Math.min(5.5, 0.75 * Math.sqrt(per / 100) + 0.4)) * Math.min(1, d / 1.2 + 0.2);
    } else if (p.type === 'decoupler') { d = below ? below.d : 1.8; h = Math.max(0.25, 0.12 * d); }
    else if (p.type === 'fairing') { continue; }
    else if (p.type === 'payload') {
      if (p.kind === 'lander') { d = p.lander.diameter; h = 0.9 * d; }
      else if (p.kind === 'bus') { d = Math.max(0.8, 0.5 * Math.cbrt(p.mass)); h = 0.8 * d; }
      else { d = p.diameter; h = 0.85 * d; }
    } else continue;
    rows.push({ part: p, y0: y, y1: y + h, d, stage: stageOf.get(p.id) || 0 });
    below = { d };
    y += h;
  }
  const fair = a.fairing;
  let fairingRow = null;
  if (fair) {
    const pay = rows.find(r => r.part.type === 'payload');
    const under = rows.filter(r => r.part.type !== 'payload').pop();
    const d = Math.max(pay ? pay.d * 1.15 : 1.8, under ? under.d : 1.8);
    const y0 = pay ? pay.y0 : y;
    fairingRow = { part: fair, y0, y1: y0 + Math.max(d * 2.2, (pay ? pay.y1 - pay.y0 : 0) + d * 1.4), d };
    y = Math.max(y, fairingRow.y1);
  }
  return { rows, fairing: fairingRow, height: y, analysis: a };
}
