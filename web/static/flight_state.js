// A terminal descent sample is not automatically a successful landing.
// Prefer explicit metrics; accept the event journal for older stored runs.
export function landingOutcome(run) {
  const outcome=run?.metrics?.outcome;
  if(typeof outcome==='string'&&outcome.trim())return outcome.trim().replace(/^:/,'').toLowerCase();
  const events=run?.events||[];
  for(let i=events.length-1;i>=0;i--) {
    const e=events[i];
    if(e.phase==='lunar'&&['touchdown','crash','tipped','propellant','timeout','diverged'].includes(e.name))return e.name;
  }
  return 'unknown';
}
export const successfulTouchdown=run=>landingOutcome(run)==='touchdown';
export function lunarEndLabel(run) {
  const result=landingOutcome(run);
  return result==='touchdown'?'SURFACE':result==='crash'||result==='surface'?'IMPACT':result==='tipped'?'TIPPED'
    :result==='propellant'?'PROPELLANT DEPLETED':result==='timeout'?'DESCENT TIMED OUT':'DESCENT ABORTED';
}
export function landingEndTime(run, surfaceDuration = 300) {
  return run.site.t_td + (successfulTouchdown(run) ? 16 + surfaceDuration : 0);
}
export function landingSummary(run) {
  const m = run?.metrics || {}, result = landingOutcome(run);
  const height = run?.descent?.h?.at(-1);
  const above = Number.isFinite(height) ? ` at ${Math.max(0, height * 1000).toFixed(0)} m above ground` : '';
  if (result === 'crash' || result === 'surface' || result === 'tipped')
    return `${result === 'tipped' ? 'Lander tipped' : 'Landing impact'}: ${(+m.touchdown_v || 0).toFixed(1)} m/s sink, ` +
      `${(+m.touchdown_vh || 0).toFixed(1)} m/s lateral.`;
  const reason = result === 'propellant' ? 'Propellant depleted' : result === 'timeout' ? 'Descent timed out' : 'Descent stopped';
  const budget = result === 'propellant' && Number.isFinite(m.loi_dv) && Number.isFinite(m.lander_dv)
    ? ` LOI/plane change used ${m.loi_dv.toFixed(0)} m/s; ` +
      `${Math.max(0, m.lander_dv - m.loi_dv - (m.doi_dv || 0)).toFixed(0)} m/s remained for descent.` : '';
  return `${reason}${above}; no touchdown simulated.${budget}`;
}
export function lunarHatchReason(run) {
  const result=landingOutcome(run);
  return result==='crash'?'landing impact':result==='tipped'?'lander tipped'
    :result==='propellant'?'propellant depleted':result==='unknown'?'touchdown unconfirmed':'descent '+result;
}
