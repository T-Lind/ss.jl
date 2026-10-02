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
  return result==='touchdown'?'SURFACE':result==='crash'?'IMPACT':result==='tipped'?'TIPPED': 'DESCENT ABORTED';
}
export function lunarHatchReason(run) {
  const result=landingOutcome(run);
  return result==='crash'?'landing impact':result==='tipped'?'lander tipped'
    :result==='propellant'?'propellant depleted':result==='unknown'?'touchdown unconfirmed':'descent '+result;
}
