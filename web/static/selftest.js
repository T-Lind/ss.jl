// The in-page assertion harness, shared by all three pages.
//
// Each page keeps its own assertions — they are about that page's physics and
// rendering — but the plumbing was copied three times verbatim. Activated by
// ?selftest=1; the summary line is what the Julia HTTP tests scrape, so its
// shape ("__selftest: N passed, M failed") is load-bearing and must not drift.

/** A fresh assertion set.
 *
 *  Values are produced by a thunk so a missing or throwing function is a
 *  readable failure rather than a crash that hides every assertion after it.
 *  Exact equality is tried first, so non-numeric answers (a class name, a
 *  formatted string, a boolean) can be asserted with the same helper; the
 *  tolerance path is for numbers only. */
export function suite() {
  const out = { pass: 0, fail: 0, failures: [] };
  out.eq = (name, fn, want, tol) => {
    let got;
    try {
      got = fn();
    } catch (e) {
      out.fail++;
      out.failures.push(`${name}: threw ${e.message}`);
      return;
    }
    if (got === want || Math.abs(got - want) <= (tol === undefined ? 1e-9 : tol)) out.pass++;
    else {
      out.fail++;
      out.failures.push(`${name}: got ${got}, want ${want}`);
    }
  };
  return out;
}

/** Publish and print the result, then hand it back.
 *
 *  Three things, because the Julia HTTP tests and the browser driver read
 *  different ones: `__selftestResult` for a driver that can evaluate script,
 *  the summary line for the log scraper, and a separate machine-readable
 *  failure line — console.log's second argument is rendered by the devtools
 *  formatter and does not survive being read back as text, so a failing
 *  assertion would otherwise report a count with no names. */
export function report(out) {
  window.__selftestResult = out;
  console.log(`__selftest: ${out.pass} passed, ${out.fail} failed`,
              out.failures.length ? out.failures : '');
  if (out.failures.length)
    console.log('__selftestFailures ' + JSON.stringify(out.failures));
  return out;
}

/** True when the page was asked to test itself. */
export const requested = () =>
  new URLSearchParams(location.search).get('selftest') === '1';
