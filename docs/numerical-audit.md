# Numerical reliability and platform roadmap

The current reference missions complete successfully, but the audit reproduced
errors in targeting and orbit-state conversion that the original mission tests
did not cover. This change fixes those errors, adds independent invariant tests,
and makes cross-engine checks mandatory in CI. The simulator remains a conceptual
mission tool: agreement between its two implementations does not establish
agreement with real flight data.

## Reproduce the calculations

```bash
julia --project=. -t 4 test/runtests.jl
julia --project=. -t 4 scripts/verify.jl --full --samples=300
node scripts/check_static.mjs
node --test test/*.test.mjs
(cd web && SSJL_REQUIRE_JULIA=1 npm test)
```

The verification command writes `output/verification.toml` without overwriting
existing trajectory products. It records the source revision, dirty-tree status,
Julia version, platform, thread count, mission-spec hash, sample count and seed,
plus every checked value and acceptance interval. It exits nonzero if any
mission or check fails. `--output=PATH` chooses another report location;
`--help` lists options. CI retains the report and freshly generated mission
products as downloadable artifacts.

The committed `output/` CSVs, plots and viewer are historical snapshots, some
predating the lifting capsule and payload-diameter changes. They are useful
illustrations, but the fresh report is the source for current numerical results.
Exact CSV equality across runtime releases is replaced by physical tolerances,
independent propagation checks and cross-engine parity. Regenerate plot inputs
and plots together when publishing a new release snapshot.

## Measured reference results

Measured with Julia 1.10.10 on Linux, four threads, current defaults,
`missions/moonshot.toml`, and Monte Carlo seed 2026. These are model outputs,
not external observations. The broad acceptance limits in `scripts/verify.jl`
check mission success; the stricter numerical invariant and resolution tests
remain in `test/runtests.jl` and `test/numerical_regressions.jl`.

| Calculation | Measured result |
| --- | --- |
| Targeted LEO entry peak deceleration | 4.2945 g |
| LEO peak stagnation heating | 28.65 W/cm² |
| LEO heat load | 112.3 MJ/m² |
| LEO splash speed and target miss | 4.524 m/s and 7.522 km |
| LEO 300-sample footprint | 300 splashdowns, CEP50 271.69 km, R95 833.05 km |
| LEO population covariance semi-axes | 415.34 × 16.33 km |
| Circumlunar perilune | 1999.976 km |
| Circumlunar return vacuum perigee | 50.202 km, first Earth passage |
| Circumlunar mission duration and peak load | 6.587 days and 5.678 g |
| Circumlunar peak heating and heat load | 250 W/cm² and 271 MJ/m² |
| Earth LEO achieved perigee | 201.740 km |
| 100 km suborbital hop | 99.944 km apogee, 2.683 km ground range |
| Terrain landing propellant remaining | 1415.15 kg |
| Lunar rendezvous total delta-v | 24.469 m/s |
| Return orbiter propellant remaining | 481.58 kg, successful Earth splashdown |
| CW rendezvous under nonlinear propagation | 113 m arrival miss |

Halving the circumlunar coast timescale factor from 0.005 to 0.0025 changes
perilune by 0.90 m, return vacuum perigee by 74.23 m, and peak entry load by
0.0146 g. The existing resolution test still enforces its tolerances. Timing is
excluded as an accuracy criterion because compilation and host load vary.

## Correctness changes

- **Orbital phase survives singular element conventions.** Circular orbits
  previously returned `nu=0` regardless of position, giving an 8316 km
  Cartesian round-trip error in an inclined 500 km orbit. Circular orbits
  now carry argument of latitude in `nu`; equatorial orbits retain longitude
  of periapsis or true longitude, including retrograde cases.
- **Scalar targeting reports an actual evaluation.** Failed endpoint retries
  used nine calls with a budget of two. Interior failures fabricated residuals
  and destroyed the root bracket. Budgets now include retries; failed interior
  missions stop as `infeasible`; exhausted budgets report `max_iter`. Illinois
  interpolation weights never replace measured values. An interval-converged
  answer comes from the final bracket, and interrupts propagate in Julia.
- **Short Lambert transfers remain feasible.** The previous bracket search
  discarded the hyperbolic branch and rejected a 100 second quarter-plane
  transfer. Independent two-body RK4 propagation now verifies the returned
  endpoint position and velocity. `max_iter` controls the solve and exhausted
  budgets raise an error. Collinear geometry is explicitly unsupported.
  Lambert transfers can be elliptic, parabolic or hyperbolic; this is consistent
  with [NASA's unified formulation](https://ntrs.nasa.gov/citations/19680026116).
- **Footprints wrap longitude.** Splashdowns at 179.9° E and 179.9° W now
  have a local 11.12 km major sigma rather than a nearly global footprint
  centered at Greenwich. Empty and all-failed samples return failure counts
  and unavailable metrics; missing target distances do not invalidate
  dispersion metrics. The ellipse remains a local tangent-plane approximation,
  unsuitable for a global or multimodal footprint.
- **Reentry respects the end epoch.** Four- and six-DOF drivers cap the last
  step at `t_max`, retain the timeout state, include initial entry loads in
  peaks, and reject zero/negative/non-finite integration steps. The browser
  driver follows the same contract.
- **Deorbit results correspond to returned elements.** An exhausted targeting
  budget returns the last flown elements, rather than a correction that was
  never simulated alongside its predecessor's result.

The analysis form exposes the evaluation budget and distinguishes an exhausted
search, an absent sign change and a failed mission. Failed values serialized as
JSON null are excluded from its search plot. Both frontends receive the same UI
changes.

## Verification infrastructure

Parity previously treated any Julia reference failure as a missing runtime.
The shared harness now skips only an absent optional executable; CI requires
Julia. Runtime errors, timeouts, buffer limits and invalid JSON fail the test.
This distinction uses the documented child-process error/status contract in
[Node's API](https://nodejs.org/api/child_process.html).

CI runs every parity test, including solver histories and failure statuses.
Independent state round trips and propagation tests check facts that two matching
implementations could both get wrong. Static checks cover both frontends, their
relative imports, core browser modules and inline JavaScript. A stale HTTP test
now requests the shipped lunar groundtrack module.

## Proposed next platform work

These are follow-up proposals, not capabilities added by this PR.

1. **Make a run a portable scientific record.** Version the mission input and
   output schemas; record engine version, model choices, units, tolerances and
   random seed on every run. Add JSON import/export and CSV bundles, migrations
   for old saved vehicles, and a replay command. Acceptance: a saved run replays
   on another machine and any model-version mismatch is visible.
2. **Finish parity coverage before expanding physics.** Add explicit parity
   for Earth-return rendezvous and uncertainty analysis, and independent
   golden cases from published orbital examples. Keep a documented coverage
   matrix for Julia-only capabilities such as six-DOF entry. Acceptance:
   every advertised mission family has a cross-engine test or a visible
   implementation limitation.
3. **Add cancellation and bounded execution.** Browser missions already use
   a worker, but lack a user cancellation contract. Add cancellation to worker
   jobs and Julia requests, bounded queues, retained failure/progress records,
   and explicit computation budgets. Acceptance: cancelling a large sweep
   frees capacity, ends progress coherently and preserves finished evaluations.
4. **Give comparisons a durable workflow.** Build on the current run history
   with named mission scenarios, pinned baselines, side-by-side metric deltas,
   parameter diffs, and trajectory overlays. Acceptance: users can explain a
   changed result from its changed inputs and models without rerunning both.
5. **Separate conceptual physics from validated models.** Introduce named
   fidelity profiles and publish calibration/validation datasets. Compare
   cislunar propagation with a trusted ephemeris, then add thrust-direction
   limits, finite burn attitude/RCS costs and lunar abort paths. Acceptance:
   each added model states its applicable regime and passes an independent
   benchmark with a reported numerical error.
6. **Strengthen uncertainty claims.** Store every sampled input, distinguish
   failed missions from censored outcomes, attach uncertainty intervals to
   percentiles, and support correlated dispersions. Keep global or multimodal
   footprints out of a single local covariance ellipse. Acceptance: reported
   reliability and tail budgets are reproducible and carry sample-size limits.
7. **Version release evidence.** Publish verified reports and plot bundles
   beside binaries, with their exact code revision and input files. Automate
   regeneration of all related plots together. Acceptance: each published
   figure names the run that generated it and release artifacts agree with
   that run's report.

Prioritize portable run records and cancellation next: they improve everyday
use while making later physics, analysis and release changes easier to verify.
