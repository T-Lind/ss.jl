# Cameras that survive the frame they fly in, and an entry that looks hot

*Design — 2026-07-25*

## 1. Why

Three of the launch page's cameras break during a lunar free-return, and they
break for one shared reason. A fourth complaint — that the re-entry heating
looks like orange blobs — is unrelated, and turns out to be the cheapest thing
on the page to fix well.

Reported:

* the **free** camera stops responding to drag once the kick stage is
  jettisoned,
* the **onboard** camera disappears at second-stage jettison,
* the **wide** camera "just shows clouds" later in the flight,
* the re-entry heating deserves better, but must stay efficient.

## 2. Evidence

Measured on this machine against `scripts/panel.jl` on port 8138, driving a
nominal free-return through the page's own JS context. Not inferred from
reading.

### 2.1 One root cause under three symptoms

`sphereMap(dr, h)` anchors the pad and entry scenes at the launch/splash site
and wraps the trajectory onto a sphere:

```js
const th = dr/RE, R = RE + h;
pos = [R*Math.sin(th), R*Math.cos(th) - RE, 0]
```

**World-Y is altitude only near the frame's anchor.** Once the vehicle is far
downrange, Y and altitude diverge without limit. Several places treat them as
the same number.

At entry interface the capsule is ~53° of arc from the splash point:

```
h = 133.9 km        pos = [5197268, -2459149, 0]
```

Its Y is **−2,459,149** — and the run is a skip entry, so by t+1399 s it
reaches −12,364,047.

### 2.2 The free camera is clamped two thousand kilometres away

Kick-stage jettison (t = 562104.9) falls *inside* the ENTRY phase (t0 =
562101), so this is `cameraEntry`. Its last line:

```js
if (eye[1] < 2) eye[1] = 2;     // meant to stop the camera going underwater
```

Sampling `cameraEntry` in free mode across the entry:

| t−tEI | h (km) | pos.y | clamped | eye→target |
|------:|-------:|------------:|:-------:|-----------:|
| 0     | 140.0  | −2,411,588  | yes     | 2,412 km   |
| 29    | 110.2  | −2,646,800  | yes     | 2,647 km   |
| 299   | 155.3  | −4,708,904  | yes     | 4,709 km   |
| 699   | 400.6  | −7,778,567  | yes     | 7,779 km   |
| 1399  | 842.4  | −12,364,047 | yes     | 12,364 km  |

The eye is slammed from −2.4 M to +2 and orbits its target from thousands of
kilometres out. A full drag moves it ±28 m against that baseline — about
0.0007° of view change. The camera is not ignoring input; it is responding
invisibly.

The pad world is unaffected (`free_dist` stays 117 m throughout ascent), which
is why it "works at the start".

### 2.3 The wide camera is a sea-level tripod watching a dot

`cameraEntry`'s wide branch pins the eye to world y = 4:

```js
eye = V.add([es.pos[0], 4, 0], V.scale(side, 120));
```

Measured, that eye is **not** at sea level — its true altitude is 1,833–2,819
km — and it sits 2,394–5,562 km from the capsule with `fov` pinned at its 0.02
floor for the entire entry. What fills the screen is the planet pass rendering
Earth from ~1,800 km up through a 0.02-rad lens: cloud tops, no capsule.

Ascent has the same defect. From t = 100 s the pad tripod is 52 km from the
stack, reaching 1,555 km by SECO, `fov` at the floor throughout — and
`autoPickPad` selects `wide` for `t < secoT − 35` (up to t = 432), so auto mode
walks into it too.

`deckFade` is fed `cam.eye[1]` at both call sites and so returns **1.0** at
every sample above, when true altitude should have faded the deck out entirely
above 45 km. That is a separate, real bug in the same family.

### 2.4 There is no onboard camera after staging

At SECO the world switches pad → eci, and `cameraEci` has no onboard branch —
it aliases `'onboard'` to `'moonframe'`:

```js
} else if (mode === 'moonframe' || mode === 'onboard') {
```

Measured, that is an exterior camera 39.8 m off the hull. The genuine
body-mounted view exists only in `cameraPad`. `cameraEntry`'s `onboard` is
likewise an exterior 4.5 m stand-off, not a hull mount.

### 2.5 The heating budget is not where it looks

From [[ssjl-launch-page-is-fragment-bound]], measured 2026-07-24: the full
frame costs 136.8 ms, of which the planet pass is ≈84 ms and the cloud pass
≈52 ms. Issuing *no draw calls at all* costs 16.8 ms. **The meshes, particle
system, decal sheet and HUD together cost under 0.1 ms.**

So "efficient" here does not mean a cheap effect. Mesh and particle work is
free; the only rule that matters is **add no third fullscreen pass**.

## 3. Design

### 3.1 An altitude helper, and two corrected clamps

Add beside the camera code:

```js
// Pad/entry frames put the sphere centre at (0,-RE,0): altitude is radial,
// not vertical. Y is altitude only directly over the frame's anchor site.
const altAt = p => Math.hypot(p[0], p[1] + RE, p[2]) - RE;
const liftToAlt = (p, minA) => {          // push radially out, never down
  const c = [p[0], p[1] + RE, p[2]], r = Math.hypot(c[0], c[1], c[2]);
  if (r - RE >= minA) return p;
  const s = (RE + minA)/r;
  return [c[0]*s, c[1]*s - RE, c[2]*s];
};
```

`cameraEntry`'s clamp becomes `eye = liftToAlt(eye, 2)`, and `cameraPad`'s
`if (eye[1] < 1.2 && st.h < 100)` gets the same treatment. The intent —
don't put the camera underwater or underground — is preserved exactly, and
stops firing at altitude.

`deckFade` is fed `altAt(cam.eye)` at both `drawClouds` call sites.

**`uCamY` stays world-Y.** The cloud shader uses it against the flat deck
plane at `uY = DECK_H` (`dh = abs(uCamY - uY)`, `under = step(uCamY, uY)`);
altitude there would be geometrically wrong. Only `deckFade`'s input changes.

### 3.2 The free camera orbits in the local frame

Even unclamped, the orbit offset is built on world axes, so at 53° downrange
"drag up" moves 53° away from the local vertical. Build the basis from the
local frame the way `cameraEci` already does with `cs.bx`/`cs.bz`: local
vertical from `es.up`, and the flight tangent across it.

### 3.3 Wide: tripod, then stand-off

Keep the tripod while it is genuinely useful — that is the launch framing
worth preserving — and cross-fade over a range band (no cut) to a stand-off
camera that frames the vehicle against the limb: offset to the side and
slightly above, distance scaled off vehicle length so it stays a wide shot
instead of a floor-clamped dot. One `standoffCam()` helper serves both the pad
and entry wide branches.

### 3.4 A real hull camera

`capStations()` already exposes the capsule geometry and `poseToWorld()`
already turns a model-space pose into a world camera — `cabinCam` is the same
pattern with an interior pose. `hullCam()` adds an exterior one: mounted on the
pod flank on a short boom, angled aft-and-down, so the hull fills one edge of
the frame and the Earth moves past below.

Wired into `cameraEci` and `cameraEntry`, `onboard` then works in every phase.
`moonframe` stays untouched for the auto director at flyby; only the manual
`onboard` chip changes.

### 3.5 The plasma sheath

Three parts, no new fullscreen pass:

**Hull incandescence** — `uHeat` (0..1 from `es.q`) and `uHeatV` (flow
direction in view space) added to the existing MESH fragment shader. Windward
faces glow on a blackbody ramp weighted by `dot(n, uHeatV)`, with a hotter
shoulder rim term. This is the largest visual gain and costs a handful of ALU
ops on pod fragments only — **zero new draw calls**.

**Bow shock** — a small procedural paraboloid cap (~200 tris) standing off the
heat shield, additively blended, scaled and coloured by `es.q`.

**Wake** — `spawnEntry` retuned: many small particles on a new plasma kind (4)
with its own ramp, driven by `es.q` rather than the shader's hardcoded orange,
so peak heating reads white-hot and the trail cools to deep red.

Particles are round `gl_PointSize` sprites and cannot stretch, so true streaky
filaments would need a small quad buffer. **Deferred** — build the three above,
then judge whether the look needs it. Not built speculatively.

### 3.6 What was actually built

Two things landed differently than designed, both after looking at the result:

**The stagnation gradient came off position, not the normal.** Driving the
falloff with `pow(dot(n, flow), 5)` rendered the shield's own tessellation as
concentric bands — each ring of facets is a discrete step in `w`. The shipped
version takes the radial distance off the flow axis in view space
(`uHeatC`, `uHeatR`), which is smooth, facet-independent, and is also the real
shape of stagnation-point heating. The normal survives only as a soft windward
gate.

**The bow-shock cap mesh was not built.** The shock-layer particles plus the
hull term already read as a standing shock, so the extra mesh, program bind and
blend-state change bought nothing. Deferred on the same terms as the streak
buffer.

A third change is a plain bug fix that fell out of the framing work: the wide
camera's tripod→stand-off blend was first written with a wide range band, which
parked the eye a few km back holding the vehicle's own span — a 1.6 deg lens.
The fov dipped and then snapped open, reading as a zoom glitch. Handing over
earlier closes the distance faster than the span shrinks, so the shot only ever
opens up (measured: max backward step in fov 0.007, imperceptible).

## 4. Verification

The page's own JS context is the harness, driven over the browser tools.

A probe samples every camera at ~20 points across all seven phases and asserts
invariants that each of the three bugs violates:

* eye→target distance stays within a sane band for the mode,
* eye altitude stays above the surface,
* `fov` never pins to its 0.02 floor,
* no NaN in eye, look, up or fov,
* `deckFade` is 0 whenever true camera altitude is above 45 km.

Before/after screenshots at entry interface cover the parts arithmetic cannot.

**Result:** 126 assertions over 7 camera modes × 3 worlds × 6–7 times each,
0 failures.

One probe assertion was wrong and was corrected rather than the code. It
flagged `pad onboard` for an up vector antiparallel to the view direction
(`dot = -0.9994`). That camera looks back down the stack with `up = st.dir`, so
antiparallel is expected — but the basis is still well conditioned, because the
roll comes from the fixed radial offset: the cross product holds at 0.033 and
the roll axis is stable at ≈(0,0,-1) across the whole ascent, and `st.dir[2]`
is identically 0 (sphereMap keeps the track in the x-y plane) so it can never
align with the offset axis. The real invariant is a well-conditioned basis, not
a perpendicular up. Working code the user is happy with was left alone.

### 4.1 Frame cost — not measured

**Unverified.** Every timing attempt ran in a backgrounded tab
(`document.hidden`), where GPU work is deferred and only JS submission time is
observed — the null render came out *more* expensive than the full one, which
is nonsense. The A/B technique in [[ssjl-dev-environment]] needs a visible
window.

What can be said without measuring: this adds ~500 particles (541 → 1026 at
peak) and a dozen ALU ops per pod fragment, and adds no pass. Against a budget
where the whole mesh + particle path was under 0.1 ms of a 136.8 ms frame, that
should be lost in the noise — but it *should be*, not *is*.

The earlier claim that fixing `deckFade` recovers the 52 ms cloud pass is
likewise **unverified and probably wrong**: in the broken wide view the camera
looks down, away from the deck plane, so the pass was likely cheap regardless.
The user-visible symptom was the planet pass rendering Earth from 1833 km up
through a 0.02 rad lens, not the deck.

## 5. Out of scope

The keydown handler indexes `CAMS` even on lunar-landing missions, where the
chip row uses `CAMS_MOON`; keys 5–7 desync the chips there. Real, but a
different bug on a mission family this design does not touch.
