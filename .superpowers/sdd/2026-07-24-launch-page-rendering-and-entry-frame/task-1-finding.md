# Task 1 finding — the remaining entry compositing defect

**Verdict: none of the four suspects in the brief is the cause. `earthCol` is never
called in this frame.** `covE` is bounded below `7.69e-6` at every pixel — 260x under
the `covE > 0.002` gate at `:904` that would call it — so nothing in the Earth/cloud
compositing path can be responsible. The frame is 100% `skyCol`, and `skyCol`
contains one genuine, reproducible shader defect — a hard seam — which is named below
with the literal replacement line. (That bound, and why it is a bound rather than an
equality, is derived in section 1: a `readPixels` byte cannot express zero.)

Frame under test: `simT = 562261`, `ENTRY` phase, camera forced to `chase`
(the brief's frame). Panel on port 8138, WebGL 1. Measured at three canvas sizes
(2560x1347, 1280x720, 1920x1080); every result below reproduced at all three.

**Source read.** The brief scoped reading to `scripts/launch_page.html:597-908` (the
`SPACE` program). Establishing the camera-aim root cause in section 1 and the
pad/eci safety argument in section 4 required going outside that range as well:
`:178-198` (`program()`, for the bisect harness), `:1340-1376` (the phase table and
`phaseAt`), `:1437-1447` (`entState`), `:3101-3258` (`autoPickPad`, `cameraPad`,
`cameraEci`, `autoPickEntry`, `cameraEntry`), `:3396-3500` (`buildScene`), and
`:3599-3660` (`drawScene`'s uniform setup). This finding therefore rests on more
than the fragment shader.

---

## 1. The measurement that eliminates all four suspects

Recompiled the `SPACE` fragment program with the final write replaced by a probe of
the coverage terms, captured the framebuffer immediately after the planet pass
(hooked `gl.drawArrays` on the 6-vertex fullscreen triangle pair, so the capsule
mesh cannot overdraw the probe), and scanned every pixel.

### What a byte readback can and cannot establish

WebGL 1 `gl.readPixels` accepts only `UNSIGNED_BYTE` for the default framebuffer, so
a probe cannot return a float. The write is quantised as
`byte = round(clamp(v, 0, 1)*255)`, so **a zero byte does not prove `v == 0`** — it
proves only `v <= 0.5/255 = 1.96e-3`. Taken alone that is uncomfortably tight against
the shader's own threshold: `:904` gates on `covE > 0.002`, and 1.96e-3 < 2.0e-3 by
only 2% of one quantisation step. Sufficient, but not a margin worth resting a
conclusion on.

So the probe was re-run with the value **pre-scaled by 255** inside the shader,
`gl_FragColor = vec4(covE*255.0, ca*255.0, covM*255.0, 1.0)`, which buys 255x more
resolution near zero. A zero byte then bounds `covE*255 <= 0.5/255`, i.e.
`covE <= 0.5/255^2 = 7.69e-6`:

```
scaled probe (x255), 1920x1080:
  pixels 2073600   maxCovE_byte 0   maxCa_byte 0   maxCovM_byte 0   nonZeroPixels 0
unscaled probe, 1280x720:    pixels  921600   maxCovE 0  maxCa 0  maxCovM 0
unscaled probe, 2560x1347:   pixels 3448320   maxCovE 0  maxCa 0
```

**The defensible claim is therefore `covE <= 7.69e-6` at every pixel** — a factor of
260 below the `:904` threshold of 0.002, not a factor of 1.02. The same bound holds
for `ca` (vs the `ca < 0.997` term) and for `covM` (vs `:903`'s `covM > 0.002`).

Because `covE` cannot exceed 7.69e-6 anywhere, the guard at `:904`
(`if (covE > 0.002 && ca < 0.997)`) provably never fires, so:

- **Suspect 1 — `covE` falls short of 1 (`limbCov`, `:639`)**: eliminated as
  *the cause of the flat blue-grey*. `covE` is not "short of 1", it is below
  7.69e-6, and it is that low because the ray genuinely misses the planet, not
  because `limbCov` is misbehaving. `limbCov`'s pixel-footprint term
  `uPxA*dot(c,d)` is in fact exactly right: `dot(c,d) == |c|cos(theta)` is
  precisely `d(impact parameter)/d(angle)`.
- **Suspect 2 — `skyCol` bleeding through the `opaque > 0.997` guard (`:901`)**:
  `opaque = max(ca, max(covE, covM)) <= 7.69e-6`, so `skyCol` is not "bleeding
  through", it is the *only* thing in the frame, correctly. The guard behaves as
  written.
- **Suspect 3 — `earthCol` returning near-flat blue (`:647-760`)**: `earthCol` is
  never executed. Not the cause.
- **Suspect 4 — settlement lights over the day side (`:744`)**: inside `earthCol`.
  Never executed. Not the cause.

### Why there is no Earth in the frame (this is camera aim, not the shader)

Measured for this frame (full precision, computed from the camera basis rather than
from rounded degrees):

| quantity | value |
|---|---|
| camera altitude (`length(uEc) - uER`) | 84,421.837 m |
| angle from `uEc` direction to camera forward (`thetaFwd`) | 90.97237 deg |
| Earth's limb (`asin(uER/length(uEc))`) | 80.72365 deg |
| half-FOV (`cam.fov/2`, fov = 0.30 rad) | 8.59437 deg |

The frame spans `theta` in [82.378, 99.567] deg. Every ray with
`theta > 80.724` deg misses the planet. **The Earth's limb is 1.654 deg below the
bottom edge of the frame.**

> **The goal is Earth pixels on screen, not the limb inside the frame.** Task 1b's
> audit of all seven entry cameras establishes that a limb off the **top** of the
> frame (`y_limb > +1`) is the *healthy* case — it means the Earth fills the frame —
> and only a limb off the **bottom** (`y_limb < -1`) is mis-framed. Do not read the
> "1.654 deg below the bottom edge" above as "move the limb into `[-1, 1]`". The
> correct diagnostic pairs the limb's screen position with an actual Earth-pixel
> fraction, which is exactly what the coverage scan in section 1 measures: not one of
> 2,073,600 pixels rises above the 7.69e-6 detection floor, let alone the shader's
> 0.002 gate. This frame is the `y_limb < -1` case, and it is mis-framed because the
> Earth-pixel fraction is zero, not because the limb is out of view.

`cameraEntry`'s chase branch, `scripts/launch_page.html:3239-3241`:

```js
if (mode === 'chase') {
  eye = V.add(p, V.add(V.scale(side, 34), V.scale(es.vdir, -10)));
  fov = 0.30;
}
```

with `look = p` (`:3233`) and `side = [0,0,1]` (`:3234`). The eye is 34 m to the
side and only 10 m behind, so the forward vector is a side-on view of the capsule,
within 1 deg of the local horizontal (`up = es.up` is the local vertical). At 84 km
the horizon has dropped `arccos(uER/length(uEc)) = 9.3` deg below the local
horizontal — more than the 8.6 deg half-FOV. So the ground cannot be in frame.

**This is not a compositing bug and Task 2 cannot fix it in the shader.** Getting
Earth pixels into the entry chase shot (not "the limb into the frame" — see the note
above) requires a camera change: bias `look` below the
capsule, widen `fov`, or raise the eye. This is the direct answer to "flat blue-grey
instead of a lit Earth with clouds" — the Earth is off-frame.

---

## 2. The genuine shader defect: a hard seam at `dot(uEc, d) == 0`

### The exact expression and line number

`scripts/launch_page.html:787` — the `if (b > 0.0)` gate in `skyCol`, together with
the line-based (not ray-based) closest approach on `:788`:

```glsl
786    float b = dot(uEc, d);
787    if (b > 0.0) {
788      float dmin = sqrt(max(dot(uEc,uEc) - b*b, 0.0)) - uER;
789      float g1 = exp(-max(dmin, 0.0)/55000.0);
790      col += (vec3(0.30,0.52,0.92)*g1 + vec3(0.9,0.5,0.3)*exp(-max(dmin,0.0)/16000.0)*0.35)
791             * (1.0 - dens) * 1.05;
792    }
```

`dmin` is the closest approach of the **infinite line** to the sphere. The
`if (b > 0.0)` gate exists to stop that closest approach being used when it lies
*behind* the camera — without it the term would come back, brighter, as the ray
turns toward the anti-Earth direction. But the gate is a hard cut, not a fade:

- at `b = 0+`: `dmin = length(uEc) - uER = camAlt`, so the term contributes
  `(0.30,0.52,0.92) * exp(-camAlt/55000) * (1 - dens) * 1.05`;
- at `b = 0-`: the term contributes exactly zero.

`b = dot(uEc, d) == 0` is the ray direction exactly perpendicular to the camera's
radius vector — **the local horizontal**. Any camera looking within one half-FOV of
the local horizontal therefore has a straight, 1-pixel-wide brightness step drawn
across its frame.

### Pixel readback: before

Column at `x = 0.3*width`, framebuffer rows (`readPixels` origin bottom-left),
single unmodified render of the frozen entry frame:

```
frac 0.02  [64, 92,147]      <- glow, brightest toward the limb
frac 0.10  [43, 67,109]
frac 0.20  [31, 50, 83]
frac 0.30  [26, 41, 69]      <- the brief's sample point
frac 0.40  [23, 38, 63]
row  598   [23, 37, 62]      <- last row below the seam  (frac 0.444)
row  599   [ 0,  0,  0]      <- first row above the seam
frac 0.50  [ 0,  0,  0]
frac 0.95  [ 0,  0,  0]
```

The seam row is where `b` changes sign. Two ways to predict it, and the distinction
matters because only one is a derivation:

- **Approximation (linear in angle):**
  `frac = 0.5 + (thetaFwd - 90)/halfFov * -0.5 = 0.44343`. This treats screen
  position as proportional to off-boresight *angle*, which the projection is not —
  `:885` builds `d = normalize(uF + uR*vP.x*uTan*uAsp + uU*vP.y*uTan)`, so screen
  position is proportional to the *tangent* of the angle. Convenient, but do not read
  its near-match to the measurement as an exact result.
- **Exact:** solve `dot(d, ecn) == 0` for `vP.y` directly, with
  `ecn = normalize(uEc)` and `vP.x = -0.4` for the probe column at `x = 0.3*width`:

  ```
  vP.y = -(dot(uF,ecn) + vP.x*uTan*uAsp*dot(uR,ecn)) / (uTan*dot(uU,ecn))
  frac = (vP.y + 1)/2 = 0.443849
  ```

**Measured: `479/1080 = 0.443519`.** The exact prediction is off by `0.00033`, which
at 1080 rows is **0.36 of a pixel** — the seam lands on the predicted row. The linear
approximation differs from the exact form by `0.00042` (0.45 px), also sub-pixel,
which is why it looked exact; at a narrower FOV or larger off-boresight angle it
would not. The seam frac reproduced at all three canvas sizes tested:
`598/1347 = 0.4440`, `320/720 = 0.4444`, `479/1080 = 0.4435`.

The step size is predicted in closed form. With `camAlt = 84422`,
`dens = exp(-84422/9000) = 8.42e-5`, `g1 = exp(-84422/55000) = 0.2155`:

```
col_below = (0.30,0.52,0.92)*0.2155*1.05  +  (0.9,0.5,0.3)*exp(-84422/16000)*0.35*1.05
          = (0.0679, 0.1176, 0.2081) + (0.0047, 0.0026, 0.0016)
gl_FragColor = pow(col, 0.9)*255 = [23, 37, 62]
```

Predicted `[23,37,62]`; measured `[23,37,62]`. The same closed form reproduces the
whole column, e.g. at `frac 0.10` (`b = 658,720`, `dmin = 50,700`):
predicted `[43,67,109]`, measured `[43,67,109]` — exact on all three channels.

**So the blue-grey slab is 100% this one `col +=` expression.** Control: zeroing it
(`col += 0.0*(vec3(0.30,0.52,0.92)*g1 + ...)`) turns the entire column
`[0,0,0]` at every sampled row, including the ones that read `[64,92,147]` before.
The `day` term at `:784` contributes nothing (`dens*1.25 = 1.05e-4`).

### Pixel readback: after

Forcing the closest approach onto the ray (`b -> max(b, 0.0)`) and removing the
gate makes the term continuous; readback of the same column:

```
frac 0.02  [64, 92,147]   (unchanged)
frac 0.40  [23, 38, 63]   (unchanged)
frac 0.443 [23, 37, 62]
frac 0.46  [23, 37, 62]   <- was [0,0,0]
frac 0.95  [23, 37, 62]   <- was [0,0,0]
```

Seam gone, but the whole upward sky becomes a uniform `[23,37,62]` navy floor —
physically wrong at 84 km, where the zenith should be black. Adding a taper over
the horizon-dip angle removes the floor and keeps the continuity (measured):

```
frac 0.05..0.43   identical to the unmodified render (bit-identical)
frac 0.46  [23, 37, 62]
frac 0.50  [23, 36, 61]
frac 0.70  [13, 22, 36]
frac 0.95  [ 1,  1,  1]      <- zenith back to black
```

### The bright speckles are the starfield — and they are *correct* code

Zeroing the star term at `:807` (`col += st * night * ...` -> `col += 0.0 * st * ...`)
removes every bright pixel: `572` pixels with `R > 95` below the seam and `652`
above become `0` and `0`. `night = smoothstep(0.42, 0.02, dens) = 1.0` at 84 km, so
stars are meant to be there. What is arguable — and is *not* the cause of the
artefact — is that they get no extinction: `:804` computes
`air = dens*min(1.0/(zc+0.10), 6.0)*0.55` from the **camera-local** density
(`8.4e-5`), so a line of sight grazing 30–90 km above the limb dims stars by
`exp(-0) = 1`. That is why the speckles sit at full brightness on top of the blue
band. Cosmetic; note it, do not let it into Task 2's critical path.

---

## 3. The minimal correction

Replace `scripts/launch_page.html:786-787`

```glsl
  float b = dot(uEc, d);
  if (b > 0.0) {
```

with

```glsl
  // closest approach of the RAY, not of the infinite line: at b <= 0 the nearest
  // point is the camera itself, so the shell term is continuous through the local
  // horizontal instead of stepping to zero across it
  float b = max(dot(uEc, d), 0.0);
  {
```

(`b` is dead after `:792`, so clamping it is safe. Keeping the braces keeps `dmin`
and `g1` scoped exactly as before, so nothing else in `skyCol` changes.)

**Recommended together with it** — otherwise the whole upward sky picks up a
uniform `exp(-camAlt/55000)` floor. Append a taper to `:791`, using `dip` already
computed on `:780` and the `up` parameter already in scope:

```glsl
           * (1.0 - dens) * 1.05 * (1.0 - smoothstep(0.0, dip, max(dot(d, up), 0.0)));
```

`dot(d, up)` is 0 exactly at the local horizontal — the same place the clamp takes
over — so the two changes join continuously, and the term fades to zero one
horizon-dip above it. Both were compiled and measured; results in section 2.

---

## 4. Why this camera regime — and the correction to the brief's premise

The brief assumed "the pad world and the `eci` world render the planet correctly
with the same shader". **That is only true of the shots the auto camera picks.**
The seam is not entry-exclusive. It is exclusive to *vehicle-mounted cameras at
roughly 5–250 km altitude looking within one half-FOV of the local horizontal*.

Its amplitude is `(1 - exp(-camAlt/9000)) * exp(-camAlt/55000)`, a product that is
killed at both ends and peaks in the 20–60 km band. Single-render measurements of
the largest one-row downward step in a column. The `pred` column uses the
**linear-in-angle approximation** from section 2, not the exact tangent solve, so
expect it to sit within about a pixel of the measurement rather than on it:

| `simT` | world / phase | camera | camAlt | seam frac (meas / pred approx) | below -> above |
|---|---|---|---|---|---|
| 60 | pad / ASCENT | chase | 11.3 km | 0.674 / 0.674 | `[127,163,235]` -> `[50,66,89]` |
| 80 | pad / ASCENT | chase (auto) | 19.7 km | 0.544 / 0.541 | `[97,130,197]` -> `[21,28,38]` |
| 200 | pad / ASCENT | chase | 86.6 km | 0.353 / 0.335 | `[22,36,60]` -> `[0,0,0]` |
| 562261 | entry / ENTRY | chase | 84.4 km | 0.444 / 0.442 | `[23,37,62]` -> `[0,0,0]` |

So the **pad world has the same defect, worse**, whenever the camera is on the
vehicle. Why nobody has reported it:

- **pad, ground shots** (`pad`, `padlow`, `wide` in `autoPickPad`, `:3101-3114`)
  put the eye at `y = 1.4 .. 3` m. `dens = exp(-camAlt/9000) ~ 1`, so the
  `(1.0 - dens)` factor on `:791` zeroes the whole term. Measured at `simT = 40`
  (auto -> `wide`): the corrected and uncorrected renders differ by
  `maxDiff = 2` over `0.0%` of the frame. Immune, for a real reason.
- **pad, vehicle shots** (`chase` from ~70 s, `onboard` around staging) are *not*
  immune — see the table. This is a falsifiable prediction: the ascent chase view
  between roughly 15 km and 150 km should show the same horizontal seam.
- **eci** is immune two ways. (a) Altitude: `exp(-camAlt/55000) <= 0.026` at and
  above 200 km, so the step is at most `[4,6,9]` out of 255 — measured
  `maxDiff = 19` at `simT = 600` and `16` at `simT = 3400`; beyond ~400 km it
  rounds to zero. (b) Aim: at `simT = 600` the `earthframe` auto camera has
  `thetaFwd = 76.3` deg with a 12 deg half-FOV, so the top edge of the frame reaches
  only `76.3 + 12 = 88.3` deg. The seam sits at `theta = 90` deg — **13.7 deg from
  the boresight, which is 1.7 deg past the top edge of the frame** (the two numbers
  are different quantities; only the second one is the clearance). So the seam is
  outside the frustum and is not drawn at all.
- **entry** is the only phase that *lingers* in the bad band with the camera bolted
  to the vehicle and pointed along the local horizontal, and the only one where the
  seam is the brightest edge in an otherwise black frame.

### Is the correction safe for pad and eci?

Yes, checked by A/B rendering the same frozen frame through the unmodified and the
corrected program and differencing the framebuffers:

- pad ground camera, `simT = 40`: `maxDiff 2`, `0.0%` of pixels. No change.
- pad vehicle camera, `simT = 80`: below the local horizontal, unchanged; above it,
  the `[97,130,197] -> [21,28,38]` step is replaced by a smooth fade. Improvement.
- eci `simT = 600` / `3400`: `maxDiff 19` / `16` — i.e. at most 4-9 levels of 255,
  in the region that was a hard cut before. Improvement, imperceptible either way.
- entry `simT = 562261`: sub-horizontal rows bit-identical, seam gone.

**Method caveat for whoever re-runs this.** A/B differencing is invalid for cameras
that mutate between calls: `cameraEci`'s `drift` does `freeYaw += 0.0009` every
frame, and `cameraPad`/`cameraEntry` add `shakeVec()` driven by `performance.now()`.
No-op control (identical shader both passes): `simT 100000` gives `maxDiff 609` over
3.5% of pixels and `simT 80` gives `maxDiff 478` over 28.1% — pure camera motion.
`simT 562261` chase gives `maxDiff 0` (`es.g = 0.2` puts `shake` under the 0.04
threshold), so every entry-frame A/B number above is clean. The pad/ascent numbers
in the table are single-render within-frame steps for exactly this reason.

---

## 5. What this means for Task 2

1. Do **not** touch `limbCov`, `earthCol`, the `opaque` guard, or the settlement
   lights on account of this frame. They are not executed.
2. Implement the `:786-787` correction (plus the `:791` taper). It is a real defect,
   it is the only shader defect in the frame, and it affects ascent as well as entry.
3. The flat blue-grey "instead of a lit Earth" is **the entry chase camera aiming
   1.654 deg above the limb** (`:3239-3241`), leaving an Earth-pixel fraction of
   zero. If the intent is that entry shows the Earth, that is a camera change, and it
   is a decision Task 2 must make explicitly rather than absorb into a shader edit.
   Judge any camera change by the Earth-pixel fraction (Task 1b's finding: a limb off
   the *top* of the frame is healthy), never by pulling the limb into `[-1, 1]`.
4. Corroborating the `deckFade` work already scoped to Task 2: at this same instant
   the entry `wide` and `free` cameras report `camAlt = 2301 km`, because
   `cameraEntry` places their eye at `y = 4` (`:3247`) in a frame whose origin is
   3714 km above the local surface 6600 km uprange. Same root cause as
   `deckFade(cam.eye[1])` treating `cam.eye[1] = -3,714,596` as an altitude.

## 6. Is this a consequence of the `deckFade`/`uDeck` bug?

**No, and Task 2's `deckFade` fix will not change a single pixel of this frame.**
Measured, not inferred:

- `gl.getUniform(SPACE.p, SPACE.u.uDeck)` returns `1` — the `deckFade(cam.eye[1])`
  bug is present, exactly as described.
- The camera is outside the cloud shell at **all 921600 pixels** (probe:
  `step(uER*uER + 2.0*uER*CLOUD_H, dot(uEc,uEc) - CLOUD_H*CLOUD_H)` = 1 everywhere),
  so `cloudLayer` takes the `sphereHit` + `rim` branch at `:824-828`.
- That branch computes `ins = RC - (impact parameter)`, measured in the range
  `-43,700 .. -78,000` m across the frame — the ray passes tens of km *outside* the
  shell's silhouette. So `rim = clamp(ins/max(uPxA*dot(uEc,d), 1.0) + 0.5, 0, 1) = 0`
  and `:829` returns on `rim < 0.002`.
- `:829` returns **before** `deck` is computed on `:832-833`. `uDeck` is never read
  for any pixel in this frame. (And it would not have mattered: `uCamW.xz` has
  length 5.88e6, so `exp(-length((d*tC + uCamW).xz)/45000.0) = exp(-130) = 0`.)

Measured consequence: `ca <= 7.69e-6` at every pixel (same scaled-probe bound as
`covE`; see section 1 for why this is a bound and not an equality), for a reason that
is independent of `uDeck`. The two defects are disjoint.

### Trap for whoever implements this

`sphereHit` (`:629-635`) **does not report a miss** for rays that pass outside the
sphere but still have `dot(c,d) > 0`. When `h2 > R*R` the `max(R*R - h2, 0.0)`
clamps to zero and the function returns `t = b`, so `tE > 0.0` is true for the
entire lower 44.4% of this frame even though not one ray touches the planet. That
is deliberate (`:625-628`: clamping to the tangent point so sub-pixel limb rays
still have a shading point) and the real miss test is the *sign of `ins`*, consumed
by `limbCov`. Do not use `tE > 0.0` as "the ray hit the Earth" anywhere in Task 2 or
Task 3 — use `covE`.
