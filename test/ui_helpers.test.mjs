// Unit tests for the pure helpers in the browser layer.
//
// The simulation is tested exhaustively; the UI is not. These four functions
// are the parts of it that are testable without a DOM, and they are where the
// quiet math bugs live (a series axis, an angle wrap, a dateline crossing).
//
//   node --test test/ui_helpers.test.mjs
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { chartExtent } from '../scripts/static/charts.js';
import { wrap180 } from '../scripts/static/groundstation.js';
import { mapPath } from '../scripts/static/groundtrack.js';

// A canvas 2D context that records the path it is asked to build, so the
// dateline logic can be asserted without a browser.
function fakeCtx() {
  const paths = [];
  let cur = null;
  return {
    paths,
    moveTo(x, y) { cur = [{ x, y }]; paths.push(cur); },
    lineTo(x, y) { cur.push({ x, y }); },
    closePath() { cur.closed = true; },
  };
}
const xy = (lon, lat) => [lon, lat];

test('chartExtent spans every series', () => {
  assert.deepEqual(chartExtent({ series: [{ xs: [3, 1, 2] }, { xs: [9, 4] }] }), [1, 9]);
});

test('chartExtent ignores non-finite samples', () => {
  assert.deepEqual(chartExtent({ series: [{ xs: [NaN, Infinity, -Infinity, 2, 4] }] }), [2, 4]);
});

test('chartExtent does not overflow on a long series', () => {
  // The regression this guards: `Math.min(...xs)` throws RangeError past
  // roughly 65k arguments, and the decimation that keeps series short is a
  // convention, not a contract.
  const xs = Array.from({ length: 200_000 }, (_, i) => i);
  assert.deepEqual(chartExtent({ series: [{ xs }] }), [0, 199_999]);
});

test('chartExtent on empty input is degenerate, not a throw', () => {
  assert.deepEqual(chartExtent({ series: [] }), [Infinity, -Infinity]);
});

test('wrap180 folds into (-180, 180]', () => {
  assert.equal(wrap180(0), 0);
  assert.equal(wrap180(90), 90);
  assert.equal(wrap180(180), -180);
  assert.equal(wrap180(181), -179);
  assert.equal(wrap180(190), -170);
  assert.equal(wrap180(-190), 170);
  assert.equal(wrap180(360), 0);
  assert.equal(wrap180(-360), 0);
  for (const a of [-1000, -359, -1, 0, 1, 359, 1000]) {
    const w = wrap180(a);
    assert.ok(w > -180 && w <= 180, `${a} -> ${w}`);
  }
});

test('mapPath draws a ring that does not cross the dateline', () => {
  const ctx = fakeCtx();
  const { drawable, split } = mapPath(ctx, [[[0, 0], [1, 1], [2, 2]]], xy);
  assert.equal(drawable, true);
  assert.equal(split, false);
  assert.equal(ctx.paths.length, 1);
  assert.equal(ctx.paths[0].length, 3);
  assert.equal(ctx.paths[0].closed, true);
});

test('mapPath splits a ring across the dateline instead of bridging it', () => {
  const ctx = fakeCtx();
  const { drawable, split } = mapPath(ctx, [[[179, 0], [-179, 0], [-178, 0]]], xy);
  assert.equal(drawable, true);
  assert.equal(split, true);
  // two separate subpaths: no line drawn from 179 across the canvas to -179
  assert.equal(ctx.paths.length, 2);
  assert.deepEqual(ctx.paths[0], [{ x: 179, y: 0 }]);
  assert.deepEqual(ctx.paths[1], [{ x: -179, y: 0 }, { x: -178, y: 0 }]);
  // a split ring is not closed, or the fill would bridge it back
  assert.equal(ctx.paths[1].closed, undefined);
});

test('mapPath reports nothing drawable for empty geometry', () => {
  const { drawable } = mapPath(fakeCtx(), [], xy);
  assert.equal(drawable, false);
});
