// Unit tests for the pure helpers in the browser layer.
//
// The simulation is tested exhaustively; the UI is not. These functions are
// the parts of it that are testable without a DOM, and they are where the
// quiet math bugs live (a series axis, a dateline crossing).
//
//   node --test test/ui_helpers.test.mjs
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { chartExtent } from '../scripts/static/charts.js';
import { mapPath } from '../scripts/static/groundtrack.js';
import { allowsShortcut, escapeHTML, keyboardAction } from '../scripts/static/ui.js';

test('global shortcuts preserve text editing, browser shortcuts and focused button activation', () => {
  const event = { key: 'a', target: { closest: () => null } };
  assert.equal(allowsShortcut(event), true);
  for (const key of ['ctrlKey', 'metaKey', 'altKey', 'isComposing', 'defaultPrevented'])
    assert.equal(allowsShortcut({ ...event, [key]: true }), false, key);
  assert.equal(allowsShortcut({ ...event, target: { closest: () => ({}) } }), false);
  const button = { closest: selector => selector.startsWith('button') ? {} : null };
  assert.equal(allowsShortcut({ ...event, key: ' ', target: button }), false);
  assert.equal(allowsShortcut({ ...event, key: 'Enter', target: button }), false);
  assert.equal(allowsShortcut({ ...event, key: 'l', target: button }), true);
  const modal = { closest: selector => selector.includes('[role="dialog"]') ? {} : null };
  assert.equal(allowsShortcut({ ...event, key: 'r', target: modal }), false);
});

test('compact controls activate once from the keyboard without triggering playback shortcuts', () => {
  const attrs = {}, listeners = [];
  let clicks = 0, prevented = 0, stopped = 0;
  const element = { tagName: 'SPAN', setAttribute: (k, v) => attrs[k] = v,
    addEventListener: (type, listener) => { if (type === 'keydown') listeners.push(listener); },
    click: () => clicks++ };
  keyboardAction(element);
  keyboardAction(element); // static and generated-control setup can overlap
  assert.equal(attrs.role, 'button');
  assert.equal(element.tabIndex, 0);
  const event = { key: ' ', preventDefault: () => prevented++, stopPropagation: () => stopped++ };
  for (const listener of listeners) listener(event);
  for (const listener of listeners) listener({ ...event, repeat: true });
  assert.equal(clicks, 1);
  assert.equal(prevented, 2);
  assert.equal(stopped, 2);
});

test('user-supplied names render as text even when they contain HTML', () => {
  assert.equal(escapeHTML('<img src="x"> & \'flight\''), '&lt;img src=&quot;x&quot;&gt; &amp; &#39;flight&#39;');
});

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
