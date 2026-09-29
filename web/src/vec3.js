// Port of src/vec3.jl. 3-vectors are plain [x, y, z] arrays; the helpers
// allocate, which the browser can afford and the Julia core deliberately
// avoided. Names mirror the Julia so a port can be read side by side.
export const v3 = (x, y, z) => [x, y, z];
export const vadd = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
export const vsub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
export const vscale = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
export const vdot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
export const vcross = (a, b) => [
  a[1] * b[2] - a[2] * b[1],
  a[2] * b[0] - a[0] * b[2],
  a[0] * b[1] - a[1] * b[0],
];
export const vnorm = a => Math.sqrt(vdot(a, a));
export function vunit(a) {
  const n = vnorm(a);
  return n > 0 ? vscale(a, 1 / n) : [0.0, 0.0, 0.0];
}

// ECI -> ECEF by Earth rotation angle theta: ecef = R3(theta) * eci
export function rot_z(a, theta) {
  const c = Math.cos(theta), s = Math.sin(theta);
  return [c * a[0] + s * a[1], -s * a[0] + c * a[1], a[2]];
}
