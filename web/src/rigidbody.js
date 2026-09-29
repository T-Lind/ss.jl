// Port of the inertia helpers in src/rigidbody.jl (the quaternion dynamics are
// not needed until the 6-DOF phase).

// bodies: [[mass, radius, length, x_centre], ...] -> [I_roll, I_transverse]
export function stack_inertia(bodies) {
  let mt = 0;
  for (const b of bodies) mt += b[0];
  if (!(mt > 0)) return [0.0, 0.0];
  let xc = 0;
  for (const b of bodies) xc += b[0] * b[3];
  xc /= mt;
  let Ir = 0.0, It = 0.0;
  for (const [m, r, L, x] of bodies) {
    Ir += 0.5 * m * r * r;
    It += m * (3 * r * r + L * L) / 12 + m * (x - xc) ** 2;
  }
  return [Ir, It];
}
