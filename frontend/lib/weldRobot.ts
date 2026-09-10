// Lightweight analytic kinematics for the animated welding robot.
//
// A simplified 4-DOF arm (base yaw + shoulder + elbow, torch held vertical) that
// we can solve in closed form so the torch tip lands exactly on the weld path.
// The renderer (WeldRobot) uses the SAME constants, so what you see matches the
// solved pose. Illustrative geometry only — not a real AR-series robot.

import type { Vec3 } from "./weldProgram";

export const WELD_ROBOT = {
  a0: 0.2, // shoulder forward offset from the base axis
  sh: 0.62, // shoulder pivot height above the robot base
  l1: 1.0, // upper arm length
  l2: 0.85, // forearm length
  l3: 0.3, // wrist -> torch tip (torch held straight down)
};

export type WeldPose = {
  S: number; // base yaw (rad)
  L: number; // upper-arm pitch (rad)
  U: number; // elbow pitch, relative (rad)
  B: number; // wrist pitch, relative (rad)
  reachable: boolean;
};

const clamp = (v: number, a: number, b: number) => Math.max(a, Math.min(b, v));

/** Neutral target used when no program is loaded (arm points at the station). */
export function weldHomeTarget(base: Vec3): Vec3 {
  return [base[0] + 1.4, 0.95, base[2]];
}

/**
 * Solve the arm so the torch tip reaches `target` (world) given the robot
 * `base` (world). Torch is kept vertical (pointing down) into the work.
 */
export function computeWeldPose(target: Vec3, base: Vec3): WeldPose {
  const { a0, sh, l1, l2, l3 } = WELD_ROBOT;

  const tx = target[0] - base[0];
  const ty = target[1] - base[1];
  const tz = target[2] - base[2];

  // Base yaw so the arm plane contains the target; r is horizontal reach.
  const S = Math.atan2(-tz, tx);
  const r = Math.hypot(tx, tz);

  // Wrist target in the arm plane (torch adds l3 straight down to the tip).
  const dx = r - a0;
  const dy = ty + l3 - sh;
  let D = Math.hypot(dx, dy);
  const reachable = D <= l1 + l2 && D >= Math.abs(l1 - l2);
  D = clamp(D, Math.abs(l1 - l2) + 1e-3, l1 + l2 - 1e-3);

  // Two-link elbow-up solution.
  const a = Math.atan2(dy, dx);
  const beta = Math.acos(clamp((l1 * l1 + D * D - l2 * l2) / (2 * l1 * D), -1, 1));
  const t1 = a + beta; // upper-arm absolute pitch
  const gamma = Math.acos(clamp((l1 * l1 + l2 * l2 - D * D) / (2 * l1 * l2), -1, 1));
  const t2 = t1 - (Math.PI - gamma); // forearm absolute pitch
  const t3 = -Math.PI / 2; // torch straight down

  return { S, L: t1, U: t2 - t1, B: t3 - t2, reachable };
}
