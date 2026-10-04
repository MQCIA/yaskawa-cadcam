/**
 * Production Motoman TOOL 0 from the DX200 cell dump (`TOOL.CND`).
 * Pose is relative to the flange / ROS-Industrial tool0 parent.
 *
 * @see docs/CELL.md
 */
export const TOOL0_TCP_MM = {
  x: -54.28,
  y: 1.052,
  z: 415.702,
} as const;

export const TOOL0_RPY_DEG = {
  rx: 1.3119,
  ry: -42.6215,
  rz: -6.8387,
} as const;

export const TOOL0_TCP_M: [number, number, number] = [
  TOOL0_TCP_MM.x / 1000,
  TOOL0_TCP_MM.y / 1000,
  TOOL0_TCP_MM.z / 1000,
];

/** Straight-line flange→tip length (metres). */
export const TOOL0_LENGTH_M = Math.hypot(...TOOL0_TCP_M);

export const TOOL0_RPY_RAD: [number, number, number] = [
  (TOOL0_RPY_DEG.rx * Math.PI) / 180,
  (TOOL0_RPY_DEG.ry * Math.PI) / 180,
  (TOOL0_RPY_DEG.rz * Math.PI) / 180,
];
