/**
 * Production DX200 TOOL 0 (from TOOL.CND on the MA2010 cell dump).
 *
 * Motoman flange frame: origin at flange face centre, +Z out of the flange
 * (approach / wire axis before RPY), +X/+Y in the flange plane.
 *
 * MA2010 URDF (`ma2010.urdf`): `joint_6_t` axis = (−1,0,0); `tool0` sits at
 * (+0.100, 0, 0) on `link_6_t` with identity RPY — so URDF +X on `tool0` /
 * `link_6_t` is the flange approach. Map Motoman → URDF with R_y(+π/2):
 *   (x,y,z)_motoman  →  (z, y, −x)_urdf
 */

export const TOOL0_XYZ_M = [-0.05428, 0.001052, 0.415702] as const;
export const TOOL0_RPY_DEG = [1.3119, -42.6215, -6.8387] as const;

/** Stick-out: wire tip (TCP) beyond copper nozzle, metres. */
export const TORCH_STICK_OUT_M = 0.015;

/** Anti-collision mount length along flange +Z, metres. */
export const TORCH_MOUNT_LEN_M = 0.12;
export const TORCH_MOUNT_DIA_M = 0.055;
