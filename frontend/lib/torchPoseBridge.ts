/**
 * Live Motoman ``tcp_0`` (torch tip) pose published from ``WeldUrdfRobot``.
 * Teach uses this — not DH FK — so the point matches the tip the user sees.
 */

import type { Vec3 } from "@/lib/weldProgram";

export type TorchWorldPose = {
  /** World-space tip position (metres), same frame as R3F scene. */
  pos: Vec3;
  /** World-space wire / stick-out direction (tcp_0 +Z). */
  dir: Vec3;
  updatedAt: number;
};

let latest: TorchWorldPose | null = null;

export function publishTorchWorldPose(pos: Vec3, dir: Vec3): void {
  latest = {
    pos: [pos[0], pos[1], pos[2]],
    dir: [dir[0], dir[1], dir[2]],
    updatedAt: performance.now(),
  };
}

export function readTorchWorldPose(maxAgeMs = 500): TorchWorldPose | null {
  if (!latest) return null;
  if (performance.now() - latest.updatedAt > maxAgeMs) return null;
  return latest;
}
