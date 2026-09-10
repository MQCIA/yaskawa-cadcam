"use client";

import WeldingTorch from "./WeldingTorch";
import { WELD_ROBOT, type WeldPose } from "@/lib/weldRobot";

/**
 * Animated welding robot: a simplified Yaskawa-style arm rendered from a solved
 * WeldPose (see lib/weldRobot). It carries a MIG torch tool on the wrist; the
 * torch tip lands exactly on the solved target so it traces the weld seam.
 */
export default function WeldRobot({
  pose,
  arcOn = false,
}: {
  pose: WeldPose;
  arcOn?: boolean;
}) {
  const { a0, sh, l1, l2, l3 } = WELD_ROBOT;
  const blue = 0x0033a0;
  const blue2 = 0x0044c4;
  const accent = 0x00a3e0;

  return (
    <group>
      {/* Base plate */}
      <mesh position={[0, 0.06, 0]} castShadow receiveShadow>
        <boxGeometry args={[0.46, 0.12, 0.5]} />
        <meshStandardMaterial color={0x1b2733} />
      </mesh>

      {/* Yaw (S) turntable + everything above rotates with it */}
      <group rotation={[0, pose.S, 0]}>
        <mesh position={[0, 0.2, 0]} castShadow>
          <cylinderGeometry args={[0.22, 0.26, 0.18, 32]} />
          <meshStandardMaterial color={blue} metalness={0.2} roughness={0.6} />
        </mesh>

        {/* Column up to the shoulder pivot */}
        <mesh position={[a0 * 0.5, sh * 0.5 + 0.1, 0]} castShadow>
          <boxGeometry args={[0.28, sh - 0.02, 0.32]} />
          <meshStandardMaterial color={blue2} metalness={0.2} roughness={0.6} />
        </mesh>

        {/* Shoulder (L) */}
        <group position={[a0, sh, 0]} rotation={[0, 0, pose.L]}>
          <mesh castShadow>
            <cylinderGeometry args={[0.13, 0.13, 0.3, 24]} />
            <meshStandardMaterial color={accent} metalness={0.3} roughness={0.5} />
          </mesh>
          <mesh position={[l1 / 2, 0, 0]} castShadow>
            <boxGeometry args={[l1, 0.16, 0.17]} />
            <meshStandardMaterial color={blue} metalness={0.25} roughness={0.55} />
          </mesh>

          {/* Elbow (U) */}
          <group position={[l1, 0, 0]} rotation={[0, 0, pose.U]}>
            <mesh castShadow>
              <cylinderGeometry args={[0.1, 0.1, 0.24, 24]} />
              <meshStandardMaterial color={accent} metalness={0.3} roughness={0.5} />
            </mesh>
            <mesh position={[l2 / 2, 0, 0]} castShadow>
              <boxGeometry args={[l2, 0.13, 0.13]} />
              <meshStandardMaterial color={blue2} metalness={0.25} roughness={0.55} />
            </mesh>

            {/* Wrist (B) + torch tool */}
            <group position={[l2, 0, 0]} rotation={[0, 0, pose.B]}>
              <mesh castShadow>
                <sphereGeometry args={[0.08, 20, 20]} />
                <meshStandardMaterial color={0xc0c0c0} metalness={0.8} roughness={0.3} />
              </mesh>
              <WeldingTorch arcOn={arcOn} tipLen={l3} />
            </group>
          </group>
        </group>
      </group>
    </group>
  );
}
