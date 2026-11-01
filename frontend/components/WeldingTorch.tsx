"use client";

import { useMemo } from "react";
import * as THREE from "three";
import {
  TOOL0_XYZ_M,
  TOOL0_RPY_DEG,
  TORCH_MOUNT_DIA_M,
  TORCH_MOUNT_LEN_M,
  TORCH_STICK_OUT_M,
} from "@/lib/tool0";

const deg2rad = (d: number) => (d * Math.PI) / 180;

/**
 * Procedural MIG torch in the Motoman flange / TOOL frame:
 *   origin = flange face centre
 *   +Z     = out of flange (before TOOL RPY)
 *   TCP    = TOOL 0 xyz from the DX200 dump
 *
 * Parent this group under URDF `tool0` / `link_6_t` after applying the
 * Motoman→URDF rotation R_y(+π/2) on the parent mount (see WeldUrdfRobot).
 */
export default function WeldingTorch({
  arcOn = false,
  showTcpGizmo = true,
}: {
  arcOn?: boolean;
  showTcpGizmo?: boolean;
}) {
  const geo = useMemo(() => {
    const tcp = new THREE.Vector3(...TOOL0_XYZ_M);
    const rpy = new THREE.Euler(
      deg2rad(TOOL0_RPY_DEG[0]),
      deg2rad(TOOL0_RPY_DEG[1]),
      deg2rad(TOOL0_RPY_DEG[2]),
      "XYZ",
    );
    const qTool = new THREE.Quaternion().setFromEuler(rpy);
    // Motoman tool +Z after RPY = wire / approach axis.
    const wireDir = new THREE.Vector3(0, 0, 1).applyQuaternion(qTool).normalize();
    const nozzle = tcp.clone().addScaledVector(wireDir, -TORCH_STICK_OUT_M);
    const mountEnd = new THREE.Vector3(0, 0, TORCH_MOUNT_LEN_M);
    const neckVec = nozzle.clone().sub(mountEnd);
    const neckLen = Math.max(0.05, neckVec.length());
    const neckDir = neckVec.normalize();
    const yUp = new THREE.Vector3(0, 1, 0);
    const neckQuat = new THREE.Quaternion().setFromUnitVectors(yUp, neckDir);
    const wireQuat = new THREE.Quaternion().setFromUnitVectors(yUp, wireDir);
    const neckMid = mountEnd.clone().addScaledVector(neckDir, neckLen / 2);
    const wireMid = tcp.clone().addScaledVector(wireDir, -TORCH_STICK_OUT_M / 2);
    return { wireDir, nozzle, neckQuat, wireQuat, neckLen, neckMid, wireMid, tcp };
  }, []);

  const mountR = TORCH_MOUNT_DIA_M / 2;

  return (
    <group>
      {/* a) Anti-collision mount at flange — cylinder along +Z */}
      <mesh
        position={[0, 0, TORCH_MOUNT_LEN_M / 2]}
        rotation={[Math.PI / 2, 0, 0]}
        castShadow
      >
        <cylinderGeometry args={[mountR, mountR * 1.05, TORCH_MOUNT_LEN_M, 24]} />
        <meshStandardMaterial color={0x1a1f26} metalness={0.35} roughness={0.55} />
      </mesh>
      <mesh position={[0, 0, 0.004]} rotation={[Math.PI / 2, 0, 0]} castShadow>
        <cylinderGeometry args={[mountR * 1.15, mountR * 1.15, 0.008, 24]} />
        <meshStandardMaterial color={0x2a3038} metalness={0.5} roughness={0.4} />
      </mesh>

      {/* b) Swan-neck tube — mount end → nozzle (~42.6° toward −X) */}
      <mesh position={geo.neckMid.toArray()} quaternion={geo.neckQuat} castShadow>
        <cylinderGeometry args={[0.01, 0.011, geo.neckLen, 16]} />
        <meshStandardMaterial color={0x3a4553} metalness={0.55} roughness={0.4} />
      </mesh>

      {/* c) Copper gas nozzle */}
      <mesh position={geo.nozzle.toArray()} quaternion={geo.wireQuat} castShadow>
        <cylinderGeometry args={[0.009, 0.013, 0.028, 16]} />
        <meshStandardMaterial color={0xb87333} metalness={0.75} roughness={0.35} />
      </mesh>

      {/* Stick-out wire — tip exactly at TOOL 0 TCP */}
      <mesh position={geo.wireMid.toArray()} quaternion={geo.wireQuat}>
        <cylinderGeometry args={[0.0012, 0.0012, TORCH_STICK_OUT_M, 8]} />
        <meshStandardMaterial color={0xd0d5db} metalness={0.8} roughness={0.25} />
      </mesh>

      {/* TCP gizmo */}
      {showTcpGizmo && (
        <mesh position={geo.tcp.toArray()}>
          <sphereGeometry args={[0.0035, 12, 12]} />
          <meshStandardMaterial
            color="#ff7a1a"
            emissive="#ff6a00"
            emissiveIntensity={0.9}
          />
        </mesh>
      )}

      {arcOn && (
        <group position={geo.tcp.toArray()}>
          <mesh>
            <sphereGeometry args={[0.012, 12, 12]} />
            <meshStandardMaterial
              color="#fff2b0"
              emissive="#ff8a00"
              emissiveIntensity={2}
              transparent
              opacity={0.85}
            />
          </mesh>
          <pointLight color="#ff9a3c" intensity={2.5} distance={0.55} />
        </group>
      )}
    </group>
  );
}
