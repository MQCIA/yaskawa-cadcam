"use client";

import { Suspense } from "react";
import { useLoader } from "@react-three/fiber";
import { STLLoader } from "three/examples/jsm/loaders/STLLoader.js";
import { withBase } from "@/lib/models";
import { TOOL0_LENGTH_M } from "@/lib/tool0";

/**
 * MIG/MAG welding torch. Visual tip is at local origin; body extends along −Z
 * (Motoman TOOL approach = +Z into the work). Mount the group at the TOOL TCP
 * with TOOL RPY so the tip matches production TOOL.CND.
 */
export default function WeldingTorch({
  arcOn = false,
  vendorMesh = true,
}: {
  arcOn?: boolean;
  /** @deprecated Tip is always at the origin; length comes from TOOL0. */
  tipLen?: number;
  vendorMesh?: boolean;
}) {
  return (
    <group>
      {vendorMesh ? (
        <Suspense fallback={<ProceduralTorch />}>
          <EsabTorchMesh />
        </Suspense>
      ) : (
        <ProceduralTorch />
      )}
      {arcOn && <ArcGlow />}
    </group>
  );
}

function EsabTorchMesh() {
  const geo = useLoader(STLLoader, withBase("/models/torch_esab/rm62_36deg.stl"));
  // Mesh units: millimetres. Tip ≈ (0,0,352); shift so tip sits at local origin,
  // then scale mm→m. Body lies along −Z after the shift.
  const tipZmm = 352;
  return (
    <group scale={[0.001, 0.001, 0.001]}>
      <mesh geometry={geo} position={[0, 0, -tipZmm]} castShadow>
        <meshStandardMaterial color={0x3a4553} metalness={0.45} roughness={0.45} />
      </mesh>
    </group>
  );
}

function ProceduralTorch() {
  const L = TOOL0_LENGTH_M;
  // Three.js cylinders are Y-up; rotate so the axis lies along −Z from the tip.
  const alongZ: [number, number, number] = [Math.PI / 2, 0, 0];
  return (
    <group>
      <mesh position={[0, 0, -L * 0.35]} rotation={alongZ} castShadow>
        <cylinderGeometry args={[0.032, 0.036, L * 0.55, 20]} />
        <meshStandardMaterial color={0x1f2937} metalness={0.4} roughness={0.5} />
      </mesh>
      <mesh position={[0, 0, -L * 0.72]} rotation={alongZ} castShadow>
        <cylinderGeometry args={[0.018, 0.02, L * 0.32, 16]} />
        <meshStandardMaterial color={0x9aa4b2} metalness={0.6} roughness={0.35} />
      </mesh>
      <mesh position={[0, 0, -L * 0.9]} rotation={alongZ} castShadow>
        <cylinderGeometry args={[0.026, 0.03, L * 0.2, 20]} />
        <meshStandardMaterial color={0xb87333} metalness={0.7} roughness={0.4} />
      </mesh>
      <mesh position={[0, 0, -L - 0.05]} rotation={alongZ}>
        <cylinderGeometry args={[0.025, 0.025, 0.16, 16]} />
        <meshStandardMaterial color={0x111418} roughness={0.9} />
      </mesh>
    </group>
  );
}

function ArcGlow() {
  return (
    <group>
      <mesh>
        <sphereGeometry args={[0.03, 16, 16]} />
        <meshStandardMaterial color="#fff2b0" emissive="#ff8a00" emissiveIntensity={2} />
      </mesh>
      <pointLight color="#ff9a3c" intensity={3} distance={0.8} />
      {[0.05, 0.08, 0.11].map((d, i) => (
        <mesh key={i} position={[(i - 1) * 0.03, -d, 0]}>
          <sphereGeometry args={[0.006, 8, 8]} />
          <meshStandardMaterial color="#ffd27f" emissive="#ff7a1a" emissiveIntensity={1.5} />
        </mesh>
      ))}
    </group>
  );
}
