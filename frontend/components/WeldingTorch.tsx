"use client";

import { Suspense } from "react";
import { useLoader } from "@react-three/fiber";
import { STLLoader } from "three/examples/jsm/loaders/STLLoader.js";
import { withBase } from "@/lib/models";

/**
 * MIG/MAG welding torch. Prefer the ESAB-style STL (mm → m) from the Verbotics
 * cell dump; fall back to a procedural torch along local +X (tip at tipLen).
 */
export default function WeldingTorch({
  arcOn = false,
  tipLen = 0.3,
  vendorMesh = true,
}: {
  arcOn?: boolean;
  tipLen?: number;
  vendorMesh?: boolean;
}) {
  return (
    <group>
      {vendorMesh ? (
        <Suspense fallback={<ProceduralTorch tipLen={tipLen} />}>
          <EsabTorchMesh tipLen={tipLen} />
        </Suspense>
      ) : (
        <ProceduralTorch tipLen={tipLen} />
      )}
      {arcOn && <ArcGlow tipLen={tipLen} />}
    </group>
  );
}

function EsabTorchMesh({ tipLen }: { tipLen: number }) {
  const geo = useLoader(STLLoader, withBase("/models/torch_esab/rm62_36deg.stl"));
  // Mesh is mm with tip roughly along +Z (~352 mm). Convert to metres, map Z→+X.
  const mm = 0.001;
  const nativeTipM = 0.352;
  const s = tipLen / nativeTipM;
  return (
    <group rotation={[0, 0, -Math.PI / 2]} scale={s}>
      <mesh geometry={geo} scale={[mm, mm, mm]} castShadow>
        <meshStandardMaterial color={0x3a4553} metalness={0.45} roughness={0.45} />
      </mesh>
    </group>
  );
}

function ProceduralTorch({ tipLen }: { tipLen: number }) {
  return (
    <group>
      <mesh position={[-0.08, 0, 0]} rotation={[0, 0, Math.PI / 2]}>
        <cylinderGeometry args={[0.025, 0.025, 0.16, 16]} />
        <meshStandardMaterial color={0x111418} roughness={0.9} />
      </mesh>
      <mesh position={[tipLen * 0.35, 0, 0]} rotation={[0, 0, Math.PI / 2]} castShadow>
        <cylinderGeometry args={[0.032, 0.036, tipLen * 0.55, 20]} />
        <meshStandardMaterial color={0x1f2937} metalness={0.4} roughness={0.5} />
      </mesh>
      <mesh position={[tipLen * 0.72, 0, 0]} rotation={[0, 0, Math.PI / 2]} castShadow>
        <cylinderGeometry args={[0.018, 0.02, tipLen * 0.32, 16]} />
        <meshStandardMaterial color={0x9aa4b2} metalness={0.6} roughness={0.35} />
      </mesh>
      <mesh position={[tipLen * 0.9, 0, 0]} rotation={[0, 0, -Math.PI / 2]} castShadow>
        <cylinderGeometry args={[0.026, 0.03, tipLen * 0.2, 20]} />
        <meshStandardMaterial color={0xb87333} metalness={0.7} roughness={0.4} />
      </mesh>
    </group>
  );
}

function ArcGlow({ tipLen }: { tipLen: number }) {
  return (
    <group position={[tipLen, 0, 0]}>
      <mesh>
        <sphereGeometry args={[0.03, 16, 16]} />
        <meshStandardMaterial color="#fff2b0" emissive="#ff8a00" emissiveIntensity={2} />
      </mesh>
      <pointLight color="#ff9a3c" intensity={3} distance={0.8} />
      {[0.05, 0.08, 0.11].map((d, i) => (
        <mesh key={i} position={[0, -d, (i - 1) * 0.03]}>
          <sphereGeometry args={[0.006, 8, 8]} />
          <meshStandardMaterial color="#ffd27f" emissive="#ff7a1a" emissiveIntensity={1.5} />
        </mesh>
      ))}
    </group>
  );
}
