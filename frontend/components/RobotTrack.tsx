"use client";

import { Suspense } from "react";
import { useLoader } from "@react-three/fiber";
import { STLLoader } from "three/examples/jsm/loaders/STLLoader.js";
import { withBase } from "@/lib/models";

/**
 * Robot travel axis (szyna).
 *
 * Primary (default): original TSL-600-style STL meshes from the Verbotics
 * example cell (`frontend/public/models/tsl600/`). Procedural boxes are only
 * a Suspense / opt-out fallback.
 */
export default function RobotTrack({
  length = 4.0,
  carriage = 0,
  color = 0x6b7683,
  /** Prefer original TSL-600 mesh (default true). */
  useVendorMesh = true,
}: {
  length?: number;
  carriage?: number;
  color?: number;
  useVendorMesh?: boolean;
}) {
  if (!useVendorMesh) {
    return <ProceduralTrack length={length} carriage={carriage} color={color} />;
  }
  return (
    <Suspense fallback={<ProceduralTrack length={length} carriage={carriage} color={color} />}>
      <TslVendorTrack length={length} carriage={carriage} color={color} />
    </Suspense>
  );
}

function ProceduralTrack({
  length,
  carriage,
  color,
}: {
  length: number;
  carriage: number;
  color: number;
}) {
  const railX = 0.32;
  const railTopY = 0.12;
  const bedW = 1.75;
  const carriageTopY = railTopY + 0.09;
  const dollyX = -0.62;

  return (
    <group>
      <mesh position={[0, 0.03, 0]} receiveShadow>
        <boxGeometry args={[bedW, 0.06, length]} />
        <meshStandardMaterial color={0x141b23} />
      </mesh>
      {[railX, -railX].map((x) => (
        <mesh key={x} position={[x, railTopY, 0]} castShadow receiveShadow>
          <boxGeometry args={[0.1, 0.08, length]} />
          <meshStandardMaterial color={0xb8c0c9} metalness={0.6} roughness={0.35} />
        </mesh>
      ))}
      <mesh position={[dollyX, railTopY, 0]} castShadow>
        <boxGeometry args={[0.1, 0.08, length]} />
        <meshStandardMaterial color={0xb8c0c9} metalness={0.6} roughness={0.35} />
      </mesh>
      <group position={[0, 0, carriage]}>
        <mesh position={[0, carriageTopY, 0]} castShadow receiveShadow>
          <boxGeometry args={[0.8, 0.1, 0.72]} />
          <meshStandardMaterial color={color} metalness={0.3} roughness={0.6} />
        </mesh>
        <mesh position={[dollyX, carriageTopY, 0]} castShadow receiveShadow>
          <boxGeometry args={[0.6, 0.1, 0.72]} />
          <meshStandardMaterial color={color} metalness={0.3} roughness={0.6} />
        </mesh>
      </group>
    </group>
  );
}

/** Measured bbox of vendored STLs (millimetres). Mesh: X lateral, Y travel, Z up. */
const BASE = {
  minX: -1203.2,
  maxX: -413.2,
  minY: -3004.0,
  maxY: 986.0,
  minZ: -243.5,
  maxZ: 50.5,
};
const CARR = {
  minX: -1145.7,
  maxX: -470.7,
  minY: 171.0,
  maxY: 961.0,
  minZ: -59.8,
  maxZ: 23.5,
};

function TslVendorTrack({
  length,
  carriage,
  color,
}: {
  length: number;
  carriage: number;
  color: number;
}) {
  const [baseGeo, carriageGeo] = useLoader(STLLoader, [
    withBase("/models/tsl600/visual/base_link_3150.stl"),
    withBase("/models/tsl600/visual/carriage_link.stl"),
  ]);

  const travelM = (BASE.maxY - BASE.minY) / 1000;
  const mm = 0.001 * (length / travelM);

  const baseCx = (BASE.minX + BASE.maxX) / 2;
  const baseCy = (BASE.minY + BASE.maxY) / 2;
  const carrCx = (CARR.minX + CARR.maxX) / 2;
  const carrCy = (CARR.minY + CARR.maxY) / 2;

  // Mesh (X,Y,Z) → scene: Rx(-90)·Ry(180) ⇒ (−X, Z, Y) so travel→+Z, up→+Y.
  const meshToScene: [number, number, number] = [-Math.PI / 2, Math.PI, 0];

  return (
    <group>
      <mesh position={[0, 0.008, 0]} receiveShadow>
        <boxGeometry args={[1.05, 0.016, length + 0.25]} />
        <meshStandardMaterial color={0x141b23} roughness={0.95} />
      </mesh>

      <group rotation={meshToScene}>
        <group position={[-baseCx * mm, -baseCy * mm, -BASE.minZ * mm]}>
          <group scale={[mm, mm, mm]}>
            <mesh geometry={baseGeo} castShadow receiveShadow>
              <meshStandardMaterial color={0xb8c0c9} metalness={0.55} roughness={0.4} />
            </mesh>
          </group>
        </group>
      </group>

      <group position={[0, 0, carriage]}>
        <group rotation={meshToScene}>
          <group position={[-carrCx * mm, -carrCy * mm, -CARR.minZ * mm]}>
            <group scale={[mm, mm, mm]}>
              <mesh geometry={carriageGeo} castShadow receiveShadow>
                <meshStandardMaterial color={color} metalness={0.35} roughness={0.55} />
              </mesh>
            </group>
          </group>
        </group>
        {/* Mount pads under robot / power-source dolly (thin — STL is the rail) */}
        <mesh position={[0, 0.26, 0]} castShadow receiveShadow>
          <boxGeometry args={[0.55, 0.04, 0.5]} />
          <meshStandardMaterial color={color} metalness={0.35} roughness={0.55} />
        </mesh>
        <mesh position={[-0.62, 0.26, 0]} castShadow receiveShadow>
          <boxGeometry args={[0.45, 0.04, 0.45]} />
          <meshStandardMaterial color={color} metalness={0.35} roughness={0.55} />
        </mesh>
      </group>
    </group>
  );
}
