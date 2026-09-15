"use client";

import { useLoader } from "@react-three/fiber";
import { STLLoader } from "three/examples/jsm/loaders/STLLoader.js";
import { withBase } from "@/lib/models";

/**
 * Procedural Yaskawa robot travel axis (rail / track), optionally dressed with
 * TSL-600 style STL meshes (mm → m) from the Verbotics example cell dump.
 */
export default function RobotTrack({
  length = 4.6,
  carriage = 0,
  color = 0x6b7683,
  useVendorMesh = true,
}: {
  length?: number;
  carriage?: number;
  color?: number;
  useVendorMesh?: boolean;
}) {
  if (useVendorMesh) {
    return <TslVendorTrack length={length} carriage={carriage} color={color} />;
  }
  return <ProceduralTrack length={length} carriage={carriage} color={color} />;
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
      <mesh position={[bedW / 2 - 0.05, 0.1, 0]} castShadow>
        <boxGeometry args={[0.06, 0.09, length]} />
        <meshStandardMaterial color={0x1b2733} />
      </mesh>
      <group position={[0, 0, carriage]}>
        <mesh position={[0, carriageTopY, 0]} castShadow receiveShadow>
          <boxGeometry args={[0.8, 0.1, 0.72]} />
          <meshStandardMaterial color={color} metalness={0.3} roughness={0.6} />
        </mesh>
        <mesh position={[(dollyX - 0.25) / 2 - 0.05, carriageTopY, 0]} castShadow>
          <boxGeometry args={[0.35, 0.06, 0.12]} />
          <meshStandardMaterial color={0x1b2733} metalness={0.3} roughness={0.6} />
        </mesh>
        <mesh position={[dollyX, carriageTopY, 0]} castShadow receiveShadow>
          <boxGeometry args={[0.6, 0.1, 0.72]} />
          <meshStandardMaterial color={color} metalness={0.3} roughness={0.6} />
        </mesh>
      </group>
    </group>
  );
}

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

  const mm = 0.001;
  const railLenM = 3.15;
  const zScale = length / railLenM;

  return (
    <group>
      <mesh position={[0, 0.02, 0]} receiveShadow>
        <boxGeometry args={[1.4, 0.04, length + 0.4]} />
        <meshStandardMaterial color={0x141b23} />
      </mesh>

      <group
        scale={[mm * zScale, mm, mm]}
        rotation={[-Math.PI / 2, 0, Math.PI / 2]}
        position={[0, 0.05, -length / 2]}
      >
        <mesh geometry={baseGeo} castShadow receiveShadow>
          <meshStandardMaterial color={0xb8c0c9} metalness={0.55} roughness={0.4} />
        </mesh>
      </group>

      <group position={[0, 0.05, carriage]}>
        <group scale={[mm, mm, mm]} rotation={[-Math.PI / 2, 0, Math.PI / 2]}>
          <mesh geometry={carriageGeo} castShadow receiveShadow>
            <meshStandardMaterial color={color} metalness={0.35} roughness={0.55} />
          </mesh>
        </group>
        <mesh position={[0, 0.21, 0]} castShadow receiveShadow>
          <boxGeometry args={[0.75, 0.08, 0.65]} />
          <meshStandardMaterial color={color} metalness={0.3} roughness={0.6} />
        </mesh>
        <mesh position={[-0.62, 0.21, 0]} castShadow receiveShadow>
          <boxGeometry args={[0.55, 0.08, 0.65]} />
          <meshStandardMaterial color={color} metalness={0.3} roughness={0.6} />
        </mesh>
      </group>
    </group>
  );
}
