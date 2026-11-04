"use client";

import { useMemo, Suspense } from "react";
import { useLoader } from "@react-three/fiber";
import * as THREE from "three";
import { STLLoader } from "three/examples/jsm/loaders/STLLoader.js";
import { withBase } from "@/lib/models";
import {
  TSL_BASE_URL,
  TSL_CARRIAGE_URL,
  TSL_CARRIAGE_TOP_Y,
} from "@/lib/trackMesh";

/**
 * Tor BS1 / RECT-X — original TSL-600 STLs (no procedural rail/boxes).
 * Procedural mesh is only a Suspense fallback while STLs load.
 */
export default function RobotTrack({
  carriage = 0,
  color = 0x6b7683,
}: {
  length?: number;
  carriage?: number;
  color?: number;
  useVendorMesh?: boolean;
}) {
  return (
    <Suspense fallback={<ProceduralTrack carriage={carriage} color={color} />}>
      <TslVendorTrack carriage={carriage} color={color} />
    </Suspense>
  );
}

function orientMmToScene(src: THREE.BufferGeometry): THREE.BufferGeometry {
  const g = src.clone();
  g.scale(0.001, 0.001, 0.001);
  // Mesh Z-up, travel +Y → scene Y-up, travel +Z.
  g.rotateX(-Math.PI / 2);
  g.rotateY(Math.PI);
  return g;
}

function TslVendorTrack({
  carriage,
  color,
}: {
  carriage: number;
  color: number;
}) {
  const [baseRaw, carriageRaw] = useLoader(STLLoader, [
    withBase(TSL_BASE_URL),
    withBase(TSL_CARRIAGE_URL),
  ]);

  const { baseGeo, carriageGeo } = useMemo(() => {
    const base = orientMmToScene(baseRaw);
    const car = orientMmToScene(carriageRaw);
    base.computeBoundingBox();
    const bb = base.boundingBox!;
    const cx = (bb.min.x + bb.max.x) / 2;
    const cz = (bb.min.z + bb.max.z) / 2;
    const ty = -bb.min.y;
    // Shared frame: floor at Y=0, rail centred in XZ.
    base.translate(-cx, ty, -cz);
    car.translate(-cx, ty, -cz);
    // Carriage slides in a group at Z=carriage — centre its local Z on 0.
    car.computeBoundingBox();
    const cbb = car.boundingBox!;
    const carCz = (cbb.min.z + cbb.max.z) / 2;
    car.translate(0, 0, -carCz);
    base.computeVertexNormals();
    car.computeVertexNormals();
    base.computeBoundingBox();
    car.computeBoundingBox();
    return { baseGeo: base, carriageGeo: car };
  }, [baseRaw, carriageRaw]);

  return (
    <group>
      <mesh geometry={baseGeo} castShadow receiveShadow>
        <meshStandardMaterial
          color={0x9aa3ad}
          metalness={0.6}
          roughness={0.38}
          side={THREE.DoubleSide}
        />
      </mesh>
      <group position={[0, 0, carriage]}>
        <mesh geometry={carriageGeo} castShadow receiveShadow>
          <meshStandardMaterial
            color={color}
            metalness={0.4}
            roughness={0.5}
            side={THREE.DoubleSide}
          />
        </mesh>
      </group>
    </group>
  );
}

function ProceduralTrack({
  carriage,
  color,
}: {
  carriage: number;
  color: number;
}) {
  const length = 4.0;
  return (
    <group>
      <mesh position={[0, 0.03, 0]} receiveShadow>
        <boxGeometry args={[1.2, 0.06, length]} />
        <meshStandardMaterial color={0x141b23} />
      </mesh>
      {[-0.28, 0.28].map((x) => (
        <mesh key={x} position={[x, 0.1, 0]} castShadow receiveShadow>
          <boxGeometry args={[0.09, 0.07, length]} />
          <meshStandardMaterial color={0xb8c0c9} metalness={0.6} roughness={0.35} />
        </mesh>
      ))}
      <group position={[0, 0, carriage]}>
        <mesh position={[0, TSL_CARRIAGE_TOP_Y - 0.04, 0]} castShadow receiveShadow>
          <boxGeometry args={[0.7, 0.08, 0.7]} />
          <meshStandardMaterial color={color} metalness={0.3} roughness={0.6} />
        </mesh>
      </group>
    </group>
  );
}
