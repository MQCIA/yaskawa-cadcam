"use client";

import { Canvas } from "@react-three/fiber";
import { Grid, OrbitControls, Environment } from "@react-three/drei";
import YaskawaManipulator from "./YaskawaManipulator";
import type { Joints } from "./AxisSliders";

export type SeamSegment = {
  start: [number, number, number];
  end: [number, number, number];
};

function Seam({ seg }: { seg: SeamSegment }) {
  const [x0, y0, z0] = seg.start;
  const [x1, y1, z1] = seg.end;
  const mid: [number, number, number] = [
    (x0 + x1) / 2,
    (y0 + y1) / 2,
    (z0 + z1) / 2,
  ];
  const len = Math.hypot(x1 - x0, y1 - y0, z1 - z0);
  return (
    <group position={mid}>
      <mesh>
        <sphereGeometry args={[Math.max(0.01, len * 0.05), 8, 8]} />
        <meshStandardMaterial color="red" emissive="red" emissiveIntensity={0.6} />
      </mesh>
    </group>
  );
}

export default function RobotWorkspace({
  joints,
  seams = [],
}: {
  joints: Joints;
  seams?: SeamSegment[];
}) {
  return (
    <Canvas shadows camera={{ position: [2.5, 2, 2.5], fov: 45 }}>
      <ambientLight intensity={0.4} />
      <directionalLight
        position={[5, 8, 5]}
        intensity={1.1}
        castShadow
        shadow-mapSize={[2048, 2048]}
      />
      <hemisphereLight intensity={0.3} groundColor="#101820" />

      <Grid
        args={[10, 10]}
        cellSize={0.25}
        cellThickness={0.5}
        sectionSize={1}
        sectionThickness={1}
        sectionColor="#00a3e0"
        cellColor="#2a3746"
        infiniteGrid
        fadeDistance={20}
      />

      <YaskawaManipulator joints={joints} />

      {seams.map((s, i) => (
        <Seam key={i} seg={s} />
      ))}

      <Environment preset="warehouse" />
      <OrbitControls makeDefault enableDamping target={[0.3, 0.6, 0]} />
    </Canvas>
  );
}
