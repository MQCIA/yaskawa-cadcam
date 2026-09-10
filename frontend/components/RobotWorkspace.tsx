"use client";

import { Suspense } from "react";
import { Canvas } from "@react-three/fiber";
import { Grid, OrbitControls, Environment, Html } from "@react-three/drei";
import YaskawaManipulator from "./YaskawaManipulator";
import UrdfModel from "./UrdfModel";
import type { Joints } from "./AxisSliders";
import {
  ROBOT_MODELS,
  POSITIONER_MODELS,
  type PositionerModel,
} from "@/lib/models";

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

function Loading() {
  return (
    <Html center>
      <div className="rounded bg-slate-800/80 px-3 py-1 text-xs text-slate-200">
        Loading model…
      </div>
    </Html>
  );
}

function SelectedRobot({
  modelId,
  joints,
}: {
  modelId: string;
  joints: Joints;
}) {
  const model = ROBOT_MODELS.find((m) => m.id === modelId) ?? ROBOT_MODELS[0];
  if (model.kind === "procedural" || !model.url) {
    return <YaskawaManipulator joints={joints} />;
  }
  const jointValuesDeg: Record<string, number> = {};
  for (const [axis, jointName] of Object.entries(model.jointMap ?? {})) {
    if (jointName) jointValuesDeg[jointName] = joints[axis as keyof Joints];
  }
  return (
    <UrdfModel url={model.url} jointValuesDeg={jointValuesDeg} color={model.color} />
  );
}

function SelectedPositioner({
  positioner,
  tilt,
  rotate,
}: {
  positioner: PositionerModel;
  tilt: number;
  rotate: number;
}) {
  const jointValuesDeg: Record<string, number> = {};
  if (positioner.axisMap.tilt) jointValuesDeg[positioner.axisMap.tilt] = tilt;
  if (positioner.axisMap.rotate) jointValuesDeg[positioner.axisMap.rotate] = rotate;
  return (
    <UrdfModel
      url={positioner.url}
      jointValuesDeg={jointValuesDeg}
      color={positioner.color}
      position={[1.6, 0, 0]}
    />
  );
}

export default function RobotWorkspace({
  modelId,
  joints,
  seams = [],
  positionerId,
  positionerTilt = 0,
  positionerRotate = 0,
}: {
  modelId: string;
  joints: Joints;
  seams?: SeamSegment[];
  positionerId?: string | null;
  positionerTilt?: number;
  positionerRotate?: number;
}) {
  const positioner = positionerId
    ? POSITIONER_MODELS.find((p) => p.id === positionerId)
    : undefined;

  return (
    <Canvas shadows camera={{ position: [3, 2.5, 3], fov: 45 }}>
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
        fadeDistance={25}
      />

      <Suspense fallback={<Loading />}>
        <SelectedRobot modelId={modelId} joints={joints} />
        {positioner && (
          <SelectedPositioner
            positioner={positioner}
            tilt={positionerTilt}
            rotate={positionerRotate}
          />
        )}
      </Suspense>

      {seams.map((s, i) => (
        <Seam key={i} seg={s} />
      ))}

      <Environment preset="warehouse" />
      <OrbitControls makeDefault enableDamping target={[0.5, 0.8, 0]} />
    </Canvas>
  );
}
