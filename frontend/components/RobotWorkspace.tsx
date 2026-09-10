"use client";

import { Suspense } from "react";
import { Canvas } from "@react-three/fiber";
import { OrbitControls, Environment, Html } from "@react-three/drei";
import YaskawaManipulator from "./YaskawaManipulator";
import UrdfModel from "./UrdfModel";
import H1000dPositioner from "./H1000dPositioner";
import RobotTrack from "./RobotTrack";
import WeldScene from "./WeldScene";
import WeldRobot from "./WeldRobot";
import type { Joints } from "./AxisSliders";
import {
  ROBOT_MODELS,
  POSITIONER_MODELS,
  type PositionerModel,
} from "@/lib/models";
import { sampleProgram, type WeldProgram } from "@/lib/weldProgram";
import { computeWeldPose, weldHomeTarget } from "@/lib/weldRobot";

export type SeamSegment = {
  start: [number, number, number];
  end: [number, number, number];
};

export type StationState = { tilt: number; rotate: number };

// Height of the rail carriage top the robot base mounts on.
const CARRIAGE_TOP_Y = 0.26;
// The two work-station positioner positions (robot faces +X, rail runs along Z).
const STATION_POS: [number, number, number][] = [
  [1.6, 0, -1.2],
  [1.6, 0, 1.2],
];

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

function StationLabel({
  text,
  position,
}: {
  text: string;
  position: [number, number, number];
}) {
  return (
    <Html position={position} center distanceFactor={8}>
      <div className="whitespace-nowrap rounded bg-slate-900/80 px-2 py-0.5 text-xs font-medium text-slate-100">
        {text}
      </div>
    </Html>
  );
}

function SelectedRobot({
  modelId,
  joints,
  base,
  program,
  simT,
}: {
  modelId: string;
  joints: Joints;
  base: [number, number, number];
  program: WeldProgram | null;
  simT: number;
}) {
  const model = ROBOT_MODELS.find((m) => m.id === modelId) ?? ROBOT_MODELS[0];
  if (model.kind === "weld") {
    const sample = program ? sampleProgram(program, simT) : null;
    const target = sample ? sample.pos : weldHomeTarget(base);
    const pose = computeWeldPose(target, base);
    return <WeldRobot pose={pose} arcOn={sample?.arcOn ?? false} />;
  }
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
  position,
  tilt,
  rotate,
}: {
  positioner: PositionerModel;
  position: [number, number, number];
  tilt: number;
  rotate: number;
}) {
  // Turn the positioner 90° about Y so its pedestal axis runs parallel to the
  // rail (rail runs along Z).
  const yaw: [number, number, number] = [0, Math.PI / 2, 0];

  if (positioner.kind === "procedural") {
    return (
      <group position={position} rotation={yaw}>
        <H1000dPositioner rotateDeg={rotate} color={positioner.color} />
      </group>
    );
  }
  const jointValuesDeg: Record<string, number> = {};
  if (positioner.axisMap?.tilt) jointValuesDeg[positioner.axisMap.tilt] = tilt;
  if (positioner.axisMap?.rotate) jointValuesDeg[positioner.axisMap.rotate] = rotate;
  return (
    <group position={position} rotation={yaw}>
      <UrdfModel
        url={positioner.url!}
        jointValuesDeg={jointValuesDeg}
        color={positioner.color}
      />
    </group>
  );
}

export default function RobotWorkspace({
  modelId,
  joints,
  seams = [],
  positionerId,
  railTravel = 0,
  stations = [
    { tilt: 0, rotate: 0 },
    { tilt: 0, rotate: 0 },
  ],
  program = null,
  simT = 0,
}: {
  modelId: string;
  joints: Joints;
  seams?: SeamSegment[];
  positionerId?: string | null;
  railTravel?: number;
  stations?: StationState[];
  program?: WeldProgram | null;
  simT?: number;
}) {
  const positioner = positionerId
    ? POSITIONER_MODELS.find((p) => p.id === positionerId)
    : undefined;

  return (
    <Canvas shadows camera={{ position: [4.5, 3, 4.5], fov: 45 }}>
      <color attach="background" args={["#ffffff"]} />
      <ambientLight intensity={0.4} />
      <directionalLight
        position={[5, 8, 5]}
        intensity={1.1}
        castShadow
        shadow-mapSize={[2048, 2048]}
      />
      <hemisphereLight intensity={0.3} groundColor="#101820" />

      {/* Dedicated Yaskawa travel rail with the robot mounted on the carriage */}
      <RobotTrack length={4.6} carriage={railTravel} />
      <group position={[0, CARRIAGE_TOP_Y, railTravel]}>
        <Suspense key={`robot-${modelId}`} fallback={<Loading />}>
          <SelectedRobot
            modelId={modelId}
            joints={joints}
            base={[0, CARRIAGE_TOP_Y, railTravel]}
            program={program}
            simT={simT}
          />
        </Suspense>
      </group>

      {/* Two work stations, each with its own positioner */}
      {positioner &&
        STATION_POS.map((pos, i) => (
          <group key={`station-${i}`}>
            <Suspense key={`pos-${positioner.id}-${i}`} fallback={null}>
              <SelectedPositioner
                positioner={positioner}
                position={pos}
                tilt={stations[i]?.tilt ?? 0}
                rotate={stations[i]?.rotate ?? 0}
              />
            </Suspense>
            <StationLabel
              text={`Stół ${i + 1}`}
              position={[pos[0], 1.7, pos[2]]}
            />
          </group>
        ))}

      {/* Generated welding program (part, seam, torch frames, TCP marker) */}
      {program && <WeldScene program={program} simT={simT} />}

      {seams.map((s, i) => (
        <Seam key={i} seg={s} />
      ))}

      <Environment preset="warehouse" />
      <OrbitControls makeDefault enableDamping target={[1, 0.7, 0]} />
    </Canvas>
  );
}
