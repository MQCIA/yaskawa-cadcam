"use client";

import { Suspense } from "react";
import { Canvas } from "@react-three/fiber";
import { OrbitControls, Html } from "@react-three/drei";
import YaskawaManipulator from "./YaskawaManipulator";
import UrdfModel from "./UrdfModel";
import H1000dPositioner from "./H1000dPositioner";
import RobotTrack from "./RobotTrack";
import WeldScene from "./WeldScene";
import WeldUrdfRobot from "./WeldUrdfRobot";
import WeldPowerSource from "./WeldPowerSource";
import type { Joints } from "./AxisSliders";
import {
  ROBOT_MODELS,
  POSITIONER_MODELS,
  type PositionerModel,
} from "@/lib/models";
import { STATION_FRAMES, type StationId, type WeldProgram } from "@/lib/weldProgram";

export type SeamSegment = {
  start: [number, number, number];
  end: [number, number, number];
};

export type StationState = { tilt: number; rotate: number };

// Height of the rail carriage top the robot base mounts on.
const CARRIAGE_TOP_Y = 0.26;
// Height of the positioner rotation axis (matches H1000dPositioner.axisHeight).
const AXLE_Y = 0.95;
// X position of the trailing power-source dolly on the carriage (matches RobotTrack).
const DOLLY_X = -0.62;

/** Floor anchors for ST1..ST4 from mined ``STATION_FRAMES`` (rail along Z). */
export const STATION_ORDER: StationId[] = ["ST1", "ST2", "ST3", "ST4"];
/** TURN-2 (ST3/ST4) offset along X so both positioners at a rail end are visible. */
const TURN2_X_OFFSET = 0.55;
export const STATION_POS: [number, number, number][] = STATION_ORDER.map((id) => {
  const f = STATION_FRAMES[id];
  const xOff = id === "ST3" || id === "ST4" ? TURN2_X_OFFSET : 0;
  // Pedestal under the weld cloud; X from mount (+ TURN-2 offset), Z from mean BC.
  return [f.mount[0] + xOff, 0, f.railZ];
});

const deg2rad = (d: number) => (d * Math.PI) / 180;

/** Index of the station a part is mounted on (GROUP2 label, else nearest rail anchor). */
function mountStationIndex(program: WeldProgram): number {
  if (program.station) {
    const idx = STATION_ORDER.indexOf(program.station);
    if (idx >= 0) return idx;
  }
  const mount = program.mount;
  let best = 0;
  let bestD = Infinity;
  STATION_POS.forEach((p, i) => {
    const d = Math.hypot(p[0] - mount[0], p[2] - mount[2]);
    if (d < bestD) {
      bestD = d;
      best = i;
    }
  });
  return best;
}

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
      <div className="rounded border border-slate-200 bg-white/90 px-3 py-1 text-xs text-slate-700">
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
      <div className="whitespace-nowrap rounded border border-slate-200 bg-white/90 px-2 py-0.5 text-xs font-medium text-slate-700">
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
  manualJog,
  pathJointsDeg,
  partPivot,
  partRotZ,
}: {
  modelId: string;
  joints: Joints;
  base: [number, number, number];
  program: WeldProgram | null;
  simT: number;
  manualJog: boolean;
  pathJointsDeg?: number[][] | null;
  partPivot?: [number, number, number];
  partRotZ?: number;
}) {
  const model = ROBOT_MODELS.find((m) => m.id === modelId) ?? ROBOT_MODELS[0];
  if (model.kind === "urdf" && model.url) {
    // Real Yaskawa model with a torch: backend-IK path, CCD fallback, or jog.
    return (
      <WeldUrdfRobot
        url={model.url}
        color={model.color}
        program={program}
        simT={simT}
        base={base}
        joints={joints}
        manual={manualJog}
        pathJointsDeg={pathJointsDeg}
        partPivot={partPivot}
        partRotZ={partRotZ}
      />
    );
  }
  return <YaskawaManipulator joints={joints} />;
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
    { tilt: 0, rotate: 0 },
    { tilt: 0, rotate: 0 },
  ],
  program = null,
  simT = 0,
  manualJog = false,
  pathJointsDeg = null,
}: {
  modelId: string;
  joints: Joints;
  seams?: SeamSegment[];
  positionerId?: string | null;
  railTravel?: number;
  stations?: StationState[];
  program?: WeldProgram | null;
  simT?: number;
  manualJog?: boolean;
  /** Per-waypoint [S,L,U,R,B,T] deg from `/api/calculate-ik`. */
  pathJointsDeg?: number[][] | null;
}) {
  const positioner = positionerId
    ? POSITIONER_MODELS.find((p) => p.id === positionerId)
    : undefined;

  // The workpiece is clamped to the positioner table, so it rotates with it.
  // Compute the pivot (axle) and rotation for the mounting station.
  const partStation = program ? mountStationIndex(program) : 0;
  const partPivot: [number, number, number] = [
    STATION_POS[partStation][0],
    AXLE_Y,
    STATION_POS[partStation][2],
  ];
  // Table turns about the axle (world Z after the positioner's 90° yaw); a
  // positive positioner "rotate" maps to a negative rotation about world Z.
  const partRotZ = -deg2rad(stations[partStation]?.rotate ?? 0);

  // Fill the flex parent explicitly — without absolute inset-0 the R3F canvas
  // can intermittently collapse to 0×0 height in the Verbotics-style layout.
  // Skip <Environment> (HDR fetch) — it flakes on restricted networks and
  // leaves a blank viewer; local lights are enough for the cell.
  return (
    <div className="absolute inset-0 h-full w-full">
    <Canvas
      camera={{ position: [4.5, 3, 4.5], fov: 45 }}
      gl={{ antialias: true, alpha: false, powerPreference: "high-performance" }}
      dpr={[1, 1.75]}
      style={{ width: "100%", height: "100%", display: "block" }}
      onCreated={({ gl }) => {
        gl.setClearColor("#ffffff", 1);
      }}
    >
      <color attach="background" args={["#ffffff"]} />
      <ambientLight intensity={0.7} />
      <directionalLight position={[5, 8, 5]} intensity={1.05} />
      <hemisphereLight intensity={0.4} groundColor="#d8dee6" />

      {/* Dedicated Yaskawa travel rail with the robot mounted on the carriage */}
      <Suspense fallback={null}>
        <RobotTrack length={4.6} carriage={railTravel} />
      </Suspense>
      <group position={[0, CARRIAGE_TOP_Y, railTravel]}>
        <Suspense key={`robot-${modelId}`} fallback={<Loading />}>
          <SelectedRobot
            modelId={modelId}
            joints={joints}
            base={[0, CARRIAGE_TOP_Y, railTravel]}
            program={program}
            simT={simT}
            manualJog={manualJog}
            pathJointsDeg={pathJointsDeg}
            partPivot={partPivot}
            partRotZ={partRotZ}
          />
        </Suspense>
        {/* MOTOWELD-E power source on the trailing dolly — rides with the robot */}
        <WeldPowerSource position={[DOLLY_X, 0, 0]} />
      </group>

      {/* Four work stations (ST1..ST4) at mined rail / mount anchors */}
      {positioner &&
        STATION_POS.map((pos, i) => (
          <group key={`station-${STATION_ORDER[i]}`}>
            <Suspense key={`pos-${positioner.id}-${i}`} fallback={null}>
              <SelectedPositioner
                positioner={positioner}
                position={pos}
                tilt={stations[i]?.tilt ?? 0}
                rotate={stations[i]?.rotate ?? 0}
              />
            </Suspense>
            <StationLabel
              text={`Stół ${i + 1} (${STATION_ORDER[i]})`}
              position={[pos[0], 1.7, pos[2]]}
            />
          </group>
        ))}

      {/* Generated welding program (part, seam, torch frames, TCP marker).
          Clamped to the positioner table, so it rotates with it about the axle. */}
      {program && (
        <group position={partPivot}>
          <group rotation={[0, 0, partRotZ]}>
            <group position={[-partPivot[0], -partPivot[1], -partPivot[2]]}>
              <WeldScene program={program} simT={simT} />
            </group>
          </group>
        </group>
      )}

      {seams.map((s, i) => (
        <Seam key={i} seg={s} />
      ))}

      <OrbitControls makeDefault enableDamping target={[1, 0.7, 0]} />
    </Canvas>
    </div>
  );
}
