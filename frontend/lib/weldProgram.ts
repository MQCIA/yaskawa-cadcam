// Client-side welding "program" model — mirrors the backend path planner +
// DX200 postprocessor closely enough to demo a finished-looking welding job
// entirely in the browser (so it also works on the static GitHub Pages preview).
//
// Inspired by automatic offline-programming tools (ArcNC / MotoSim): a CAD part
// on the positioner, an auto-detected fillet seam, a generated weld path with
// approach/retract air-moves, torch-orientation frames, an ARCON/ARCOF waypoint
// list and a weld-schedule table.
//
// Everything here is an APPROXIMATE placeholder — validate in MotoSim before
// real hardware use.

import {
  computeDx200Durations,
  sampleDx200,
  type SimWarning,
} from "@/lib/dx200Sim";

export type Vec3 = [number, number, number];
/** INFORM: MOVC = circular weld; MOVS = TouchSense search (123123.JBI). */
export type MoveType = "MOVJ" | "MOVL" | "MOVC" | "MOVS";

export type Waypoint = {
  id: string;
  pos: Vec3;
  move: MoveType;
  speedLabel: string;
  arc: boolean;
  // ArcNC-style stages: sense = TouchSense; travel/clearance = air between seams;
  // index_safe = parked torch while EC# indexes (02025 C00163–C00165).
  kind:
    | "approach"
    | "sense"
    | "weld"
    | "travel"
    | "retract"
    | "clearance"
    | "index_safe";
  tag?: "ARCON" | "ARCOF" | "TOUCH";
  /**
   * Optional per-point TURN EC#(ec1,ec2) pulses — enables ST3 circumferential
   * welding (1-DEFRO-5.JBI: fixed C#, rotating axis-2).
   */
  ec?: [number, number];
  /** Optional RECT-X BS1 zone pulse (stable plateaus from 02025_KOSZ). */
  bc?: number;
  /**
   * Optional dwell after ARCON (s) — tack welds emit ``TIMER T=2.00`` like ``12.JBI``.
   */
  timerSec?: number;
  /**
   * Optional joint angles [S,L,U,R,B,T] deg — kept when a PULSE .JBI is opened
   * so re-export can stay in pulse units without re-running IK.
   */
  jointsDeg?: number[];
  /** INFORM C##### index from the job (may repeat on tack-in-place). */
  cIndex?: number;
  /** Raw PULSE S..T when imported from .JBI. */
  pulses?: number[];
};

export type WeldScheduleRow = {
  condition: number;
  job: number;
  currentA: number;
  voltageV: number;
  wireMmin: number;
  travelCmMin: number;
  process: string;
};

/** Optional solid for demo fixtures (pipe / tube) — rendered in WeldScene. */
export type WeldPipe = {
  /** Outer radius (m). */
  radius: number;
  /** Length along local axis (m). */
  length: number;
  /** Centre in world (m). */
  center: Vec3;
  /** Euler XYZ radians for the cylinder (Three.js default is Y-up). */
  rotation: Vec3;
};

export type WeldProgram = {
  name: string;
  mount: Vec3;
  plates: { size: Vec3; center: Vec3 }[];
  /** Optional pipe / tube solids (hard demo). */
  pipes?: WeldPipe[];
  seam: Vec3[];
  torchFrames: { pos: Vec3; dir: Vec3 }[];
  waypoints: Waypoint[];
  segDurations: number[];
  cycleSec: number;
  weldLenMm: number;
  schedule: WeldScheduleRow;
  /** ArcNC-style planner metadata (demo / placeholder). */
  meta?: {
    collisionFree: boolean;
    touchSense: boolean;
    sequenceOptimized: boolean;
    singularitySafe: boolean;
    /**
     * Torch roll about the wire axis (deg) chosen by the backend spin
     * optimiser — redundant DOF for MIG/MAG with a bent ABIROB neck.
     */
    spinAngleDeg?: number;
    /** Dense [S,L,U,R,B,T] deg along MOVL/MOVC for smooth URDF playback. */
    denseJointAnglesDeg?: number[][];
    /** Max consecutive joint step (deg) on the dense weld samples. */
    maxJointStepDeg?: number;
    /** Per-seam inspector overrides (spin / travel / approach / weave / V). */
    seamEdits?: import("@/lib/trajectoryViz").SeamEdit[];
    /** Active WEV# for visualisation (0 = no weave / no zigzag). */
    activeWeaveNo?: number;
    /** BC pulse at carriage Z = 0 (PULSE .JBI import). */
    bcRefPulse?: number;
    /** True when program came from a PULSE .JBI (XYZ aligned, not pendant world). */
    importedPulse?: boolean;
  };
};

/**
 * Lorch Tiptronic schedules mined from the DX200 dump (see docs/BACKUP_AND_LORCH_ANALYSIS.md).
 * ``job`` = MACRO1 MJ#(0) ARGFn = Tiptronic PROG-NR (MACRO.DAT tag PROG-NR: → job LORCH).
 * A/V come from the shared ARCSRT.CND template (200 A / 20.0 V); wire ≈ WELDER.DAT
 * curve at 200 A. Travel speeds match live jobs for that ARGF.
 */
export const LORCH_S8_SCHEDULES: WeldScheduleRow[] = [
  { condition: 1, job: 8, currentA: 200, voltageV: 20.0, wireMmin: 7.2, travelCmMin: 5.0, process: "Lorch Tiptronic" },
  { condition: 2, job: 5, currentA: 200, voltageV: 20.0, wireMmin: 7.2, travelCmMin: 4.2, process: "Lorch Tiptronic" },
  { condition: 3, job: 4, currentA: 200, voltageV: 20.0, wireMmin: 7.2, travelCmMin: 5.0, process: "Lorch Tiptronic" },
  { condition: 4, job: 1, currentA: 200, voltageV: 20.0, wireMmin: 7.2, travelCmMin: 5.0, process: "Lorch Tiptronic" },
];

/** Alias — Tiptronic PROG-NR / ARGF tables. */
export const MOTOWELD_SCHEDULES = LORCH_S8_SCHEDULES;

/** Weave files WEV#(n) from WEAV.CND (amp L/R mm, angle L/R °, frequency Hz). */
export type WeaveFileRow = {
  wevNo: number;
  mode: number;
  ampLmm: number;
  ampRmm: number;
  angleLdeg: number;
  angleRdeg: number;
  frequencyHz: number;
  label: string;
};

export const WEAVE_FILES: WeaveFileRow[] = [
  { wevNo: 1, mode: 0, ampLmm: 2.0, ampRmm: 2.0, angleLdeg: 70, angleRdeg: 70, frequencyHz: 2.0, label: "WEV#(1) sym 2 mm / 70° / 2 Hz" },
  { wevNo: 15, mode: 1, ampLmm: 3.0, ampRmm: 3.0, angleLdeg: 45, angleRdeg: 45, frequencyHz: 3.0, label: "WEV#(15) sym 3 mm / 45° / 3 Hz" },
  { wevNo: 16, mode: 1, ampLmm: 2.0, ampRmm: 2.0, angleLdeg: 45, angleRdeg: 0, frequencyHz: 2.0, label: "WEV#(16) 2 mm / 45°·0° / 2 Hz" },
  { wevNo: 21, mode: 0, ampLmm: 4.0, ampRmm: 0.0, angleLdeg: 0, angleRdeg: 0, frequencyHz: 2.0, label: "WEV#(21) 4 mm L / 2 Hz (default)" },
];

export const DEFAULT_WEAVE_NO = 21;

/** TouchSense search defaults from 123123.JBI. */
export const TOUCH_SENSE = {
  opcode: "MOVS" as const,
  searchSpeedCmMin: 11.0,
  stationVj: 0.78,
  referenceJob: "123123.JBI",
};

// ---------------------------------------------------------------------------
// TOOL 0 — single frontend source of truth (DX200 dump TOOL.CND / ///TOOL 0)
// Must match backend/app/robot_config.py exactly.
// ---------------------------------------------------------------------------

/** Motoman tcp_0 translation relative to Motoman tool0 (mm). */
export const TOOL0_XYZ_MM: readonly [number, number, number] = [
  -54.280, 1.052, 415.702,
];
/** Motoman tcp_0 RPY (Rx, Ry, Rz) degrees in tool0. */
export const TOOL0_RPY_DEG: readonly [number, number, number] = [
  1.3119, -42.6215, -6.8387,
];
export const TOOL0_XYZ_M: readonly [number, number, number] = [
  TOOL0_XYZ_MM[0] / 1000,
  TOOL0_XYZ_MM[1] / 1000,
  TOOL0_XYZ_MM[2] / 1000,
];
/** Euclidean tool0 → tcp_0 length (mm) ≈ 419.232. */
export const TOOL0_LENGTH_MM = Math.hypot(...TOOL0_XYZ_MM);
export const TOOL0_LENGTH_M = TOOL0_LENGTH_MM / 1000;

/**
 * Flange (REP-103) → Motoman tool0 rotation matrix (row-major 3×3).
 * Columns = tool0 basis in flange — same as backend FLANGE_TO_TOOL0_R.
 */
export const FLANGE_TO_TOOL0_R: readonly (readonly [number, number, number])[] = [
  [0, 0, 1],
  [0, -1, 0],
  [1, 0, 0],
];

/** Motoman TOOL RPY Euler order (matches kinematics_engine tool_file_se3). */
export const TOOL_RPY_ORDER = "ZYX" as const;

/**
 * Production home C00000 pulses (DEFRO / cell jobs).
 * Deg ≈ S=-0.21, L=-53.03, U=-52.87, R=+3.65, B=-81.80, T=-4.26.
 */
export const CELL_HOME_PULSES: readonly [
  number,
  number,
  number,
  number,
  number,
  number,
] = [-281, -101163, -84214, 3730, -80188, -1936];

/** MA2010 pulses/deg (RC1G softlimits) — same as backend pulse_config. */
export const MA2010_PULSES_PER_DEGREE: readonly [
  number,
  number,
  number,
  number,
  number,
  number,
] = [
  1341.3833333333334,
  1907.6768049155146,
  1592.8945494186046,
  1022.8666666666667,
  980.2444444444444,
  454.75714285714287,
];

export function cellHomeDeg(): [
  number,
  number,
  number,
  number,
  number,
  number,
] {
  return CELL_HOME_PULSES.map(
    (p, i) => p / MA2010_PULSES_PER_DEGREE[i],
  ) as [number, number, number, number, number, number];
}

// Mount point on work-station 1's positioner faceplate (world coords, metres).
export const DEMO_MOUNT: Vec3 = [1.1, 0.98, -1.2];
/** Hard demo (90° fillet + pipe) on station 2 — forces rail travel to +Z. */
export const DEMO_HARD_MOUNT: Vec3 = [1.1, 0.98, 1.2];

const add = (a: Vec3, b: Vec3): Vec3 => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const dist = (a: Vec3, b: Vec3) => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
const norm = (a: Vec3): Vec3 => {
  const l = Math.hypot(a[0], a[1], a[2]) || 1;
  return [a[0] / l, a[1] / l, a[2] / l];
};
const pad = (n: number) => `P${String(n).padStart(3, "0")}`;

type Aabb = { min: Vec3; max: Vec3 };

function pointInAabb(p: Vec3, box: Aabb, margin = 0.02): boolean {
  return (
    p[0] >= box.min[0] - margin &&
    p[0] <= box.max[0] + margin &&
    p[1] >= box.min[1] - margin &&
    p[1] <= box.max[1] + margin &&
    p[2] >= box.min[2] - margin &&
    p[2] <= box.max[2] + margin
  );
}

function segmentHitsAabb(a: Vec3, b: Vec3, box: Aabb, steps = 8): boolean {
  for (let i = 0; i <= steps; i++) {
    const t = i / steps;
    const p: Vec3 = [
      a[0] + (b[0] - a[0]) * t,
      a[1] + (b[1] - a[1]) * t,
      a[2] + (b[2] - a[2]) * t,
    ];
    if (pointInAabb(p, box)) return true;
  }
  return false;
}

/** Coarse cell obstacles (rail + H1000D pedestals) for demo collision checks. */
function cellObstacles(): Aabb[] {
  return [
    { min: [-0.25, 0.0, -2.4], max: [0.25, 0.28, 2.4] },
    { min: [0.85, 0.0, -1.55], max: [1.05, 0.9, -1.35] },
    { min: [1.15, 0.0, -1.55], max: [1.35, 0.9, -1.35] },
    { min: [0.85, 0.0, 1.05], max: [1.05, 0.9, 1.25] },
    { min: [1.15, 0.0, 1.05], max: [1.35, 0.9, 1.25] },
  ];
}

function plateObstacles(prog: WeldProgram): Aabb[] {
  return prog.plates.map((pl) => ({
    min: [
      pl.center[0] - pl.size[0] / 2,
      pl.center[1] - pl.size[1] / 2,
      pl.center[2] - pl.size[2] / 2,
    ] as Vec3,
    max: [
      pl.center[0] + pl.size[0] / 2,
      pl.center[1] + pl.size[1] / 2,
      pl.center[2] + pl.size[2] / 2,
    ] as Vec3,
  }));
}

/** True when TCP air-moves clear coarse cell / part AABBs. */
export function checkCollisionFree(prog: WeldProgram): boolean {
  const obstacles = [...cellObstacles(), ...plateObstacles(prog)];
  for (let i = 0; i < prog.waypoints.length - 1; i++) {
    const a = prog.waypoints[i];
    const b = prog.waypoints[i + 1];
    if (a.kind === "weld" && b.kind === "weld") continue;
    if (a.kind === "sense" || b.kind === "sense") continue;
    for (const box of obstacles) {
      if (segmentHitsAabb(a.pos, b.pos, box)) return false;
    }
  }
  return true;
}

function hasTouchSense(prog: WeldProgram): boolean {
  return prog.waypoints.some((w) => w.kind === "sense" || w.tag === "TOUCH");
}

/** Simple singularity heuristic: torch nearly parallel to world-up is risky. */
export function checkSingularitySafe(prog: WeldProgram): boolean {
  for (const fr of prog.torchFrames) {
    if (Math.abs(fr.dir[1]) < 0.15) return false;
  }
  return true;
}

function rebuildDurations(prog: WeldProgram): WeldProgram {
  const { segDurations, cycleSec } = computeDx200Durations(prog);
  return { ...prog, segDurations, cycleSec };
}

function renumberWaypoints(waypoints: Waypoint[]): Waypoint[] {
  return waypoints.map((w, i) => ({ ...w, id: pad(i + 1) }));
}

/** Re-tag ARCON/ARCOF on each contiguous weld run after edits. */
function retagWeldArcs(waypoints: Waypoint[]): Waypoint[] {
  const out = waypoints.map((w) => {
    if (w.kind === "weld" && (w.tag === "ARCON" || w.tag === "ARCOF")) {
      return { ...w, tag: undefined };
    }
    if (w.kind !== "weld" && (w.tag === "ARCON" || w.tag === "ARCOF")) {
      return { ...w, tag: undefined };
    }
    return w;
  });
  let i = 0;
  while (i < out.length) {
    if (out[i].kind !== "weld") {
      i++;
      continue;
    }
    const start = i;
    while (i < out.length && out[i].kind === "weld") i++;
    const end = i - 1;
    if (start === end) {
      // Tack / single sample — keep ARCON; ARCOF is emitted by the JBI writer
      // when ``timerSec`` is set (12.JBI pattern) or on the same point.
      out[start] = {
        ...out[start],
        tag: "ARCON",
        arc: true,
        timerSec: out[start].timerSec ?? 2.0,
      };
    } else {
      out[start] = { ...out[start], tag: "ARCON", arc: true };
      out[end] = { ...out[end], tag: "ARCOF", arc: true };
      for (let j = start + 1; j < end; j++) {
        out[j] = { ...out[j], arc: true, tag: undefined };
      }
    }
  }
  return out;
}

function finalizeWaypoints(prog: WeldProgram, waypoints: Waypoint[]): WeldProgram {
  const cleaned = renumberWaypoints(retagWeldArcs(waypoints));
  return refreshMeta(rebuildDurations({ ...prog, waypoints: cleaned }));
}

export type IdleInsertKind =
  | "travel"
  | "clearance"
  | "approach"
  | "retract"
  | "index_safe";

const IDLE_SPEED: Record<IdleInsertKind, string> = {
  travel: "VJ=50.00",
  clearance: "VJ=80.00",
  approach: "VJ=80.00",
  retract: "VJ=50.00",
  index_safe: "VJ=50.00",
};

/** Midpoint between two poses, optionally lifted in world Y (metres). */
function midLift(a: Vec3, b: Vec3, liftM: number): Vec3 {
  return [
    (a[0] + b[0]) / 2,
    (a[1] + b[1]) / 2 + liftM,
    (a[2] + b[2]) / 2,
  ];
}

/**
 * Delete a program waypoint by index. Keeps at least two points.
 * Rebuilds C# order, ARCON/ARCOF tags, and segment durations.
 */
export function deleteWaypoint(prog: WeldProgram, index: number): WeldProgram {
  if (prog.waypoints.length <= 2) return prog;
  if (index < 0 || index >= prog.waypoints.length) return prog;
  const waypoints = prog.waypoints.filter((_, i) => i !== index);
  return finalizeWaypoints(prog, waypoints);
}

/**
 * Insert an idle / via point after ``afterIndex`` (−1 = before first).
 * Default pose is the midpoint to the next point, lifted ``liftMm`` (clearance).
 */
export function insertIdleWaypoint(
  prog: WeldProgram,
  afterIndex: number,
  kind: IdleInsertKind = "clearance",
  opts?: { liftMm?: number; pos?: Vec3; torchDir?: Vec3 },
): WeldProgram {
  const n = prog.waypoints.length;
  if (n === 0) return prog;
  const liftM = (opts?.liftMm ?? (kind === "clearance" || kind === "index_safe" ? 150 : 40)) / 1000;
  const insertAt = Math.max(0, Math.min(n, afterIndex + 1));
  const prev = prog.waypoints[Math.max(0, insertAt - 1)] ?? prog.waypoints[0];
  const next = prog.waypoints[Math.min(n - 1, insertAt)] ?? prev;
  const pos = opts?.pos ?? midLift(prev.pos, next.pos, liftM);
  const wp: Waypoint = {
    id: "tmp",
    pos,
    move: "MOVJ",
    speedLabel: IDLE_SPEED[kind],
    arc: false,
    kind,
    bc: prev.bc ?? next.bc,
    ec: prev.ec ?? next.ec,
  };
  const waypoints = [
    ...prog.waypoints.slice(0, insertAt),
    wp,
    ...prog.waypoints.slice(insertAt),
  ];
  let torchFrames = prog.torchFrames;
  if (opts?.torchDir) {
    const dir = norm(opts.torchDir);
    torchFrames = [
      ...torchFrames.slice(0, insertAt),
      { pos, dir },
      ...torchFrames.slice(insertAt),
    ];
  }
  return finalizeWaypoints({ ...prog, torchFrames }, waypoints);
}

/** Insert a duplicate of the selected point offset slightly (manual tweak seed). */
export function duplicateWaypoint(
  prog: WeldProgram,
  index: number,
  kind: IdleInsertKind = "travel",
): WeldProgram {
  if (index < 0 || index >= prog.waypoints.length) return prog;
  const src = prog.waypoints[index];
  const pos: Vec3 = [src.pos[0], src.pos[1] + 0.05, src.pos[2]];
  return insertIdleWaypoint(prog, index, kind, { pos, liftMm: 0 });
}

/**
 * Motoman ``tcp_0`` wire / stick-out axis (+Z of TOOL.CND) from FK RPY (deg).
 * Matches backend ``SE3.RPY(..., order='xyz')`` = Rx·Ry·Rz.
 */
export function motomanRpyToWireDir(rx: number, ry: number, rz: number): Vec3 {
  const d = Math.PI / 180;
  const cx = Math.cos(rx * d);
  const sx = Math.sin(rx * d);
  const cy = Math.cos(ry * d);
  const sy = Math.sin(ry * d);
  const cz = Math.cos(rz * d);
  const sz = Math.sin(rz * d);
  // R = Rx·Ry·Rz; wire = R·[0,0,1] = third column [sy, −sx·cy, cx·cy].
  void cz;
  void sz;
  return norm([sy, -sx * cy, cx * cy]);
}

/** Rotate a direction by −table Z rotation (world → program frame). */
export function rotateDirIntoProgramFrame(dir: Vec3, tableRotateDeg: number): Vec3 {
  const partRotZ = (-tableRotateDeg * Math.PI) / 180;
  const c = Math.cos(-partRotZ);
  const s = Math.sin(-partRotZ);
  return norm([dir[0] * c - dir[1] * s, dir[0] * s + dir[1] * c, dir[2]]);
}

/** Overwrite XYZ of an existing waypoint (metres) — torch tip / Motoman ``tcp_0``. */
export function setWaypointPosition(
  prog: WeldProgram,
  index: number,
  pos: Vec3,
  opts?: { torchDir?: Vec3 },
): WeldProgram {
  if (index < 0 || index >= prog.waypoints.length) return prog;
  const waypoints = prog.waypoints.map((w, i) =>
    i === index ? { ...w, pos: [pos[0], pos[1], pos[2]] as Vec3 } : w,
  );
  let torchFrames = prog.torchFrames;
  if (opts?.torchDir) {
    const dir = norm(opts.torchDir);
    // Keep frame count in sync with seam when possible; also stamp nearest frame.
    if (torchFrames.length === 0) {
      torchFrames = [{ pos, dir }];
    } else {
      const fi = Math.max(
        0,
        Math.min(torchFrames.length - 1, Math.round(
          (index / Math.max(1, waypoints.length - 1)) * (torchFrames.length - 1),
        )),
      );
      torchFrames = torchFrames.map((f, i) =>
        i === fi ? { pos, dir } : f,
      );
    }
  }
  return finalizeWaypoints({ ...prog, torchFrames }, waypoints);
}

/** Change kind / move type of an existing waypoint (e.g. weld → travel). */
export function setWaypointKind(
  prog: WeldProgram,
  index: number,
  kind: Waypoint["kind"],
): WeldProgram {
  if (index < 0 || index >= prog.waypoints.length) return prog;
  const waypoints: Waypoint[] = prog.waypoints.map((w, i) => {
    if (i !== index) return w;
    const isWeld = kind === "weld";
    const isSense = kind === "sense";
    const move: MoveType = isSense
      ? "MOVS"
      : isWeld
        ? w.move === "MOVC"
          ? "MOVC"
          : "MOVL"
        : "MOVJ";
    const idleKey: IdleInsertKind = (
      ["travel", "clearance", "approach", "retract", "index_safe"] as string[]
    ).includes(kind)
      ? (kind as IdleInsertKind)
      : "travel";
    return {
      ...w,
      kind,
      move,
      arc: isWeld,
      speedLabel: isWeld
        ? `V=${prog.schedule.travelCmMin.toFixed(1)}`
        : isSense
          ? `V=${TOUCH_SENSE.searchSpeedCmMin}`
          : IDLE_SPEED[idleKey],
      tag: isSense ? "TOUCH" : undefined,
    };
  });
  return finalizeWaypoints(prog, waypoints);
}

export type SketchSeamKind = "tack" | "line" | "arc" | "spline";

export type SketchedSeamInput = {
  kind: SketchSeamKind;
  /** Sampled seam points in program / world frame (metres). */
  points: Vec3[];
  /** Per-point torch wire directions (into the joint). */
  dirs: Vec3[];
  approachMm?: number;
  label?: string;
};

/**
 * Append a hand-drawn seam (tack / MOVL / MOVC / spline) with approach + retract.
 * Keeps existing waypoints; stitches a clearance hop from the previous tip.
 */
export function appendSketchedSeam(
  prog: WeldProgram,
  input: SketchedSeamInput,
): WeldProgram {
  const pts = input.points;
  if (!pts.length) return prog;
  const dirs =
    input.dirs.length === pts.length
      ? input.dirs.map((d) => norm(d))
      : pts.map(() => norm(input.dirs[0] ?? ([0, -Math.SQRT1_2, -Math.SQRT1_2] as Vec3)));
  const approachMm = input.approachMm ?? 60;
  const lift = approachMm / 1000;
  const sch = prog.schedule;
  const speedLabel = `V=${sch.travelCmMin.toFixed(1)}`;
  const move: MoveType =
    input.kind === "arc" ? "MOVC" : input.kind === "tack" ? "MOVL" : "MOVL";

  const first = pts[0];
  const last = pts[pts.length - 1];
  const dir0 = dirs[0];
  const dirN = dirs[dirs.length - 1];
  const approachPos: Vec3 = [
    first[0] - dir0[0] * lift,
    first[1] - dir0[1] * lift,
    first[2] - dir0[2] * lift,
  ];
  const retractPos: Vec3 = [
    last[0] - dirN[0] * lift,
    last[1] - dirN[1] * lift,
    last[2] - dirN[2] * lift,
  ];

  const extra: Waypoint[] = [];
  if (prog.waypoints.length > 0) {
    const tip = prog.waypoints[prog.waypoints.length - 1].pos;
    const mid: Vec3 = [
      (tip[0] + approachPos[0]) / 2,
      Math.max(tip[1], approachPos[1]) + 0.08,
      (tip[2] + approachPos[2]) / 2,
    ];
    extra.push({
      id: "TMP",
      pos: mid,
      move: "MOVJ",
      speedLabel: "VJ=80.00",
      arc: false,
      kind: "clearance",
    });
  }
  extra.push({
    id: "TMP",
    pos: approachPos,
    move: "MOVJ",
    speedLabel: "VJ=80.00",
    arc: false,
    kind: "approach",
  });

  if (input.kind === "tack") {
    // 12.JBI pattern: ARCON → TIMER T=2.00 → ARCOF at one TCP.
    extra.push({
      id: "TMP",
      pos: first,
      move: "MOVL",
      speedLabel,
      arc: true,
      kind: "weld",
      tag: "ARCON",
      timerSec: 2.0,
    });
  } else {
    for (let i = 0; i < pts.length; i++) {
      extra.push({
        id: "TMP",
        pos: pts[i],
        move,
        speedLabel,
        arc: true,
        kind: "weld",
        tag:
          i === 0
            ? "ARCON"
            : i === pts.length - 1
              ? "ARCOF"
              : undefined,
      });
    }
  }

  extra.push({
    id: "TMP",
    pos: retractPos,
    move: "MOVJ",
    speedLabel: "VJ=50.00",
    arc: false,
    kind: "retract",
  });

  const waypoints = [...prog.waypoints, ...extra];
  const seam = [...prog.seam, ...pts];
  const torchFrames = [
    ...prog.torchFrames,
    ...pts.map((pos, i) => ({ pos, dir: dirs[i] })),
  ];

  let next = finalizeWaypoints({ ...prog, seam, torchFrames }, waypoints);

  // retagWeldArcs may drop timerSec — restore dwell on the last tack weld.
  if (input.kind === "tack") {
    const wps = [...next.waypoints];
    for (let i = wps.length - 1; i >= 0; i--) {
      if (wps[i].kind === "weld") {
        wps[i] = { ...wps[i], timerSec: 2.0 };
        break;
      }
    }
    next = rebuildDurations({ ...next, waypoints: wps });
  }

  let end = -1;
  for (let i = next.waypoints.length - 1; i >= 0; i--) {
    if (next.waypoints[i].kind === "weld") {
      end = i;
      break;
    }
  }
  let start = end;
  while (start > 0 && next.waypoints[start - 1].kind === "weld") start--;
  let approachIdx = start;
  for (let j = start - 1; j >= 0; j--) {
    if (next.waypoints[j].kind === "approach") {
      approachIdx = j;
      break;
    }
  }
  let retractIdx = end;
  for (let j = end + 1; j < next.waypoints.length; j++) {
    if (next.waypoints[j].kind === "retract") {
      retractIdx = j;
      break;
    }
  }
  const prevEdits = prog.meta?.seamEdits ?? [];
  const n = prevEdits.length + 1;
  const kindLabel =
    input.kind === "tack"
      ? "TACK"
      : input.kind === "arc"
        ? "ARC"
        : input.kind === "spline"
          ? "SPLINE"
          : "LINE";
  const seamEdits = [
    ...prevEdits,
    {
      id: `seam-sk-${Date.now().toString(36)}`,
      label: input.label ?? `Spoina ${n} (${kindLabel})`,
      weldStart: start,
      weldEnd: end,
      approachIdx,
      retractIdx,
      spinOffsetDeg: 0,
      travelAngleDeg: 12,
      approachMm,
      weaveNo: input.kind === "tack" ? 0 : 21,
      weldSpeedV: sch.travelCmMin,
      reversed: false,
      sketchKind: input.kind,
    },
  ];

  return withMeta(next, { seamEdits });
}

/** Remove one logical seam (approach…retract inclusive when dedicated). */
export function deleteSeam(
  prog: WeldProgram,
  weldStart: number,
  weldEnd: number,
): WeldProgram {
  if (weldStart < 0 || weldEnd < weldStart || weldEnd >= prog.waypoints.length) {
    return prog;
  }
  let from = weldStart;
  let to = weldEnd;
  for (let j = weldStart - 1; j >= 0; j--) {
    const k = prog.waypoints[j].kind;
    if (k === "approach" || k === "clearance" || k === "travel") {
      from = j;
      break;
    }
    if (k === "weld") break;
  }
  for (let j = weldEnd + 1; j < prog.waypoints.length; j++) {
    const k = prog.waypoints[j].kind;
    if (k === "retract" || k === "clearance" || k === "travel") {
      to = j;
      break;
    }
    if (k === "weld") break;
  }
  if (prog.waypoints.length - (to - from + 1) < 2) return prog;
  const waypoints = prog.waypoints.filter((_, i) => i < from || i > to);
  const seamEdits = (prog.meta?.seamEdits ?? []).filter(
    (e) => !(e.weldStart === weldStart && e.weldEnd === weldEnd),
  );
  return withMeta(finalizeWaypoints(prog, waypoints), { seamEdits });
}

function withMeta(
  prog: WeldProgram,
  patch: Partial<NonNullable<WeldProgram["meta"]>>,
): WeldProgram {
  const base = prog.meta ?? {
    collisionFree: checkCollisionFree(prog),
    touchSense: hasTouchSense(prog),
    sequenceOptimized: false,
    singularitySafe: checkSingularitySafe(prog),
  };
  return { ...prog, meta: { ...base, ...patch } };
}

function refreshMeta(prog: WeldProgram): WeldProgram {
  return withMeta(prog, {
    collisionFree: checkCollisionFree(prog),
    touchSense: hasTouchSense(prog),
    singularitySafe: checkSingularitySafe(prog),
  });
}

/** Build a T-fillet demo weld program mounted on the given positioner. */
export function buildDemoProgram(mount: Vec3 = DEMO_MOUNT, condition = 1): WeldProgram {
  const sch =
    LORCH_S8_SCHEDULES.find((s) => s.condition === condition) ?? LORCH_S8_SCHEDULES[0];

  const platesLocal: { size: Vec3; center: Vec3 }[] = [
    { size: [0.5, 0.02, 0.34], center: [0, 0.01, 0] },
    { size: [0.5, 0.24, 0.02], center: [0, 0.13, -0.15] },
  ];
  const plates = platesLocal.map((p) => ({ size: p.size, center: add(mount, p.center) }));

  const seamStart = add(mount, [-0.22, 0.03, -0.135]);
  const seamEnd = add(mount, [0.22, 0.03, -0.135]);

  const N = 12;
  const seam: Vec3[] = [];
  for (let i = 0; i <= N; i++) {
    const t = i / N;
    seam.push([
      seamStart[0] + (seamEnd[0] - seamStart[0]) * t,
      seamStart[1] + (seamEnd[1] - seamStart[1]) * t,
      seamStart[2] + (seamEnd[2] - seamStart[2]) * t,
    ]);
  }

  // Fillet approach: tcp_0 +Z into the T-joint (−bisector of floor/wall normals).
  const torchDir = norm([0, -1, -1]);
  const torchFrames = seam.map((pos) => ({ pos, dir: torchDir }));

  const lift: Vec3 = [0, 0.12, 0.08];
  const approach = add(seamStart, lift);
  const sense1 = add(seamStart, [0, 0.04, 0.02]);
  const sense2 = add(seamStart, [0.03, 0.04, 0.0]);
  const retract = add(seamEnd, lift);

  const weldLabel = `V=${sch.travelCmMin} cm/min`;
  const airLabel = "VJ=50%";
  // TouchSense search speed from 123123.JBI: MOVS … V=11.0
  const senseLabel = `V=${TOUCH_SENSE.searchSpeedCmMin} cm/min`;

  const waypoints: Waypoint[] = [];
  let n = 1;
  waypoints.push({
    id: pad(n++),
    pos: approach,
    move: "MOVJ",
    speedLabel: airLabel,
    arc: false,
    kind: "approach",
  });
  waypoints.push({
    id: pad(n++),
    pos: sense1,
    move: "MOVS",
    speedLabel: senseLabel,
    arc: false,
    kind: "sense",
    tag: "TOUCH",
  });
  waypoints.push({
    id: pad(n++),
    pos: sense2,
    move: "MOVS",
    speedLabel: senseLabel,
    arc: false,
    kind: "sense",
    tag: "TOUCH",
  });
  waypoints.push({
    id: pad(n++),
    pos: seamStart,
    move: "MOVL",
    speedLabel: "V=15 cm/min",
    arc: true,
    kind: "weld",
    tag: "ARCON",
  });
  for (let i = 1; i < N; i++) {
    waypoints.push({
      id: pad(n++),
      pos: seam[i],
      move: "MOVL",
      speedLabel: weldLabel,
      arc: true,
      kind: "weld",
    });
  }
  waypoints.push({
    id: pad(n++),
    pos: seamEnd,
    move: "MOVL",
    speedLabel: weldLabel,
    arc: true,
    kind: "weld",
    tag: "ARCOF",
  });
  waypoints.push({
    id: pad(n++),
    pos: retract,
    move: "MOVL",
    speedLabel: airLabel,
    arc: false,
    kind: "retract",
  });

  const draft: WeldProgram = {
    name: "DEMO_TFILLET",
    mount,
    plates,
    seam,
    torchFrames,
    waypoints,
    segDurations: [],
    cycleSec: 0,
    weldLenMm: dist(seamStart, seamEnd) * 1000,
    schedule: sch,
  };
  return refreshMeta(rebuildDurations(draft));
}

/**
 * Harder offline demo on station 2:
 *  1) Vertical 90° fillet (torch climbs a corner)
 *  2) Air travel
 *  3) Circumferential pipe-to-plate weld (MOVC ring)
 */
export function buildHardDemoProgram(
  mount: Vec3 = DEMO_HARD_MOUNT,
  condition = 1,
): WeldProgram {
  const sch =
    LORCH_S8_SCHEDULES.find((s) => s.condition === condition) ?? LORCH_S8_SCHEDULES[0];

  // --- Fixture: floor + vertical wall at 90°, plus Ø114 pipe on the floor ---
  const platesLocal: { size: Vec3; center: Vec3 }[] = [
    { size: [0.55, 0.02, 0.5], center: [0, 0.01, 0] },
    { size: [0.55, 0.36, 0.02], center: [0, 0.19, -0.24] },
  ];
  const plates = platesLocal.map((p) => ({ size: p.size, center: add(mount, p.center) }));

  const pipeRadius = 0.057;
  const pipeLen = 0.36;
  const pipeLocalCenter: Vec3 = [0.08, 0.02 + pipeRadius, 0.05];
  const pipes: WeldPipe[] = [
    {
      radius: pipeRadius,
      length: pipeLen,
      center: add(mount, pipeLocalCenter),
      // Cylinder default axis = Y; rotate to lie along +X on the floor plate.
      rotation: [0, 0, Math.PI / 2],
    },
  ];

  // Vertical 90° fillet seam (climbs the wall).
  const vStart = add(mount, [-0.2, 0.03, -0.23]);
  const vEnd = add(mount, [-0.2, 0.34, -0.23]);
  const Nv = 10;
  const seamV: Vec3[] = [];
  for (let i = 0; i <= Nv; i++) {
    const t = i / Nv;
    seamV.push([
      vStart[0] + (vEnd[0] - vStart[0]) * t,
      vStart[1] + (vEnd[1] - vStart[1]) * t,
      vStart[2] + (vEnd[2] - vStart[2]) * t,
    ]);
  }
  const torchDirV = norm([0, 0, 1]); // into the corner / wall

  // Circumferential seam on the pipe OD at the wall-side end (pipe-to-plate).
  const ringCenter = add(mount, [
    pipeLocalCenter[0] - pipeLen / 2 + 0.01,
    pipeLocalCenter[1],
    pipeLocalCenter[2],
  ]);
  const Nr = 16;
  const seamR: Vec3[] = [];
  for (let i = 0; i <= Nr; i++) {
    const a = (i / Nr) * Math.PI * 2;
    // Ring in YZ plane (pipe axis = X).
    seamR.push([
      ringCenter[0],
      ringCenter[1] + Math.cos(a) * pipeRadius,
      ringCenter[2] + Math.sin(a) * pipeRadius,
    ]);
  }

  const seam = [...seamV, ...seamR];
  const torchFrames = [
    ...seamV.map((pos) => ({ pos, dir: torchDirV })),
    ...seamR.map((pos) => {
      // Radial outward from pipe axis (in YZ).
      const dir = norm([
        0,
        pos[1] - ringCenter[1],
        pos[2] - ringCenter[2],
      ]);
      // Slight push toward −X (into the wall joint).
      return { pos, dir: norm([dir[0] - 0.35, dir[1], dir[2]]) };
    }),
  ];

  const weldLabel = `V=${sch.travelCmMin} cm/min`;
  const airNearLabel = "VJ=50.00";
  const airFarLabel = "VJ=80.00";
  const senseLabel = `V=${TOUCH_SENSE.searchSpeedCmMin} cm/min`;
  // Demo BS1 zones — left fillet vs ring (02025-style plateaus).
  const bcLeft = 124171;
  const bcMid = 250331;

  const approach = add(vStart, [0.05, 0.1, 0.12]);
  const sense1 = add(vStart, [0.03, 0.05, 0.04]);
  const sense2 = add(vStart, [0.0, 0.08, 0.04]);
  const midTravel = add(mount, [0.05, 0.45, -0.05]);
  const ringApproach = add(seamR[0], [0.08, 0.06, 0.06]);
  const retract = add(seamR[Math.floor(Nr / 4)], [0.1, 0.12, 0.08]);

  const waypoints: Waypoint[] = [];
  let n = 1;
  waypoints.push({
    id: pad(n++),
    pos: approach,
    move: "MOVJ",
    speedLabel: airFarLabel,
    arc: false,
    kind: "approach",
    bc: bcLeft,
  });
  waypoints.push({
    id: pad(n++),
    pos: sense1,
    move: "MOVS",
    speedLabel: senseLabel,
    arc: false,
    kind: "sense",
    tag: "TOUCH",
    bc: bcLeft,
  });
  waypoints.push({
    id: pad(n++),
    pos: sense2,
    move: "MOVS",
    speedLabel: senseLabel,
    arc: false,
    kind: "sense",
    tag: "TOUCH",
    bc: bcLeft,
  });
  waypoints.push({
    id: pad(n++),
    pos: seamV[0],
    move: "MOVL",
    speedLabel: "V=15 cm/min",
    arc: true,
    kind: "weld",
    tag: "ARCON",
    bc: bcLeft,
  });
  for (let i = 1; i < seamV.length - 1; i++) {
    waypoints.push({
      id: pad(n++),
      pos: seamV[i],
      move: "MOVL",
      speedLabel: weldLabel,
      arc: true,
      kind: "weld",
      bc: bcLeft,
    });
  }
  waypoints.push({
    id: pad(n++),
    pos: seamV[seamV.length - 1],
    move: "MOVL",
    speedLabel: weldLabel,
    arc: true,
    kind: "weld",
    tag: "ARCOF",
    bc: bcLeft,
  });
  // Clearance via between faces — VJ=80 DEC=20, BC indexes mid-air.
  waypoints.push({
    id: pad(n++),
    pos: midTravel,
    move: "MOVJ",
    speedLabel: airFarLabel,
    arc: false,
    kind: "clearance",
    bc: bcMid,
  });
  waypoints.push({
    id: pad(n++),
    pos: ringApproach,
    move: "MOVJ",
    speedLabel: airNearLabel,
    arc: false,
    kind: "travel",
    bc: bcMid,
  });
  waypoints.push({
    id: pad(n++),
    pos: seamR[0],
    move: "MOVC",
    speedLabel: "V=15 cm/min",
    arc: true,
    kind: "weld",
    tag: "ARCON",
    bc: bcMid,
  });
  for (let i = 1; i < seamR.length - 1; i++) {
    waypoints.push({
      id: pad(n++),
      pos: seamR[i],
      move: "MOVC",
      speedLabel: weldLabel,
      arc: true,
      kind: "weld",
      bc: bcMid,
    });
  }
  waypoints.push({
    id: pad(n++),
    pos: seamR[seamR.length - 1],
    move: "MOVC",
    speedLabel: weldLabel,
    arc: true,
    kind: "weld",
    tag: "ARCOF",
    bc: bcMid,
  });
  waypoints.push({
    id: pad(n++),
    pos: retract,
    move: "MOVJ",
    speedLabel: airNearLabel,
    arc: false,
    kind: "retract",
    bc: bcMid,
  });

  let weldLen = 0;
  for (let i = 0; i < seamV.length - 1; i++) weldLen += dist(seamV[i], seamV[i + 1]);
  for (let i = 0; i < seamR.length - 1; i++) weldLen += dist(seamR[i], seamR[i + 1]);

  const draft: WeldProgram = {
    name: "DEMO_HARD_90_PIPE",
    mount,
    plates,
    pipes,
    seam,
    torchFrames,
    waypoints,
    segDurations: [],
    cycleSec: 0,
    weldLenMm: weldLen * 1000,
    schedule: sch,
  };
  return refreshMeta(rebuildDurations(draft));
}

/** Insert TouchSense search points before the first weld if missing. */
export function ensureTouchSense(prog: WeldProgram): WeldProgram {
  if (hasTouchSense(prog)) return refreshMeta(prog);

  const firstWeld = prog.waypoints.findIndex((w) => w.kind === "weld");
  if (firstWeld < 0) return refreshMeta(prog);

  const seamStart = prog.seam[0] ?? prog.waypoints[firstWeld].pos;
  const sense1 = add(seamStart, [0, 0.04, 0.02]);
  const sense2 = add(seamStart, [0.03, 0.04, 0.0]);
  const insertAt = Math.max(1, firstWeld);
  const senseLabel = `V=${TOUCH_SENSE.searchSpeedCmMin} cm/min`;
  const extra: Waypoint[] = [
    {
      id: "TMP1",
      pos: sense1,
      move: "MOVS",
      speedLabel: senseLabel,
      arc: false,
      kind: "sense",
      tag: "TOUCH",
    },
    {
      id: "TMP2",
      pos: sense2,
      move: "MOVS",
      speedLabel: senseLabel,
      arc: false,
      kind: "sense",
      tag: "TOUCH",
    },
  ];
  const waypoints = [...prog.waypoints.slice(0, insertAt), ...extra, ...prog.waypoints.slice(insertAt)].map(
    (w, i) => ({ ...w, id: pad(i + 1) }),
  );
  return refreshMeta(rebuildDurations({ ...prog, waypoints }));
}

/** Contiguous weld runs as [start, end] inclusive waypoint indices. */
function weldRuns(waypoints: Waypoint[]): Array<{ start: number; end: number }> {
  const runs: Array<{ start: number; end: number }> = [];
  let i = 0;
  while (i < waypoints.length) {
    if (waypoints[i].kind !== "weld") {
      i++;
      continue;
    }
    const start = i;
    while (i < waypoints.length && waypoints[i].kind === "weld") i++;
    runs.push({ start, end: i - 1 });
  }
  return runs;
}

/**
 * Per-seam sequence optimize: reverse a weld run only when that shortens
 * travel from the previous air point (or approach) to the run start.
 * Never flattens multiple seams into one reverse (that broke hard-demo paths).
 */
export function optimizeSequence(prog: WeldProgram): WeldProgram {
  if (!prog.waypoints.length) {
    return withMeta(prog, { sequenceOptimized: true });
  }

  const runs = weldRuns(prog.waypoints);
  if (!runs.length) {
    return withMeta(prog, { sequenceOptimized: true });
  }

  const waypoints = prog.waypoints.map((w) => ({
    ...w,
    pos: [...w.pos] as Vec3,
  }));
  let seam = prog.seam.map((p) => [...p] as Vec3);
  let torchFrames = prog.torchFrames.map((f) => ({
    pos: [...f.pos] as Vec3,
    dir: [...f.dir] as Vec3,
  }));
  let reversedAny = false;

  for (const run of runs) {
    const n = run.end - run.start + 1;
    if (n < 2) continue;
    const prevIdx = run.start - 1;
    const from =
      prevIdx >= 0 ? waypoints[prevIdx].pos : waypoints[0].pos;
    const startPos = waypoints[run.start].pos;
    const endPos = waypoints[run.end].pos;
    const reverse =
      dist(from, endPos) + 1e-4 < dist(from, startPos);
    if (!reverse) continue;
    reversedAny = true;

    const slice = waypoints.slice(run.start, run.end + 1).reverse();
    for (let k = 0; k < slice.length; k++) {
      waypoints[run.start + k] = {
        ...slice[k],
        kind: "weld",
        arc: true,
        move: slice[k].move === "MOVC" ? "MOVC" : "MOVL",
        tag: undefined,
      };
    }

    // Best-effort: reverse matching seam / torch-frame span by equal count.
    if (seam.length === n) {
      seam = [...seam].reverse();
      torchFrames = [...torchFrames].reverse();
    } else if (seam.length >= n) {
      // Reverse only the head span that matches this run (first seam) once.
      // Subsequent runs leave geometry as-is if counts diverge.
      if (run === runs[0]) {
        const head = seam.slice(0, n).reverse();
        seam = [...head, ...seam.slice(n)];
        const th = torchFrames.slice(0, n).reverse();
        torchFrames = [...th, ...torchFrames.slice(n)];
      }
    }
  }

  if (!reversedAny) {
    return withMeta(prog, { sequenceOptimized: true });
  }

  // Reposition approach / retract standoffs near the (possibly reversed) tips.
  const firstRun = runs[0];
  const lastRun = runs[runs.length - 1];
  const ap = waypoints.findIndex((w) => w.kind === "approach");
  let rt = -1;
  for (let i = waypoints.length - 1; i >= 0; i--) {
    if (waypoints[i].kind === "retract") {
      rt = i;
      break;
    }
  }
  if (ap >= 0 && firstRun) {
    const tip = waypoints[firstRun.start].pos;
    waypoints[ap] = {
      ...waypoints[ap],
      pos: [tip[0] + 0.05, tip[1] + 0.1, tip[2] + 0.08],
    };
  }
  if (rt >= 0 && lastRun) {
    const tip = waypoints[lastRun.end].pos;
    waypoints[rt] = {
      ...waypoints[rt],
      pos: [tip[0] + 0.08, tip[1] + 0.1, tip[2] + 0.06],
    };
  }

  return withMeta(
    finalizeWaypoints({ ...prog, seam, torchFrames }, waypoints),
    { sequenceOptimized: true },
  );
}

const CLEARANCE_Z_LIFT_M = 0.15;
const CLEARANCE_LATERAL_M = 0.15;

function airHitsObstacles(prog: WeldProgram, a: Vec3, b: Vec3): boolean {
  const obstacles = [...cellObstacles(), ...plateObstacles(prog)];
  return obstacles.some((box) => segmentHitsAabb(a, b, box, 16));
}

/**
 * Insert MOVJ clearance vias between weld runs when the retract→approach chord
 * clips the part / cell AABB or spans a long lateral hop (backend safe_travel).
 */
export function insertSafeTravelClearance(prog: WeldProgram): WeldProgram {
  const wps = prog.waypoints;
  const runs = weldRuns(wps);
  if (runs.length < 2) return prog;

  const out: Waypoint[] = [];
  let i = 0;

  for (let r = 0; r < runs.length; r++) {
    const run = runs[r];
    let retractIdx = run.end;
    for (let j = run.end + 1; j < wps.length; j++) {
      if (wps[j].kind === "retract") {
        retractIdx = j;
        break;
      }
      if (wps[j].kind === "weld") break;
    }

    // Copy through this seam's retract.
    out.push(...wps.slice(i, retractIdx + 1));

    if (r === runs.length - 1) {
      i = retractIdx + 1;
      break;
    }

    const next = runs[r + 1];
    let approachIdx = next.start;
    for (let j = next.start - 1; j > run.end; j--) {
      if (wps[j].kind === "approach") {
        approachIdx = j;
        break;
      }
      if (wps[j].kind === "weld") break;
    }

    const from = wps[retractIdx];
    const to = wps[approachIdx];
    const gap = wps.slice(retractIdx + 1, approachIdx);
    const alreadyClearance = gap.some(
      (w) => w.kind === "clearance" || w.kind === "index_safe",
    );
    const lateral = Math.hypot(to.pos[0] - from.pos[0], to.pos[2] - from.pos[2]);
    const needs =
      !alreadyClearance &&
      (airHitsObstacles(prog, from.pos, to.pos) || lateral > CLEARANCE_LATERAL_M);

    if (alreadyClearance) {
      // Keep existing vias between retract and next approach.
      out.push(...gap);
    } else if (needs) {
      const zSafe =
        Math.max(from.pos[1], to.pos[1], prog.mount[1] + 0.15) + CLEARANCE_Z_LIFT_M;
      const highA: Vec3 = [from.pos[0], zSafe, from.pos[2]];
      const highB: Vec3 = [to.pos[0], zSafe, to.pos[2]];
      const mid: Vec3 = [
        (highA[0] + highB[0]) / 2,
        zSafe,
        (highA[2] + highB[2]) / 2,
      ];
      const pull = norm([-mid[0], 0, -mid[2]]);
      mid[0] += pull[0] * 0.08;
      mid[2] += pull[2] * 0.08;
      const mk = (pos: Vec3, vj: string): Waypoint => ({
        id: "TMP",
        pos,
        move: "MOVJ",
        speedLabel: vj,
        arc: false,
        kind: "clearance",
        bc: from.bc ?? to.bc,
        ec: from.ec ?? to.ec,
      });
      out.push(mk(highA, "VJ=80.00"));
      if (lateral > CLEARANCE_LATERAL_M) out.push(mk(mid, "VJ=80.00"));
      out.push(mk(highB, "VJ=50.00"));
    }
    // else: short clear air hop — leave gap empty (direct retract→approach)

    i = approachIdx;
  }

  if (i < wps.length) out.push(...wps.slice(i));
  return finalizeWaypoints(prog, out);
}

/** Re-run coarse motion checks, insert clearance vias, refresh planner flags. */
export function planMotions(prog: WeldProgram): WeldProgram {
  try {
    const withSense = ensureTouchSense(prog);
    const withTravel = insertSafeTravelClearance(withSense);
    const cleaned = finalizeWaypoints(withTravel, withTravel.waypoints);
    return refreshMeta(
      withMeta(cleaned, {
        sequenceOptimized: withSense.meta?.sequenceOptimized ?? true,
      }),
    );
  } catch {
    return refreshMeta(withMeta(prog, { sequenceOptimized: false }));
  }
}

const DEFAULT_TORCH_DIR: Vec3 = [0, -Math.SQRT1_2, -Math.SQRT1_2];

function torchDirAt(prog: WeldProgram, wpIndex: number): Vec3 {
  const frames = prog.torchFrames;
  if (!frames.length) return DEFAULT_TORCH_DIR;
  const i = Math.max(0, Math.min(frames.length - 1, wpIndex));
  return frames[i]?.dir ?? frames[0].dir;
}

export type ProgramSample = {
  pos: Vec3;
  dir: Vec3;
  wpIndex: number;
  arcOn: boolean;
  elapsedSec: number;
  jointsDeg?: number[];
  move?: MoveType | "TIMER";
  weaveActive?: boolean;
  warnings?: SimWarning[];
  pathClass?: "joint" | "cart" | "dwell";
  bc?: number;
  ecDeg?: number;
};

/** Interpolate TCP / joints along the program at progress p∈[0,1] (DX200 physics). */
export function sampleProgram(
  prog: WeldProgram,
  p: number,
  weaveNo = 0,
): ProgramSample {
  const s = sampleDx200(prog, p, weaveNo || prog.meta?.activeWeaveNo || 0);
  return {
    pos: s.pos,
    dir: s.dir,
    wpIndex: s.wpIndex,
    arcOn: s.arcOn,
    elapsedSec: s.elapsedSec,
    jointsDeg: s.jointsDeg,
    move: s.move,
    weaveActive: s.weaveActive,
    warnings: s.warnings,
    pathClass: s.pathClass,
    bc: s.bc,
    ecDeg: s.ecDeg,
  };
}

const f3 = (v: number) => v.toFixed(3);
const f4 = (v: number) => v.toFixed(4);
const f1 = (v: number) => v.toFixed(1);

/**
 * Pulse scaling mined from the production DX200 dump
 * (`reference/robot-cell-dx200`: RC1G softlimits + RBCALIB station step).
 */
export const CELL_PULSE = {
  /** S,L,U,R,B,T pulses per degree (RC1G ↔ MA2010 URDF limits). */
  robotPulsesPerDeg: [
    1341.3833333333334,
    1907.6768049155146,
    1592.8945494186046,
    1022.8666666666667,
    980.2444444444444,
    454.75714285714287,
  ] as const,
  /** TURN ST1: RBCALIB ΔE≈91771 over ~90°. */
  stationPulsesPerDeg: 91771 / 90,
  /** RECT-X park from jobs/000.JBI. */
  baseParkPulse: 878036,
} as const;

export function degToJointPulses(qDeg: number[]): number[] {
  if (qDeg.length !== 6) throw new Error("Expected 6 joint angles");
  return qDeg.map((q, i) => Math.round(q * CELL_PULSE.robotPulsesPerDeg[i]));
}

/** Approximate Motoman RECTAN Rx,Ry,Rz [deg] from torch direction (tool −Z). */
export function torchDirToRpy(dir: Vec3): [number, number, number] {
  const d = norm(dir);
  const rx = (Math.atan2(d[1], -d[2]) * 180) / Math.PI;
  const ry = (Math.asin(Math.max(-1, Math.min(1, -d[0]))) * 180) / Math.PI;
  return [rx, ry, 0];
}

/** Parse planner speed labels into DX200 V= / VJ= magnitudes. */
function parseSpeed(label: string, fallbackLinear: number): { kind: "V" | "VJ"; value: number } {
  const vj = label.match(/VJ\s*=\s*([\d.]+)/i);
  if (vj) return { kind: "VJ", value: Number(vj[1]) };
  const v = label.match(/V\s*=\s*([\d.]+)/i);
  if (v) {
    const n = Number(v[1]);
    // "cm/min" weld tags stay as V=; bare air VJ already handled above.
    if (/cm\/min/i.test(label)) return { kind: "V", value: n };
    return { kind: "V", value: n };
  }
  return { kind: "V", value: fallbackLinear };
}

/** TCP poses for backend IK (`/api/calculate-ik`). Positions in metres. */
export function waypointsToIkPoints(
  prog: WeldProgram,
): { x: number; y: number; z: number; rx: number; ry: number; rz: number }[] {
  return prog.waypoints.map((p, i) => {
    const [rx, ry, rz] = torchDirToRpy(torchDirAt(prog, i));
    return { x: p.pos[0], y: p.pos[1], z: p.pos[2], rx, ry, rz };
  });
}

/**
 * Client-side DX200 .JBI generator aligned with the production MA2010 cell
 * (`reference/robot-cell-dx200/jobs/000.JBI`): GROUP1 RB1,BS1 + GROUP2 ST1,
 * ARCON/WVON, coordinated +MOVJ EC#.
 * Pass `jointAnglesDeg` ([S,L,U,R,B,T] per waypoint) for full PULSE C# export;
 * otherwise C# stays RECTAN for offline review.
 */
export function generateJbi(
  prog: WeldProgram,
  opts?: {
    userFrame?: number;
    stationPulsesPerDeg?: number;
    stationDeg?: number | number[];
    /** Per-point EC#(ec1,ec2) pulses (overrides stationDeg). ST3 circumferential. */
    ecPulses?: Array<[number, number] | number[]>;
    weaveNo?: number;
    jointAnglesDeg?: number[][];
    baseParkPulse?: number;
    group2?: string;
  },
): string {
  const pts = prog.waypoints;
  const npos = pts.length;
  const pulsesPerDeg = opts?.stationPulsesPerDeg ?? CELL_PULSE.stationPulsesPerDeg;
  const basePark = opts?.baseParkPulse ?? CELL_PULSE.baseParkPulse;
  const weaveNo = opts?.weaveNo ?? 21;
  const group2 = (opts?.group2 ?? "ST1").toUpperCase().replace(/\s+/g, "");
  const jointsFromWaypoints =
    pts.every((p) => Array.isArray(p.jointsDeg) && p.jointsDeg!.length === 6)
      ? pts.map((p) => p.jointsDeg as number[])
      : undefined;
  const jointAnglesDeg =
    Array.isArray(opts?.jointAnglesDeg) && opts!.jointAnglesDeg!.length === npos
      ? opts!.jointAnglesDeg!
      : jointsFromWaypoints;
  const usePulse = Array.isArray(jointAnglesDeg) && jointAnglesDeg.length === npos;
  const stationDeg = pts.map((_, i) => {
    if (Array.isArray(opts?.stationDeg)) return opts!.stationDeg![i] ?? opts!.stationDeg![0] ?? 0;
    return opts?.stationDeg ?? 0;
  });
  const ecPairs: [number, number][] = pts.map((p, i) => {
    if (opts?.ecPulses && opts.ecPulses[i]) {
      const e = opts.ecPulses[i];
      return [Math.round(e[0]), Math.round(e[1] ?? 0)];
    }
    if (p.ec) return [Math.round(p.ec[0]), Math.round(p.ec[1])];
    return [Math.round(stationDeg[i] * pulsesPerDeg), 0];
  });
  const lines: string[] = [];
  const sch = prog.schedule;

  lines.push("/JOB");
  lines.push(`//NAME ${prog.name}`);
  if (usePulse) {
    lines.push("' PULSE export — MA2010 cell (RC1G / RBCALIB scaling)");
  } else {
    lines.push("' Offline RECTAN review — supply IK joints for PULSE C#");
  }
  lines.push("' Cell: MA2010 + RECT-X (BS1) + TURN (ST1–ST4); TOOL0 TCP Z≈415.7 mm");
  lines.push(`' Station pulses/deg~${pulsesPerDeg.toFixed(4)}; BC park=${basePark}`);
  lines.push("' MOVC = circular weld (≥3 pts); per-point EC = ST3 circumferential");
  lines.push("//POS");
  lines.push(`///NPOS ${npos},${npos},${npos},0,0,0`);
  lines.push("///TOOL 0");

  if (usePulse && jointAnglesDeg) {
    lines.push("///POSTYPE PULSE");
    lines.push("///PULSE");
    jointAnglesDeg.forEach((q, i) => {
      const pulses = degToJointPulses(q);
      lines.push(`C${String(i).padStart(5, "0")}=${pulses.join(",")}`);
    });
  } else {
    lines.push("///POSTYPE USER");
    lines.push("///RECTAN");
    lines.push(`///RCONF ${Array(24).fill(0).join(",")}`);
    pts.forEach((p, i) => {
      const x = p.pos[0] * 1000;
      const y = p.pos[1] * 1000;
      const z = p.pos[2] * 1000;
      const [rx, ry, rz] = torchDirToRpy(torchDirAt(prog, i));
      lines.push(
        `C${String(i).padStart(5, "0")}=${f3(x)},${f3(y)},${f3(z)},${f4(rx)},${f4(ry)},${f4(rz)}`,
      );
    });
  }

  lines.push(`' BC# RECT-X — per-point zone (park fallback ${basePark})`);
  pts.forEach((p, i) => {
    const bcPulse =
      typeof p.bc === "number" && Number.isFinite(p.bc) ? Math.round(p.bc) : basePark;
    lines.push(`BC${String(i).padStart(5, "0")}=${bcPulse}`);
  });

  if (!usePulse) {
    lines.push("///POSTYPE PULSE");
    lines.push("///PULSE");
  }
  ecPairs.forEach(([e1, e2], i) => {
    lines.push(`EC${String(i).padStart(5, "0")}=${e1},${e2}`);
  });

  lines.push("//INST");
  const now = new Date();
  const stamp = `${now.getFullYear()}/${String(now.getMonth() + 1).padStart(2, "0")}/${String(
    now.getDate(),
  ).padStart(2, "0")} ${String(now.getHours()).padStart(2, "0")}:${String(
    now.getMinutes(),
  ).padStart(2, "0")}`;
  lines.push(`///DATE ${stamp}`);
  lines.push("///ATTR SC,RW");
  lines.push("///GROUP1 RB1,BS1");
  lines.push(`///GROUP2 ${group2}`);
  lines.push("NOP");

  pts.forEach((p, i) => {
    const c = `C${String(i).padStart(5, "0")}`;
    const bc = `BC${String(i).padStart(5, "0")}`;
    const ec = `EC${String(i).padStart(5, "0")}`;
    const defaultVj =
      p.kind === "clearance" || p.kind === "index_safe" || p.kind === "approach"
        ? 80
        : p.kind === "travel" || p.kind === "retract"
          ? 50
          : 80;
    const parsed = parseSpeed(
      p.speedLabel,
      p.kind === "weld" ? sch.travelCmMin : defaultVj,
    );
    const useJoint =
      p.move === "MOVJ" ||
      p.kind === "approach" ||
      p.kind === "retract" ||
      p.kind === "travel" ||
      p.kind === "clearance" ||
      p.kind === "index_safe";

    if (p.tag === "ARCON") {
      const isTack = (p.timerSec ?? 0) >= 1.5;
      if (!isTack) {
        lines.push("TIMER T=0.50");
        lines.push(`MACRO1 MJ#(0) ARGF${sch.job}`);
        lines.push(
          `' Lorch Tiptronic PROG-NR/ARGF${sch.job} (UI cond ${sch.condition}): ` +
            `${sch.currentA}A / ${sch.voltageV.toFixed(1)}V / wire ${sch.wireMmin.toFixed(1)} m/min ` +
            `— ARCSRT template; validate WPS`,
        );
      }
      lines.push("ARCON");
      if (isTack) {
        // 12.JBI tack: ARCON / TIMER T=2.00 / ARCOF (no weave).
        lines.push(`TIMER T=${(p.timerSec ?? 2).toFixed(2)}`);
      } else {
        lines.push(`WVON WEV#(${weaveNo})`);
      }
    }

    if (p.kind === "sense" || p.move === "MOVS") {
      // 123123.JBI: MOVS C# BC# V=11.0  +MOVJ EC# VJ=0.78
      const v =
        parsed.kind === "V" ? parsed.value : TOUCH_SENSE.searchSpeedCmMin;
      lines.push(
        `MOVS ${c} ${bc} V=${f1(v)}  +MOVJ ${ec} VJ=${f1(TOUCH_SENSE.stationVj)}`,
      );
      lines.push(`' TouchSense search (${TOUCH_SENSE.referenceJob} pattern)`);
    } else if (useJoint && p.kind !== "weld") {
      const vj = parsed.kind === "VJ" ? parsed.value : defaultVj;
      lines.push(`MOVJ ${c} ${bc} VJ=${f1(vj)} DEC=20  +MOVJ ${ec} VJ=${f1(vj)}`);
    } else {
      const v =
        p.kind === "weld"
          ? parsed.kind === "V"
            ? parsed.value
            : sch.travelCmMin
          : 5.0;
      const op = p.move === "MOVC" ? "MOVC" : "MOVL";
      lines.push(`${op} ${c} ${bc} V=${f1(v)} DEC=20  +MOVJ ${ec} VJ=80.00`);
    }

    if (p.tag === "ARCOF") {
      lines.push("ARCOF");
      lines.push("WVOF");
    } else if (p.tag === "ARCON" && (p.timerSec ?? 0) >= 1.5) {
      // Tack: ARCON + TIMER already written; close without weave.
      lines.push("ARCOF");
    }
  });
  lines.push("END");

  return `${lines.join("\n")}\n`;
}
