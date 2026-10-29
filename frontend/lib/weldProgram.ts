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

export type Vec3 = [number, number, number];
export type MoveType = "MOVJ" | "MOVL";

export type Waypoint = {
  id: string;
  pos: Vec3;
  move: MoveType;
  speedLabel: string;
  arc: boolean;
  // ArcNC-style stages: sense = TouchSense search, travel = air move between seams
  kind: "approach" | "sense" | "weld" | "travel" | "retract";
  tag?: "ARCON" | "ARCOF" | "TOUCH";
};

export type WeldScheduleRow = {
  condition: number;
  /** MACRO1 MJ#(0) ARGFn — from production .JBI (pendant condition). */
  argf: number;
  /** UI estimate only — verify on pendant / ARCSRT.CND. */
  currentA: number;
  voltageV: number;
  wireMmin: number;
  travelCmMin: number;
  process: string;
  argfSource: string;
  paramsSource: string;
};

export type WeldProgram = {
  name: string;
  mount: Vec3;
  plates: { size: Vec3; center: Vec3 }[];
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
  };
};

/** Cell identity shown in UI headers. */
export const CELL_LABEL = "DX200 · MA2010 · RECT-X · TURN-ST1 · MOTOWELD-E 350A";

/**
 * Mirror of backend MOTOWELD_SCHEDULES.
 * `argf` comes from production MACRO1 lines; amps/volts are UI placeholders.
 */
export const MOTOWELD_SCHEDULES: WeldScheduleRow[] = [
  {
    condition: 1,
    argf: 1,
    currentA: 180,
    voltageV: 22.0,
    wireMmin: 6.5,
    travelCmMin: 5.0,
    process: "MOTOWELD-E",
    argfSource: "production .JBI MACRO1 ARGF",
    paramsSource: "UI placeholder — verify on pendant / ARCSRT.CND",
  },
  {
    condition: 2,
    argf: 4,
    currentA: 220,
    voltageV: 24.0,
    wireMmin: 8.0,
    travelCmMin: 6.7,
    process: "MOTOWELD-E",
    argfSource: "production .JBI MACRO1 ARGF",
    paramsSource: "UI placeholder — verify on pendant / ARCSRT.CND",
  },
  {
    condition: 3,
    argf: 5,
    currentA: 140,
    voltageV: 19.0,
    wireMmin: 5.0,
    travelCmMin: 7.5,
    process: "MOTOWELD-E",
    argfSource: "production .JBI MACRO1 ARGF",
    paramsSource: "UI placeholder — verify on pendant / ARCSRT.CND",
  },
];

/** Weave files seen in production jobs (optional; omit to skip WVON). */
export const PRODUCTION_WEAVE_NOS = [1, 15, 16, 21, 22, 24, 25] as const;

/** Safe home over the rail (HOME_TRACK / CELL_POSES). */
export const SAFE_HOME_PULSES = [-283, -101162, -84213, 3710, -80188, -1936] as const;
export const SAFE_HOME_EC: [number, number] = [192, 0];

// Mount point on work-station 1's positioner faceplate (world coords, metres).
export const DEMO_MOUNT: Vec3 = [1.6, 0.98, -1.2];

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
    { min: [1.35, 0.0, -1.55], max: [1.55, 0.9, -1.35] },
    { min: [1.65, 0.0, -1.55], max: [1.85, 0.9, -1.35] },
    { min: [1.35, 0.0, 1.05], max: [1.55, 0.9, 1.25] },
    { min: [1.65, 0.0, 1.05], max: [1.85, 0.9, 1.25] },
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
  const airSpeed = 0.5;
  const weldSpeed = prog.schedule.travelCmMin / 100 / 60;
  const segDurations: number[] = [];
  for (let i = 0; i < prog.waypoints.length - 1; i++) {
    const a = prog.waypoints[i];
    const b = prog.waypoints[i + 1];
    const welding = a.arc && b.arc;
    segDurations.push(dist(a.pos, b.pos) / (welding ? weldSpeed : airSpeed));
  }
  const cycleSec = segDurations.reduce((s, d) => s + d, 0);
  return { ...prog, segDurations, cycleSec };
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
    MOTOWELD_SCHEDULES.find((s) => s.condition === condition) ?? MOTOWELD_SCHEDULES[0];

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

  const torchDir = norm([0, -1, -1]);
  const torchFrames = seam.map((pos) => ({ pos, dir: torchDir }));

  const lift: Vec3 = [0, 0.12, 0.08];
  const approach = add(seamStart, lift);
  const sense1 = add(seamStart, [0, 0.04, 0.02]);
  const sense2 = add(seamStart, [0.03, 0.04, 0.0]);
  const retract = add(seamEnd, lift);

  const weldLabel = `V=${sch.travelCmMin} cm/min`;
  const airLabel = "VJ=50%";
  const senseLabel = "V=5 cm/min";

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
    move: "MOVL",
    speedLabel: senseLabel,
    arc: false,
    kind: "sense",
    tag: "TOUCH",
  });
  waypoints.push({
    id: pad(n++),
    pos: sense2,
    move: "MOVL",
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

/** Insert TouchSense search points before the first weld if missing. */
export function ensureTouchSense(prog: WeldProgram): WeldProgram {
  if (hasTouchSense(prog)) return refreshMeta(prog);

  const firstWeld = prog.waypoints.findIndex((w) => w.kind === "weld");
  if (firstWeld < 0) return refreshMeta(prog);

  const seamStart = prog.seam[0] ?? prog.waypoints[firstWeld].pos;
  const sense1 = add(seamStart, [0, 0.04, 0.02]);
  const sense2 = add(seamStart, [0.03, 0.04, 0.0]);
  const insertAt = Math.max(1, firstWeld);
  const senseLabel = "V=5 cm/min";
  const extra: Waypoint[] = [
    {
      id: "TMP1",
      pos: sense1,
      move: "MOVL",
      speedLabel: senseLabel,
      arc: false,
      kind: "sense",
      tag: "TOUCH",
    },
    {
      id: "TMP2",
      pos: sense2,
      move: "MOVL",
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

/** Mark sequence optimized; optionally reverse seam if that shortens approach. */
export function optimizeSequence(prog: WeldProgram): WeldProgram {
  const approach = prog.waypoints.find((w) => w.kind === "approach");
  if (!approach || prog.seam.length < 2) {
    return withMeta(prog, { sequenceOptimized: true });
  }

  const start = prog.seam[0];
  const end = prog.seam[prog.seam.length - 1];
  const reverse = dist(approach.pos, end) + 0.001 < dist(approach.pos, start);
  if (!reverse) return withMeta(prog, { sequenceOptimized: true });

  const seam = [...prog.seam].reverse();
  const torchFrames = [...prog.torchFrames].reverse();
  const firstWeld = prog.waypoints.findIndex((w) => w.kind === "weld");
  const lastWeld = (() => {
    for (let i = prog.waypoints.length - 1; i >= 0; i--) {
      if (prog.waypoints[i].kind === "weld") return i;
    }
    return -1;
  })();
  if (firstWeld < 0 || lastWeld < 0) return withMeta(prog, { sequenceOptimized: true });

  const weldCount = lastWeld - firstWeld + 1;
  const weldWps = prog.waypoints.slice(firstWeld, lastWeld + 1).reverse();
  const waypoints = [
    ...prog.waypoints.slice(0, firstWeld),
    ...weldWps.map((w, i) => {
      const tag =
        i === 0 ? ("ARCON" as const) : i === weldCount - 1 ? ("ARCOF" as const) : undefined;
      return {
        ...w,
        pos: seam[Math.min(i, seam.length - 1)],
        tag,
        arc: true,
        kind: "weld" as const,
      };
    }),
    ...prog.waypoints.slice(lastWeld + 1),
  ].map((w, i) => ({ ...w, id: pad(i + 1) }));

  return withMeta(rebuildDurations({ ...prog, seam, torchFrames, waypoints }), {
    sequenceOptimized: true,
  });
}

/** Re-run coarse motion checks and refresh planner flags. */
export function planMotions(prog: WeldProgram): WeldProgram {
  const withSense = ensureTouchSense(prog);
  return refreshMeta(
    withMeta(withSense, {
      sequenceOptimized: withSense.meta?.sequenceOptimized ?? true,
    }),
  );
}

const DEFAULT_TORCH_DIR: Vec3 = [0, -Math.SQRT1_2, -Math.SQRT1_2];

function torchDirAt(prog: WeldProgram, wpIndex: number): Vec3 {
  const frames = prog.torchFrames;
  if (!frames.length) return DEFAULT_TORCH_DIR;
  const i = Math.max(0, Math.min(frames.length - 1, wpIndex));
  return frames[i]?.dir ?? frames[0].dir;
}

/** Interpolate the TCP along the program at progress p in [0,1]. */
export function sampleProgram(prog: WeldProgram, p: number) {
  const target = Math.max(0, Math.min(1, p)) * prog.cycleSec;
  let acc = 0;
  for (let i = 0; i < prog.segDurations.length; i++) {
    const d = prog.segDurations[i];
    if (target <= acc + d || i === prog.segDurations.length - 1) {
      const local = d > 0 ? (target - acc) / d : 0;
      const a = prog.waypoints[i].pos;
      const b = prog.waypoints[i + 1].pos;
      const pos: Vec3 = [
        a[0] + (b[0] - a[0]) * local,
        a[1] + (b[1] - a[1]) * local,
        a[2] + (b[2] - a[2]) * local,
      ];
      return {
        pos,
        dir: torchDirAt(prog, i),
        wpIndex: i,
        arcOn: prog.waypoints[i].arc && prog.waypoints[i + 1].arc,
        elapsedSec: target,
      };
    }
    acc += d;
  }
  const last = prog.waypoints[prog.waypoints.length - 1];
  return {
    pos: last.pos,
    dir: torchDirAt(prog, prog.waypoints.length - 1),
    wpIndex: prog.waypoints.length - 1,
    arcOn: false,
    elapsedSec: prog.cycleSec,
  };
}

const f3 = (v: number) => v.toFixed(3);
const f4 = (v: number) => v.toFixed(4);
const f1 = (v: number) => v.toFixed(1);

/**
 * Pulse scaling mined from the production DX200 dump
 * (`reference/robot-cell-dx200`: RC1G softlimits + RBCALIB station step).
 *
 * RECT-X / BS1: no rack-lead register in the kept RC1G text. `pulsesPerMm` is
 * calibrated from taught BC envelopes (ST2 mid ↔ ST1/ST3 mid) over the 2400 mm
 * station span. Encoder SV1G = 4096 alone is not enough without lead/gear.
 */
const _BC_ST2_MID = (124171 + 375361) / 2;
const _BC_ST13_MID = (808680 + 927851) / 2;
const _RECT_X_PULSES_PER_MM = (_BC_ST13_MID - _BC_ST2_MID) / 2400;

export const BC_HOME_PULSE = 570125;
export const BC_PARK_PULSE = 878036;
/** Alias kept for older call sites. */
export const BASE_PARK_PULSE = BC_PARK_PULSE;
export const PULSES_PER_MM = _RECT_X_PULSES_PER_MM;

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
  /** RECT-X home / rail centre (production C00000 jobs). */
  baseHomePulse: BC_HOME_PULSE,
  /** RECT-X park from jobs/000.JBI. */
  baseParkPulse: BC_PARK_PULSE,
  /** Calibrated pulses per millimetre along BS1. */
  pulsesPerMm: PULSES_PER_MM,
  /** Taught station midpoints (BC#). */
  stationBcMid: {
    ST1: Math.round(_BC_ST13_MID),
    ST2: Math.round(_BC_ST2_MID),
    ST3: Math.round(_BC_ST13_MID),
    ST4: Math.round(_BC_ST2_MID),
  } as const,
} as const;

export function degToJointPulses(qDeg: number[]): number[] {
  if (qDeg.length !== 6) throw new Error("Expected 6 joint angles");
  return qDeg.map((q, i) => Math.round(q * CELL_PULSE.robotPulsesPerDeg[i]));
}

/** RECT-X travel in mm → INFORM BC#. ``mm = 0`` at ``BC_HOME_PULSE``. */
export function trackMmToPulses(
  mm: number,
  originPulse: number = BC_HOME_PULSE,
  pulsesPerMm: number = PULSES_PER_MM,
): number {
  return Math.round(originPulse + mm * pulsesPerMm);
}

/** INFORM BC# → millimetres from rail home. */
export function trackPulsesToMm(
  pulses: number,
  originPulse: number = BC_HOME_PULSE,
  pulsesPerMm: number = PULSES_PER_MM,
): number {
  return (pulses - originPulse) / pulsesPerMm;
}

/**
 * World Z (metres if |z|<10, else mm) → track mm from home.
 * Rail along Z; ST1 at Z≈−1.2 m (high BC), ST2 at Z≈+1.2 m (low BC).
 */
export function trackMmFromWorldZ(z: number): number {
  const zMm = Math.abs(z) < 10 ? z * 1000 : z;
  return -zMm;
}

/** Build per-point BC# for a job of length `npos`. */
export function resolveBasePulses(
  npos: number,
  opts?: {
    basePulse?: number | number[];
    baseMm?: number | number[];
    station?: string;
    /** World points with `.z` — used when nothing else is set. */
    points?: { z: number }[];
    defaultPark?: boolean;
  },
): number[] {
  const bp = opts?.basePulse;
  if (typeof bp === "number") return Array(npos).fill(bp);
  if (Array.isArray(bp) && bp.length) {
    if (bp.length === 1) return Array(npos).fill(bp[0]);
    if (bp.length !== npos) throw new Error(`basePulse length ${bp.length} != npos ${npos}`);
    return bp.map((p) => Math.round(p));
  }

  const bm = opts?.baseMm;
  if (typeof bm === "number") return Array(npos).fill(trackMmToPulses(bm));
  if (Array.isArray(bm) && bm.length) {
    if (bm.length === 1) return Array(npos).fill(trackMmToPulses(bm[0]));
    if (bm.length !== npos) throw new Error(`baseMm length ${bm.length} != npos ${npos}`);
    return bm.map((m) => trackMmToPulses(m));
  }

  if (opts?.station) {
    const key = opts.station.toUpperCase().replace(/\s+/g, "") as keyof typeof CELL_PULSE.stationBcMid;
    const mid = CELL_PULSE.stationBcMid[key];
    if (mid == null) throw new Error(`Unknown station '${opts.station}'. Use ST1..ST4.`);
    return Array(npos).fill(mid);
  }

  if (opts?.points && opts.points.length === npos) {
    return opts.points.map((p) => trackMmToPulses(trackMmFromWorldZ(p.z)));
  }

  const fill = opts?.defaultPark ? BC_PARK_PULSE : BC_HOME_PULSE;
  return Array(npos).fill(fill);
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
 * Client-side DX200 .JBI generator aligned with production MA2010 cell jobs:
 * NPOS n,n,n / TOOL 0 / PULSE / GROUP1 RB1,BS1 + GROUP2 STn / home bookends /
 * MACRO1+ARCON / optional WVON / DEC=20 / CRLF.
 */
export function generateJbi(
  prog: WeldProgram,
  opts?: {
    userFrame?: number;
    stationPulsesPerDeg?: number;
    stationDeg?: number | number[];
    /** WEV#(n); omit/undefined → 21. Pass null to disable weave. */
    weaveNo?: number | null;
    jointAnglesDeg?: number[][];
    /** Legacy: force every BC# to this park/home pulse. */
    baseParkPulse?: number;
    basePulse?: number | number[];
    baseMm?: number | number[];
    station?: string;
    macroArg?: number;
    bookendHome?: boolean;
  },
): string {
  const pathPts = prog.waypoints;
  const pathN = pathPts.length;
  const pulsesPerDeg = opts?.stationPulsesPerDeg ?? CELL_PULSE.stationPulsesPerDeg;
  const weaveNo = opts?.weaveNo === undefined ? 21 : opts.weaveNo;
  const usePulse =
    Array.isArray(opts?.jointAnglesDeg) && opts!.jointAnglesDeg!.length === pathN;
  const doBookend = (opts?.bookendHome ?? true) && usePulse;
  const group2 = (opts?.station ?? "ST1").toUpperCase().replace(/\s+/g, "");
  const sch = prog.schedule;
  const macroArg = opts?.macroArg ?? sch.argf;

  const pathStationDeg = pathPts.map((_, i) => {
    if (Array.isArray(opts?.stationDeg)) return opts!.stationDeg![i] ?? opts!.stationDeg![0] ?? 0;
    return opts?.stationDeg ?? 0;
  });
  const pathBc = resolveBasePulses(pathN, {
    basePulse: opts?.basePulse ?? opts?.baseParkPulse,
    baseMm: opts?.baseMm,
    station: opts?.station,
    points: pathPts.map((p) => ({ z: p.pos[2] })),
  });

  type Row = {
    kind: "home" | "path";
    pathIndex: number;
    bc: number;
    ec: [number, number];
    qDeg?: number[];
  };
  const rows: Row[] = [];
  if (doBookend) {
    rows.push({
      kind: "home",
      pathIndex: -1,
      bc: BC_HOME_PULSE,
      ec: [...SAFE_HOME_EC],
    });
  }
  pathPts.forEach((_, i) => {
    rows.push({
      kind: "path",
      pathIndex: i,
      bc: pathBc[i],
      ec: [Math.round(pathStationDeg[i] * pulsesPerDeg), 0],
      qDeg: usePulse ? opts!.jointAnglesDeg![i] : undefined,
    });
  });
  if (doBookend) {
    rows.push({
      kind: "home",
      pathIndex: -1,
      bc: BC_HOME_PULSE,
      ec: [...SAFE_HOME_EC],
    });
  }
  const npos = rows.length;

  const lines: string[] = [];
  lines.push("/JOB");
  lines.push(`//NAME ${prog.name}`);
  if (usePulse) {
    lines.push("' PULSE export — DX200 MA2010 + RECT-X + TURN + MOTOWELD-E");
  } else {
    lines.push("' Offline RECTAN review — supply IK joints for PULSE C#");
  }
  lines.push(`' Cell: ${CELL_LABEL}; TOOL0 TCP Z≈415.7 mm`);
  lines.push(
    `' Station pulses/deg~${pulsesPerDeg.toFixed(4)}; RECT-X ~${PULSES_PER_MM.toFixed(4)} p/mm ` +
      `(home BC=${BC_HOME_PULSE}, park=${BC_PARK_PULSE})`,
  );
  if (doBookend) {
    lines.push(
      `' Home bookends C00000/C${String(npos - 1).padStart(5, "0")} = ${SAFE_HOME_PULSES.join(",")} BC=${BC_HOME_PULSE}`,
    );
  }
  lines.push("//POS");
  lines.push(`///NPOS ${npos},${npos},${npos},0,0,0`);
  lines.push("///TOOL 0");

  if (usePulse) {
    lines.push("///POSTYPE PULSE");
    lines.push("///PULSE");
    rows.forEach((row, i) => {
      const pulses =
        row.kind === "home" ? [...SAFE_HOME_PULSES] : degToJointPulses(row.qDeg!);
      lines.push(`C${String(i).padStart(5, "0")}=${pulses.join(",")}`);
    });
  } else {
    lines.push("///POSTYPE USER");
    lines.push("///RECTAN");
    lines.push(`///RCONF ${Array(24).fill(0).join(",")}`);
    rows.forEach((row, i) => {
      const p = pathPts[Math.max(0, row.pathIndex)] ?? pathPts[0];
      const x = p.pos[0] * 1000;
      const y = p.pos[1] * 1000;
      const z = p.pos[2] * 1000;
      const [rx, ry, rz] = torchDirToRpy(torchDirAt(prog, Math.max(0, row.pathIndex)));
      lines.push(
        `C${String(i).padStart(5, "0")}=${f3(x)},${f3(y)},${f3(z)},${f4(rx)},${f4(ry)},${f4(rz)}`,
      );
    });
  }

  lines.push(
    `' BC# RECT-X BS1 (~${PULSES_PER_MM.toFixed(4)} p/mm; 0 mm @ BC=${BC_HOME_PULSE})`,
  );
  rows.forEach((row, i) => lines.push(`BC${String(i).padStart(5, "0")}=${row.bc}`));

  if (!usePulse) {
    lines.push("///POSTYPE PULSE");
    lines.push("///PULSE");
  }
  rows.forEach((row, i) => {
    lines.push(`EC${String(i).padStart(5, "0")}=${row.ec[0]},${row.ec[1]}`);
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

  let weaveOn = false;
  rows.forEach((row, i) => {
    const c = `C${String(i).padStart(5, "0")}`;
    const bc = `BC${String(i).padStart(5, "0")}`;
    const ec = `EC${String(i).padStart(5, "0")}`;

    if (row.kind === "home") {
      lines.push(`MOVJ ${c} ${bc} VJ=80.00 DEC=20  +MOVJ ${ec} VJ=80.00`);
      return;
    }

    const p = pathPts[row.pathIndex];
    const parsed = parseSpeed(p.speedLabel, p.kind === "weld" ? sch.travelCmMin : 80);
    const useJoint =
      p.move === "MOVJ" || p.kind === "approach" || p.kind === "retract" || p.kind === "travel";

    if (p.tag === "ARCON") {
      lines.push("TIMER T=0.50");
      lines.push(`MACRO1 MJ#(0) ARGF${macroArg}`);
      lines.push(
        `' MOTOWELD-E cond ${sch.condition} / MACRO1 ARGF${macroArg}: ${sch.currentA}A / ${sch.voltageV.toFixed(1)}V / wire ${sch.wireMmin.toFixed(
          1,
        )} m/min — UI estimate; verify ARCSRT.CND`,
      );
      lines.push("ARCON");
      if (weaveNo != null) {
        lines.push(`WVON WEV#(${weaveNo})`);
        weaveOn = true;
      }
    }

    if (useJoint && p.kind !== "weld" && p.kind !== "sense") {
      const vj = parsed.kind === "VJ" ? parsed.value : 80;
      lines.push(`MOVJ ${c} ${bc} VJ=${f1(vj)} DEC=20  +MOVJ ${ec} VJ=${f1(vj)}`);
    } else {
      const v =
        p.kind === "weld" || p.kind === "sense"
          ? parsed.kind === "V"
            ? parsed.value
            : sch.travelCmMin
          : 5.0;
      lines.push(`MOVL ${c} ${bc} V=${f1(v)} DEC=20  +MOVJ ${ec} VJ=80.00`);
    }

    if (p.tag === "TOUCH") {
      lines.push("' TouchSense search (CAD↔reality offset)");
    }
    if (p.tag === "ARCOF") {
      if (weaveOn) {
        lines.push("WVOF");
        weaveOn = false;
      }
      lines.push("ARCOF");
    }
  });
  lines.push("END");

  return `${lines.join("\r\n")}\r\n`;
}
