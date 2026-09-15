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
  job: number;
  currentA: number;
  voltageV: number;
  wireMmin: number;
  travelCmMin: number;
  process: string;
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

// Mirror of backend welding schedules — Motoweld-E cell (travel V≈5–8).
export const LORCH_S8_SCHEDULES: WeldScheduleRow[] = [
  { condition: 1, job: 8, currentA: 180, voltageV: 22.0, wireMmin: 6.5, travelCmMin: 5.0, process: "Motoweld" },
  { condition: 2, job: 8, currentA: 220, voltageV: 24.0, wireMmin: 8.0, travelCmMin: 6.7, process: "Motoweld" },
  { condition: 3, job: 4, currentA: 140, voltageV: 19.0, wireMmin: 5.0, travelCmMin: 7.5, process: "Motoweld" },
];

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
    weaveNo?: number;
    jointAnglesDeg?: number[][];
    baseParkPulse?: number;
  },
): string {
  const pts = prog.waypoints;
  const npos = pts.length;
  const pulsesPerDeg = opts?.stationPulsesPerDeg ?? CELL_PULSE.stationPulsesPerDeg;
  const basePark = opts?.baseParkPulse ?? CELL_PULSE.baseParkPulse;
  const weaveNo = opts?.weaveNo ?? 21;
  const usePulse =
    Array.isArray(opts?.jointAnglesDeg) && opts!.jointAnglesDeg!.length === npos;
  const stationDeg = pts.map((_, i) => {
    if (Array.isArray(opts?.stationDeg)) return opts!.stationDeg![i] ?? opts!.stationDeg![0] ?? 0;
    return opts?.stationDeg ?? 0;
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
  lines.push("' Cell: MA2010 + RECT-X (BS1) + TURN (ST1); TOOL0 TCP Z≈415.7 mm");
  lines.push(`' Station pulses/deg~${pulsesPerDeg.toFixed(4)}; BC park=${basePark}`);
  lines.push("//POS");
  lines.push(`///NPOS ${npos},${npos},${npos},0,0,0`);
  lines.push("///TOOL 0");

  if (usePulse) {
    lines.push("///POSTYPE PULSE");
    lines.push("///PULSE");
    opts!.jointAnglesDeg!.forEach((q, i) => {
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

  lines.push(`' BC# RECT-X park pulse (${basePark})`);
  pts.forEach((_, i) => lines.push(`BC${String(i).padStart(5, "0")}=${basePark}`));

  if (!usePulse) {
    lines.push("///POSTYPE PULSE");
    lines.push("///PULSE");
  }
  stationDeg.forEach((ang, i) => {
    lines.push(`EC${String(i).padStart(5, "0")}=${Math.round(ang * pulsesPerDeg)},0`);
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
  lines.push("///GROUP2 ST1");
  lines.push("NOP");

  pts.forEach((p, i) => {
    const c = `C${String(i).padStart(5, "0")}`;
    const bc = `BC${String(i).padStart(5, "0")}`;
    const ec = `EC${String(i).padStart(5, "0")}`;
    const parsed = parseSpeed(p.speedLabel, p.kind === "weld" ? sch.travelCmMin : 80);
    const useJoint = p.move === "MOVJ" || p.kind === "approach" || p.kind === "retract" || p.kind === "travel";

    if (p.tag === "ARCON") {
      lines.push("TIMER T=0.50");
      lines.push(`MACRO1 MJ#(0) ARGF${sch.job}`);
      lines.push(
        `' Motoweld cond ${sch.condition}: ${sch.currentA}A / ${sch.voltageV.toFixed(1)}V / wire ${sch.wireMmin.toFixed(
          1,
        )} m/min / ${sch.process}`,
      );
      lines.push("ARCON");
      lines.push(`WVON WEV#(${weaveNo})`);
    }

    if (useJoint && p.kind !== "weld" && p.kind !== "sense") {
      const vj = parsed.kind === "VJ" ? parsed.value : 80;
      lines.push(`MOVJ ${c} ${bc} VJ=${f1(vj)} DEC=20  +MOVJ ${ec} VJ=${f1(vj)}`);
    } else {
      const v = p.kind === "weld" || p.kind === "sense" ? (parsed.kind === "V" ? parsed.value : sch.travelCmMin) : 5.0;
      lines.push(`MOVL ${c} ${bc} V=${f1(v)} DEC=20  +MOVJ ${ec} VJ=80.00`);
    }

    if (p.tag === "TOUCH") {
      lines.push("' TouchSense search (CAD↔reality offset)");
    }
    if (p.tag === "ARCOF") {
      lines.push("ARCOF");
      lines.push("WVOF");
    }
  });
  lines.push("END");

  return `${lines.join("\n")}\n`;
}
