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

type SeamSpan = { start: Vec3; end: Vec3; weldIdx: number[] };

/** Split waypoints into weld groups (ARCON…ARCOF) for multi-seam NN. */
function extractSeamSpans(prog: WeldProgram): SeamSpan[] {
  const spans: SeamSpan[] = [];
  let cur: number[] = [];
  prog.waypoints.forEach((w, i) => {
    if (w.tag === "ARCON" || (w.kind === "weld" && cur.length === 0)) cur = [i];
    else if (w.kind === "weld") cur.push(i);
    if (w.tag === "ARCOF" || (w.kind === "weld" && i + 1 < prog.waypoints.length && prog.waypoints[i + 1].kind !== "weld")) {
      if (cur.length) {
        const a = prog.waypoints[cur[0]].pos;
        const b = prog.waypoints[cur[cur.length - 1]].pos;
        spans.push({ start: a, end: b, weldIdx: [...cur] });
        cur = [];
      }
    }
  });
  if (cur.length) {
    spans.push({
      start: prog.waypoints[cur[0]].pos,
      end: prog.waypoints[cur[cur.length - 1]].pos,
      weldIdx: cur,
    });
  }
  return spans;
}

/** Nearest-neighbour seam order with optional start↔end flip (M2.6). */
export function orderSeamsNearestNeighbor(
  seams: { start: Vec3; end: Vec3 }[],
  seed: Vec3 = CELL_PULSE.cellHomeTcpMm.map((v) => v / 1000) as Vec3,
): { order: number[]; flipped: boolean[] } {
  if (!seams.length) return { order: [], flipped: [] };
  const rem = seams.map((_, i) => i);
  const order: number[] = [];
  const flipped: boolean[] = [];
  let cur: Vec3 = seed;
  while (rem.length) {
    let best = rem[0];
    let bestFlip = false;
    let bestD = Infinity;
    for (const i of rem) {
      const a = seams[i].start;
      const b = seams[i].end;
      const da = dist(cur, a);
      const db = dist(cur, b);
      if (da <= db && da < bestD) {
        bestD = da;
        best = i;
        bestFlip = false;
      } else if (db < bestD) {
        bestD = db;
        best = i;
        bestFlip = true;
      }
    }
    order.push(best);
    flipped.push(bestFlip);
    rem.splice(rem.indexOf(best), 1);
    cur = bestFlip ? seams[best].start : seams[best].end;
  }
  return { order, flipped };
}

/**
 * Mark sequence optimized. Multi-seam: NN order + direction flip to cut air
 * MOVJ. Single seam: reverse weld if that shortens approach from home/seed.
 */
export function optimizeSequence(prog: WeldProgram): WeldProgram {
  const spans = extractSeamSpans(prog);
  if (spans.length <= 1) {
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

  // Multi-seam NN: rebuild approach/weld/retract/travel chain.
  const { order, flipped } = orderSeamsNearestNeighbor(spans);
  const lift: Vec3 = [0, 0.08, 0];
  const waypoints: Waypoint[] = [];
  const push = (w: Omit<Waypoint, "id">) =>
    waypoints.push({ ...w, id: pad(waypoints.length + 1) });

  // Home bookend (safe start).
  const homePos: Vec3 = CELL_PULSE.cellHomeTcpMm.map((v) => v / 1000) as Vec3;
  push({
    pos: homePos,
    move: "MOVJ",
    speedLabel: "VJ=80",
    arc: false,
    kind: "travel",
  });

  order.forEach((si, k) => {
    const span = spans[si];
    const flip = flipped[k];
    const weldPts = span.weldIdx.map((i) => prog.waypoints[i].pos);
    const chain = flip ? [...weldPts].reverse() : weldPts;
    const start = chain[0];
    const end = chain[chain.length - 1];
    if (k > 0) {
      // Safe inter-seam MOVJ (retract → approach) with DEC=20 in JBI export.
      push({
        pos: add(start, lift),
        move: "MOVJ",
        speedLabel: "VJ=80 DEC=20",
        arc: false,
        kind: "travel",
      });
    }
    push({
      pos: add(start, lift),
      move: "MOVJ",
      speedLabel: "VJ=80",
      arc: false,
      kind: "approach",
    });
    chain.forEach((pos, i) => {
      push({
        pos,
        move: "MOVL",
        speedLabel: `V=${prog.schedule.travelCmMin}`,
        arc: true,
        kind: "weld",
        tag: i === 0 ? "ARCON" : i === chain.length - 1 ? "ARCOF" : undefined,
      });
    });
    push({
      pos: add(end, lift),
      move: "MOVJ",
      speedLabel: "VJ=80",
      arc: false,
      kind: "retract",
    });
  });

  push({
    pos: homePos,
    move: "MOVJ",
    speedLabel: "VJ=80",
    arc: false,
    kind: "travel",
  });

  const seam = waypoints.filter((w) => w.kind === "weld").map((w) => w.pos);
  const fallbackDir: Vec3 = [0, -Math.SQRT1_2, -Math.SQRT1_2];
  const torchFrames = seam.map((pos) => ({
    pos,
    dir: prog.torchFrames[0]?.dir ?? fallbackDir,
  }));

  return withMeta(
    rebuildDurations({ ...prog, seam, torchFrames, waypoints }),
    { sequenceOptimized: true },
  );
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
 * RECT-X lead is NOT in RC1G/RC2G → ``rectXLeadVerified=false`` and
 * ``rectXPulsesPerMm`` is UNVERIFIED_PLACEHOLDER (mirror of pulse_config.py).
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
  /** Production bookend BC# (1-DEFRO-5 / 02025_KOSZ… C00000). */
  bcHomePulse: 570125,
  /** RECT-X park from jobs/000.JBI. */
  bcPark000Pulse: 878036,
  /** @deprecated alias of bcPark000Pulse */
  baseParkPulse: 878036,
  st2BcRange: [124171, 375361] as const,
  st3BcRange: [808680, 847209] as const,
  st1BcMax: 927851,
  /** SV1G encoder PPR. */
  encoderResolution: 4096,
  /** RC1G[8] travel softlimit hint. */
  baseSoftlimitPulse: 1082000,
  /**
   * UNVERIFIED_PLACEHOLDER — dump has no ballscrew lead mm.
   * TODO: replace after rail teach or RECT-X datasheet lead×gear.
   */
  rectXPulsesPerMm: 1000,
  rectXLeadVerified: false,
  /** Taught home C# pulses (POSE_01 / production bookend). */
  cellHomePulses: [-281, -101163, -84214, 3730, -80188, -1936] as const,
  /** APP_ESTIMATE home TCP mm (URDF FK × TOOL.CND). */
  cellHomeTcpMm: [1041.0, -32.7, 1182.0] as const,
} as const;

export type StationId = "ST1" | "ST2" | "ST3" | "ST4";

/** Mirror of backend CELL_STATIONS (docs/CELL_FRAMES.md). */
export const CELL_STATIONS: Record<
  StationId,
  { id: StationId; ecAxes: 1 | 2; defaultBc: number; mountXyzM: Vec3 }
> = {
  ST1: { id: "ST1", ecAxes: 1, defaultBc: 927851, mountXyzM: [-0.1499, 1.7606, 1.0101] },
  ST2: { id: "ST2", ecAxes: 1, defaultBc: 250331, mountXyzM: [0.0508, 1.6563, 0.6338] },
  ST3: { id: "ST3", ecAxes: 2, defaultBc: 808680, mountXyzM: [0.3633, -1.5604, 0.4048] },
  ST4: { id: "ST4", ecAxes: 2, defaultBc: 190240, mountXyzM: [0.6996, -1.273, 0.9618] },
};

export function degToJointPulses(qDeg: number[]): number[] {
  if (qDeg.length !== 6) throw new Error("Expected 6 joint angles");
  return qDeg.map((q, i) => Math.round(q * CELL_PULSE.robotPulsesPerDeg[i]));
}

/** Rail mm → BC# (origin = BC_HOME). Scale is UNVERIFIED until lead verified. */
export function baseMmToBc(
  xMm: number,
  originBc: number = CELL_PULSE.bcHomePulse,
  pulsesPerMm: number = CELL_PULSE.rectXPulsesPerMm,
): number {
  if (pulsesPerMm === 0) throw new Error("RECT-X pulses/mm must be non-zero");
  return Math.round(originBc + xMm * pulsesPerMm);
}

/** BC# → rail mm relative to BC_HOME. */
export function bcToBaseMm(
  bcPulse: number,
  originBc: number = CELL_PULSE.bcHomePulse,
  pulsesPerMm: number = CELL_PULSE.rectXPulsesPerMm,
): number {
  if (pulsesPerMm === 0) throw new Error("RECT-X pulses/mm must be non-zero");
  return (bcPulse - originBc) / pulsesPerMm;
}

export class PulseExportError extends Error {
  constructor(message: string) {
    super(message);
    this.name = "PulseExportError";
  }
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

function jointsReady(joints: number[][] | undefined, npos: number): boolean {
  if (!Array.isArray(joints) || joints.length !== npos) return false;
  return joints.every((q) => Array.isArray(q) && q.length === 6 && q.every((v) => Number.isFinite(v)));
}

/**
 * Client-side DX200 .JBI generator aligned with the production MA2010 cell
 * (`reference/robot-cell-dx200/jobs/000.JBI`): GROUP1 RB1,BS1 + GROUP2 ST1,
 * ARCON/WVON, coordinated +MOVJ EC#.
 *
 * Production PULSE requires `jointAnglesDeg` from IK for every waypoint.
 * Without IK, pass `allowPreviewRectan: true` for a diagnostic
 * `PREVIEW_RECTAN_DO_NOT_LOAD` file — never invents C# pulse integers.
 */
export function generateJbi(
  prog: WeldProgram,
  opts?: {
    userFrame?: number;
    station?: StationId;
    stationPulsesPerDeg?: number;
    stationDeg?: number | number[];
    stationDeg2?: number | number[];
    weaveNo?: number;
    jointAnglesDeg?: number[][];
    /** @deprecated ignored — BC comes from station zone / baseMm */
    baseParkPulse?: number;
    baseMm?: number | number[];
    allowPreviewRectan?: boolean;
    homeBookends?: boolean;
  },
): string {
  const station: StationId = opts?.station ?? "ST1";
  const stMeta = CELL_STATIONS[station];
  const homeBookends = opts?.homeBookends !== false;
  let pts = [...prog.waypoints];
  let joints = opts?.jointAnglesDeg ? [...opts.jointAnglesDeg] : undefined;

  const homePos: Vec3 = CELL_PULSE.cellHomeTcpMm.map((v) => v / 1000) as Vec3;
  const homeWp = (): Waypoint => ({
    id: "HOME",
    pos: homePos,
    move: "MOVJ",
    speedLabel: "VJ=80",
    arc: false,
    kind: "travel",
  });
  if (homeBookends) {
    const firstHome = pts[0]?.kind === "travel" && dist(pts[0].pos, homePos) < 1e-3;
    const lastHome =
      pts[pts.length - 1]?.kind === "travel" &&
      dist(pts[pts.length - 1].pos, homePos) < 1e-3;
    if (!firstHome) {
      pts = [homeWp(), ...pts];
      if (joints) joints = [[...CELL_PULSE.cellHomePulses].map((p, i) => p / CELL_PULSE.robotPulsesPerDeg[i]), ...joints];
    }
    if (!lastHome) {
      pts = [...pts, homeWp()];
      if (joints) {
        joints = [
          ...joints,
          [...CELL_PULSE.cellHomePulses].map((p, i) => p / CELL_PULSE.robotPulsesPerDeg[i]),
        ];
      }
    }
  }

  const npos = pts.length;
  const pulsesPerDeg = opts?.stationPulsesPerDeg ?? CELL_PULSE.stationPulsesPerDeg;
  const weaveNo = opts?.weaveNo ?? 21;
  const usePulse = jointsReady(joints, npos);
  if (!usePulse && !opts?.allowPreviewRectan) {
    throw new PulseExportError(
      "Production POSTYPE PULSE requires IK jointAnglesDeg for every point — no fake C# pulses.",
    );
  }
  const stationDeg = pts.map((_, i) => {
    if (Array.isArray(opts?.stationDeg)) return opts!.stationDeg![i] ?? opts!.stationDeg![0] ?? 0;
    return opts?.stationDeg ?? 0;
  });
  const stationDeg2 = pts.map((_, i) => {
    if (Array.isArray(opts?.stationDeg2)) return opts!.stationDeg2![i] ?? opts!.stationDeg2![0] ?? 0;
    return opts?.stationDeg2 ?? 0;
  });
  const lines: string[] = [];
  const sch = prog.schedule;
  const jobName = usePulse
    ? prog.name
    : prog.name.toUpperCase().includes("PREVIEW_RECTAN_DO_NOT_LOAD")
      ? prog.name
      : `PREVIEW_RECTAN_DO_NOT_LOAD_${prog.name}`;

  lines.push("/JOB");
  lines.push(`//NAME ${jobName}`);
  if (usePulse) {
    lines.push("' PULSE export — MA2010 cell (RC1G / RBCALIB scaling)");
    lines.push("' C# from IK → degToJointPulses (no invented pulses)");
  } else {
    lines.push("' PREVIEW_RECTAN_DO_NOT_LOAD — offline diagnostic only");
    lines.push("' NOT for DX200 load — IK missing or unreachable; no PULSE C#");
  }
  lines.push(`' Cell: MA2010 + RECT-X (BS1) + TURN (${station}); TOOL0 TCP Z≈415.7 mm`);
  lines.push(
    `' Station pulses/deg~${pulsesPerDeg.toFixed(4)}; BC_HOME=${CELL_PULSE.bcHomePulse}; zone_BC=${stMeta.defaultBc}; RECT_X_LEAD_VERIFIED=${CELL_PULSE.rectXLeadVerified}`,
  );
  if (!CELL_PULSE.rectXLeadVerified) {
    lines.push(
      `' WARN: RECT-X pulses/mm=${CELL_PULSE.rectXPulsesPerMm} UNVERIFIED_PLACEHOLDER (RC1G/RC2G lead TBD) — BC# mm mapping is APP_ESTIMATE only`,
    );
  }
  lines.push("//POS");
  lines.push(`///NPOS ${npos},${npos},${npos},0,0,0`);
  lines.push("///TOOL 0");

  if (usePulse) {
    lines.push("///POSTYPE PULSE");
    lines.push("///PULSE");
    joints!.forEach((q, i) => {
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
      const [rx, ry, rz] = torchDirToRpy(torchDirAt(prog, Math.min(i, prog.waypoints.length - 1)));
      lines.push(
        `C${String(i).padStart(5, "0")}=${f3(x)},${f3(y)},${f3(z)},${f4(rx)},${f4(ry)},${f4(rz)}`,
      );
    });
  }

  lines.push(
    `' BC# home=${CELL_PULSE.bcHomePulse}; work zone=${stMeta.defaultBc} (${station})`,
  );
  pts.forEach((p, i) => {
    const isBookend = homeBookends && (i === 0 || i === npos - 1);
    const isHomeTravel = p.kind === "travel" && dist(p.pos, homePos) < 1e-3;
    let bc: number;
    if (isBookend || isHomeTravel) {
      bc = CELL_PULSE.bcHomePulse;
    } else if (Array.isArray(opts?.baseMm) && opts!.baseMm![i] != null) {
      bc = baseMmToBc(opts!.baseMm![i]);
    } else if (typeof opts?.baseMm === "number") {
      bc = baseMmToBc(opts.baseMm);
    } else {
      bc = stMeta.defaultBc;
    }
    lines.push(`BC${String(i).padStart(5, "0")}=${bc}`);
  });

  if (!usePulse) {
    lines.push("///POSTYPE PULSE");
    lines.push("///PULSE");
  }
  stationDeg.forEach((ang, i) => {
    const e1 = Math.round(ang * pulsesPerDeg);
    const e2 = stMeta.ecAxes >= 2 ? Math.round(stationDeg2[i] * pulsesPerDeg) : 0;
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
  lines.push(`///GROUP2 ${station}`);
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
