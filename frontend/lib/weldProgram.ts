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
  /** Motoweld condition / MACRO1 ARGF for this ARCON (multi-seam schedule changes). */
  condition?: number;
  argf?: number;
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

export type StationId = "ST1" | "ST2" | "ST3" | "ST4";

/** One fillet / butt seam for multi-seam planning (mirrors backend plan_from_seams). */
export type SeamSpec = {
  start: Vec3;
  end: Vec3;
  n1?: Vec3;
  n2?: Vec3;
  bisector?: Vec3;
  /** Motoweld UI condition → MACRO1 ARGF via schedule table. */
  condition?: number;
  weldSpeed?: number;
};

export type SeamOrientation = { index: number; reversed: boolean };

export type WeldProgram = {
  name: string;
  mount: Vec3;
  /** GROUP2 station the part is clamped on (ST1..ST4). */
  station?: StationId;
  plates: { size: Vec3; center: Vec3 }[];
  seam: Vec3[];
  /** Source seams after order/direction optimisation (multi-seam jobs). */
  seams?: SeamSpec[];
  seamOrder?: SeamOrientation[];
  torchFrames: { pos: Vec3; dir: Vec3 }[];
  waypoints: Waypoint[];
  segDurations: number[];
  cycleSec: number;
  weldLenMm: number;
  schedule: WeldScheduleRow;
  /**
   * Outward fillet bisector ``normalize(n1+n2)`` when known (CAD or demo T-joint).
   * Approach/retract offset along +bisector; wire aims along −bisector + travel tilt.
   */
  filletBisector?: Vec3;
  filletNormals?: { n1: Vec3; n2: Vec3 };
  /** ArcNC-style planner metadata (demo / placeholder). */
  meta?: {
    collisionFree: boolean;
    touchSense: boolean;
    sequenceOptimized: boolean;
    singularitySafe: boolean;
  };
};

/** Soft cap for CAD → path (production ``02025_KOSZ_…`` is large; default 50). */
export const DEFAULT_MAX_SEAMS = 50;
export const DEFAULT_CLEARANCE_HEIGHT = 0.08;
export const DEFAULT_BISECTOR_CLEARANCE_DOT = 0.35;

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

/**
 * Station workspaces (M2) — mirrored from ``backend/app/robot_config.STATION_FRAMES``.
 *
 * ``center`` / ``bbox*`` are torch TCP in the MA2010 robot-base frame (metres),
 * mined by ``tools/compare_pulses.py --analyze-stations``. ``mount`` is the CAD
 * part origin for IK (same frame). ``railZ`` is cell-world carriage Z at
 * ``bcMean`` (positive track mm → negative Z).
 */
export type StationFrame = {
  bcMean: number;
  bcMin: number;
  bcMax: number;
  center: Vec3;
  bboxMin: Vec3;
  bboxMax: Vec3;
  railZ: number;
  mount: Vec3;
  approximate: boolean;
};

/** Taught BC envelope midpoints (same calibration as ``pulse_config``). */
const _BC_ST2_MID = (124171 + 375361) / 2;
const _BC_ST13_MID = (808680 + 927851) / 2;

const _ST1_CENTER: Vec3 = [-0.032, 1.3555, 0.5876];
const _ST1_BBOX_MIN: Vec3 = [-0.6833, 0.8571, 0.3307];
const _ST1_BBOX_MAX: Vec3 = [0.688, 1.9002, 0.9089];
const _ST1_MOUNT: Vec3 = [-0.032, 1.3255, 0.5876];

export const STATION_FRAMES: Record<StationId, StationFrame> = {
  ST1: {
    bcMean: 878037,
    bcMin: 878036,
    bcMax: 878037,
    center: _ST1_CENTER,
    bboxMin: _ST1_BBOX_MIN,
    bboxMax: _ST1_BBOX_MAX,
    railZ: -1.1948,
    mount: _ST1_MOUNT,
    approximate: false,
  },
  ST2: {
    bcMean: Math.round(_BC_ST2_MID),
    bcMin: Math.round(_BC_ST2_MID),
    bcMax: Math.round(_BC_ST2_MID),
    center: _ST1_CENTER,
    bboxMin: _ST1_BBOX_MIN,
    bboxMax: _ST1_BBOX_MAX,
    railZ: 1.2431,
    mount: _ST1_MOUNT,
    approximate: true,
  },
  ST3: {
    bcMean: Math.round(_BC_ST13_MID),
    bcMin: Math.round(_BC_ST13_MID),
    bcMax: Math.round(_BC_ST13_MID),
    center: _ST1_CENTER,
    bboxMin: _ST1_BBOX_MIN,
    bboxMax: _ST1_BBOX_MAX,
    railZ: -1.1569,
    mount: _ST1_MOUNT,
    approximate: true,
  },
  ST4: {
    bcMean: Math.round(_BC_ST2_MID),
    bcMin: Math.round(_BC_ST2_MID),
    bcMax: Math.round(_BC_ST2_MID),
    center: _ST1_CENTER,
    bboxMin: _ST1_BBOX_MIN,
    bboxMax: _ST1_BBOX_MAX,
    railZ: 1.2431,
    mount: _ST1_MOUNT,
    approximate: true,
  },
};

/** Mount point on ST1 — real weld-cloud origin (robot-base metres) for IK. */
export const DEMO_MOUNT: Vec3 = [...STATION_FRAMES.ST1.mount];

export function stationMount(station: StationId | string = "ST1"): Vec3 {
  const key = station.toUpperCase().replace(/\s+/g, "") as StationId;
  const frame = STATION_FRAMES[key];
  if (!frame) throw new Error(`Unknown station '${station}'. Use ST1..ST4.`);
  return [...frame.mount];
}

export function stationRailZ(station: StationId | string = "ST1"): number {
  const key = station.toUpperCase().replace(/\s+/g, "") as StationId;
  const frame = STATION_FRAMES[key];
  if (!frame) throw new Error(`Unknown station '${station}'. Use ST1..ST4.`);
  return frame.railZ;
}

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

/** Coarse cell obstacles (rail + positioner pedestals at mined station anchors). */
function cellObstacles(): Aabb[] {
  const rail: Aabb = { min: [-0.25, 0.0, -2.4], max: [0.25, 0.28, 2.4] };
  const pedestals: Aabb[] = (Object.keys(STATION_FRAMES) as StationId[]).map((id) => {
    const f = STATION_FRAMES[id];
    const xOff = id === "ST3" || id === "ST4" ? 0.55 : 0;
    const x = f.mount[0] + xOff;
    const z = f.railZ;
    return {
      min: [x - 0.12, 0.0, z - 0.12] as Vec3,
      max: [x + 0.12, 0.9, z + 0.12] as Vec3,
    };
  });
  return [rail, ...pedestals];
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

/** Build a T-fillet demo weld program mounted on the given positioner / station. */
export function buildDemoProgram(
  mount: Vec3 = DEMO_MOUNT,
  condition = 1,
  station: StationId = "ST1",
): WeldProgram {
  const sch =
    MOTOWELD_SCHEDULES.find((s) => s.condition === condition) ?? MOTOWELD_SCHEDULES[0];
  const resolvedMount = mount ?? stationMount(station);

  const platesLocal: { size: Vec3; center: Vec3 }[] = [
    { size: [0.5, 0.02, 0.34], center: [0, 0.01, 0] },
    { size: [0.5, 0.24, 0.02], center: [0, 0.13, -0.15] },
  ];
  const plates = platesLocal.map((p) => ({
    size: p.size,
    center: add(resolvedMount, p.center),
  }));

  const seamStart = add(resolvedMount, [-0.22, 0.03, -0.135]);
  const seamEnd = add(resolvedMount, [0.22, 0.03, -0.135]);

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

  // Demo T-fillet: horizontal plate normal +Y, vertical wall normal +Z →
  // bisector out of the joint; wire aims −bisector with ~12° push travel tilt.
  const filletNormals = { n1: [0, 1, 0] as Vec3, n2: [0, 0, 1] as Vec3 };
  const filletBisector = filletBisectorFromNormals(filletNormals.n1, filletNormals.n2);
  const travel: Vec3 = [
    seamEnd[0] - seamStart[0],
    seamEnd[1] - seamStart[1],
    seamEnd[2] - seamStart[2],
  ];
  const torchDir = torchWireDirection(filletBisector, travel, 12, true);
  const torchFrames = seam.map((pos) => ({ pos, dir: torchDir }));

  const approachDist = 0.14;
  const approach = offsetAlongBisector(seamStart, filletBisector, approachDist);
  const sense1 = offsetAlongBisector(seamStart, filletBisector, 0.05);
  const sense2 = add(offsetAlongBisector(seamStart, filletBisector, 0.04), [
    0.03 * Math.sign(travel[0] || 1),
    0,
    0,
  ]);
  const retract = offsetAlongBisector(seamEnd, filletBisector, approachDist);

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
    mount: resolvedMount,
    station,
    plates,
    seam,
    seams: [
      {
        start: seamStart,
        end: seamEnd,
        n1: filletNormals.n1,
        n2: filletNormals.n2,
        bisector: filletBisector,
        condition: sch.condition,
      },
    ],
    seamOrder: [{ index: 0, reversed: false }],
    torchFrames,
    waypoints,
    segDurations: [],
    cycleSec: 0,
    weldLenMm: dist(seamStart, seamEnd) * 1000,
    schedule: sch,
    filletBisector,
    filletNormals,
  };
  return refreshMeta(rebuildDurations(draft));
}

/** Insert TouchSense search points before the first weld if missing. */
export function ensureTouchSense(prog: WeldProgram): WeldProgram {
  if (hasTouchSense(prog)) return refreshMeta(prog);

  const firstWeld = prog.waypoints.findIndex((w) => w.kind === "weld");
  if (firstWeld < 0) return refreshMeta(prog);

  const seamStart = prog.seam[0] ?? prog.waypoints[firstWeld].pos;
  const bis = prog.filletBisector ?? ([0, 1, 0] as Vec3);
  const sense1 = offsetAlongBisector(seamStart, bis, 0.05);
  const sense2 = add(offsetAlongBisector(seamStart, bis, 0.04), [0.03, 0, 0]);
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

function seamBisectorOf(seam: SeamSpec): Vec3 {
  if (seam.bisector) return norm(seam.bisector);
  if (seam.n1 && seam.n2) return filletBisectorFromNormals(seam.n1, seam.n2);
  return [0, 0, 1];
}

function seamEndpoints(seam: SeamSpec, reversed: boolean): { start: Vec3; end: Vec3 } {
  return reversed
    ? { start: [...seam.end], end: [...seam.start] }
    : { start: [...seam.start], end: [...seam.end] };
}

function approachOffset(seam: SeamSpec, height: number): Vec3 {
  const b = seamBisectorOf(seam);
  const o: Vec3 = [b[0] * height, b[1] * height, b[2] * height];
  if (Math.hypot(...o) < 1e-9) return [0, 0, height];
  return o;
}

function seamApproach(seam: SeamSpec, reversed: boolean, height: number): Vec3 {
  const { start } = seamEndpoints(seam, reversed);
  const o = approachOffset(seam, height);
  return add(start, o);
}

function seamRetract(seam: SeamSpec, reversed: boolean, height: number): Vec3 {
  const { end } = seamEndpoints(seam, reversed);
  const o = approachOffset(seam, height);
  return add(end, o);
}

function airTransition(
  a: SeamSpec,
  aRev: boolean,
  b: SeamSpec,
  bRev: boolean,
  height: number,
): number {
  return dist(seamRetract(a, aRev, height), seamApproach(b, bRev, height));
}

export function totalAirLength(
  seams: SeamSpec[],
  order: SeamOrientation[],
  approachHeight = 0.05,
  startPoint?: Vec3,
): number {
  if (!order.length) return 0;
  let total = 0;
  const first = order[0];
  const firstAp = seamApproach(seams[first.index], first.reversed, approachHeight);
  if (startPoint) total += dist(startPoint, firstAp);
  for (let k = 0; k < order.length - 1; k++) {
    const a = order[k];
    const b = order[k + 1];
    total += airTransition(
      seams[a.index],
      a.reversed,
      seams[b.index],
      b.reversed,
      approachHeight,
    );
  }
  return total;
}

export function greedyNearestNeighborOrder(
  seams: SeamSpec[],
  approachHeight = 0.05,
  startPoint: Vec3 = [0, 0, 0],
): SeamOrientation[] {
  const remaining = new Set(seams.map((_, i) => i));
  const order: SeamOrientation[] = [];
  let bestI = -1;
  let bestRev = false;
  let bestD = Infinity;
  for (const i of Array.from(remaining)) {
    for (const rev of [false, true]) {
      const d = dist(startPoint, seamApproach(seams[i], rev, approachHeight));
      if (d < bestD) {
        bestD = d;
        bestI = i;
        bestRev = rev;
      }
    }
  }
  if (bestI < 0) return [];
  order.push({ index: bestI, reversed: bestRev });
  remaining.delete(bestI);
  while (remaining.size) {
    const prev = order[order.length - 1];
    let nextI = -1;
    let nextRev = false;
    let nextD = Infinity;
    for (const i of Array.from(remaining)) {
      for (const rev of [false, true]) {
        const d = airTransition(
          seams[prev.index],
          prev.reversed,
          seams[i],
          rev,
          approachHeight,
        );
        if (d < nextD) {
          nextD = d;
          nextI = i;
          nextRev = rev;
        }
      }
    }
    if (nextI < 0) break;
    order.push({ index: nextI, reversed: nextRev });
    remaining.delete(nextI);
  }
  return order;
}

function twoOptReverse(order: SeamOrientation[], i: number, j: number): SeamOrientation[] {
  const mid = order
    .slice(i, j + 1)
    .reverse()
    .map((o) => ({ index: o.index, reversed: !o.reversed }));
  return [...order.slice(0, i), ...mid, ...order.slice(j + 1)];
}

export function twoOptImprove(
  seams: SeamSpec[],
  order: SeamOrientation[],
  approachHeight = 0.05,
  startPoint?: Vec3,
  maxPasses = 32,
): SeamOrientation[] {
  if (order.length < 2) return [...order];
  let current = [...order];
  let bestLen = totalAirLength(seams, current, approachHeight, startPoint);
  const n = current.length;
  for (let pass = 0; pass < maxPasses; pass++) {
    let improved = false;
    for (let k = 0; k < n; k++) {
      const trial = current.map((o, idx) =>
        idx === k ? { index: o.index, reversed: !o.reversed } : o,
      );
      const L = totalAirLength(seams, trial, approachHeight, startPoint);
      if (L + 1e-12 < bestLen) {
        current = trial;
        bestLen = L;
        improved = true;
        break;
      }
    }
    if (improved) continue;
    outer: for (let i = 0; i < n - 1; i++) {
      for (let j = i + 1; j < n; j++) {
        const trial = twoOptReverse(current, i, j);
        const L = totalAirLength(seams, trial, approachHeight, startPoint);
        if (L + 1e-12 < bestLen) {
          current = trial;
          bestLen = L;
          improved = true;
          break outer;
        }
      }
    }
    if (!improved) break;
  }
  return current;
}

/** Greedy NN + 2-opt over seam order and start↔end direction. */
export function optimizeSeamSequence(
  seams: SeamSpec[],
  opts?: { approachHeight?: number; startPoint?: Vec3 },
): SeamOrientation[] {
  if (seams.length <= 1) return seams.length ? [{ index: 0, reversed: false }] : [];
  const height = opts?.approachHeight ?? 0.05;
  const seed = greedyNearestNeighborOrder(seams, height, opts?.startPoint ?? [0, 0, 0]);
  return twoOptImprove(seams, seed, height, opts?.startPoint);
}

function needsClearance(
  a: SeamSpec,
  b: SeamSpec,
  dotThreshold = DEFAULT_BISECTOR_CLEARANCE_DOT,
): boolean {
  const b0 = seamBisectorOf(a);
  const b1 = seamBisectorOf(b);
  return b0[0] * b1[0] + b0[1] * b1[1] + b0[2] * b1[2] < dotThreshold;
}

function clearancePoint(
  from: SeamSpec,
  fromRev: boolean,
  to: SeamSpec,
  toRev: boolean,
  approachHeight: number,
  clearanceHeight: number,
): Vec3 {
  const retract = seamRetract(from, fromRev, approachHeight);
  const approach = seamApproach(to, toRev, approachHeight);
  const mid: Vec3 = [
    (retract[0] + approach[0]) / 2,
    (retract[1] + approach[1]) / 2,
    (retract[2] + approach[2]) / 2,
  ];
  let lift = norm([
    seamBisectorOf(from)[0] + seamBisectorOf(to)[0],
    seamBisectorOf(from)[1] + seamBisectorOf(to)[1],
    seamBisectorOf(from)[2] + seamBisectorOf(to)[2] + 0.35,
  ]);
  if (Math.hypot(...lift) < 1e-9) lift = [0, 0, 1];
  const pos = add(mid, [lift[0] * clearanceHeight, lift[1] * clearanceHeight, lift[2] * clearanceHeight]);
  const zFloor = Math.max(retract[2], approach[2]) + 0.5 * clearanceHeight;
  if (pos[2] < zFloor) return [pos[0], pos[1], zFloor];
  return pos;
}

function scheduleForCondition(condition: number): WeldScheduleRow {
  return MOTOWELD_SCHEDULES.find((s) => s.condition === condition) ?? MOTOWELD_SCHEDULES[0];
}

/**
 * Build a multi-seam weld program: NN+2-opt order/direction, clearance air
 * moves between seams, MACRO1 ARGF metadata on each ARCON.
 */
export function buildProgramFromSeams(
  seamsIn: SeamSpec[],
  opts?: {
    mount?: Vec3;
    station?: StationId;
    condition?: number;
    samplesPerSeam?: number;
    approachHeight?: number;
    clearanceHeight?: number;
    maxSeams?: number;
    optimize?: boolean;
    name?: string;
  },
): WeldProgram {
  const maxSeams = opts?.maxSeams ?? DEFAULT_MAX_SEAMS;
  const seams = seamsIn.slice(0, Math.max(1, maxSeams));
  const mount = opts?.mount ?? DEMO_MOUNT;
  const station = opts?.station ?? "ST1";
  const approachHeight = opts?.approachHeight ?? 0.14;
  const clearanceHeight = opts?.clearanceHeight ?? DEFAULT_CLEARANCE_HEIGHT;
  const samples = Math.max(2, opts?.samplesPerSeam ?? 8);
  const defaultCond = opts?.condition ?? 1;
  const defaultSch = scheduleForCondition(defaultCond);
  const optimize = opts?.optimize ?? true;

  const order = optimize
    ? optimizeSeamSequence(seams, { approachHeight, startPoint: mount })
    : seams.map((_, i) => ({ index: i, reversed: false }));

  const waypoints: Waypoint[] = [];
  const torchFrames: { pos: Vec3; dir: Vec3 }[] = [];
  const allSeamPts: Vec3[] = [];
  let weldLen = 0;
  const airLabel = "VJ=50%";

  for (let k = 0; k < order.length; k++) {
    const { index, reversed } = order[k];
    const seam = seams[index];
    const { start, end } = seamEndpoints(seam, reversed);
    const bis = seamBisectorOf(seam);
    const travel: Vec3 = [end[0] - start[0], end[1] - start[1], end[2] - start[2]];
    const torchDir = torchWireDirection(bis, travel, 12, true);
    const cond = seam.condition ?? defaultCond;
    const sch = scheduleForCondition(cond);
    const weldLabel = `V=${seam.weldSpeed ?? sch.travelCmMin} cm/min`;

    if (k > 0) {
      const prev = order[k - 1];
      let extra = clearanceHeight;
      if (needsClearance(seams[prev.index], seam)) {
        extra = Math.max(extra, clearanceHeight * 1.5);
      }
      const clr = clearancePoint(
        seams[prev.index],
        prev.reversed,
        seam,
        reversed,
        approachHeight,
        extra,
      );
      waypoints.push({
        id: pad(waypoints.length + 1),
        pos: clr,
        move: "MOVJ",
        speedLabel: airLabel,
        arc: false,
        kind: "travel",
      });
    }

    const approach = add(start, approachOffset(seam, approachHeight));
    const retract = add(end, approachOffset(seam, approachHeight));
    waypoints.push({
      id: pad(waypoints.length + 1),
      pos: approach,
      move: "MOVJ",
      speedLabel: airLabel,
      arc: false,
      kind: "approach",
    });

    const seamPts: Vec3[] = [];
    for (let s = 0; s <= samples; s++) {
      const t = s / samples;
      seamPts.push([
        start[0] + (end[0] - start[0]) * t,
        start[1] + (end[1] - start[1]) * t,
        start[2] + (end[2] - start[2]) * t,
      ]);
    }
    allSeamPts.push(...seamPts);
    weldLen += dist(start, end) * 1000;
    for (const pos of seamPts) torchFrames.push({ pos, dir: torchDir });

    waypoints.push({
      id: pad(waypoints.length + 1),
      pos: seamPts[0],
      move: "MOVL",
      speedLabel: weldLabel,
      arc: true,
      kind: "weld",
      tag: "ARCON",
      condition: sch.condition,
      argf: sch.argf,
    });
    for (let s = 1; s < seamPts.length - 1; s++) {
      waypoints.push({
        id: pad(waypoints.length + 1),
        pos: seamPts[s],
        move: "MOVL",
        speedLabel: weldLabel,
        arc: true,
        kind: "weld",
      });
    }
    waypoints.push({
      id: pad(waypoints.length + 1),
      pos: seamPts[seamPts.length - 1],
      move: "MOVL",
      speedLabel: weldLabel,
      arc: true,
      kind: "weld",
      tag: "ARCOF",
    });
    waypoints.push({
      id: pad(waypoints.length + 1),
      pos: retract,
      move: "MOVJ",
      speedLabel: airLabel,
      arc: false,
      kind: "retract",
    });
  }

  const firstBis = seams.length ? seamBisectorOf(seams[order[0]?.index ?? 0]) : undefined;
  const draft: WeldProgram = {
    name: opts?.name ?? "MULTI_SEAM",
    mount,
    station,
    plates: [],
    seam: allSeamPts,
    seams,
    seamOrder: order,
    torchFrames,
    waypoints,
    segDurations: [],
    cycleSec: 0,
    weldLenMm: weldLen,
    schedule: defaultSch,
    filletBisector: firstBis,
  };
  return withMeta(rebuildDurations(draft), { sequenceOptimized: optimize });
}

/**
 * Optimise multi-seam order/direction (NN + 2-opt). Single-seam programs still
 * flip start↔end when that shortens the first approach.
 */
export function optimizeSequence(prog: WeldProgram): WeldProgram {
  if (prog.seams && prog.seams.length >= 2) {
    return refreshMeta(
      buildProgramFromSeams(prog.seams, {
        mount: prog.mount,
        station: prog.station,
        condition: prog.schedule.condition,
        optimize: true,
        name: prog.name,
      }),
    );
  }

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

/**
 * Re-apply fillet bisector approach/retract + torch frames, then refresh planner flags.
 * When ``filletBisector`` is missing, keeps existing poses (legacy demo).
 */
export function planMotions(prog: WeldProgram): WeldProgram {
  let next = prog;
  if (prog.filletBisector && prog.seam.length >= 2) {
    const bis = prog.filletBisector;
    const travel: Vec3 = [
      prog.seam[prog.seam.length - 1][0] - prog.seam[0][0],
      prog.seam[prog.seam.length - 1][1] - prog.seam[0][1],
      prog.seam[prog.seam.length - 1][2] - prog.seam[0][2],
    ];
    const dir = torchWireDirection(bis, travel, 12, true);
    const torchFrames = prog.seam.map((pos) => ({ pos, dir }));
    const waypoints = prog.waypoints.map((w) => {
      if (w.kind === "approach") {
        return { ...w, pos: offsetAlongBisector(prog.seam[0], bis, 0.14) };
      }
      if (w.kind === "retract") {
        return {
          ...w,
          pos: offsetAlongBisector(prog.seam[prog.seam.length - 1], bis, 0.14),
        };
      }
      return w;
    });
    next = { ...prog, torchFrames, waypoints };
  }
  const withSense = ensureTouchSense(next);
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
        segLocal: local,
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
    segLocal: 0,
    arcOn: false,
    elapsedSec: prog.cycleSec,
  };
}

const AXIS_ORDER = ["S", "L", "U", "R", "B", "T"] as const;

/** Convert [S,L,U,R,B,T] degrees → jog slider map. */
export function jointArrayToMap(q: number[]): Record<(typeof AXIS_ORDER)[number], number> {
  const out = { S: 0, L: 0, U: 0, R: 0, B: 0, T: 0 };
  AXIS_ORDER.forEach((axis, i) => {
    out[axis] = Number(q[i] ?? 0);
  });
  return out;
}

/**
 * Linear joint-space interpolation along the same time base as `sampleProgram`.
 * `jointAnglesDeg[i]` is [S,L,U,R,B,T] for waypoint i (from `/api/calculate-ik`).
 */
export function sampleJointPath(
  jointAnglesDeg: number[][],
  prog: WeldProgram,
  p: number,
): { q: number[]; wpIndex: number; segLocal: number } {
  const n = prog.waypoints.length;
  if (!jointAnglesDeg.length || n === 0) {
    return { q: [0, 0, 0, 0, 0, 0], wpIndex: 0, segLocal: 0 };
  }
  if (jointAnglesDeg.length !== n) {
    const i = Math.min(jointAnglesDeg.length - 1, Math.max(0, sampleProgram(prog, p).wpIndex));
    return { q: [...jointAnglesDeg[i]], wpIndex: i, segLocal: 0 };
  }
  const sample = sampleProgram(prog, p);
  const i = sample.wpIndex;
  const local = sample.segLocal;
  const a = jointAnglesDeg[i];
  const b = jointAnglesDeg[Math.min(i + 1, n - 1)];
  const q = a.map((va, k) => va + (b[k] - va) * local);
  return { q, wpIndex: i, segLocal: local };
}

export type JbiPreviewSummary = {
  npos: string | null;
  pulse: boolean;
  rectan: boolean;
  group2: string | null;
  macroArgs: number[];
  weaveNos: number[];
  bcMin: number | null;
  bcMax: number | null;
  ecSamples: string[];
  seamCount: number;
  lineCount: number;
  homeBookend: boolean;
};

/** Parse generated INFORM text for the preview header strip. */
export function summarizeJbi(text: string): JbiPreviewSummary {
  const lines = text.replace(/\r\n/g, "\n").split("\n");
  const npos = lines.find((l) => l.startsWith("///NPOS "))?.replace("///NPOS ", "") ?? null;
  const pulse = lines.some((l) => l.includes("///POSTYPE PULSE"));
  const rectan = lines.some((l) => l.includes("///RECTAN"));
  const group2 =
    lines.find((l) => l.startsWith("///GROUP2 "))?.replace("///GROUP2 ", "") ?? null;
  const macroArgs = Array.from(
    new Set(
      Array.from(text.matchAll(/MACRO1\s+MJ#\(0\)\s+ARGF(\d+)/g)).map((m) =>
        Number(m[1]),
      ),
    ),
  ).sort((a, b) => a - b);
  const weaveNos = Array.from(
    new Set(
      Array.from(text.matchAll(/WVON\s+WEV#\((\d+)\)/g)).map((m) => Number(m[1])),
    ),
  ).sort((a, b) => a - b);
  const bcs = Array.from(text.matchAll(/^BC\d+=(-?\d+)/gm)).map((m) => Number(m[1]));
  const ecs = Array.from(text.matchAll(/^EC\d+=(-?\d+,-?\d+)/gm)).map((m) => m[1]);
  const seamCount = (text.match(/\bARCON\b/g) ?? []).length;
  const homeBookend =
    text.includes(`BC00000=${BC_HOME_PULSE}`) &&
    /C00000=-?\d+,-?\d+,-?\d+,-?\d+,-?\d+,-?\d+/.test(text);
  return {
    npos,
    pulse,
    rectan,
    group2,
    macroArgs,
    weaveNos,
    bcMin: bcs.length ? Math.min(...bcs) : null,
    bcMax: bcs.length ? Math.max(...bcs) : null,
    ecSamples: ecs.slice(0, 3),
    seamCount,
    lineCount: lines.filter((l) => l.length > 0).length,
    homeBookend,
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
  /** Nominal BC per station (matches ``STATION_FRAMES`` / ``pulse_config``). */
  stationBcMid: {
    ST1: STATION_FRAMES.ST1.bcMean,
    ST2: STATION_FRAMES.ST2.bcMean,
    ST3: STATION_FRAMES.ST3.bcMean,
    ST4: STATION_FRAMES.ST4.bcMean,
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

const cross = (a: Vec3, b: Vec3): Vec3 => [
  a[1] * b[2] - a[2] * b[1],
  a[2] * b[0] - a[0] * b[2],
  a[0] * b[1] - a[1] * b[0],
];

/** Rodrigues rotate ``v`` about unit ``axis`` by ``angleDeg``. */
function rotateAbout(v: Vec3, axis: Vec3, angleDeg: number): Vec3 {
  const a = norm(axis);
  const rad = (angleDeg * Math.PI) / 180;
  const c = Math.cos(rad);
  const s = Math.sin(rad);
  const dot = a[0] * v[0] + a[1] * v[1] + a[2] * v[2];
  const cr = cross(a, v);
  return [
    v[0] * c + cr[0] * s + a[0] * dot * (1 - c),
    v[1] * c + cr[1] * s + a[1] * dot * (1 - c),
    v[2] * c + cr[2] * s + a[2] * dot * (1 - c),
  ];
}

/**
 * Fillet wire direction: into the joint along ``−bisector``, then tilted by a
 * travel (push/pull) angle. Mirrors ``backend/app/path_planning.py``.
 */
export function torchWireDirection(
  bisector: Vec3,
  travel: Vec3,
  travelAngleDeg = 12,
  push = true,
): Vec3 {
  let wire = norm([-bisector[0], -bisector[1], -bisector[2]]);
  const trav = norm(travel);
  if (Math.abs(travelAngleDeg) < 1e-9 || Math.hypot(...trav) < 1e-9) return wire;
  let axis = cross(wire, trav);
  if (Math.hypot(...axis) < 1e-9) {
    const helper: Vec3 = Math.abs(wire[2]) < 0.9 ? [0, 0, 1] : [1, 0, 0];
    axis = cross(wire, helper);
  }
  const signed = push ? travelAngleDeg : -travelAngleDeg;
  return norm(rotateAbout(wire, axis, signed));
}

/** Outward fillet bisector from face normals (or a provided bisector). */
export function filletBisectorFromNormals(n1: Vec3, n2: Vec3): Vec3 {
  return norm([n1[0] + n2[0], n1[1] + n2[1], n1[2] + n2[2]]);
}

/** Approach / retract standoff along +bisector (out of the joint). */
export function offsetAlongBisector(
  point: Vec3,
  bisector: Vec3,
  distance: number,
): Vec3 {
  const b = norm(bisector);
  return add(point, [b[0] * distance, b[1] * distance, b[2] * distance]);
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
  // MACRO1 at first ARCON, then only when ARGF / schedule changes between seams.
  let lastMacroArg: number | null = null;
  rows.forEach((row, i) => {
    const c = `C${String(i).padStart(5, "0")}`;
    const bc = `BC${String(i).padStart(5, "0")}`;
    const ec = `EC${String(i).padStart(5, "0")}`;

    if (row.kind === "home") {
      lines.push(`MOVJ ${c} ${bc} VJ=80.00 DEC=20  +MOVJ ${ec} VJ=80.00`);
      return;
    }

    const p = pathPts[row.pathIndex];
    const wpSch =
      p.condition != null ? scheduleForCondition(p.condition) : sch;
    const wpArgf = p.argf ?? opts?.macroArg ?? wpSch.argf;
    const parsed = parseSpeed(
      p.speedLabel,
      p.kind === "weld" ? wpSch.travelCmMin : 80,
    );
    const useJoint =
      p.move === "MOVJ" || p.kind === "approach" || p.kind === "retract" || p.kind === "travel";

    if (p.tag === "ARCON") {
      lines.push("TIMER T=0.50");
      if (lastMacroArg == null || wpArgf !== lastMacroArg) {
        lines.push(`MACRO1 MJ#(0) ARGF${wpArgf}`);
        lines.push(
          `' MOTOWELD-E cond ${wpSch.condition} / MACRO1 ARGF${wpArgf}: ${wpSch.currentA}A / ${wpSch.voltageV.toFixed(1)}V / wire ${wpSch.wireMmin.toFixed(
            1,
          )} m/min — UI estimate; verify ARCSRT.CND`,
        );
        lastMacroArg = wpArgf;
      }
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
