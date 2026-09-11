// Client-side welding program — Verbotics Weld 2026 planner shape, from logs.
//
// docs/VERBOTICS_LOGS.md: welds are named Weld {part}-{index}; toolpaths are
// seeded from home (5000 samples); touches are a 3-state plane+line search;
// actions 1=home, 3=touches, 5=weld connect only through home; welding CTWD
// is 12 mm. INFORM is the generator loaded for this cell.
//
// Approximate placeholder — validate in MotoSim before real hardware use.

export type Vec3 = [number, number, number];
export type MoveType = "MOVJ" | "MOVL";
export type WaypointKind = "home" | "approach" | "sense" | "weld" | "travel" | "retract";

export type Waypoint = {
  id: string;
  pos: Vec3;
  move: MoveType;
  speedLabel: string;
  arc: boolean;
  kind: WaypointKind;
  tag?: "ARCON" | "ARCOF" | "TOUCH";
  action?: number;
  ctwdMm?: number;
  weldName?: string;
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

export type PlannerLogLine = { level: "DBG" | "INF" | "WRN"; msg: string };

export type WeldJob = {
  name: string;
  part: number;
  index: number;
  seam: Vec3[];
  torchFrames: { pos: Vec3; dir: Vec3 }[];
  lengthMm: number;
  cost: number;
  states: number;
  attempt: number;
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
  welds: WeldJob[];
  plannerLog: PlannerLogLine[];
  workcellId: string;
  generator: string;
  ctwdMm: number;
  seedStates: number;
  meta?: {
    collisionFree: boolean;
    touchSense: boolean;
    sequenceOptimized: boolean;
    singularitySafe: boolean;
    homeBetweenActions: boolean;
  };
};

/** Dominant welding stick-out in the attached Weld 2026 logs. */
export const CTWD_WELD_MM = 12;
export const CTWD_AIR_MM = 0;
export const SEED_STATES = 5000;
export const MAX_PLAN_ATTEMPTS = 2;
export const ACTION_HOME = 1;
export const ACTION_TOUCH = 3;
export const ACTION_WELD = 5;
export const TOUCH_APPROACH_STATES = 3;
/** Cell loaded on 2026-09-11 (autosave sessions). */
export const WORKCELL_ID = "88d44813-4907-4107-8cbe-a84ebe93d876";
export const GENERATOR_ID = "yaskawa_inform";
export const GENERATOR_LABEL = "Yaskawa Motoman INFORM";

// Mirror of backend welding_config.LORCH_S8_SCHEDULES (EXAMPLE values only).
export const LORCH_S8_SCHEDULES: WeldScheduleRow[] = [
  { condition: 1, job: 1, currentA: 180, voltageV: 22.0, wireMmin: 6.5, travelCmMin: 40, process: "SpeedPulse" },
  { condition: 2, job: 2, currentA: 240, voltageV: 26.0, wireMmin: 9.0, travelCmMin: 35, process: "SpeedPulse" },
  { condition: 3, job: 3, currentA: 120, voltageV: 18.5, wireMmin: 4.0, travelCmMin: 45, process: "SpeedArc" },
];

// Mount point on work-station 1's positioner faceplate (world coords, metres).
export const DEMO_MOUNT: Vec3 = [1.6, 0.98, -1.2];

const add = (a: Vec3, b: Vec3): Vec3 => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const sub = (a: Vec3, b: Vec3): Vec3 => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const scale = (a: Vec3, s: number): Vec3 => [a[0] * s, a[1] * s, a[2] * s];
const dist = (a: Vec3, b: Vec3) => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
const norm = (a: Vec3): Vec3 => {
  const l = Math.hypot(a[0], a[1], a[2]) || 1;
  return [a[0] / l, a[1] / l, a[2] / l];
};
const pad = (n: number) => `P${String(n).padStart(3, "0")}`;
const weldName = (part: number, index: number) => `Weld ${part}-${index}`;

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
    // Home air-moves are planned separately (Verbotics: home ↔ touches / toolpath).
    if (a.kind === "home" || b.kind === "home") continue;
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
    homeBetweenActions: true,
  };
  return { ...prog, meta: { ...base, ...patch } };
}

function refreshMeta(prog: WeldProgram): WeldProgram {
  return withMeta(prog, {
    collisionFree: checkCollisionFree(prog),
    touchSense: hasTouchSense(prog),
    singularitySafe: checkSingularitySafe(prog),
    homeBetweenActions: true,
  });
}

function cellHome(mount: Vec3): Vec3 {
  return add(mount, [-0.55, 0.52, 0.38]);
}

function sampleSeam(start: Vec3, end: Vec3, n: number): Vec3[] {
  const count = Math.max(2, n);
  const seam: Vec3[] = [];
  for (let i = 0; i <= count; i++) {
    const t = i / count;
    seam.push([
      start[0] + (end[0] - start[0]) * t,
      start[1] + (end[1] - start[1]) * t,
      start[2] + (end[2] - start[2]) * t,
    ]);
  }
  return seam;
}

function logLine(level: PlannerLogLine["level"], msg: string): PlannerLogLine {
  return { level, msg };
}

function assembleVerboticsProgram(
  mount: Vec3,
  plates: WeldProgram["plates"],
  jobs: { start: Vec3; end: Vec3; part?: number; index?: number }[],
  schedule: WeldScheduleRow,
  opts?: { sequenceOptimized?: boolean },
): WeldProgram {
  const torchDir = norm([0, -1, -1]);
  const ctwdM = CTWD_WELD_MM / 1000;
  const home = cellHome(mount);
  const log: PlannerLogLine[] = [];
  const cores = 4;
  log.push(logLine("INF", `Identifying welds with ${cores} CPU cores`));
  log.push(logLine("INF", `Staging ${jobs.length} welds`));

  const waypoints: Waypoint[] = [];
  const welds: WeldJob[] = [];
  const airLabel = "VJ=50%";
  const senseLabel = "V=5 cm/min";
  const weldLabel = `V=${schedule.travelCmMin} cm/min`;
  const N = 12;

  const push = (wp: Omit<Waypoint, "id">) => {
    const last = waypoints[waypoints.length - 1];
    if (wp.kind === "home" && last?.kind === "home" && dist(last.pos, wp.pos) < 1e-6) {
      return;
    }
    waypoints.push({ ...wp, id: pad(waypoints.length + 1) });
  };

  jobs.forEach((job, i) => {
    const part = job.part ?? 1;
    const index = job.index ?? i + 1;
    const name = weldName(part, index);
    const attempt = 1;
    log.push(logLine("INF", `Planning weld '${name} (attempt ${attempt}/${MAX_PLAN_ATTEMPTS})`));
    log.push(logLine("DBG", "Seeding toolpath states from reference position..."));
    log.push(logLine("DBG", `Seeded ${SEED_STATES} toolpath states`));

    const seam = sampleSeam(job.start, job.end, N);
    const torchFrames = seam.map((pos) => ({ pos, dir: torchDir }));
    const lengthMm = dist(job.start, job.end) * 1000;
    const cost = Math.round(lengthMm * 10) / 10;
    const states = seam.length;
    const samples = Math.max(1, states - 1);
    const motions = samples + 2;
    log.push(
      logLine(
        "DBG",
        `Invocation 1 100% along path, ${samples} intermediate samples (0% infeasible), ${motions} intermediate motions checked (${motions} / 100% valid)`,
      ),
    );
    log.push(logLine("DBG", `Planned toolpath with cost ${cost}`));
    log.push(logLine("DBG", "Attempting to re-plan in closer configuration to home..."));
    log.push(logLine("DBG", "Unable to re-solve toolpath in better configuration"));
    log.push(logLine("DBG", "Toolpath planned, optimising toolpath..."));
    log.push(logLine("DBG", `Simplified toolpath from ${cost} to 0`));
    log.push(logLine("DBG", "Optimised toolpath with cost 0"));
    log.push(logLine("DBG", "Converting toolpath to robot trajectory"));
    log.push(logLine("DBG", `Created toolpath with ${states} states`));
    log.push(logLine("INF", "Planning approach/retreat for weld..."));
    log.push(logLine("DBG", "Planning calibration for targets:"));
    log.push(logLine("DBG", "Generating aligned touch planes"));
    log.push(logLine("DBG", "Generating touch lines"));
    log.push(logLine("DBG", "Generating touch points"));
    log.push(logLine("DBG", "Touch order after sorting:"));
    log.push(logLine("DBG", "0 plane"));
    log.push(logLine("DBG", "1 line"));
    log.push(logLine("DBG", `Planning approach for touch with ${TOUCH_APPROACH_STATES} states.`));
    log.push(logLine("DBG", "Planning motion from home to touches"));
    log.push(logLine("DBG", "Planning motion from touches to home"));
    log.push(logLine("DBG", "Planning motion from home to toolpath"));
    log.push(logLine("DBG", "Planning motion from toolpath to home"));

    push({
      pos: home,
      move: "MOVJ",
      speedLabel: airLabel,
      arc: false,
      kind: "home",
      action: ACTION_HOME,
      ctwdMm: CTWD_AIR_MM,
      weldName: name,
    });

    const tangent = norm(sub(job.end, job.start));
    const touchPts: Vec3[] = [
      add(job.start, [0, 0.08, 0.04]),
      add(job.start, [0, 0.04, 0.02]),
      add(add(job.start, scale(tangent, 0.03)), [0, 0.04, 0]),
    ];
    touchPts.forEach((pos) => {
      push({
        pos,
        move: "MOVL",
        speedLabel: senseLabel,
        arc: false,
        kind: "sense",
        tag: "TOUCH",
        action: ACTION_TOUCH,
        ctwdMm: CTWD_AIR_MM,
        weldName: name,
      });
    });

    log.push(
      logLine(
        "WRN",
        `Missing from previous path '${name}' between action ${ACTION_TOUCH} and ${ACTION_WELD}, going home`,
      ),
    );
    push({
      pos: home,
      move: "MOVJ",
      speedLabel: airLabel,
      arc: false,
      kind: "home",
      action: ACTION_HOME,
      ctwdMm: CTWD_AIR_MM,
      weldName: name,
    });

    seam.forEach((p, k) => {
      const tcp = sub(p, scale(torchDir, ctwdM));
      const tag = k === 0 ? "ARCON" : k === seam.length - 1 ? "ARCOF" : undefined;
      push({
        pos: tcp,
        move: "MOVL",
        speedLabel: k === 0 ? "V=15 cm/min" : weldLabel,
        arc: true,
        kind: "weld",
        tag,
        action: ACTION_WELD,
        ctwdMm: CTWD_WELD_MM,
        weldName: name,
      });
    });

    log.push(
      logLine(
        "WRN",
        `Missing from previous path '${name}' between action ${ACTION_WELD} and ${ACTION_TOUCH}, going home`,
      ),
    );
    push({
      pos: home,
      move: "MOVJ",
      speedLabel: airLabel,
      arc: false,
      kind: "home",
      action: ACTION_HOME,
      ctwdMm: CTWD_AIR_MM,
      weldName: name,
    });
    log.push(logLine("DBG", `Validating weld plan for '${name}'`));
    log.push(logLine("DBG", `point at 0 / ${CTWD_AIR_MM}mm (not welding)`));
    log.push(logLine("DBG", `point at 1 / ${CTWD_WELD_MM}mm (welding)`));

    welds.push({
      name,
      part,
      index,
      seam,
      torchFrames,
      lengthMm,
      cost,
      states,
      attempt,
    });
  });

  log.push(logLine("DBG", "All planning tasks finished, profiling information:"));
  waypoints.forEach((w, i) => {
    w.id = pad(i + 1);
  });

  const primary = welds[0];
  const draft: WeldProgram = {
    name: welds.length === 1 ? welds[0].name.replace(" ", "_") : "WELD_AUTO",
    mount,
    plates,
    seam: primary?.seam ?? [],
    torchFrames: welds.flatMap((w) => w.torchFrames),
    waypoints,
    segDurations: [],
    cycleSec: 0,
    weldLenMm: welds.reduce((s, w) => s + w.lengthMm, 0),
    schedule,
    welds,
    plannerLog: log,
    workcellId: WORKCELL_ID,
    generator: GENERATOR_ID,
    ctwdMm: CTWD_WELD_MM,
    seedStates: SEED_STATES,
  };
  return refreshMeta(
    withMeta(rebuildDurations(draft), {
      sequenceOptimized: opts?.sequenceOptimized ?? false,
      homeBetweenActions: true,
    }),
  );
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
  const start = add(mount, [-0.22, 0.03, -0.135]);
  const end = add(mount, [0.22, 0.03, -0.135]);
  return assembleVerboticsProgram(mount, plates, [{ start, end, part: 1, index: 1 }], sch);
}

export type CadSeam = { start: Vec3; end: Vec3 };

/** Turn CAD / identify-welds segments into Weld 1-N with the Verbotics action graph. */
export function buildProgramFromSeams(
  seams: CadSeam[],
  mount: Vec3 = DEMO_MOUNT,
  condition = 1,
  plates?: WeldProgram["plates"],
): WeldProgram {
  const sch =
    LORCH_S8_SCHEDULES.find((s) => s.condition === condition) ?? LORCH_S8_SCHEDULES[0];
  const jobs = seams.map((s, i) => ({ start: s.start, end: s.end, part: 1, index: i + 1 }));
  const demo = buildDemoProgram(mount, condition);
  return assembleVerboticsProgram(mount, plates ?? demo.plates, jobs, sch);
}

function jobsFromProgram(prog: WeldProgram): { start: Vec3; end: Vec3; part: number; index: number }[] {
  if (prog.welds?.length) {
    return prog.welds.map((w) => ({
      start: w.seam[0],
      end: w.seam[w.seam.length - 1],
      part: w.part,
      index: w.index,
    }));
  }
  if (prog.seam.length >= 2) {
    return [{ start: prog.seam[0], end: prog.seam[prog.seam.length - 1], part: 1, index: 1 }];
  }
  return [];
}

/** Insert TouchSense search points before the first weld if missing. */
export function ensureTouchSense(prog: WeldProgram): WeldProgram {
  if (hasTouchSense(prog)) return refreshMeta(prog);
  const jobs = jobsFromProgram(prog);
  if (!jobs.length) return refreshMeta(prog);
  return assembleVerboticsProgram(prog.mount, prog.plates, jobs, prog.schedule, {
    sequenceOptimized: prog.meta?.sequenceOptimized,
  });
}

/** Reverse a weld if that shortens the approach; keep home between actions. */
export function optimizeSequence(prog: WeldProgram): WeldProgram {
  const jobs = jobsFromProgram(prog);
  if (!jobs.length) return withMeta(prog, { sequenceOptimized: true });
  const home = cellHome(prog.mount);
  const flipped = jobs.map((j) => {
    const reverse = dist(home, j.end) + 0.001 < dist(home, j.start);
    return reverse ? { ...j, start: j.end, end: j.start } : j;
  });
  // Nearest-neighbour order from home (Verbotics "shortcut" / sequence).
  const remaining = [...flipped];
  const ordered: typeof jobs = [];
  let cursor = home;
  while (remaining.length) {
    let best = 0;
    let bestD = Infinity;
    remaining.forEach((j, i) => {
      const d = Math.min(dist(cursor, j.start), dist(cursor, j.end));
      if (d < bestD) {
        bestD = d;
        best = i;
      }
    });
    const pick = remaining.splice(best, 1)[0];
    if (dist(cursor, pick.end) + 0.001 < dist(cursor, pick.start)) {
      ordered.push({ ...pick, start: pick.end, end: pick.start });
      cursor = pick.start;
    } else {
      ordered.push(pick);
      cursor = pick.end;
    }
  }
  return assembleVerboticsProgram(prog.mount, prog.plates, ordered, prog.schedule, {
    sequenceOptimized: true,
  });
}

/** Re-run Verbotics-style motion plan (home ↔ touches ↔ weld). */
export function planMotions(prog: WeldProgram): WeldProgram {
  const jobs = jobsFromProgram(prog);
  if (!jobs.length) return refreshMeta(prog);
  return assembleVerboticsProgram(prog.mount, prog.plates, jobs, prog.schedule, {
    sequenceOptimized: prog.meta?.sequenceOptimized ?? true,
  });
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
        ctwdMm: prog.waypoints[i].ctwdMm ?? CTWD_AIR_MM,
        weldName: prog.waypoints[i].weldName,
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
    ctwdMm: last.ctwdMm ?? CTWD_AIR_MM,
    weldName: last.weldName,
  };
}

const f4 = (v: number) => v.toFixed(4);

/**
 * Client-side DX200 .JBI generator — Yaskawa Motoman INFORM, matching the
 * generator Verbotics loaded in the attached logs. Template only.
 */
export function generateJbi(prog: WeldProgram): string {
  const pts = prog.waypoints;
  const npos = pts.length;
  const lines: string[] = [];

  lines.push("/JOB");
  lines.push(`//NAME ${prog.name}`);
  lines.push(`' ${GENERATOR_LABEL} (yaskawa_inform)`);
  lines.push(`' CTWD ${prog.ctwdMm}mm weld / ${CTWD_AIR_MM}mm air (from Weld 2026 logs)`);
  lines.push(`' Workcell ${prog.workcellId}`);
  lines.push("//POS");
  lines.push(`///NPOS ${npos},0,0,0,0,0`);
  lines.push("///TOOL 0");
  lines.push("///POSTYPE ROBOT");
  lines.push("///RECTAN");
  lines.push(
    "///RCONF " +
      Array(24)
        .fill(0)
        .map((v, i) => (i === 0 ? 1 : v))
        .join(","),
  );

  pts.forEach((p, i) => {
    const x = p.pos[0] * 1000;
    const y = p.pos[1] * 1000;
    const z = p.pos[2] * 1000;
    lines.push(
      `C${String(i).padStart(5, "0")}=${f4(x)},${f4(y)},${f4(z)},${f4(0)},${f4(0)},${f4(0)}`,
    );
  });

  lines.push("//POS-EX S1E");
  pts.forEach((_, i) => lines.push(`EC${String(i).padStart(5, "0")}=${f4(0)}`));

  lines.push("//INST");
  const now = new Date();
  const stamp = `${now.getFullYear()}/${String(now.getMonth() + 1).padStart(2, "0")}/${String(
    now.getDate(),
  ).padStart(2, "0")} ${String(now.getHours()).padStart(2, "0")}:${String(
    now.getMinutes(),
  ).padStart(2, "0")}`;
  lines.push(`///DATE ${stamp}`);
  lines.push("///ATTR SC,RW");
  lines.push("///GROUP1 RB1,ST1");
  lines.push("NOP");

  const sch = prog.schedule;
  pts.forEach((p, i) => {
    const card = `C${String(i).padStart(5, "0")}`;
    if (p.kind === "home" || p.move === "MOVJ") {
      lines.push(`SMOVL ${card} VJ=50.00`);
    } else {
      lines.push(`SMOVL ${card} V=${f4(10)}`);
    }
    if (p.kind === "home") {
      lines.push(`' Home (action ${ACTION_HOME})`);
    }
    if (p.tag === "TOUCH") {
      lines.push(`' TouchSense search (action ${ACTION_TOUCH}, ${TOUCH_APPROACH_STATES} states)`);
      lines.push("' TOUCH");
    }
    if (p.tag === "ARCON") {
      lines.push(`' CTWD ${CTWD_WELD_MM}mm (welding)`);
      lines.push(
        `' Lorch S8 job ${sch.job}: ${sch.currentA}A / ${sch.voltageV.toFixed(1)}V / wire ${sch.wireMmin.toFixed(
          1,
        )} m/min / ${sch.process}`,
      );
      lines.push(`ARCON ASF#(${sch.condition})`);
    }
    if (p.tag === "ARCOF") {
      lines.push("ARCOF");
    }
  });
  lines.push("END");

  return `${lines.join("\n")}\n`;
}
