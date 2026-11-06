/**
 * DX200-style motion simulation (client):
 *  - MOVJ: joint-space trapezoidal interp → TCP via approximate MA2010 DH FK
 *  - MOVL: linear TCP + SLERP torch dir
 *  - MOVC: circle-through-3 at constant V
 *  - TIMER dwell, weave overlay, wrist-singularity / joint-rate warnings
 *  - Cycle time matching controller physics (V / VJ / DEC / TIMER)
 */

import type { MoveType, Vec3, Waypoint, WeldProgram } from "@/lib/weldProgram";

/** Motoman TOOL.CND — duplicated to avoid circular import with weldProgram. */
const TOOL0_XYZ_M: Vec3 = [-0.05428, 0.001052, 0.415702];
const TOOL0_RPY_DEG: Vec3 = [1.3119, -42.6215, -6.8387];

const WEAVE_TABLE: { wevNo: number; ampLmm: number; ampRmm: number; frequencyHz: number; mode: number }[] = [
  { wevNo: 1, ampLmm: 2, ampRmm: 2, frequencyHz: 2, mode: 0 },
  { wevNo: 15, ampLmm: 3, ampRmm: 3, frequencyHz: 3, mode: 1 },
  { wevNo: 16, ampLmm: 2, ampRmm: 2, frequencyHz: 2, mode: 1 },
  { wevNo: 21, ampLmm: 4, ampRmm: 0, frequencyHz: 2, mode: 0 },
];

export const JOINT_MAX_DEG_S = [197, 175, 175, 360, 360, 520] as const;
export const BC_MAX_MMS = 1500;
export const EC_MAX_DEG_S = 90;
export const WRIST_SING_B_DEG = 5;
export const JOINT_RATE_WARN_DEG_S = 120;

/** MA2010 approximate DH (matches backend robot_config._ar_model). */
const DH = [
  { a: 0.15, alpha: -Math.PI / 2, d: 0.54, offset: 0 },
  { a: 0.76, alpha: 0, d: 0, offset: -Math.PI / 2 },
  { a: 0.2, alpha: -Math.PI / 2, d: 0, offset: 0 },
  { a: 0, alpha: Math.PI / 2, d: 1.082, offset: 0 },
  { a: 0, alpha: -Math.PI / 2, d: 0, offset: 0 },
  { a: 0, alpha: 0, d: 0.1, offset: 0 },
] as const;

const FLANGE_TO_TOOL0 = [
  [0, 0, 1],
  [0, -1, 0],
  [1, 0, 0],
] as const;

export type SimWarning =
  | { kind: "singularity"; message: string }
  | { kind: "joint_rate"; axis: string; rate: number; message: string }
  | { kind: "collision"; message: string };

export type SimSample = {
  pos: Vec3;
  dir: Vec3;
  wpIndex: number;
  arcOn: boolean;
  elapsedSec: number;
  move: MoveType | "TIMER";
  /** Interpolated joints when available (deg S..T). */
  jointsDeg?: number[];
  bc?: number;
  ecDeg?: number;
  weaveActive: boolean;
  warnings: SimWarning[];
  /** Path class for viz: joint vs cartesian. */
  pathClass: "joint" | "cart" | "dwell";
};

const add = (a: Vec3, b: Vec3): Vec3 => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const sub = (a: Vec3, b: Vec3): Vec3 => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const scale = (a: Vec3, s: number): Vec3 => [a[0] * s, a[1] * s, a[2] * s];
const dot = (a: Vec3, b: Vec3) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
export const vnorm = (a: Vec3): Vec3 => {
  const l = Math.hypot(a[0], a[1], a[2]) || 1;
  return [a[0] / l, a[1] / l, a[2] / l];
};
const cross = (a: Vec3, b: Vec3): Vec3 => [
  a[1] * b[2] - a[2] * b[1],
  a[2] * b[0] - a[0] * b[2],
  a[0] * b[1] - a[1] * b[0],
];
const dist = (a: Vec3, b: Vec3) => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);

function mul4(A: number[][], B: number[][]): number[][] {
  const C = Array.from({ length: 4 }, () => [0, 0, 0, 0]);
  for (let i = 0; i < 4; i++)
    for (let j = 0; j < 4; j++)
      for (let k = 0; k < 4; k++) C[i][j] += A[i][k] * B[k][j];
  return C;
}

function dhMat(a: number, alpha: number, d: number, theta: number): number[][] {
  const ca = Math.cos(alpha);
  const sa = Math.sin(alpha);
  const ct = Math.cos(theta);
  const st = Math.sin(theta);
  return [
    [ct, -st * ca, st * sa, a * ct],
    [st, ct * ca, -ct * sa, a * st],
    [0, sa, ca, d],
    [0, 0, 0, 1],
  ];
}

function rpyXyz(rx: number, ry: number, rz: number): number[][] {
  const cx = Math.cos(rx);
  const sx = Math.sin(rx);
  const cy = Math.cos(ry);
  const sy = Math.sin(ry);
  const cz = Math.cos(rz);
  const sz = Math.sin(rz);
  // R = Rx·Ry·Rz
  return [
    [cy * cz, -cy * sz, sy, 0],
    [sx * sy * cz + cx * sz, -sx * sy * sz + cx * cz, -sx * cy, 0],
    [-cx * sy * cz + sx * sz, cx * sy * sz + sx * cz, cx * cy, 0],
    [0, 0, 0, 1],
  ];
}

/** Approximate Motoman tool0 tip (DH chain uses Motoman Z+ tip). */
export function fkTcp(qDeg: number[]): { pos: Vec3; dir: Vec3 } {
  let T = [
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1],
  ];
  for (let i = 0; i < 6; i++) {
    const link = DH[i];
    const th = (qDeg[i] * Math.PI) / 180 + link.offset;
    T = mul4(T, dhMat(link.a, link.alpha, link.d, th));
  }
  // tip = Motoman tool0; tcp = tip · TOOL.CND
  const d2r = Math.PI / 180;
  const tool = rpyXyz(
    TOOL0_RPY_DEG[0] * d2r,
    TOOL0_RPY_DEG[1] * d2r,
    TOOL0_RPY_DEG[2] * d2r,
  );
  tool[0][3] = TOOL0_XYZ_M[0];
  tool[1][3] = TOOL0_XYZ_M[1];
  tool[2][3] = TOOL0_XYZ_M[2];
  T = mul4(T, tool);
  const pos: Vec3 = [T[0][3], T[1][3], T[2][3]];
  // Wire = +Z of tcp (stick-out); into-work ≈ −Z for fillet demos.
  const dir = vnorm([-T[0][2], -T[1][2], -T[2][2]]);
  void FLANGE_TO_TOOL0;
  return { pos, dir };
}

export function trapezoidS(u: number, decPct = 20): number {
  const uu = Math.max(0, Math.min(1, u));
  const dec = Math.max(0.01, Math.min(0.45, decPct / 100));
  const acc = dec;
  const cruise = Math.max(0, 1 - acc - dec);
  let raw: number;
  if (uu < acc) {
    const x = uu / acc;
    raw = 0.5 * acc * x * x;
  } else if (uu < acc + cruise) {
    raw = 0.5 * acc + (uu - acc);
  } else {
    const x = (uu - (acc + cruise)) / dec;
    raw = 0.5 * acc + cruise + dec * (x - 0.5 * x * x);
  }
  const end = (() => {
    const x = 1;
    void x;
    const eDec = dec;
    return 0.5 * acc + cruise + eDec * (1 - 0.5);
  })();
  return Math.max(0, Math.min(1, raw / Math.max(end, 1e-9)));
}

export function movjDurationSec(
  q0: number[],
  q1: number[],
  vj: number,
  dec = 20,
  opts?: { bc0?: number; bc1?: number; ec0?: number; ec1?: number },
): number {
  const vjFrac = Math.max(vj, 0.1) / 100;
  const stretch = 1 + 0.35 * (dec / 100);
  let tMax = 0;
  for (let i = 0; i < 6; i++) {
    const w = JOINT_MAX_DEG_S[i] * vjFrac;
    tMax = Math.max(tMax, Math.abs(q1[i] - q0[i]) / Math.max(w, 1e-6));
  }
  if (opts?.bc0 != null && opts?.bc1 != null) {
    let dbc = Math.abs(opts.bc1 - opts.bc0);
    if (dbc > 50) dbc = (dbc * 0.002) / 1000;
    tMax = Math.max(tMax, dbc / Math.max((BC_MAX_MMS / 1000) * vjFrac, 1e-6));
  }
  if (opts?.ec0 != null && opts?.ec1 != null) {
    tMax = Math.max(
      tMax,
      Math.abs(opts.ec1 - opts.ec0) / Math.max(EC_MAX_DEG_S * vjFrac, 1e-6),
    );
  }
  return tMax * stretch;
}

/** V in cm/min → duration for length metres. */
export function cartDurationSec(lengthM: number, vCmMin: number): number {
  const vMms = Math.max(vCmMin, 0.01) * 10 / 60;
  return (Math.max(lengthM, 0) * 1000) / vMms;
}

export function slerpDir(a: Vec3, b: Vec3, s: number): Vec3 {
  const A = vnorm(a);
  const B = vnorm(b);
  const d = Math.max(-1, Math.min(1, dot(A, B)));
  if (d > 0.9995) return vnorm(add(scale(A, 1 - s), scale(B, s)));
  const omega = Math.acos(d);
  const so = Math.sin(omega);
  return vnorm(
    add(scale(A, Math.sin((1 - s) * omega) / so), scale(B, Math.sin(s * omega) / so)),
  );
}

export function circlePoint(p1: Vec3, p2: Vec3, p3: Vec3, s: number): Vec3 {
  const ab = sub(p2, p1);
  const ac = sub(p3, p1);
  const n = cross(ab, ac);
  if (Math.hypot(...n) < 1e-10) return add(scale(p1, 1 - s), scale(p3, s));
  const ab2 = dot(ab, ab);
  const ac2 = dot(ac, ac);
  const abac = dot(ab, ac);
  const den = 2 * (ab2 * ac2 - abac * abac);
  if (Math.abs(den) < 1e-14) return add(scale(p1, 1 - s), scale(p3, s));
  const k1 = (ab2 * ac2 - ac2 * abac) / den;
  const k2 = (ac2 * ab2 - ab2 * abac) / den;
  const centre = add(p1, add(scale(ab, k1), scale(ac, k2)));
  const r = dist(centre, p1);
  const nn = vnorm(n);
  const u = vnorm(sub(p1, centre));
  const v = vnorm(cross(nn, u));
  const angOf = (p: Vec3) => {
    const d = vnorm(sub(p, centre));
    return Math.atan2(dot(d, v), dot(d, u));
  };
  let a0 = angOf(p1);
  let a1 = angOf(p2);
  let a2 = angOf(p3);
  const unwrap = (prev: number, next: number) => {
    let x = next;
    while (x - prev > Math.PI) x -= 2 * Math.PI;
    while (x - prev < -Math.PI) x += 2 * Math.PI;
    return x;
  };
  a1 = unwrap(a0, a1);
  a2 = unwrap(a1, a2);
  const ang = (1 - s) * (1 - s) * a0 + 2 * (1 - s) * s * a1 + s * s * a2;
  return add(centre, add(scale(u, Math.cos(ang) * r), scale(v, Math.sin(ang) * r)));
}

export function circleLength(p1: Vec3, p2: Vec3, p3: Vec3): number {
  const ab = sub(p2, p1);
  const ac = sub(p3, p1);
  const n = cross(ab, ac);
  if (Math.hypot(...n) < 1e-10) return dist(p1, p3);
  const ab2 = dot(ab, ab);
  const ac2 = dot(ac, ac);
  const abac = dot(ab, ac);
  const den = 2 * (ab2 * ac2 - abac * abac);
  if (Math.abs(den) < 1e-14) return dist(p1, p3);
  const k1 = (ab2 * ac2 - ac2 * abac) / den;
  const k2 = (ac2 * ab2 - ab2 * abac) / den;
  const centre = add(p1, add(scale(ab, k1), scale(ac, k2)));
  const r = dist(centre, p1);
  const nn = vnorm(n);
  const u = vnorm(sub(p1, centre));
  const v = vnorm(cross(nn, u));
  const angOf = (p: Vec3) => {
    const d = vnorm(sub(p, centre));
    return Math.atan2(dot(d, v), dot(d, u));
  };
  let a0 = angOf(p1);
  let a1 = angOf(p2);
  let a2 = angOf(p3);
  const unwrap = (prev: number, next: number) => {
    let x = next;
    while (x - prev > Math.PI) x -= 2 * Math.PI;
    while (x - prev < -Math.PI) x += 2 * Math.PI;
    return x;
  };
  a1 = unwrap(a0, a1);
  a2 = unwrap(a1, a2);
  return Math.abs(a2 - a0) * r;
}

export function weaveOffset(
  travel: Vec3,
  bisector: Vec3,
  tSec: number,
  ampMm: number,
  freqHz: number,
  triangular = false,
): Vec3 {
  const tr = vnorm(travel);
  let lat = vnorm(cross(tr, vnorm(bisector)));
  if (Math.hypot(...lat) < 1e-8) lat = vnorm(cross(tr, [0, 1, 0]));
  let wave: number;
  if (triangular) {
    const u = (tSec * freqHz) % 1;
    wave = 4 * Math.abs(u - 0.5) - 1;
  } else {
    wave = Math.sin(2 * Math.PI * freqHz * tSec);
  }
  return scale(lat, (ampMm / 1000) * wave);
}

function parseVj(label: string, fallback = 80): number {
  const m = /VJ\s*=\s*([\d.]+)/i.exec(label);
  return m ? Number(m[1]) : fallback;
}

function parseV(label: string, fallback: number): number {
  const m = /(?:^|[^A-Z])V\s*=\s*([\d.]+)/i.exec(label);
  return m ? Number(m[1]) : fallback;
}

function isJointMove(w: Waypoint): boolean {
  return (
    w.move === "MOVJ" ||
    w.kind === "approach" ||
    w.kind === "retract" ||
    w.kind === "travel" ||
    w.kind === "clearance" ||
    w.kind === "index_safe"
  );
}

/** Synthetic seed joints from TCP (coarse, for MOVJ timing / path when IK missing). */
export function estimateJoints(tcp: Vec3, seed?: number[]): number[] {
  const base = seed ?? [0, -45, -45, 0, -80, 0];
  const yaw = (Math.atan2(tcp[1], tcp[0]) * 180) / Math.PI;
  const reach = Math.hypot(tcp[0], tcp[1], Math.max(0.05, tcp[2] - 0.54));
  const L = Math.max(-80, Math.min(60, ((reach - 1.2) / 0.8) * 40 - 40));
  const U = Math.max(-120, Math.min(80, -L * 0.7 - 20));
  return [
    Math.max(-160, Math.min(160, yaw * 0.85 + base[0] * 0.1)),
    L,
    U,
    base[3],
    base[4],
    base[5],
  ];
}

function jointsAt(
  prog: WeldProgram,
  index: number,
  cache: number[][],
): number[] {
  if (cache[index]) return cache[index];
  const wp = prog.waypoints[index];
  // Prefer taught / imported PULSE joints on the waypoint itself.
  if (wp.jointsDeg && wp.jointsDeg.length === 6) {
    cache[index] = wp.jointsDeg;
    return cache[index];
  }
  const dense = prog.meta?.denseJointAnglesDeg;
  if (dense && dense.length === prog.waypoints.length && dense[index]?.length === 6) {
    cache[index] = dense[index];
    return cache[index];
  }
  const prev = index > 0 ? cache[index - 1] : undefined;
  cache[index] = estimateJoints(wp.pos, prev);
  return cache[index];
}

/** True when the program carries real joint samples (PULSE .JBI / IK export). */
function hasTaughtJoints(prog: WeldProgram, i: number): boolean {
  const wp = prog.waypoints[i];
  if (wp.jointsDeg && wp.jointsDeg.length === 6) return true;
  const dense = prog.meta?.denseJointAnglesDeg;
  return Boolean(dense && dense.length === prog.waypoints.length && dense[i]?.length === 6);
}

function dirAt(prog: WeldProgram, i: number): Vec3 {
  const f = prog.torchFrames;
  if (!f.length) return [0, -Math.SQRT1_2, -Math.SQRT1_2];
  const j = Math.max(0, Math.min(f.length - 1, i));
  return f[j]?.dir ?? f[0].dir;
}

function ecDeg(wp: Waypoint): number | undefined {
  if (!wp.ec) return undefined;
  // Approx: EC pulses → deg via ~1019.7 p/deg (TURN).
  return wp.ec[0] / 1019.7;
}

/**
 * DX200 cycle times per waypoint segment (and TIMER dwells folded into the
 * segment that leaves a tack / crater point).
 */
export function computeDx200Durations(prog: WeldProgram): {
  segDurations: number[];
  cycleSec: number;
} {
  const wps = prog.waypoints;
  const cache: number[][] = [];
  const segs: number[] = [];
  const schV = prog.schedule.travelCmMin;

  for (let i = 0; i < wps.length - 1; i++) {
    const a = wps[i];
    const b = wps[i + 1];
    let dwell = a.timerSec ?? 0;
    // Tack: TIMER after ARCON before motion — also count when leaving weld with timer.
    if (a.kind === "weld" && a.tag === "ARCON" && (a.timerSec ?? 0) >= 1.5) {
      dwell = a.timerSec ?? 2;
    }

    let t = dwell;
    if (isJointMove(a) && a.kind !== "weld") {
      const q0 = jointsAt(prog, i, cache);
      const q1 = jointsAt(prog, i + 1, cache);
      const vj = parseVj(a.speedLabel, 80);
      t += movjDurationSec(q0, q1, vj, 20, {
        bc0: a.bc,
        bc1: b.bc,
        ec0: ecDeg(a),
        ec1: ecDeg(b),
      });
    } else if (a.move === "MOVC" && b.move === "MOVC") {
      // Prefer true arc length when a third neighbour exists.
      const prev = i > 0 ? wps[i - 1] : a;
      const mid = a;
      const end = b;
      const len =
        prev.move === "MOVC"
          ? circleLength(prev.pos, mid.pos, end.pos) / Math.max(2, 1)
          : dist(a.pos, b.pos);
      // For densified MOVC polylines use chord; for INFORM triples use circle.
      const useCircle =
        i + 1 < wps.length - 1 &&
        wps[i + 1].move === "MOVC" &&
        wps[i].move === "MOVC" &&
        (i === 0 || wps[i - 1].move !== "MOVC");
      if (useCircle && i + 2 < wps.length) {
        t += cartDurationSec(
          circleLength(wps[i].pos, wps[i + 1].pos, wps[i + 2].pos),
          parseV(a.speedLabel, schV),
        );
        // Duration of the whole triple will be attributed mostly here — OK for demo.
      } else {
        t += cartDurationSec(len, parseV(a.speedLabel, schV));
      }
    } else {
      const v = parseV(a.speedLabel, a.kind === "sense" ? 11 : schV);
      t += cartDurationSec(dist(a.pos, b.pos), v);
    }
    segs.push(Math.max(t, 1e-4));
  }
  return { segDurations: segs, cycleSec: segs.reduce((s, d) => s + d, 0) };
}

export function weaveParams(wevNo: number): { ampMm: number; freqHz: number; tri: boolean } {
  const row = WEAVE_TABLE.find((w) => w.wevNo === wevNo);
  if (!row) return { ampMm: 3, freqHz: 2.5, tri: false };
  return {
    ampMm: Math.max(row.ampLmm, row.ampRmm) || 2,
    freqHz: row.frequencyHz || 2.5,
    tri: row.mode === 1,
  };
}

/** Sample program at progress p∈[0,1] with DX200 interpolation. */
export function sampleDx200(prog: WeldProgram, p: number, weaveNo = 0): SimSample {
  const wps = prog.waypoints;
  const empty: SimSample = {
    pos: [0, 0, 0],
    dir: [0, -1, 0],
    wpIndex: 0,
    arcOn: false,
    elapsedSec: 0,
    move: "MOVJ",
    weaveActive: false,
    warnings: [],
    pathClass: "dwell",
  };
  if (!wps.length) return empty;

  const { segDurations, cycleSec } = (() => {
    if (prog.segDurations.length === wps.length - 1 && prog.cycleSec > 1e-9) {
      return { segDurations: prog.segDurations, cycleSec: prog.cycleSec };
    }
    return computeDx200Durations(prog);
  })();
  const cycle = Math.max(cycleSec, 1e-6);
  const target = Math.max(0, Math.min(1, p)) * cycle;
  const cache: number[][] = [];
  const wev = weaveNo || prog.meta?.activeWeaveNo || 0;
  const wp = weaveParams(wev);

  let acc = 0;
  for (let i = 0; i < segDurations.length; i++) {
    const d = segDurations[i];
    if (target > acc + d && i < segDurations.length - 1) {
      acc += d;
      continue;
    }
    const localT = d > 0 ? (target - acc) / d : 0;
    const a = wps[i];
    const b = wps[i + 1] ?? a;
    const dwell = a.timerSec ?? 0;
    const moveFrac = d > dwell ? Math.max(0, (target - acc - dwell) / (d - dwell || 1)) : 0;
    const u = Math.max(0, Math.min(1, moveFrac));
    const warnings: SimWarning[] = [];
    const arcOn = Boolean(a.arc && (b.arc || a.timerSec));
    const jointAir = isJointMove(a) && a.kind !== "weld";

    // Dwell / TIMER at start of segment.
    if (dwell > 0 && target - acc < dwell) {
      return {
        pos: [...a.pos] as Vec3,
        dir: dirAt(prog, i),
        wpIndex: i,
        arcOn: a.kind === "weld",
        elapsedSec: target,
        move: "TIMER",
        jointsDeg: jointsAt(prog, i, cache),
        bc: a.bc,
        ecDeg: ecDeg(a),
        weaveActive: false,
        warnings,
        pathClass: "dwell",
      };
    }

    let pos: Vec3;
    let dir: Vec3;
    let jointsDeg: number[] | undefined;
    let pathClass: SimSample["pathClass"] = "cart";
    let move: MoveType | "TIMER" = a.move;

    if (jointAir) {
      pathClass = "joint";
      move = "MOVJ";
      const q0 = jointsAt(prog, i, cache);
      const q1 = jointsAt(prog, i + 1, cache);
      const s = trapezoidS(u, 20);
      jointsDeg = q0.map((q, k) => q + (q1[k] - q) * s);
      // Stay on the drawn path (waypoint.pos / polyline), not a second FK model.
      // URDF tip is driven to this TCP by CCD IK in WeldUrdfRobot.
      pos = add(scale(a.pos, 1 - s), scale(b.pos, s));
      dir = slerpDir(dirAt(prog, i), dirAt(prog, i + 1), s);
      if (Math.abs(jointsDeg[4]) < WRIST_SING_B_DEG) {
        warnings.push({
          kind: "singularity",
          message: "Uwaga: Osobliwość kiści (Oś B blisko 0°)",
        });
      }
    } else if (
      a.move === "MOVC" &&
      i + 2 < wps.length &&
      wps[i + 1].move === "MOVC" &&
      wps[i + 2].move === "MOVC" &&
      (i === 0 || wps[i - 1].move !== "MOVC")
    ) {
      // Full INFORM triple on first chord of a MOVC run — also handle densified.
      move = "MOVC";
      pos = circlePoint(a.pos, b.pos, wps[i + 2].pos, u);
      dir = slerpDir(dirAt(prog, i), dirAt(prog, i + 2), u);
      if (hasTaughtJoints(prog, i) && hasTaughtJoints(prog, i + 1)) {
        const q0 = jointsAt(prog, i, cache);
        const q1 = jointsAt(prog, i + 1, cache);
        jointsDeg = q0.map((q, k) => q + (q1[k] - q) * u);
      }
    } else if (a.move === "MOVC") {
      move = "MOVC";
      pos = add(scale(a.pos, 1 - u), scale(b.pos, u));
      dir = slerpDir(dirAt(prog, i), dirAt(prog, i + 1), u);
      if (hasTaughtJoints(prog, i) && hasTaughtJoints(prog, i + 1)) {
        const q0 = jointsAt(prog, i, cache);
        const q1 = jointsAt(prog, i + 1, cache);
        jointsDeg = q0.map((q, k) => q + (q1[k] - q) * u);
      }
    } else {
      move = a.move === "MOVS" ? "MOVS" : "MOVL";
      const s = a.kind === "weld" ? u : trapezoidS(u, 20);
      pos = add(scale(a.pos, 1 - s), scale(b.pos, s));
      dir = slerpDir(dirAt(prog, i), dirAt(prog, i + 1), s);
      if (hasTaughtJoints(prog, i) && hasTaughtJoints(prog, i + 1)) {
        const q0 = jointsAt(prog, i, cache);
        const q1 = jointsAt(prog, i + 1, cache);
        jointsDeg = q0.map((q, k) => q + (q1[k] - q) * s);
      }
    }

    let weaveActive = false;
    if (arcOn && wev > 0 && a.kind === "weld") {
      weaveActive = true;
      const travel = vnorm(sub(b.pos, a.pos));
      const off = weaveOffset(travel, dir, target, wp.ampMm, wp.freqHz, wp.tri);
      pos = add(pos, off);
    }

    // Joint-rate check when we have neighbouring joint samples.
    if (jointsDeg && i > 0) {
      const qPrev = jointsAt(prog, i, cache);
      const dt = Math.max(d * 0.05, 1e-3);
      for (let k = 0; k < 6; k++) {
        const rate = Math.abs(jointsDeg[k] - qPrev[k]) / dt;
        const axis = ["S", "L", "U", "R", "B", "T"][k];
        if (rate > JOINT_RATE_WARN_DEG_S && (axis === "R" || axis === "T" || axis === "B")) {
          warnings.push({
            kind: "joint_rate",
            axis,
            rate,
            message: `Prędkość osi ${axis}: ${rate.toFixed(0)}°/s`,
          });
        }
      }
    }

    // Coarse torch-body collision vs plates (ignore tip in groove).
    if (prog.plates?.length) {
      const shaft = add(pos, scale(dir, -0.12)); // 120 mm back along wire
      for (const pl of prog.plates) {
        const half = scale(pl.size, 0.5);
        const dlt = sub(shaft, pl.center);
        const inside =
          Math.abs(dlt[0]) < half[0] + 0.02 &&
          Math.abs(dlt[1]) < half[1] + 0.02 &&
          Math.abs(dlt[2]) < half[2] + 0.02;
        // Only flag if shaft penetrates but tip is outside the plate top.
        const tipOutside =
          Math.abs(pos[0] - pl.center[0]) > half[0] * 0.9 ||
          Math.abs(pos[2] - pl.center[2]) > half[2] * 0.9 ||
          pos[1] > pl.center[1] + half[1] + 0.01;
        if (inside && tipOutside) {
          warnings.push({
            kind: "collision",
            message: "Kolizja: korpus palnika / ramię ↔ detal",
          });
          break;
        }
      }
    }

    return {
      pos,
      dir,
      wpIndex: i,
      arcOn,
      elapsedSec: target,
      move,
      jointsDeg,
      bc: a.bc != null && b.bc != null ? a.bc + (b.bc - a.bc) * u : a.bc,
      ecDeg:
        ecDeg(a) != null && ecDeg(b) != null
          ? (ecDeg(a) as number) + ((ecDeg(b) as number) - (ecDeg(a) as number)) * u
          : ecDeg(a),
      weaveActive,
      warnings,
      pathClass,
    };
  }

  const last = wps[wps.length - 1];
  return {
    pos: last.pos,
    dir: dirAt(prog, wps.length - 1),
    wpIndex: wps.length - 1,
    arcOn: false,
    elapsedSec: cycle,
    move: last.move,
    jointsDeg: jointsAt(prog, wps.length - 1, cache),
    weaveActive: false,
    warnings: [],
    pathClass: "cart",
  };
}

/**
 * Dense samples along a MOVJ segment for the blue air-path polyline.
 * Uses the same trapezoid blend of waypoint.pos as ``sampleDx200`` so the
 * dashed path matches the TCP the robot is IK-driven toward.
 */
export function movjTcpPolyline(
  prog: WeldProgram,
  i0: number,
  i1: number,
  samples = 20,
): Vec3[] {
  const a = prog.waypoints[i0]?.pos;
  const b = prog.waypoints[i1]?.pos;
  if (!a || !b) return [];
  const out: Vec3[] = [];
  for (let i = 0; i < samples; i++) {
    const s = trapezoidS(i / (samples - 1), 20);
    out.push(add(scale(a, 1 - s), scale(b, s)));
  }
  return out;
}

/** Zigzag weld bead with physical weave for the growing seam trail. */
export function weaveBeadPoints(
  seam: Vec3[],
  dirs: Vec3[],
  wevNo: number,
  progress = 1,
): Vec3[] {
  if (seam.length < 2) return seam;
  const { ampMm, freqHz, tri } = weaveParams(wevNo);
  const out: Vec3[] = [];
  let along = 0;
  const cum = [0];
  for (let i = 0; i < seam.length - 1; i++) cum.push(cum[i] + dist(seam[i], seam[i + 1]));
  const total = cum[cum.length - 1] || 1;
  const limit = progress * total;
  for (let i = 0; i < seam.length; i++) {
    if (cum[i] > limit) break;
    along = cum[i];
    const travel =
      i < seam.length - 1
        ? vnorm(sub(seam[i + 1], seam[i]))
        : vnorm(sub(seam[i], seam[i - 1]));
    const dir = dirs[Math.min(i, dirs.length - 1)] ?? ([0, -1, 0] as Vec3);
    const tSec = along / Math.max(0.001, 0.05); // ~5 cm/min scale for phase
    out.push(add(seam[i], weaveOffset(travel, dir, tSec / freqHz, ampMm, freqHz, tri)));
  }
  return out;
}
