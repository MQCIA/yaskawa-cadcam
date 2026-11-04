/**
 * Diagnostics + seam-edit helpers for the 3D trajectory viewer / inspector.
 * Keeps INFORM-style labels (C#####) aligned with the DX200 pendant FWD/BWD flow.
 */

import {
  checkCollisionFree,
  checkSingularitySafe,
  type Vec3,
  type Waypoint,
  type WeldProgram,
} from "@/lib/weldProgram";

export type IkStatus = "ok" | "near_limit" | "collision";

export type WaypointDiag = {
  index: number;
  /** INFORM C# label — C00000 … */
  cLabel: string;
  move: string;
  speedLabel: string;
  kind: Waypoint["kind"];
  bc?: number;
  ec?: [number, number];
  ik: IkStatus;
  ikLabel: string;
};

export type TimelinePhase =
  | { kind: "movj"; start: number; end: number }
  | { kind: "weld"; start: number; end: number }
  | { kind: "sense"; start: number; end: number };

export type TimelineMarker = {
  /** Progress 0..1 */
  t: number;
  kind: "ec_bc" | "warn";
  wpIndex: number;
  label: string;
};

/** Local per-seam overrides edited in the Seam Inspector. */
export type SeamEdit = {
  id: string;
  label: string;
  /** Inclusive waypoint indices covering weld samples for this seam. */
  weldStart: number;
  weldEnd: number;
  approachIdx: number;
  retractIdx: number;
  spinOffsetDeg: number;
  travelAngleDeg: number;
  approachMm: number;
  /** 0 = no weave */
  weaveNo: number;
  weldSpeedV: number;
  reversed: boolean;
};

export function informCLabel(index: number): string {
  return `C${String(index).padStart(5, "0")}`;
}

function hitObstacles(prog: WeldProgram, a: Vec3, b: Vec3): boolean {
  // Re-use coarse collision via a tiny temp program check on one segment.
  const mini: WeldProgram = {
    ...prog,
    waypoints: [
      {
        id: "a",
        pos: a,
        move: "MOVJ",
        speedLabel: "VJ=80",
        arc: false,
        kind: "travel",
      },
      {
        id: "b",
        pos: b,
        move: "MOVJ",
        speedLabel: "VJ=80",
        arc: false,
        kind: "travel",
      },
    ],
    segDurations: [1],
    cycleSec: 1,
  };
  return !checkCollisionFree(mini);
}

/** Build per-waypoint diagnostics (IK / collision heuristics for the demo). */
export function diagnoseWaypoints(prog: WeldProgram): WaypointDiag[] {
  const singular = !checkSingularitySafe(prog);
  const out: WaypointDiag[] = [];
  for (let i = 0; i < prog.waypoints.length; i++) {
    const w = prog.waypoints[i];
    let ik: IkStatus = "ok";
    if (i > 0 && w.kind !== "weld" && w.kind !== "sense") {
      if (hitObstacles(prog, prog.waypoints[i - 1].pos, w.pos)) ik = "collision";
    }
    if (ik === "ok" && singular && (w.kind === "weld" || w.kind === "approach")) {
      ik = "near_limit";
    }
    // Dense joint-step warning from backend planner meta.
    const maxStep = prog.meta?.maxJointStepDeg ?? 0;
    if (ik === "ok" && maxStep > 25 && w.kind === "weld") ik = "near_limit";

    const ikLabel =
      ik === "ok" ? "OK" : ik === "near_limit" ? "Blisko limitu osi" : "Kolizja";

    out.push({
      index: i,
      cLabel: informCLabel(i),
      move: w.move,
      speedLabel: w.speedLabel,
      kind: w.kind,
      bc: w.bc,
      ec: w.ec,
      ik,
      ikLabel,
    });
  }
  return out;
}

/** Progress fractions (0..1) at each waypoint — matches page `wpProgress`. */
export function waypointProgress(prog: WeldProgram): number[] {
  const cum = [0];
  for (const d of prog.segDurations) cum.push(cum[cum.length - 1] + d);
  const total = prog.cycleSec || 1;
  return cum.map((c) => c / total);
}

/** Colored phase strips under the sim scrubber. */
export function buildTimelinePhases(prog: WeldProgram): TimelinePhase[] {
  const progFrac = waypointProgress(prog);
  const phases: TimelinePhase[] = [];
  for (let i = 0; i < prog.waypoints.length - 1; i++) {
    const a = prog.waypoints[i];
    const b = prog.waypoints[i + 1];
    const start = progFrac[i];
    const end = progFrac[i + 1];
    if (a.arc && b.arc) phases.push({ kind: "weld", start, end });
    else if (a.kind === "sense" || b.kind === "sense" || a.move === "MOVS")
      phases.push({ kind: "sense", start, end });
    else phases.push({ kind: "movj", start, end });
  }
  return phases;
}

export function buildTimelineMarkers(
  prog: WeldProgram,
  diags: WaypointDiag[],
): TimelineMarker[] {
  const progFrac = waypointProgress(prog);
  const markers: TimelineMarker[] = [];
  for (let i = 0; i < prog.waypoints.length; i++) {
    const w = prog.waypoints[i];
    const t = progFrac[i] ?? 0;
    const prev = i > 0 ? prog.waypoints[i - 1] : null;
    const bcChanged = prev && w.bc != null && prev.bc != null && w.bc !== prev.bc;
    const ecChanged =
      prev &&
      w.ec &&
      prev.ec &&
      (w.ec[0] !== prev.ec[0] || w.ec[1] !== prev.ec[1]);
    if (
      bcChanged ||
      ecChanged ||
      w.kind === "index_safe" ||
      w.kind === "clearance"
    ) {
      markers.push({
        t,
        kind: "ec_bc",
        wpIndex: i,
        label: bcChanged ? `BC→${w.bc}` : "EC/BC",
      });
    }
    if (diags[i]?.ik !== "ok") {
      markers.push({
        t,
        kind: "warn",
        wpIndex: i,
        label: diags[i].ikLabel,
      });
    }
  }
  return markers;
}

const add = (a: Vec3, b: Vec3): Vec3 => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const sub = (a: Vec3, b: Vec3): Vec3 => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const scale = (a: Vec3, s: number): Vec3 => [a[0] * s, a[1] * s, a[2] * s];
const norm = (a: Vec3): Vec3 => {
  const l = Math.hypot(a[0], a[1], a[2]) || 1;
  return [a[0] / l, a[1] / l, a[2] / l];
};
const cross = (a: Vec3, b: Vec3): Vec3 => [
  a[1] * b[2] - a[2] * b[1],
  a[2] * b[0] - a[0] * b[2],
  a[0] * b[1] - a[1] * b[0],
];

function rotateAboutAxis(v: Vec3, axis: Vec3, deg: number): Vec3 {
  const a = norm(axis);
  const rad = (deg * Math.PI) / 180;
  const c = Math.cos(rad);
  const s = Math.sin(rad);
  // Rodrigues
  const term1 = scale(v, c);
  const term2 = scale(cross(a, v), s);
  const term3 = scale(a, (1 - c) * (a[0] * v[0] + a[1] * v[1] + a[2] * v[2]));
  return add(add(term1, term2), term3);
}

/** Discover logical seams from contiguous weld waypoint runs. */
export function discoverSeams(prog: WeldProgram): SeamEdit[] {
  const seams: SeamEdit[] = [];
  let i = 0;
  let n = 1;
  while (i < prog.waypoints.length) {
    if (prog.waypoints[i].kind !== "weld") {
      i++;
      continue;
    }
    const weldStart = i;
    while (i < prog.waypoints.length && prog.waypoints[i].kind === "weld") i++;
    const weldEnd = i - 1;
    let approachIdx = weldStart;
    for (let j = weldStart - 1; j >= 0; j--) {
      const k = prog.waypoints[j].kind;
      if (k === "approach" || k === "travel" || k === "clearance") {
        approachIdx = j;
        break;
      }
      if (k === "weld") break;
    }
    let retractIdx = weldEnd;
    for (let j = weldEnd + 1; j < prog.waypoints.length; j++) {
      const k = prog.waypoints[j].kind;
      if (k === "retract" || k === "travel" || k === "clearance") {
        retractIdx = j;
        break;
      }
      if (k === "weld") break;
    }
    const existing = prog.meta?.seamEdits?.find(
      (s) => s.weldStart === weldStart && s.weldEnd === weldEnd,
    );
    seams.push(
      existing ?? {
        id: `seam-${n}`,
        label: `Spoina ${n}`,
        weldStart,
        weldEnd,
        approachIdx,
        retractIdx,
        spinOffsetDeg: prog.meta?.spinAngleDeg ?? 0,
        travelAngleDeg: 12,
        approachMm: 60,
        weaveNo: 21,
        weldSpeedV: prog.schedule.travelCmMin,
        reversed: false,
      },
    );
    n++;
  }
  return seams;
}

/** Zigzag polyline for weave visualisation (WVON). */
export function weavePolyline(points: Vec3[], ampMm = 4): Vec3[] {
  if (points.length < 2) return points;
  const amp = ampMm / 1000;
  const out: Vec3[] = [points[0]];
  for (let i = 1; i < points.length; i++) {
    const a = points[i - 1];
    const b = points[i];
    const along = norm(sub(b, a));
    // Prefer world-up cross travel for lateral weave.
    let side = cross(along, [0, 1, 0]);
    if (Math.hypot(...side) < 1e-6) side = cross(along, [1, 0, 0]);
    side = norm(side);
    const mid = add(scale(add(a, b), 0.5), scale(side, i % 2 === 0 ? amp : -amp));
    out.push(mid, b);
  }
  return out;
}

/**
 * Apply inspector edits to one seam and rebuild durations / meta.
 * Spin / travel tilt torch frames; approachMm moves approach & retract along −dir.
 */
export function applySeamEdit(
  prog: WeldProgram,
  edit: SeamEdit,
  opts?: { reverse?: boolean },
): WeldProgram {
  const doReverse = Boolean(opts?.reverse);
  const target: SeamEdit = {
    ...edit,
    reversed: doReverse ? !edit.reversed : edit.reversed,
  };
  const waypoints = prog.waypoints.map((w) => ({ ...w, pos: [...w.pos] as Vec3 }));
  let seamPts = prog.seam.map((p) => [...p] as Vec3);
  let frames = prog.torchFrames.map((f) => ({
    pos: [...f.pos] as Vec3,
    dir: [...f.dir] as Vec3,
  }));

  if (doReverse) {
    const slice = waypoints
      .slice(target.weldStart, target.weldEnd + 1)
      .map((w) => ({ ...w, pos: [...w.pos] as Vec3 }))
      .reverse();
    for (let k = 0; k < slice.length; k++) {
      const src = slice[k];
      waypoints[target.weldStart + k] = {
        ...src,
        tag:
          k === 0
            ? "ARCON"
            : k === slice.length - 1
              ? "ARCOF"
              : undefined,
      };
    }
    const ap = waypoints[target.approachIdx];
    const rt = waypoints[target.retractIdx];
    if (ap && rt && target.approachIdx !== target.retractIdx) {
      const tmp = [...ap.pos] as Vec3;
      waypoints[target.approachIdx] = { ...ap, pos: [...rt.pos] as Vec3 };
      waypoints[target.retractIdx] = { ...rt, pos: tmp };
    }
    seamPts = [...seamPts].reverse();
    frames = [...frames].reverse();
  }

  // Preferred wire from mid frame, then apply travel tilt + spin bank for viz.
  const mid = frames[Math.floor(frames.length / 2)]?.dir ??
    ([0, -Math.SQRT1_2, -Math.SQRT1_2] as Vec3);
  const travel = norm(
    sub(waypoints[target.weldEnd].pos, waypoints[target.weldStart].pos),
  );
  let dir = norm(mid);
  const tiltAxis = cross(dir, travel);
  if (Math.hypot(...tiltAxis) > 1e-6) {
    dir = rotateAboutAxis(dir, tiltAxis, target.travelAngleDeg - 12);
  }
  // Spin about the wire: bank the aim slightly around travel so the cone updates.
  if (Math.abs(target.spinOffsetDeg) > 1e-3 && Math.hypot(...travel) > 1e-6) {
    dir = rotateAboutAxis(dir, travel, target.spinOffsetDeg * 0.15);
  }
  dir = norm(dir);

  const weldA = waypoints[target.weldStart].pos;
  const weldB = waypoints[target.weldEnd].pos;
  // Standoff out of the joint (opposite torch-into-work).
  const lift = scale(dir, -(target.approachMm / 1000));
  if (waypoints[target.approachIdx] && waypoints[target.approachIdx].kind !== "weld") {
    waypoints[target.approachIdx] = {
      ...waypoints[target.approachIdx],
      pos: add(weldA, lift),
      move: "MOVJ",
      speedLabel: "VJ=80.00",
    };
  }
  if (
    target.retractIdx !== target.approachIdx &&
    waypoints[target.retractIdx] &&
    waypoints[target.retractIdx].kind !== "weld"
  ) {
    waypoints[target.retractIdx] = {
      ...waypoints[target.retractIdx],
      pos: add(weldB, lift),
      move: "MOVJ",
      speedLabel: "VJ=50.00",
    };
  }

  const speedLabel = `V=${target.weldSpeedV.toFixed(1)}`;
  for (let i = target.weldStart; i <= target.weldEnd; i++) {
    waypoints[i] = {
      ...waypoints[i],
      speedLabel,
      move: waypoints[i].move === "MOVC" ? "MOVC" : "MOVL",
    };
  }

  const weldCount = target.weldEnd - target.weldStart + 1;
  if (weldCount >= 2) {
    seamPts = [];
    for (let i = target.weldStart; i <= target.weldEnd; i++) {
      seamPts.push([...waypoints[i].pos] as Vec3);
    }
  }
  frames = seamPts.map((pos) => ({ pos, dir }));

  const otherSeams = discoverSeams(prog)
    .filter((s) => s.id !== target.id)
    .concat([target]);

  const airSpeed = 0.5;
  const weldSpeed = Math.max(target.weldSpeedV / 100 / 60, 1e-6);
  const segDurations: number[] = [];
  for (let i = 0; i < waypoints.length - 1; i++) {
    const a = waypoints[i];
    const b = waypoints[i + 1];
    const d = Math.hypot(
      a.pos[0] - b.pos[0],
      a.pos[1] - b.pos[1],
      a.pos[2] - b.pos[2],
    );
    segDurations.push(d / (a.arc && b.arc ? weldSpeed : airSpeed));
  }

  const next: WeldProgram = {
    ...prog,
    waypoints,
    seam: seamPts,
    torchFrames: frames,
    schedule: { ...prog.schedule, travelCmMin: target.weldSpeedV },
    segDurations,
    cycleSec: segDurations.reduce((s, d) => s + d, 0),
    weldLenMm: (() => {
      let L = 0;
      for (let i = 0; i < seamPts.length - 1; i++) {
        const a = seamPts[i];
        const b = seamPts[i + 1];
        L += Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
      }
      return L * 1000;
    })(),
    meta: {
      collisionFree: true,
      touchSense: prog.meta?.touchSense ?? false,
      sequenceOptimized: prog.meta?.sequenceOptimized ?? false,
      singularitySafe: true,
      ...prog.meta,
      spinAngleDeg: target.spinOffsetDeg,
      seamEdits: otherSeams,
      activeWeaveNo: target.weaveNo,
    },
  };
  next.meta = {
    ...next.meta!,
    collisionFree: checkCollisionFree(next),
    singularitySafe: checkSingularitySafe(next),
  };
  return next;
}
