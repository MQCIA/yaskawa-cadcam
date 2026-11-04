/**
 * 3D Seam Sketcher helpers — snap to mesh edges/vertices, circle-through-3,
 * Catmull-Rom sampling, and fillet bisector normals for torch aim.
 */

import type { Vec3 } from "@/lib/weldProgram";

export type SketchMode = "orbit" | "tack" | "line" | "arc" | "spline";

export type SnapKind = "none" | "vertex" | "edge";

export type SketchHit = {
  point: Vec3;
  /** Outward surface / fillet-bisector normal (unit). */
  normal: Vec3;
  snap: SnapKind;
};

export const SNAP_RADIUS_M = 0.010; // 10 mm default edge/vertex snap

const add = (a: Vec3, b: Vec3): Vec3 => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const sub = (a: Vec3, b: Vec3): Vec3 => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const scale = (a: Vec3, s: number): Vec3 => [a[0] * s, a[1] * s, a[2] * s];
const dot = (a: Vec3, b: Vec3) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
export const sketchNorm = (a: Vec3): Vec3 => {
  const l = Math.hypot(a[0], a[1], a[2]) || 1;
  return [a[0] / l, a[1] / l, a[2] / l];
};
const dist = (a: Vec3, b: Vec3) => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);

/** Closest point on segment AB to P. */
export function closestOnSegment(p: Vec3, a: Vec3, b: Vec3): { point: Vec3; t: number } {
  const ab = sub(b, a);
  const len2 = dot(ab, ab);
  if (len2 < 1e-18) return { point: a, t: 0 };
  const t = Math.max(0, Math.min(1, dot(sub(p, a), ab) / len2));
  return { point: add(a, scale(ab, t)), t };
}

/**
 * Snap a hit point to the nearest triangle vertex or edge within ``radiusM``.
 * ``tri`` = three world-space corners; ``faceNormal`` already world-transformed.
 */
export function snapToTriangle(
  hit: Vec3,
  tri: [Vec3, Vec3, Vec3],
  faceNormal: Vec3,
  radiusM = SNAP_RADIUS_M,
  snapEnabled = true,
): SketchHit {
  const n = sketchNorm(faceNormal);
  if (!snapEnabled) {
    return { point: hit, normal: n, snap: "none" };
  }

  let bestV = Infinity;
  let bestVpt: Vec3 = hit;
  for (const v of tri) {
    const d = dist(hit, v);
    if (d < bestV) {
      bestV = d;
      bestVpt = v;
    }
  }
  if (bestV <= radiusM) {
    return { point: [...bestVpt] as Vec3, normal: n, snap: "vertex" };
  }

  let bestE = Infinity;
  let bestEpt: Vec3 = hit;
  const edges: [Vec3, Vec3][] = [
    [tri[0], tri[1]],
    [tri[1], tri[2]],
    [tri[2], tri[0]],
  ];
  for (const [a, b] of edges) {
    const { point } = closestOnSegment(hit, a, b);
    const d = dist(hit, point);
    if (d < bestE) {
      bestE = d;
      bestEpt = point;
    }
  }
  if (bestE <= radiusM) {
    return { point: [...bestEpt] as Vec3, normal: n, snap: "edge" };
  }
  return { point: hit, normal: n, snap: "none" };
}

/** Fillet bisector of two unit face normals (out of the joint). */
export function bisectorNormal(n1: Vec3, n2: Vec3): Vec3 {
  const s = add(sketchNorm(n1), sketchNorm(n2));
  if (Math.hypot(...s) < 1e-9) return sketchNorm(n1);
  return sketchNorm(s);
}

/**
 * Unique circle through 3 non-collinear points in 3D.
 * Returns dense samples along the shorter arc A→B→C (or full when requested).
 */
export function circleThrough3Points(
  a: Vec3,
  b: Vec3,
  c: Vec3,
  samples = 24,
): Vec3[] | null {
  const ab = sub(b, a);
  const ac = sub(c, a);
  const n = [
    ab[1] * ac[2] - ab[2] * ac[1],
    ab[2] * ac[0] - ab[0] * ac[2],
    ab[0] * ac[1] - ab[1] * ac[0],
  ] as Vec3;
  if (Math.hypot(...n) < 1e-10) return null; // collinear

  // Circumcentre in plane: solve for point equidistant from A,B,C.
  // Using perpendicular bisectors in the plane spanned by AB, AC.
  const ab2 = dot(ab, ab);
  const ac2 = dot(ac, ac);
  const abac = dot(ab, ac);
  const den = 2 * (ab2 * ac2 - abac * abac);
  if (Math.abs(den) < 1e-14) return null;
  const k1 = (ab2 * ac2 - ac2 * abac) / den;
  const k2 = (ac2 * ab2 - ab2 * abac) / den;
  const center = add(a, add(scale(ab, k1), scale(ac, k2)));
  const r = dist(center, a);
  if (!(r > 1e-6) || !Number.isFinite(r)) return null;

  const nn = sketchNorm(n);
  // Orthonormal basis in plane.
  let u = sketchNorm(sub(a, center));
  let v = [
    nn[1] * u[2] - nn[2] * u[1],
    nn[2] * u[0] - nn[0] * u[2],
    nn[0] * u[1] - nn[1] * u[0],
  ] as Vec3;
  v = sketchNorm(v);

  const angleOf = (p: Vec3) => {
    const d = sketchNorm(sub(p, center));
    return Math.atan2(dot(d, v), dot(d, u));
  };
  let a0 = angleOf(a);
  let a1 = angleOf(b);
  let a2 = angleOf(c);
  // Unwrap so a0 → a1 → a2 is monotonic short path.
  const unwrap = (prev: number, next: number) => {
    let x = next;
    while (x - prev > Math.PI) x -= 2 * Math.PI;
    while (x - prev < -Math.PI) x += 2 * Math.PI;
    return x;
  };
  a1 = unwrap(a0, a1);
  a2 = unwrap(a1, a2);

  const out: Vec3[] = [];
  const nSamp = Math.max(3, samples);
  for (let i = 0; i < nSamp; i++) {
    const t = i / (nSamp - 1);
    // Quadratic Bezier in angle space through a0,a1,a2 (parametric by chord).
    const ang =
      (1 - t) * (1 - t) * a0 + 2 * (1 - t) * t * a1 + t * t * a2;
    out.push(
      add(center, add(scale(u, Math.cos(ang) * r), scale(v, Math.sin(ang) * r))),
    );
  }
  // Force exact endpoints / mid.
  out[0] = [...a] as Vec3;
  out[nSamp - 1] = [...c] as Vec3;
  const midIdx = Math.floor(nSamp / 2);
  out[midIdx] = [...b] as Vec3;
  return out;
}

/** Uniform Catmull-Rom samples through control points (centripetal α=0.5). */
export function sampleCatmullRom(controls: Vec3[], samplesPerSeg = 8): Vec3[] {
  if (controls.length === 0) return [];
  if (controls.length === 1) return [[...controls[0]] as Vec3];
  if (controls.length === 2) {
    const out: Vec3[] = [];
    for (let i = 0; i <= samplesPerSeg; i++) {
      const t = i / samplesPerSeg;
      out.push(add(scale(controls[0], 1 - t), scale(controls[1], t)));
    }
    return out;
  }
  const pts = [
    controls[0],
    ...controls,
    controls[controls.length - 1],
  ];
  const out: Vec3[] = [];
  for (let i = 1; i < pts.length - 2; i++) {
    const p0 = pts[i - 1];
    const p1 = pts[i];
    const p2 = pts[i + 1];
    const p3 = pts[i + 2];
    const n = i === pts.length - 3 ? samplesPerSeg : samplesPerSeg;
    for (let j = 0; j < n; j++) {
      const t = j / samplesPerSeg;
      const t2 = t * t;
      const t3 = t2 * t;
      out.push([
        0.5 *
          (2 * p1[0] +
            (-p0[0] + p2[0]) * t +
            (2 * p0[0] - 5 * p1[0] + 4 * p2[0] - p3[0]) * t2 +
            (-p0[0] + 3 * p1[0] - 3 * p2[0] + p3[0]) * t3),
        0.5 *
          (2 * p1[1] +
            (-p0[1] + p2[1]) * t +
            (2 * p0[1] - 5 * p1[1] + 4 * p2[1] - p3[1]) * t2 +
            (-p0[1] + 3 * p1[1] - 3 * p2[1] + p3[1]) * t3),
        0.5 *
          (2 * p1[2] +
            (-p0[2] + p2[2]) * t +
            (2 * p0[2] - 5 * p1[2] + 4 * p2[2] - p3[2]) * t2 +
            (-p0[2] + 3 * p1[2] - 3 * p2[2] + p3[2]) * t3),
      ]);
    }
  }
  out.push([...controls[controls.length - 1]] as Vec3);
  return out;
}

/** Torch wire into the joint ≈ −surface / bisector normal. */
export function torchDirFromNormal(normal: Vec3): Vec3 {
  const n = sketchNorm(normal);
  // Prefer into-work; if normal points mostly up, tip slightly toward −Y/−Z.
  const into: Vec3 = [-n[0], -n[1], -n[2]];
  if (Math.abs(into[1]) < 0.15 && Math.abs(into[2]) < 0.15) {
    return sketchNorm([into[0], -0.5, -0.5]);
  }
  return sketchNorm(into);
}

export function modeNeedsPoints(mode: SketchMode): number {
  switch (mode) {
    case "tack":
      return 1;
    case "line":
      return 2;
    case "arc":
      return 3;
    case "spline":
      return 2; // minimum; more allowed
    default:
      return 0;
  }
}
