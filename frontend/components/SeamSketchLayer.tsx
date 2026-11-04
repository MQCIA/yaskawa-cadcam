"use client";

import { useCallback, useMemo, useRef, useState } from "react";
import { useThree, type ThreeEvent } from "@react-three/fiber";
import { Line } from "@react-three/drei";
import * as THREE from "three";
import type { Vec3, WeldProgram } from "@/lib/weldProgram";
import {
  bisectorNormal,
  circleThrough3Points,
  sampleCatmullRom,
  snapToTriangle,
  torchDirFromNormal,
  type SketchHit,
  type SketchMode,
  SNAP_RADIUS_M,
} from "@/lib/seamSketch";

export type DraftPoint = {
  point: Vec3;
  normal: Vec3;
  snap: SketchHit["snap"];
};

type Props = {
  program: WeldProgram;
  mode: SketchMode;
  snapOn: boolean;
  draft: DraftPoint[];
  onDraftChange: (pts: DraftPoint[]) => void;
  onCommit: (pts: DraftPoint[]) => void;
  /** Disable OrbitControls while drawing on the mesh. */
  onDrawingHover?: (active: boolean) => void;
};

function worldTriFromFace(
  mesh: THREE.Mesh,
  face: THREE.Face,
  point: THREE.Vector3,
): { tri: [Vec3, Vec3, Vec3]; normal: Vec3 } | null {
  const geom = mesh.geometry as THREE.BufferGeometry;
  const pos = geom.attributes.position as THREE.BufferAttribute | undefined;
  if (!pos) return null;
  mesh.updateWorldMatrix(true, false);
  const a = new THREE.Vector3().fromBufferAttribute(pos, face.a);
  const b = new THREE.Vector3().fromBufferAttribute(pos, face.b);
  const c = new THREE.Vector3().fromBufferAttribute(pos, face.c);
  a.applyMatrix4(mesh.matrixWorld);
  b.applyMatrix4(mesh.matrixWorld);
  c.applyMatrix4(mesh.matrixWorld);
  const nLocal = face.normal.clone();
  const nMat = new THREE.Matrix3().getNormalMatrix(mesh.matrixWorld);
  nLocal.applyMatrix3(nMat).normalize();
  // Prefer geometric normal if buffer normal is degenerate.
  if (nLocal.lengthSq() < 1e-8) {
    const e1 = new THREE.Vector3().subVectors(b, a);
    const e2 = new THREE.Vector3().subVectors(c, a);
    nLocal.crossVectors(e1, e2).normalize();
  }
  void point;
  return {
    tri: [
      [a.x, a.y, a.z],
      [b.x, b.y, b.z],
      [c.x, c.y, c.z],
    ],
    normal: [nLocal.x, nLocal.y, nLocal.z],
  };
}

function Crosshair({
  hit,
}: {
  hit: SketchHit | null;
}) {
  if (!hit) return null;
  const col =
    hit.snap === "vertex" ? "#eab308" : hit.snap === "edge" ? "#22c55e" : "#38bdf8";
  const dir = torchDirFromNormal(hit.normal);
  const tip: Vec3 = [
    hit.point[0] + dir[0] * 0.09,
    hit.point[1] + dir[1] * 0.09,
    hit.point[2] + dir[2] * 0.09,
  ];
  return (
    <group>
      <mesh position={hit.point}>
        <sphereGeometry args={[hit.snap === "none" ? 0.008 : 0.011, 12, 12]} />
        <meshStandardMaterial color={col} emissive={col} emissiveIntensity={0.7} />
      </mesh>
      <Line points={[hit.point, tip]} color="#16a34a" lineWidth={2} />
      {/* Snap ring */}
      <mesh position={hit.point} rotation={[Math.PI / 2, 0, 0]}>
        <ringGeometry args={[0.014, 0.018, 24]} />
        <meshBasicMaterial color={col} transparent opacity={0.85} side={THREE.DoubleSide} />
      </mesh>
    </group>
  );
}

/** Pickable CAD solids + rubber-band preview for the seam sketcher. */
export default function SeamSketchLayer({
  program,
  mode,
  snapOn,
  draft,
  onDraftChange,
  onCommit,
  onDrawingHover,
}: Props) {
  const { camera, gl } = useThree();
  const raycaster = useMemo(() => new THREE.Raycaster(), []);
  const pointer = useMemo(() => new THREE.Vector2(), []);
  const meshRefs = useRef<THREE.Mesh[]>([]);
  const [hover, setHover] = useState<SketchHit | null>(null);
  const lastClick = useRef(0);

  const drawing = mode !== "orbit";

  const register = useCallback((el: THREE.Mesh | null, index: number) => {
    if (el) meshRefs.current[index] = el;
  }, []);

  const pick = useCallback(
    (clientX: number, clientY: number): SketchHit | null => {
      const rect = gl.domElement.getBoundingClientRect();
      pointer.x = ((clientX - rect.left) / rect.width) * 2 - 1;
      pointer.y = -((clientY - rect.top) / rect.height) * 2 + 1;
      raycaster.setFromCamera(pointer, camera);
      const meshes = meshRefs.current.filter(Boolean);
      const hits = raycaster.intersectObjects(meshes, false);
      if (!hits.length) return null;
      const h = hits[0];
      if (!h.face || !(h.object instanceof THREE.Mesh)) return null;
      const world = worldTriFromFace(h.object, h.face, h.point);
      if (!world) return null;
      const raw: Vec3 = [h.point.x, h.point.y, h.point.z];
      let snapped = snapToTriangle(
        raw,
        world.tri,
        world.normal,
        SNAP_RADIUS_M,
        snapOn,
      );
      // Fillet / sharp edge: if a second solid also lies near the snap point,
      // use the bisector of the two face normals (45° into a T-joint).
      if (snapOn && (snapped.snap === "edge" || snapped.snap === "vertex")) {
        for (let i = 1; i < Math.min(hits.length, 4); i++) {
          const h2 = hits[i];
          if (!h2.face || !(h2.object instanceof THREE.Mesh)) continue;
          if (h2.object === h.object) continue;
          const d = h2.point.distanceTo(h.point);
          if (d > SNAP_RADIUS_M * 2.5) continue;
          const w2 = worldTriFromFace(h2.object, h2.face, h2.point);
          if (!w2) continue;
          snapped = {
            ...snapped,
            normal: bisectorNormal(snapped.normal, w2.normal),
          };
          break;
        }
      }
      return snapped;
    },
    [camera, gl, pointer, raycaster, snapOn],
  );

  const onPointerMove = useCallback(
    (e: ThreeEvent<PointerEvent>) => {
      if (!drawing) {
        setHover(null);
        onDrawingHover?.(false);
        return;
      }
      e.stopPropagation();
      const hit = pick(e.clientX, e.clientY);
      setHover(hit);
      onDrawingHover?.(Boolean(hit));
      if (hit) {
        document.body.style.cursor = "crosshair";
      }
    },
    [drawing, onDrawingHover, pick],
  );

  const onPointerOut = useCallback(() => {
    setHover(null);
    onDrawingHover?.(false);
    document.body.style.cursor = "auto";
  }, [onDrawingHover]);

  const onPointerDown = useCallback(
    (e: ThreeEvent<PointerEvent>) => {
      if (!drawing) return;
      // Left button / primary touch only.
      if (e.button !== 0) return;
      e.stopPropagation();
      const hit = pick(e.clientX, e.clientY);
      if (!hit) return;

      const now = performance.now();
      const dbl = now - lastClick.current < 320;
      lastClick.current = now;

      const next: DraftPoint[] = [
        ...draft,
        { point: hit.point, normal: hit.normal, snap: hit.snap },
      ];

      if (mode === "tack") {
        onCommit(next);
        onDraftChange([]);
        return;
      }
      if (mode === "line" && next.length >= 2) {
        onCommit(next.slice(0, 2));
        onDraftChange([]);
        return;
      }
      if (mode === "arc" && next.length >= 3) {
        onCommit(next.slice(0, 3));
        onDraftChange([]);
        return;
      }
      if (mode === "spline") {
        if (dbl && next.length >= 2) {
          // Double-click finishes (last click duplicated — drop the duplicate).
          onCommit(next.slice(0, -1));
          onDraftChange([]);
          return;
        }
        onDraftChange(next);
        return;
      }
      onDraftChange(next);
    },
    [draft, drawing, mode, onCommit, onDraftChange, pick],
  );

  const previewLine = useMemo(() => {
    if (!drawing) return null;
    const pts = draft.map((d) => d.point);
    if (hover) pts.push(hover.point);
    if (pts.length < 2) return null;
    if (mode === "arc" && draft.length === 2 && hover) {
      const circ = circleThrough3Points(draft[0].point, draft[1].point, hover.point, 28);
      return circ;
    }
    if (mode === "spline" && draft.length >= 2) {
      return sampleCatmullRom(pts, 6);
    }
    return pts;
  }, [draft, drawing, hover, mode]);

  const plateCount = program.plates.length;
  const pipeCount = program.pipes?.length ?? 0;

  return (
    <group
      onPointerMove={onPointerMove}
      onPointerOut={onPointerOut}
      onPointerDown={onPointerDown}
    >
      {program.plates.map((pl, i) => (
        <mesh
          key={`sk-pl-${i}`}
          ref={(el) => register(el, i)}
          position={pl.center}
          // Slightly expanded hit volume so thin plates are easier to pick.
          userData={{ sketch: true }}
        >
          <boxGeometry args={pl.size} />
          <meshStandardMaterial
            color={0x8a939f}
            metalness={0.35}
            roughness={0.55}
            transparent
            opacity={drawing ? 0.92 : 1}
          />
        </mesh>
      ))}

      {program.pipes?.map((pp, i) => (
        <mesh
          key={`sk-pipe-${i}`}
          ref={(el) => register(el, plateCount + i)}
          position={pp.center}
          rotation={pp.rotation}
        >
          <cylinderGeometry args={[pp.radius, pp.radius, pp.length, 48]} />
          <meshStandardMaterial
            color={0x7a8490}
            metalness={0.45}
            roughness={0.4}
            transparent
            opacity={drawing ? 0.92 : 1}
          />
        </mesh>
      ))}

      {/* Hide original solids? WeldScene still draws them — skip duplicate when
          sketcher owns the meshes. Parent should pass skipSolids. */}
      {void pipeCount}

      {draft.map((d, i) => (
        <mesh key={`draft-${i}`} position={d.point}>
          <sphereGeometry args={[0.01, 12, 12]} />
          <meshStandardMaterial
            color="#e87722"
            emissive="#e87722"
            emissiveIntensity={0.55}
          />
        </mesh>
      ))}

      {previewLine && previewLine.length >= 2 && (
        <Line points={previewLine} color="#f97316" lineWidth={3} dashed dashSize={0.02} gapSize={0.012} />
      )}

      <Crosshair hit={hover} />
    </group>
  );
}
