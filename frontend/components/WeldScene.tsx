"use client";

import { useMemo, useRef } from "react";
import { useFrame } from "@react-three/fiber";
import { Line } from "@react-three/drei";
import * as THREE from "three";
import { sampleProgram, type Vec3, type WeldProgram } from "@/lib/weldProgram";
import {
  diagnoseWaypoints,
  informCLabel,
  weavePolyline,
  type WaypointDiag,
} from "@/lib/trajectoryViz";

const dist = (a: Vec3, b: Vec3) => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);

/** Points along the seam already welded, given weld progress fraction f. */
function beadPoints(seam: Vec3[], f: number): Vec3[] {
  const cum = [0];
  for (let i = 0; i < seam.length - 1; i++) cum.push(cum[i] + dist(seam[i], seam[i + 1]));
  const total = cum[cum.length - 1] || 1;
  const target = f * total;
  const out: Vec3[] = [];
  for (let i = 0; i < seam.length; i++) {
    if (cum[i] <= target) out.push(seam[i]);
  }
  return out;
}

/** Flickering arc light + molten-pool glow at the torch tip. */
function Arc({ pos }: { pos: Vec3 }) {
  const light = useRef<THREE.PointLight>(null);
  const core = useRef<THREE.Mesh>(null);
  useFrame(({ clock }) => {
    const flick = 0.7 + 0.3 * Math.sin(clock.elapsedTime * 45);
    if (light.current) light.current.intensity = 4 * flick;
    if (core.current) core.current.scale.setScalar(0.9 + 0.25 * flick);
  });
  return (
    <group position={pos}>
      <mesh ref={core}>
        <sphereGeometry args={[0.045, 16, 16]} />
        <meshStandardMaterial color="#ffffff" emissive="#ffb84d" emissiveIntensity={3} />
      </mesh>
      <mesh>
        <sphereGeometry args={[0.09, 16, 16]} />
        <meshStandardMaterial
          color="#ff8a00"
          emissive="#ff6a00"
          emissiveIntensity={1.4}
          transparent
          opacity={0.35}
        />
      </mesh>
      <pointLight ref={light} color="#ffb066" intensity={4} distance={1.1} />
    </group>
  );
}

/** Green cone + shaft showing wire / approach direction at a TCP. */
function WireArrow({
  pos,
  dir,
  length = 0.11,
}: {
  pos: Vec3;
  dir: Vec3;
  length?: number;
}) {
  const quat = useMemo(() => {
    const d = new THREE.Vector3(dir[0], dir[1], dir[2]).normalize();
    const q = new THREE.Quaternion();
    q.setFromUnitVectors(new THREE.Vector3(0, 1, 0), d);
    return q;
  }, [dir]);
  const tip: Vec3 = [
    pos[0] + dir[0] * length,
    pos[1] + dir[1] * length,
    pos[2] + dir[2] * length,
  ];
  return (
    <group>
      <Line points={[pos, tip]} color="#22c55e" lineWidth={2} />
      <mesh position={tip} quaternion={quat}>
        <coneGeometry args={[0.012, 0.028, 10]} />
        <meshStandardMaterial color="#16a34a" emissive="#14532d" emissiveIntensity={0.35} />
      </mesh>
    </group>
  );
}

function PathHit({
  pos,
  diag,
  selected,
  onSelect,
}: {
  pos: Vec3;
  diag: WaypointDiag;
  selected: boolean;
  onSelect: (i: number) => void;
}) {
  const color =
    diag.ik === "collision"
      ? "#ef4444"
      : diag.ik === "near_limit"
        ? "#f59e0b"
        : selected
          ? "#e87722"
          : "#64748b";
  return (
    <group position={pos}>
      <mesh
        onPointerOver={() => {
          document.body.style.cursor = "pointer";
        }}
        onPointerOut={() => {
          document.body.style.cursor = "auto";
        }}
        onClick={(e) => {
          e.stopPropagation();
          onSelect(diag.index);
        }}
      >
        <sphereGeometry args={[selected ? 0.018 : 0.011, 10, 10]} />
        <meshStandardMaterial
          color={color}
          emissive={color}
          emissiveIntensity={selected ? 0.55 : 0.2}
        />
      </mesh>
      {/* Finger-sized invisible hit target */}
      <mesh
        visible={false}
        onClick={(e) => {
          e.stopPropagation();
          onSelect(diag.index);
        }}
      >
        <sphereGeometry args={[0.045, 8, 8]} />
        <meshBasicMaterial />
      </mesh>
    </group>
  );
}

/**
 * Workpiece + full TCP trajectory with INFORM-style diagnostics:
 * red/orange weld (zigzag when WVON), dashed blue MOVJ, green wire arrows,
 * yellow TouchSense, hover tooltips with C# / V / BC / IK status.
 */
export default function WeldScene({
  program,
  simT,
  selectedWp = null,
  onSelectWp,
  weaveNo = 0,
}: {
  program: WeldProgram;
  simT: number;
  selectedWp?: number | null;
  onSelectWp?: (index: number) => void;
  /** Active WEV# — non-zero draws a zigzag weld polyline. */
  weaveNo?: number;
}) {
  const s = sampleProgram(program, simT);
  const diags = useMemo(() => diagnoseWaypoints(program), [program]);
  const activeWeave = weaveNo || program.meta?.activeWeaveNo || 0;

  const approachDur = program.segDurations[0] ?? 0;
  const retractDur = program.segDurations[program.segDurations.length - 1] ?? 0;
  const weldDur = Math.max(1e-6, program.cycleSec - approachDur - retractDur);
  const f = Math.max(0, Math.min(1, (s.elapsedSec - approachDur) / weldDur));
  const bead = beadPoints(program.seam, f);

  // Segment polylines by motion class.
  const weldRuns: Vec3[][] = [];
  const airRuns: Vec3[][] = [];
  {
    let run: Vec3[] = [];
    let mode: "weld" | "air" | null = null;
    const flush = () => {
      if (run.length >= 2) {
        if (mode === "weld") weldRuns.push(run);
        else if (mode === "air") airRuns.push(run);
      }
      run = [];
    };
    for (let i = 0; i < program.waypoints.length - 1; i++) {
      const a = program.waypoints[i];
      const b = program.waypoints[i + 1];
      const isWeld = a.arc && b.arc;
      const nextMode: "weld" | "air" = isWeld ? "weld" : "air";
      if (mode !== nextMode) {
        flush();
        mode = nextMode;
        run = [a.pos];
      }
      run.push(b.pos);
    }
    flush();
  }

  const select = onSelectWp ?? (() => undefined);

  return (
    <group>
      {program.plates.map((pl, i) => (
        <mesh key={`pl-${i}`} position={pl.center} castShadow receiveShadow>
          <boxGeometry args={pl.size} />
          <meshStandardMaterial color={0x8a939f} metalness={0.35} roughness={0.55} />
        </mesh>
      ))}

      {program.pipes?.map((pp, i) => (
        <mesh
          key={`pipe-${i}`}
          position={pp.center}
          rotation={pp.rotation}
          castShadow
          receiveShadow
        >
          <cylinderGeometry args={[pp.radius, pp.radius, pp.length, 48]} />
          <meshStandardMaterial color={0x7a8490} metalness={0.45} roughness={0.4} />
        </mesh>
      ))}

      {/* CAD seam highlight */}
      <Line points={program.seam} color="#fb923c" lineWidth={3} />

      {/* Weld TCP — thick orange/red; zigzag when weave active */}
      {weldRuns.map((pts, i) => {
        const drawn = activeWeave > 0 ? weavePolyline(pts, activeWeave === 21 ? 4 : 2.5) : pts;
        return (
          <Line
            key={`weld-${i}`}
            points={drawn}
            color={activeWeave > 0 ? "#ea580c" : "#dc2626"}
            lineWidth={activeWeave > 0 ? 4 : 5}
          />
        );
      })}

      {/* Air MOVJ / clearance — dashed blue */}
      {airRuns.map((pts, i) => (
        <Line
          key={`air-${i}`}
          points={pts}
          color="#2563eb"
          lineWidth={2}
          dashed
          dashSize={0.035}
          gapSize={0.022}
        />
      ))}

      {/* Growing weld bead */}
      {bead.map((p, i) => (
        <mesh key={`bead-${i}`} position={p}>
          <sphereGeometry args={[0.014, 10, 10]} />
          <meshStandardMaterial
            color="#ffb066"
            emissive="#ff6a00"
            emissiveIntensity={0.8}
            metalness={0.4}
            roughness={0.5}
          />
        </mesh>
      ))}

      {/* Yellow TouchSense points */}
      {program.waypoints.map((w, i) =>
        w.kind === "sense" || w.move === "MOVS" ? (
          <mesh key={`sense-${w.id}`} position={w.pos}>
            <sphereGeometry args={[0.018, 12, 12]} />
            <meshStandardMaterial
              color="#eab308"
              emissive="#ca8a04"
              emissiveIntensity={0.65}
            />
          </mesh>
        ) : null,
      )}

      {/* Green approach / retract wire arrows */}
      {program.waypoints.map((w, i) => {
        if (w.kind !== "approach" && w.kind !== "retract") return null;
        const frm =
          program.torchFrames[
            Math.max(0, Math.min(program.torchFrames.length - 1, i))
          ] ?? program.torchFrames[0];
        const dir = frm?.dir ?? [0, -0.7, -0.7];
        return <WireArrow key={`arr-${w.id}`} pos={w.pos} dir={dir as Vec3} />;
      })}

      {/* Torch frames along the seam (wire aim) */}
      {program.torchFrames.map((frm, i) => {
        if (i % Math.max(1, Math.floor(program.torchFrames.length / 8)) !== 0) return null;
        return <WireArrow key={`tf-${i}`} pos={frm.pos} dir={frm.dir} length={0.08} />;
      })}

      {/* Interactive TCP samples with INFORM tooltips */}
      {program.waypoints.map((w, i) => (
        <PathHit
          key={`hit-${w.id}`}
          pos={w.pos}
          diag={diags[i] ?? {
            index: i,
            cLabel: informCLabel(i),
            move: w.move,
            speedLabel: w.speedLabel,
            kind: w.kind,
            bc: w.bc,
            ik: "ok",
            ikLabel: "OK",
          }}
          selected={selectedWp === i}
          onSelect={select}
        />
      ))}

      {s.arcOn && <Arc pos={s.pos} />}
    </group>
  );
}
