"use client";

import { useRef } from "react";
import { useFrame } from "@react-three/fiber";
import { Line } from "@react-three/drei";
import * as THREE from "three";
import { sampleProgram, type Vec3, type WeldProgram } from "@/lib/weldProgram";

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

/**
 * Renders the demo workpiece, the highlighted weld seam, approach/retract air
 * moves (dashed), torch-orientation frames, a growing weld bead, and an
 * animated arc at the torch tip — driven by `simT` (0..1 program progress).
 */
export default function WeldScene({
  program,
  simT,
}: {
  program: WeldProgram;
  simT: number;
}) {
  const s = sampleProgram(program, simT);
  const seams = program.welds?.length
    ? program.welds.map((w) => w.seam)
    : program.seam.length
      ? [program.seam]
      : [];
  const primarySeam = seams[0] ?? program.seam;

  // Weld-phase progress fraction (excludes air / home / touch).
  const weldDurations = program.waypoints.slice(0, -1).map((wp, i) =>
    wp.kind === "weld" && program.waypoints[i + 1]?.kind === "weld"
      ? program.segDurations[i] ?? 0
      : 0,
  );
  const weldDur = Math.max(1e-6, weldDurations.reduce((a, b) => a + b, 0));
  let weldElapsed = 0;
  let acc = 0;
  const target = s.elapsedSec;
  for (let i = 0; i < program.segDurations.length; i++) {
    const d = program.segDurations[i];
    const isWeld =
      program.waypoints[i]?.kind === "weld" && program.waypoints[i + 1]?.kind === "weld";
    if (target >= acc + d) {
      if (isWeld) weldElapsed += d;
    } else if (isWeld) {
      weldElapsed += Math.max(0, target - acc);
      break;
    } else if (target < acc + d) {
      break;
    }
    acc += d;
  }
  const f = Math.max(0, Math.min(1, weldElapsed / weldDur));
  const bead = beadPoints(primarySeam, f);

  const airRuns: Vec3[][] = [];
  let run: Vec3[] = [];
  const flush = () => {
    if (run.length >= 2) airRuns.push(run);
    run = [];
  };
  program.waypoints.forEach((wp) => {
    if (wp.kind === "weld") {
      flush();
      return;
    }
    run.push(wp.pos);
  });
  flush();

  return (
    <group>
      {/* Workpiece plates (T-fillet coupon) */}
      {program.plates.map((pl, i) => (
        <mesh key={i} position={pl.center} castShadow receiveShadow>
          <boxGeometry args={pl.size} />
          <meshStandardMaterial color={0x8a939f} metalness={0.35} roughness={0.55} />
        </mesh>
      ))}

      {/* All identified seams — Weld 1-N */}
      {seams.map((seam, i) => (
        <Line key={`seam-${i}`} points={seam} color="#ff7a1a" lineWidth={5} />
      ))}

      {/* Home (action 1) */}
      {program.waypoints
        .filter((w) => w.kind === "home")
        .slice(0, 1)
        .map((w) => (
          <mesh key={w.id} position={w.pos}>
            <octahedronGeometry args={[0.025, 0]} />
            <meshStandardMaterial color="#64748b" emissive="#334155" emissiveIntensity={0.35} />
          </mesh>
        ))}

      {/* TouchSense search points */}
      {program.waypoints
        .filter((w) => w.kind === "sense")
        .map((w) => (
          <mesh key={w.id} position={w.pos}>
            <sphereGeometry args={[0.012, 10, 10]} />
            <meshStandardMaterial color="#2563eb" emissive="#1d4ed8" emissiveIntensity={0.5} />
          </mesh>
        ))}


      {/* Growing weld bead over the welded portion of the seam */}
      {bead.map((p, i) => (
        <mesh key={i} position={p}>
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

      {/* Home ↔ touch ↔ weld air-moves (dashed) */}
      {airRuns.map((pts, i) => (
        <Line
          key={`air-${i}`}
          points={pts}
          color="#39d0ff"
          lineWidth={2}
          dashed
          dashSize={0.03}
          gapSize={0.02}
        />
      ))}

      {/* Torch-orientation frames along the seam */}
      {program.torchFrames.map((frm, i) => {
        const tip: Vec3 = [
          frm.pos[0] - frm.dir[0] * 0.13,
          frm.pos[1] - frm.dir[1] * 0.13,
          frm.pos[2] - frm.dir[2] * 0.13,
        ];
        return <Line key={i} points={[frm.pos, tip]} color="#ffd400" lineWidth={1.5} />;
      })}

      {/* Animated arc at the torch tip (only while the arc is on) */}
      {s.arcOn && <Arc pos={s.pos} />}
    </group>
  );
}
