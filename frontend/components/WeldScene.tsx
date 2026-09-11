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
  const approach = program.waypoints[0].pos;
  const retract = program.waypoints[program.waypoints.length - 1].pos;
  const seamStart = program.seam[0];
  const seamEnd = program.seam[program.seam.length - 1];

  // Weld-phase progress fraction (excludes the approach/retract air moves).
  const approachDur = program.segDurations[0] ?? 0;
  const retractDur = program.segDurations[program.segDurations.length - 1] ?? 0;
  const weldDur = Math.max(1e-6, program.cycleSec - approachDur - retractDur);
  const f = Math.max(0, Math.min(1, (s.elapsedSec - approachDur) / weldDur));
  const bead = beadPoints(program.seam, f);

  return (
    <group>
      {/* Workpiece plates (T-fillet coupon) */}
      {program.plates.map((pl, i) => (
        <mesh key={i} position={pl.center} castShadow receiveShadow>
          <boxGeometry args={pl.size} />
          <meshStandardMaterial color={0x8a939f} metalness={0.35} roughness={0.55} />
        </mesh>
      ))}

      {/* Weld seam — solid orange line on the geometry */}
      <Line points={program.seam} color="#ff7a1a" lineWidth={5} />

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

      {/* Approach / retract — dashed cyan air-moves */}
      <Line
        points={[approach, seamStart]}
        color="#39d0ff"
        lineWidth={2}
        dashed
        dashSize={0.03}
        gapSize={0.02}
      />
      <Line
        points={[seamEnd, retract]}
        color="#39d0ff"
        lineWidth={2}
        dashed
        dashSize={0.03}
        gapSize={0.02}
      />

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
