"use client";

import { Line } from "@react-three/drei";
import { sampleProgram, type WeldProgram } from "@/lib/weldProgram";

/**
 * Renders the demo workpiece, the highlighted weld seam, approach/retract air
 * moves (dashed), torch-orientation frames, and an animated TCP/torch marker
 * driven by `simT` (0..1 program progress).
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
      <Line points={program.seam} color="#ff7a1a" lineWidth={4} />

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
        const tip: [number, number, number] = [
          frm.pos[0] - frm.dir[0] * 0.13,
          frm.pos[1] - frm.dir[1] * 0.13,
          frm.pos[2] - frm.dir[2] * 0.13,
        ];
        return <Line key={i} points={[frm.pos, tip]} color="#ffd400" lineWidth={1.5} />;
      })}

      {/* Animated TCP / torch marker */}
      <group position={s.pos}>
        <mesh>
          <sphereGeometry args={[s.arcOn ? 0.03 : 0.02, 16, 16]} />
          <meshStandardMaterial
            color={s.arcOn ? "#fff2b0" : "#9aa4b2"}
            emissive={s.arcOn ? "#ff8a00" : "#000000"}
            emissiveIntensity={s.arcOn ? 1.6 : 0}
          />
        </mesh>
        {s.arcOn && <pointLight color="#ff9a3c" intensity={2.5} distance={0.7} />}
      </group>
    </group>
  );
}
