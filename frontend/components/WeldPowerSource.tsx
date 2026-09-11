"use client";

import { Html } from "@react-three/drei";

/**
 * Procedural Lorch S8 welding power source ("spawarka") on a travelling dolly.
 *
 * In this cell it rides the same rail carriage as the robot, so it moves in
 * sync with the robot along the track. The model is an approximate stand-in
 * (anthracite cabinet + touch panel, a wire feeder + spool on top and a gas
 * cylinder alongside) — swap in real CAD once available.
 */
export default function WeldPowerSource({
  position = [0, 0, 0],
  label = "LORCH S8",
}: {
  position?: [number, number, number];
  label?: string;
}) {
  const body = 0x2a2f36; // anthracite cabinet
  const panel = 0x3b424b;
  const lorchBlue = 0x0072ce;
  const cabW = 0.42;
  const cabH = 0.72;
  const cabD = 0.5;
  const cabCy = cabH / 2 + 0.06; // cabinet centre height (sits on its skid)

  return (
    <group position={position}>
      {/* Skid / wheeled base */}
      <mesh position={[0, 0.05, 0]} castShadow receiveShadow>
        <boxGeometry args={[cabW + 0.04, 0.1, cabD + 0.04]} />
        <meshStandardMaterial color={0x14181d} metalness={0.3} roughness={0.7} />
      </mesh>
      {[
        [cabW / 2 - 0.03, cabD / 2 - 0.05],
        [-(cabW / 2 - 0.03), cabD / 2 - 0.05],
        [cabW / 2 - 0.03, -(cabD / 2 - 0.05)],
        [-(cabW / 2 - 0.03), -(cabD / 2 - 0.05)],
      ].map(([x, z], i) => (
        <mesh key={i} position={[x, 0.03, z]} rotation={[Math.PI / 2, 0, 0]} castShadow>
          <cylinderGeometry args={[0.035, 0.035, 0.05, 20]} />
          <meshStandardMaterial color={0x0c0e11} metalness={0.4} roughness={0.6} />
        </mesh>
      ))}

      {/* Main cabinet */}
      <mesh position={[0, cabCy, 0]} castShadow receiveShadow>
        <boxGeometry args={[cabW, cabH, cabD]} />
        <meshStandardMaterial color={body} metalness={0.35} roughness={0.55} />
      </mesh>

      {/* Blue Lorch accent stripe near the top */}
      <mesh position={[cabW / 2 + 0.001, cabCy + cabH / 2 - 0.11, 0]}>
        <boxGeometry args={[0.004, 0.09, cabD * 0.96]} />
        <meshStandardMaterial color={lorchBlue} emissive={lorchBlue} emissiveIntensity={0.25} />
      </mesh>

      {/* Front control panel (faces +X, toward the robot / camera) */}
      <mesh position={[cabW / 2 + 0.002, cabCy + 0.05, 0]}>
        <boxGeometry args={[0.01, 0.34, 0.34]} />
        <meshStandardMaterial color={panel} metalness={0.2} roughness={0.6} />
      </mesh>
      {/* Colour touch display */}
      <mesh position={[cabW / 2 + 0.009, cabCy + 0.12, 0]}>
        <boxGeometry args={[0.006, 0.14, 0.22]} />
        <meshStandardMaterial
          color={0x0a2a3a}
          emissive={0x24c6e6}
          emissiveIntensity={0.9}
        />
      </mesh>
      {/* Two rotary encoders / knobs */}
      {[-0.07, 0.07].map((z) => (
        <mesh
          key={z}
          position={[cabW / 2 + 0.012, cabCy - 0.06, z]}
          rotation={[0, 0, Math.PI / 2]}
          castShadow
        >
          <cylinderGeometry args={[0.028, 0.028, 0.02, 20]} />
          <meshStandardMaterial color={0xced4da} metalness={0.5} roughness={0.4} />
        </mesh>
      ))}

      {/* Wire feeder unit on top */}
      <mesh position={[0, cabH + 0.14, 0]} castShadow>
        <boxGeometry args={[cabW * 0.8, 0.16, cabD * 0.7]} />
        <meshStandardMaterial color={panel} metalness={0.3} roughness={0.55} />
      </mesh>
      {/* Wire spool (axis along X) */}
      <mesh
        position={[cabW * 0.34, cabH + 0.23, 0]}
        rotation={[0, 0, Math.PI / 2]}
        castShadow
      >
        <cylinderGeometry args={[0.11, 0.11, 0.08, 28]} />
        <meshStandardMaterial color={0xdfe4ea} metalness={0.1} roughness={0.7} />
      </mesh>

      {/* Gas cylinder strapped to the side (-Z) */}
      <group position={[0, 0, -(cabD / 2 + 0.13)]}>
        <mesh position={[0, 0.42, 0]} castShadow>
          <cylinderGeometry args={[0.1, 0.1, 0.84, 28]} />
          <meshStandardMaterial color={0x2f7d32} metalness={0.25} roughness={0.6} />
        </mesh>
        <mesh position={[0, 0.9, 0]} castShadow>
          <cylinderGeometry args={[0.045, 0.06, 0.14, 20]} />
          <meshStandardMaterial color={0x8a8f96} metalness={0.5} roughness={0.4} />
        </mesh>
      </group>

      {/* Branding badge */}
      <Html
        position={[cabW / 2 + 0.02, cabCy + 0.3, 0]}
        center
        distanceFactor={7}
        occlude={false}
      >
        <div className="whitespace-nowrap rounded bg-[#0072ce] px-1.5 py-0.5 text-[11px] font-bold tracking-wide text-white shadow">
          {label}
        </div>
      </Html>
    </group>
  );
}
