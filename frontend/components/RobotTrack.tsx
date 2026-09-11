"use client";

/**
 * Procedural Yaskawa robot travel axis (rail / track).
 *
 * A dedicated linear rail the robot base rides on, running along Z. A carriage
 * slides along it (driven by `carriage`, in metres) and the robot mounts on top.
 * Proportions are approximate — swap in real CAD/URDF once available.
 */
export default function RobotTrack({
  length = 4.6,
  carriage = 0,
  color = 0x6b7683,
}: {
  length?: number; // rail length along Z
  carriage?: number; // carriage Z position (metres)
  color?: number;
}) {
  const railX = 0.32;
  const railTopY = 0.12;
  const bedW = 1.75; // widened so the power-source dolly rides on the same bed
  const carriageTopY = railTopY + 0.09;
  const dollyX = -0.62; // power-source dolly sits behind the robot (-X)

  return (
    <group>
      {/* Bed / base plate */}
      <mesh position={[0, 0.03, 0]} receiveShadow>
        <boxGeometry args={[bedW, 0.06, length]} />
        <meshStandardMaterial color={0x141b23} />
      </mesh>

      {/* Two guide rails under the robot carriage */}
      {[railX, -railX].map((x) => (
        <mesh key={x} position={[x, railTopY, 0]} castShadow receiveShadow>
          <boxGeometry args={[0.1, 0.08, length]} />
          <meshStandardMaterial color={0xb8c0c9} metalness={0.6} roughness={0.35} />
        </mesh>
      ))}
      {/* Guide rail under the power-source dolly */}
      <mesh position={[dollyX, railTopY, 0]} castShadow receiveShadow>
        <boxGeometry args={[0.1, 0.08, length]} />
        <meshStandardMaterial color={0xb8c0c9} metalness={0.6} roughness={0.35} />
      </mesh>

      {/* Rack / energy chain along one side */}
      <mesh position={[bedW / 2 - 0.05, 0.1, 0]} castShadow>
        <boxGeometry args={[0.06, 0.09, length]} />
        <meshStandardMaterial color={0x1b2733} />
      </mesh>

      {/* Moving carriage (robot + coupled power-source dolly move together) */}
      <group position={[0, 0, carriage]}>
        {/* Robot carriage */}
        <mesh position={[0, carriageTopY, 0]} castShadow receiveShadow>
          <boxGeometry args={[0.8, 0.1, 0.72]} />
          <meshStandardMaterial color={color} metalness={0.3} roughness={0.6} />
        </mesh>
        {/* Coupling bar to the trailing dolly */}
        <mesh position={[(dollyX - 0.25) / 2 - 0.05, carriageTopY, 0]} castShadow>
          <boxGeometry args={[0.35, 0.06, 0.12]} />
          <meshStandardMaterial color={0x1b2733} metalness={0.3} roughness={0.6} />
        </mesh>
        {/* Trailing dolly platform for the welding power source */}
        <mesh position={[dollyX, carriageTopY, 0]} castShadow receiveShadow>
          <boxGeometry args={[0.6, 0.1, 0.72]} />
          <meshStandardMaterial color={color} metalness={0.3} roughness={0.6} />
        </mesh>
      </group>
    </group>
  );
}
