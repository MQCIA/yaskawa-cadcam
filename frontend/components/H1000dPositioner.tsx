"use client";

const deg2rad = (d: number) => (d * Math.PI) / 180;

/**
 * Procedural H1000D welding positioner.
 *
 * Single-axis, horizontal rotating headstock, ~1000 kg class (axis height
 * ~0.99 m, ±360° faceplate rotation). Proportions are approximate — swap in a
 * real CAD/URDF once available (see docs/MODELS.md).
 *
 * The faceplate (where the workpiece mounts) spins about the horizontal X axis
 * driven by `rotateDeg`.
 */
export default function H1000dPositioner({
  rotateDeg = 0,
  position = [0, 0, 0],
  color = 0x9aa4b2,
}: {
  rotateDeg?: number;
  position?: [number, number, number];
  color?: number;
}) {
  const axisHeight = 0.99;
  const accent = 0x00a3e0;

  return (
    <group position={position}>
      {/* Floor base */}
      <mesh position={[0, 0.06, 0]} castShadow receiveShadow>
        <boxGeometry args={[0.7, 0.12, 0.8]} />
        <meshStandardMaterial color={0x1b2733} />
      </mesh>

      {/* Pedestal column up to the rotation axis */}
      <mesh position={[0.12, axisHeight / 2, 0]} castShadow>
        <boxGeometry args={[0.42, axisHeight, 0.55]} />
        <meshStandardMaterial color={color} metalness={0.2} roughness={0.7} />
      </mesh>

      {/* Motor / bearing housing, axis horizontal along X */}
      <mesh position={[-0.16, axisHeight, 0]} rotation={[0, 0, Math.PI / 2]} castShadow>
        <cylinderGeometry args={[0.19, 0.19, 0.34, 32]} />
        <meshStandardMaterial color={color} metalness={0.3} roughness={0.6} />
      </mesh>

      {/* Rotating faceplate group (spins about the horizontal X axis) */}
      <group position={[-0.34, axisHeight, 0]} rotation={[deg2rad(rotateDeg), 0, 0]}>
        {/* Faceplate disc, axis along X */}
        <mesh rotation={[0, 0, Math.PI / 2]} castShadow>
          <cylinderGeometry args={[0.34, 0.34, 0.06, 48]} />
          <meshStandardMaterial color={0xc0c0c0} metalness={0.7} roughness={0.35} />
        </mesh>
        {/* Chuck cross-spokes spanning the diameter (protrude both faces so the
            rotation is clearly visible from any camera angle) */}
        <mesh position={[0, 0, 0]} castShadow>
          <boxGeometry args={[0.1, 0.64, 0.05]} />
          <meshStandardMaterial color={accent} emissive={accent} emissiveIntensity={0.35} />
        </mesh>
        <mesh position={[0, 0, 0]} castShadow>
          <boxGeometry args={[0.1, 0.05, 0.64]} />
          <meshStandardMaterial color={0x33404d} metalness={0.3} roughness={0.6} />
        </mesh>
        {/* Single highlighted jaw at the rim so the zero mark is identifiable */}
        <mesh position={[0, 0.3, 0]} castShadow>
          <boxGeometry args={[0.12, 0.1, 0.1]} />
          <meshStandardMaterial color={0xffcc00} emissive={0xffcc00} emissiveIntensity={0.25} />
        </mesh>
        {/* Central mounting boss */}
        <mesh rotation={[0, 0, Math.PI / 2]} position={[0, 0, 0]} castShadow>
          <cylinderGeometry args={[0.09, 0.09, 0.14, 24]} />
          <meshStandardMaterial color={0x333a44} />
        </mesh>
      </group>
    </group>
  );
}
