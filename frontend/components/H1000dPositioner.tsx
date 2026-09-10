"use client";

const deg2rad = (d: number) => (d * Math.PI) / 180;

/**
 * Procedural H1000D welding positioner.
 *
 * Turn-over / rotary-table style: two identical support pedestals (head- and
 * tailstock) stand on either side, and a rectangular table-frame is mounted
 * between them on a common horizontal axis. The frame (where the workpiece is
 * clamped) rotates ±360° about that axis, driven by `rotateDeg`.
 *
 * Proportions are approximate — swap in real CAD/URDF once available
 * (see docs/MODELS.md).
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
  const axisHeight = 0.95; // height of the rotation axis
  const span = 1.7; // distance between the two pedestal centres
  const accent = 0x00a3e0;

  const frameLenX = span - 0.7; // table length along the rotation axis
  const frameDepthZ = 0.95; // table depth
  const bar = 0.07; // frame bar cross-section
  const barH = 0.06;

  return (
    <group position={position}>
      {/* Two identical support pedestals (head- and tailstock) */}
      {[-1, 1].map((sx) => (
        <group key={sx} position={[sx * (span / 2), 0, 0]}>
          {/* Floor base */}
          <mesh position={[0, 0.06, 0]} castShadow receiveShadow>
            <boxGeometry args={[0.5, 0.12, 0.85]} />
            <meshStandardMaterial color={0x1b2733} />
          </mesh>
          {/* Column up to the rotation axis */}
          <mesh position={[0, axisHeight / 2, 0]} castShadow>
            <boxGeometry args={[0.34, axisHeight, 0.6]} />
            <meshStandardMaterial color={color} metalness={0.2} roughness={0.7} />
          </mesh>
          {/* Bearing / drive housing on the axis, cylinder along X facing inward */}
          <mesh
            position={[-sx * 0.13, axisHeight, 0]}
            rotation={[0, 0, Math.PI / 2]}
            castShadow
          >
            <cylinderGeometry args={[0.18, 0.18, 0.26, 32]} />
            <meshStandardMaterial color={color} metalness={0.3} roughness={0.6} />
          </mesh>
        </group>
      ))}

      {/* Rotating table-frame between the pedestals (spins about horizontal X) */}
      <group position={[0, axisHeight, 0]} rotation={[deg2rad(rotateDeg), 0, 0]}>
        {/* Central axle linking both pedestals */}
        <mesh rotation={[0, 0, Math.PI / 2]} castShadow>
          <cylinderGeometry args={[0.05, 0.05, span, 24]} />
          <meshStandardMaterial color={0x333a44} metalness={0.5} roughness={0.4} />
        </mesh>

        {/* Rectangular table-frame (rama) — long bars along X */}
        {[frameDepthZ / 2, -frameDepthZ / 2].map((z) => (
          <mesh key={z} position={[0, 0, z]} castShadow>
            <boxGeometry args={[frameLenX, barH, bar]} />
            <meshStandardMaterial color={color} metalness={0.25} roughness={0.6} />
          </mesh>
        ))}
        {/* End bars along Z */}
        {[frameLenX / 2, -frameLenX / 2].map((x) => (
          <mesh key={x} position={[x, 0, 0]} castShadow>
            <boxGeometry args={[bar, barH, frameDepthZ]} />
            <meshStandardMaterial color={color} metalness={0.25} roughness={0.6} />
          </mesh>
        ))}
        {/* Central cross member (accent) so the rotation is clearly visible */}
        <mesh castShadow>
          <boxGeometry args={[bar, barH, frameDepthZ]} />
          <meshStandardMaterial
            color={accent}
            emissive={accent}
            emissiveIntensity={0.3}
          />
        </mesh>
        {/* Corner marker to identify the zero orientation */}
        <mesh
          position={[frameLenX / 2 - 0.06, 0.03, frameDepthZ / 2 - 0.06]}
          castShadow
        >
          <boxGeometry args={[0.1, 0.09, 0.1]} />
          <meshStandardMaterial
            color={0xffcc00}
            emissive={0xffcc00}
            emissiveIntensity={0.25}
          />
        </mesh>
      </group>
    </group>
  );
}
