"use client";

/**
 * Example MIG/MAG welding torch tool, modelled along local +X with the contact
 * tip at [tipLen, 0, 0]. Mount it on the robot wrist so +X points where the
 * torch should aim. Shows an arc/spark glow at the tip when `arcOn`.
 */
export default function WeldingTorch({
  arcOn = false,
  tipLen = 0.3,
}: {
  arcOn?: boolean;
  tipLen?: number;
}) {
  return (
    <group>
      {/* Cable stub feeding into the back of the torch */}
      <mesh position={[-0.08, 0, 0]} rotation={[0, 0, Math.PI / 2]}>
        <cylinderGeometry args={[0.025, 0.025, 0.16, 16]} />
        <meshStandardMaterial color={0x111418} roughness={0.9} />
      </mesh>

      {/* Torch body / handle */}
      <mesh position={[tipLen * 0.35, 0, 0]} rotation={[0, 0, Math.PI / 2]} castShadow>
        <cylinderGeometry args={[0.032, 0.036, tipLen * 0.55, 20]} />
        <meshStandardMaterial color={0x1f2937} metalness={0.4} roughness={0.5} />
      </mesh>

      {/* Neck */}
      <mesh position={[tipLen * 0.72, 0, 0]} rotation={[0, 0, Math.PI / 2]} castShadow>
        <cylinderGeometry args={[0.018, 0.02, tipLen * 0.32, 16]} />
        <meshStandardMaterial color={0x9aa4b2} metalness={0.6} roughness={0.35} />
      </mesh>

      {/* Gas nozzle (points +X toward the tip) */}
      <mesh position={[tipLen * 0.9, 0, 0]} rotation={[0, 0, -Math.PI / 2]} castShadow>
        <cylinderGeometry args={[0.026, 0.03, tipLen * 0.2, 20]} />
        <meshStandardMaterial color={0xb87333} metalness={0.7} roughness={0.4} />
      </mesh>

      {/* Arc / spark at the contact tip */}
      {arcOn && (
        <group position={[tipLen, 0, 0]}>
          <mesh>
            <sphereGeometry args={[0.03, 16, 16]} />
            <meshStandardMaterial
              color="#fff2b0"
              emissive="#ff8a00"
              emissiveIntensity={2}
            />
          </mesh>
          <pointLight color="#ff9a3c" intensity={3} distance={0.8} />
          {[0.05, 0.08, 0.11].map((d, i) => (
            <mesh key={i} position={[0, -d, (i - 1) * 0.03]}>
              <sphereGeometry args={[0.006, 8, 8]} />
              <meshStandardMaterial
                color="#ffd27f"
                emissive="#ff7a1a"
                emissiveIntensity={1.5}
              />
            </mesh>
          ))}
        </group>
      )}
    </group>
  );
}
