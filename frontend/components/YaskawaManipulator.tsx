"use client";

import { useGLTF } from "@react-three/drei";
import type { Joints } from "./AxisSliders";

const deg = (d: number) => (d * Math.PI) / 180;

/**
 * Procedural stand-in for a Yaskawa AR/MA 6-axis welding arm.
 *
 * The links (base -> S -> L -> U -> R -> B -> T) are NESTED so each joint
 * rotates everything above it, forming a real kinematic chain. Proportions
 * are illustrative only.
 *
 * To use real geometry, drop per-link .glb files into /public/models and
 * replace each <mesh> with the <LinkGLB .../> pattern shown at the bottom.
 */
export default function YaskawaManipulator({ joints }: { joints: Joints }) {
  return (
    <group>
      {/* Base */}
      <mesh position={[0, 0.1, 0]} castShadow receiveShadow>
        <cylinderGeometry args={[0.35, 0.4, 0.2, 32]} />
        <meshStandardMaterial color="#1b2733" />
      </mesh>

      {/* S axis: rotation about vertical (Y) */}
      <group position={[0, 0.2, 0]} rotation={[0, deg(joints.S), 0]}>
        <mesh position={[0, 0.15, 0]} castShadow>
          <cylinderGeometry args={[0.28, 0.32, 0.3, 32]} />
          <meshStandardMaterial color="#0033a0" />
        </mesh>

        {/* L axis: shoulder pitch about Z, with a small X offset */}
        <group position={[0.12, 0.35, 0]} rotation={[0, 0, deg(joints.L)]}>
          <mesh position={[0, 0.35, 0]} castShadow>
            <boxGeometry args={[0.2, 0.75, 0.2]} />
            <meshStandardMaterial color="#0044c4" />
          </mesh>

          {/* U axis: elbow pitch about Z */}
          <group position={[0, 0.72, 0]} rotation={[0, 0, deg(joints.U)]}>
            <mesh position={[0.28, 0, 0]} castShadow>
              <boxGeometry args={[0.6, 0.16, 0.16]} />
              <meshStandardMaterial color="#00a3e0" />
            </mesh>

            {/* R axis: forearm roll about X */}
            <group position={[0.55, 0, 0]} rotation={[deg(joints.R), 0, 0]}>
              <mesh position={[0.12, 0, 0]} rotation={[0, 0, Math.PI / 2]} castShadow>
                <cylinderGeometry args={[0.08, 0.08, 0.3, 24]} />
                <meshStandardMaterial color="#0033a0" />
              </mesh>

              {/* B axis: wrist bend about Z */}
              <group position={[0.28, 0, 0]} rotation={[0, 0, deg(joints.B)]}>
                <mesh castShadow>
                  <sphereGeometry args={[0.09, 24, 24]} />
                  <meshStandardMaterial color="#00a3e0" />
                </mesh>

                {/* T axis: tool flange roll about X + a torch stub */}
                <group position={[0.08, 0, 0]} rotation={[deg(joints.T), 0, 0]}>
                  <mesh position={[0.06, 0, 0]} rotation={[0, 0, Math.PI / 2]} castShadow>
                    <cylinderGeometry args={[0.05, 0.05, 0.1, 24]} />
                    <meshStandardMaterial color="#c0c0c0" metalness={0.8} roughness={0.3} />
                  </mesh>
                  {/* Torch */}
                  <mesh position={[0.18, -0.05, 0]} rotation={[0, 0, deg(-30)]} castShadow>
                    <cylinderGeometry args={[0.015, 0.02, 0.22, 16]} />
                    <meshStandardMaterial color="#ffcc00" />
                  </mesh>
                </group>
              </group>
            </group>
          </group>
        </group>
      </group>
    </group>
  );
}

/*
 * Real-asset pattern (uncomment once you have exported per-link .glb files):
 *
 * function LinkGLB({ url, children, ...props }: any) {
 *   const { scene } = useGLTF(url);
 *   return (
 *     <group {...props}>
 *       <primitive object={scene.clone()} />
 *       {children}
 *     </group>
 *   );
 * }
 * useGLTF.preload("/models/base.glb");
 */
void useGLTF; // keep import referenced for the pattern above
