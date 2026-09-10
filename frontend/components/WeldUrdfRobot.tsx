"use client";

import { useEffect, useRef } from "react";
import { useLoader, useFrame } from "@react-three/fiber";
import * as THREE from "three";
import { STLLoader } from "three/examples/jsm/loaders/STLLoader.js";
import URDFLoader, { type URDFRobot } from "urdf-loader";
import WeldingTorch from "./WeldingTorch";
import { sampleProgram, type WeldProgram } from "@/lib/weldProgram";

// Base -> tip revolute chain of the Motoman AR2010 URDF.
const CHAIN = [
  "joint_1_s",
  "joint_2_l",
  "joint_3_u",
  "joint_4_r",
  "joint_5_b",
  "joint_6_t",
];
const TORCH_LEN = 0.33; // flange -> torch contact tip along the tool axis

type URDFJointLike = THREE.Object3D & {
  axis: THREE.Vector3;
  angle: number;
  limit?: { lower: number; upper: number };
  setJointValue: (v: number) => boolean;
};

/**
 * Real Yaskawa MOTOMAN-AR2010 (URDF + STL) with a welding torch on the flange,
 * driven by a browser-side CCD inverse-kinematics solver so the torch tip tracks
 * the weld path. This makes the animated welding robot the actual AR2010 model.
 */
export default function WeldUrdfRobot({
  url,
  color = 0x1f4fb0,
  program,
  simT,
  base,
}: {
  url: string;
  color?: number;
  program: WeldProgram | null;
  simT: number;
  base: [number, number, number];
}) {
  const robot = useLoader(
    URDFLoader as unknown as new () => THREE.Loader,
    url,
    (loader) => {
      const l = loader as unknown as InstanceType<typeof URDFLoader>;
      l.loadMeshCb = (path, manager, material, done) => {
        new STLLoader(manager).load(
          path,
          (geo) => {
            const mesh = new THREE.Mesh(
              geo,
              new THREE.MeshStandardMaterial({ color, metalness: 0.25, roughness: 0.6 }),
            );
            mesh.castShadow = true;
            mesh.receiveShadow = true;
            done(mesh);
          },
          undefined,
          (err) => done(undefined as unknown as THREE.Object3D, err as Error),
        );
      };
    },
  ) as unknown as URDFRobot;

  const tcpRef = useRef<THREE.Object3D | null>(null);
  const mountRef = useRef<URDFJointLike | null>(null);
  const chainRef = useRef<URDFJointLike[]>([]);
  const torchRef = useRef<THREE.Group>(null);
  const simRef = useRef(simT);
  const progRef = useRef(program);
  simRef.current = simT;
  progRef.current = program;

  // Attach an end-effector marker (torch tip) to the flange for IK.
  useEffect(() => {
    const joints = (robot as unknown as { joints: Record<string, URDFJointLike> }).joints;
    chainRef.current = CHAIN.map((n) => joints[n]).filter(Boolean);
    const mount = joints["joint_6_t"] ?? null;
    mountRef.current = mount;
    if (mount) {
      const tcp = new THREE.Object3D();
      const ax = (mount.axis?.clone?.() ?? new THREE.Vector3(0, 0, 1)).normalize();
      tcp.position.copy(ax.multiplyScalar(TORCH_LEN));
      mount.add(tcp);
      tcpRef.current = tcp;
    }
    return () => {
      if (mountRef.current && tcpRef.current) mountRef.current.remove(tcpRef.current);
    };
  }, [robot]);

  // Scratch objects (avoid per-frame allocation).
  const target = useRef(new THREE.Vector3()).current;
  const P = useRef(new THREE.Vector3()).current;
  const A = useRef(new THREE.Vector3()).current;
  const E = useRef(new THREE.Vector3()).current;
  const ve = useRef(new THREE.Vector3()).current;
  const vt = useRef(new THREE.Vector3()).current;
  const cr = useRef(new THREE.Vector3()).current;
  const mWorld = useRef(new THREE.Vector3()).current;
  const approachW = useRef(new THREE.Vector3()).current;
  const q = useRef(new THREE.Quaternion()).current;
  const xAxis = useRef(new THREE.Vector3(1, 0, 0)).current;
  const desired = useRef(new THREE.Matrix4()).current;
  const parentInv = useRef(new THREE.Matrix4()).current;
  const scaleOne = useRef(new THREE.Vector3(1, 1, 1)).current;

  useFrame(() => {
    const chain = chainRef.current;
    const tcp = tcpRef.current;
    const mount = mountRef.current;
    if (!chain.length || !tcp || !mount) return;

    const prog = progRef.current;
    const s = prog ? sampleProgram(prog, simRef.current) : null;
    if (s) target.set(s.pos[0], s.pos[1], s.pos[2]);
    else target.set(base[0] + 1.2, 0.95, base[2]);

    // Cyclic Coordinate Descent (position only).
    for (let it = 0; it < 12; it++) {
      for (let k = chain.length - 1; k >= 0; k--) {
        const j = chain[k];
        j.getWorldPosition(P);
        A.copy(j.axis).transformDirection(j.matrixWorld).normalize();
        tcp.getWorldPosition(E);
        ve.copy(E).sub(P);
        vt.copy(target).sub(P);
        ve.addScaledVector(A, -ve.dot(A));
        vt.addScaledVector(A, -vt.dot(A));
        if (ve.lengthSq() < 1e-8 || vt.lengthSq() < 1e-8) continue;
        ve.normalize();
        vt.normalize();
        let ang = Math.acos(THREE.MathUtils.clamp(ve.dot(vt), -1, 1));
        cr.crossVectors(ve, vt);
        if (cr.dot(A) < 0) ang = -ang;
        let next = (j.angle ?? 0) + ang;
        const lim = j.limit;
        if (lim && typeof lim.lower === "number" && lim.upper > lim.lower) {
          next = THREE.MathUtils.clamp(next, lim.lower, lim.upper);
        }
        j.setJointValue(next);
        robot.updateMatrixWorld(true);
      }
      tcp.getWorldPosition(E);
      if (E.distanceTo(target) < 0.003) break;
    }

    // Place the torch visual on the flange, aligned to the tool axis.
    if (torchRef.current) {
      mount.updateWorldMatrix(true, false);
      mount.getWorldPosition(mWorld);
      approachW.copy(mount.axis).transformDirection(mount.matrixWorld).normalize();
      q.setFromUnitVectors(xAxis, approachW);
      desired.compose(mWorld, q, scaleOne);
      const parent = torchRef.current.parent;
      if (parent) {
        parent.updateWorldMatrix(true, false);
        parentInv.copy(parent.matrixWorld).invert();
        desired.premultiply(parentInv);
      }
      torchRef.current.matrixAutoUpdate = false;
      torchRef.current.matrix.copy(desired);
    }
  });

  const arcOn = program ? sampleProgram(program, simT).arcOn : false;

  return (
    <group>
      {/* URDF is Z-up in metres; rotate -90° about X to stand upright. */}
      <group rotation={[-Math.PI / 2, 0, 0]}>
        <primitive object={robot} />
      </group>
      <group ref={torchRef}>
        <WeldingTorch arcOn={arcOn} tipLen={TORCH_LEN} />
      </group>
    </group>
  );
}
