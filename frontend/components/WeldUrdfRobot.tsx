"use client";

import { useEffect, useRef } from "react";
import { useLoader, useFrame } from "@react-three/fiber";
import * as THREE from "three";
import { STLLoader } from "three/examples/jsm/loaders/STLLoader.js";
import URDFLoader, { type URDFRobot } from "urdf-loader";
import WeldingTorch from "./WeldingTorch";
import { sampleProgram, type Vec3, type WeldProgram } from "@/lib/weldProgram";
import { AXES, type Joints } from "./AxisSliders";

// Base -> tip revolute chain of the Motoman AR2010 URDF.
// Order matches the S/L/U/R/B/T jog axes so manual jog maps 1:1.
const CHAIN = [
  "joint_1_s",
  "joint_2_l",
  "joint_3_u",
  "joint_4_r",
  "joint_5_b",
  "joint_6_t",
];
const TORCH_LEN = 0.33; // flange -> torch contact tip along the tool axis
const deg2rad = (d: number) => (d * Math.PI) / 180;

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
  joints,
  manual = false,
  partPivot,
  partRotZ = 0,
}: {
  url: string;
  color?: number;
  program: WeldProgram | null;
  simT: number;
  base: [number, number, number];
  joints: Joints;
  // When true (or when no program is loaded) the robot follows the jog sliders
  // instead of the inverse-kinematics weld path.
  manual?: boolean;
  // Pivot + rotation of the workpiece on the positioner, so the IK target
  // tracks the seam when the table is rotated (matches WeldScene wrapping).
  partPivot?: Vec3;
  partRotZ?: number;
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
  const jointsRef = useRef(joints);
  const manualRef = useRef(manual);
  const pivotRef = useRef(partPivot);
  const rotZRef = useRef(partRotZ);
  simRef.current = simT;
  progRef.current = program;
  jointsRef.current = joints;
  manualRef.current = manual;
  pivotRef.current = partPivot;
  rotZRef.current = partRotZ;

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
  const desiredDir = useRef(new THREE.Vector3()).current;
  const toolDir = useRef(new THREE.Vector3()).current;
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

  function applyJoint(j: URDFJointLike, delta: number) {
    let next = (j.angle ?? 0) + delta;
    const lim = j.limit;
    if (lim && typeof lim.lower === "number" && lim.upper > lim.lower) {
      next = THREE.MathUtils.clamp(next, lim.lower, lim.upper);
    }
    j.setJointValue(next);
    robot.updateMatrixWorld(true);
  }

  // Place the torch visual on the flange, aligned to the tool axis.
  function placeTorch() {
    const mount = mountRef.current;
    if (!mount || !torchRef.current) return;
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

  useFrame(() => {
    const chain = chainRef.current;
    const tcp = tcpRef.current;
    const mount = mountRef.current;
    if (!chain.length || !tcp || !mount) return;

    const prog = progRef.current;
    const ikActive = !!prog && !manualRef.current;

    // Manual jog: drive each axis straight from the S/L/U/R/B/T sliders.
    if (!ikActive) {
      const jm = jointsRef.current;
      for (let k = 0; k < CHAIN.length; k++) {
        const axis = AXES[k];
        if (!axis) continue;
        const rad = deg2rad(jm[axis] ?? 0);
        // Prefer the robot-level API (same as UrdfModel) — more reliable than
        // calling setJointValue on a joint handle that may be stale.
        const robotJoints = (robot as unknown as { setJointValue?: (n: string, v: number) => boolean }).setJointValue;
        if (robotJoints) robotJoints.call(robot, CHAIN[k], rad);
        else if (chain[k]) chain[k].setJointValue(rad);
      }
      robot.updateMatrixWorld(true);
      placeTorch();
      return;
    }

    const s = prog ? sampleProgram(prog, simRef.current) : null;
    if (s) {
      target.set(s.pos[0], s.pos[1], s.pos[2]);
      desiredDir.set(s.dir[0], s.dir[1], s.dir[2]).normalize();
    } else {
      target.set(base[0] + 1.2, 0.95, base[2]);
      desiredDir.set(0, -1, 0); // default: torch pointing straight down
    }

    // Rotate target + approach with the positioner so the torch tracks the seam
    // (and stays at a fixed work angle) when the table turns about world Z.
    const pivot = pivotRef.current;
    const rz = rotZRef.current ?? 0;
    if (s && pivot && Math.abs(rz) > 1e-6) {
      const c = Math.cos(rz);
      const sn = Math.sin(rz);
      const dx = target.x - pivot[0];
      const dy = target.y - pivot[1];
      target.set(pivot[0] + dx * c - dy * sn, pivot[1] + dx * sn + dy * c, target.z);
      const ddx = desiredDir.x;
      const ddy = desiredDir.y;
      desiredDir.set(ddx * c - ddy * sn, ddx * sn + ddy * c, desiredDir.z).normalize();
    }

    // Freeze torch roll (T / joint_6) so the torch does not spin about its own
    // axis while travelling — orientation stays fixed unless the user overrides.
    if (chain.length >= 6) {
      chain[5].setJointValue(0);
      robot.updateMatrixWorld(true);
    }

    // CCD: position (arm) + orientation (wrist) interleaved so the tip stays on
    // the seam AND the torch axis stays locked to desiredDir (perpendicular /
    // constant work angle along the path).
    for (let it = 0; it < 10; it++) {
      // --- position pass (skip frozen T) ---
      for (let k = chain.length - 1; k >= 0; k--) {
        if (k === 5) continue;
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
        applyJoint(j, ang);
      }

      // --- orientation pass (U / R / B) to align tool axis with desiredDir ---
      for (let k = Math.min(4, chain.length - 1); k >= 2; k--) {
        const j = chain[k];
        j.getWorldPosition(P);
        A.copy(j.axis).transformDirection(j.matrixWorld).normalize();
        toolDir.copy(mount.axis).transformDirection(mount.matrixWorld).normalize();
        ve.copy(toolDir).addScaledVector(A, -toolDir.dot(A));
        vt.copy(desiredDir).addScaledVector(A, -desiredDir.dot(A));
        if (ve.lengthSq() < 1e-8 || vt.lengthSq() < 1e-8) continue;
        ve.normalize();
        vt.normalize();
        let ang = Math.acos(THREE.MathUtils.clamp(ve.dot(vt), -1, 1));
        cr.crossVectors(ve, vt);
        if (cr.dot(A) < 0) ang = -ang;
        applyJoint(j, ang * 0.85);
      }

      tcp.getWorldPosition(E);
      toolDir.copy(mount.axis).transformDirection(mount.matrixWorld).normalize();
      if (E.distanceTo(target) < 0.004 && toolDir.dot(desiredDir) > 0.995) break;
    }

    placeTorch();
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
