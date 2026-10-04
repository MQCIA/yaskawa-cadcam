"use client";

import { useEffect, useRef } from "react";
import { useLoader, useFrame } from "@react-three/fiber";
import * as THREE from "three";
import { STLLoader } from "three/examples/jsm/loaders/STLLoader.js";
import URDFLoader, { type URDFRobot } from "urdf-loader";
import WeldingTorch from "./WeldingTorch";
import { sampleProgram, type Vec3, type WeldProgram } from "@/lib/weldProgram";
import { TOOL0_RPY_RAD, TOOL0_TCP_M } from "@/lib/tool0";
import { AXES, type Joints } from "./AxisSliders";

// Base -> tip revolute chain (S/L/U/R/B/T).
const CHAIN = [
  "joint_1_s",
  "joint_2_l",
  "joint_3_u",
  "joint_4_r",
  "joint_5_b",
  "joint_6_t",
];
const deg2rad = (d: number) => (d * Math.PI) / 180;

type URDFJointLike = THREE.Object3D & {
  axis: THREE.Vector3;
  angle: number;
  limit?: { lower: number; upper: number };
  setJointValue: (v: number) => boolean;
};

type RobotWithFrames = URDFRobot & {
  joints: Record<string, URDFJointLike>;
  links: Record<string, THREE.Object3D>;
  frames: Record<string, THREE.Object3D>;
  setJointValue?: (n: string, v: number) => boolean;
};

/**
 * Motoman URDF (MA2010 / AR2010) with the production TOOL 0 torch on the flange.
 * IK targets the TOOL tip (not joint_6), and the visual tip matches TOOL.CND.
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
  manual?: boolean;
  partPivot?: Vec3;
  partRotZ?: number;
}) {
  const robot = useLoader(
    URDFLoader as unknown as new () => THREE.Loader,
    url,
    (loader) => {
      const l = loader as unknown as InstanceType<typeof URDFLoader>;
      l.parseVisual = true;
      l.parseCollision = false;
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
  ) as unknown as RobotWithFrames;

  const tcpRef = useRef<THREE.Object3D | null>(null);
  const flangeRef = useRef<THREE.Object3D | null>(null);
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

  // Production TOOL 0 relative to flange (Motoman Rx,Ry,Rz → ZYX euler).
  const toolLocal = useRef(new THREE.Matrix4()).current;
  const toolQuat = useRef(new THREE.Quaternion()).current;
  const toolEuler = useRef(
    new THREE.Euler(TOOL0_RPY_RAD[0], TOOL0_RPY_RAD[1], TOOL0_RPY_RAD[2], "ZYX"),
  ).current;
  toolQuat.setFromEuler(toolEuler);
  toolLocal.compose(
    new THREE.Vector3(TOOL0_TCP_M[0], TOOL0_TCP_M[1], TOOL0_TCP_M[2]),
    toolQuat,
    new THREE.Vector3(1, 1, 1),
  );

  useEffect(() => {
    chainRef.current = CHAIN.map((n) => robot.joints[n]).filter(Boolean);
    // Prefer ROS-Industrial flange, then tool0, then wrist link.
    const flange =
      robot.frames?.flange ??
      robot.links?.flange ??
      robot.frames?.tool0 ??
      robot.links?.tool0 ??
      robot.joints?.joint_6_t ??
      null;
    flangeRef.current = flange;
    if (!flange) return;

    const tcp = new THREE.Object3D();
    tcp.position.set(TOOL0_TCP_M[0], TOOL0_TCP_M[1], TOOL0_TCP_M[2]);
    tcp.quaternion.copy(toolQuat);
    flange.add(tcp);
    tcpRef.current = tcp;

    return () => {
      if (flangeRef.current && tcpRef.current) flangeRef.current.remove(tcpRef.current);
      tcpRef.current = null;
    };
  }, [robot, toolQuat]);

  const target = useRef(new THREE.Vector3()).current;
  const desiredDir = useRef(new THREE.Vector3()).current;
  const toolDir = useRef(new THREE.Vector3()).current;
  const P = useRef(new THREE.Vector3()).current;
  const A = useRef(new THREE.Vector3()).current;
  const E = useRef(new THREE.Vector3()).current;
  const ve = useRef(new THREE.Vector3()).current;
  const vt = useRef(new THREE.Vector3()).current;
  const cr = useRef(new THREE.Vector3()).current;
  const desired = useRef(new THREE.Matrix4()).current;
  const parentInv = useRef(new THREE.Matrix4()).current;
  const zAxis = useRef(new THREE.Vector3(0, 0, 1)).current;

  function applyJoint(j: URDFJointLike, delta: number) {
    let next = (j.angle ?? 0) + delta;
    const lim = j.limit;
    if (lim && typeof lim.lower === "number" && lim.upper > lim.lower) {
      next = THREE.MathUtils.clamp(next, lim.lower, lim.upper);
    }
    j.setJointValue(next);
    robot.updateMatrixWorld(true);
  }

  /** World matrix of the TOOL frame (flange × TOOL0). Torch tip = origin. */
  function placeTorch() {
    const flange = flangeRef.current;
    if (!flange || !torchRef.current) return;
    flange.updateWorldMatrix(true, false);
    desired.multiplyMatrices(flange.matrixWorld, toolLocal);
    const parent = torchRef.current.parent;
    if (parent) {
      parent.updateWorldMatrix(true, false);
      parentInv.copy(parent.matrixWorld).invert();
      desired.premultiply(parentInv);
    }
    torchRef.current.matrixAutoUpdate = false;
    torchRef.current.matrix.copy(desired);
  }

  function toolApproachWorld(out: THREE.Vector3) {
    const flange = flangeRef.current;
    if (!flange) {
      out.set(0, -1, 0);
      return out;
    }
    flange.updateWorldMatrix(true, false);
    desired.multiplyMatrices(flange.matrixWorld, toolLocal);
    out.copy(zAxis).transformDirection(desired).normalize();
    return out;
  }

  useFrame(() => {
    const chain = chainRef.current;
    const tcp = tcpRef.current;
    const flange = flangeRef.current;
    if (!chain.length || !tcp || !flange) return;

    const prog = progRef.current;
    const ikActive = !!prog && !manualRef.current;

    if (!ikActive) {
      const jm = jointsRef.current;
      for (let k = 0; k < CHAIN.length; k++) {
        const axis = AXES[k];
        if (!axis) continue;
        const rad = deg2rad(jm[axis] ?? 0);
        if (robot.setJointValue) robot.setJointValue(CHAIN[k], rad);
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
      desiredDir.set(0, -1, 0);
    }

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

    if (chain.length >= 6) {
      chain[5].setJointValue(0);
      robot.updateMatrixWorld(true);
    }

    for (let it = 0; it < 10; it++) {
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

      // Orient TOOL +Z (wire) to desiredDir — not the raw joint_6 axis.
      for (let k = Math.min(4, chain.length - 1); k >= 2; k--) {
        const j = chain[k];
        j.getWorldPosition(P);
        A.copy(j.axis).transformDirection(j.matrixWorld).normalize();
        toolApproachWorld(toolDir);
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
      toolApproachWorld(toolDir);
      if (E.distanceTo(target) < 0.004 && toolDir.dot(desiredDir) > 0.995) break;
    }

    placeTorch();
  });

  const arcOn = program ? sampleProgram(program, simT).arcOn : false;

  return (
    <group>
      <group rotation={[-Math.PI / 2, 0, 0]}>
        <primitive object={robot} />
      </group>
      <group ref={torchRef}>
        <WeldingTorch arcOn={arcOn} />
      </group>
    </group>
  );
}
