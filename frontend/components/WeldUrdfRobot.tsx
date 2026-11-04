"use client";

import { useEffect, useLayoutEffect, useMemo, useRef, useState } from "react";
import { createPortal, useLoader, useFrame } from "@react-three/fiber";
import * as THREE from "three";
import { STLLoader } from "three/examples/jsm/loaders/STLLoader.js";
import URDFLoader, { type URDFRobot } from "urdf-loader";
import WeldTorch3D from "./WeldTorch3D";
import { sampleProgram, type Vec3, type WeldProgram } from "@/lib/weldProgram";
import { AXES, type Joints } from "./AxisSliders";
import {
  DEFAULT_TORCH_CALIBRATION,
  flangeToTool0Quaternion,
  tcpMetres,
  tool0ToTcp0Quaternion,
  type TorchCalibration,
} from "@/lib/tool0";
import { publishTorchWorldPose } from "@/lib/torchPoseBridge";

const CHAIN = [
  "joint_1_s",
  "joint_2_l",
  "joint_3_u",
  "joint_4_r",
  "joint_5_b",
  "joint_6_t",
];

/**
 * MA2010 URDF flange-face translation on `link_6_t` (metres).
 * Matches `joint_6_t-tool0` xyz=(0.100,0,0) — approach = URDF +X (REP-103 flange).
 */
const MA2010_FLANGE_ORIGIN: [number, number, number] = [0.1, 0, 0];

const deg2rad = (d: number) => (d * Math.PI) / 180;

type URDFJointLike = THREE.Object3D & {
  axis: THREE.Vector3;
  angle: number;
  limit?: { lower: number; upper: number };
  setJointValue: (v: number) => boolean;
};

type URDFRobotLike = URDFRobot & {
  links: Record<string, THREE.Object3D>;
  frames?: Record<string, THREE.Object3D>;
  joints: Record<string, URDFJointLike>;
  setJointValue: (name: string, v: number) => boolean;
};

/**
 * MA2010 / AR2010 URDF with torch on the MotoROS2 TF chain
 * (`doc/ros_api.md` — Provided TF transforms):
 *
 *   world → base → flange (REP-103, X+ out) → tool0 (Yaskawa Z+ out)
 *     → tcp_0 (TOOL.CND in tool0)
 *
 * Torch mesh and `tcp_0` live in Motoman `tool0`. Wire approach is `tcp_0`
 * +Z (= TOOL.CND RPY applied to tool0 +Z), not raw flange +X / tool0 +Z.
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
  torchCalib = DEFAULT_TORCH_CALIBRATION,
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
  torchCalib?: TorchCalibration;
}) {
  const robot = useLoader(
    URDFLoader as unknown as new () => THREE.Loader,
    url,
    (loader) => {
      const l = loader as unknown as InstanceType<typeof URDFLoader>;
      l.parseVisual = true;
      l.parseCollision = false;
      l.loadMeshCb = (path, manager, _material, done) => {
        new STLLoader(manager).load(
          path,
          (geo) => {
            const mesh = new THREE.Mesh(
              geo,
              new THREE.MeshStandardMaterial({
                color,
                metalness: 0.25,
                roughness: 0.6,
              }),
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
  ) as unknown as URDFRobotLike;

  const tcpRef = useRef<THREE.Object3D | null>(null);
  const tool0Ref = useRef<THREE.Object3D | null>(null);
  const chainRef = useRef<URDFJointLike[]>([]);
  const [tool0Node, setTool0Node] = useState<THREE.Object3D | null>(null);
  const syntheticRef = useRef<THREE.Object3D[]>([]);

  const simRef = useRef(simT);
  const progRef = useRef(program);
  const jointsRef = useRef(joints);
  const manualRef = useRef(manual);
  const pivotRef = useRef(partPivot);
  const rotZRef = useRef(partRotZ);
  const calibRef = useRef(torchCalib);
  simRef.current = simT;
  progRef.current = program;
  jointsRef.current = joints;
  manualRef.current = manual;
  pivotRef.current = partPivot;
  rotZRef.current = partRotZ;
  calibRef.current = torchCalib;

  useEffect(() => {
    chainRef.current = CHAIN.map((n) => robot.joints[n]).filter(Boolean);
  }, [robot]);

  // Resolve REP-103 flange, then Motoman tool0 (Z+ out) for the torch portal.
  useLayoutEffect(() => {
    const links = robot.links ?? {};
    const frames = robot.frames ?? {};
    const link6: THREE.Object3D | null = links.link_6_t ?? frames.link_6_t ?? null;
    const rosFlange: THREE.Object3D | null = links.flange ?? frames.flange ?? null;
    const rosTool0: THREE.Object3D | null = links.tool0 ?? frames.tool0 ?? null;
    const created: THREE.Object3D[] = [];

    const detach = () => {
      for (const n of created) {
        n.parent?.remove(n);
      }
      syntheticRef.current = [];
      tool0Ref.current = null;
      setTool0Node(null);
    };

    // AR2010 (and any URDF with ROS-I flange + Motoman tool0 under it).
    if (rosFlange && rosTool0 && rosTool0.parent === rosFlange) {
      tool0Ref.current = rosTool0;
      setTool0Node(rosTool0);
      return detach;
    }

    // Build flange (REP-103, X+ out of face) then Motoman tool0 child.
    let flange: THREE.Object3D | null = rosFlange;
    if (!flange && link6) {
      // MA2010: URDF "tool0" is flange-face with identity ≡ REP-103 approach (+X).
      if (rosTool0) {
        flange = rosTool0;
        if (flange.parent !== link6) {
          flange.position.set(...MA2010_FLANGE_ORIGIN);
          flange.quaternion.identity();
          link6.add(flange);
        }
      } else {
        const synFlange = new THREE.Group();
        synFlange.name = "flange_synthetic";
        synFlange.position.set(...MA2010_FLANGE_ORIGIN);
        link6.add(synFlange);
        created.push(synFlange);
        flange = synFlange;
      }
    }

    if (!flange) {
      detach();
      return detach;
    }

    // Motoman tool0: Z+ out of flange (ros_api.md flange → tool0).
    let tool0: THREE.Object3D | null =
      rosTool0 && rosTool0.parent === flange ? rosTool0 : null;
    if (!tool0) {
      const synTool0 = new THREE.Group();
      synTool0.name = "tool0_motoman";
      synTool0.quaternion.copy(flangeToTool0Quaternion());
      flange.add(synTool0);
      created.push(synTool0);
      tool0 = synTool0;
    }

    syntheticRef.current = created;
    tool0Ref.current = tool0;
    setTool0Node(tool0);
    return detach;
  }, [robot]);

  const mountEuler = useMemo(
    () =>
      [
        deg2rad(torchCalib.mountRxDeg),
        deg2rad(torchCalib.mountRyDeg),
        deg2rad(torchCalib.mountRzDeg),
      ] as [number, number, number],
    [torchCalib],
  );

  const mountPos = useMemo(
    () =>
      [
        torchCalib.mountXmm / 1000,
        torchCalib.mountYmm / 1000,
        torchCalib.mountZmm / 1000,
      ] as [number, number, number],
    [torchCalib],
  );

  const tcpPos = tcpMetres(torchCalib);
  const tcpQuat = useMemo(
    () =>
      tool0ToTcp0Quaternion([
        torchCalib.tcpRxDeg,
        torchCalib.tcpRyDeg,
        torchCalib.tcpRzDeg,
      ]),
    [torchCalib.tcpRxDeg, torchCalib.tcpRyDeg, torchCalib.tcpRzDeg],
  );

  const target = useRef(new THREE.Vector3()).current;
  const desiredDir = useRef(new THREE.Vector3()).current;
  const toolDir = useRef(new THREE.Vector3()).current;
  const P = useRef(new THREE.Vector3()).current;
  const A = useRef(new THREE.Vector3()).current;
  const E = useRef(new THREE.Vector3()).current;
  const ve = useRef(new THREE.Vector3()).current;
  const vt = useRef(new THREE.Vector3()).current;
  const cr = useRef(new THREE.Vector3()).current;
  const qWorld = useRef(new THREE.Quaternion()).current;

  function applyJoint(j: URDFJointLike, delta: number) {
    let next = (j.angle ?? 0) + delta;
    const lim = j.limit;
    if (lim && typeof lim.lower === "number" && lim.upper > lim.lower) {
      next = THREE.MathUtils.clamp(next, lim.lower, lim.upper);
    }
    j.setJointValue(next);
    robot.updateMatrixWorld(true);
  }

  function readToolDir(out: THREE.Vector3) {
    // tcp_0 +Z = wire stick-out / fillet approach (TOOL.CND RPY in tool0).
    const tcp = tcpRef.current;
    if (!tcp) {
      out.set(0, -1, 0);
      return out;
    }
    tcp.getWorldQuaternion(qWorld);
    return out.set(0, 0, 1).applyQuaternion(qWorld).normalize();
  }

  useFrame(() => {
    const chain = chainRef.current;
    const tcp = tcpRef.current;
    if (!chain.length || !tcp) return;

    const prog = progRef.current;
    const ikActive = !!prog && !manualRef.current;

    const publishTip = () => {
      tcp.updateWorldMatrix(true, false);
      tcp.getWorldPosition(E);
      readToolDir(toolDir);
      publishTorchWorldPose(
        [E.x, E.y, E.z],
        [toolDir.x, toolDir.y, toolDir.z],
      );
    };

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
      publishTip();
      return;
    }

    const s = prog ? sampleProgram(prog, simRef.current) : null;

    // DX200 MOVJ (and any sample with baked joints): drive axes directly —
    // TCP follows FK, not a straight Cartesian chord.
    if (s?.jointsDeg && s.jointsDeg.length >= 6 && s.pathClass === "joint") {
      for (let k = 0; k < CHAIN.length; k++) {
        const axis = AXES[k];
        if (!axis) continue;
        const rad = deg2rad(s.jointsDeg[k] ?? 0);
        if (robot.setJointValue) robot.setJointValue(CHAIN[k], rad);
        else if (chain[k]) chain[k].setJointValue(rad);
      }
      robot.updateMatrixWorld(true);
      publishTip();
      return;
    }

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
      target.set(
        pivot[0] + dx * c - dy * sn,
        pivot[1] + dx * sn + dy * c,
        target.z,
      );
      const ddx = desiredDir.x;
      const ddy = desiredDir.y;
      desiredDir
        .set(ddx * c - ddy * sn, ddx * sn + ddy * c, desiredDir.z)
        .normalize();
    }

    // Prefer dense joint stream for Cartesian segments when available.
    if (s?.jointsDeg && s.jointsDeg.length >= 6 && prog?.meta?.denseJointAnglesDeg) {
      for (let k = 0; k < CHAIN.length; k++) {
        const axis = AXES[k];
        if (!axis) continue;
        const rad = deg2rad(s.jointsDeg[k] ?? 0);
        if (robot.setJointValue) robot.setJointValue(CHAIN[k], rad);
        else if (chain[k]) chain[k].setJointValue(rad);
      }
      robot.updateMatrixWorld(true);
      publishTip();
      return;
    }

    if (chain.length >= 6) {
      chain[5].setJointValue(0);
      robot.updateMatrixWorld(true);
    }

    for (let it = 0; it < 14; it++) {
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
        applyJoint(j, ang * 0.9);
      }

      tcp.getWorldPosition(E);
      const posErr = E.distanceTo(target);

      if (posErr < 0.025 && chain.length > 4) {
        const j = chain[4];
        j.getWorldPosition(P);
        A.copy(j.axis).transformDirection(j.matrixWorld).normalize();
        readToolDir(toolDir);
        ve.copy(toolDir).addScaledVector(A, -toolDir.dot(A));
        vt.copy(desiredDir).addScaledVector(A, -desiredDir.dot(A));
        if (ve.lengthSq() > 1e-8 && vt.lengthSq() > 1e-8) {
          ve.normalize();
          vt.normalize();
          let ang = Math.acos(THREE.MathUtils.clamp(ve.dot(vt), -1, 1));
          cr.crossVectors(ve, vt);
          if (cr.dot(A) < 0) ang = -ang;
          applyJoint(j, ang * 0.2);
          tcp.getWorldPosition(E);
        }
      }

      if (E.distanceTo(target) < 0.004) break;
    }
    publishTip();
  });

  const arcOn = program ? sampleProgram(program, simT).arcOn : false;

  // Portal into Motoman tool0: torch + tcp_0 (TOOL.CND) share that frame.
  const torchPortal =
    tool0Node &&
    createPortal(
      <group position={mountPos} rotation={mountEuler}>
        <WeldTorch3D calib={torchCalib} arcOn={arcOn} />
        <group
          ref={(node) => {
            tcpRef.current = node;
          }}
          position={tcpPos}
          quaternion={tcpQuat}
          name="tcp_0"
        />
      </group>,
      tool0Node,
    );

  return (
    <group>
      <group rotation={[-Math.PI / 2, 0, 0]}>
        <primitive object={robot} />
      </group>
      {torchPortal}
    </group>
  );
}
