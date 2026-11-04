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
  flangePresetEuler,
  tcpMetres,
  tcpRpyRad,
  type TorchCalibration,
} from "@/lib/tool0";

const CHAIN = [
  "joint_1_s",
  "joint_2_l",
  "joint_3_u",
  "joint_4_r",
  "joint_5_b",
  "joint_6_t",
];

/** MA2010 URDF: flange face / tool0 translation on link_6_t (metres). */
const MA2010_TOOL0_ORIGIN: [number, number, number] = [0.1, 0, 0];

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
 * MA2010 / AR2010 URDF with torch portaled onto the wrist flange.
 *
 * MA2010 `link_6_t.stl` flange disc sits at X∈[0.087, 0.100] — matching
 * `joint_6_t-tool0` xyz=(0.100,0,0). Empty `tool0` links are still created by
 * urdf-loader; if missing we synthesise the same transform on `link_6_t`.
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
  const chainRef = useRef<URDFJointLike[]>([]);
  const [flangeNode, setFlangeNode] = useState<THREE.Object3D | null>(null);
  const flangeRef = useRef<THREE.Object3D | null>(null);
  const syntheticRef = useRef<THREE.Group | null>(null);

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

  // Resolve / synthesise the flange parent for createPortal.
  useLayoutEffect(() => {
    const links = robot.links ?? {};
    const frames = robot.frames ?? {};
    const link6: THREE.Object3D | null = links.link_6_t ?? frames.link_6_t ?? null;
    const rosFlange: THREE.Object3D | null = links.flange ?? frames.flange ?? null;
    let tool0: THREE.Object3D | null = links.tool0 ?? frames.tool0 ?? null;

    // Prefer ROS-I flange when present (AR2010).
    if (rosFlange) {
      flangeRef.current = rosFlange;
      setFlangeNode(rosFlange);
      return () => {
        flangeRef.current = null;
        setFlangeNode(null);
      };
    }

    // Ensure tool0 exists under link_6_t with the URDF joint_6_t-tool0 pose.
    if (!tool0 && link6) {
      const syn = new THREE.Group();
      syn.name = "tool0_synthetic";
      syn.position.set(...MA2010_TOOL0_ORIGIN);
      link6.add(syn);
      syntheticRef.current = syn;
      tool0 = syn;
    } else if (tool0 && link6 && tool0.parent !== link6) {
      // Loader created tool0 but parenting failed — re-attach.
      tool0.position.set(...MA2010_TOOL0_ORIGIN);
      link6.add(tool0);
    }

    const parent = tool0 ?? link6;
    flangeRef.current = parent;
    setFlangeNode(parent);
    return () => {
      if (syntheticRef.current?.parent) {
        syntheticRef.current.parent.remove(syntheticRef.current);
      }
      syntheticRef.current = null;
      flangeRef.current = null;
      setFlangeNode(null);
    };
  }, [robot]);

  // Motoman→URDF mount + user calibration.
  const mountEuler = useMemo(() => {
    const preset = flangePresetEuler(torchCalib.flangePreset);
    return [
      preset[0] + deg2rad(torchCalib.mountRxDeg),
      preset[1] + deg2rad(torchCalib.mountRyDeg),
      preset[2] + deg2rad(torchCalib.mountRzDeg),
    ] as [number, number, number];
  }, [torchCalib]);

  const mountPos = useMemo(
    () =>
      [
        torchCalib.mountXmm / 1000,
        torchCalib.mountYmm / 1000,
        torchCalib.mountZmm / 1000,
      ] as [number, number, number],
    [torchCalib],
  );

  const [tcpRx, tcpRy, tcpRz] = tcpRpyRad(torchCalib);
  const tcpPos = tcpMetres(torchCalib);

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
    // Approach = Motoman +Z ≡ URDF tool0 +X. Ignore TOOL 0 RPY here — that
    // only tilts the wire for mesh/export; using it in CCD made the arm dive.
    const fl = flangeRef.current;
    if (!fl) {
      out.set(0, -1, 0);
      return out;
    }
    fl.getWorldQuaternion(qWorld);
    return out.set(1, 0, 0).applyQuaternion(qWorld).normalize();
  }

  useFrame(() => {
    const chain = chainRef.current;
    const tcp = tcpRef.current;
    if (!chain.length || !tcp) return;

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
        // Slight damping keeps CCD stable with the ~0.42 m TOOL 0 lever.
        applyJoint(j, ang * 0.9);
      }

      tcp.getWorldPosition(E);
      const posErr = E.distanceTo(target);

      // Soft wrist bend (B only) once the tip is on the seam. Full U/R/B
      // orientation CCD fights the longer TOOL 0 offset and dives into the floor.
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
  });

  const arcOn = program ? sampleProgram(program, simT).arcOn : false;

  const torchPortal =
    flangeNode &&
    createPortal(
      <group position={mountPos} rotation={mountEuler}>
        <WeldTorch3D calib={torchCalib} arcOn={arcOn} />
        <group
          ref={(node) => {
            tcpRef.current = node;
          }}
          position={tcpPos}
          rotation={[tcpRx, tcpRy, tcpRz]}
        />
      </group>,
      flangeNode,
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
