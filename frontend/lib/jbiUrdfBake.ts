/**
 * Bake PULSE .JBI TCP poses with the same MA2010 URDF + TOOL 0 chain used in
 * the 3D view (``WeldUrdfRobot``), so path lines match the animated arm.
 *
 * Positions are world metres with carriage at ``(0, CARRIAGE_Y, railZ(bc))``.
 */

import * as THREE from "three";
import URDFLoader, { type URDFRobot } from "urdf-loader";
import { STLLoader } from "three/examples/jsm/loaders/STLLoader.js";
import { ROBOT_MODELS } from "@/lib/models";
import { TSL_CARRIAGE_TOP_Y } from "@/lib/trackMesh";
import {
  DEFAULT_TORCH_CALIBRATION,
  flangeToTool0Quaternion,
  tcpMetres,
  tool0ToTcp0Quaternion,
} from "@/lib/tool0";
import { railZFromBc } from "@/lib/jbiParse";
import type { Vec3, WeldProgram } from "@/lib/weldProgram";
import { computeDx200Durations } from "@/lib/dx200Sim";

const CHAIN = [
  "joint_1_s",
  "joint_2_l",
  "joint_3_u",
  "joint_4_r",
  "joint_5_b",
  "joint_6_t",
] as const;

const MA2010_FLANGE_ORIGIN: [number, number, number] = [0.1, 0, 0];

type URDFRobotLike = URDFRobot & {
  links: Record<string, THREE.Object3D>;
  frames?: Record<string, THREE.Object3D>;
  joints: Record<string, THREE.Object3D & { setJointValue: (v: number) => boolean }>;
  setJointValue?: (name: string, v: number) => boolean;
};

let cachedRobot: Promise<URDFRobotLike> | null = null;

function loadMa2010(): Promise<URDFRobotLike> {
  if (cachedRobot) return cachedRobot;
  const model = ROBOT_MODELS.find((m) => m.id === "ma2010");
  const url = model?.url;
  if (!url) return Promise.reject(new Error("Brak URDF MA2010"));

  cachedRobot = new Promise((resolve, reject) => {
    const manager = new THREE.LoadingManager();
    const loader = new URDFLoader(manager);
    loader.parseVisual = true;
    loader.parseCollision = false;
    loader.loadMeshCb = (path, mgr, _mat, done) => {
      // Skip heavy STL meshes for FK — empty placeholders are enough.
      const stub = new THREE.Object3D();
      stub.name = path;
      done(stub);
    };
    loader.load(
      url,
      (robot) => resolve(robot as unknown as URDFRobotLike),
      undefined,
      (err) => reject(err ?? new Error("URDF load failed")),
    );
    void STLLoader;
  });
  return cachedRobot;
}

function ensureTool0Tcp(robot: URDFRobotLike): {
  root: THREE.Group;
  tcp: THREE.Object3D;
} {
  const root = new THREE.Group();
  root.name = "jbi_fk_root";
  // Same as WeldUrdfRobot: URDF Z-up → Three Y-up.
  const urdfGroup = new THREE.Group();
  urdfGroup.rotation.set(-Math.PI / 2, 0, 0);
  urdfGroup.add(robot);
  root.add(urdfGroup);

  const links = robot.links ?? {};
  const frames = robot.frames ?? {};
  const link6: THREE.Object3D | undefined = links.link_6_t ?? frames.link_6_t;
  const rosTool0: THREE.Object3D | null = links.tool0 ?? frames.tool0 ?? null;
  let flange: THREE.Object3D | null = links.flange ?? frames.flange ?? null;

  if (!flange && link6) {
    if (rosTool0) {
      flange = rosTool0;
      if (flange.parent !== link6) {
        flange.position.set(...MA2010_FLANGE_ORIGIN);
        flange.quaternion.identity();
        link6.add(flange);
      }
    } else {
      const syn = new THREE.Group();
      syn.name = "flange_synthetic";
      syn.position.set(...MA2010_FLANGE_ORIGIN);
      link6.add(syn);
      flange = syn;
    }
  }
  if (!flange) throw new Error("URDF: brak flange / link_6_t");

  let tool0: THREE.Object3D =
    rosTool0 && rosTool0.parent === flange ? rosTool0 : new THREE.Group();
  if (!(rosTool0 && rosTool0.parent === flange)) {
    tool0.name = "tool0_motoman";
    tool0.quaternion.copy(flangeToTool0Quaternion());
    flange.add(tool0);
  }

  const calib = DEFAULT_TORCH_CALIBRATION;
  const tcp = new THREE.Group();
  tcp.name = "tcp_0";
  const [tx, ty, tz] = tcpMetres(calib);
  tcp.position.set(tx, ty, tz);
  tcp.quaternion.copy(
    tool0ToTcp0Quaternion([calib.tcpRxDeg, calib.tcpRyDeg, calib.tcpRzDeg]),
  );
  tool0.add(tcp);

  root.updateMatrixWorld(true);
  return { root, tcp };
}

function setJoints(robot: URDFRobotLike, qDeg: number[]) {
  for (let i = 0; i < CHAIN.length; i++) {
    const rad = (qDeg[i] * Math.PI) / 180;
    if (robot.setJointValue) robot.setJointValue(CHAIN[i], rad);
    else robot.joints[CHAIN[i]]?.setJointValue(rad);
  }
  robot.updateMatrixWorld(true);
}

/**
 * Rewrite ``waypoint.pos`` / seam / torchFrames from URDF tcp_0 + RECT-X BC.
 */
export async function bakeJbiTcpFromUrdf(
  prog: WeldProgram,
  opts?: { bcRef?: number },
): Promise<WeldProgram> {
  if (!prog.waypoints.some((w) => w.jointsDeg?.length === 6)) {
    return prog;
  }

  const robot = await loadMa2010();
  const { root, tcp } = ensureTool0Tcp(robot);
  const bcRef =
    opts?.bcRef ??
    prog.meta?.bcRefPulse ??
    prog.waypoints.find((w) => typeof w.bc === "number")?.bc ??
    0;

  const tmp = new THREE.Vector3();
  const dir = new THREE.Vector3();
  const q = new THREE.Quaternion();

  const waypoints = prog.waypoints.map((w) => {
    if (!w.jointsDeg || w.jointsDeg.length !== 6) return { ...w };
    setJoints(robot, w.jointsDeg);
    root.updateMatrixWorld(true);
    tcp.getWorldPosition(tmp);
    tcp.getWorldQuaternion(q);
    // Wire ≈ tcp_0 +Z in Motoman TOOL frame.
    dir.set(0, 0, 1).applyQuaternion(q).normalize();
    const railZ =
      typeof w.bc === "number" ? railZFromBc(w.bc, bcRef) : 0;
    const pos: Vec3 = [
      tmp.x,
      tmp.y + TSL_CARRIAGE_TOP_Y,
      tmp.z + railZ,
    ];
    return { ...w, pos };
  });

  const weldPts = waypoints.filter((w) => w.kind === "weld");
  const seam = weldPts.map((w) => w.pos);
  const torchFrames = weldPts.map((w) => {
    // Recompute dir at this pose — joints already baked into pos; use stored frames fallback.
    return {
      pos: w.pos,
      dir: [0, -Math.SQRT1_2, -Math.SQRT1_2] as Vec3,
    };
  });

  // Fill torch dirs with a second pass for weld points.
  for (let i = 0; i < waypoints.length; i++) {
    const w = waypoints[i];
    if (w.kind !== "weld" || !w.jointsDeg) continue;
    setJoints(robot, w.jointsDeg);
    root.updateMatrixWorld(true);
    tcp.getWorldQuaternion(q);
    dir.set(0, 0, 1).applyQuaternion(q).normalize();
    const wi = weldPts.indexOf(w);
    if (wi >= 0) {
      torchFrames[wi] = {
        pos: w.pos,
        dir: [dir.x, dir.y, dir.z],
      };
    }
  }

  const mount: Vec3 =
    seam.length > 0
      ? [
          seam.reduce((s, p) => s + p[0], 0) / seam.length,
          seam.reduce((s, p) => s + p[1], 0) / seam.length,
          seam.reduce((s, p) => s + p[2], 0) / seam.length,
        ]
      : waypoints[0]?.pos ?? prog.mount;

  let weldLenMm = 0;
  for (let k = 1; k < seam.length; k++) {
    weldLenMm +=
      Math.hypot(
        seam[k][0] - seam[k - 1][0],
        seam[k][1] - seam[k - 1][1],
        seam[k][2] - seam[k - 1][2],
      ) * 1000;
  }

  const draft: WeldProgram = {
    ...prog,
    mount,
    waypoints,
    seam: seam.length ? seam : prog.seam,
    torchFrames: torchFrames.length ? torchFrames : prog.torchFrames,
    weldLenMm: weldLenMm || prog.weldLenMm,
    meta: {
      collisionFree: false,
      touchSense: prog.meta?.touchSense ?? false,
      sequenceOptimized: prog.meta?.sequenceOptimized ?? false,
      singularitySafe: prog.meta?.singularitySafe ?? true,
      spinAngleDeg: prog.meta?.spinAngleDeg,
      denseJointAnglesDeg: prog.meta?.denseJointAnglesDeg,
      maxJointStepDeg: prog.meta?.maxJointStepDeg,
      seamEdits: prog.meta?.seamEdits,
      activeWeaveNo: prog.meta?.activeWeaveNo,
      bcRefPulse: bcRef,
      importedPulse: true,
      urdfBaked: true,
    },
  };
  const { segDurations, cycleSec } = computeDx200Durations(draft);
  return { ...draft, segDurations, cycleSec };
}
