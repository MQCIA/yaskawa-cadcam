/**
 * Torch calibration UI + Motoman frame helpers.
 *
 * TOOL 0 numeric constants live in ``weldProgram.ts`` (single FE source of
 * truth, mirrored from ``backend/app/robot_config.py``). This module adds
 * live calibration state and ``T_flange_to_tool0`` / ``T_tool0_to_tcp0`` helpers
 * used by ``WeldUrdfRobot`` / ``WeldTorch3D``.
 */

import * as THREE from "three";
import {
  FLANGE_TO_TOOL0_R,
  TOOL0_LENGTH_M,
  TOOL0_RPY_DEG,
  TOOL0_XYZ_M,
  TOOL0_XYZ_MM,
  TOOL_RPY_ORDER,
} from "@/lib/weldProgram";

export {
  FLANGE_TO_TOOL0_R,
  TOOL0_LENGTH_M,
  TOOL0_RPY_DEG,
  TOOL0_XYZ_M,
  TOOL0_XYZ_MM,
  TOOL_RPY_ORDER,
};

/** @deprecated use TOOL0_LENGTH_M */
export const TOOL0_LENGTH_M_ALIAS = TOOL0_LENGTH_M;

/** Stick-out beyond copper nozzle for procedural mesh only (m). */
export const TORCH_STICK_OUT_M = 0.018;

/** Quaternion for ``T_flange_to_tool0`` (same matrix as backend). */
export function flangeToTool0Quaternion(): THREE.Quaternion {
  const R = FLANGE_TO_TOOL0_R;
  // Three.js Matrix4.fromArray is column-major: col0, col1, col2, col3.
  const m = new THREE.Matrix4().fromArray([
    R[0][0], R[1][0], R[2][0], 0,
    R[0][1], R[1][1], R[2][1], 0,
    R[0][2], R[1][2], R[2][2], 0,
    0, 0, 0, 1,
  ]);
  return new THREE.Quaternion().setFromRotationMatrix(m);
}

/** ``T_tool0_to_tcp0`` quaternion from TOOL.CND RPY. */
export function tool0ToTcp0Quaternion(
  rpyDeg: readonly [number, number, number] = TOOL0_RPY_DEG,
): THREE.Quaternion {
  const d = Math.PI / 180;
  return new THREE.Quaternion().setFromEuler(
    new THREE.Euler(rpyDeg[0] * d, rpyDeg[1] * d, rpyDeg[2] * d, TOOL_RPY_ORDER),
  );
}

/**
 * @deprecated Prefer the explicit flange → tool0 chain.
 */
export type FlangeAxisPreset = "plusZ_iso" | "plusX_raw" | "minusX" | "plusY";

export type TorchCalibration = {
  tcpXmm: number;
  tcpYmm: number;
  tcpZmm: number;
  tcpRxDeg: number;
  tcpRyDeg: number;
  tcpRzDeg: number;
  mountXmm: number;
  mountYmm: number;
  mountZmm: number;
  mountRxDeg: number;
  mountRyDeg: number;
  mountRzDeg: number;
  flangePreset: FlangeAxisPreset;
  showAxes: boolean;
  meshMode: "vendor" | "procedural" | "custom";
  customMeshUrl: string | null;
  customMeshKind: "stl" | "glb" | null;
  customXmm: number;
  customYmm: number;
  customZmm: number;
  customRxDeg: number;
  customRyDeg: number;
  customRzDeg: number;
  customScale: number;
};

export const DEFAULT_TORCH_CALIBRATION: TorchCalibration = {
  tcpXmm: TOOL0_XYZ_MM[0],
  tcpYmm: TOOL0_XYZ_MM[1],
  tcpZmm: TOOL0_XYZ_MM[2],
  tcpRxDeg: TOOL0_RPY_DEG[0],
  tcpRyDeg: TOOL0_RPY_DEG[1],
  tcpRzDeg: TOOL0_RPY_DEG[2],
  mountXmm: 0,
  mountYmm: 0,
  mountZmm: 0,
  mountRxDeg: 0,
  mountRyDeg: 0,
  mountRzDeg: 0,
  flangePreset: "plusZ_iso",
  showAxes: false,
  meshMode: "vendor",
  customMeshUrl: null,
  customMeshKind: null,
  customXmm: 0,
  customYmm: 0,
  customZmm: 0,
  customRxDeg: 0,
  customRyDeg: 0,
  customRzDeg: 0,
  customScale: 1,
};

const STORAGE_KEY = "yaskawa-torch-calibration-v3";

export function saveTorchCalibration(c: TorchCalibration) {
  try {
    const { customMeshUrl: _u, customMeshKind: _k, ...rest } = c;
    localStorage.setItem(STORAGE_KEY, JSON.stringify(rest));
  } catch {
    /* ignore */
  }
}

export function loadTorchCalibration(): TorchCalibration {
  try {
    const raw = localStorage.getItem(STORAGE_KEY);
    if (!raw) return { ...DEFAULT_TORCH_CALIBRATION };
    const parsed = JSON.parse(raw) as Partial<TorchCalibration>;
    const meshMode =
      parsed.meshMode === "vendor" ||
      parsed.meshMode === "procedural" ||
      parsed.meshMode === "custom"
        ? parsed.meshMode
        : DEFAULT_TORCH_CALIBRATION.meshMode;
    return {
      ...DEFAULT_TORCH_CALIBRATION,
      ...parsed,
      // Always re-seed dump TOOL 0 as defaults when keys missing.
      tcpXmm: parsed.tcpXmm ?? TOOL0_XYZ_MM[0],
      tcpYmm: parsed.tcpYmm ?? TOOL0_XYZ_MM[1],
      tcpZmm: parsed.tcpZmm ?? TOOL0_XYZ_MM[2],
      tcpRxDeg: parsed.tcpRxDeg ?? TOOL0_RPY_DEG[0],
      tcpRyDeg: parsed.tcpRyDeg ?? TOOL0_RPY_DEG[1],
      tcpRzDeg: parsed.tcpRzDeg ?? TOOL0_RPY_DEG[2],
      meshMode,
      customMeshUrl: null,
      customMeshKind: null,
    };
  } catch {
    return { ...DEFAULT_TORCH_CALIBRATION };
  }
}

export function tcpMetres(c: TorchCalibration): [number, number, number] {
  return [c.tcpXmm / 1000, c.tcpYmm / 1000, c.tcpZmm / 1000];
}

export function tcpRpyRad(c: TorchCalibration): [number, number, number] {
  const d = Math.PI / 180;
  return [c.tcpRxDeg * d, c.tcpRyDeg * d, c.tcpRzDeg * d];
}

export function flangePresetEuler(
  preset: FlangeAxisPreset,
): [number, number, number] {
  const h = Math.PI / 2;
  switch (preset) {
    case "plusZ_iso":
      return [0, h, 0];
    case "plusX_raw":
      return [0, 0, 0];
    case "minusX":
      return [0, -h, 0];
    case "plusY":
      return [-h, 0, 0];
    default:
      return [0, h, 0];
  }
}

export const FLANGE_PRESET_OPTIONS: {
  id: FlangeAxisPreset;
  label: string;
}[] = [
  { id: "plusZ_iso", label: "+Z (ISO / Yaskawa TOOL)" },
  { id: "plusX_raw", label: "+X (ROS URDF raw / AR2010 flange)" },
  { id: "minusX", label: "−X" },
  { id: "plusY", label: "+Y" },
];

/** Dump defaults for reset buttons. */
export const DUMP_TOOL0 = {
  xyzMm: TOOL0_XYZ_MM,
  xyzM: TOOL0_XYZ_M,
  rpyDeg: TOOL0_RPY_DEG,
} as const;
