/**
 * Production DX200 TOOL 0 + MotoROS2 / ROS-Industrial frame chain.
 *
 * Official MotoROS2 (`doc/ros_api.md` — Provided TF transforms):
 *
 *   flange  — REP-103 relative to the link: X+ forward (out of flange),
 *             Y+ left, Z+ up.
 *   tool0   — Yaskawa "all zeros toolframe": Z+ forward (out of flange).
 *             Static `flange → tool0` (never follows a configured tool file).
 *   tcp_0   — active Yaskawa Tool No. 0 from `TOOL.CND`, expressed in the
 *             Motoman / `tool0` frame (Z+ out).
 *
 * MA2010 URDF `joint_6_t-tool0` (0.100,0,0) places a flange-face frame with
 * approach = URDF +X (REP-103 `flange`). AR2010 already has `flange` +
 * `flange-tool0` with the Motoman rotation below.
 */

import * as THREE from "three";

export const TOOL0_XYZ_MM = [-54.28, 1.052, 415.702] as const;
export const TOOL0_RPY_DEG = [1.3119, -42.6215, -6.8387] as const;

export const TOOL0_XYZ_M = [
  TOOL0_XYZ_MM[0] / 1000,
  TOOL0_XYZ_MM[1] / 1000,
  TOOL0_XYZ_MM[2] / 1000,
] as const;

/** Straight-line tool0-origin → tip length (metres). */
export const TOOL0_LENGTH_M = Math.hypot(...TOOL0_XYZ_M);

/** Stick-out beyond copper nozzle (m). */
export const TORCH_STICK_OUT_M = 0.018;

/**
 * ROS-Industrial / Motoman `flange → tool0` (AR2010 `flange-tool0` URDF).
 * Motoman +Z = flange +X, Motoman +X = flange +Z, Motoman +Y = flange −Y.
 * Matches backend `flange_to_tool0_se3` (Three.js Matrix4 column-major).
 */
export function flangeToTool0Quaternion(): THREE.Quaternion {
  const m = new THREE.Matrix4().fromArray([
    0, 0, 1, 0, // col0 = tool0 +X = flange +Z
    0, -1, 0, 0, // col1 = tool0 +Y = flange −Y
    1, 0, 0, 0, // col2 = tool0 +Z = flange +X
    0, 0, 0, 1,
  ]);
  return new THREE.Quaternion().setFromRotationMatrix(m);
}

/**
 * @deprecated Prefer the explicit `flange → tool0` chain. Kept for the
 * calibration panel so an operator can force a raw axis mapping.
 */
export type FlangeAxisPreset = "plusZ_iso" | "plusX_raw" | "minusX" | "plusY";

export type TorchCalibration = {
  /** TOOL 0 TCP offset from Motoman `tool0`, millimetres. */
  tcpXmm: number;
  tcpYmm: number;
  tcpZmm: number;
  /** TOOL 0 RPY, degrees (Motoman Rx, Ry, Rz) in `tool0`. */
  tcpRxDeg: number;
  tcpRyDeg: number;
  tcpRzDeg: number;
  /** Extra mount offset on the Motoman `tool0` parent, millimetres. */
  mountXmm: number;
  mountYmm: number;
  mountZmm: number;
  mountRxDeg: number;
  mountRyDeg: number;
  mountRzDeg: number;
  /**
   * Override for the Motoman←flange map. Default `plusZ_iso` matches
   * `flange → tool0` approach (Z_m ≡ X_flange); use only when skipping the
   * explicit Motoman child (legacy).
   */
  flangePreset: FlangeAxisPreset;
  /** Show AxesHelper at tool0 origin and tcp_0. */
  showAxes: boolean;
  /**
   * Geometry mode.
   * - `vendor` — bundled ESAB `rm62_36deg.stl` (Verbotics cell mesh)
   * - `procedural` — simple CAD cylinders / swan-neck
   * - `custom` — user-uploaded STL/GLB
   */
  meshMode: "vendor" | "procedural" | "custom";
  /** Object URL for uploaded STL/GLB (session-only). */
  customMeshUrl: string | null;
  customMeshKind: "stl" | "glb" | null;
  /** Extra transform for a custom mesh in Motoman `tool0` (mm / deg). */
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

/** Bumped so previous "procedural" localStorage defaults pick up the ESAB mesh. */
const STORAGE_KEY = "yaskawa-torch-calibration-v2";

/** Persist numeric/settings fields (not blob object URLs). */
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

/** Motoman TOOL RPY Euler order used with the ESAB mesh / TOOL.CND mount. */
export const TOOL_RPY_ORDER = "ZYX" as const;

/**
 * Euler (XYZ) that maps Motoman flange (+Z out) onto a REP-103 parent (+X out).
 * Prefer parenting under an explicit Motoman `tool0` child instead.
 */
export function flangePresetEuler(
  preset: FlangeAxisPreset,
): [number, number, number] {
  const h = Math.PI / 2;
  switch (preset) {
    case "plusZ_iso":
      return [0, h, 0]; // Motoman Z → URDF +X (approach only; roll differs from ROS-I)
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
