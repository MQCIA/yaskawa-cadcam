/**
 * Production DX200 TOOL 0 + live mount/TCP calibration.
 *
 * Motoman flange frame: origin at flange-face centre, +Z out of the flange
 * (ISO / Yaskawa tool axis before RPY).
 *
 * MA2010 URDF: `link_6_t.stl` flange face at X≈0.100 m (Y/Z disc ±50 mm);
 * `joint_6_t-tool0` = xyz (0.100, 0, 0), rpy (0,0,0) → URDF +X = flange approach.
 * Map Motoman → URDF with R_y(+π/2): (x,y,z)_m → (z, y, −x)_urdf.
 */

export const TOOL0_XYZ_MM = [-54.28, 1.052, 415.702] as const;
export const TOOL0_RPY_DEG = [1.3119, -42.6215, -6.8387] as const;

export const TOOL0_XYZ_M = [
  TOOL0_XYZ_MM[0] / 1000,
  TOOL0_XYZ_MM[1] / 1000,
  TOOL0_XYZ_MM[2] / 1000,
] as const;

/** Straight-line flange→tip length (metres). */
export const TOOL0_LENGTH_M = Math.hypot(...TOOL0_XYZ_M);

/** Stick-out beyond copper nozzle (m). */
export const TORCH_STICK_OUT_M = 0.018;

export type FlangeAxisPreset = "plusZ_iso" | "plusX_raw" | "minusX" | "plusY";

export type TorchCalibration = {
  /** TOOL 0 TCP offset from flange, millimetres (Motoman). */
  tcpXmm: number;
  tcpYmm: number;
  tcpZmm: number;
  /** TOOL 0 RPY, degrees (Motoman Rx, Ry, Rz). */
  tcpRxDeg: number;
  tcpRyDeg: number;
  tcpRzDeg: number;
  /** Extra mount offset on the flange parent, millimetres. */
  mountXmm: number;
  mountYmm: number;
  mountZmm: number;
  mountRxDeg: number;
  mountRyDeg: number;
  mountRzDeg: number;
  /** How Motoman +Z maps onto the URDF flange/tool0 axes. */
  flangePreset: FlangeAxisPreset;
  /** Show AxesHelper at flange origin and TCP. */
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
  /** Extra transform for a custom mesh in Motoman flange frame (mm / deg). */
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
 * Euler (XYZ) that maps Motoman flange (+Z out) onto the URDF parent axes.
 * `plusZ_iso` is correct for MA2010 `tool0` (approach = URDF +X).
 * `plusX_raw` is identity — use when parenting to an AR2010-style `flange`
 * that already applies R_y(+π/2) in the URDF.
 */
export function flangePresetEuler(
  preset: FlangeAxisPreset,
): [number, number, number] {
  const h = Math.PI / 2;
  switch (preset) {
    case "plusZ_iso":
      return [0, h, 0]; // Motoman Z → URDF +X
    case "plusX_raw":
      return [0, 0, 0];
    case "minusX":
      return [0, -h, 0]; // Motoman Z → URDF −X
    case "plusY":
      return [-h, 0, 0]; // Motoman Z → URDF +Y
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
