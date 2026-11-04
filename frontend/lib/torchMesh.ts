/**
 * Alignment of the viewport torch mesh into Motoman ``tool0``
 * (tip at origin after transform, wire approach = +Z).
 *
 * Cell hardware (from ABICOR BINZEL PDFs + DX200 dump):
 *   iCAT mini collision mount + ABIROB® G/W neck (catalog 45° closest to
 *   TOOL.CND Ry = −42.6215°). See ``docs/BACKUP_AND_LORCH_ANALYSIS.md``.
 *
 * Viewport mesh: community Marathon OS STEP of an ABIROB® A300 ECO holder +
 * neck assembly (listed as “Welding Torch Robo WH PP W500”). Tip/wire are
 * forced to dump TOOL 0; mount residual ≈ 56 mm vs cell G/W neck geometry.
 */

export const TORCH_MESH_URL = "/models/torch/abirob_a300_eco.stl";
/** @deprecated use TORCH_MESH_URL — name kept for older imports */
export const TORCH_ESAB_URL = TORCH_MESH_URL;

export const TORCH_HARDWARE = {
  mount: "ABICOR BINZEL iCAT mini",
  neck: "ABIROB G/W (catalog 45° ≈ TOOL Ry −42.62°)",
  meshProvenance:
    "Marathon OS community STEP — ABIROB A300 ECO holder + neck (not cell OEM G/W CAD)",
  tcpSource: "DX200 TOOL.CND TOOL 0",
} as const;

/** Tip point in raw mesh millimetres (before align quat). */
export const TORCH_MESH_TIP_MM: [number, number, number] = [
  -144.54, 371.291, 2.851,
];

/** Quaternion (x,y,z,w) mapping mesh coords → TCP after tip subtraction. */
export const TORCH_MESH_ALIGN_QUAT: [number, number, number, number] = [
  0.636678, 0.307638, -0.30163, 0.639546,
];
