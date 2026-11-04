/**
 * Alignment of the viewport torch mesh into Motoman ``tool0``
 * (tip at origin after transform, wire approach = +Z).
 *
 * Cell hardware (from ABICOR BINZEL PDFs, not CAD):
 *   iCAT mini collision mount + ABIROB® G/W neck (catalog 45° closest to
 *   TOOL.CND Ry = −42.6215°). See ``docs/BACKUP_AND_LORCH_ANALYSIS.md``.
 *
 * Mesh file: still the Verbotics ``rm62_36deg.stl`` placeholder (historical
 * path ``torch_esab/``) — **not** an official ABIROB STEP. Tip/RPY are forced
 * to dump TOOL 0; replace when a real STEP lands in ``public/models/torch/``.
 */

export const TORCH_MESH_URL = "/models/torch_esab/rm62_36deg.stl";
/** @deprecated use TORCH_MESH_URL — name kept for older imports */
export const TORCH_ESAB_URL = TORCH_MESH_URL;

export const TORCH_HARDWARE = {
  mount: "ABICOR BINZEL iCAT mini",
  neck: "ABIROB G/W (catalog 45° ≈ TOOL Ry −42.62°)",
  meshProvenance: "Verbotics example STL placeholder (not OEM CAD)",
  tcpSource: "DX200 TOOL.CND TOOL 0",
} as const;

export const TORCH_MESH_TIP_MM: [number, number, number] = [
  75.599, -0.021, 352.266,
];

/** Quaternion (x,y,z,w) mapping mesh coords → TOOL after tip subtraction. */
export const TORCH_MESH_ALIGN_QUAT: [number, number, number, number] = [
  0.433551, -0.000485, 0.901129, -0.000233,
];
