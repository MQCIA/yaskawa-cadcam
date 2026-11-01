/**
 * Alignment of `public/models/torch_esab/rm62_36deg.stl` (mm) into the
 * Motoman TOOL frame (tip at origin, wire approach = +Z).
 *
 * Source: Verbotics Weld example cell PLY → STL (see SOURCE.txt).
 * Measured tip ≈ (75.6, 0, 352.3); after tip subtract + this quaternion,
 * the mount end points within ~4° of the flange direction from TOOL.CND.
 */
export const TORCH_ESAB_URL = "/models/torch_esab/rm62_36deg.stl";

export const TORCH_MESH_TIP_MM: [number, number, number] = [
  75.599, -0.021, 352.266,
];

/** Quaternion (x,y,z,w) mapping mesh coords → TOOL after tip subtraction. */
export const TORCH_MESH_ALIGN_QUAT: [number, number, number, number] = [
  0.433551, -0.000485, 0.901129, -0.000233,
];
