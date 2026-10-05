/**
 * Alignment of `rm62_36deg.stl` (mm) into the Motoman TOOL frame.
 *
 * Mesh tip is not at the origin and the contact-tip axis is not +Z.
 * Measured from the STL: tip ≈ (75.6, 0, 352.3); approach from neck→tip.
 * After translate-to-tip and Rz(180)·R_align, wire = TOOL +Z and the mount
 * points within ~4° of the flange direction implied by TOOL.CND.
 */
export const TORCH_MESH_TIP_MM: [number, number, number] = [
  75.599, -0.021, 352.266,
];

/** Quaternion (x,y,z,w) mapping mesh coords → TOOL after tip subtraction. */
export const TORCH_MESH_ALIGN_QUAT: [number, number, number, number] = [
  0.433551, -0.000485, 0.901129, -0.000233,
];
