/**
 * Yaskawa TSL-600 / RECT-X style travel-axis meshes (Verbotics cell dump).
 * STL units: millimetres. Long axis = mesh +Y (~3990 mm for base_link_3150).
 */
export const TSL_BASE_URL = "/models/tsl600/visual/base_link_3150.stl";
export const TSL_CARRIAGE_URL = "/models/tsl600/visual/carriage_link.stl";

/**
 * Carriage deck top after shared base-frame placement (metres).
 * Measured from `carriage_link.stl` with the same transform as the rail.
 * Robot base mounts on this plane (`RobotWorkspace` CARRIAGE_TOP_Y).
 */
export const TSL_CARRIAGE_TOP_Y = 0.267;
