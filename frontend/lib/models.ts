import type { Joints } from "@/components/AxisSliders";

// Prepends the deploy base path (e.g. "/yaskawa-cadcam" on GitHub Pages) so
// static assets under public/ resolve both locally and on a project site.
const BASE = process.env.NEXT_PUBLIC_BASE_PATH || "";
export const withBase = (p: string) => `${BASE}${p}`;

export type BuiltinModel = {
  id: string;
  label: string;
  kind: "procedural" | "urdf";
  url?: string;
  color?: number;
  // Maps the S/L/U/R/B/T sliders to URDF joint names.
  jointMap?: Partial<Record<keyof Joints, string>>;
  source?: string;
};

// Example Yaskawa models vendored from ROS-Industrial `motoman` (Apache-2.0).
export const ROBOT_MODELS: BuiltinModel[] = [
  {
    id: "procedural",
    label: "Procedural placeholder",
    kind: "procedural",
  },
  {
    id: "ar2010",
    label: "Yaskawa MOTOMAN-AR2010 (URDF)",
    kind: "urdf",
    url: withBase("/models/ar2010/ar2010.urdf"),
    color: 0x1f4fb0,
    jointMap: {
      S: "joint_1_s",
      L: "joint_2_l",
      U: "joint_3_u",
      R: "joint_4_r",
      B: "joint_5_b",
      T: "joint_6_t",
    },
    source: "ros-industrial/motoman · motoman_ar2010_support",
  },
];

export type PositionerModel = {
  id: string;
  label: string;
  kind: "urdf" | "procedural";
  url?: string;
  color?: number;
  // Which control axes this positioner exposes.
  hasTilt: boolean;
  hasRotate: boolean;
  // For URDF models: rotary axes -> URDF joint names.
  axisMap?: { tilt?: string; rotate?: string };
  source?: string;
  approximate?: boolean;
};

export const POSITIONER_MODELS: PositionerModel[] = [
  {
    id: "h1000d",
    label: "H1000D (1-axis horizontal rotary, ~1000 kg)",
    kind: "procedural",
    color: 0x9aa4b2,
    hasTilt: false,
    hasRotate: true,
    source: "Procedural placeholder (no public CAD) — see docs/MODELS.md",
    approximate: true,
  },
  {
    id: "motopos_d500",
    label: "MotoPos D500 (2-axis rotary positioner)",
    kind: "urdf",
    url: withBase("/models/motopos_d500/motopos_d500.urdf"),
    color: 0x9aa4b2,
    hasTilt: true,
    hasRotate: true,
    axisMap: { tilt: "joint_1", rotate: "joint_2" },
    source: "ros-industrial/motoman · motoman_motopos_d500_support",
  },
];
