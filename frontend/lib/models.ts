import type { Joints } from "@/components/AxisSliders";

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
    url: "/models/ar2010/ar2010.urdf",
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
  url: string;
  color?: number;
  // Rotary positioner axes -> URDF joint names.
  axisMap: { tilt?: string; rotate?: string };
  source?: string;
};

export const POSITIONER_MODELS: PositionerModel[] = [
  {
    id: "motopos_d500",
    label: "MotoPos D500 (2-axis rotary positioner)",
    url: "/models/motopos_d500/motopos_d500.urdf",
    color: 0x9aa4b2,
    axisMap: { tilt: "joint_1", rotate: "joint_2" },
    source: "ros-industrial/motoman · motoman_motopos_d500_support",
  },
];
