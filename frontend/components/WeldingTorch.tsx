"use client";

import WeldTorch3D from "./WeldTorch3D";
import { DEFAULT_TORCH_CALIBRATION } from "@/lib/tool0";

/**
 * Legacy entry point — wraps `WeldTorch3D` with dump TOOL 0 defaults.
 * Prefer passing a live `TorchCalibration` through `WeldUrdfRobot`.
 */
export default function WeldingTorch({
  arcOn = false,
}: {
  arcOn?: boolean;
  tipLen?: number;
  vendorMesh?: boolean;
}) {
  return <WeldTorch3D calib={DEFAULT_TORCH_CALIBRATION} arcOn={arcOn} />;
}
