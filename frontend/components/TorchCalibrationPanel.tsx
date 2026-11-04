"use client";

import { useI18n } from "@/lib/i18n";
import {
  DEFAULT_TORCH_CALIBRATION,
  FLANGE_PRESET_OPTIONS,
  type FlangeAxisPreset,
  type TorchCalibration,
} from "@/lib/tool0";

const snapAngles = [-180, -90, 0, 90, 180] as const;

export default function TorchCalibrationPanel({
  calib,
  onChange,
  defaultOpen = false,
}: {
  calib: TorchCalibration;
  onChange: (next: TorchCalibration) => void;
  defaultOpen?: boolean;
}) {
  const { t } = useI18n();

  function patch(p: Partial<TorchCalibration>) {
    onChange({ ...calib, ...p });
  }

  function onFile(e: React.ChangeEvent<HTMLInputElement>) {
    const file = e.target.files?.[0];
    if (!file) return;
    if (calib.customMeshUrl) URL.revokeObjectURL(calib.customMeshUrl);
    const url = URL.createObjectURL(file);
    const lower = file.name.toLowerCase();
    const kind: "stl" | "glb" = lower.endsWith(".glb") || lower.endsWith(".gltf") ? "glb" : "stl";
    patch({
      meshMode: "custom",
      customMeshUrl: url,
      customMeshKind: kind,
    });
  }

  return (
    <details
      open={defaultOpen || undefined}
      className="mt-3 rounded border border-slate-200 bg-slate-50 open:bg-white"
    >
      <summary className="cursor-pointer select-none px-2 py-2 text-[11px] font-semibold uppercase tracking-wide text-slate-700">
        {t("torch.panelTitle")}
      </summary>
      <div className="space-y-3 border-t border-slate-200 p-2 text-xs text-slate-700">
        <p className="text-[10px] leading-snug text-slate-500">{t("torch.panelHelp")}</p>

        {/* TOOL 0 TCP */}
        <div>
          <div className="mb-1 text-[10px] font-semibold uppercase text-slate-500">
            {t("torch.tcpSection")}
          </div>
          <div className="grid grid-cols-3 gap-1.5">
            {(
              [
                ["tcpXmm", "X", calib.tcpXmm],
                ["tcpYmm", "Y", calib.tcpYmm],
                ["tcpZmm", "Z", calib.tcpZmm],
              ] as const
            ).map(([key, lab, val]) => (
              <label key={key} className="block">
                <span className="text-[10px] text-slate-500">{lab} mm</span>
                <input
                  type="number"
                  step={0.1}
                  value={Number(val.toFixed(3))}
                  onChange={(e) => patch({ [key]: Number(e.target.value) })}
                  className="mt-0.5 w-full rounded border border-slate-300 bg-white px-1 py-1.5 font-mono text-[11px]"
                />
              </label>
            ))}
            {(
              [
                ["tcpRxDeg", "Rx", calib.tcpRxDeg],
                ["tcpRyDeg", "Ry", calib.tcpRyDeg],
                ["tcpRzDeg", "Rz", calib.tcpRzDeg],
              ] as const
            ).map(([key, lab, val]) => (
              <label key={key} className="block">
                <span className="text-[10px] text-slate-500">{lab} °</span>
                <input
                  type="number"
                  step={0.1}
                  value={Number(val.toFixed(3))}
                  onChange={(e) => patch({ [key]: Number(e.target.value) })}
                  className="mt-0.5 w-full rounded border border-slate-300 bg-white px-1 py-1.5 font-mono text-[11px]"
                />
              </label>
            ))}
          </div>
          <button
            type="button"
            className="mt-1.5 w-full rounded border border-slate-300 bg-white px-2 py-1.5 text-[11px] hover:bg-slate-50"
            onClick={() =>
              patch({
                tcpXmm: DEFAULT_TORCH_CALIBRATION.tcpXmm,
                tcpYmm: DEFAULT_TORCH_CALIBRATION.tcpYmm,
                tcpZmm: DEFAULT_TORCH_CALIBRATION.tcpZmm,
                tcpRxDeg: DEFAULT_TORCH_CALIBRATION.tcpRxDeg,
                tcpRyDeg: DEFAULT_TORCH_CALIBRATION.tcpRyDeg,
                tcpRzDeg: DEFAULT_TORCH_CALIBRATION.tcpRzDeg,
              })
            }
          >
            {t("torch.resetTcp")}
          </button>
        </div>

        {/* Mount calibration */}
        <div>
          <div className="mb-1 text-[10px] font-semibold uppercase text-slate-500">
            {t("torch.mountSection")}
          </div>
          <label className="mb-1.5 block text-[10px] text-slate-500">
            {t("torch.flangePreset")}
            <select
              value={calib.flangePreset}
              onChange={(e) =>
                patch({ flangePreset: e.target.value as FlangeAxisPreset })
              }
              className="mt-0.5 w-full rounded border border-slate-300 bg-white px-2 py-1.5 text-[11px]"
            >
              {FLANGE_PRESET_OPTIONS.map((o) => (
                <option key={o.id} value={o.id}>
                  {o.label}
                </option>
              ))}
            </select>
          </label>
          <div className="grid grid-cols-3 gap-1.5">
            {(
              [
                ["mountXmm", "dX", calib.mountXmm],
                ["mountYmm", "dY", calib.mountYmm],
                ["mountZmm", "dZ", calib.mountZmm],
              ] as const
            ).map(([key, lab, val]) => (
              <label key={key} className="block">
                <span className="text-[10px] text-slate-500">{lab} mm</span>
                <input
                  type="number"
                  step={0.5}
                  value={Number(val.toFixed(2))}
                  onChange={(e) => patch({ [key]: Number(e.target.value) })}
                  className="mt-0.5 w-full rounded border border-slate-300 bg-white px-1 py-1.5 font-mono text-[11px]"
                />
              </label>
            ))}
          </div>
          <div className="mt-1.5 space-y-1">
            {(
              [
                ["mountRxDeg", "Roll X"],
                ["mountRyDeg", "Pitch Y"],
                ["mountRzDeg", "Yaw Z"],
              ] as const
            ).map(([key, lab]) => (
              <div key={key}>
                <div className="mb-0.5 text-[10px] text-slate-500">{lab}</div>
                <div className="flex flex-wrap gap-1">
                  {snapAngles.map((a) => (
                    <button
                      key={a}
                      type="button"
                      onClick={() => patch({ [key]: a })}
                      className={`rounded border px-1.5 py-1 font-mono text-[10px] ${
                        calib[key] === a
                          ? "border-[#e87722] bg-orange-50 text-[#e87722]"
                          : "border-slate-300 bg-white text-slate-600"
                      }`}
                    >
                      {a}°
                    </button>
                  ))}
                </div>
              </div>
            ))}
          </div>
        </div>

        {/* Mesh mode */}
        <div>
          <div className="mb-1 text-[10px] font-semibold uppercase text-slate-500">
            {t("torch.meshSection")}
          </div>
          <div className="mb-1.5 flex flex-wrap gap-1">
            {(
              [
                ["vendor", "torch.meshVendor"],
                ["procedural", "torch.meshProcedural"],
                ["custom", "torch.meshCustom"],
              ] as const
            ).map(([mode, labelKey]) => (
              <button
                key={mode}
                type="button"
                onClick={() => patch({ meshMode: mode })}
                className={`min-w-[30%] flex-1 rounded border px-2 py-1.5 text-[11px] ${
                  calib.meshMode === mode
                    ? "border-[#e87722] bg-orange-50 text-[#e87722]"
                    : "border-slate-300 bg-white"
                }`}
              >
                {t(labelKey)}
              </button>
            ))}
          </div>
          <label className="block">
            <span className="text-[10px] text-slate-500">{t("torch.uploadMesh")}</span>
            <input
              type="file"
              accept=".stl,.glb,.gltf,model/stl,model/gltf-binary"
              onChange={onFile}
              className="mt-0.5 block w-full text-[10px]"
            />
          </label>
          {calib.meshMode === "custom" && (
            <div className="mt-1.5 grid grid-cols-3 gap-1.5">
              {(
                [
                  ["customXmm", "X", calib.customXmm],
                  ["customYmm", "Y", calib.customYmm],
                  ["customZmm", "Z", calib.customZmm],
                  ["customRxDeg", "Rx", calib.customRxDeg],
                  ["customRyDeg", "Ry", calib.customRyDeg],
                  ["customRzDeg", "Rz", calib.customRzDeg],
                ] as const
              ).map(([key, lab, val]) => (
                <label key={key} className="block">
                  <span className="text-[10px] text-slate-500">{lab}</span>
                  <input
                    type="number"
                    step={0.5}
                    value={Number(val.toFixed(2))}
                    onChange={(e) => patch({ [key]: Number(e.target.value) })}
                    className="mt-0.5 w-full rounded border border-slate-300 bg-white px-1 py-1 font-mono text-[11px]"
                  />
                </label>
              ))}
              <label className="col-span-3 block">
                <span className="text-[10px] text-slate-500">{t("torch.customScale")}</span>
                <input
                  type="number"
                  step={0.01}
                  min={0.001}
                  value={calib.customScale}
                  onChange={(e) => patch({ customScale: Number(e.target.value) || 1 })}
                  className="mt-0.5 w-full rounded border border-slate-300 bg-white px-1 py-1 font-mono text-[11px]"
                />
              </label>
            </div>
          )}
        </div>

        <label className="flex items-center gap-2 text-[11px]">
          <input
            type="checkbox"
            checked={calib.showAxes}
            onChange={(e) => patch({ showAxes: e.target.checked })}
          />
          {t("torch.showAxes")}
        </label>

        <button
          type="button"
          className="w-full rounded border border-slate-300 bg-white px-2 py-1.5 text-[11px] hover:bg-slate-50"
          onClick={() => {
            if (calib.customMeshUrl) URL.revokeObjectURL(calib.customMeshUrl);
            onChange({ ...DEFAULT_TORCH_CALIBRATION });
          }}
        >
          {t("torch.resetAll")}
        </button>
      </div>
    </details>
  );
}
