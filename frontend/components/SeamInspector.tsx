"use client";

import { WEAVE_FILES } from "@/lib/weldProgram";
import type { SeamEdit } from "@/lib/trajectoryViz";

const WELD_SPEEDS = [3.3, 4.2, 5.0] as const;

/**
 * Per-seam parameter dock — reverse, spin, travel angle, approach, weave, V.
 * Live-updates the parent via ``onChange`` (parent rebuilds the WeldProgram).
 */
export default function SeamInspector({
  seam,
  onChange,
  onReverse,
  labels,
}: {
  seam: SeamEdit | null;
  onChange: (next: SeamEdit) => void;
  onReverse: () => void;
  labels: {
    title: string;
    reverse: string;
    spin: string;
    travel: string;
    approach: string;
    weave: string;
    weaveNone: string;
    speed: string;
    empty: string;
  };
}) {
  if (!seam) {
    return (
      <p className="px-1 text-xs text-slate-500">{labels.empty}</p>
    );
  }

  return (
    <div className="space-y-2 rounded border border-slate-200 bg-slate-50 p-2">
      <div className="flex items-center justify-between gap-2">
        <span className="text-xs font-semibold text-slate-800">{seam.label}</span>
        <span className="font-mono text-[10px] text-slate-500">
          C{String(seam.weldStart).padStart(5, "0")}–C
          {String(seam.weldEnd).padStart(5, "0")}
        </span>
      </div>
      <div className="text-[10px] font-medium uppercase tracking-wide text-slate-500">
        {labels.title}
      </div>

      <button
        type="button"
        onClick={onReverse}
        className="w-full rounded border border-slate-300 bg-white px-2 py-1.5 text-left text-xs font-medium text-slate-700 transition hover:bg-orange-50"
      >
        ⇄ {labels.reverse}
      </button>

      <label className="block text-[10px] text-slate-500">
        {labels.spin}: {seam.spinOffsetDeg.toFixed(0)}°
        <input
          type="range"
          min={-180}
          max={180}
          step={1}
          value={seam.spinOffsetDeg}
          onChange={(e) =>
            onChange({ ...seam, spinOffsetDeg: Number(e.target.value) })
          }
          className="touch-range touch-pan-y mt-1 w-full accent-[#e87722]"
        />
      </label>

      <label className="block text-[10px] text-slate-500">
        {labels.travel}: {seam.travelAngleDeg.toFixed(0)}°
        <input
          type="range"
          min={-30}
          max={30}
          step={1}
          value={seam.travelAngleDeg}
          onChange={(e) =>
            onChange({ ...seam, travelAngleDeg: Number(e.target.value) })
          }
          className="touch-range touch-pan-y mt-1 w-full accent-[#e87722]"
        />
      </label>

      <label className="block text-[10px] text-slate-500">
        {labels.approach}: {seam.approachMm.toFixed(0)} mm
        <input
          type="range"
          min={20}
          max={200}
          step={5}
          value={seam.approachMm}
          onChange={(e) =>
            onChange({ ...seam, approachMm: Number(e.target.value) })
          }
          className="touch-range touch-pan-y mt-1 w-full accent-[#e87722]"
        />
      </label>

      <label className="block text-[10px] text-slate-500">
        {labels.weave}
        <select
          value={seam.weaveNo}
          onChange={(e) =>
            onChange({ ...seam, weaveNo: Number(e.target.value) })
          }
          className="mt-0.5 w-full rounded bg-white px-2 py-1.5 text-xs"
        >
          <option value={0}>{labels.weaveNone}</option>
          {WEAVE_FILES.map((w) => (
            <option key={w.wevNo} value={w.wevNo}>
              {w.label}
            </option>
          ))}
        </select>
      </label>

      <label className="block text-[10px] text-slate-500">
        {labels.speed}
        <select
          value={seam.weldSpeedV}
          onChange={(e) =>
            onChange({ ...seam, weldSpeedV: Number(e.target.value) })
          }
          className="mt-0.5 w-full rounded bg-white px-2 py-1.5 font-mono text-xs"
        >
          {WELD_SPEEDS.map((v) => (
            <option key={v} value={v}>
              V={v.toFixed(1)}
            </option>
          ))}
        </select>
      </label>
    </div>
  );
}
