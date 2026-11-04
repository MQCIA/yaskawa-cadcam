"use client";

import type { SketchMode } from "@/lib/seamSketch";

export type SeamSketchToolbarProps = {
  mode: SketchMode;
  snapOn: boolean;
  draftCount: number;
  canFinishSpline: boolean;
  disabled?: boolean;
  onMode: (m: SketchMode) => void;
  onSnap: (on: boolean) => void;
  onUndo: () => void;
  onCancel: () => void;
  onFinishSpline: () => void;
  labels: {
    title: string;
    orbit: string;
    tack: string;
    line: string;
    arc: string;
    spline: string;
    snap: string;
    undo: string;
    cancel: string;
    finish: string;
    hint: string;
  };
};

const btnBase =
  "rounded border px-2 py-1 text-[10px] font-medium transition sm:text-[11px]";
const idle = "border-slate-200 bg-white text-slate-700 hover:bg-slate-50";
const active = "border-[#e87722] bg-orange-50 text-[#e87722]";

export default function SeamSketchToolbar({
  mode,
  snapOn,
  draftCount,
  canFinishSpline,
  disabled,
  onMode,
  onSnap,
  onUndo,
  onCancel,
  onFinishSpline,
  labels,
}: SeamSketchToolbarProps) {
  const modes: { id: SketchMode; label: string }[] = [
    { id: "orbit", label: labels.orbit },
    { id: "tack", label: labels.tack },
    { id: "line", label: labels.line },
    { id: "arc", label: labels.arc },
    { id: "spline", label: labels.spline },
  ];

  return (
    <div className="pointer-events-auto absolute bottom-2 left-1/2 z-20 w-[min(96vw,42rem)] -translate-x-1/2 sm:bottom-3">
      <div className="rounded-lg border border-slate-200 bg-white/95 shadow-md backdrop-blur">
        <div className="flex items-center justify-between gap-2 border-b border-slate-100 px-2.5 py-1.5">
          <div className="text-[10px] font-semibold uppercase tracking-wide text-slate-600 sm:text-[11px]">
            {labels.title}
          </div>
          <div className="truncate text-[10px] text-slate-500">{labels.hint}</div>
        </div>
        <div className="flex flex-wrap items-center gap-1.5 p-2">
          {modes.map((m) => (
            <button
              key={m.id}
              type="button"
              disabled={disabled}
              onClick={() => onMode(m.id)}
              className={`${btnBase} ${mode === m.id ? active : idle}`}
            >
              {m.label}
            </button>
          ))}
          <div className="mx-0.5 hidden h-5 w-px bg-slate-200 sm:block" />
          <button
            type="button"
            disabled={disabled}
            onClick={() => onSnap(!snapOn)}
            className={`${btnBase} ${snapOn ? active : idle}`}
          >
            {labels.snap}: {snapOn ? "ON" : "OFF"}
          </button>
          <button
            type="button"
            disabled={disabled || draftCount === 0}
            onClick={onUndo}
            className={`${btnBase} ${idle} disabled:opacity-40`}
          >
            {labels.undo}
          </button>
          <button
            type="button"
            disabled={disabled || draftCount === 0}
            onClick={onCancel}
            className={`${btnBase} ${idle} disabled:opacity-40`}
          >
            {labels.cancel}
          </button>
          {mode === "spline" && (
            <button
              type="button"
              disabled={disabled || !canFinishSpline}
              onClick={onFinishSpline}
              className={`${btnBase} border-emerald-300 bg-emerald-50 text-emerald-800 disabled:opacity-40`}
            >
              {labels.finish}
            </button>
          )}
        </div>
      </div>
    </div>
  );
}
