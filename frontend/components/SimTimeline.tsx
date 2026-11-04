"use client";

import type { TimelineMarker, TimelinePhase } from "@/lib/trajectoryViz";

/**
 * Colored process-phase strip under the simulation scrubber:
 * blue = MOVJ, red = ARCON weld, yellow = TouchSense;
 * violet ticks = EC/BC index; red ! = IK warning.
 */
export default function SimTimeline({
  phases,
  markers,
  simT,
  disabled,
  onSeek,
}: {
  phases: TimelinePhase[];
  markers: TimelineMarker[];
  simT: number;
  disabled?: boolean;
  onSeek: (t: number) => void;
}) {
  return (
    <div className="relative w-full select-none">
      <div
        className={`relative flex h-3 w-full overflow-hidden rounded-sm border border-slate-200 bg-slate-100 ${
          disabled ? "opacity-40" : "cursor-pointer"
        }`}
        onClick={(e) => {
          if (disabled) return;
          const rect = e.currentTarget.getBoundingClientRect();
          const t = Math.max(0, Math.min(1, (e.clientX - rect.left) / rect.width));
          onSeek(t);
        }}
      >
        {phases.map((p, i) => {
          const left = `${p.start * 100}%`;
          const width = `${Math.max(0.15, (p.end - p.start) * 100)}%`;
          const bg =
            p.kind === "weld"
              ? "bg-red-500"
              : p.kind === "sense"
                ? "bg-yellow-400"
                : "bg-blue-500";
          return (
            <div
              key={i}
              className={`absolute top-0 h-full ${bg}`}
              style={{ left, width }}
              title={p.kind}
            />
          );
        })}
        {/* Playhead */}
        <div
          className="pointer-events-none absolute top-0 z-10 h-full w-0.5 bg-slate-900"
          style={{ left: `${simT * 100}%` }}
        />
      </div>
      <div className="relative mt-0.5 h-3 w-full">
        {markers.map((m, i) => (
          <button
            key={`${m.kind}-${m.wpIndex}-${i}`}
            type="button"
            disabled={disabled}
            title={m.label}
            onClick={() => onSeek(m.t)}
            className={`absolute -translate-x-1/2 text-[10px] leading-none ${
              m.kind === "warn" ? "text-red-600" : "text-violet-600"
            }`}
            style={{ left: `${m.t * 100}%` }}
          >
            {m.kind === "warn" ? "!" : "◆"}
          </button>
        ))}
      </div>
      <div className="mt-0.5 flex flex-wrap gap-2 text-[9px] text-slate-500">
        <span className="inline-flex items-center gap-1">
          <span className="inline-block h-2 w-3 rounded-sm bg-blue-500" /> MOVJ
        </span>
        <span className="inline-flex items-center gap-1">
          <span className="inline-block h-2 w-3 rounded-sm bg-red-500" /> ARCON
        </span>
        <span className="inline-flex items-center gap-1">
          <span className="inline-block h-2 w-3 rounded-sm bg-yellow-400" /> Sense
        </span>
        <span className="inline-flex items-center gap-1 text-violet-600">◆ EC/BC</span>
        <span className="inline-flex items-center gap-1 text-red-600">! Limit/kolizja</span>
      </div>
    </div>
  );
}
