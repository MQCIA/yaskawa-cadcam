"use client";

import { useI18n } from "@/lib/i18n";

export const AXES = ["S", "L", "U", "R", "B", "T"] as const;
export type Joints = Record<(typeof AXES)[number], number>;

const LIMITS: Record<(typeof AXES)[number], [number, number]> = {
  S: [-170, 170],
  L: [-90, 155],
  U: [-175, 250],
  R: [-180, 180],
  B: [-135, 135],
  T: [-360, 360],
};

export function defaultJoints(): Joints {
  return { S: 0, L: 0, U: 0, R: 0, B: 0, T: 0 };
}

function clamp(axis: (typeof AXES)[number], v: number) {
  const [min, max] = LIMITS[axis];
  return Math.max(min, Math.min(max, v));
}

export default function AxisSliders({
  joints,
  onChange,
  step = 1,
  disabled = false,
  showHome = false,
}: {
  joints: Joints;
  onChange: (j: Joints) => void;
  step?: number;
  disabled?: boolean;
  showHome?: boolean;
}) {
  const { t } = useI18n();

  function setAxis(axis: (typeof AXES)[number], value: number) {
    onChange({ ...joints, [axis]: clamp(axis, value) });
  }

  function nudge(axis: (typeof AXES)[number], dir: 1 | -1) {
    setAxis(axis, joints[axis] + dir * step);
  }

  return (
    <div className={`space-y-3 ${disabled ? "pointer-events-none opacity-40" : ""}`}>
      {AXES.map((axis) => {
        const [min, max] = LIMITS[axis];
        return (
          <div key={axis}>
            <div className="mb-0.5 flex items-center justify-between text-[10px] text-slate-500">
              <span>
                <span className="font-mono text-[#e87722]">{axis}</span>
                {" · "}
                {t(`axis.${axis}`)}
              </span>
              <span className="font-mono text-slate-300">{joints[axis].toFixed(1)}°</span>
            </div>
            <div className="flex items-center gap-1.5">
              <button
                type="button"
                title={`−${step}°`}
                onClick={() => nudge(axis, -1)}
                className="flex h-7 w-7 shrink-0 items-center justify-center rounded border border-slate-600 text-sm text-slate-200 hover:bg-slate-800"
              >
                −
              </button>
              <input
                type="range"
                min={min}
                max={max}
                step={0.5}
                value={joints[axis]}
                onChange={(e) => setAxis(axis, Number(e.target.value))}
                className="flex-1 accent-[#e87722]"
              />
              <button
                type="button"
                title={`+${step}°`}
                onClick={() => nudge(axis, 1)}
                className="flex h-7 w-7 shrink-0 items-center justify-center rounded border border-slate-600 text-sm text-slate-200 hover:bg-slate-800"
              >
                +
              </button>
              <input
                type="number"
                min={min}
                max={max}
                step={0.5}
                value={Number(joints[axis].toFixed(1))}
                onChange={(e) => setAxis(axis, Number(e.target.value))}
                className="w-16 rounded border border-slate-700 bg-slate-900 px-1 py-1 text-right font-mono text-xs text-slate-200"
              />
            </div>
          </div>
        );
      })}
      {showHome && (
        <button
          type="button"
          onClick={() => onChange(defaultJoints())}
          className="mt-1 w-full rounded bg-slate-700 px-3 py-1.5 text-sm hover:bg-slate-600"
        >
          {t("axis.home")}
        </button>
      )}
    </div>
  );
}
