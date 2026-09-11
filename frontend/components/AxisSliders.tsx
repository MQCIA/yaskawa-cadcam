"use client";

export const AXES = ["S", "L", "U", "R", "B", "T"] as const;
export type Joints = Record<(typeof AXES)[number], number>;

// Approximate travel ranges (deg). Replace with your robot's real limits.
const LIMITS: Record<(typeof AXES)[number], [number, number]> = {
  S: [-170, 170],
  L: [-90, 155],
  U: [-175, 250],
  R: [-180, 180],
  B: [-135, 135],
  T: [-360, 360],
};

const AXIS_HELP: Record<(typeof AXES)[number], string> = {
  S: "Base rotation",
  L: "Lower arm",
  U: "Upper arm",
  R: "Arm roll",
  B: "Wrist bend",
  T: "Torch roll",
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
}: {
  joints: Joints;
  onChange: (j: Joints) => void;
  step?: number;
  disabled?: boolean;
}) {
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
                <span className="font-mono text-yaskawa-accent">{axis}</span>
                {" · "}
                {AXIS_HELP[axis]}
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
                className="flex-1 accent-yaskawa-accent"
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
      <div className="flex gap-2 pt-1">
        <button
          type="button"
          onClick={() => onChange(defaultJoints())}
          className="flex-1 rounded bg-slate-700 px-3 py-1.5 text-sm hover:bg-slate-600"
        >
          Home (0°)
        </button>
      </div>
    </div>
  );
}
