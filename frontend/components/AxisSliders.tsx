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

export function defaultJoints(): Joints {
  return { S: 0, L: 0, U: 0, R: 0, B: 0, T: 0 };
}

export default function AxisSliders({
  joints,
  onChange,
}: {
  joints: Joints;
  onChange: (j: Joints) => void;
}) {
  return (
    <div className="space-y-3">
      {AXES.map((axis) => {
        const [min, max] = LIMITS[axis];
        return (
          <div key={axis} className="flex items-center gap-3">
            <span className="w-6 font-mono text-yaskawa-accent">{axis}</span>
            <input
              type="range"
              min={min}
              max={max}
              step={0.5}
              value={joints[axis]}
              onChange={(e) =>
                onChange({ ...joints, [axis]: Number(e.target.value) })
              }
              className="flex-1 accent-yaskawa-accent"
            />
            <span className="w-16 text-right font-mono text-sm">
              {joints[axis].toFixed(1)}&deg;
            </span>
          </div>
        );
      })}
      <button
        onClick={() => onChange(defaultJoints())}
        className="mt-2 rounded bg-slate-700 px-3 py-1 text-sm hover:bg-slate-600"
      >
        Reset to home
      </button>
    </div>
  );
}
