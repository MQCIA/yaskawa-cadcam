"use client";

import { useState } from "react";
import dynamic from "next/dynamic";
import AxisSliders, { defaultJoints, type Joints } from "@/components/AxisSliders";
import type { SeamSegment } from "@/components/RobotWorkspace";
import { analyzeCad } from "@/lib/api";
import { ROBOT_MODELS, POSITIONER_MODELS } from "@/lib/models";

// R3F must render client-side only.
const RobotWorkspace = dynamic(() => import("@/components/RobotWorkspace"), {
  ssr: false,
});

export default function Home() {
  const [modelId, setModelId] = useState<string>("ar2010");
  const [joints, setJoints] = useState<Joints>(defaultJoints());
  const [seams, setSeams] = useState<SeamSegment[]>([]);
  const [status, setStatus] = useState<string>("");

  const [positionerId, setPositionerId] = useState<string | null>("motopos_d500");
  const [tilt, setTilt] = useState(0);
  const [rotate, setRotate] = useState(0);

  async function onFile(e: React.ChangeEvent<HTMLInputElement>) {
    const file = e.target.files?.[0];
    if (!file) return;
    setStatus("Analyzing CAD...");
    try {
      const result = await analyzeCad(file);
      setSeams(result.segments ?? []);
      setStatus(`Found ${result.seam_count} candidate seam(s).`);
    } catch (err) {
      setStatus(`Error: ${(err as Error).message}`);
    }
  }

  const activeModel = ROBOT_MODELS.find((m) => m.id === modelId);
  const activePositioner = POSITIONER_MODELS.find((p) => p.id === positionerId);

  return (
    <main className="flex h-screen flex-col">
      <header className="border-b border-slate-700 bg-yaskawa-blue/20 px-4 py-2">
        <h1 className="text-lg font-semibold">
          Yaskawa CAD/CAM Prototype{" "}
          <span className="text-xs font-normal text-yaskawa-accent">
            DX200 · AR series · H1000D · Lorch S8
          </span>
        </h1>
      </header>

      <div className="bg-amber-900/40 px-4 py-1.5 text-xs text-amber-200">
        ⚠ Prototype with placeholder kinematics. Not validated for real
        hardware. Verify every path in MotoSim before use.
      </div>

      <div className="flex flex-1 overflow-hidden">
        <aside className="w-80 space-y-6 overflow-y-auto border-r border-slate-700 p-4">
          <section>
            <h2 className="mb-2 text-sm font-semibold uppercase text-slate-400">
              Robot model
            </h2>
            <select
              value={modelId}
              onChange={(e) => setModelId(e.target.value)}
              className="w-full rounded bg-slate-800 px-2 py-1 text-sm"
            >
              {ROBOT_MODELS.map((m) => (
                <option key={m.id} value={m.id}>
                  {m.label}
                </option>
              ))}
            </select>
            {activeModel?.source && (
              <p className="mt-1 text-[10px] text-slate-500">
                {activeModel.source}
              </p>
            )}
          </section>

          <section>
            <h2 className="mb-2 text-sm font-semibold uppercase text-slate-400">
              Jog axes
            </h2>
            <AxisSliders joints={joints} onChange={setJoints} />
          </section>

          <section>
            <h2 className="mb-2 text-sm font-semibold uppercase text-slate-400">
              Positioner
            </h2>
            <select
              value={positionerId ?? ""}
              onChange={(e) => setPositionerId(e.target.value || null)}
              className="w-full rounded bg-slate-800 px-2 py-1 text-sm"
            >
              <option value="">None</option>
              {POSITIONER_MODELS.map((p) => (
                <option key={p.id} value={p.id}>
                  {p.label}
                </option>
              ))}
            </select>
            {activePositioner && (
              <div className="mt-3 space-y-3">
                <div className="flex items-center gap-3">
                  <span className="w-14 font-mono text-xs text-yaskawa-accent">
                    Tilt
                  </span>
                  <input
                    type="range"
                    min={-135}
                    max={135}
                    step={0.5}
                    value={tilt}
                    onChange={(e) => setTilt(Number(e.target.value))}
                    className="flex-1 accent-yaskawa-accent"
                  />
                  <span className="w-14 text-right font-mono text-xs">
                    {tilt.toFixed(0)}&deg;
                  </span>
                </div>
                <div className="flex items-center gap-3">
                  <span className="w-14 font-mono text-xs text-yaskawa-accent">
                    Rotate
                  </span>
                  <input
                    type="range"
                    min={-200}
                    max={200}
                    step={0.5}
                    value={rotate}
                    onChange={(e) => setRotate(Number(e.target.value))}
                    className="flex-1 accent-yaskawa-accent"
                  />
                  <span className="w-14 text-right font-mono text-xs">
                    {rotate.toFixed(0)}&deg;
                  </span>
                </div>
                <p className="text-[10px] text-slate-500">
                  {activePositioner.source}
                </p>
              </div>
            )}
          </section>

          <section>
            <h2 className="mb-2 text-sm font-semibold uppercase text-slate-400">
              Load CAD (STEP/STL)
            </h2>
            <input
              type="file"
              accept=".step,.stp,.stl,.obj,.ply,.glb"
              onChange={onFile}
              className="w-full text-sm"
            />
            {status && <p className="mt-2 text-xs text-slate-400">{status}</p>}
          </section>
        </aside>

        <div className="flex-1">
          <RobotWorkspace
            modelId={modelId}
            joints={joints}
            seams={seams}
            positionerId={positionerId}
            positionerTilt={tilt}
            positionerRotate={rotate}
          />
        </div>
      </div>
    </main>
  );
}
