"use client";

import { useState } from "react";
import dynamic from "next/dynamic";
import AxisSliders, { defaultJoints, type Joints } from "@/components/AxisSliders";
import type { SeamSegment } from "@/components/RobotWorkspace";
import { analyzeCad } from "@/lib/api";

// R3F must render client-side only.
const RobotWorkspace = dynamic(() => import("@/components/RobotWorkspace"), {
  ssr: false,
});

export default function Home() {
  const [joints, setJoints] = useState<Joints>(defaultJoints());
  const [seams, setSeams] = useState<SeamSegment[]>([]);
  const [status, setStatus] = useState<string>("");

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
              Jog axes
            </h2>
            <AxisSliders joints={joints} onChange={setJoints} />
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
          <RobotWorkspace joints={joints} seams={seams} />
        </div>
      </div>
    </main>
  );
}
