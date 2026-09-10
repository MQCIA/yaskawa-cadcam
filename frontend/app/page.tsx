"use client";

import { useState, useEffect, useRef } from "react";
import dynamic from "next/dynamic";
import AxisSliders, { defaultJoints, type Joints } from "@/components/AxisSliders";
import type { SeamSegment } from "@/components/RobotWorkspace";
import { analyzeCad } from "@/lib/api";
import { ROBOT_MODELS, POSITIONER_MODELS } from "@/lib/models";
import {
  buildDemoProgram,
  sampleProgram,
  generateJbi,
  DEMO_MOUNT,
  LORCH_S8_SCHEDULES,
  type WeldProgram,
} from "@/lib/weldProgram";

// R3F must render client-side only.
const RobotWorkspace = dynamic(() => import("@/components/RobotWorkspace"), {
  ssr: false,
});

// Wall-clock seconds to play the whole program at 1x.
const PLAYBACK_BASE_SEC = 12;

export default function Home() {
  const [modelId, setModelId] = useState<string>("ar2010");
  const [joints, setJoints] = useState<Joints>(defaultJoints());
  const [seams, setSeams] = useState<SeamSegment[]>([]);
  const [status, setStatus] = useState<string>("");

  const [positionerId, setPositionerId] = useState<string | null>("h1000d");
  const [railTravel, setRailTravel] = useState(0);
  const [stations, setStations] = useState([
    { tilt: 0, rotate: 0 },
    { tilt: 0, rotate: 0 },
  ]);

  const [program, setProgram] = useState<WeldProgram | null>(null);
  const [simT, setSimT] = useState(0);
  const [simPlaying, setSimPlaying] = useState(false);
  const [simSpeed, setSimSpeed] = useState(1);

  const rafRef = useRef<number | undefined>(undefined);
  const lastRef = useRef<number | undefined>(undefined);

  const setStation = (i: number, patch: Partial<{ tilt: number; rotate: number }>) =>
    setStations((prev) =>
      prev.map((s, idx) => (idx === i ? { ...s, ...patch } : s)),
    );

  // Simulation playback clock.
  useEffect(() => {
    if (!simPlaying || !program) return;
    lastRef.current = undefined;
    const tick = (ts: number) => {
      if (lastRef.current == null) lastRef.current = ts;
      const dt = (ts - lastRef.current) / 1000;
      lastRef.current = ts;
      setSimT((prev) => Math.min(1, prev + (dt * simSpeed) / PLAYBACK_BASE_SEC));
      rafRef.current = requestAnimationFrame(tick);
    };
    rafRef.current = requestAnimationFrame(tick);
    return () => {
      if (rafRef.current) cancelAnimationFrame(rafRef.current);
    };
  }, [simPlaying, simSpeed, program]);

  useEffect(() => {
    if (simT >= 1 && simPlaying) setSimPlaying(false);
  }, [simT, simPlaying]);

  function loadDemo() {
    const prog = buildDemoProgram();
    setProgram(prog);
    setSimT(0);
    setSimPlaying(false);
    // Use the animated weld robot (with torch) so the arm follows the path.
    setModelId("weld");
    // Slide the robot to work-station 1 (where the demo part is mounted).
    setRailTravel(Math.max(-1.6, Math.min(1.6, DEMO_MOUNT[2])));
    setStation(0, { rotate: 0 });
    setStatus(
      `Program ready: T-fillet · ${prog.weldLenMm.toFixed(0)} mm seam · cycle ≈ ${prog.cycleSec.toFixed(
        1,
      )} s`,
    );
  }

  function downloadJbi() {
    if (!program) return;
    const blob = new Blob([generateJbi(program)], { type: "text/plain" });
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url;
    a.download = `${program.name}.JBI`;
    a.click();
    URL.revokeObjectURL(url);
  }

  function togglePlay() {
    if (!program) return;
    if (simT >= 1) setSimT(0);
    setSimPlaying((p) => !p);
  }

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
  const sample = program ? sampleProgram(program, simT) : null;

  return (
    <main className="flex h-screen flex-col">
      <header className="border-b border-slate-700 bg-yaskawa-blue/20 px-4 py-2">
        <h1 className="text-lg font-semibold">
          Yaskawa Welding Navigator{" "}
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
          {/* --- Welding program (marquee workflow) --- */}
          <section>
            <h2 className="mb-2 text-sm font-semibold uppercase text-slate-400">
              Welding program
            </h2>
            <div className="flex gap-2">
              <button
                onClick={loadDemo}
                className="flex-1 rounded bg-yaskawa-accent px-2 py-1.5 text-sm font-medium text-slate-900 hover:brightness-110"
              >
                Load demo part
              </button>
              <button
                onClick={downloadJbi}
                disabled={!program}
                className="flex-1 rounded border border-slate-600 px-2 py-1.5 text-sm hover:bg-slate-800 disabled:opacity-40"
              >
                Export .JBI
              </button>
            </div>
            {program && (
              <div className="mt-3 max-h-56 overflow-y-auto rounded border border-slate-700 bg-slate-900/60 text-[11px]">
                <table className="w-full">
                  <thead className="sticky top-0 bg-slate-800 text-slate-400">
                    <tr>
                      <th className="px-2 py-1 text-left font-medium">Pt</th>
                      <th className="px-1 py-1 text-left font-medium">Move</th>
                      <th className="px-1 py-1 text-left font-medium">Speed</th>
                      <th className="px-1 py-1 text-left font-medium">Arc</th>
                    </tr>
                  </thead>
                  <tbody className="font-mono">
                    {program.waypoints.map((wp, i) => (
                      <tr
                        key={wp.id}
                        className={
                          sample && i === sample.wpIndex
                            ? "bg-yaskawa-accent/20 text-yaskawa-accent"
                            : "text-slate-300"
                        }
                      >
                        <td className="px-2 py-0.5">{wp.id}</td>
                        <td className="px-1 py-0.5">{wp.move}</td>
                        <td className="px-1 py-0.5">{wp.speedLabel}</td>
                        <td className="px-1 py-0.5">
                          {wp.tag === "ARCON" ? (
                            <span className="text-orange-400">ARCON</span>
                          ) : wp.tag === "ARCOF" ? (
                            <span className="text-slate-400">ARCOF</span>
                          ) : wp.arc ? (
                            "•"
                          ) : (
                            ""
                          )}
                        </td>
                      </tr>
                    ))}
                  </tbody>
                </table>
              </div>
            )}
          </section>

          {/* --- Weld schedule --- */}
          <section>
            <h2 className="mb-2 text-sm font-semibold uppercase text-slate-400">
              Weld schedule (Lorch S8)
            </h2>
            <div className="overflow-hidden rounded border border-slate-700 text-[11px]">
              <table className="w-full">
                <thead className="bg-slate-800 text-slate-400">
                  <tr>
                    <th className="px-2 py-1 text-left font-medium">#</th>
                    <th className="px-1 py-1 text-right font-medium">A</th>
                    <th className="px-1 py-1 text-right font-medium">V</th>
                    <th className="px-1 py-1 text-right font-medium">m/min</th>
                    <th className="px-1 py-1 text-right font-medium">cm/min</th>
                  </tr>
                </thead>
                <tbody className="font-mono text-slate-300">
                  {LORCH_S8_SCHEDULES.map((s) => (
                    <tr
                      key={s.condition}
                      className={
                        program && program.schedule.condition === s.condition
                          ? "bg-yaskawa-accent/10"
                          : ""
                      }
                    >
                      <td className="px-2 py-0.5">{s.condition}</td>
                      <td className="px-1 py-0.5 text-right">{s.currentA}</td>
                      <td className="px-1 py-0.5 text-right">{s.voltageV.toFixed(1)}</td>
                      <td className="px-1 py-0.5 text-right">{s.wireMmin.toFixed(1)}</td>
                      <td className="px-1 py-0.5 text-right">{s.travelCmMin}</td>
                    </tr>
                  ))}
                </tbody>
              </table>
            </div>
            <p className="mt-1 text-[10px] text-slate-500">
              EXAMPLE parameters — replace with your qualified WPS.
            </p>
          </section>

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
              Rail (travel axis)
            </h2>
            <div className="flex items-center gap-3">
              <span className="w-14 font-mono text-xs text-yaskawa-accent">
                Travel
              </span>
              <input
                type="range"
                min={-1.6}
                max={1.6}
                step={0.01}
                value={railTravel}
                onChange={(e) => setRailTravel(Number(e.target.value))}
                className="flex-1 accent-yaskawa-accent"
              />
              <span className="w-14 text-right font-mono text-xs">
                {railTravel.toFixed(2)} m
              </span>
            </div>
          </section>

          <section>
            <h2 className="mb-2 text-sm font-semibold uppercase text-slate-400">
              Positioners (2 stations)
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
              <div className="mt-3 space-y-4">
                {stations.map((st, i) => (
                  <div key={i} className="space-y-2">
                    <p className="text-xs font-semibold text-slate-300">
                      Stół {i + 1}
                    </p>
                    {activePositioner.hasTilt && (
                      <div className="flex items-center gap-3">
                        <span className="w-14 font-mono text-xs text-yaskawa-accent">
                          Tilt
                        </span>
                        <input
                          type="range"
                          min={-135}
                          max={135}
                          step={0.5}
                          value={st.tilt}
                          onChange={(e) =>
                            setStation(i, { tilt: Number(e.target.value) })
                          }
                          className="flex-1 accent-yaskawa-accent"
                        />
                        <span className="w-14 text-right font-mono text-xs">
                          {st.tilt.toFixed(0)}&deg;
                        </span>
                      </div>
                    )}
                    {activePositioner.hasRotate && (
                      <div className="flex items-center gap-3">
                        <span className="w-14 font-mono text-xs text-yaskawa-accent">
                          Rotate
                        </span>
                        <input
                          type="range"
                          min={-360}
                          max={360}
                          step={0.5}
                          value={st.rotate}
                          onChange={(e) =>
                            setStation(i, { rotate: Number(e.target.value) })
                          }
                          className="flex-1 accent-yaskawa-accent"
                        />
                        <span className="w-14 text-right font-mono text-xs">
                          {st.rotate.toFixed(0)}&deg;
                        </span>
                      </div>
                    )}
                  </div>
                ))}
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

        <div className="relative flex-1">
          <RobotWorkspace
            modelId={modelId}
            joints={joints}
            seams={seams}
            positionerId={positionerId}
            railTravel={railTravel}
            stations={stations}
            program={program}
            simT={simT}
          />

          {/* --- HUD --- */}
          {program && sample && (
            <div className="pointer-events-none absolute left-4 top-4 rounded-lg bg-slate-900/85 px-3 py-2 font-mono text-xs text-slate-200 shadow-lg">
              <div className="mb-1 font-sans text-[11px] font-semibold uppercase text-slate-400">
                {program.name}
              </div>
              <div className="flex items-center gap-2">
                <span
                  className={`inline-block h-2 w-2 rounded-full ${
                    sample.arcOn ? "animate-pulse bg-orange-400" : "bg-slate-600"
                  }`}
                />
                ARC {sample.arcOn ? "ON" : "OFF"}
              </div>
              <div>
                t = {sample.elapsedSec.toFixed(1)} / {program.cycleSec.toFixed(1)} s
              </div>
              <div>point {program.waypoints[sample.wpIndex]?.id}</div>
              <div>{(simT * 100).toFixed(0)}% complete</div>
            </div>
          )}

          {/* --- Simulation bar --- */}
          {program && (
            <div className="absolute bottom-4 left-1/2 flex w-[min(680px,90%)] -translate-x-1/2 items-center gap-3 rounded-lg bg-slate-900/90 px-4 py-2 shadow-lg">
              <button
                onClick={togglePlay}
                className="rounded bg-yaskawa-accent px-3 py-1 text-sm font-semibold text-slate-900 hover:brightness-110"
              >
                {simPlaying ? "Pause" : simT >= 1 ? "Replay" : "Play"}
              </button>
              <button
                onClick={() => {
                  setSimPlaying(false);
                  setSimT(0);
                }}
                className="rounded border border-slate-600 px-2 py-1 text-xs hover:bg-slate-800"
              >
                Reset
              </button>
              <input
                type="range"
                min={0}
                max={1}
                step={0.001}
                value={simT}
                onChange={(e) => {
                  setSimPlaying(false);
                  setSimT(Number(e.target.value));
                }}
                className="flex-1 accent-yaskawa-accent"
              />
              <select
                value={simSpeed}
                onChange={(e) => setSimSpeed(Number(e.target.value))}
                className="rounded bg-slate-800 px-2 py-1 text-xs"
              >
                <option value={0.5}>0.5×</option>
                <option value={1}>1×</option>
                <option value={2}>2×</option>
                <option value={4}>4×</option>
              </select>
            </div>
          )}
        </div>
      </div>
    </main>
  );
}
