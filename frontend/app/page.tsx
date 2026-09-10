"use client";

import { useState, useEffect, useRef, useMemo } from "react";
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

const RobotWorkspace = dynamic(() => import("@/components/RobotWorkspace"), {
  ssr: false,
});

const PLAYBACK_BASE_SEC = 12;

type View =
  | "cell-robot"
  | "cell-rail"
  | "cell-station-0"
  | "cell-station-1"
  | "part"
  | "seams"
  | "path"
  | "simulate"
  | "export";

const STEPS: { id: number; label: string; view: View }[] = [
  { id: 1, label: "Cell", view: "cell-robot" },
  { id: 2, label: "Part", view: "part" },
  { id: 3, label: "Seams", view: "seams" },
  { id: 4, label: "Path", view: "path" },
  { id: 5, label: "Simulate", view: "simulate" },
  { id: 6, label: "Export", view: "export" },
];

const stepOf = (v: View) =>
  v.startsWith("cell")
    ? 1
    : v === "part"
    ? 2
    : v === "seams"
    ? 3
    : v === "path"
    ? 4
    : v === "simulate"
    ? 5
    : 6;

const btnPrimary =
  "rounded-md bg-yaskawa-accent px-3 py-1.5 text-sm font-semibold text-slate-900 transition hover:brightness-110 disabled:cursor-not-allowed disabled:opacity-40";
const btnGhost =
  "rounded-md border border-slate-600 px-3 py-1.5 text-sm text-slate-200 transition hover:bg-slate-800 disabled:cursor-not-allowed disabled:opacity-40";
const iconBtn =
  "flex h-8 w-8 items-center justify-center rounded-md border border-slate-600 text-slate-200 transition hover:bg-slate-800 disabled:opacity-40";

function PanelHeader({ children }: { children: React.ReactNode }) {
  return (
    <h2 className="mb-3 text-xs font-semibold uppercase tracking-wide text-slate-400">
      {children}
    </h2>
  );
}

export default function Home() {
  const [modelId, setModelId] = useState<string>("weld");
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
  const [weldCondition, setWeldCondition] = useState(1);
  const [simT, setSimT] = useState(0);
  const [simPlaying, setSimPlaying] = useState(false);
  const [simSpeed, setSimSpeed] = useState(1);

  const [view, setView] = useState<View>("part");

  const rafRef = useRef<number | undefined>(undefined);
  const lastRef = useRef<number | undefined>(undefined);

  const setStation = (i: number, patch: Partial<{ tilt: number; rotate: number }>) =>
    setStations((prev) => prev.map((s, idx) => (idx === i ? { ...s, ...patch } : s)));

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

  // Progress (0..1) at the start of each waypoint, for step-to-next/prev.
  const wpProgress = useMemo(() => {
    if (!program) return [] as number[];
    const cum = [0];
    for (const d of program.segDurations) cum.push(cum[cum.length - 1] + d);
    return cum.map((c) => c / program.cycleSec);
  }, [program]);

  function makeProgram(cond: number) {
    const prog = buildDemoProgram(DEMO_MOUNT, cond);
    setProgram(prog);
    setSimT(0);
    setSimPlaying(false);
    setModelId("weld");
    setRailTravel(Math.max(-1.6, Math.min(1.6, DEMO_MOUNT[2])));
    setStation(0, { rotate: 0 });
    setStatus(
      `Program ready: T-fillet · ${prog.weldLenMm.toFixed(0)} mm · cycle ≈ ${prog.cycleSec.toFixed(
        1,
      )} s`,
    );
  }

  function loadDemo() {
    makeProgram(weldCondition);
    setView("path");
  }

  function changeCondition(c: number) {
    setWeldCondition(c);
    if (program) makeProgram(c);
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

  function stepTo(dir: 1 | -1) {
    if (!program) return;
    setSimPlaying(false);
    const eps = 1e-4;
    if (dir === 1) {
      const next = wpProgress.find((p) => p > simT + eps);
      setSimT(next ?? 1);
    } else {
      const prev = [...wpProgress].reverse().find((p) => p < simT - eps);
      setSimT(prev ?? 0);
    }
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
  const curWp = program && sample ? program.waypoints[sample.wpIndex] : null;
  const instruction = curWp
    ? `${curWp.move} ${curWp.id}  ${curWp.speedLabel}${curWp.tag ? "  " + curWp.tag : ""}`
    : "—";
  const activeStep = stepOf(view);

  const treeItem = (v: View, label: string, depth = 0) => (
    <button
      key={v}
      onClick={() => setView(v)}
      className={`flex w-full items-center gap-2 rounded px-2 py-1 text-left text-sm transition ${
        view === v
          ? "bg-yaskawa-accent/15 text-yaskawa-accent"
          : "text-slate-300 hover:bg-slate-800"
      }`}
      style={{ paddingLeft: 8 + depth * 14 }}
    >
      <span className="text-[10px] text-slate-500">●</span>
      {label}
    </button>
  );

  return (
    <main className="flex h-screen flex-col bg-slate-950 text-slate-100">
      {/* App bar */}
      <header className="flex items-center justify-between border-b border-slate-800 bg-slate-900 px-4 py-2">
        <div className="flex items-center gap-3">
          <span className="flex h-7 w-7 items-center justify-center rounded-md bg-yaskawa-blue text-sm font-bold text-white">
            Y
          </span>
          <div>
            <div className="text-sm font-semibold leading-tight">
              Yaskawa Welding Navigator
            </div>
            <div className="text-[10px] leading-tight text-slate-400">
              DX200 · AR series · H1000D · Lorch S8
            </div>
          </div>
        </div>
        <div className="flex items-center gap-2">
          <button onClick={loadDemo} className={btnPrimary}>
            Load demo part
          </button>
          <button onClick={downloadJbi} disabled={!program} className={btnGhost}>
            Export .JBI
          </button>
        </div>
      </header>

      {/* Workflow stepper */}
      <nav className="flex items-center gap-1 border-b border-slate-800 bg-slate-900/60 px-3 py-1.5">
        {STEPS.map((s, i) => {
          const active = activeStep === s.id;
          const done = activeStep > s.id;
          return (
            <div key={s.id} className="flex items-center">
              <button
                onClick={() => setView(s.view)}
                className={`flex items-center gap-2 rounded-md px-2.5 py-1 text-xs font-medium transition ${
                  active
                    ? "bg-yaskawa-accent/15 text-yaskawa-accent"
                    : "text-slate-400 hover:bg-slate-800"
                }`}
              >
                <span
                  className={`flex h-5 w-5 items-center justify-center rounded-full text-[10px] ${
                    active
                      ? "bg-yaskawa-accent text-slate-900"
                      : done
                      ? "bg-yaskawa-blue text-white"
                      : "bg-slate-700 text-slate-300"
                  }`}
                >
                  {s.id}
                </span>
                {s.label}
              </button>
              {i < STEPS.length - 1 && (
                <span className="mx-1 text-slate-600">›</span>
              )}
            </div>
          );
        })}
      </nav>

      <div className="bg-amber-900/40 px-4 py-1 text-[11px] text-amber-200">
        ⚠ Prototype with placeholder kinematics — validate every path in MotoSim
        before real hardware use.
      </div>

      <div className="flex flex-1 overflow-hidden">
        {/* Left: project / cell tree */}
        <aside className="w-60 shrink-0 overflow-y-auto border-r border-slate-800 bg-slate-900/40 p-3">
          <PanelHeader>Project</PanelHeader>
          <div className="mb-1 flex items-center gap-1 text-xs font-semibold text-slate-400">
            <span>▾</span> Cell
          </div>
          {treeItem("cell-robot", "Robot (AR + torch)", 1)}
          {treeItem("cell-rail", "Rail (travel axis)", 1)}
          {treeItem("cell-station-0", "Stół 1 — positioner", 1)}
          {treeItem("cell-station-1", "Stół 2 — positioner", 1)}
          <div className="mb-1 mt-3 flex items-center gap-1 text-xs font-semibold text-slate-400">
            <span>▾</span> Part
          </div>
          {treeItem("part", program ? program.name : "No part", 1)}
          <div className="mb-1 mt-3 flex items-center gap-1 text-xs font-semibold text-slate-400">
            <span>▾</span> Welds
          </div>
          {treeItem("seams", program ? "Seam 1 (fillet)" : "No welds", 1)}
          {treeItem("path", "Path & parameters", 1)}
          {treeItem("simulate", "Simulation", 1)}
          {treeItem("export", "Export job", 1)}
        </aside>

        {/* Center: viewport */}
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

          {/* HUD */}
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
              <div>{(simT * 100).toFixed(0)}% complete</div>
            </div>
          )}

          {/* Simulation transport */}
          {program && (
            <div className="absolute bottom-4 left-1/2 w-[min(720px,94%)] -translate-x-1/2 rounded-lg bg-slate-900/90 px-4 py-2 shadow-lg">
              <div className="mb-1.5 flex items-center gap-3">
                <button onClick={() => stepTo(-1)} className={iconBtn} title="Previous point">
                  ⏮
                </button>
                <button
                  onClick={togglePlay}
                  className="rounded-md bg-yaskawa-accent px-4 py-1 text-sm font-semibold text-slate-900 hover:brightness-110"
                >
                  {simPlaying ? "Pause" : simT >= 1 ? "Replay" : "Play"}
                </button>
                <button onClick={() => stepTo(1)} className={iconBtn} title="Next point">
                  ⏭
                </button>
                <button
                  onClick={() => {
                    setSimPlaying(false);
                    setSimT(0);
                  }}
                  className={iconBtn}
                  title="Reset"
                >
                  ⟲
                </button>
                <div className="flex-1 truncate px-2 font-mono text-xs text-slate-300">
                  {instruction}
                </div>
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
                className="w-full accent-yaskawa-accent"
              />
            </div>
          )}
        </div>

        {/* Right: contextual properties */}
        <aside className="w-80 shrink-0 overflow-y-auto border-l border-slate-800 bg-slate-900/40 p-4">
          {view === "cell-robot" && (
            <>
              <PanelHeader>Robot</PanelHeader>
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
                <p className="mt-1 text-[10px] text-slate-500">{activeModel.source}</p>
              )}
              {modelId === "weld" ? (
                <p className="mt-3 rounded bg-slate-800/60 p-2 text-[11px] text-slate-400">
                  The animated weld robot is driven by the program path (inverse
                  kinematics). Manual jog applies to the other models.
                </p>
              ) : (
                <div className="mt-4">
                  <PanelHeader>Jog axes</PanelHeader>
                  <AxisSliders joints={joints} onChange={setJoints} />
                </div>
              )}
            </>
          )}

          {view === "cell-rail" && (
            <>
              <PanelHeader>Rail — travel axis</PanelHeader>
              <div className="flex items-center gap-3">
                <span className="w-14 font-mono text-xs text-yaskawa-accent">Travel</span>
                <input
                  type="range"
                  min={-1.6}
                  max={1.6}
                  step={0.01}
                  value={railTravel}
                  onChange={(e) => setRailTravel(Number(e.target.value))}
                  className="flex-1 accent-yaskawa-accent"
                />
                <span className="w-16 text-right font-mono text-xs">
                  {railTravel.toFixed(2)} m
                </span>
              </div>
              <p className="mt-3 text-[10px] text-slate-500">
                Dedicated Yaskawa travel rail; the robot rides the carriage between
                the two stations.
              </p>
            </>
          )}

          {(view === "cell-station-0" || view === "cell-station-1") && activePositioner && (
            <>
              {(() => {
                const i = view === "cell-station-0" ? 0 : 1;
                const st = stations[i];
                return (
                  <>
                    <PanelHeader>Stół {i + 1} — positioner</PanelHeader>
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
                    <div className="mt-4 space-y-3">
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
                            onChange={(e) => setStation(i, { tilt: Number(e.target.value) })}
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
                            onChange={(e) => setStation(i, { rotate: Number(e.target.value) })}
                            className="flex-1 accent-yaskawa-accent"
                          />
                          <span className="w-14 text-right font-mono text-xs">
                            {st.rotate.toFixed(0)}&deg;
                          </span>
                        </div>
                      )}
                    </div>
                    <p className="mt-3 text-[10px] text-slate-500">
                      {activePositioner.source}
                    </p>
                  </>
                );
              })()}
            </>
          )}

          {view === "part" && (
            <>
              <PanelHeader>Part</PanelHeader>
              <div className="flex gap-2">
                <button onClick={loadDemo} className={`${btnPrimary} flex-1`}>
                  Load demo part
                </button>
              </div>
              <div className="mt-3">
                <label className="text-[11px] text-slate-400">Import CAD (STEP/STL)</label>
                <input
                  type="file"
                  accept=".step,.stp,.stl,.obj,.ply,.glb"
                  onChange={onFile}
                  className="mt-1 w-full text-xs"
                />
              </div>
              {program && (
                <dl className="mt-4 space-y-1 text-xs text-slate-300">
                  <div className="flex justify-between">
                    <dt className="text-slate-500">Name</dt>
                    <dd className="font-mono">{program.name}</dd>
                  </div>
                  <div className="flex justify-between">
                    <dt className="text-slate-500">Type</dt>
                    <dd>T-fillet coupon</dd>
                  </div>
                  <div className="flex justify-between">
                    <dt className="text-slate-500">Mounted on</dt>
                    <dd>Stół 1</dd>
                  </div>
                </dl>
              )}
              {status && <p className="mt-3 text-xs text-slate-400">{status}</p>}
            </>
          )}

          {view === "seams" && (
            <>
              <PanelHeader>Weld seams</PanelHeader>
              {program ? (
                <>
                  <div className="rounded border border-slate-700 bg-slate-900/60 p-3">
                    <div className="flex items-center justify-between text-sm">
                      <span className="font-medium text-slate-200">Seam 1</span>
                      <span className="rounded bg-orange-500/20 px-2 py-0.5 text-[10px] text-orange-300">
                        fillet
                      </span>
                    </div>
                    <div className="mt-1 text-[11px] text-slate-400">
                      Length {program.weldLenMm.toFixed(0)} mm
                    </div>
                    <label className="mt-3 block text-[11px] text-slate-400">
                      Weld condition (ARCON)
                    </label>
                    <select
                      value={weldCondition}
                      onChange={(e) => changeCondition(Number(e.target.value))}
                      className="mt-1 w-full rounded bg-slate-800 px-2 py-1 text-sm"
                    >
                      {LORCH_S8_SCHEDULES.map((s) => (
                        <option key={s.condition} value={s.condition}>
                          #{s.condition} — {s.currentA}A / {s.voltageV.toFixed(1)}V /{" "}
                          {s.travelCmMin} cm/min
                        </option>
                      ))}
                    </select>
                  </div>
                  <p className="mt-3 text-[10px] text-slate-500">
                    Auto-detected fillet seam (demo). Real seams are extracted from the
                    imported CAD.
                  </p>
                </>
              ) : (
                <p className="text-xs text-slate-400">
                  Load a part to detect weld seams.
                </p>
              )}
            </>
          )}

          {view === "path" && (
            <>
              <PanelHeader>Path & parameters</PanelHeader>
              {program ? (
                <>
                  <dl className="mb-3 space-y-1 text-xs text-slate-300">
                    <div className="flex justify-between">
                      <dt className="text-slate-500">Waypoints</dt>
                      <dd className="font-mono">{program.waypoints.length}</dd>
                    </div>
                    <div className="flex justify-between">
                      <dt className="text-slate-500">Weld length</dt>
                      <dd className="font-mono">{program.weldLenMm.toFixed(0)} mm</dd>
                    </div>
                    <div className="flex justify-between">
                      <dt className="text-slate-500">Cycle time</dt>
                      <dd className="font-mono">{program.cycleSec.toFixed(1)} s</dd>
                    </div>
                  </dl>
                  <div className="max-h-72 overflow-y-auto rounded border border-slate-700 bg-slate-900/60 text-[11px]">
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
                </>
              ) : (
                <p className="text-xs text-slate-400">Load a part to plan a path.</p>
              )}
            </>
          )}

          {view === "simulate" && (
            <>
              <PanelHeader>Weld schedule (Lorch S8)</PanelHeader>
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
              <p className="mt-2 text-[10px] text-slate-500">
                EXAMPLE parameters — replace with your qualified WPS.
              </p>
              <p className="mt-3 text-[11px] text-slate-400">
                {program
                  ? "Use the transport bar below the viewport to play, step and scrub the program."
                  : "Load a part first."}
              </p>
            </>
          )}

          {view === "export" && (
            <>
              <PanelHeader>Export job</PanelHeader>
              <button onClick={downloadJbi} disabled={!program} className={`${btnPrimary} w-full`}>
                Download {program ? `${program.name}.JBI` : ".JBI"}
              </button>
              {program && (
                <pre className="mt-3 max-h-80 overflow-auto rounded border border-slate-700 bg-slate-950 p-2 text-[10px] leading-tight text-slate-300">
                  {generateJbi(program).split("\n").slice(0, 22).join("\n")}
                  {"\n…"}
                </pre>
              )}
              <p className="mt-2 text-[10px] text-slate-500">
                DX200 INFORM III job (coordinated RB1,ST1). Structurally plausible
                template — validate on the controller / MotoSim.
              </p>
            </>
          )}
        </aside>
      </div>
    </main>
  );
}
